#ifndef GRAPH_SEARCH_CONTACT_PLANNER_CONTACT_GRAPH_H
#define GRAPH_SEARCH_CONTACT_PLANNER_CONTACT_GRAPH_H

#include <graph_search/graph.h>
#include <graph_search_contact_planner/util.h>
#include <graph_search_contact_planner/contact_state.h>
#include <graph_search_contact_planner/contact_node.h>
#include <ik_constraint2_distance_field/ik_constraint2_distance_field.h>
#include <ik_constraint2_bullet/ik_constraint2_bullet.h>
#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>

namespace graph_search_contact_planner{
  class ContactPlanner : public graph_search::Planner {
  public:
    class ContactTransitionCheckParam : public graph_search::Planner::TransitionCheckParam {
    public:
      ContactState preState;
      ContactState postState;
      std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
      std::vector<cnoid::LinkPtr> variables;
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints;
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejections;
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
      prioritized_inverse_kinematics_solver2::IKParam pikParam;
      global_inverse_kinematics_solver::GIKParam gikParam;
    };
    std::shared_ptr<graph_search::Planner::TransitionCheckParam> generateCheckParam() override;
    void preCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> extend_node) override;
    void postCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> extend_node) override;
    bool checkTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override;
    bool isGoalSatisfied(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) override; // nodeのcontactsがgoalContactStatesのcontactsを含んでいればtrue. nameのみで座標は無視.
    std::vector<std::shared_ptr<graph_search::Node> > gatherAdjacentNodes(std::shared_ptr<graph_search::Node> extend_node) override;
    void calcHeuristic(std::shared_ptr<graph_search::Node> node) override;

    bool checkTransitionImpl(const ContactState& preState,
			     ContactState& postState,
			     const std::vector<cnoid::LinkPtr>& variables,
			     const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& constraints,
			     const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections,
			     const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
			     const prioritized_inverse_kinematics_solver2::IKParam& pikParam,
			     const global_inverse_kinematics_solver::GIKParam& gikParam
			     ); // preStateからPostStateまでの遷移が可能ならtrue. trueのとき、postStateのframeやlocalPoseを書き換える.
    bool solveContactIK(const ContactState& preState,
			Contact& moveContact,
			ContactState& postState,
			const IKState& ikState,
			const std::vector<cnoid::LinkPtr>& variables,
			const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& constraints,
			const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections,
			const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
			prioritized_inverse_kinematics_solver2::IKParam pikParam,
			global_inverse_kinematics_solver::GIKParam gikParam
		        );
    bool solve();
    void goalPath(std::vector<ContactState>& path);
    class GSCPParam {
    public:
      std::vector<cnoid::BodyPtr> bodies; // デストラクトされないように保持しておく
      std::vector<cnoid::LinkPtr> variables;
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints;
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejections;
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
      std::shared_ptr<ContactState> currentContactState;
      std::shared_ptr<ContactState> goalContactState;
      std::vector<std::shared_ptr<ContactCandidate> > contactDynamicCandidates; // 
      std::vector<std::shared_ptr<ContactCandidate> > contactStaticCandidates; // staticCondidate同士の接触は起こりえない
      prioritized_inverse_kinematics_solver2::IKParam pikParam;
      global_inverse_kinematics_solver::GIKParam gikParam;
      std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField> field;
      std::shared_ptr<choreonoid_viewer::Viewer> viewer = nullptr;

      GSCPParam() {
	gikParam.maxTranslation = 0.5;
	gikParam.pikParam.maxIteration = 100; // max iterationに達するか、convergeしたら終了する. isSatisfiedでは終了しない. ゼロ空間でreference angleに可能な限り近づけるタスクがあるので. 1 iterationで0.5msくらいかかるので、stateを1つ作るための時間の上限が見積もれる. 一見、この値を小さくすると早くなりそうだが、goalSampling時に本当はgoalに到達できるのにその前に返ってしまうことで遅くなることがあるため、少ないiterationでも収束するように他のパラメータを調整したほうがいい
	gikParam.pikParam.checkFinalState = true;
	gikParam.pikParam.calcVelocity = false;
	gikParam.delta = 0.2; // この距離内のstateは、中間のconstraintチェック無しで遷移可能. stateごとの距離がこの距離以内だとそもそも同じstateとみなされてあたらしくstateを作らない. 足を浮かせるとき等はstateが大きく変化しないので、deltaも小さくしておかないとstateが増えない.
	gikParam.projectCellSize = 0.02;
	gikParam.threads = 1;
	gikParam.timeout = 0.5;
	gikParam.goalBias = 0.2;
	gikParam.pikParam.we = 1e2; // 逆運動学が振動しないこと優先. 1e0だと不安定. 1e3だと大きすぎる
	gikParam.pikParam.wmax = 1e1; // 1e2程度にすると関節がめり込まなくなるが、ほとんど動かない.
	gikParam.pikParam.convergeThre = 5e-3;

	pikParam.checkFinalState=true;
	pikParam.calcVelocity = false;
	pikParam.debugLevel = 0;
	pikParam.we = 1e2;
	pikParam.wmax = 1e1;
	pikParam.convergeThre = 5e-3;
	pikParam.maxIteration = 100;
      }
    };
    ContactPlanner() {}
    GSCPParam param;
  };
}

#endif
