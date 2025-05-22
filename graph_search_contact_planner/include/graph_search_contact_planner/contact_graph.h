#ifndef GRAPH_SEARCH_CONTACT_PLANNER_CONTACT_GRAPH_H
#define GRAPH_SEARCH_CONTACT_PLANNER_CONTACT_GRAPH_H

#include <graph_search/graph.h>
#include <graph_search_contact_planner/util.h>
#include <graph_search_contact_planner/contact_state.h>
#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>

namespace graph_search_contact_planner{
  class ContactPlanner : public graph_search::Planner {
  public:
    bool isGoalSatisfied(std::shared_ptr<graph_search::Node> node) override; // nodeのcontactsがgoalContactStatesのcontactsを含んでいればtrue. nameのみで座標は無視.
    void calcHeuristic(std::shared_ptr<graph_search::Node> node) override;
    std::vector<std::shared_ptr<graph_search::Node> > gatherAdjacentNodes(std::shared_ptr<graph_search::Node> extend_node) override;
    bool checkTransition(ContactState preState, ContactState& postState); // preStateからPostStateまでの遷移が可能ならtrue. trueのとき、postStateのframeやlocalPoseを書き換える.
    bool solveContactIK(const ContactState& preState,
			Contact& moveContact,
			const IKState ikState,
			const std::shared_ptr<std::vector<std::vector<double> > > path = nullptr
			);
    class GSCPParam {
    public:
      std::vector<cnoid::BodyPtr> robots;
      std::vector<cnoid::LinkPtr> variables;
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints;
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
      std::shared_ptr<ContactState> currentContactState;
      std::shared_ptr<ContactState> goalContactState;
      std::vector<std::shared_ptr<ContactCandidate> > contactDynamicCandidates; // 
      std::vector<std::shared_ptr<ContactCandidate> > contactStaticCandidates; // staticCondidate同士の接触は起こりえない
      prioritized_inverse_kinematics_solver2::IKParam pikParam;
      global_inverse_kinematics_solver::GIKParam gikParam;
      std::shared_ptr<choreonoid_viewer::Viewer> viewer = nullptr;
    };
    ContactPlanner() {
      Eigen::SparseMatrix<double,Eigen::RowMajor> C_(11,6);
      C_.insert(0,2) = 1.0;
      C_.insert(1,0) = 1.0; C_.insert(1,2) = 0.2;
      C_.insert(2,0) = -1.0; C_.insert(2,2) = 0.2;
      C_.insert(3,1) = 1.0; C_.insert(3,2) = 0.2;
      C_.insert(4,1) = -1.0; C_.insert(4,2) = 0.2;
      C_.insert(5,2) = 0.05; C_.insert(5,3) = 1.0;
      C_.insert(6,2) = 0.05; C_.insert(6,3) = -1.0;
      C_.insert(7,2) = 0.05; C_.insert(7,4) = 1.0;
      C_.insert(8,2) = 0.05; C_.insert(8,4) = -1.0;
      C_.insert(9,2) = 0.005; C_.insert(9,5) = 1.0;
      C_.insert(10,2) = 0.005; C_.insert(10,5) = -1.0;
      cnoid::VectorX dl_ = Eigen::VectorXd::Zero(11);
      cnoid::VectorX du_ = 1e10 * Eigen::VectorXd::Ones(11);
      du_[0] = 20000.0;
      this->C = C_;
      this->dl = dl_;
      this->du = du_;
    }
    GSCPParam param;
  private:
    Eigen::SparseMatrix<double,Eigen::RowMajor> C;
    cnoid::VectorX dl;
    cnoid::VectorX du;
  };
}

#endif
