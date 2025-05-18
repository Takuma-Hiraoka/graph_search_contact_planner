#ifndef GRAPH_SEARCH_CONTACT_PLANNER_CONTACT_GRAPH_H
#define GRAPH_SEARCH_CONTACT_PLANNER_CONTACT_GRAPH_H

#include <graph_search/graph.h>
#include <graph_search_contact_planner/contact_state.h>
#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>

namespace graph_search_contact_planner{
  class ContactPlanner : public graph_search::Planner {
  public:
    bool isGoalSatisfied(std::shared_ptr<graph_search::Node> node) override; // nodeのcontactsがgoalContactStatesのcontactsを含んでいればtrue. nameのみで座標は無視.
    void calcHeuristic(std::shared_ptr<graph_search::Node> node) override;
    std::vector<std::shared_ptr<graph_search::Node> > gatherAdjacentNodes(std::shared_ptr<graph_search::Node> extend_node) override;
    class GSCPParam {
    public:
      cnoid::BodyPtr robot;
      std::vector<cnoid::LinkPtr> variables;
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints;
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
      std::shared_ptr<ContactState> currentContactState;
      std::shared_ptr<ContactState> goalContactState;
      std::vector<std::shared_ptr<ContactCandidate> > contactDynamicCandidates; // 
      std::vector<std::shared_ptr<ContactCandidate> > contactStaticCandidates; // staticCondidate同士の接触は起こりえない
      prioritized_inverse_kinematics_solver2::IKParam pikParam;
      global_inverse_kinematics_solver::GIKParam gikParam;
    };
  private:
    GSCPParam param;
  };
}

#endif
