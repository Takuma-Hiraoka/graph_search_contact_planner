#include <graph_search_contact_planner/contact_graph.h>
#include <graph_search_contact_planner/contact_node.h>

namespace graph_search_contact_planner{
  bool ContactPlanner::isGoalSatisfied(std::shared_ptr<graph_search::Node> node) {
    ContactState state = std::static_pointer_cast<ContactNode>(node)->state();
    for (int i=0;i<this->param.goalContactState->contacts.size();i++) {
      bool satisfied = false;
      for (int j=0;j<state.contacts.size();j++) {
	if (((this->param.goalContactState->contacts[i].c1.name == state.contacts[j].c1.name) &&
	     (this->param.goalContactState->contacts[i].c2.name == state.contacts[j].c2.name))
	    || ((this->param.goalContactState->contacts[i].c1.name == state.contacts[j].c2.name) &&
	     (this->param.goalContactState->contacts[i].c2.name == state.contacts[j].c1.name))
	    ) satisfied = true;
      }
      if (!satisfied) return false;
    }
    return true;
  }
  void ContactPlanner::calcHeuristic(std::shared_ptr<graph_search::Node> node) {
    ContactState state = std::static_pointer_cast<ContactNode>(node)->state();
  }
  std::vector<std::shared_ptr<graph_search::Node> > ContactPlanner::gatherAdjacentNodes(std::shared_ptr<graph_search::Node> extend_node) {
    ContactState extend_state = std::static_pointer_cast<ContactNode>(extend_node)->state();
    std::vector<std::shared_ptr<graph_search::Node> > adjacentNodes;
    // 接触の減少
    if (extend_state.contacts.size() >= 2) {
      for (int i=0; i<extend_state.contacts.size();i++) {
	std::shared_ptr<ContactNode> newNode = std::shared_ptr<ContactNode>();
	newNode->parent() = extend_node;
	newNode->state() = extend_state;
	newNode->state().contacts.erase(newNode->state().contacts.begin()+i);
	adjacentNodes.push_back(newNode);
      }
    }

    // 接触の追加. staticCandidateと接触しているものを更にstaticCandidateと接触させることはしない
    for (int i=0; i<param.contactDynamicCandidates.size(); i++) {
      bool skip = false;
      for (int j=0; j<extend_state.contacts.size(); j++) {
	if (((param.contactDynamicCandidates[i]->name == extend_state.contacts[j].c1.name) && (extend_state.contacts[j].c2.isStatic)) ||
	    ((param.contactDynamicCandidates[i]->name == extend_state.contacts[j].c2.name) && (extend_state.contacts[j].c1.isStatic))) {
	  skip = true;
	  break;
	}
      }
      if (skip) continue;
      
    }
    return std::vector<std::shared_ptr<graph_search::Node> >{nullptr};
  }
}
