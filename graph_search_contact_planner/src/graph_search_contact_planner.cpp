#include <graph_search_contact_planner/contact_graph.h>
#include <graph_search_contact_planner/contact_node.h>
#include <graph_search_contact_planner/util.h>

namespace graph_search_contact_planner{
  bool ContactPlanner::solve() {
    std::shared_ptr<ContactNode> current_node = std::make_shared<ContactNode>();
    current_node->state() = *(this->param.currentContactState);
    this->graph().push_back(current_node);

    return this->search();
  }

  bool ContactPlanner::isGoalSatisfied(std::shared_ptr<graph_search::Node> node) {
    ContactState state = std::static_pointer_cast<ContactNode>(node)->state();
    for (int i=0;i<this->param.goalContactState->contacts.size();i++) {
      bool satisfied = false;
      for (int j=0;j<state.contacts.size();j++) {
	if (this->param.goalContactState->contacts[i] == state.contacts[j]) satisfied = true;
      }
      if (!satisfied) return false;
    }
    return true;
  }

  void ContactPlanner::calcHeuristic(std::shared_ptr<graph_search::Node> node) {
    // goalContactStateのcontactsのうち満たしていないもの
    // TODO 追加する
    ContactState state = std::static_pointer_cast<ContactNode>(node)->state();
    int unsatisfied_num = 0;
    for (int i=0;i<this->param.goalContactState->contacts.size();i++) {
      bool satisfied = false;
      for (int j=0;j<state.contacts.size();j++) {
	if (this->param.goalContactState->contacts[i] == state.contacts[j]) satisfied = true;
      }
      if (!satisfied) unsatisfied_num++;
    }
    node->heuristic() = unsatisfied_num;
  }

  std::vector<std::shared_ptr<graph_search::Node> > ContactPlanner::gatherAdjacentNodes(std::shared_ptr<graph_search::Node> extend_node) {
    ContactState extend_state = std::static_pointer_cast<ContactNode>(extend_node)->state();
    if (this->debugLevel() >= 1) {
      std::cerr << "extend_state" << std::endl;
      std::cerr << extend_state << std::endl;
    }
    std::vector<std::shared_ptr<ContactNode> > adjacentNodeCandidates;
    // 接触の減少
    if (extend_state.contacts.size() >= 2) {
      for (int i=0; i<extend_state.contacts.size();i++) {
	std::shared_ptr<ContactNode> newNode = std::make_shared<ContactNode>();
	newNode->parent() = extend_node;
	newNode->state() = extend_state;
	newNode->state().contacts.erase(newNode->state().contacts.begin()+i);
	adjacentNodeCandidates.push_back(newNode);
      }
    }

    // 接触の追加.
    // static contact
    for (int i=0; i<param.contactDynamicCandidates.size(); i++) {
      // staticCandidateと接触しているものを更にstaticCandidateと接触させることはしない
      bool skip = false;
      for (int j=0; j<extend_state.contacts.size(); j++) {
	if (((param.contactDynamicCandidates[i]->name == extend_state.contacts[j].c1.name) && (extend_state.contacts[j].c2.isStatic)) ||
	    ((param.contactDynamicCandidates[i]->name == extend_state.contacts[j].c2.name) && (extend_state.contacts[j].c1.isStatic))) {
	  skip = true;
	  break;
	}
      }
      if (skip) continue;

      for (int j=0; j<param.contactStaticCandidates.size(); j++) {
	std::shared_ptr<ContactNode> newNode = std::make_shared<ContactNode>();
	newNode->parent() = extend_node;
	newNode->state() = extend_state;
	newNode->state().contacts.push_back(Contact(*(param.contactDynamicCandidates[i]), *(param.contactStaticCandidates[j])));
	adjacentNodeCandidates.push_back(newNode);
      }
    }

    // dynamic contact
    for (int i=0; i<param.contactDynamicCandidates.size(); i++) {
      for (int j=i+1; j<param.contactDynamicCandidates.size(); j++) {
	std::shared_ptr<ContactNode> newNode = std::make_shared<ContactNode>();
	newNode->parent() = extend_node;
	newNode->state() = extend_state;
	newNode->state().contacts.push_back(Contact(*(param.contactDynamicCandidates[i]), *(param.contactDynamicCandidates[j])));
	adjacentNodeCandidates.push_back(newNode);
      }
    }

    // 再訪しない
    for (int i=0;i<this->graph().size();i++) {
      for(int j=0;j<adjacentNodeCandidates.size();j++) {
	if (std::static_pointer_cast<ContactNode>(this->graph()[i])->state() == adjacentNodeCandidates[j]->state()) {
	  adjacentNodeCandidates.erase(adjacentNodeCandidates.begin()+j);
	  break;
	}
      }
    }

    std::vector<std::shared_ptr<graph_search::Node> > adjacentNodes;
    for (int i=0; i<adjacentNodeCandidates.size(); i++) {
      if(checkTransition(extend_state, adjacentNodeCandidates[i]->state())) adjacentNodes.push_back(adjacentNodeCandidates[i]);
    }

    if (this->debugLevel() >= 1) {
      std::cerr << "adjacent node : " << adjacentNodes.size() << " in " << adjacentNodeCandidates.size() << " candidates"<< std::endl;
    }

    return adjacentNodes;
  }

  bool ContactPlanner::checkTransition(ContactState preState, ContactState& postState) {
    if (this->debugLevel() >= 2) {
      std::cerr << "[GraphSearchContactPlanner] checkTransition" << std::endl;
      std::cerr << "preState" << std::endl;
      std::cerr << preState << std::endl;
      std::cerr << "postState" << std::endl;
      std::cerr << postState << std::endl;
    }

    global_inverse_kinematics_solver::frame2Link(preState.frame, param.variables);
    for(int i=0; i<param.robots.size(); i++) {
      param.robots[i]->calcForwardKinematics(false);
      param.robots[i]->calcCenterOfMass();
    }
    postState.transition.clear();

    if (postState.contacts.size() > preState.contacts.size()) {
      // attach
      if (!solveContactIK(preState, postState.contacts.back(), postState, IKState::ATTACH_PRE)) return false;
      if (!solveContactIK(preState, postState.contacts.back(), postState, IKState::ATTACH_FIXED)) return false;
    } else if (postState.contacts.size() < preState.contacts.size()) {
      // detach
      bool find_detach_contact = false;
      Contact moveContact;
      for(int i=0;i<preState.contacts.size() && !find_detach_contact;i++) {
	if(std::find(postState.contacts.begin(), postState.contacts.end(), preState.contacts[i]) == postState.contacts.end()) {
	  moveContact = preState.contacts[i];
	  find_detach_contact = true;
	}
      }
      if (!find_detach_contact) {
	std::cerr << "[GraphSearchContactPlanner] checkTransition failed!! cannot find detach contact" << std::endl;
	return false;
      }
      if (!solveContactIK(postState, moveContact, postState, IKState::DETACH_FIXED)) return false;
    } else {
      std::cerr << "[GraphSearchContactPlanner] checkTransition failed!! postState.contacts.size() is same as preState.contacts.size()" << std::endl;
      return false;
    }
    global_inverse_kinematics_solver::link2Frame(param.variables, postState.frame);
    return true;
  }

  void ContactPlanner::goalPath(std::vector<ContactState>& path) {
    if (!this->goal()) {
      std::cerr << "[ContactPlanner] goal not found!!" << std::endl;
    } else {
      path.clear();
      path.push_back(std::static_pointer_cast<graph_search_contact_planner::ContactNode>(this->goal())->state());
      std::weak_ptr<graph_search::Node> node = this->goal()->parent();
      while (!node.expired()) {
	path.push_back(std::static_pointer_cast<graph_search_contact_planner::ContactNode>(node.lock())->state());
	node = node.lock()->parent();
      }
    }
    std::reverse(path.begin(), path.end());
  }

}
