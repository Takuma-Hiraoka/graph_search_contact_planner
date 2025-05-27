#include <graph_search_contact_planner/contact_graph.h>
#include <graph_search_contact_planner/contact_node.h>
#include <graph_search_contact_planner/util.h>

namespace graph_search_contact_planner{
  inline std::set<cnoid::BodyPtr> getBodies(const std::vector<cnoid::LinkPtr>& links){
    std::set<cnoid::BodyPtr> bodies;
    for(size_t i=0;i<links.size();i++){
      if(links[i]->body()) bodies.insert(links[i]->body());
    }
    return bodies;
  }

  bool ContactPlanner::solve() {
    std::shared_ptr<ContactNode> current_node = std::make_shared<ContactNode>();
    current_node->state() = *(this->param.currentContactState);
    this->graph().push_back(current_node);

    return this->search();
  }

  std::shared_ptr<graph_search::Planner::TransitionCheckParam> ContactPlanner::generateCheckParam() {
    std::set<cnoid::BodyPtr> bodies = getBodies(this->param.variables);
    std::shared_ptr<ContactTransitionCheckParam> checkParam = std::make_shared<ContactTransitionCheckParam>();
    for(std::set<cnoid::BodyPtr>::iterator it = bodies.begin(); it != bodies.end(); it++){
      checkParam->modelMap[*it] = (*it)->clone();
    }
    checkParam->variables = std::vector<cnoid::LinkPtr>(param.variables.size());
    for(int v=0;v<param.variables.size();v++){
      checkParam->variables[v] = checkParam->modelMap[param.variables[v]->body()]->link(param.variables[v]->index());
    }
    checkParam->constraints = std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(param.constraints.size());
    for(int j=0;j<param.constraints.size();j++){
      checkParam->constraints[j] = param.constraints[j]->clone(checkParam->modelMap);
    }
    checkParam->rejections = std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(param.rejections.size());
    for(int j=0;j<param.rejections.size();j++){
      checkParam->rejections[j] = param.rejections[j]->clone(checkParam->modelMap);
    }
    checkParam->nominals = std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(param.nominals.size());
    for(int j=0;j<param.nominals.size();j++){
      checkParam->nominals[j] = param.nominals[j]->clone(checkParam->modelMap);
    }
    checkParam->pikParam = param.pikParam;
    checkParam->gikParam = param.gikParam;

    return checkParam;
  }

  void ContactPlanner::preCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> extend_node) {
    std::weak_ptr<graph_search::Node> parent = extend_node->parent();
    if (parent.expired()) std::static_pointer_cast<ContactPlanner::ContactTransitionCheckParam>(checkParam)->preState = std::static_pointer_cast<ContactNode>(extend_node)->state(); // 初期状態なので絶対に遷移可能にしておく.
    else std::static_pointer_cast<ContactPlanner::ContactTransitionCheckParam>(checkParam)->preState = std::static_pointer_cast<ContactNode>(parent.lock())->state();
    std::static_pointer_cast<ContactPlanner::ContactTransitionCheckParam>(checkParam)->postState = std::static_pointer_cast<ContactNode>(extend_node)->state();
  }

  void ContactPlanner::postCheckTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam, std::shared_ptr<graph_search::Node> extend_node) {
    std::static_pointer_cast<ContactNode>(extend_node)->state() = std::static_pointer_cast<ContactPlanner::ContactTransitionCheckParam>(checkParam)->postState;
  }

  bool ContactPlanner::checkTransition(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    std::shared_ptr<ContactTransitionCheckParam> contactCheckParam = std::static_pointer_cast<ContactPlanner::ContactTransitionCheckParam>(checkParam);
    return this->checkTransitionImpl(contactCheckParam->preState,
				     contactCheckParam->postState,
				     contactCheckParam->variables,
				     contactCheckParam->constraints,
				     contactCheckParam->rejections,
				     contactCheckParam->nominals,
				     contactCheckParam->pikParam,
				     contactCheckParam->gikParam);
  }

  bool ContactPlanner::isGoalSatisfied(std::shared_ptr<graph_search::Planner::TransitionCheckParam> checkParam) {
    ContactState state = std::static_pointer_cast<ContactPlanner::ContactTransitionCheckParam>(checkParam)->postState;
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
    if (this->debugLevel() >= 2) {
      std::cerr << "extend_state" << std::endl;
      std::cerr << extend_state << std::endl;
    }
    std::vector<std::shared_ptr<graph_search::Node> > adjacentNodes;
    // 接触の減少
    if (extend_state.contacts.size() >= 2) {
      for (int i=0; i<extend_state.contacts.size();i++) {
	std::shared_ptr<ContactNode> newNode = std::make_shared<ContactNode>();
	newNode->parent() = extend_node;
	newNode->state() = extend_state;
	newNode->state().contacts.erase(newNode->state().contacts.begin()+i);
	adjacentNodes.push_back(newNode);
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
	adjacentNodes.push_back(newNode);
      }
    }

    // dynamic contact
    for (int i=0; i<param.contactDynamicCandidates.size(); i++) {
      for (int j=i+1; j<param.contactDynamicCandidates.size(); j++) {
	std::shared_ptr<ContactNode> newNode = std::make_shared<ContactNode>();
	newNode->parent() = extend_node;
	newNode->state() = extend_state;
	newNode->state().contacts.push_back(Contact(*(param.contactDynamicCandidates[i]), *(param.contactDynamicCandidates[j])));
	adjacentNodes.push_back(newNode);
      }
    }

    // 再訪しない
    for (int i=0;i<this->graph().size();i++) {
      for(int j=0;j<adjacentNodes.size();j++) {
	if (std::static_pointer_cast<ContactNode>(this->graph()[i])->state() == std::static_pointer_cast<ContactNode>(adjacentNodes[j])->state()) {
	  adjacentNodes.erase(adjacentNodes.begin()+j);
	  break;
	}
      }
    }

    return adjacentNodes;
  }

  bool ContactPlanner::checkTransitionImpl(const ContactState& preState,
					   ContactState& postState,
					   const std::vector<cnoid::LinkPtr>& variables,
					   const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& constraints,
					   const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections,
					   const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
					   const prioritized_inverse_kinematics_solver2::IKParam& pikParam,
					   const global_inverse_kinematics_solver::GIKParam& gikParam
					   ) {
    if (this->debugLevel() >= 2) {
      std::cerr << "[GraphSearchContactPlanner] checkTransition" << std::endl;
      std::cerr << "preState" << std::endl;
      std::cerr << preState << std::endl;
      std::cerr << "postState" << std::endl;
      std::cerr << postState << std::endl;
    }

    global_inverse_kinematics_solver::frame2Link(preState.frame, variables);
    postState.transition.clear();

    if (postState.contacts.size() > preState.contacts.size()) {
      // attach
      if (!solveContactIK(preState, postState.contacts.back(), postState, IKState::ATTACH_PRE, variables, constraints, rejections, nominals, pikParam, gikParam)) return false;
      if (!solveContactIK(preState, postState.contacts.back(), postState, IKState::ATTACH_FIXED, variables, constraints, rejections, nominals, pikParam, gikParam)) return false;
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
      if (!solveContactIK(postState, moveContact, postState, IKState::DETACH_FIXED, variables, constraints, rejections, nominals, pikParam, gikParam)) return false;
    } else {
      if (preState == postState) return true; // 同じ. はじめの接触.
      std::cerr << "[GraphSearchContactPlanner] checkTransition failed!! postState.contacts.size() is same as preState.contacts.size()" << std::endl;
      return false;
    }
    global_inverse_kinematics_solver::link2Frame(variables, postState.frame);
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
