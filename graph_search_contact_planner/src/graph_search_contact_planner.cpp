#include <graph_search_contact_planner/contact_graph.h>
#include <graph_search_contact_planner/contact_node.h>
#include <graph_search_contact_planner/util.h>
#include <thread>
#include <mutex>

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

    if (this->param.threads > 1) {
      // copy
      this->modelMaps.clear();
      this->variabless.clear();
      this->constraintss.clear();
      this->rejectionss.clear();
      this->nominalss.clear();
      for (int i=0; i< this->param.threads; i++){
	std::set<cnoid::BodyPtr> bodies = getBodies(param.variables);
	std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
	for(std::set<cnoid::BodyPtr>::iterator it = bodies.begin(); it != bodies.end(); it++){
	  modelMap[*it] = (*it)->clone();
	}
	this->modelMaps.push_back(modelMap); // cloneしたbodyがデストラクトされないように、保管しておく
	this->variabless.push_back(std::vector<cnoid::LinkPtr>(param.variables.size()));
	for(int v=0;v<param.variables.size();v++){
	  this->variabless.back()[v] = modelMap[param.variables[v]->body()]->link(param.variables[v]->index());
	}
	this->constraintss.push_back(std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >(param.constraints.size()));
	for(int j=0;j<param.constraints.size();j++){
	  this->constraintss.back()[j].resize(param.constraints[j].size());
	  for(int k=0;k<param.constraints[j].size();k++){
	    this->constraintss.back()[j][k] = param.constraints[j][k]->clone(modelMap);
	  }
	}
	this->rejectionss.push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(param.rejections.size()));
	for(int j=0;j<param.rejections.size();j++){
	  this->rejectionss.back()[j] = param.rejections[j]->clone(modelMap);
	}
	this->nominalss.push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(param.nominals.size()));
	for(int j=0;j<param.nominals.size();j++){
	  this->nominalss.back()[j] = param.nominals[j]->clone(modelMap);
	}
      } // copy
    }

    bool solved = this->search();
    this->modelMaps.clear();
    this->variabless.clear();
    this->constraintss.clear();
    this->rejectionss.clear();
    this->nominalss.clear();
    return solved;
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
    int numThreads = (this->param.threads > adjacentNodeCandidates.size()) ? adjacentNodeCandidates.size() : this->param.threads;
    if (numThreads > 1) {
      std::shared_ptr<unsigned int> count = std::make_shared<unsigned int>(0);
      std::vector<std::thread *> th(numThreads);
      std::shared_ptr<std::mutex> threadLock = std::make_shared<std::mutex>();
      std::shared_ptr<std::vector<bool> > transitions = std::make_shared<std::vector<bool> >(adjacentNodeCandidates.size(), false);
      for (unsigned int i=0; i < numThreads; i++) {
	th[i] = new std::thread([&count, &threadLock, &transitions, extend_state, &adjacentNodeCandidates, this, i]
				{ return checkTransitionThread(count, threadLock, transitions, extend_state, adjacentNodeCandidates, this->variabless[i], this->constraintss[i], this->rejectionss[i], this->nominalss[i], this->param.pikParam, this->param.gikParam);
				});
      }
      for (unsigned int i=0; i < numThreads; i++) {
	th[i]->join();
	delete th[i];
      }
      for (int i=0; i<adjacentNodeCandidates.size(); i++) {
	if ((*transitions)[i]) adjacentNodes.push_back(adjacentNodeCandidates[i]);
      }

    } else {
      for (int i=0; i<adjacentNodeCandidates.size(); i++) {
	if(checkTransition(extend_state, adjacentNodeCandidates[i]->state(), param.variables, param.constraints, param.rejections, param.nominals, param.pikParam, param.gikParam)) adjacentNodes.push_back(adjacentNodeCandidates[i]);
      }
    }

    if (this->debugLevel() >= 1) {
      std::cerr << "adjacent node : " << adjacentNodes.size() << " in " << adjacentNodeCandidates.size() << " candidates"<< std::endl;
    }

    return adjacentNodes;
  }

  bool ContactPlanner::checkTransitionThread(std::shared_ptr<unsigned int> count,
					     std::shared_ptr<std::mutex> mutex,
					     std::shared_ptr<std::vector<bool> > transitions,
					     ContactState preState,
					     const std::vector<std::shared_ptr<ContactNode> >& adjacentNodeCandidates,
					     const std::vector<cnoid::LinkPtr>& variables,
					     const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
					     const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections,
					     const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
					     prioritized_inverse_kinematics_solver2::IKParam pikParam,
					     global_inverse_kinematics_solver::GIKParam gikParam
					     ) {
    while (true) {
      mutex->lock();
      if ((*count) >= adjacentNodeCandidates.size()) {
        mutex->unlock();
        return true;
      } else {
	unsigned int count_ = *count;
        (*count)++;
        mutex->unlock();
	std::shared_ptr<ContactNode> node = adjacentNodeCandidates[count_];
	if(checkTransition(preState, node->state(), variables, constraints, rejections, nominals, pikParam, gikParam)) {
	  mutex->lock();
	  (*transitions)[count_] = true;
	  mutex->unlock();
	}
      }
    }
  }

  bool ContactPlanner::checkTransition(const ContactState& preState,
				       ContactState& postState,
				       const std::vector<cnoid::LinkPtr>& variables,
				       const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
				       const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections,
				       const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
				       prioritized_inverse_kinematics_solver2::IKParam pikParam,
				       global_inverse_kinematics_solver::GIKParam gikParam
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
