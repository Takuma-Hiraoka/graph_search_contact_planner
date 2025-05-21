#include <graph_search_contact_planner/contact_graph.h>
#include <ik_constraint2_scfr/SlideScfrConstraint.h>

namespace graph_search_contact_planner{
  bool ContactPlanner::solveContactIK(const ContactState& preState,
                      Contact& moveContact,
                      const IKState ikState,
                      const std::shared_ptr<std::vector<std::vector<double> > > path
                      ) {
    std::shared_ptr<std::vector<std::vector<double> > > tmpPath = std::make_shared<std::vector<std::vector<double> > >();
    std::vector<cnoid::LinkPtr> variables;
    for (int i=0;i<this->param.variables.size(); i++) {
      variables.push_back(this->param.variables[i]);
    }
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
    std::vector<std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> > scfrConstraints;
    for (int i=0;i<this->param.robots.size();i++) {
      std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> scfrConstraint = std::make_shared<ik_constraint2_scfr::ScfrConstraint>();
      scfrConstraint->A_robot() = this->param.robots[i];
      scfrConstraints.push_back(scfrConstraint);
    }

    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints2;
    {
      for (int i=0; i<preState.contacts.size(); i++) {
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
	if (preState.contacts[i].c1.isStatic) { constraint->A_link() = nullptr; }
	else {
	  for (int j=0; j<param.robots.size();j++) {
	    if (param.robots[j]->joint(preState.contacts[i].c1.name)) {
	      constraint->A_link() = param.robots[j]->joint(preState.contacts[i].c1.name);
	      break;
	    }
	  }
	  if (!constraint->A_link()) std::cerr << "[GraphSearchContactPlanner] error!! param.robots do not have preState.contacts[i].c1.name" << std::endl;
	}
        constraint->A_localpos() = preState.contacts[i].c1.localPose;
	if (preState.contacts[i].c2.isStatic) { constraint->B_link() = nullptr; }
	else {
	  for (int j=0; j<param.robots.size();j++) {
	    if (param.robots[j]->joint(preState.contacts[i].c2.name)) {
	      constraint->B_link() = param.robots[j]->joint(preState.contacts[i].c2.name);
	      break;
	    }
	  }
	  if (!constraint->B_link()) std::cerr << "[GraphSearchContactPlanner] error!! param.robots do not have preState.contacts[i].c2.name" << std::endl;
	}
        constraint->B_localpos() = preState.contacts[i].c2.localPose;
	constraint->B_localpos().linear() = constraint->B_localpos().linear().transpose(); // scfrを作る関係上localposのZはrobotの内側を向いている. PositionConstraintで一致させるために回転だけ逆にする.
        constraint->eval_link() = constraint->B_link();
	constraint->eval_localR() = constraint->B_localpos().linear();
        constraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        constraints1.push_back(constraint);
	for (int j=0;j<scfrConstraints.size();j++) {
	  if (scfrConstraints[j]->A_robot()->joint(preState.contacts[i].c1.name)) {
	    scfrConstraints[j]->links().push_back(scfrConstraints[j]->A_robot()->joint(preState.contacts[i].c1.name));
	    scfrConstraints[j]->poses().push_back(preState.contacts[i].c1.localPose);
	    scfrConstraints[j]->As().emplace_back(0,6);
	    scfrConstraints[j]->bs().emplace_back(0);
	    scfrConstraints[j]->Cs().push_back(this->C);
	    scfrConstraints[j]->dls().push_back(this->dl);
	    scfrConstraints[j]->dus().push_back(this->du);
	  }
	}
      }
    }

    std::shared_ptr<ik_constraint2::PositionConstraint> moveContactConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
    if (moveContact.c1.isStatic) { moveContactConstraint->A_link() = nullptr; }
    else {
      for (int j=0; j<param.robots.size();j++) {
	if (param.robots[j]->joint(moveContact.c1.name)) {
	  moveContactConstraint->A_link() = param.robots[j]->joint(moveContact.c1.name);
	  break;
	}
      }
      if (!moveContactConstraint->A_link()) std::cerr << "[GraphSearchContactPlanner] error!! param.robots do not have postState.contacts[i].c1.name" << std::endl;
    }
    moveContactConstraint->A_localpos() = moveContact.c1.localPose;
    if (moveContact.c2.isStatic) { moveContactConstraint->B_link() = nullptr; }
    else {
      for (int j=0; j<param.robots.size();j++) {
	if (param.robots[j]->joint(moveContact.c2.name)) {
	  moveContactConstraint->B_link() = param.robots[j]->joint(moveContact.c2.name);
	  break;
	}
      }
      if (!moveContactConstraint->B_link()) std::cerr << "[GraphSearchContactPlanner] error!! param.robots do not have postState.contacts[i].c2.name" << std::endl;
    }
    moveContactConstraint->B_localpos() = moveContact.c2.localPose;
    moveContactConstraint->B_localpos().linear() = moveContactConstraint->B_localpos().linear().transpose(); // scfrを作る関係上localposのZはrobotの内側を向いている. PositionConstraintで一致させるために回転だけ逆にする.

    if ((ikState==IKState::DETACH_FIXED) ||
	(ikState==IKState::ATTACH_PRE)) moveContactConstraint->B_localpos().translation() += moveContactConstraint->B_link()->R() * moveContactConstraint->B_localpos().linear() * cnoid::Vector3(0,0,0.02);
    moveContactConstraint->eval_link() = moveContactConstraint->B_link();
    moveContactConstraint->eval_localR() = moveContactConstraint->B_localpos().linear();
    moveContactConstraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 0.0;
    constraints2.push_back(moveContactConstraint);

    for (int i=0;i<scfrConstraints.size();i++) constraints0.push_back(scfrConstraints[i]);

    bool solved = false;
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0, constraints1, constraints2, param.nominals};

    std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks;
    solved  =  prioritized_inverse_kinematics_solver2::solveIKLoop(variables,
                                                                   constraints,
                                                                   prevTasks,
                                                                   param.pikParam,
                                                                   tmpPath
                                                                   );
    if(!solved) {
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > gikConstraints{constraints0, constraints1};
      param.gikParam.projectLink.resize(1);
      param.gikParam.projectLink[0] = moveContactConstraint->A_link() ? moveContactConstraint->A_link() : moveContactConstraint->B_link();
      param.gikParam.projectLocalPose = moveContactConstraint->A_link() ? moveContactConstraint->A_localpos() : moveContactConstraint->B_localpos();
      // 関節角度上下限を厳密に満たしていないと、omplのstart stateがエラーになるので
      for(int i=0;i<param.variables.size();i++){
        if(param.variables[i]->isRevoluteJoint() || param.variables[i]->isPrismaticJoint()) {
          param.variables[i]->q() = std::max(std::min(param.variables[i]->q(),param.variables[i]->q_upper()),param.variables[i]->q_lower());
        }
      }
      solved = global_inverse_kinematics_solver::solveGIK(variables,
                                                          gikConstraints,
                                                          constraints2,
                                                          param.nominals,
                                                          param.gikParam,
                                                          tmpPath);
    }

    moveContact.c1.localPose.linear() = moveContactConstraint->A_localpos().linear();
    cnoid::Matrix3d B_rot = cnoid::Matrix3d::Identity();
    if (moveContactConstraint->B_link()) B_rot = moveContactConstraint->B_link()->R();
    cnoid::Matrix3d A_rot;
    if (moveContactConstraint->A_link()) A_rot = moveContactConstraint->A_link()->R() * moveContactConstraint->A_localpos().linear();
    else A_rot = moveContactConstraint->A_localpos().linear();
    moveContact.c2.localPose.linear() = (B_rot.transpose() * A_rot).transpose();//((moveContactConstraint->B_link() ? moveContactConstraint->B_link()->R() : cnoid::Isometry3::Identity().linear()) * (moveContactConstraint->A_link() ? moveContactConstraint->A_link()->R() * moveContactConstraint->A_localpos().linear() : moveContactConstraint->A_localpos().linear())).transpose();

    return solved;

  }

}
