#include <graph_search_contact_planner/contact_graph.h>
#include <ik_constraint2_scfr/SlideScfrConstraint.h>
#include <set>

namespace graph_search_contact_planner{
  bool ContactPlanner::solveContactIK(const ContactState& preState,
				      Contact& moveContact,
				      ContactState& postState,
				      const IKState& ikState,
				      const std::vector<cnoid::LinkPtr>& variables,
				      const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& constraints,
				      const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections,
				      const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
				      prioritized_inverse_kinematics_solver2::IKParam pikParam,
				      global_inverse_kinematics_solver::GIKParam gikParam
				      ) {
    std::shared_ptr<std::vector<std::vector<double> > > tmpPath = std::make_shared<std::vector<std::vector<double> > >();
    std::set<cnoid::BodyPtr> bodies;
    for(size_t i=0;i<variables.size();i++){
      if(variables[i]->body()) bodies.insert(variables[i]->body());
    }
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0; // 幾何干渉や重心制約. 
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1; // 動かさない接触
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints2; // 動かす接触

    for (int i=0; i<constraints.size(); i++) {
      bool skip=false;
      if (typeid(*(param.constraints[i]))==typeid(ik_constraint2_distance_field::DistanceFieldCollisionConstraint)) {
	for (int j=0; j<preState.contacts.size() && !skip; j++) {
	  if (((preState.contacts[j].c1.name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param.constraints[i])->A_link()->name()) && preState.contacts[j].c2.isStatic) ||
	      ((preState.contacts[j].c2.name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param.constraints[i])->A_link()->name()) && preState.contacts[j].c1.isStatic)) skip = true;
	}
	if (!skip ||
	    (ikState==IKState::ATTACH_FIXED) ||
	    (ikState==IKState::DETACH_FIXED)) {
	  if ((moveContact.c1.name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param.constraints[i])->A_link()->name()) ||
	      (moveContact.c2.name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param.constraints[i])->A_link()->name())) skip = true;
	}
      }
      if (typeid(*(param.constraints[i]))==typeid(ik_constraint2_bullet::BulletCollisionConstraint)) {
	for (int j=0; j<preState.contacts.size() && !skip; j++) {
	  if (((preState.contacts[j].c1.name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param.constraints[i])->A_link()->name()) && (preState.contacts[j].c2.name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param.constraints[i])->B_link()->name())) ||
	      ((preState.contacts[j].c2.name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param.constraints[i])->A_link()->name()) && (preState.contacts[j].c1.name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param.constraints[i])->B_link()->name()))) skip = true;
	}
	if (!skip ||
	    (ikState==IKState::ATTACH_FIXED) ||
	    (ikState==IKState::DETACH_FIXED)) {
	  if (((moveContact.c1.name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param.constraints[i])->A_link()->name()) && (moveContact.c2.name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param.constraints[i])->B_link()->name())) ||
	      ((moveContact.c2.name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param.constraints[i])->A_link()->name()) && (moveContact.c1.name == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param.constraints[i])->B_link()->name()))) skip = true;
	}
      }
      if (!skip) constraints0.push_back(constraints[i]);
    }
    // scfrConstraint
    std::vector<std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> > scfrConstraints;
    for (std::set<cnoid::BodyPtr>::iterator it=bodies.begin(); it != bodies.end(); it++) {
      std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> scfrConstraint = std::make_shared<ik_constraint2_scfr::ScfrConstraint>();
      scfrConstraint->A_robot() = (*it);
      scfrConstraints.push_back(scfrConstraint);
    }

    {
      for (int i=0; i<preState.contacts.size(); i++) {
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
	if (preState.contacts[i].c1.isStatic) { constraint->A_link() = nullptr; }
	else {
	  for (std::set<cnoid::BodyPtr>::iterator it=bodies.begin(); it != bodies.end(); it++) {
	    if ((*it)->joint(preState.contacts[i].c1.name)) {
	      constraint->A_link() = (*it)->joint(preState.contacts[i].c1.name);
	      break;
	    }
	  }
	  if (!constraint->A_link()) std::cerr << "[GraphSearchContactPlanner] error!! bodies do not have preState.contacts[i].c1.name" << std::endl;
	}
        constraint->A_localpos() = preState.contacts[i].c1.localPose;
	if (preState.contacts[i].c2.isStatic) { constraint->B_link() = nullptr; }
	else {
	  for (std::set<cnoid::BodyPtr>::iterator it=bodies.begin(); it != bodies.end(); it++) {
	    if ((*it)->joint(preState.contacts[i].c2.name)) {
	      constraint->B_link() = (*it)->joint(preState.contacts[i].c2.name);
	      break;
	    }
	  }
	  if (!constraint->B_link()) std::cerr << "[GraphSearchContactPlanner] error!! bodies do not have preState.contacts[i].c2.name" << std::endl;
	}
        constraint->B_localpos() = preState.contacts[i].c2.localPose;
	constraint->B_localpos().linear() = constraint->B_localpos().linear() * cnoid::rotFromRpy(0.0, M_PI, M_PI/2).transpose(); // scfrを作る関係上localposのZはrobotの内側を向いている. PositionConstraintで一致させるためにZの向きを揃える.
        constraint->eval_link() = constraint->B_link();
	constraint->eval_localR() = constraint->B_localpos().linear();
        constraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        constraints1.push_back(constraint);
	Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6);
	C.insert(0,2) = 1.0;
	C.insert(1,0) = 1.0; C.insert(1,2) = 0.2;
	C.insert(2,0) = -1.0; C.insert(2,2) = 0.2;
	C.insert(3,1) = 1.0; C.insert(3,2) = 0.2;
	C.insert(4,1) = -1.0; C.insert(4,2) = 0.2;
	C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
	C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
	C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
	C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
	C.insert(9,2) = 0.005; C.insert(9,5) = 1.0;
	C.insert(10,2) = 0.005; C.insert(10,5) = -1.0;
	cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
	cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
	du[0] = 20000.0;
	for (int j=0;j<scfrConstraints.size();j++) {
	  if (scfrConstraints[j]->A_robot()->joint(preState.contacts[i].c1.name)) {
	    scfrConstraints[j]->links().push_back(scfrConstraints[j]->A_robot()->joint(preState.contacts[i].c1.name));
	    scfrConstraints[j]->poses().push_back(preState.contacts[i].c1.localPose);
	    scfrConstraints[j]->As().emplace_back(0,6);
	    scfrConstraints[j]->bs().emplace_back(0);
	    scfrConstraints[j]->Cs().push_back(C);
	    scfrConstraints[j]->dls().push_back(dl);
	    scfrConstraints[j]->dus().push_back(du);
	  }
	  if (scfrConstraints[j]->A_robot()->joint(preState.contacts[i].c2.name)) {
	    scfrConstraints[j]->links().push_back(scfrConstraints[j]->A_robot()->joint(preState.contacts[i].c2.name));
	    scfrConstraints[j]->poses().push_back(preState.contacts[i].c2.localPose);
	    scfrConstraints[j]->As().emplace_back(0,6);
	    scfrConstraints[j]->bs().emplace_back(0);
	    scfrConstraints[j]->Cs().push_back(C);
	    scfrConstraints[j]->dls().push_back(dl);
	    scfrConstraints[j]->dus().push_back(du);
	  }
	}
      }
    }

    std::shared_ptr<ik_constraint2::PositionConstraint> moveContactConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
    if (moveContact.c1.isStatic) { moveContactConstraint->A_link() = nullptr; }
    else {
      for (std::set<cnoid::BodyPtr>::iterator it=bodies.begin(); it != bodies.end(); it++) {
	if ((*it)->joint(moveContact.c1.name)) {
	  moveContactConstraint->A_link() = (*it)->joint(moveContact.c1.name);
	  break;
	}
      }
      if (!moveContactConstraint->A_link()) std::cerr << "[GraphSearchContactPlanner] error!! bodies do not have postState.contacts[i].c1.name" << std::endl;
    }
    moveContactConstraint->A_localpos() = moveContact.c1.localPose;
    if (moveContact.c2.isStatic) { moveContactConstraint->B_link() = nullptr; }
    else {
      for (std::set<cnoid::BodyPtr>::iterator it=bodies.begin(); it != bodies.end(); it++) {
	if ((*it)->joint(moveContact.c2.name)) {
	  moveContactConstraint->B_link() = (*it)->joint(moveContact.c2.name);
	  break;
	}
      }
      if (!moveContactConstraint->B_link()) std::cerr << "[GraphSearchContactPlanner] error!! bodies do not have postState.contacts[i].c2.name" << std::endl;
    }
    moveContactConstraint->B_localpos() = moveContact.c2.localPose;
    moveContactConstraint->B_localpos().linear() = moveContactConstraint->B_localpos().linear() * cnoid::rotFromRpy(0.0, M_PI, M_PI/2).transpose(); // scfrを作る関係上localposのZはrobotの内側を向いている. PositionConstraintで一致させるために回転だけ逆にする.

    if ((ikState==IKState::DETACH_FIXED) ||
	(ikState==IKState::ATTACH_PRE)) {
      if (moveContactConstraint->B_link()) moveContactConstraint->B_localpos().translation() += moveContactConstraint->B_link()->R() * moveContactConstraint->B_localpos().linear() * cnoid::Vector3(0,0,0.02);
      else moveContactConstraint->B_localpos().translation() += moveContactConstraint->B_localpos().linear() * cnoid::Vector3(0,0,0.02);
    }
    moveContactConstraint->eval_link() = moveContactConstraint->B_link();
    moveContactConstraint->eval_localR() = moveContactConstraint->B_localpos().linear();
    moveContactConstraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 0.0;
    constraints2.push_back(moveContactConstraint);

    for (int i=0;i<scfrConstraints.size();i++) constraints0.push_back(scfrConstraints[i]);

    bool solved = false;
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraint{constraints0, constraints1, constraints2, nominals};

    std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks;
    solved  =  prioritized_inverse_kinematics_solver2::solveIKLoop(variables,
                                                                   constraint,
								   rejections,
                                                                   prevTasks,
                                                                   pikParam,
                                                                   tmpPath
                                                                   );
    if(!solved) {
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > gikConstraints{constraints0, constraints1};
      gikParam.projectLink.resize(1);
      gikParam.projectLink[0] = moveContactConstraint->A_link() ? moveContactConstraint->A_link() : moveContactConstraint->B_link();
      gikParam.projectLocalPose = moveContactConstraint->A_link() ? moveContactConstraint->A_localpos() : moveContactConstraint->B_localpos();
      // 関節角度上下限を厳密に満たしていないと、omplのstart stateがエラーになるので
      for(int i=0;i<variables.size();i++){
        if(variables[i]->isRevoluteJoint() || variables[i]->isPrismaticJoint()) {
          variables[i]->q() = std::max(std::min(variables[i]->q(),variables[i]->q_upper()),variables[i]->q_lower());
        }
      }
      solved = global_inverse_kinematics_solver::solveGIK(variables,
                                                          gikConstraints,
                                                          constraints2,
                                                          nominals,
                                                          gikParam,
                                                          tmpPath);
    }

    postState.transition.insert(postState.transition.end(), (*tmpPath).begin(), (*tmpPath).end());

    moveContact.c1.localPose.linear() = moveContactConstraint->A_localpos().linear();
    cnoid::Matrix3d B_rot = cnoid::Matrix3d::Identity();
    if (moveContactConstraint->B_link()) B_rot = moveContactConstraint->B_link()->R();
    cnoid::Matrix3d A_rot;
    if (moveContactConstraint->A_link()) A_rot = moveContactConstraint->A_link()->R() * moveContactConstraint->A_localpos().linear();
    else A_rot = moveContactConstraint->A_localpos().linear();
    moveContact.c2.localPose.linear() = (B_rot.transpose() * A_rot) * cnoid::rotFromRpy(0.0, M_PI, M_PI/2);

    return solved;

  }
  std::vector<cnoid::SgNodePtr> generateCandidateMakers(const std::vector<cnoid::LinkPtr>& variables, const std::vector<std::shared_ptr<ContactCandidate> >& ccs) {
    std::set<cnoid::BodyPtr> bodies;
    for(size_t i=0;i<variables.size();i++){
      if(variables[i]->body()) bodies.insert(variables[i]->body());
    }

    std::vector<cnoid::SgNodePtr> drawOnObjects;
    for(int i=0;i<ccs.size();i++) {
      cnoid::SgLineSetPtr lines = new cnoid::SgLineSet;
      lines->setLineWidth(5.0);
      lines->getOrCreateColors()->resize(3);
      lines->getOrCreateColors()->at(0) = cnoid::Vector3f(1.0,0.0,0.0);
      lines->getOrCreateColors()->at(1) = cnoid::Vector3f(0.0,1.0,0.0);
      lines->getOrCreateColors()->at(2) = cnoid::Vector3f(0.0,0.0,1.0);
      lines->getOrCreateVertices()->resize(4);
      lines->colorIndices().resize(0);
      lines->addLine(0,1); lines->colorIndices().push_back(0); lines->colorIndices().push_back(0);
      lines->addLine(0,2); lines->colorIndices().push_back(1); lines->colorIndices().push_back(1);
      lines->addLine(0,3); lines->colorIndices().push_back(2); lines->colorIndices().push_back(2);
      cnoid::Isometry3 pose;
      if(ccs[i]->isStatic) pose = ccs[i]->localPose;
      else {
	for (std::set<cnoid::BodyPtr>::iterator it=bodies.begin(); it != bodies.end(); it++) {
	  if((*it)->joint(ccs[i]->name)) {
	    pose = (*it)->joint(ccs[i]->name)->T() * ccs[i]->localPose;
	  }
	}
      }
      lines->getOrCreateVertices()->at(0) = pose.translation().cast<cnoid::Vector3f::Scalar>();
      lines->getOrCreateVertices()->at(1) = (pose * (0.05 * cnoid::Vector3::UnitX())).cast<cnoid::Vector3f::Scalar>();
      lines->getOrCreateVertices()->at(2) = (pose * (0.05 * cnoid::Vector3::UnitY())).cast<cnoid::Vector3f::Scalar>();
      lines->getOrCreateVertices()->at(3) = (pose * (0.05 * cnoid::Vector3::UnitZ())).cast<cnoid::Vector3f::Scalar>();
      drawOnObjects.push_back(lines);
    }
    return drawOnObjects;
  }

}
