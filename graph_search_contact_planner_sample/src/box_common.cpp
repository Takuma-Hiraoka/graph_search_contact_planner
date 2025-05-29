#include "box_common.h"
#include <cnoid/MeshGenerator>
#include <cnoid/BodyLoader>
#include <choreonoid_bullet/choreonoid_bullet.h>
#include <ros/package.h>

namespace graph_search_contact_planner_sample{
  void generateBOX(graph_search_contact_planner::ContactPlanner::GSCPParam& param
		   ) {
    cnoid::MeshGenerator meshGenerator;
    cnoid::BodyLoader bodyLoader;
    cnoid::BodyPtr box = bodyLoader.load(ros::package::getPath("eusurdfwrl") + "/models/maxvalu-supermarket-basket/maxvalu-supermarket-basket.wrl");
    param.bodies.push_back(box);
    param.variables.push_back(box->rootLink());
    box->rootLink()->setName("box");
    box->rootLink()->p() = cnoid::Vector3(0.6,0,1.0);
    box->rootLink()->R() = cnoid::rotFromRpy(0.0, 0.0, M_PI/2);
    box->calcForwardKinematics();
    box->calcCenterOfMass();
    global_inverse_kinematics_solver::link2Frame(param.variables, param.currentContactState->frame);

    // currentContactState
    {
      graph_search_contact_planner::ContactCandidate c1;
      c1.name = "box";
      c1.isStatic = false;
      c1.localPose.translation() = cnoid::Vector3(0, 0, 0.0);
      c1.localPose.linear() = cnoid::rotFromRpy(0.0, 0.0, -M_PI/2);
      graph_search_contact_planner::ContactCandidate c2;
      c2.name = "table1";
      c2.isStatic = true;
      c2.localPose.translation() = cnoid::Vector3(0.6, 0.0, 1.0);
      c2.localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
      param.currentContactState->contacts.push_back(graph_search_contact_planner::Contact(c1,c2));
    }

    // contactDynamicCandidates
    {
      // base
      {
	std::shared_ptr<graph_search_contact_planner::ContactCandidate> base = std::make_shared<graph_search_contact_planner::ContactCandidate>();
	base->name = "box";
	base->isStatic = false;
	base->localPose.translation() = cnoid::Vector3(0,0,0.0);
	base->localPose.linear() = cnoid::rotFromRpy(0.0, 0.0, -M_PI/2);
	param.contactDynamicCandidates.push_back(base);
      }
      // rhandle
      {
	std::shared_ptr<graph_search_contact_planner::ContactCandidate> rhandle = std::make_shared<graph_search_contact_planner::ContactCandidate>();
	rhandle->name = "box";
	rhandle->isStatic = false;
	rhandle->localPose.translation() = cnoid::Vector3(0.15,0.0,0.1);
        rhandle->localPose.linear() = cnoid::rotFromRpy(0.0, -M_PI/2, 0.0);
	param.contactDynamicCandidates.push_back(rhandle);
      }
      // lhandle
      {
	std::shared_ptr<graph_search_contact_planner::ContactCandidate> lhandle = std::make_shared<graph_search_contact_planner::ContactCandidate>();
	lhandle->name = "box";
	lhandle->isStatic = false;
	lhandle->localPose.translation() = cnoid::Vector3(-0.15,0.0,0.1);
        lhandle->localPose.linear() = cnoid::rotFromRpy(0.0, M_PI/2, 0.0);
	param.contactDynamicCandidates.push_back(lhandle);
      }
    }

    // collision
    {
      {
	std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
	constraint->A_link() = box->rootLink();
	constraint->field() = param.field;
	constraint->tolerance() = 0.015; // ちょうど干渉すると法線ベクトルが変になることがあるので, 1回のiterationで動きうる距離よりも大きくせよ.
	constraint->precision() = 0.010; // 角で不正確になりがちなので, toleranceを大きくしてprecisionも大きくして、best effort的にする. precisionはdistanceFieldのサイズの倍数より大きくする. 
	constraint->ignoreDistance() = 0.1; // 大きく動くので、ignoreも大きくする必要がある
	//      constraint->maxError() = 0.1; // めり込んだら一刻も早く離れたい
	constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
	param.constraints.push_back(constraint);
      }

      {
        std::shared_ptr<btConvexShape> collisionModel = choreonoid_bullet::convertToBulletModel(box->rootLink()->collisionShape());
	for(int i=0;i<param.bodies[0]->numLinks();i++){
	  if ((param.bodies[0]->link(i)->name() == "LARM_F_JOINT0") ||
	      (param.bodies[0]->link(i)->name() == "LARM_F_JOINT1") ||
	      (param.bodies[0]->link(i)->name() == "RARM_F_JOINT0") ||
	      (param.bodies[0]->link(i)->name() == "RARM_F_JOINT1")) continue;
	  std::shared_ptr<ik_constraint2_bullet::BulletCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletCollisionConstraint>();
	  constraint->A_link() = box->rootLink();
	  constraint->B_link() = param.bodies[0]->link(i);
	  constraint->A_link_bulletModel() = constraint->A_link();
	  constraint->A_bulletModel().push_back(collisionModel);
	  constraint->B_link_bulletModel() = constraint->B_link();
	  constraint->B_bulletModel().push_back(choreonoid_bullet::convertToBulletModel(param.bodies[0]->link(i)->collisionShape()));
	  constraint->tolerance() = 0.002;
	  constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
	  param.constraints.push_back(constraint);
	}
      }
    }

  }
}
