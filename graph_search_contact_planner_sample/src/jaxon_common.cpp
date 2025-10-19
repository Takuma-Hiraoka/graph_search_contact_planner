#include "jaxon_common.h"
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <ros/package.h>
#include <choreonoid_bullet/choreonoid_bullet.h>

namespace graph_search_contact_planner_sample{
  void generateJAXON(graph_search_contact_planner::ContactPlanner::GSCPParam& param
                     ) {
    cnoid::BodyLoader bodyLoader;
    cnoid::BodyPtr robot = bodyLoader.load(ros::package::getPath("jvrc_models") + "/JAXON_JVRC/JAXON_JVRCmain.wrl");
    if(!robot) std::cerr << "!robot" << std::endl;
    param.bodies.push_back(robot);
    robot->setName("JAXON_JVRC");
    // reset manip pose
    robot->rootLink()->p() = cnoid::Vector3(0,0,1.0);
    robot->rootLink()->v().setZero();
    robot->rootLink()->R() = cnoid::Matrix3::Identity();
    robot->rootLink()->w().setZero();
    std::vector<double> reset_manip_pose{
                                         0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// rleg
                                         0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// lleg
                                         0.0, 0.0, 0.0, // torso
                                         0.0, 0.0, // head
                                         0.0, 0.959931, -0.349066, -0.261799, -1.74533, -0.436332, 0.0, -0.785398,// rarm
                                         0.0, 0.959931, 0.349066, 0.261799, -1.74533, 0.436332, 0.0, -0.785398,// larm
                                         -1.39, 1.39, -1.39, 1.39};
    for(int j=0; j < robot->numJoints(); ++j){
      robot->joint(j)->q() = reset_manip_pose[j];
    }
    robot->calcForwardKinematics();
    robot->calcCenterOfMass();

    // variables
    {
      param.variables.push_back(robot->rootLink());
      for(int i=0;i<robot->numJoints();i++){
        if ((robot->joint(i)->name() == "motor_joint") ||
            (robot->joint(i)->name() == "LARM_F_JOINT0") ||
            (robot->joint(i)->name() == "LARM_F_JOINT1") ||
            (robot->joint(i)->name() == "RARM_F_JOINT0") ||
            (robot->joint(i)->name() == "RARM_F_JOINT1")) continue;
        param.variables.push_back(robot->joint(i));
      }
    }

    // task: nominal constairnt
    {
      for(int i=0;i<robot->numJoints();i++){
        std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
        constraint->joint() = robot->joint(i);
        constraint->targetq() = reset_manip_pose[i];
        constraint->precision() = 1e10; // always satisfied
        param.nominals.push_back(constraint);
      }
    }

    {
      param.currentContactState = std::make_shared<graph_search_contact_planner::ContactState>();
      global_inverse_kinematics_solver::link2Frame(param.variables, param.currentContactState->frame);
      // rleg
      {
        graph_search_contact_planner::ContactCandidate c1;
        c1.bodyName = robot->name();
        c1.linkName = "RLEG_JOINT5";
        c1.isStatic = false;
        c1.localPose.translation() = cnoid::Vector3(0, 0, -0.1);
        graph_search_contact_planner::ContactCandidate c2;
        c2.bodyName = "floor";
        c2.linkName = "floor";
        c2.isStatic = true;
        c2.localPose.translation() = cnoid::Vector3(0.0, -0.1, 0.0);
        c2.localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
        param.currentContactState->contacts.push_back(graph_search_contact_planner::Contact(c1,c2));
      }
      // lleg
      {
        graph_search_contact_planner::ContactCandidate c1;
        c1.bodyName = robot->name();
        c1.linkName = "LLEG_JOINT5";
        c1.isStatic = false;
        c1.localPose.translation() = cnoid::Vector3(0, 0, -0.1);
        graph_search_contact_planner::ContactCandidate c2;
        c2.bodyName = "floor";
        c2.linkName = "floor";
        c2.isStatic = true;
        c2.localPose.translation() = cnoid::Vector3(0.0, 0.1, 0.0);
        c2.localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
        param.currentContactState->contacts.push_back(graph_search_contact_planner::Contact(c1,c2));
      }
    }

    // contactDynamicCandidates
    {
      // rleg
      {
        std::shared_ptr<graph_search_contact_planner::ContactCandidate> rleg = std::make_shared<graph_search_contact_planner::ContactCandidate>();
        rleg->bodyName = robot->name();
        rleg->linkName = "RLEG_JOINT5";
        rleg->isStatic = false;
        rleg->localPose.translation() = cnoid::Vector3(0,0,-0.11);
        param.contactDynamicCandidates.push_back(rleg);
      }
      // lleg
      {
        std::shared_ptr<graph_search_contact_planner::ContactCandidate> lleg = std::make_shared<graph_search_contact_planner::ContactCandidate>();
        lleg->bodyName = robot->name();
        lleg->linkName = "LLEG_JOINT5";
        lleg->isStatic = false;
        lleg->localPose.translation() = cnoid::Vector3(0,0,-0.11);
        param.contactDynamicCandidates.push_back(lleg);
      }
      // rarm
      {
        std::shared_ptr<graph_search_contact_planner::ContactCandidate> rarm = std::make_shared<graph_search_contact_planner::ContactCandidate>();
        rarm->bodyName = robot->name();
        rarm->linkName = "RARM_JOINT7";
        rarm->isStatic = false;
        rarm->localPose.translation() = cnoid::Vector3(0,0,-0.22);
        param.contactDynamicCandidates.push_back(rarm);
      }
      // larm
      {
        std::shared_ptr<graph_search_contact_planner::ContactCandidate> larm = std::make_shared<graph_search_contact_planner::ContactCandidate>();
        larm->bodyName = robot->name();
        larm->linkName = "LARM_JOINT7";
        larm->isStatic = false;
        larm->localPose.translation() = cnoid::Vector3(0,0,-0.22);
        param.contactDynamicCandidates.push_back(larm);
      }
    }

    // joint limit
    for(int i=0;i<robot->numJoints();i++){
      std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
      constraint->joint() = robot->joint(i);
      param.constraints.push_back(constraint);
    }

    // environmental collision
    double envCollisionDefaultTolerance = 0.015;
    double envCollisionDefaultPrecision = 0.010;
    for (int i=0; i<robot->numLinks(); i++) {
      {
        if ((robot->link(i)->name() == "LLEG_JOINT4") ||
            (robot->link(i)->name() == "RLEG_JOINT4") ||
            (robot->link(i)->name() == "LARM_F_JOINT0") ||
            (robot->link(i)->name() == "LARM_F_JOINT1") ||
            (robot->link(i)->name() == "RARM_F_JOINT0") ||
            (robot->link(i)->name() == "RARM_F_JOINT1")) continue;
      }
      std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
      constraint->A_link() = robot->link(i);
      constraint->field() = param.field;
      constraint->tolerance() = envCollisionDefaultTolerance; // ちょうど干渉すると法線ベクトルが変になることがあるので, 1回のiterationで動きうる距離よりも大きくせよ.
      constraint->precision() = envCollisionDefaultPrecision; // 角で不正確になりがちなので, toleranceを大きくしてprecisionも大きくして、best effort的にする. precisionはdistanceFieldのサイズの倍数より大きくする. 大きく動くのでつま先が近かったときにつま先は近くならないがかかとが地面にめり込む、ということは起こりうる.
      constraint->ignoreDistance() = 0.1; // 大きく動くので、ignoreも大きくする必要がある
      //      constraint->maxError() = 0.1; // めり込んだら一刻も早く離れたい
      constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
      param.constraints.push_back(constraint);
    }

    // task: self collision
    {
      std::vector<std::vector<std::string> > pairs {
        std::vector<std::string>{"RLEG_JOINT2","LLEG_JOINT2"}, std::vector<std::string>{"RLEG_JOINT2","LLEG_JOINT3"}, std::vector<std::string>{"RLEG_JOINT2","LLEG_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT3","LLEG_JOINT2"}, std::vector<std::string>{"RLEG_JOINT3","LLEG_JOINT3"}, std::vector<std::string>{"RLEG_JOINT3","LLEG_JOINT5"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT5","LLEG_JOINT2"}, std::vector<std::string>{"RLEG_JOINT5","LLEG_JOINT3"}, std::vector<std::string>{"RLEG_JOINT5","LLEG_JOINT5"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT2"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT2"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT6"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT3"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT4"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT5"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT6"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT2","WAIST"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT3","WAIST"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT4","WAIST"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT5","WAIST"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT6","WAIST"}, std::vector<std::string>{"LARM_JOINT2","WAIST"}, std::vector<std::string>{"LARM_JOINT3","WAIST"}, std::vector<std::string>{"LARM_JOINT4","WAIST"}, std::vector<std::string>{"LARM_JOINT5","WAIST"}, std::vector<std::string>{"LARM_JOINT6","WAIST"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT7"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT7"}, std::vector<std::string>{"LARM_JOINT7","WAIST"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT7"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT7","WAIST"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT2"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT2"}
      };
      std::unordered_map<cnoid::LinkPtr, std::shared_ptr<btConvexShape> > collisionModels;
      for(int i=0;i<robot->numLinks();i++){
        collisionModels[robot->link(i)] = choreonoid_bullet::convertToBulletModel(robot->link(i)->collisionShape());
      }

      for(int i=0;i<pairs.size();i++){
        std::shared_ptr<ik_constraint2_bullet::BulletCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletCollisionConstraint>();
        constraint->A_link() = robot->link(pairs[i][0]);
        constraint->B_link() = robot->link(pairs[i][1]);
        constraint->A_link_bulletModel() = constraint->A_link();
        constraint->A_bulletModel().push_back(collisionModels[constraint->A_link()]);
        constraint->B_link_bulletModel() = constraint->B_link();
        constraint->B_bulletModel().push_back(collisionModels[constraint->B_link()]);
        constraint->tolerance() = 0.002;
        constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
        param.constraints.push_back(constraint);
      }
    }

  }
}
