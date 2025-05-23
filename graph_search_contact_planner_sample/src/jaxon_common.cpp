#include "jaxon_common.h"
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <ros/package.h>

namespace graph_search_contact_planner_sample{
  void generateJAXON(const std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField>& field,
		     graph_search_contact_planner::ContactPlanner::GSCPParam& param
		     ) {
    cnoid::BodyLoader bodyLoader;
    cnoid::BodyPtr robot = bodyLoader.load(ros::package::getPath("jvrc_models") + "/JAXON_JVRC/JAXON_JVRCmain.wrl");
    if(!robot) std::cerr << "!robot" << std::endl;
    param.bodies.push_back(robot);
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
	c1.name = "RLEG_JOINT5";
	c1.isStatic = false;
	c1.localPose.translation() = cnoid::Vector3(0, 0, -0.1);
	graph_search_contact_planner::ContactCandidate c2;
	c2.name = "floor2";
	c2.isStatic = true;
	c2.localPose.translation() = cnoid::Vector3(0.0, -0.1, 0.0);
	c2.localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
	param.currentContactState->contacts.push_back(graph_search_contact_planner::Contact(c1,c2));
      }
      // lleg
      {
	graph_search_contact_planner::ContactCandidate c1;
	c1.name = "LLEG_JOINT5";
	c1.isStatic = false;
	c1.localPose.translation() = cnoid::Vector3(0, 0, -0.1);
	graph_search_contact_planner::ContactCandidate c2;
	c2.name = "floor2";
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
	rleg->name = "RLEG_JOINT5";
	rleg->isStatic = false;
	rleg->localPose.translation() = cnoid::Vector3(0,0,-0.11);
	param.contactDynamicCandidates.push_back(rleg);
      }
      // lleg
      {
	std::shared_ptr<graph_search_contact_planner::ContactCandidate> lleg = std::make_shared<graph_search_contact_planner::ContactCandidate>();
	lleg->name = "LLEG_JOINT5";
	lleg->isStatic = false;
	lleg->localPose.translation() = cnoid::Vector3(0,0,-0.11);
	param.contactDynamicCandidates.push_back(lleg);
      }
      // rarm
      {
	std::shared_ptr<graph_search_contact_planner::ContactCandidate> rarm = std::make_shared<graph_search_contact_planner::ContactCandidate>();
        rarm->name = "RARM_JOINT7";
	rarm->isStatic = false;
        rarm->localPose.translation() = cnoid::Vector3(0,0,-0.22);
	param.contactDynamicCandidates.push_back(rarm);
      }
      // larm
      {
	std::shared_ptr<graph_search_contact_planner::ContactCandidate> larm = std::make_shared<graph_search_contact_planner::ContactCandidate>();
        larm->name = "LARM_JOINT7";
	larm->isStatic = false;
        larm->localPose.translation() = cnoid::Vector3(0,0,-0.22);
	param.contactDynamicCandidates.push_back(larm);
      }
    }

  }
}
