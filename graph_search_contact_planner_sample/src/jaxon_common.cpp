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
    param.robots.push_back(robot);
    if(!robot) std::cerr << "!robot" << std::endl;
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
  }
}
