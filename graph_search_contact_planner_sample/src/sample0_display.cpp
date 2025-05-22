#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <ros/package.h>
#include "world_common.h"
#include "jaxon_common.h"

namespace graph_search_contact_planner_sample{
  void sample0_display(){
    cnoid::BodyPtr obstacle;
    std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField> field;
    graph_search_contact_planner::ContactPlanner planner;
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    planner.param.viewer = viewer;
    generateStepWorld(obstacle, field, planner.param);
    generateJAXON(field, planner.param);

    std::shared_ptr<graph_search_contact_planner::ContactState> goalContactState = std::make_shared<graph_search_contact_planner::ContactState>();
    goalContactState->contacts.push_back(graph_search_contact_planner::Contact(graph_search_contact_planner::ContactCandidate("RARM_JOINT7"), graph_search_contact_planner::ContactCandidate("floor1")));
    goalContactState->contacts.push_back(graph_search_contact_planner::Contact(graph_search_contact_planner::ContactCandidate("LARM_JOINT7"), graph_search_contact_planner::ContactCandidate("floor1")));
    planner.param.goalContactState = goalContactState;
    planner.debugLevel() = 1;

    viewer->objects(obstacle);
    for(int i=0; i<planner.param.robots.size(); i++) viewer->objects(planner.param.robots[i]);

    std::vector<cnoid::SgNodePtr> drawOnObjects;
    std::vector<cnoid::SgNodePtr> csc = graph_search_contact_planner::generateCandidateMakers(planner.param.robots, planner.param.contactStaticCandidates);
    std::vector<cnoid::SgNodePtr> cdc = graph_search_contact_planner::generateCandidateMakers(planner.param.robots, planner.param.contactDynamicCandidates);
    drawOnObjects.insert(drawOnObjects.end(), csc.begin(), csc.end());
    drawOnObjects.insert(drawOnObjects.end(), cdc.begin(), cdc.end());

    viewer->drawOn(drawOnObjects);

    viewer->drawObjects();

    if(planner.solve()) {
      std::cerr << "solved!" << std::endl;
      std::vector<graph_search_contact_planner::ContactState> path;
      planner.goalPath(path);
      std::cerr << "path size : " << path.size() << std::endl;
      while(true) {
	for(int i=0;i<path.size();i++){
	  global_inverse_kinematics_solver::frame2Link(path[i].frame, planner.param.variables);
	  for(int j=0; j<planner.param.robots.size(); j++) {
	    planner.param.robots[j]->calcForwardKinematics(false);
	  }
	  viewer->drawObjects();
	  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
      }
    }
  }
}
