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

    viewer->objects(obstacle);
    for(int i=0; i<planner.param.robots.size(); i++) viewer->objects(planner.param.robots[i]);

    std::vector<cnoid::SgNodePtr> drawOnObjects;
    std::vector<cnoid::SgNodePtr> csc = graph_search_contact_planner::generateCandidateMakers(planner.param.robots, planner.param.contactStaticCandidates);
    std::vector<cnoid::SgNodePtr> cdc = graph_search_contact_planner::generateCandidateMakers(planner.param.robots, planner.param.contactDynamicCandidates);
    drawOnObjects.insert(drawOnObjects.end(), csc.begin(), csc.end());
    drawOnObjects.insert(drawOnObjects.end(), cdc.begin(), cdc.end());

    viewer->drawOn(drawOnObjects);

    viewer->drawObjects();
  }
}
