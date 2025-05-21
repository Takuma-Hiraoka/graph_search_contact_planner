#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <ros/package.h>
#include "world_common.h"

namespace graph_search_contact_planner_sample{
  void sample0_display(){
    cnoid::BodyPtr obstacle;
    std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField> field;
    generateStepWorld(obstacle, field);

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(obstacle);

    viewer->drawObjects();
  }
}
