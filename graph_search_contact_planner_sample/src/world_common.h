#ifndef GRAPH_SEARCH_CONTACT_PLANNER_SAMPLE_WORLD_COMMON_H
#define GRAPH_SEARCH_CONTACT_PLANNER_SAMPLE_WORLD_COMMON_H

#include <cnoid/Body>
#include <ik_constraint2_distance_field/ik_constraint2_distance_field.h>
#include <graph_search_contact_planner/contact_graph.h>

namespace graph_search_contact_planner_sample{
  void generateStepWorld(cnoid::BodyPtr& obstacle,
			 std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField> field,
			 graph_search_contact_planner::ContactPlanner::GSCPParam& param
			 );
}
#endif
