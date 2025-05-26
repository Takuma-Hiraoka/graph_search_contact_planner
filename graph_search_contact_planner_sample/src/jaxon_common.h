#ifndef GRAPH_SEARCH_CONTACT_PLANNER_SAMPLE_JAXON_COMMON_H
#define GRAPH_SEARCH_CONTACT_PLANNER_SAMPLE_JAXON_COMMON_H

#include <graph_search_contact_planner/contact_graph.h>
#include <graph_search_contact_planner/contact_node.h>
#include <ik_constraint2_distance_field/ik_constraint2_distance_field.h>

namespace graph_search_contact_planner_sample{
  void generateJAXON(graph_search_contact_planner::ContactPlanner::GSCPParam& param
		     );
}
#endif
