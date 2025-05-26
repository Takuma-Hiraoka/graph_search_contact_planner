cmake_minimum_required(VERSION 3.0.2)
project(graph_search_contact_planner_sample)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  graph_search_contact_planner
  choreonoid_viewer
  ik_constraint2_distance_field
  ik_constraint2_bullet
  choreonoid_bullet
  roslib
  )

find_package(choreonoid REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  )

add_library(CnoidGraphSearchContactPlannerSamplePlugin SHARED
  src/CnoidGraphSearchContactPlannerSamplePlugin.cpp
  src/sample0_display.cpp
  src/sample1_box.cpp
  src/world_common.cpp
  src/jaxon_common.cpp
  )

target_link_libraries(CnoidGraphSearchContactPlannerSamplePlugin  PUBLIC
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )

set_target_properties(CnoidGraphSearchContactPlannerSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
