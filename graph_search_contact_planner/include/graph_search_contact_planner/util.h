#ifndef GRAPH_SEARCH_CONTACT_PLANNER_UTIL_H
#define GRAPH_SEARCH_CONTACT_PLANNER_UTIL_H

#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <graph_search_contact_planner/contact_state.h>

namespace graph_search_contact_planner{
  enum class IKState
    {
      DETACH_FIXED, // ついている接触を離す.
      ATTACH_PRE, // 触れる直前にまで近づける.
      ATTACH_FIXED, // 離れている接触をつける.
      ATTACH_SEARCH, // 離れている接触をつける. 接触ローカル位置を探索する
      SLIDE, // ついている接触をついたまま移動する, 接触ローカル位置は前回ついていた場所
    };
  std::vector<cnoid::SgNodePtr> generateCandidateMakers(std::vector<cnoid::BodyPtr> robots, std::vector<std::shared_ptr<ContactCandidate> > ccs);
}

#endif
