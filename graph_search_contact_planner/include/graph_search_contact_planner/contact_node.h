#ifndef GRAPH_SEARCH_CONTACT_PLANNER_CONTACT_NODE_H
#define GRAPH_SEARCH_CONTACT_PLANNER_CONTACT_NODE_H

#include <graph_search/node.h>
#include <graph_search_contact_planner/contact_state.h>

namespace graph_search_contact_planner{
  class ContactNode : public graph_search::Node {
  public:
    const ContactState& state() const {return state_;}
    ContactState& state() {return state_;}
  private:
    ContactState state_;
  };
}

#endif
