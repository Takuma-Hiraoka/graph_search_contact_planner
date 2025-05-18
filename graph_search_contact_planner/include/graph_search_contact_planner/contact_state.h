#ifndef GRAPH_SEARCH_CONTACT_PLANNER_CONTACT_STATE_H
#define GRAPH_SEARCH_CONTACT_PLANNER_CONTACT_STATE_H

#include <cnoid/EigenUtil>

namespace graph_search_contact_planner{
  class ContactCandidate {
  public:
    std::string name;
    bool isStatic=true;
    cnoid::Isometry3 localPose;
  };
  class Contact {
  public:
    ContactCandidate c1;
    ContactCandidate c2;
  };
  class ContactState {
  public:
    std::vector<double> frame;
    std::vector<Contact> contacts;
  };
}

#endif
