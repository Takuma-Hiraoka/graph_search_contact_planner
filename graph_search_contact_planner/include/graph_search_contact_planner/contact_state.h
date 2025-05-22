#ifndef GRAPH_SEARCH_CONTACT_PLANNER_CONTACT_STATE_H
#define GRAPH_SEARCH_CONTACT_PLANNER_CONTACT_STATE_H

#include <cnoid/EigenUtil>

namespace graph_search_contact_planner{
  class ContactCandidate {
  public:
    ContactCandidate() {}
    ContactCandidate(std::string name_, cnoid::Isometry3 localPose_=cnoid::Isometry3::Identity(), bool isStatic_=true) : name(name_), localPose(localPose_), isStatic(isStatic_) {}
    std::string name;
    cnoid::Isometry3 localPose = cnoid::Isometry3::Identity();
    bool isStatic=true;
  };
  class Contact {
    friend bool operator==(const Contact& a, const Contact& b) { // localPose違いも同じ接触とみなす. 別の接触とみなしてほしい場合はnameを別にすること.
      return ((a.c1.name == b.c1.name) && (a.c2.name == b.c2.name)) ||
	     ((a.c1.name == b.c2.name) && (a.c2.name == b.c1.name));
    }
  public:
    Contact() {}
    Contact(ContactCandidate c1_, ContactCandidate c2_) : c1(c1_), c2(c2_) {}
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
