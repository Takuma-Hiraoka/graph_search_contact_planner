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
    friend bool operator==(const Contact& a, const Contact& b) { // ContactはlocalPose違いも同じ接触とみなす. 終了条件等で複数のcontactをまとめて扱うため. 別の接触とみなしてほしい場合はnameを別にすること.
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
    friend bool operator==(const ContactState& a, const ContactState& b) { // ContactStateはlocalPoseのrotationの違いのみ無視する. rotationは接触時にZ方向を無視するため. graphとして再訪を防ぐために比較する.
      if (a.contacts.size() != b.contacts.size()) return false; // 数が違う
      for (int i=0;i<a.contacts.size();i++) {
	bool has_a = false;
	for (int j=0;(j<b.contacts.size()) && !has_a; j++) {
	  if (!(a.contacts[i] == b.contacts[j])) continue; // 名前が違う
	  if (!((a.contacts[i].c1.name == b.contacts[j].c1.name) && (a.contacts[i].c1.localPose.translation() == b.contacts[j].c1.localPose.translation()) && (a.contacts[i].c2.name == b.contacts[j].c2.name) && (a.contacts[i].c2.localPose.translation() == b.contacts[j].c2.localPose.translation()) ||
		((a.contacts[i].c1.name == b.contacts[j].c2.name) && (a.contacts[i].c1.localPose.translation() == b.contacts[j].c2.localPose.translation()) && (a.contacts[i].c2.name == b.contacts[j].c1.name) && (a.contacts[i].c2.localPose.translation() == b.contacts[j].c1.localPose.translation())))) continue; // translationが違う
	  has_a = true;
	}
	if (!has_a) return false;
      }
      return true;
    }
  public:
    std::vector<std::vector<double> > transition;
    std::vector<double> frame;
    std::vector<Contact> contacts;
  };
}

#endif
