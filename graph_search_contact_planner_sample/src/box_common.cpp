#include "box_common.h"
#include <cnoid/MeshGenerator>

namespace graph_search_contact_planner_sample{
  void generateBOX(graph_search_contact_planner::ContactPlanner::GSCPParam& param
		   ) {
    cnoid::MeshGenerator meshGenerator;
    cnoid::BodyPtr box = new cnoid::Body();
    param.bodies.push_back(box);
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      rootLink->setJointType(cnoid::Link::JointType::FreeJoint);
      rootLink->setName("box");
      box->setRootLink(rootLink);
      box->rootLink()->p() = cnoid::Vector3(1.0,0,1.1);
      param.variables.push_back(rootLink);
      {
	cnoid::SgShapePtr shape = new cnoid::SgShape();
	shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.2,0.4,0.2)));
	cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
	material->setTransparency(0);
	material->setDiffuseColor(cnoid::Vector3f(1.0, 0.6, 0.6));
	shape->setMaterial(material);
	cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
	posTransform->addChild(shape);
	rootLink->addShapeNode(posTransform);
      }
    }

    // currentContactState
    {
      graph_search_contact_planner::ContactCandidate c1;
      c1.name = "box";
      c1.isStatic = false;
      c1.localPose.translation() = cnoid::Vector3(0, 0, -0.1);
      graph_search_contact_planner::ContactCandidate c2;
      c2.name = "table1";
      c2.isStatic = true;
      c2.localPose.translation() = cnoid::Vector3(1.0, 0.0, 1.0);
      c2.localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
      param.currentContactState->contacts.push_back(graph_search_contact_planner::Contact(c1,c2));
    }

    // contactDynamicCandidates
    {
      // base
      {
	std::shared_ptr<graph_search_contact_planner::ContactCandidate> base = std::make_shared<graph_search_contact_planner::ContactCandidate>();
	base->name = "box";
	base->isStatic = false;
	base->localPose.translation() = cnoid::Vector3(0,0,-0.1);
	param.contactDynamicCandidates.push_back(base);
      }
      // rhandle
      {
	std::shared_ptr<graph_search_contact_planner::ContactCandidate> rhandle = std::make_shared<graph_search_contact_planner::ContactCandidate>();
	rhandle->name = "box";
	rhandle->isStatic = false;
	rhandle->localPose.translation() = cnoid::Vector3(0,-0.2,0.0);
        rhandle->localPose.linear() = cnoid::rotFromRpy(-M_PI/2, 0.0, 0.0);
	param.contactDynamicCandidates.push_back(rhandle);
      }
      // lhandle
      {
	std::shared_ptr<graph_search_contact_planner::ContactCandidate> lhandle = std::make_shared<graph_search_contact_planner::ContactCandidate>();
	lhandle->name = "box";
	lhandle->isStatic = false;
	lhandle->localPose.translation() = cnoid::Vector3(0,0.2,0.0);
        lhandle->localPose.linear() = cnoid::rotFromRpy(M_PI/2, 0.0, 0.0);
	param.contactDynamicCandidates.push_back(lhandle);
      }
    }
  }
}
