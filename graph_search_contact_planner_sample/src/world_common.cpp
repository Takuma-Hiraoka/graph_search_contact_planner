#include "world_common.h"
#include <cnoid/MeshGenerator>

namespace graph_search_contact_planner_sample{
  void generateStepWorld(cnoid::BodyPtr& obstacle,
			 graph_search_contact_planner::ContactPlanner::GSCPParam& param) {
    cnoid::MeshGenerator meshGenerator;
    obstacle = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(4,2,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(0.6, 0.6, 0.6));
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0,0,-0.05);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1,1,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(0.6, 0.6, 0.6));
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(1.5,0,0.2+0.05);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
      }
      obstacle->setRootLink(rootLink);
    }
    param.field = std::make_shared<moveit_extensions::InterpolatedPropagationDistanceField>(5,//size_x
										      5,//size_y
										      5,//size_z
										      0.02,//resolution // constratintのtoleranceよりも小さい必要がある.
										      -2.5,//origin_x
										      -2.5,//origin_y
										      -2.5,//origin_z
										      0.5, // max_distance
										      true// propagate_negative_distances
										      );
    EigenSTL::vector_Vector3d vertices;
    for(int i=0;i<obstacle->numLinks();i++){
      std::vector<Eigen::Vector3f> vertices_ = ik_constraint2_distance_field::getSurfaceVertices(obstacle->link(i), 0.01);
      for(int j=0;j<vertices_.size();j++){
	vertices.push_back(obstacle->link(i)->T() * vertices_[j].cast<double>());
      }
    }
    param.field->addPointsToField(vertices);

    // contactStaticCandidates
    {
      for(int i=0; i<5; i++) {
	for(int j=0; j<5; j++) {
	  std::shared_ptr<graph_search_contact_planner::ContactCandidate> cc1 = std::make_shared<graph_search_contact_planner::ContactCandidate>("floor1");
	  cc1->localPose.translation() = cnoid::Vector3(0.2+0.2*i, 0.1+0.2*j, 0.0);
	  cc1->localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
	  param.contactStaticCandidates.push_back(cc1);
	  std::shared_ptr<graph_search_contact_planner::ContactCandidate> cc2 = std::make_shared<graph_search_contact_planner::ContactCandidate>("floor");
	  cc2->localPose.translation() = cnoid::Vector3(-0.2*i, 0.1+0.2*j, 0.0);
	  cc2->localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
	  param.contactStaticCandidates.push_back(cc2);
	  std::shared_ptr<graph_search_contact_planner::ContactCandidate> cc3 = std::make_shared<graph_search_contact_planner::ContactCandidate>("floor1");
	  cc3->localPose.translation() = cnoid::Vector3(0.2+0.2*i, -0.1-0.2*j, 0.0);
	  cc3->localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
	  param.contactStaticCandidates.push_back(cc3);
	  std::shared_ptr<graph_search_contact_planner::ContactCandidate> cc4 = std::make_shared<graph_search_contact_planner::ContactCandidate>("floor");
	  cc4->localPose.translation() = cnoid::Vector3(-0.2*i, -0.1-0.2*j, 0.0);
	  cc4->localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
	  param.contactStaticCandidates.push_back(cc4);
	}
      }

      for(int i=0; i<2; i++) {
	for(int j=0; j<2; j++) {
	  std::shared_ptr<graph_search_contact_planner::ContactCandidate> cc1 = std::make_shared<graph_search_contact_planner::ContactCandidate>("floor2");
	  cc1->localPose.translation() = cnoid::Vector3(1.6+0.2*i, 0.1+0.2*j, 0.3);
	  cc1->localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
	  param.contactStaticCandidates.push_back(cc1);
	  std::shared_ptr<graph_search_contact_planner::ContactCandidate> cc2 = std::make_shared<graph_search_contact_planner::ContactCandidate>("floor2");
	  cc2->localPose.translation() = cnoid::Vector3(1.6-0.2-0.2*i, 0.1+0.2*j, 0.3);
	  cc2->localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
	  param.contactStaticCandidates.push_back(cc2);
	  std::shared_ptr<graph_search_contact_planner::ContactCandidate> cc3 = std::make_shared<graph_search_contact_planner::ContactCandidate>("floor2");
	  cc3->localPose.translation() = cnoid::Vector3(1.6+0.2*i, -0.1-0.2*j, 0.3);
	  cc3->localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
	  param.contactStaticCandidates.push_back(cc3);
	  std::shared_ptr<graph_search_contact_planner::ContactCandidate> cc4 = std::make_shared<graph_search_contact_planner::ContactCandidate>("floor2");
	  cc4->localPose.translation() = cnoid::Vector3(1.6-0.2-0.2*i, -0.1-0.2*j, 0.3);
	  cc4->localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
	  param.contactStaticCandidates.push_back(cc4);
	}
      }

    }
  }

  void generateTableWorld(cnoid::BodyPtr& obstacle,
			 graph_search_contact_planner::ContactPlanner::GSCPParam& param) {
    cnoid::MeshGenerator meshGenerator;
    obstacle = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(2.2,2.2,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(0.6, 0.6, 0.6));
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0,0,-0.05);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.5,0.5,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(0.6, 0.6, 0.6));
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0.6,0,1.0-0.05);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
	{
	  cnoid::SgShapePtr shape = new cnoid::SgShape();
	  shape->setMesh(meshGenerator.generateCylinder(0.1, 0.9));
	  cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
	  material->setTransparency(0);
	  material->setDiffuseColor(cnoid::Vector3f(0.6,0.6,0.6));
	  shape->setMaterial(material);
	  cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0.6,0,0.45);
	  posTransform->rotation() = cnoid::rotFromRpy(M_PI /2, 0, 0);
	  posTransform->addChild(shape);
	  rootLink->addShapeNode(posTransform);
	}
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.5,0.5,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(0.6, 0.6, 0.6));
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0,0.6,1.0-0.05);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
	{
	  cnoid::SgShapePtr shape = new cnoid::SgShape();
	  shape->setMesh(meshGenerator.generateCylinder(0.1, 0.9));
	  cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
	  material->setTransparency(0);
	  material->setDiffuseColor(cnoid::Vector3f(0.6,0.6,0.6));
	  shape->setMaterial(material);
	  cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0,0.6,0.45);
	  posTransform->rotation() = cnoid::rotFromRpy(M_PI /2, 0, 0);
	  posTransform->addChild(shape);
	  rootLink->addShapeNode(posTransform);
	}
      }
      obstacle->setRootLink(rootLink);
    }
    param.field = std::make_shared<moveit_extensions::InterpolatedPropagationDistanceField>(5,//size_x
										      5,//size_y
										      5,//size_z
										      0.02,//resolution // constratintのtoleranceよりも小さい必要がある.
										      -2.5,//origin_x
										      -2.5,//origin_y
										      -2.5,//origin_z
										      0.5, // max_distance
										      true// propagate_negative_distances
										      );
    EigenSTL::vector_Vector3d vertices;
    for(int i=0;i<obstacle->numLinks();i++){
      std::vector<Eigen::Vector3f> vertices_ = ik_constraint2_distance_field::getSurfaceVertices(obstacle->link(i), 0.01);
      for(int j=0;j<vertices_.size();j++){
	vertices.push_back(obstacle->link(i)->T() * vertices_[j].cast<double>());
      }
    }
    param.field->addPointsToField(vertices);

    // contactStaticCandidates
    {
      for(int i=0; i<5; i++) {
	for(int j=0; j<5; j++) {
	  std::shared_ptr<graph_search_contact_planner::ContactCandidate> cc1 = std::make_shared<graph_search_contact_planner::ContactCandidate>("floor");
	  cc1->localPose.translation() = cnoid::Vector3(0.2+0.2*i, 0.1+0.2*j, 0.0);
	  cc1->localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
	  param.contactStaticCandidates.push_back(cc1);
	  std::shared_ptr<graph_search_contact_planner::ContactCandidate> cc2 = std::make_shared<graph_search_contact_planner::ContactCandidate>("floor");
	  cc2->localPose.translation() = cnoid::Vector3(-0.2*i, 0.1+0.2*j, 0.0);
	  cc2->localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
	  param.contactStaticCandidates.push_back(cc2);
	  std::shared_ptr<graph_search_contact_planner::ContactCandidate> cc3 = std::make_shared<graph_search_contact_planner::ContactCandidate>("floor");
	  cc3->localPose.translation() = cnoid::Vector3(0.2+0.2*i, -0.1-0.2*j, 0.0);
	  cc3->localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
	  param.contactStaticCandidates.push_back(cc3);
	  std::shared_ptr<graph_search_contact_planner::ContactCandidate> cc4 = std::make_shared<graph_search_contact_planner::ContactCandidate>("floor");
	  cc4->localPose.translation() = cnoid::Vector3(-0.2*i, -0.1-0.2*j, 0.0);
	  cc4->localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
	  param.contactStaticCandidates.push_back(cc4);
	}
      }
      for(int i=0; i<3; i++) {
	for(int j=0; j<3; j++) {
	  std::shared_ptr<graph_search_contact_planner::ContactCandidate> cc1 = std::make_shared<graph_search_contact_planner::ContactCandidate>("table1");
	  cc1->localPose.translation() = cnoid::Vector3(0.6-0.2+0.2*i, -0.2+0.2*j, 1.0);
	  cc1->localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
	  param.contactStaticCandidates.push_back(cc1);
	}
      }
      for(int i=0; i<3; i++) {
	for(int j=0; j<3; j++) {
	  std::shared_ptr<graph_search_contact_planner::ContactCandidate> cc1 = std::make_shared<graph_search_contact_planner::ContactCandidate>("table2");
	  cc1->localPose.translation() = cnoid::Vector3(-0.2+0.2*i, 0.6-0.2+0.2*j, 1.0);
	  cc1->localPose.linear() = cnoid::rotFromRpy(0.0, M_PI, M_PI/2);
	  param.contactStaticCandidates.push_back(cc1);
	}
      }
    }
  }

}
