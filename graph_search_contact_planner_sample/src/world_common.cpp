#include "world_common.h"
#include <cnoid/MeshGenerator>

namespace graph_search_contact_planner_sample{
  void generateStepWorld(cnoid::BodyPtr& obstacle,
			 std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField> field) {
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
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(1.5,0,0.2+0.05);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
      }
      obstacle->setRootLink(rootLink);
    }
    field = std::make_shared<moveit_extensions::InterpolatedPropagationDistanceField>(5,//size_x
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
    field->addPointsToField(vertices);
  }

}
