#include "Element.h"

using namespace Eigen;

namespace DrakeShapes
{ 
  Element::Element(const Element& other) 
    : T_element_to_world(other.getWorldTransform()),
      T_element_to_local(other.getLocalTransform()),
      geometry(other.getGeometry().clone()) 
  {
  }

  Element* Element::clone() const
  {
    return new Element(*this);
  }

  const Matrix4d& Element::getWorldTransform() const
  {
    return T_element_to_world;
  }

  const Matrix4d& Element::getLocalTransform() const
  {
    return T_element_to_local;
  }

  const Shape Element::getShape() const
  {
    return geometry->getShape();
  }

  void Element::setGeometry(const Geometry& geometry)
  {
    this->geometry = std::unique_ptr<Geometry>(geometry.clone());
  }

  const Geometry& Element::getGeometry() const
  {
    return *geometry;
  }

  bool Element::hasGeometry() const
  {
    return geometry != nullptr;
  }

  void Element::getTerrainContactPoints(Eigen::Matrix3Xd &local_points)
  {
    if ( !hasGeometry() ) {
      local_points = Eigen::Matrix3Xd();
      return;
    }
    
    Eigen::Matrix3Xd points;
    geometry->getTerrainContactPoints(points);

    Eigen::Matrix4Xd transformed_points = T_element_to_local * points.colwise().homogeneous();
    local_points = transformed_points.colwise().hnormalized();
  }
  
  void Element::updateWorldTransform(const Eigen::Matrix4d& T_local_to_world)
  {
    setWorldTransform(T_local_to_world*(this->T_element_to_local));
  }

  void Element::setWorldTransform(const Matrix4d& T_element_to_world)
  {
    this->T_element_to_world = T_element_to_world;
  }
}
