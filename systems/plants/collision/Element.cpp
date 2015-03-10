#include "Element.h"

using namespace Eigen;

namespace DrakeCollision
{ 
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

  const Geometry* Element::getGeometry() const
  {
    return geometry.get();
  }

  ElementId Element::getId() const
  {
    return id;
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
