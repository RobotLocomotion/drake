#include "DrakeCollision.h"
#include "Element.h"

using namespace std;

namespace DrakeCollision
{
  Element::Element(const Matrix4d& T_elem_to_link, Shape shape, 
                   const vector<double>& params) 
    : T_elem_to_link(T_elem_to_link),shape(shape)
  {
    setWorldTransform(Matrix4d::Identity());
  }

  const Matrix4d& Element::getWorldTransform() const
  {
    return T_elem_to_world;
  }

  const Matrix4d& Element::getLinkTransform() const
  {
    return T_elem_to_link;
  }

  const Shape& Element::getShape() const
  {
    return shape;
  }

  void Element::updateWorldTransform(const Matrix4d& T_link_to_world)
  {
    setWorldTransform(T_link_to_world*(this->T_elem_to_link));
  }

  void Element::setWorldTransform(const Matrix4d& T_elem_to_world)
  {
    this->T_elem_to_world = T_elem_to_world;
  }
}
