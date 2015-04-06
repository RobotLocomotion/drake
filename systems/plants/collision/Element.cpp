#include "Element.h"
#include <iostream>

using namespace Eigen;
using namespace std;

namespace DrakeCollision
{ 
  Element::Element(const Matrix4d& T_element_to_local)
    : DrakeShapes::Element(T_element_to_local)
  { 
    id = (ElementId) this;
  }

  Element::Element(const DrakeShapes::Geometry& geometry, 
      const Matrix4d& T_element_to_local)
    : DrakeShapes::Element(geometry, T_element_to_local)
  {
    id = (ElementId) this;
  }

  Element::Element(const Element& other) 
    : DrakeShapes::Element(other), id((ElementId) this)  
  {
  }

  Element* Element::clone() const
  {
    return new Element(*this);
  }

  ElementId Element::getId() const
  {
    return id;
  }
}
