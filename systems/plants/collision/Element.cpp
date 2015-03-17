#include "Element.h"
#include <iostream>

using namespace Eigen;
using namespace std;

namespace DrakeCollision
{ 
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
