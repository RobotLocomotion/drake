#include "Element.h"

#include <iostream>

using namespace Eigen;
using namespace std;

namespace DrakeCollision {
Element::Element(const Isometry3d& T_element_to_local)
    : DrakeShapes::Element(T_element_to_local) {
  id = (ElementId) this;
}

Element::Element(const DrakeShapes::Geometry& geometry,
                 const Isometry3d& T_element_to_local)
    : DrakeShapes::Element(geometry, T_element_to_local) {
  id = (ElementId) this;
}

Element::Element(const Element& other)
    : DrakeShapes::Element(other), id((ElementId) this) {}

Element* Element::clone() const { return new Element(*this); }

ElementId Element::getId() const { return id; }

ostream& operator<<(ostream& out, const Element& ee) {
  out << "DrakeCollision::Element:\n"
      << "  - id = " << ee.id << "\n"
      << "  - T_element_to_world =\n"
      << ee.T_element_to_world.matrix() << "\n"
      << "  - T_element_to_local =\n"
      << ee.T_element_to_local.matrix() << "\n"
      << "  - geometry = " << *ee.geometry << std::endl;
  return out;
}

}  // namespace DrakeCollision
