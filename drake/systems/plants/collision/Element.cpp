#include "Element.h"

#include <iostream>

#include "drake/util/testUtil.h"

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
      << "  - T_element_to_world =\n" << ee.T_element_to_world.matrix() << "\n"
      << "  - T_element_to_local =\n" << ee.T_element_to_local.matrix() << "\n"
      << "  - geometry = " << *ee.geometry
      << std::endl;
  return out;
}

#define PRINT_STMT(x) std::cout << "DrakeCollision::Element::EQUALS: " << x << std::endl;

bool operator==(const Element & e1, const Element & e2) {
  bool result = true;

  try {
    valuecheckMatrix(e1.getWorldTransform().matrix(), e2.getWorldTransform().matrix(), 1e-10);
  } catch(std::runtime_error& re) {
    PRINT_STMT("[" << e1.getId() << ", " << e2.getId() << "] world transforms do not match")
    result = false;
  }

  if (result) {
    try {
      valuecheckMatrix(e1.getLocalTransform().matrix(), e2.getLocalTransform().matrix(), 1e-10);
    } catch(std::runtime_error& re) {
      PRINT_STMT("[" << e1.getId() << ", " << e2.getId() << "] local transforms do not match")
      result = false;
    }
  }

  if (result && e1.getShape() != e2.getShape()) {
    PRINT_STMT("[" << e1.getId() << ", " << e2.getId() << "] shapes do not match")
    result = false;
  }

  if (result) {
    if (e1.hasGeometry() && !e2.hasGeometry() || !e1.hasGeometry() && e2.hasGeometry()) {
      PRINT_STMT("[" << e1.getId() << ", " << e2.getId() << "] has gometry mismatch")
      result = false;
    }
  }

  if (result && e1.hasGeometry() && e2.hasGeometry()) {
    if (e1.getGeometry() != e2.getGeometry()) {
      PRINT_STMT("[" << e1.getId() << ", " << e2.getId() << "] geometry mismatch")
      result = false;
    }
  }

  return result;
}

#undef PRINT_STMT

bool operator!=(const Element & e1, const Element & e2) {
  return !operator==(e1, e2);
}

}  // namespace DrakeCollision