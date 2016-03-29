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

bool Element::Compare(const Element & ee, std::string * explanation) const {
  bool result = true;

  try {
    valuecheckMatrix(getWorldTransform().matrix(), ee.getWorldTransform().matrix(), 1e-10);
  } catch(std::runtime_error& re) {
    if (explanation != nullptr) {
      std::stringstream ss;
      ss << "[" << getId() << ", " << ee.getId() << "] world transforms do not match";
      *explanation = ss.str();
    }
    result = false;
  }

  if (result) {
    try {
      valuecheckMatrix(getLocalTransform().matrix(), ee.getLocalTransform().matrix(), 1e-10);
    } catch(std::runtime_error& re) {
      if (explanation != nullptr) {
        std::stringstream ss;
        ss << "[" << getId() << ", " << ee.getId() << "] local transforms do not match";
        *explanation = ss.str();
      }
      result = false;
    }
  }

  if (result && getShape() != ee.getShape()) {
    if (explanation != nullptr) {
      std::stringstream ss;
      ss << "[" << getId() << ", " << ee.getId() << "] shapes do not match";
      *explanation = ss.str();
    }
    result = false;
  }

  if (result) {
    if ((hasGeometry() && !ee.hasGeometry()) || (!hasGeometry() && ee.hasGeometry())) {
      if (explanation != nullptr) {
        std::stringstream ss;
        ss << "[" << getId() << ", " << ee.getId() << "] has gometry mismatch";
        *explanation = ss.str();
      }
      result = false;
    }
  }

  if (result && hasGeometry() && ee.hasGeometry()) {
    if (getGeometry() != ee.getGeometry()) {
      if (explanation != nullptr) {
        std::stringstream ss;
        ss << "[" << getId() << ", " << ee.getId() << "] geometry mismatch";
        *explanation = ss.str();
      }
      result = false;
    }
  }

  return result;
}
bool operator==(const Element & e1, const Element & e2) {
  return e1.Compare(e2);
}

bool operator!=(const Element & e1, const Element & e2) {
  return !operator==(e1, e2);
}

}  // namespace DrakeCollision