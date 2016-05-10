#include "Element.h"

#include <iostream>

using namespace Eigen;
using namespace std;

namespace DrakeCollision {
Element::Element(const Isometry3d& T_element_to_local,
                 const RigidBody* const body)
    : DrakeShapes::Element(T_element_to_local), body_(body) {
  id = (ElementId) this;
}

Element::Element(const DrakeShapes::Geometry& geometry,
                 const Isometry3d& T_element_to_local,
                 const RigidBody* const body)
    : DrakeShapes::Element(geometry, T_element_to_local), body_(body) {
  id = (ElementId) this;
}

Element::Element(const Element& other)
    : DrakeShapes::Element(other), id((ElementId) this), body_(other.body_) {}

Element* Element::clone() const { return new Element(*this); }

ElementId Element::getId() const { return id; }

const RigidBody& Element::getBody() const { return *body_; }

bool Element::CollidesWith(const Element* other) const {
  // Do not collide with self
  if(this == other) return false;

  // If collision_groups_.size() = N and other->collision_groups_.size() = M
  // The worst case (no intersection) is O(N+M).
  return !have_intersection(collision_groups_.begin(), collision_groups_.end(),
                            other->collision_groups_.begin(),
                            other->collision_groups_.end());
}

void Element::add_to_collision_group(int group_id) {
  collision_groups_.push_back(group_id);
  std::sort(collision_groups_.begin(), collision_groups_.end());
}

ostream& operator<<(ostream& out, const Element& ee) {
  out << "DrakeCollision::Element:\n"
      << "  - id = " << ee.id << "\n"
      << "  - T_element_to_world =\n" << ee.T_element_to_world.matrix() << "\n"
      << "  - T_element_to_local =\n" << ee.T_element_to_local.matrix() << "\n"
      << "  - geometry = " << *ee.geometry << std::endl;
  return out;
}

}  // namespace DrakeCollision
