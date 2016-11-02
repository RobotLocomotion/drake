#include "drake/systems/plants/collision/Element.h"

#include <iostream>

#include "drake/common/sorted_vectors_have_intersection.h"

using Eigen::Isometry3d;
using drake::SortedVectorsHaveIntersection;
using std::ostream;

namespace DrakeCollision {

Element::Element()
    : DrakeShapes::Element(Eigen::Isometry3d::Identity()), body_(nullptr) {
  id = (ElementId) this;
}

Element::Element(const DrakeShapes::Geometry& geometry,
                 const Isometry3d& T_element_to_local)
    : DrakeShapes::Element(geometry, T_element_to_local), body_(nullptr) {
  id = (ElementId) this;
}

Element::Element(const Isometry3d& T_element_to_link, const RigidBody* body)
    : DrakeShapes::Element(T_element_to_link), body_(body) {
  id = (ElementId) this;
}

Element::Element(const DrakeShapes::Geometry& geometry,
                 const Isometry3d& T_element_to_link, const RigidBody* body)
    : DrakeShapes::Element(geometry, T_element_to_link), body_(body) {
  id = (ElementId) this;
}

Element::Element(const Element& other)
    : DrakeShapes::Element(other),
      // Id's should be assigned by the model, not here.
      // In addition casting to an int is a bad idea.
      // Issue #2662 tracks the resolution of these problems.
      id(reinterpret_cast<ElementId>(this)),
      is_static_(other.is_static_),
      body_(other.body_),
      collision_cliques_(other.collision_cliques_) {}

Element* Element::clone() const { return new Element(*this); }

ElementId Element::getId() const { return id; }

const RigidBody* Element::get_body() const { return body_; }

void Element::set_body(const RigidBody *body) { body_ = body; }

bool Element::CanCollideWith(const Element *other) const {
  // If collision_cliques_.size() = N and other->collision_cliques_.size() = M
  // The worst case (overlapping elements without intersection) is O(N+M).
  return !SortedVectorsHaveIntersection(collision_cliques_,
                                        other->collision_cliques_);
}

void Element::AddToCollisionClique(int clique_id) {
  // Order(N) insertion.
  // Member Element::collision_cliques_ is a sorted vector so that checking if
  // two collision elements belong to a same group can be performed
  // efficiently in order N.
  // See Element::CanCollideWith() and Element::collision_cliques_ for
  // explanation.
  auto it = std::lower_bound(collision_cliques_.begin(),
                             collision_cliques_.end(), clique_id);
  if (it == collision_cliques_.end() || clique_id < *it)
    collision_cliques_.insert(it, clique_id);
}

int Element::get_num_cliques() const {
  return collision_cliques_.size();
}

const std::vector<int>& Element::collision_cliques() const {
  return collision_cliques_;
}

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
