#include "collision_element.h"

#include <iostream>

#include "drake/common/sorted_vectors_have_intersection.h"

using Eigen::Isometry3d;
using drake::SortedVectorsHaveIntersection;

namespace DrakeCollision {
CollisionElement::CollisionElement(const Isometry3d& T_element_to_local)
    : DrakeShapes::Element(T_element_to_local) {
  id = (CollisionElementId) this;
}

CollisionElement::CollisionElement(const DrakeShapes::Geometry& geometry,
                                   const Isometry3d& T_element_to_local)
    : DrakeShapes::Element(geometry, T_element_to_local) {
  id = (CollisionElementId) this;
}

CollisionElement::CollisionElement(const CollisionElement& other)
    : DrakeShapes::Element(other),
      id((CollisionElementId) this),
      body_(other.body_),
      collision_cliques_(other.collision_cliques_) {}

CollisionElement* CollisionElement::clone() const {
  return new CollisionElement(*this);
}

CollisionElementId CollisionElement::getId() const { return id; }

const RigidBody* const CollisionElement::getBody() const { return body_; }

void CollisionElement::set_rigid_body(const RigidBody* body) { body_ = body; }

bool CollisionElement::CanCollideWith(const CollisionElement* other) const {
  // If collision_cliques_.size() = N and other->collision_cliques_.size() = M
  // The worst case (overlapping elements without intersection) is O(N+M).
  return !SortedVectorsHaveIntersection(collision_cliques_,
                                        other->collision_cliques_);
}

// Order(N) insertion.
// Member CollisionElement::collision_cliques_ is sorted so that checking if two
// collision elements belong to a same group can be performed in order N.
// See CollisionElement::CanCollideWith
void CollisionElement::AddToCollisionClique(int clique_id) {
  auto it = std::lower_bound(collision_cliques_.begin(),
                             collision_cliques_.end(), clique_id);
  if (it == collision_cliques_.end() || clique_id < *it)
    collision_cliques_.insert(it, clique_id);
}

int CollisionElement::number_of_cliques() const {
  return collision_cliques_.size();
}

const std::vector<int>& CollisionElement::collision_cliques() const {
  return collision_cliques_;
}

std::ostream& operator<<(std::ostream& out, const CollisionElement& ee) {
  out << "DrakeCollision::Element:\n"
      << "  - id = " << ee.id << "\n"
      << "  - T_element_to_world =\n" << ee.T_element_to_world.matrix() << "\n"
      << "  - T_element_to_local =\n" << ee.T_element_to_local.matrix() << "\n"
      << "  - geometry = " << *ee.geometry << std::endl;
  return out;
}

}  // namespace DrakeCollision
