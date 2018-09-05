#include "drake/multibody/collision/element.h"

#include <iostream>

#include "drake/common/sorted_vectors_have_intersection.h"

using Eigen::Isometry3d;
using drake::SortedVectorsHaveIntersection;
using std::ostream;

namespace drake {
namespace multibody {
namespace collision {

Element::Element()
    : DrakeShapes::Element(Eigen::Isometry3d::Identity()), body_(nullptr) {
}

Element::Element(const DrakeShapes::Geometry& geometry_in,
                 const Isometry3d& T_element_to_local_in)
    : DrakeShapes::Element(geometry_in, T_element_to_local_in), body_(nullptr) {
}

Element::Element(const Isometry3d& T_element_to_link,
                 const ::RigidBody<double>* body)
    : DrakeShapes::Element(T_element_to_link), body_(body) {
}

Element::Element(const DrakeShapes::Geometry& geometry_in,
                 const Isometry3d& T_element_to_link,
                 const ::RigidBody<double>* body)
    : DrakeShapes::Element(geometry_in, T_element_to_link), body_(body) {
}

Element* Element::clone() const { return new Element(*this); }

ElementId Element::getId() const {
  // Id's should be assigned by the model, not here.
  // In addition casting to an int is a bad idea.
  // Issue #2662 tracks the resolution of these problems.
  return reinterpret_cast<ElementId>(this);
}

const ::RigidBody<double>* Element::get_body() const { return body_; }

void Element::set_body(const ::RigidBody<double> *body) { body_ = body; }

bool Element::CanCollideWith(const Element* other) const {
  // Determines if the elements filter each other via filter *groups*. The
  // pair is *excluded* from consideration if this element's group is ignored
  // by the other, or the other's group is ignored by this element. Pairs
  // consisting of two anchored geometries are also explicitly excluded.
  bool excluded =
      (is_anchored() && other->is_anchored()) ||
      (collision_filter_group_ & other->collision_filter_ignores_).any() ||
      (other->collision_filter_group_ & collision_filter_ignores_).any();
  if (!excluded) {
    // Group filtering didn't exclude the pair, try cliques.
    // If collision_cliques_.size() = N and other->collision_cliques_.size() = M
    // The worst case (overlapping elements without intersection) is O(N+M).
    excluded = SortedVectorsHaveIntersection(collision_cliques_,
                                             other->collision_cliques_);
  }
  return !excluded;
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

  // This test precludes duplicate clique id values.
  if (it == collision_cliques_.end() || clique_id < *it)
    collision_cliques_.insert(it, clique_id);
}

void Element::AddCliquesFromElement(const Element& element) {
  for (int clique_id : element.collision_cliques()) {
    this->AddToCollisionClique(clique_id);
  }
}

int Element::get_num_cliques() const {
  return static_cast<int>(collision_cliques_.size());
}

const std::vector<int>& Element::collision_cliques() const {
  return collision_cliques_;
}

void Element::set_collision_filter(const bitmask& group,
                                   const bitmask& ignores) {
  collision_filter_group_ = group;
  collision_filter_ignores_ = ignores;
}

void Element::merge_collision_filter(const bitmask& group,
                                     const bitmask& ignores) {
  collision_filter_group_ |= group;
  collision_filter_ignores_ |= ignores;
}

ostream& operator<<(ostream& out, const Element& ee) {
  out << "drake::multibody::collision::Element:\n"
      << "  - id = " << ee.getId() << "\n"
      << "  - T_element_to_world =\n"
      << ee.T_element_to_world.matrix() << "\n"
      << "  - T_element_to_local =\n"
      << ee.T_element_to_local.matrix() << "\n"
      << "  - geometry = " << *ee.geometry << std::endl;
  return out;
}

}  // namespace collision
}  // namespace multibody
}  // namespace drake
