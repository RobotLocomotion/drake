#include "rigid_body_collision_element.h"

RigidBodyCollisionElement::RigidBodyCollisionElement(
    const RigidBodyCollisionElement& other)
    : DrakeCollision::Element(other) {}

RigidBodyCollisionElement::RigidBodyCollisionElement(
    const Eigen::Isometry3d& T_element_to_link, const RigidBody* const body)
    : DrakeCollision::Element(T_element_to_link) {
  set_body(body);
}

RigidBodyCollisionElement::RigidBodyCollisionElement(
    const DrakeShapes::Geometry& geometry,
    const Eigen::Isometry3d& T_element_to_link, const RigidBody* const body)
    : DrakeCollision::Element(geometry, T_element_to_link) {
  set_body(body);
  // This is a temporary hack to avoid having the user to set collision
  // elements to static when added to the world.
  // Collision elements should be set to static in a later Initialize() stage as
  // described in issue #2661.
  // TODO(amcastro-tri): remove this hack.
  if (body->get_name() == "world") set_static();
}

RigidBodyCollisionElement* RigidBodyCollisionElement::clone() const {
  return new RigidBodyCollisionElement(*this);
}

bool RigidBodyCollisionElement::CollidesWith(
    const DrakeCollision::Element* other) const {
  auto other_rb = dynamic_cast<const RigidBodyCollisionElement*>(other);
  bool collides = true;
  if (other_rb != nullptr) {
    collides = get_body()->CollidesWith(*other_rb->get_body());
  }
  return collides;
}
