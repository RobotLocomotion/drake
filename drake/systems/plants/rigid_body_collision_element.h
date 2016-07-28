#pragma once

#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/RigidBody.h"
#include "drake/systems/plants/collision/DrakeCollision.h"

class DRAKERBM_EXPORT RigidBodyCollisionElement
    : public DrakeCollision::Element {
 public:
  RigidBodyCollisionElement(const RigidBodyCollisionElement& other);
  // TODO(amcastro-tri): The RigidBody should be const?
  // TODO(amcastro-tri): It should not be possible to construct a
  // RigidBodyCollisionElement without specifying a geometry. Remove
  // this constructor.
  RigidBodyCollisionElement(const Eigen::Isometry3d& T_element_to_link,
                            const RigidBody* const body);
  RigidBodyCollisionElement(const DrakeShapes::Geometry& geometry,
                            const Eigen::Isometry3d& T_element_to_link,
                            const RigidBody* const body);
  virtual ~RigidBodyCollisionElement() {}

  RigidBodyCollisionElement* clone() const override;

  bool CollidesWith(const DrakeCollision::Element* other) const override;

#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};
