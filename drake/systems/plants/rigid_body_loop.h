#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_export.h"
#include "drake/systems/plants/RigidBodyFrame.h"

/**
 * Defines a "loop joint" that models a kinematic loop formed by a chain
 * of rigid bodies and their regular joints. The loop joint is specified by two
 * `RigidBodyFrame` objects that must be attached to two different `RigidBody`
 * objects. The coordinate frames defined by the two `RigidBodyFrame` objects
 * are constrained to have the same origin. The orientations of the two frames
 * are partially constrained based on the axis of rotation (i.e., the two frames
 * are only allowed to rotate relative to each other along the axis of
 * rotation).
 */
class DRAKE_EXPORT RigidBodyLoop {
 public:
  /**
   * Constructs a kinematic loop by fully constraining the origins and partially
   * constraining the orientations of two frames. The two frames are defined
   * relative to two rigid bodies that should be connected via a loop joint.
   *
   * @param[in] frameA A frame defined relative to the loop joint's "parent"
   * rigid body.
   *
   * @param[in] frameB A frame defined relative to the loop joint's "child"
   * rigid body.
   *
   * @param[in] axis The loop joint's axis of rotation expressed in the
   * coordinate frame of @p frameA.
   */
  RigidBodyLoop(std::shared_ptr<RigidBodyFrame> frameA,
                std::shared_ptr<RigidBodyFrame> frameB,
                const Eigen::Vector3d& axis);

  const std::shared_ptr<RigidBodyFrame> frameA_, frameB_;
  const Eigen::Vector3d axis_;

  friend std::ostream& operator<<(std::ostream& os, const RigidBodyLoop& obj);

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};
