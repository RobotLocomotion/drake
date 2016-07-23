#pragma once

#include <Eigen/Dense>

#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/RigidBodyFrame.h"

/**
 * Defines a special "loop joint" that models a kinematic loop formed by a chain
 * of rigid bodies and their regular joints. The loop joint is specified by two
 * `RigidBodyFrame` objects that must be attached to two different `RigidBody`
 * objects. The coordinate frames defined by the two `RigidBodyFrame` objects
 * are constrained to be in the same in terms of both position and orientation,
 * which thus forms the loop joint.
 */
class DRAKERBM_EXPORT RigidBodyLoop {
 public:
  /**
   * Constructs a kinematic loop by constraining the origins and orientations
   * of two frames. The two frames are defined relative to two rigid bodies that
   * should be connected via a special loop joint.
   *
   * @param[in] frameA_in A frame defined relative to the loop joint's "parent"
   * rigid body.
   *
   * @param[in] frameB_in A frame defined relative to the loop joint's "child"
   * rigid body.
   *
   * @param[in] axis_in The loop joint's axis of rotation. The loop joint's
   * coordinate frame is equal to the coordinate frame defined by
   * @p frameA_in and @p frameB_in.
   */
  RigidBodyLoop(std::shared_ptr<RigidBodyFrame> frameA_in,
                std::shared_ptr<RigidBodyFrame> frameB_in,
                const Eigen::Vector3d& axis_in);

  const std::shared_ptr<RigidBodyFrame> frameA, frameB;
  const Eigen::Vector3d axis;

  friend std::ostream& operator<<(std::ostream& os, const RigidBodyLoop& obj);

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};
