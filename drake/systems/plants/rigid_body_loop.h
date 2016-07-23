#pragma once

#include <Eigen/Dense>

#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/RigidBodyFrame.h"

class DRAKERBM_EXPORT RigidBodyLoop {
 public:
  //
  // Constructs a RigidBodyLoop between two frames. Is this the correct API?
  // TODO(amcastro-tri): review the correctness of this API
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
