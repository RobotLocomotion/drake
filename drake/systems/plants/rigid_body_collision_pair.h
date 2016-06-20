#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "RigidBody.h"
#include "drake/drakeRBM_export.h"


struct DRAKERBM_EXPORT RigidBodyCollisionPair {
  RigidBodyCollisionPair(const RigidBody& bodyA, const RigidBody& bodyB,
                         const Eigen::Vector3d normal,
                         double distance,
                         const Eigen::Vector3d ptA,
                         const Eigen::Vector3d ptB)
      : bodyA_(bodyA),
        bodyB_(bodyB),
        ptA_(ptA),
        ptB_(ptB),
        normal_(normal),
        distance_(distance) {}

  const RigidBody& bodyA_;
  const RigidBody& bodyB_;
  Eigen::Vector3d ptA_;
  Eigen::Vector3d ptB_;
  Eigen::Vector3d normal_;
  double distance_;
};

