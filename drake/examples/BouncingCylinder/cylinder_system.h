#pragma once

#include "drake/Path.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/RigidBodyTree.h"

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Isometry3d;

namespace drake {

class DRAKERBSYSTEM_EXPORT CylinderSystem: public Drake::RigidBodySystem {
 public:
  CylinderSystem();

  const VectorXd& get_initial_state() const;

 private:
  RigidBodyTree* tree_;
  VectorXd x0_; // Initial configuration.

  // Sets the initial pose/condition.
  void SetZeroConfiguration();

  void SetUpTerrain();
};


} //namespace drake
