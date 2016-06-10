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

class DRAKERBSYSTEM_EXPORT AtlasSystem: public Drake::RigidBodySystem {
 public:
  AtlasSystem();

  const VectorXd& get_initial_state() const;

  static const int kNumberOfPositions;

 private:
  RigidBodyTree* tree_;
  VectorXd x0_; // Atlas's initial configuration.

  // Sets the initial pose of Atlas.
  // Magic numbers are initial conditions used in runAtlasWalking.m.
  void SetInitialConfiguration();

  void SetUpTerrain();
};

} //namespace drake
