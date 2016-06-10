#pragma once

#include "drake/Path.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/RigidBodyTree.h"

using Eigen::VectorXd;

namespace drake {

class DRAKERBSYSTEM_EXPORT AtlasSystem: public Drake::RigidBodySystem {
 public:
  AtlasSystem();

  const VectorXd& get_initial_state() const;

  static const int kNumberOfPositions;

 private:
  RigidBodyTree* tree_;
  VectorXd x0_; // Atlas's initial configuration.

  // Sets the initial pose for Atlas.
  // Magic numbers are initial conditions used in runAtlasWalking.m.
  void SetInitialConfiguration();

  void SetUpTerrain();
};

} //namespace drake
