#pragma once

#include "drake/Path.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {

class DRAKERBSYSTEM_EXPORT AtlasSystem: public Drake::RigidBodySystem {
 public:
  AtlasSystem() {
    addRobotFromFile(
        Drake::getDrakePath() + "/examples/Atlas/urdf/atlas_convex_hull.urdf",
        DrakeJoint::QUATERNION);
      tree_ = getRigidBodyTree().get();
  }

 private:
  RigidBodyTree* tree_;
};


} //namespace drake
