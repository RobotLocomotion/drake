#pragma once

#include "drake/Path.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/RigidBodyTree.h"

using Eigen::VectorXd;

namespace drake {

/** Implements the RigidBodySystem concept for the Atlas robot.

This class is implemented to conveniently place all methods and functionality
needed to simulate and design controllers for Atlas.

 TODO(amcastro-tri): Future extensions will include the additions of actuators,
 sensors, controllers and, methods implementing specific capabilities for Atlas.
 **/
class AtlasSystem : public Drake::RigidBodySystem {
 public:
  /** Creates a default instance of the Atlas robot as described by the URDF
  file in `drake/examples/Atlas/urdf/atlas_convex_hull.urdf`. **/
  AtlasSystem();

  /** Returns an initial state vector describing the configuration of Atlas in a
  standing position with the knees slightly bent and the arms down. **/
  const VectorXd& get_initial_state() const;

 private:
  VectorXd x0_;  // Atlas's initial configuration.

  // Sets the initial pose for Atlas.
  void SetInitialConfiguration();

  void SetUpTerrain();
};

}  // namespace drake
