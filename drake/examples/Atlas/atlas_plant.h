#pragma once

#include "drake/common/drake_path.h"
#include "drake/multibody/rigid_body_system1/RigidBodySystem.h"
#include "drake/multibody/rigid_body_tree.h"

using Eigen::VectorXd;

namespace drake {

/** Implements the System concept for the Atlas robot.

This class is implemented to conveniently place all methods and functionality
needed to simulate the Atlas robot.
 **/
class AtlasPlant {
 public:
  template<typename T>
  using InputVector = drake::RigidBodySystem::InputVector<T>;
  template<typename T>
  using StateVector = drake::RigidBodySystem::StateVector<T>;
  template<typename T>
  using OutputVector = drake::RigidBodySystem::OutputVector<T>;

  /** Creates a default instance of the Atlas robot as described by the URDF
  file in `drake/examples/Atlas/urdf/atlas_convex_hull.urdf`. **/
  AtlasPlant();

  /** Returns an initial state vector describing the configuration of Atlas in a
  standing position with the knees slightly bent and the arms down. **/
  const VectorXd& get_initial_state() const;

  bool isTimeVarying() const;

  size_t getNumInputs() const;

  /** @returns the underlying RigidBodyTree for the Atlas plant. **/
  const std::shared_ptr<RigidBodyTree<double>>& get_rigid_body_tree() const;

  /** @returns the generalized positions followed by the sensors' outputs as
  the output of the system. **/
  StateVector<double> output(const double& t,
                             const StateVector<double>& x,
                             const InputVector<double>& u) const;

  /** Given an InputVector @p u of generalized forces, this method computes the
  dynamics of the plant at a given StateVector @p x and time @p t. **/
  StateVector<double> dynamics(const double& t,
                               const StateVector<double>& x,
                               const InputVector<double>& u) const;

 private:
  // The underlying rigid body system
  std::unique_ptr<drake::RigidBodySystem> sys_;

  // Atlas's initial configuration.
  VectorXd x0_;

  // Sets the initial pose for Atlas.
  void SetInitialConfiguration();

  void SetUpTerrain();
};

}  // namespace drake
