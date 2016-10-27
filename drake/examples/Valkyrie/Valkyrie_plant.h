#pragma once

#include "drake/common/drake_path.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/RigidBodyTree.h"

using Eigen::VectorXd;

namespace drake {

/** Implements the System concept for the Valkyrie robot.

This class is implemented to conveniently place all methods and functionality
needed to simulate the Atlas robot.
 **/
class ValkyriePlant {
 public:
  template <typename T>
  using InputVector = drake::RigidBodySystem::InputVector<T>;
  template <typename T>
  using StateVector = drake::RigidBodySystem::StateVector<T>;
  template <typename T>
  using OutputVector = drake::RigidBodySystem::OutputVector<T>;

  /** Creates a default instance of the Valkyrie robot as described by the URDF
  file in `"/examples/Valkyrie/val_description/urdf/valkyrie_sim_drake.urdf"`.
  **/
  ValkyriePlant();

  /** Returns an initial state vector describing the configuration of valkyrie
  in a
  standing position with the knees slightly bent and the arms down. **/
  const VectorXd& get_initial_state() const;

  bool isTimeVarying() const;

  size_t getNumInputs() const;

  /** @returns the underlying RigidBodyTree for the valkyrie plant. **/
  const std::shared_ptr<RigidBodyTree>& get_rigid_body_tree() const;

  /** @returns the generalized positions followed by the sensors' outputs as
  the output of the system. **/
  StateVector<double> output(const double& t, const StateVector<double>& x,
                             const InputVector<double>& u) const;

  /** Given an InputVector @p u of generalized forces, this method computes the
  dynamics of the plant at a given StateVector @p x and time @p t. **/
  StateVector<double> dynamics(const double& t, const StateVector<double>& x,
                               const InputVector<double>& u) const;

 private:
  // The underlying rigid body system
  std::unique_ptr<drake::RigidBodySystem> sys_;

  // valkyrie's initial configuration.
  VectorXd x0_;

  // Sets the initial pose for valkyrie.
  void SetInitialConfiguration();
};

}  // namespace drake
