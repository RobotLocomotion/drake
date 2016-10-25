#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/system1/System.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/system1/vector.h"

namespace drake {

/**
 * Wraps an existing RigidBodySystem with a controller with PD position
 * feedback and gravity compensation control (the new system represents the
 * closed-loop controller + system)
 * @concept{system_concept}
 *
 * ![GravityCompensatedPDPositionControlSystem](https://github.com/RobotLocomotion/drake/tree/master/drake/doc/images/GravityCompensatedPositionControl.svg)
 *
 */
template <class System>
class GravityCompensatedPDPositionControlSystem {
 public:
  template <typename ScalarType>
  using StateVector = typename System::template StateVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = typename System::template StateVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = typename System::template OutputVector<ScalarType>;

  // Some convenient typedefs.
  typedef std::shared_ptr<RigidBodySystem> RigidBodySystemPtr;
  typedef std::shared_ptr<RigidBodyTree> RigidBodyTreePtr;

  template <typename DerivedA, typename DerivedB>
  GravityCompensatedPDPositionControlSystem(
      const RigidBodySystemPtr& sys, const Eigen::MatrixBase<DerivedA>& Kp,
      const Eigen::MatrixBase<DerivedB>& Kd)
      : sys_(sys), Kp_(Kp), Kd_(Kd) {
    DRAKE_ASSERT(static_cast<int>(drake::getNumInputs(*sys)) == Kp.rows() &&
                 "Kp must have the same number of rows as the system has"
                 " inputs");
    DRAKE_ASSERT(Kp.rows() == Kd.rows() &&
                 "Kd must have the same number of rows as Kp");
    DRAKE_ASSERT(static_cast<int>(drake::getNumStates(*sys)) ==
                     (Kp.cols() + Kd.cols()) &&
                 "Kp and Kd must match the number of states");
    sys_tree_ = sys->getRigidBodyTree();
  }

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& t,
                                   const StateVector<ScalarType>& x,
                                   const InputVector<ScalarType>& u) const {
    size_t num_DoF = Kp_.cols();
    KinematicsCache<double> cache_ = sys_tree_->doKinematics(
        toEigen(x).head(num_DoF), toEigen(x).tail(num_DoF));
    const RigidBodyTree::BodyToWrenchMap<double> no_external_wrenches;
    Eigen::VectorXd vd(num_DoF);
    vd.setZero();
    auto G = sys_tree_->inverseDynamics(cache_, no_external_wrenches, vd,
                                        false);

    typename System::template InputVector<ScalarType> system_u =
        Kp_ * (toEigen(u).head(Kp_.cols()) - toEigen(x).head(Kp_.cols())) +
        Kd_ * (toEigen(u).tail(Kd_.cols()) - toEigen(x).tail(Kd_.cols())) + G;
    return sys_->dynamics(t, x, system_u);
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& t,
                                  const StateVector<ScalarType>& x,
                                  const InputVector<ScalarType>& u) const {
    size_t num_DoF = Kp_.cols();
    KinematicsCache<double> cache_ = sys_tree_->doKinematics(
        toEigen(x).head(num_DoF), toEigen(x).tail(num_DoF));
    const RigidBodyTree::BodyToWrenchMap<double> no_external_wrenches;
    Eigen::VectorXd vd(num_DoF);
    vd.setZero();

    // The generalized gravity effort is computed by calling inverse dynamics
    // with 0 external forces, 0 velocities and 0 accelerations.
    // TODO(naveenoid): Update to use simpler API once issue #3114 is
    // resolved.
    auto G = sys_tree_->inverseDynamics(cache_, no_external_wrenches, vd,
                                        false);

    typename System::template InputVector<ScalarType> system_u =
        Kp_ * (toEigen(u).head(Kp_.cols()) - toEigen(x).head(Kp_.cols())) +
        Kd_ * (toEigen(u).tail(Kd_.cols()) - toEigen(x).tail(Kd_.cols())) + G;
    return sys_->output(t, x, system_u);
  }

  bool isTimeVarying() const { return sys_->isTimeVarying(); }
  bool isDirectFeedthrough() const { return sys_->isDirectFeedthrough(); }
  size_t getNumStates() const { return drake::getNumStates(*sys_); }
  size_t getNumInputs() const { return drake::getNumStates(*sys_); }
  size_t getNumOutputs() const { return drake::getNumOutputs(*sys_); }

 public:
  const RigidBodySystemPtr& getSys() const { return sys_; }
  friend StateVector<double> getInitialState(
      const GravityCompensatedPDPositionControlSystem<System>& sys) {
    return getInitialState(*sys.sys);
  }

 private:
  RigidBodySystemPtr sys_;
  RigidBodyTreePtr sys_tree_;
  Eigen::MatrixXd Kp_, Kd_;
};

}  // end namespace drake
