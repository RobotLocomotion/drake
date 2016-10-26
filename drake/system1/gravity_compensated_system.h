#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/system1/System.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/system1/vector.h"

namespace drake {

/**
 * Wraps an existing RigidBodySystem with a controller with pure gravity
 * compensation control (the new system represents the closed-loop controller
 * + system). On applying torques to this system, it adds the torque
 * inputs to the gravity torques and supplies them to the system. This
 * controller is inspired by the IIWA torque controller.
 *
 * @concept{system_concept}
 *
 * ![GravityCompensatedSystem](https://github.com/RobotLocomotion/drake/blob/master/drake/doc/images/GravityCompensatedSystem.svg)
 *
 */
template <class System>
class GravityCompensatedSystem {
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

  explicit GravityCompensatedSystem(const RigidBodySystemPtr& sys)
      : sys_(sys) {
    sys_tree_ = sys->getRigidBodyTree();
  }

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& t,
                                   const StateVector<ScalarType>& x,
                                   const InputVector<ScalarType>& u) const {
    InputVector<ScalarType> system_u = GravityCompensatingInput(x, u);
    return sys_->dynamics(t, x, system_u);
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& t,
                                  const StateVector<ScalarType>& x,
                                  const InputVector<ScalarType>& u) const {
    InputVector<ScalarType> system_u = GravityCompensatingInput(x, u);
    return sys_->output(t, x, system_u);
  }

  bool isTimeVarying() const { return sys_->isTimeVarying(); }
  bool isDirectFeedthrough() const { return sys_->isDirectFeedthrough(); }
  size_t getNumStates() const { return drake::getNumStates(*sys_); }
  size_t getNumInputs() const { return drake::getNumInputs(*sys_); }
  size_t getNumOutputs() const { return drake::getNumOutputs(*sys_); }

 public:
  const RigidBodySystemPtr& getSys() const { return sys_; }
  friend StateVector<double> getInitialState(
      const GravityCompensatedSystem<System>& sys) {
    return getInitialState(*sys.sys);
  }

 private:
  RigidBodySystemPtr sys_;
  RigidBodyTreePtr sys_tree_;

  template <typename ScalarType>
  InputVector<ScalarType> GravityCompensatingInput(
      const StateVector<ScalarType>& x,
      const InputVector<ScalarType>& u) const {
    int num_DoF = sys_->get_num_positions();
    KinematicsCache<double> cache = sys_tree_->doKinematics(
        toEigen(x).head(num_DoF), toEigen(x).tail(num_DoF));
    const RigidBodyTree::BodyToWrenchMap<double> no_external_wrenches;
    Eigen::VectorXd vd(num_DoF);
    vd.setZero();

    // The generalized gravity effort is computed by calling inverse dynamics
    // with 0 external forces, 0 velocities and 0 accelerations.
    // TODO(naveenoid): Update to use simpler API once issue #3114 is
    // resolved.
    Eigen::VectorXd G = sys_tree_->inverseDynamics(cache, no_external_wrenches,
                                                   vd, false);
    InputVector<ScalarType> system_u = toEigen(u) + G;
    return system_u;
  }
};

}  // namespace drake
