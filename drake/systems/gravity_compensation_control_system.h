#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/systems/System.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/vector.h"
#include "drake/systems/plants/KinematicsCache.h"

namespace drake {

/** GravityCompensatedPDPositionControlSystem<System>
 * @brief Wraps an existing RigidBodySystem with a controller with PD position
 * feedback and gravity compensation control (the new system represents the
 * closed-loop controller + system)
 * @concept{system_concept}
 * x_d --->[ Kp, Kd ]-->(+)----->(+)---->[ sys ]----> yout
 *                       | -      | +            |
                         |    [ G(x) ]<----------|
 *                       |                       |
 *                       ----[ Kp, Kd ]<----------
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
  typedef std::shared_ptr<RigidBodySystem> RigidBodySystemPtr;
  typedef std::shared_ptr<RigidBodyTree> RigidBodyTreePtr;

  template <typename DerivedA, typename DerivedB>
  GravityCompensatedPDPositionControlSystem(const RigidBodySystemPtr& sys,
                                            const Eigen::MatrixBase<DerivedA>& Kp,
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
   KinematicsCache<double> cache_ = sys_tree_->doKinematics(toEigen(x).head(num_DoF), toEigen(x).tail(num_DoF));
   eigen_aligned_unordered_map<RigidBody const*, drake::TwistVector<double>>
        f_ext;
    f_ext.clear();
    Eigen::VectorXd vd(num_DoF);
    vd.setZero();
    auto G = sys_tree_->inverseDynamics(cache_, f_ext, vd, false);

    typename System::template InputVector<ScalarType> system_u =
        Kp_ * (toEigen(u).head(Kp_.cols()) -
            toEigen(x).head(Kp_.cols())) +
            Kd_ * (toEigen(u).tail(Kd_.cols()) -
                toEigen(x).tail(Kd_.cols())) + G;
    return sys_->dynamics(t, x, system_u);
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& t,
                                  const StateVector<ScalarType>& x,
                                  const InputVector<ScalarType>& u) const {

    size_t num_DoF = Kp_.cols();
    KinematicsCache<double> cache_ = sys_tree_->doKinematics(toEigen(x).head(num_DoF), toEigen(x).tail(num_DoF));
    eigen_aligned_unordered_map<RigidBody const*, drake::TwistVector<double>>
        f_ext;
    f_ext.clear();
    Eigen::VectorXd vd(num_DoF);
    vd.setZero();
    auto G = sys_tree_->inverseDynamics(cache_, f_ext, vd, false);

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
