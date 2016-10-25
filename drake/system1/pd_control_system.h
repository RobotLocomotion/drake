#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/system1/System.h"
#include "drake/system1/vector.h"

namespace drake {

/**
 * Wraps an existing system with a PD controller. The new system represents the
 * closed-loop controller + system.
 *
 * Specifically, it implements the following feedback controller:
 *
 * <pre>
 *   x_d --->[ Kp, Kd ]-->(+)----->[ sys ]----------> y_out
 *                     | -                  |
 *                     -------[ Kp, Kd ]<----
 * </pre>
 *
 * @concept{system_concept}
 */
template <class System>
class PDControlSystem {
 public:
  template <typename ScalarType>
  using StateVector = typename System::template StateVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = typename System::template StateVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = typename System::template OutputVector<ScalarType>;
  typedef std::shared_ptr<System> SystemPtr;

  template <typename DerivedA, typename DerivedB>
  PDControlSystem(const SystemPtr& sys, const Eigen::MatrixBase<DerivedA>& Kp,
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
  }

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& t,
                                   const StateVector<ScalarType>& x,
                                   const InputVector<ScalarType>& u) const {
    typename System::template InputVector<ScalarType> system_u =
        Kp_ * (toEigen(u).head(Kp_.cols()) - toEigen(x).head(Kp_.cols())) +
        Kd_ * (toEigen(u).tail(Kd_.cols()) - toEigen(x).tail(Kd_.cols()));
    return sys_->dynamics(t, x, system_u);
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& t,
                                  const StateVector<ScalarType>& x,
                                  const InputVector<ScalarType>& u) const {
    typename System::template InputVector<ScalarType> system_u =
        Kp_ * (toEigen(u).head(Kp_.cols()) - toEigen(x).head(Kp_.cols())) +
        Kd_ * (toEigen(u).tail(Kd_.cols()) - toEigen(x).tail(Kd_.cols()));
    return sys_->output(t, x, system_u);
  }

  bool isTimeVarying() const { return sys_->isTimeVarying(); }
  bool isDirectFeedthrough() const { return sys_->isDirectFeedthrough(); }
  size_t getNumStates() const { return drake::getNumStates(*sys_); }
  size_t getNumInputs() const { return drake::getNumStates(*sys_); }
  size_t getNumOutputs() const { return drake::getNumOutputs(*sys_); }

 public:
  const SystemPtr& getSys() const { return sys_; }
  friend StateVector<double> getInitialState(
      const PDControlSystem<System>& sys) {
    return getInitialState(*sys.sys);
  }

 private:
  SystemPtr sys_;
  Eigen::MatrixXd Kp_, Kd_;
};

}  // end namespace drake
