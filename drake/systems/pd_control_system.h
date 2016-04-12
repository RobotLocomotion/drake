#ifndef DRAKE_SYSTEMS_PD_CONTROL_SYSTEM_H_
#define DRAKE_SYSTEMS_PD_CONTROL_SYSTEM_H_

#include <memory>

#include "drake/core/Function.h"
#include "drake/core/Gradient.h"
#include "drake/core/Vector.h"
#include "drake/systems/System.h"

namespace Drake {

/** PDControlSystem<System>
 * @brief Wraps an existing system with a PD controller (the new system
 * represents the closed-loop controller + system)
 * @concept{system_concept}
 *
 *   x_d --->[ Kp, Kd ]-->(+)----->[ sys ]----------> yout
 *                     | -                 |
 *                     -------[ Kp, Kd ]<----
 *
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
      : sys(sys), Kp(Kp), Kd(Kd) {
    assert(Drake::getNumInputs(*sys) == Kp.rows() &&
           "Kp must have the same number of rows as the system has inputs");
    assert(Kp.rows() == Kd.rows() &&
           "Kd must have the same number of rows as Kp");
    assert(Drake::getNumStates(*sys) == Kp.cols() + Kd.cols() &&
           "Kp and Kd must match the number of states");
  }

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& t,
                                   const StateVector<ScalarType>& x,
                                   const InputVector<ScalarType>& u) const {
    typename System::template InputVector<ScalarType> system_u =
        Kp * (toEigen(u).head(Kp.cols()) - toEigen(x).head(Kp.cols())) +
        Kd * (toEigen(u).tail(Kd.cols()) - toEigen(x).tail(Kd.cols()));
    return sys->dynamics(t, x, system_u);
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& t,
                                  const StateVector<ScalarType>& x,
                                  const InputVector<ScalarType>& u) const {
    typename System::template InputVector<ScalarType> system_u =
        Kp * (toEigen(u).head(Kp.cols()) - toEigen(x).head(Kp.cols())) +
        Kd * (toEigen(u).tail(Kd.cols()) - toEigen(x).tail(Kd.cols()));
    return sys->output(t, x, system_u);
  }

  bool isTimeVarying() const { return sys->isTimeVarying(); }
  bool isDirectFeedthrough() const { return sys->isDirectFeedthrough(); }
  size_t getNumStates() const { return Drake::getNumStates(*sys); }
  size_t getNumInputs() const { return Drake::getNumStates(*sys); }
  size_t getNumOutputs() const { return Drake::getNumOutputs(*sys); }

 public:
  const SystemPtr& getSys() const { return sys; }
  friend StateVector<double> getInitialState(
      const PDControlSystem<System>& sys) {
    return getInitialState(*sys.sys);
  }

 private:
  SystemPtr sys;
  Eigen::MatrixXd Kp, Kd;
};

}  // end namespace Drake

#endif  // DRAKE_SYSTEMS_PD_CONTROL_SYSTEM_H_
