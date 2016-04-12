#ifndef DRAKE_SYSTEMS_FEEDBACK_SYSTEM_H_
#define DRAKE_SYSTEMS_FEEDBACK_SYSTEM_H_

#include <memory>

#include "drake/core/Function.h"
#include "drake/core/Gradient.h"
#include "drake/core/Vector.h"
#include "drake/systems/System.h"

namespace Drake {

/** FeedbackSystem<System1,System2>
 * @brief Builds a new system from the feedback connection of two simpler
 * systems
 * @concept{system_concept}
 *
 * ![Feedback combination of two
 * systems](http://underactuated.csail.mit.edu/figures/feedback_system.svg)
 *
 */

template <class System1, class System2>
class FeedbackSystem {
 public:
  template <typename ScalarType>
  using StateVector1 = typename System1::template StateVector<ScalarType>;
  template <typename ScalarType>
  using StateVector2 = typename System2::template StateVector<ScalarType>;
  template <typename ScalarType>
  using StateVector = typename CombinedVectorUtil<
      System1::template StateVector,
      System2::template StateVector>::template type<ScalarType>;
  template <typename ScalarType>
  using InputVector = typename System1::template InputVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = typename System1::template OutputVector<ScalarType>;
  typedef CombinedVectorUtil<StateVector1, StateVector2> util;

  typedef std::shared_ptr<System1> System1Ptr;
  typedef std::shared_ptr<System2> System2Ptr;

  static_assert(
      std::is_same<typename System1::template OutputVector<double>,
                   typename System2::template InputVector<double>>::value,
      "System 2 input vector must match System 1 output vector");
  static_assert(
      std::is_same<typename System2::template OutputVector<double>,
                   typename System1::template InputVector<double>>::value,
      "System 1 input vector must match System 2 output vector");

  FeedbackSystem(const System1Ptr& _sys1, const System2Ptr& _sys2)
      : sys1(_sys1), sys2(_sys2) {}

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& t,
                                   const StateVector<ScalarType>& x,
                                   const InputVector<ScalarType>& u) const {
    OutputVector<ScalarType> y1;
    InputVector<ScalarType> y2;
    auto x1 = util::first(x);
    auto x2 = util::second(x);
    subsystemOutputs(t, x1, x2, u, &y1, &y2);

    StateVector<ScalarType> xdot = util::combine(
        sys1->dynamics(t, x1, static_cast<InputVector<ScalarType>>(toEigen(y2) +
                                                                   toEigen(u))),
        sys2->dynamics(t, x2, y1));
    return xdot;
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& t,
                                  const StateVector<ScalarType>& x,
                                  const InputVector<ScalarType>& u) const {
    OutputVector<ScalarType> y1;
    auto x1 = util::first(x);
    if (!sys1->isDirectFeedthrough()) {
      y1 = sys1->output(t, x1,
                        u);  // then don't need u+y2 here, u will be ignored
    } else {
      InputVector<ScalarType> y2;
      auto x2 = util::second(x);
      y2 = sys2->output(
          t, x2, y1);  // y1 might be uninitialized junk, but has to be ok
      y1 = sys1->output(t, x1, static_cast<InputVector<ScalarType>>(
                                   toEigen(y2) + toEigen(u)));
    }
    return y1;
  }

  bool isTimeVarying() const {
    return sys1->isTimeVarying() || sys2->isTimeVarying();
  }
  bool isDirectFeedthrough() const { return sys1->isDirectFeedthrough(); }
  size_t getNumStates() const {
    return Drake::getNumStates(*sys1) + Drake::getNumStates(*sys2);
  }
  size_t getNumInputs() const { return Drake::getNumInputs(*sys1); }
  size_t getNumOutputs() const { return Drake::getNumOutputs(*sys1); }

  const System1Ptr& getSys1() const { return sys1; }

  const System2Ptr& getSys2() const { return sys2; }

  friend StateVector<double> getInitialState(
      const FeedbackSystem<System1, System2>& sys) {
    return util::combine(getInitialState(*(sys.sys1)),
                         getInitialState(*(sys.sys2)));
  }

 private:
  template <typename ScalarType>
  void subsystemOutputs(const ScalarType& t, const StateVector1<ScalarType>& x1,
                        const StateVector2<ScalarType>& x2,
                        const InputVector<ScalarType>& u,
                        OutputVector<ScalarType>* y1,
                        InputVector<ScalarType>* y2) const {
    if (!sys1->isDirectFeedthrough()) {
      *y1 = sys1->output(t, x1, u);  // output does not depend on u (so it's ok
                                    // that we're not passing u+y2)
      *y2 = sys2->output(t, x2, *y1);
    } else {
      *y2 = sys2->output(
          t, x2, *y1);  // y1 might be uninitialized junk, but has to be ok
      *y1 = sys1->output(t, x1, static_cast<InputVector<ScalarType>>(
                                   toEigen(*y2) + toEigen(u)));
    }
  }

  System1Ptr sys1;
  System2Ptr sys2;
};

/** feedback(sys1, sys2)
 * @brief Convenience method to create a feedback combination of two systems
 * @ingroup modeling
 */
template <typename System1, typename System2>
std::shared_ptr<FeedbackSystem<System1, System2>> feedback(
    const std::shared_ptr<System1>& sys1,
    const std::shared_ptr<System2>& sys2) {
  return std::make_shared<FeedbackSystem<System1, System2>>(sys1, sys2);
}

}  // namespace Drake

#endif  // DRAKE_SYSTEMS_FEEDBACK_SYSTEM_H_
