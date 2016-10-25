#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/system1/System.h"
#include "drake/system1/vector.h"

namespace drake {

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
      : sys1(_sys1), sys2(_sys2) {
    if (sys1->isDirectFeedthrough() && sys2->isDirectFeedthrough()) {
      throw std::runtime_error("Algebraic loop in FeedbackSystem");
    }
  }

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& t,
                                   const StateVector<ScalarType>& x,
                                   const InputVector<ScalarType>& u) const {
    const auto& x1 = util::first(x);
    const auto& x2 = util::second(x);
    OutputVector<ScalarType> y1;
    InputVector<ScalarType> y2;
    InputVector<ScalarType> u1;
    subsystemOutputs(t, x1, x2, u, true, &y1, &y2, &u1);
    return util::combine(
        sys1->dynamics(t, x1, u1),
        sys2->dynamics(t, x2, y1));
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& t,
                                  const StateVector<ScalarType>& x,
                                  const InputVector<ScalarType>& u) const {
    const auto& x1 = util::first(x);
    const auto& x2 = util::second(x);
    OutputVector<ScalarType> y1;
    InputVector<ScalarType> y2;
    InputVector<ScalarType> u1;
    subsystemOutputs(t, x1, x2, u, false, &y1, &y2, &u1);
    return y1;
  }

  bool isTimeVarying() const {
    return sys1->isTimeVarying() || sys2->isTimeVarying();
  }
  bool isDirectFeedthrough() const { return sys1->isDirectFeedthrough(); }
  size_t getNumStates() const {
    return drake::getNumStates(*sys1) + drake::getNumStates(*sys2);
  }
  size_t getNumInputs() const { return drake::getNumInputs(*sys1); }
  size_t getNumOutputs() const { return drake::getNumOutputs(*sys1); }

  const System1Ptr& getSys1() const { return sys1; }

  const System2Ptr& getSys2() const { return sys2; }

  friend StateVector<double> getInitialState(
      const FeedbackSystem<System1, System2>& sys) {
    return util::combine(getInitialState(*(sys.sys1)),
                         getInitialState(*(sys.sys2)));
  }

 private:
  // x1 is sys1 state.
  // x2 is sys2 state.
  // u  is the feedback_system's input.
  ///
  // y1 is sys1's output.
  // y2 is sys2's output.
  // u1 is the input to sys1 (u + y2).
  //
  // If want_y2_u1 is true, y2 and u1 are always computed.
  // If want_y2_u1 is false, y2 and u1 may or may not be computed.
  template <typename ScalarType>
  void subsystemOutputs(const ScalarType& t,
                        const StateVector1<ScalarType>& x1,
                        const StateVector2<ScalarType>& x2,
                        const InputVector<ScalarType>& u,
                        bool want_y2_u1,
                        OutputVector<ScalarType>* y1,
                        InputVector<ScalarType>* y2,
                        InputVector<ScalarType>* u1) const {
    if (!sys1->isDirectFeedthrough()) {
      // sys1->output doesn't use u1, so it's okay that it isn't filled in yet.
      *y1 = sys1->output(t, x1, *u1);
      if (want_y2_u1) {
        *y2 = sys2->output(t, x2, *y1);
        *u1 = static_cast<InputVector<ScalarType>>(toEigen(*y2) + toEigen(u));
      }
    } else {
      DRAKE_ASSERT(!sys2->isDirectFeedthrough());  // Per our constructor.
      // sys2->output doesn't use y1, so it's okay that it isn't filled in yet.
      *y2 = sys2->output(t, x2, *y1);
      *u1 = static_cast<InputVector<ScalarType>>(toEigen(*y2) + toEigen(u));
      *y1 = sys1->output(t, x1, *u1);
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

}  // namespace drake
