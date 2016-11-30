#pragma once

#include <memory>

#include "drake/system1/System.h"
#include "drake/system1/vector.h"

namespace drake {

/** CascadeSystem<System1,System2>
 * @brief Builds a new system from the cascade connection of two simpler systems
 * @concept{system_concept}
 *
 * ![Cascade combination of two
 * systems](http://underactuated.csail.mit.edu/figures/cascade_system.svg)
 *
 */
template <class System1, class System2>
class CascadeSystem {
 public:
  template <typename ScalarType>
  using StateVector1 = typename System1::template StateVector<ScalarType>;
  template <typename ScalarType>
  using StateVector2 = typename System2::template StateVector<ScalarType>;
  using util = CombinedVectorUtil<StateVector1, StateVector2>;
  template <typename ScalarType>
  using StateVector = typename util::template type<ScalarType>;
  template <typename ScalarType>
  using InputVector = typename System1::template InputVector<ScalarType>;
  template <typename ScalarType>
  using System1OutputVector =
      typename System1::template OutputVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = typename System2::template OutputVector<ScalarType>;
  using System1Ptr = std::shared_ptr<System1>;
  using System2Ptr = std::shared_ptr<System2>;

  static_assert(
      std::is_same<typename System1::template OutputVector<double>,
                   typename System2::template InputVector<double>>::value,
      "System 2 input vector must match System 1 output vector");

  CascadeSystem(const System1Ptr& _sys1, const System2Ptr& _sys2)
      : sys1(_sys1), sys2(_sys2) {}

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& t,
                                   const StateVector<ScalarType>& x,
                                   const InputVector<ScalarType>& u) const {
    auto x1 = util::first(x);
    auto x2 = util::second(x);
    System1OutputVector<ScalarType> y1 = sys1->output(t, x1, u);
    StateVector<ScalarType> xdot =
        util::combine(sys1->dynamics(t, x1, u), sys2->dynamics(t, x2, y1));
    return xdot;
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& t,
                                  const StateVector<ScalarType>& x,
                                  const InputVector<ScalarType>& u) const {
    auto x1 = util::first(x);
    auto x2 = util::second(x);
    System1OutputVector<ScalarType> y1 = sys1->output(t, x1, u);
    OutputVector<ScalarType> y2 = sys2->output(t, x2, y1);
    return y2;
  }

  bool isTimeVarying() const {
    return sys1->isTimeVarying() || sys2->isTimeVarying();
  }
  bool isDirectFeedthrough() const {
    return sys1->isDirectFeedthrough() && sys2->isDirectFeedthrough();
  }
  size_t getNumStates() const {
    return drake::getNumStates(*sys1) + drake::getNumStates(*sys2);
  }
  size_t getNumInputs() const { return drake::getNumInputs(*sys1); }
  size_t getNumOutputs() const { return drake::getNumOutputs(*sys2); }

 public:
  const System1Ptr& getSys1() const { return sys1; }

  const System2Ptr& getSys2() const { return sys2; }

  friend StateVector<double> getInitialState(
      const CascadeSystem<System1, System2>& sys) {
    return util::combine(getInitialState(*(sys.sys1)),
                         getInitialState(*(sys.sys2)));
  }

 private:
  System1Ptr sys1;
  System2Ptr sys2;
};

/** cascade(sys1, sys2)
 * @brief Convenience method to create a cascade combination of two systems
 * @ingroup modeling
 */
template <typename System1, typename System2>
std::shared_ptr<CascadeSystem<System1, System2>> cascade(
    const std::shared_ptr<System1>& sys1,
    const std::shared_ptr<System2>& sys2) {
  return std::make_shared<CascadeSystem<System1, System2>>(sys1, sys2);
}

}  // namespace drake
