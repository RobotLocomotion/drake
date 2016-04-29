#pragma once

#include <vector>

#include "drake/examples/MultiCar/n_ary_state.h"


namespace Drake {

template <class UnitSystem>
class NArySystem {
 public:
  template <typename ScalarType>
  using StateVector = NAryState<ScalarType, UnitSystem::template StateVector>;
  template <typename ScalarType>
  using InputVector = NAryState<ScalarType, UnitSystem::template InputVector>;
  template <typename ScalarType>
  using OutputVector = NAryState<ScalarType, UnitSystem::template OutputVector>;

  explicit NArySystem() {}

  void addSystem(std::shared_ptr<UnitSystem> system) {
    systems_.push_back(system);
  }

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& time,
                                   const StateVector<ScalarType>& state,
                                   const InputVector<ScalarType>& input) const {
    StateVector<ScalarType> xdot;
    for (std::size_t i = 0; i < systems_.size(); ++i) {
      xdot.append(systems_[i]->dynamics(time, state.get(i), input.get(i)));
    }
    return xdot;
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& time,
                                  const StateVector<ScalarType>& state,
                                  const InputVector<ScalarType>& input) const {
    OutputVector<ScalarType> y;
    for (std::size_t i = 0; i < systems_.size(); ++i) {
      y.append(systems_[i]->output(time, state.get(i), input.get(i)));
    }
    return y;
  }

  bool isTimeVarying() const {
    return (systems_.size() > 0) && (systems_[0]->isTimeVarying());
  }

  bool isDirectFeedthrough() const {
    return (systems_.size() > 0) && (systems_[0]->isDirectFeedthrough());
  }

  std::size_t getNumStates() const {
    return systems_.size() * UnitSystem::template StateVector<double>::RowsAtCompileTime;
  }

  std::size_t getNumInputs() const {
    return systems_.size() * UnitSystem::template InputVector<double>::RowsAtCompileTime;
  }

  std::size_t getNumOutputs() const {
    return systems_.size() * UnitSystem::template OutputVector<double>::RowsAtCompileTime;
  }

 private:
  std::vector<std::shared_ptr<UnitSystem> > systems_;
};
} // namespace Drake
