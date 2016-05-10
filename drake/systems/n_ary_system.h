#pragma once

#include <vector>

#include "drake/systems/n_ary_state.h"


namespace drake {

template <class UnitSystem>
class NArySystem {
 public:
  // Required by Drake::System concept.
  template <typename ScalarType>
  using StateVector = NAryState<ScalarType, UnitSystem::template StateVector>;
  // Required by Drake::System concept.
  template <typename ScalarType>
  using InputVector = NAryState<ScalarType, UnitSystem::template InputVector>;
  // Required by Drake::System concept.
  template <typename ScalarType>
  using OutputVector = NAryState<ScalarType, UnitSystem::template OutputVector>;

  NArySystem() {}

  void addSystem(std::shared_ptr<UnitSystem> system) {
    systems_.push_back(system);
  }

  // Required by Drake::System concept.
  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& time,
                                   const StateVector<ScalarType>& state,
                                   const InputVector<ScalarType>& input) const {
    StateVector<ScalarType> xdot(systems_.size());
    for (std::size_t i = 0; i < systems_.size(); ++i) {
      xdot.set(i, systems_[i]->dynamics(time, state.get(i), input.get(i)));
    }
    return xdot;
  }

  // Required by Drake::System concept.
  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& time,
                                  const StateVector<ScalarType>& state,
                                  const InputVector<ScalarType>& input) const {
    OutputVector<ScalarType> y(systems_.size());
    for (std::size_t i = 0; i < systems_.size(); ++i) {
      y.set(i, systems_[i]->output(time, state.get(i), input.get(i)));
    }
    return y;
  }

  // Required by Drake::System concept.
  bool isTimeVarying() const {
    return (systems_.size() > 0) && (systems_[0]->isTimeVarying());
  }

  // Required by Drake::System concept.
  bool isDirectFeedthrough() const {
    return (systems_.size() > 0) && (systems_[0]->isDirectFeedthrough());
  }

  // Required by Drake::System concept.
  std::size_t getNumStates() const {
    return StateVector<double>::rowsFromUnitCount(systems_.size());
  }

  // Required by Drake::System concept.
  std::size_t getNumInputs() const {
    return InputVector<double>::rowsFromUnitCount(systems_.size());
  }

  // Required by Drake::System concept.
  std::size_t getNumOutputs() const {
    return OutputVector<double>::rowsFromUnitCount(systems_.size());
  }

 private:
  std::vector<std::shared_ptr<UnitSystem> > systems_;
};

} // namespace Drake
