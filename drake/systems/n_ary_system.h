#pragma once

#include <memory>
#include <vector>

#include "drake/systems/n_ary_state.h"


namespace drake {

/// A System which aggregates multiple instances of a UnitSystem system.
/// The aggregate state, input, and output vectors are composed of the
/// concatenation of the respective vectors of the component systems.
template <class UnitSystem>
class NArySystem {
 public:
  // Required by Drake::System concept.
  template <typename ScalarType>
  using StateVector = NAryState<
    typename UnitSystem::template StateVector<ScalarType> >;
  // Required by Drake::System concept.
  template <typename ScalarType>
  using InputVector = NAryState<
    typename UnitSystem::template InputVector<ScalarType> >;
  // Required by Drake::System concept.
  template <typename ScalarType>
  using OutputVector = NAryState<
    typename UnitSystem::template OutputVector<ScalarType> >;

  NArySystem() {}

  /// Adds @param system to the end of the NArySystem's list of UnitSystems.
  // TODO(maddog) shared_ptr is not strictly necessary here; we are using it
  //              to be compatible with cascade().  At some point, neither
  //              should use a shared_ptr interface.
  void AddSystem(std::shared_ptr<UnitSystem> system) {
    systems_.push_back(system);
  }

  // Required by Drake::System concept.
  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& time,
                                   const StateVector<ScalarType>& state,
                                   const InputVector<ScalarType>& input) const {
    if ((state.count() >= 0) && (state.count() != systems_size())) {
      throw std::invalid_argument("State count differs from systems count.");
    }
    if ((input.count() >= 0) && (input.count() != systems_size())) {
      throw std::invalid_argument("Input count differs from systems count.");
    }
    StateVector<ScalarType> xdot(systems_size());
    for (int i = 0; i < systems_size(); ++i) {
      xdot.set(i, systems_[i]->dynamics(time, state.get(i), input.get(i)));
    }
    return xdot;
  }

  // Required by Drake::System concept.
  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& time,
                                  const StateVector<ScalarType>& state,
                                  const InputVector<ScalarType>& input) const {
    if ((state.count() >= 0) && (state.count() != systems_size())) {
      throw std::invalid_argument("State count differs from systems count.");
    }
    if ((input.count() >= 0) && (input.count() != systems_size())) {
      throw std::invalid_argument("Input count differs from systems count.");
    }
    OutputVector<ScalarType> y(systems_size());
    for (int i = 0; i < systems_size(); ++i) {
      y.set(i, systems_[i]->output(time, state.get(i), input.get(i)));
    }
    return y;
  }

  // Required by Drake::System concept.
  bool isTimeVarying() const {
    for (auto s : systems_) {
      if (s->isTimeVarying()) { return true; }
    }
    return false;
  }

  // Required by Drake::System concept.
  bool isDirectFeedthrough() const {
    for (auto s : systems_) {
      if (s->isDirectFeedthrough()) { return true; }
    }
    return false;
  }

  // Required by Drake::System concept.
  std::size_t getNumStates() const {
    return StateVector<double>::RowsFromUnitCount(systems_size());
  }

  // Required by Drake::System concept.
  std::size_t getNumInputs() const {
    return InputVector<double>::RowsFromUnitCount(systems_size());
  }

  // Required by Drake::System concept.
  std::size_t getNumOutputs() const {
    return OutputVector<double>::RowsFromUnitCount(systems_size());
  }

 private:
  int systems_size() const { return systems_.size(); }

  std::vector<std::shared_ptr<UnitSystem> > systems_;
};

}  // namespace drake
