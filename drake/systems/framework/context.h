#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// The ContinuousState is a container for all the State variables that are
/// unique to continuous Systems, i.e. Systems that satisfy
/// ContinuousSystemInterface and have defined dynamics at all times.
template <typename ScalarType>
struct ContinuousState {
  // Generalized velocity v of the state.
  std::unique_ptr<VectorInterface<ScalarType>> generalized_velocity;
  // Generalized position q of the state.
  std::unique_ptr<VectorInterface<ScalarType>> generalized_position;
};

/// The State is a container for all the data defining the configuration of a
/// particular System at a particular moment. Any field in the State may be
/// empty if it is not applicable to the System in question. A System may not
/// maintain state in any place other than the State object.
template <typename ScalarType>
struct State {
  ContinuousState<ScalarType> continuous_state;
};

template <typename ScalarType>
struct Time {
  /// The time, in seconds.  For typical ScalarType implementations based on
  /// doubles, time precision will gradually degrade as time increases.
  /// TODO(sherm1): Consider whether this is sufficiently robust.
  ScalarType time_sec;
};

/// The Context is a container for all of the data necessary to compute the
/// dynamics of any System. Specifically, a Context contains and owns the
/// State, and also contains (but does not own) pointers to the Input, as
/// well as the simulation time.
template <typename ScalarType>
struct Context {
  Time<ScalarType> time;

  // The inputs are not owned.
  std::vector<VectorInterface<ScalarType>*> continuous_inputs;

  // The state is owned.
  State<ScalarType> state;
};

}  // namespace systems
}  // namesapce drake
