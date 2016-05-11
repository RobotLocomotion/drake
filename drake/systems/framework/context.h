#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// The ContinuousState is a container for all the State variables that are
/// unique to continuous Systems, i.e. Systems that satisfy
/// ContinuousSystemInterface and have defined dynamics at all times.
template <typename T>
struct ContinuousState {
  /// Generalized coordinates representing System configuration, conventionally
  /// denoted `q`. These are second-order state variables.
  std::unique_ptr<VectorInterface<T>> generalized_position;

  /// Generalized speeds representing System velocity. conventionally denoted
  /// `v`. These are first-order state variables that the System can linearly
  /// map to time derivatives `qdot` of `q` above.
  std::unique_ptr<VectorInterface<T>> generalized_velocities;

  /// Additional continuous, first-order state variables not representing
  /// multibody system motion.  Conventionally denoted `z`.
  std::unique_ptr<VectorInterface<T>> other_continuous_state;
};

/// The State is a container for all the data comprising the complete state of
/// a particular System at a particular moment. Any field in the State may be
/// empty if it is not applicable to the System in question. A System may not
/// maintain state in any place other than the State object.
template <typename T>
struct State {
  ContinuousState<T> continuous_state;
};

template <typename T>
struct InputPort {
  VectorInterface<T>* input = nullptr;
};

/// The Input is a container for pointers to all the data that is connected to
/// a particular System from other Systems. The input data itself is not owned
/// by the Input struct.
template <typename T>
struct Input {
  std::vector<InputPort<T>> continuous_ports;
};

template <typename T>
struct Time {
  /// The time, in seconds.  For typical T implementations based on
  /// doubles, time precision will gradually degrade as time increases.
  /// TODO(sherm1): Consider whether this is sufficiently robust.
  T time_sec;
};

/// The Context is a container for all of the data necessary to compute the
/// dynamics of any System. Specifically, a Context contains and owns the
/// State, and also contains (but does not own) pointers to the Input, as
/// well as the simulation time.
///
/// @tparam T The mathematical type of the context (e.g. double).
template <typename T>
struct Context {
  /// The current time.
  Time<T> time;

  /// The external inputs to the System.
  Input<T> input;

  /// The internal state of the System.
  State<T> state;
};

}  // namespace systems
}  // namesapce drake
