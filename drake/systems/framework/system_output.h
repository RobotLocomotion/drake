#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/value.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

template <typename T>
struct OutputPort {
  /// The port data, if the port is vector-valued.
  /// TODO(david-german-tri): Add abstract-valued ports.
  std::unique_ptr<VectorInterface<T>> vector_output;

  /// The rate at which this port produces output, in seconds.
  /// If zero, the port is continuous.
  double sample_time_sec{};
};

/// A container for all the output of a System.
/// @tparam T The mathematical type of the output.
template <typename T>
struct SystemOutput {
  std::vector<OutputPort<T>> ports;
};

}  // namespace systems
}  // namespace drake
