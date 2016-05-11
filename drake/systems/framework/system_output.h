#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/value.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

template <typename T>
struct OutputPort {
  std::unique_ptr<VectorInterface<T>> output;
};

/// A container for all the output of a System.
/// @tparam T The mathematical type of the output.
template <typename T>
struct SystemOutput {
  std::vector<OutputPort<T>> continuous_ports;
};

}  // namespace systems
}  // namesapce drake
