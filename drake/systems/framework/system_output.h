#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/value.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

template <typename ScalarType>
struct ContinuousOutputPort {
  std::unique_ptr<VectorInterface<ScalarType>> output;
};

template <typename ScalarType>
struct SystemOutput {
  std::vector<ContinuousOutputPort<ScalarType>> continuous_ports;
};

}  // namespace systems
}  // namesapce drake
