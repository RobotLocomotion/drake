#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace perception {

template <typename T>
class NeuralNetwork : public systems::LeafSystem<T> {
  /// Base class for different kinds of neural network. Will contain code shared
  /// across different NN types as these develop.
};
}  // namespace perception
}  // namespace drake
