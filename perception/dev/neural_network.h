#pragma once

#include <utility>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace perception {

/// Base class for different kinds of neural network. Will contain code shared
/// across different NN types as these develop.
template <typename T>
class NeuralNetwork : public systems::LeafSystem<T> {
 protected:
  explicit NeuralNetwork(systems::SystemScalarConverter converter)
      : systems::LeafSystem<T>(std::move(converter)) {}

  ~NeuralNetwork() override {}
};

}  // namespace perception
}  // namespace drake
