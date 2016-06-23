#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_system_interface.h"
#include "drake/systems/framework/state_vector.h"

namespace drake {
namespace systems {

/// An abstract base class template for Systems that have continuous dynamics.
/// Defines some reasonable default implementations.
template <typename T>
class ContinuousSystem : public ContinuousSystemInterface<T> {
 public:
  ~ContinuousSystem() override {}

  /// Applies the identity mapping. Throws std::out_of_range if the
  /// @p generalized_velocity and @p configuration_derivatives are not the
  /// same size. Child classes should override this function if qdot != v.
  void MapVelocityToConfigurationDerivatives(
      const Context<T>& context, const StateVector<T>& generalized_velocity,
      StateVector<T>* configuration_derivatives) const override {
    if (generalized_velocity.size() != configuration_derivatives->size()) {
      throw std::out_of_range(
          "generalized_velocity.size() " +
          std::to_string(generalized_velocity.size()) +
          " != configuration_derivatives.size() " +
          std::to_string(configuration_derivatives->size()) +
          ". Do you need to override the default implementation of " +
          "MapVelocityToConfigurationDerivatives?");
    }

    for (int i = 0; i < generalized_velocity.size(); ++i) {
      configuration_derivatives->SetAtIndex(i,
                                            generalized_velocity.GetAtIndex(i));
    }
  }

 protected:
  ContinuousSystem() {}

 private:
  // ContinuousSystem objects are neither copyable nor moveable.
  ContinuousSystem(const ContinuousSystem<T>& other) = delete;
  ContinuousSystem& operator=(const ContinuousSystem<T>& other) = delete;
  ContinuousSystem(ContinuousSystem<T>&& other) = delete;
  ContinuousSystem& operator=(ContinuousSystem<T>&& other) = delete;
};

}  // namespace systems
}  // namespace drake
