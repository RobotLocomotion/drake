#pragma once

#include <string>

#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_interface.h"

namespace drake {
namespace systems {

/// A template interface for Systems that have continuous dynamics.
template <typename T>
class ContinuousSystemInterface : public SystemInterface<T> {
 public:
  virtual ~ContinuousSystemInterface() {}

  /// Produces the derivatives of the generalized position q. The derivatives
  /// vector `qdot` will be the same length as
  /// Context.state.continuous_state.generalized_position.
  ///
  /// @param derivatives Must never be nullptr. Implementations of this
  ///                    interface are not obliged to check, since this is a
  ///                    performance-sensitive inner loop function.
  virtual void GetDerivativesOfGeneralizedPosition(
      const Context<T>& context, VectorInterface<T>* derivatives) const = 0;

  /// Produces the derivatives of the generalized velocity v. The derivatives
  /// vector `vdot` (generalized acceleration) will be the same length as
  /// Context.state.continuous_state.generalized_velocity.
  ///
  /// @param derivatives Must never be nullptr. Implementations of this
  ///                    interface are not obliged to check, since this is a
  ///                    performance-sensitive inner loop function.
  virtual void GetDerivativesOfGeneralizedVelocity(
      const Context<T>& context, VectorInterface<T>* derivatives) const = 0;

  /// Produces the derivatives of the other continuous state. The derivatives
  /// vector `zdot` will be the same length as
  /// Context.state.continuous_state.other_continuous_state.
  ///
  /// @param derivatives Must never be nullptr. Implementations of this
  ///                    interface are not obliged to check, since this is a
  ///                    performance-sensitive inner loop function.
  virtual void GetDerivativesOfOtherContinuousState(
      const Context<T>& context, VectorInterface<T>* derivatives) const = 0;

  /// Transforms the given velocity (v) to the derivative of the configuration
  /// (qdot). The transformation must be linear (qdot = N(q) * v), and it must
  /// require no more than O(N) time to compute in the number of generalized
  /// velocity states.
  ///
  /// @param derivatives Must never be nullptr. Implementations of this
  ///                    interface are not obliged to check, since this is a
  ///                    performance-sensitive inner loop function.
  virtual void MapVelocityToConfigurationDerivative(
      const Context<T>& context, VectorInterface<T>* derivatives) const = 0;

 protected:
  ContinuousSystemInterface() {}

 private:
  // ContinuousSystemInterface objects are neither copyable nor moveable.
  ContinuousSystemInterface(const ContinuousSystemInterface<T>& other) = delete;
  ContinuousSystemInterface& operator=(
      const ContinuousSystemInterface<T>& other) = delete;
  ContinuousSystemInterface(ContinuousSystemInterface<T>&& other) = delete;
  ContinuousSystemInterface& operator=(ContinuousSystemInterface<T>&& other) =
      delete;
};

}  // namespace systems
}  // namespace drake
