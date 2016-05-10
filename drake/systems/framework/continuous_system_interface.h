#pragma once

#include <string>

#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_interface.h"

namespace drake {
namespace systems {

/// A template interface for Systems that have continuous dynamics.
template <typename ScalarType>
class ContinuousSystemInterface : public SystemInterface<ScalarType> {
 public:
  ContinuousSystemInterface() {}
  virtual ~ContinuousSystemInterface() {}

  /// Produces the derivatives of the generalized velocity v. The derivatives
  /// vector will be the same length as
  /// Context.state.continuous_state.generalized_velocity.
  virtual void GetContinuousDerivativesOfGeneralizedVelocity(
      const Context<ScalarType>& context, Cache<ScalarType>* cache,
      VectorInterface<ScalarType>* derivatives) const = 0;

  /// Produces the derivatives of the generalized velocity v. The derivatives
  /// vector will be the same length as
  /// Context.state.continuous_state.generalized_position.
  virtual void GetContinuousDerivativesOfGeneralizedPosition(
      const Context<ScalarType>& context, Cache<ScalarType>* cache,
      VectorInterface<ScalarType>* derivatives) const = 0;

  /// Transforms the given velocity (v) to the derivative of the
  /// configuration (qdot).
  /// Unlike most things in Drake, this is intentionally black-box:
  /// solvers need to invoke the transform, but they don’t need to analyze
  /// its inner workings.
  virtual void MapVelocityToConfigurationDerivative(
      const VectorInterface<ScalarType>& generalized_velocity,
      Cache<ScalarType>* cache,
      VectorInterface<ScalarType>* derivatives) const = 0;

 private:
  // ContinuousSystemInterface objects are neither copyable nor moveable.
  ContinuousSystemInterface(
      const ContinuousSystemInterface<ScalarType>& other) = delete;
  ContinuousSystemInterface& operator=(
      const ContinuousSystemInterface<ScalarType>& other) = delete;
  ContinuousSystemInterface(ContinuousSystemInterface<ScalarType>&& other) =
      delete;
  ContinuousSystemInterface& operator=(
      ContinuousSystemInterface<ScalarType>&& other) = delete;
};

}  // namespace systems
}  // namespace drake
