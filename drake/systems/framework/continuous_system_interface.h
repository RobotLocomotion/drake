#pragma once

#include <string>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/state_vector.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_interface.h"

namespace drake {
namespace systems {

/// A template interface for Systems that have continuous dynamics.
template <typename T>
class ContinuousSystemInterface : public SystemInterface<T> {
 public:
  ~ContinuousSystemInterface() override {}

  /// Returns a vector of the same size as the continuous_state allocated in
  /// CreateDefaultContext. Solvers will provide this vector as the output
  /// argument to Dynamics.
  virtual std::unique_ptr<StateVector<T>> AllocateStateDerivatives() const = 0;

  /// Produces the derivatives of the continuous state xc with respect to time.
  /// The @p derivatives vector will correspond elementwise with the state
  /// vector Context.state.continuous_state.get_state(). Thus, if the state in
  /// the Context has second-order structure, that same structure applies to
  /// the derivatives.
  ///
  /// Implementations may assume that the output is of the type constructed in
  /// CreateDefaultStateDerivatives.
  ///
  /// @param derivatives The output vector. Will be the same length as the
  ///                    state vector Context.state.continuous_state.
  virtual void Dynamics(const Context<T>& context,
                        StateVector<T>* derivatives) const = 0;

  /// Transforms the velocity (v) in the given Context state to the derivative
  /// of the configuration (qdot). The transformation must be linear
  /// (qdot = N(q) * v), and it must require no more than O(N) time to compute
  /// in the number of generalized velocity states.
  ///
  /// Implementations may assume that @p configuration_derivatives is of
  /// the same size as the generalized position allocated in
  /// CreateDefaultContext()->continuous_state.get_generalized_position(),
  /// and should populate it with elementwise-corresponding derivatives of
  /// position. Implementations that are not second-order systems may simply
  /// do nothing.
  ///
  /// @param context The complete evaluation context.
  /// @param generalized_velocity The velocity to transform.
  /// @param configuration_derivatives The output vector.  Must not be nullptr.
  virtual void MapVelocityToConfigurationDerivatives(
      const Context<T>& context, const StateVector<T>& generalized_velocity,
      StateVector<T>* configuration_derivatives) const = 0;

  // TODO(david-german-tri): Add MapConfigurationDerivativesToVelocity.

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
