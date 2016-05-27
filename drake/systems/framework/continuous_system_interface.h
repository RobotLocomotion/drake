#pragma once

#include <string>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/state_vector_interface.h"
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
  /// argument to GetStateDerivatives.
  virtual std::unique_ptr<StateVectorInterface<T>>
  AllocateStateDerivatives() const = 0;

  /// Produces the derivatives of the continuous state xc with respect to time.
  /// The @p derivatives vector will correspond placewise with the state vector
  /// Context.state.continuous_state.get_state(). Thus, if the state in the
  /// Context has second-order structure, it will also apply to the derivatives.
  ///
  /// Implementations may assume that the output is of the type constructed in
  /// CreateDefaultStateDerivatives.
  ///
  /// @param derivatives The output vector. Will be the same length as the
  ///                    state vector Context.state.continuous_state.
  virtual void Dynamics(const Context<T>& context,
                        StateVectorInterface<T>* derivatives) const = 0;

  /// Transforms the velocity (v) in the given Context state to the derivative
  /// of the configuration (qdot). The transformation must be linear
  /// (qdot = N(q) * v), and it must require no more than O(N) time to compute
  /// in the number of generalized velocity states.
  ///
  /// Implementations may assume that the output is of the type constructed in
  /// CreateDefaultStateDerivatives. Implementations must populate the
  /// derivatives of configuration, and set the derivatives of all other state
  /// variables to zero. Implementations that are not second-order systems
  /// must simply set all state variables to zero.
  ///
  /// @param derivatives The output vector.  Must never be nullptr.
  virtual void MapVelocityToConfigurationDerivatives(
      const Context<T>& context,
      StateVectorInterface<T>* derivatives) const = 0;

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
