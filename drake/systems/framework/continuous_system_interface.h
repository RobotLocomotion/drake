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
  /// Returns a ContinuousState of the same size as the continuous_state
  /// allocated in CreateDefaultContext. Solvers will provide this state as the
  /// output argument to EvalTimeDerivatives.
  virtual std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives()
      const = 0;

  /// Produces the derivatives of the continuous state xc with respect to time.
  /// The @p derivatives vector will correspond elementwise with the state
  /// vector Context.state.continuous_state.get_state(). Thus, if the state in
  /// the Context has second-order structure, that same structure applies to
  /// the derivatives.
  ///
  /// Implementations may assume that the given @p derivatives argument has the
  /// same constituent structure as was produced by AllocateTimeDerivatives.
  ///
  /// @param derivatives The output vector. Will be the same length as the
  ///                    state vector Context.state.continuous_state.
  virtual void EvalTimeDerivatives(const Context<T>& context,
                                   ContinuousState<T>* derivatives) const = 0;

  /// Transforms the velocity (v) in the given Context state to the derivative
  /// of the configuration (qdot). The transformation must be linear in velocity
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

  // TODO(sherm): these two power methods should be present only for continuous
  // systems that represent some kind of physical system that can inject or
  // dissipate energy into the simulation. Consider whether to introduce
  // a class like PhysicalSystemInterface that could add these methods only
  // when appropriate; that is awkward to mix with ContinuousSystemInterface
  // so for now I'm breaking the no-code-in-interface rule to provide
  // zero defaults so that these don't have to be implemented in non-physical
  // systems.

  /// Return the rate at which mechanical energy is being converted *from* 
  /// potential energy *to* kinetic energy by this system in the given Context.
  /// This quantity will be positive when potential energy is decreasing. Note
  /// that kinetic energy will also be affected by non-conservative forces so we
  /// can't say which direction it is moving, only whether the conservative
  /// power is increasing or decreasing the kinetic energy. Power is in watts
  /// (J/s). This method is meaningful only for physical systems; others
  /// return 0.
  virtual T EvalConservativePower(const Context<T>& context) const {
    return T(0);
  }

  /// Return the rate at which mechanical energy is being generated (positive)
  /// or dissipated (negative) *other than* by conversion between potential and
  /// kinetic energy (in the given Context). Integrating this quantity yields
  /// work W, and the total energy `E=PE+KE-W` should be conserved by any
  /// physically-correct model, to within integration accuracy of W. Power is in
  /// watts (J/s). (Watts are abbreviated W but not to be confused with work!)
  /// This method is meaningful only for physical systems; others return 0.
  virtual T EvalNonConservativePower(const Context<T>& context) const {
    return T(0);
  }

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
