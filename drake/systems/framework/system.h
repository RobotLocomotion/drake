#pragma once

#include <string>

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// A superclass template for systems that receive input, maintain state, and
/// produce output of a given mathematical type T.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class System {
 public:
  virtual ~System() {}

  /// Returns a default context, initialized with the correct
  /// numbers of concrete input ports and state variables for this System.
  /// Since input port pointers are not owned by the context, they should
  /// simply be initialized to nullptr.
  virtual std::unique_ptr<ContextBase<T>> CreateDefaultContext() const = 0;

  /// Returns a default output, initialized with the correct number of
  /// concrete output ports for this System. @p context is provided as
  /// an argument to support some specialized use cases. Most typical
  /// System implementations should ignore it.
  virtual std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const ContextBase<T>& context) const = 0;

  /// Computes the output for the given context, possibly updating values
  /// in the cache.
  virtual void EvalOutput(const ContextBase<T>& context,
                          SystemOutput<T>* output) const = 0;

  /// Returns the potential energy currently stored in the configuration
  /// provided in the given @p context. Non-physical Systems will return 0.
  virtual T EvalPotentialEnergy(const ContextBase<T>& context) const {
    return T(0);
  }

  /// Returns the kinetic energy currently present in the motion provided in
  /// the given @p context. Non-physical Systems will return 0.
  virtual T EvalKineticEnergy(const ContextBase<T>& context) const {
    return T(0);
  }

  /// Returns the rate at which mechanical energy is being converted *from*
  /// potential energy *to* kinetic energy by this system in the given Context.
  /// This quantity will be positive when potential energy is decreasing. Note
  /// that kinetic energy will also be affected by non-conservative forces so we
  /// can't say which direction it is moving, only whether the conservative
  /// power is increasing or decreasing the kinetic energy. Power is in watts
  /// (J/s).
  ///
  /// By default, returns zero. Continuous, physical systems should override.
  virtual T EvalConservativePower(const ContextBase<T>& context) const {
    return T(0);
  }

  /// Returns the rate at which mechanical energy is being generated (positive)
  /// or dissipated (negative) *other than* by conversion between potential and
  /// kinetic energy (in the given Context). Integrating this quantity yields
  /// work W, and the total energy `E=PE+KE-W` should be conserved by any
  /// physically-correct model, to within integration accuracy of W. Power is in
  /// watts (J/s). (Watts are abbreviated W but not to be confused with work!)
  /// This method is meaningful only for physical systems; others return 0.
  ///
  /// By default, returns zero. Continuous, physical systems should override.
  virtual T EvalNonConservativePower(const ContextBase<T>& context) const {
    return T(0);
  }
  /// Returns a ContinuousState of the same size as the continuous_state
  /// allocated in CreateDefaultContext. Solvers will provide this state as the
  /// output argument to EvalTimeDerivatives.
  ///
  /// By default, allocates no derivatives. Systems with continuous state
  /// variables should override.
  virtual std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const {
    return nullptr;
  }

  /// Produces the derivatives of the continuous state xc with respect to time.
  /// The @p derivatives vector will correspond elementwise with the state
  /// vector Context.state.continuous_state.get_state(). Thus, if the state in
  /// the Context has second-order structure, that same structure applies to
  /// the derivatives.
  ///
  /// Implementations may assume that the given @p derivatives argument has the
  /// same constituent structure as was produced by AllocateTimeDerivatives.
  ///
  /// @param context The context in which to evaluate the derivatives.
  ///
  /// @param derivatives The output vector. Will be the same length as the
  ///                    state vector Context.state.continuous_state.
  virtual void EvalTimeDerivatives(const ContextBase<T>& context,
                                   ContinuousState<T>* derivatives) const {
    return;
  }

  /// Transforms the velocity (v) in the given Context state to the derivative
  /// of the configuration (qdot). The transformation must be linear in velocity
  /// (qdot = N(q) * v), and it must require no more than O(N) time to compute
  /// in the number of generalized velocity states.
  ///
  /// The default implementation uses the identity mapping. It throws
  /// std::out_of_range if the @p generalized_velocity and
  /// @p configuration_derivatives are not the same size. Child classes should
  /// override this function if qdot != v.
  ///
  /// Implementations may assume that @p configuration_derivatives is of
  /// the same size as the generalized position allocated in
  /// CreateDefaultContext()->continuous_state.get_generalized_position(),
  /// and should populate it with elementwise-corresponding derivatives of
  /// position. Implementations that are not second-order systems may simply
  /// do nothing.
  virtual void MapVelocityToConfigurationDerivatives(
      const ContextBase<T>& context,
      const StateVector<T>& generalized_velocity,
      StateVector<T>* configuration_derivatives) const {
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

  // TODO(david-german-tri): Add MapConfigurationDerivativesToVelocity
  // and MapAccelerationToConfigurationSecondDerivatives.

  virtual void set_name(const std::string& name) { name_ = name; }
  virtual std::string get_name() const { return name_; }

 protected:
  System() {}

 private:
  // SystemInterface objects are neither copyable nor moveable.
  System(const System<T>& other) = delete;
  System& operator=(const System<T>& other) = delete;
  System(System<T>&& other) = delete;
  System& operator=(System<T>&& other) = delete;

  std::string name_;
};

}  // namespace systems
}  // namespace drake
