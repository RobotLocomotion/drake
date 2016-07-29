#pragma once

#include <string>

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/continuous_system_interface.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// A superclass template for systems that receive input, maintain state, and
/// produce output of a given mathematical type T.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class System : public ContinuousSystemInterface<T> {
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

  /// Return the potential energy currently stored in the configuration provided
  /// in the given @p context. Non-physical Systems will return 0.
  virtual T EvalPotentialEnergy(const ContextBase<T>& context) const {
    return T(0);
  }

  /// Return the kinetic energy currently present in the motion provided in the
  /// given @p context. Non-physical Systems will return 0.
  virtual T EvalKineticEnergy(const ContextBase<T>& context) const {
    return T(0);
  }

  /// By default, returns zero. Physical systems should override.
  ///
  /// Part of ContinuousSystemInterface.
  T EvalConservativePower(const ContextBase<T>& context) const override {
    return T(0);
  }

  /// By default, returns zero. Physical systems should override.
  ///
  /// Part of ContinuousSystemInterface.
  T EvalNonConservativePower(const ContextBase<T>& context) const override {
    return T(0);
  }

  /// By default, allocates no derivatives. Systems with continuous state
  /// variables should override.
  ///
  /// Part of ContinuousSystemInterface.
  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    return nullptr;
  }

  /// By default, does nothing. Systems with continuous state variables should
  /// override.
  ///
  /// Part of ContinuousSystemInterface.
  void EvalTimeDerivatives(const ContextBase<T>& context,
                           ContinuousState<T>* derivatives) const override {
    return;
  }

  /// Applies the identity mapping. Throws std::out_of_range if the
  /// @p generalized_velocity and @p configuration_derivatives are not the
  /// same size. Child classes should override this function if qdot != v.
  ///
  /// Part of ContinuousSystemInterface.
  void MapVelocityToConfigurationDerivatives(
      const ContextBase<T>& context,
      const StateVector<T>& generalized_velocity,
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
