#pragma once

#include <memory>
#include <utility>

#include "drake/systems/analysis/initial_value_problem.h"
#include "drake/systems/analysis/runge_kutta3_integrator-inl.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// A LeafSystem subclass used to describe general ODE systems
/// i.e. d𝐱/dt = f(t, 𝐱, 𝐩) where f : t ⨯ 𝐱 ⊆ ℝ ⁿ⁺¹ →  d𝐱/dt ⊆ ℝ ⁿ, t ∈ ℝ ,
/// 𝐱 ∈ ℝ ⁿ, 𝐩 ∈ ℝ ᵐ.
///
/// @tparam T The ℝ domain scalar type, which must be a valid scalar type.
template <typename T>
class AnySystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AnySystem);

  typedef typename InitialValueProblem<T>::ODEFunction SystemFunction;

  /// Constructs a system that will use the given @p system_function,
  /// parameterized as described by the @p param_model, to compute the
  /// derivatives and advance the @p state_model.
  ///
  /// @param system_function The system function f(t, 𝐱, 𝐩).
  /// @param state_model The state model 𝐱₀, with initial values.
  /// @param param_model The parameter model 𝐩₀, with default values.
  AnySystem(const SystemFunction& system_function,
            const BasicVector<T>& state_model,
            const Parameters<T>& param_model);

 protected:
  /// Calculates the time derivatives for this system.
  /// @param context The current Context under integration.
  /// @param derivatives The derivatives vector.
  void DoCalcTimeDerivatives(
      const Context<T>& context,
      ContinuousState<T>* derivatives) const override;

 private:
  /// General ODE system d𝐱/dt = f(t, 𝐱, 𝐩) function.
  const SystemFunction system_function_;
};


template <typename T>
AnySystem<T>::AnySystem(
    const typename AnySystem<T>::SystemFunction& system_function,
    const BasicVector<T>& state_model, const Parameters<T>& param_model)
    : system_function_(system_function) {
  // Abstract parameters are not supported.
  DRAKE_DEMAND(param_model.num_abstract_parameters() == 0);
  // Models system state after the given state model.
  this->DeclareContinuousState(state_model);
  // Models system parameters after the given param model.
  for (int i = 0; i < param_model.num_numeric_parameters() ; ++i) {
    this->DeclareNumericParameter(param_model.get_numeric_parameter(i));
  }
}

template <typename T>
void AnySystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  // Computes the derivatives at the current time and state, and for the
  // current paramaterization.
  system_function_(
      context.get_time(), context.get_continuous_state_vector(),
      context.get_parameters(), &derivatives->get_mutable_vector());
}

template<typename T>
const T InitialValueProblem<T>::kDefaultAccuracy = static_cast<T>(1e-4);

template<typename T>
const T InitialValueProblem<T>::kInitialStepSize = static_cast<T>(1e-4);

template<typename T>
const T InitialValueProblem<T>::kMaxStepSize = static_cast<T>(1e-1);

template <typename T>
InitialValueProblem<T>::InitialValueProblem(
    const typename InitialValueProblem<T>::ODEFunction& ode_function,
    const T& default_initial_time,
    const BasicVector<T>& default_initial_state,
    const Parameters<T>& default_parameters) {
  // Instantiates the system using the given defaults as models.
  system_ = std::make_unique<AnySystem<T>>(
      ode_function, default_initial_state, default_parameters);

  // Instantiates an explicit RK3 integrator by default.
  integrator_ = std::make_unique<RungeKutta3Integrator<T>>(*system_);

  // Sets step size and accuracy defaults.
  integrator_->request_initial_step_size_target(
      InitialValueProblem<T>::kInitialStepSize);
  integrator_->set_maximum_step_size(
      InitialValueProblem<T>::kMaxStepSize);
  integrator_->set_target_accuracy(
      InitialValueProblem<T>::kDefaultAccuracy);

  // Keeps a default context instance (adding initial time information).
  default_context_ = system_->CreateDefaultContext();
  default_context_->set_time(default_initial_time);
  initial_context_ = std::move(default_context_->Clone());
}

template <typename T>
bool InitialValueProblem<T>::AreContextsValid(
    const Context<T>& initial_context,
    const Context<T>& current_context,
    const T& initial_time, const VectorBase<T>& initial_state,
    const T& time, const Parameters<T>& parameters) const {
  if (current_context.get_time() > time) return false;
  if (initial_context.get_time() != initial_time) return false;
  const VectorBase<T>& initial_context_state =
      initial_context.get_continuous_state_vector();
  for (int i = 0 ; i < initial_context_state.size() ; ++i) {
    if (initial_context_state[i] != initial_state[i])
      return false;
  }
  for (int i = 0 ; i < parameters.num_numeric_parameters() ; ++i) {
    const BasicVector<T>& numeric_parameter =
        parameters.get_numeric_parameter(i);
    const BasicVector<T>& cached_numeric_parameter =
        current_context.get_numeric_parameter(i);
    if (numeric_parameter.get_value() !=
        cached_numeric_parameter.get_value()) {
      return false;
    }
  }
  return true;
}

template <typename T>
const VectorBase<T>& InitialValueProblem<T>::Solve(
    const T& t0, const VectorBase<T>& x0,
    const T& t, const Parameters<T>& p) const {
  DRAKE_THROW_UNLESS(t >= t0);

  if (!context_ || !AreContextsValid(*initial_context_,
                                     *context_, t0, x0, t, p)) {
    // Modifies cached initial context time.
    initial_context_->set_time(t0);

    // Modifies cached initial state.
    VectorBase<T>& initial_state =
        initial_context_->get_mutable_continuous_state_vector();
    initial_state.SetFrom(x0);

    // Modifies cached parameters.
    Parameters<T>& parameters =
        initial_context_->get_mutable_parameters();
    parameters.SetFrom(p);

    // Clones initial context for solver usage.
    context_ = std::move(initial_context_->Clone());

    // Keeps track of current step size and accuracy settings.
    const T initial_step_size = integrator_->get_initial_step_size_target();
    const T max_step_size = integrator_->get_maximum_step_size();
    const T target_accuracy = integrator_->get_target_accuracy();

    // Resets the integrator internal state and context.
    integrator_->Reset();
    integrator_->reset_context(context_.get());

    // Reinitializes the integrator internal state and settings.
    integrator_->request_initial_step_size_target(initial_step_size);
    integrator_->set_maximum_step_size(max_step_size);
    integrator_->set_target_accuracy(target_accuracy);
    integrator_->Initialize();
  }

  // Integrates up to the requested time.
  integrator_->IntegrateWithMultipleSteps(t - context_->get_time());

  // Retrieves the system's continuous state vector.
  return context_->get_continuous_state_vector();
}

template <typename T>
template <typename I>
IntegratorBase<T>* InitialValueProblem<T>::reset_integrator() {
  integrator_ = std::make_unique<I>(*system_);
  return static_cast<I*>(integrator_.get());
}

}  // namespace systems
}  // namespace drake
