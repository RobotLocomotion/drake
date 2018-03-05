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

/// A LeafSystem subclass used to describe parameterized ODE systems
/// i.e. d𝐱/dt = f(t, 𝐱; 𝐤) where f : t ⨯ 𝐱 →  ℝⁿ, t ∈ ℝ , 𝐱 ∈ ℝⁿ, 𝐤 ∈ ℝᵐ. The
/// vector variable 𝐱 corresponds to the system state that is evolved through
/// time t by the function f, which is in turn parameterized by a vector 𝐤.
///
/// @tparam T The ℝ domain scalar type, which must be a valid Eigen scalar.
template <typename T>
class AnySystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AnySystem);

  typedef typename InitialValueProblem<T>::ODEFunction SystemFunction;

  /// Constructs a system that will use the given @p system_function,
  /// parameterized as described by the @p param_model, to compute the
  /// derivatives and advance the @p state_model.
  ///
  /// @remarks Here, the 'model' term has been borrowed from LeafSystem
  /// terminology, where these vectors are used both to provide initial
  /// values and to convey information about the dimensionality of the
  /// variables involved.
  ///
  /// @param system_function The system function f(t, 𝐱; 𝐤).
  /// @param state_model The state model vector 𝐱₀, with initial values.
  /// @param param_model The parameter model vector 𝐤₀, with default values.
  AnySystem(const SystemFunction& system_function,
            const VectorX<T>& state_model,
            const VectorX<T>& param_model);

 protected:
  void DoCalcTimeDerivatives(
      const Context<T>& context,
      ContinuousState<T>* derivatives) const override;

 private:
  // General ODE system d𝐱/dt = f(t, 𝐱; 𝐤) function.
  const SystemFunction system_function_;
};


template <typename T>
AnySystem<T>::AnySystem(
    const typename AnySystem<T>::SystemFunction& system_function,
    const VectorX<T>& state_model, const VectorX<T>& param_model)
    : system_function_(system_function) {
  // Models system state after the given state model.
  this->DeclareContinuousState(BasicVector<T>(state_model));
  // Models system parameters after the given parameter model.
  this->DeclareNumericParameter(BasicVector<T>(param_model));
}

template <typename T>
void AnySystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  // Retrieves the state vector. This cast is safe because the
  // ContinuousState<T> of a LeafSystem<T> is flat i.e. it is just
  // a BasicVector<T>, and the implementation deals with LeafSystem<T>
  // instances only by design.
  const BasicVector<T>& state_vector = dynamic_cast<const BasicVector<T>&>(
          context.get_continuous_state_vector());
  // Retrieves the parameter vector.
  const BasicVector<T>& parameter_vector =
      context.get_numeric_parameter(0);

  // Retrieves the derivatives vector. This cast is safe because the
  // ContinuousState<T> of a LeafSystem<T> is flat i.e. it is just
  // a BasicVector<T>, and the implementation deals with LeafSystem<T>
  // instances only by design.
  BasicVector<T>& derivatives_vector =
      dynamic_cast<BasicVector<T>&>(derivatives->get_mutable_vector());
  // Computes the derivatives vector using the given system function
  // for the given time and state and with the given parameterization.
  derivatives_vector.set_value(system_function_(
      context.get_time(), state_vector.get_value(),
      parameter_vector.get_value()));
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
    const T& default_initial_time, const VectorX<T>& default_initial_state,
    const VectorX<T>& default_parameters)
    : default_initial_time_(default_initial_time),
      default_initial_state_(default_initial_state),
      default_parameters_(default_parameters) {
  // Instantiates the system using the given defaults as models.
  system_ = std::make_unique<AnySystem<T>>(
      ode_function, default_initial_state_, default_parameters_);

  // Instantiates an explicit RK3 integrator by default.
  integrator_ = std::make_unique<RungeKutta3Integrator<T>>(*system_);

  // Sets step size and accuracy defaults.
  integrator_->request_initial_step_size_target(
      InitialValueProblem<T>::kInitialStepSize);
  integrator_->set_maximum_step_size(
      InitialValueProblem<T>::kMaxStepSize);
  integrator_->set_target_accuracy(
      InitialValueProblem<T>::kDefaultAccuracy);
}

template <typename T>
VectorX<T> InitialValueProblem<T>::Solve(
    const T& initial_time, const VectorX<T>& initial_state,
    const T& time, const VectorX<T>& parameters) const {
  DRAKE_THROW_UNLESS(time >= initial_time);
  DRAKE_THROW_UNLESS(initial_state.size() == default_initial_state_.size());
  DRAKE_THROW_UNLESS(parameters.size() == default_parameters_.size());

  if (!context_ || initial_time != current_initial_time_
      || initial_state != current_initial_state_
      || parameters != current_parameters_
      || time < context_->get_time()) {
    // Allocates a new integration context.
    std::unique_ptr<Context<T>> newly_allocated_context =
        system_->CreateDefaultContext();

    // Sets context (initial) time.
    newly_allocated_context->set_time(initial_time);

    // Sets context (initial) state. This cast is safe because the
    // ContinuousState<T> of a LeafSystem<T> is flat i.e. it is just
    // a BasicVector<T>, and the implementation deals with LeafSystem<T>
    // instances only by design.
    BasicVector<T>& state_vector = dynamic_cast<BasicVector<T>&>(
        newly_allocated_context->get_mutable_continuous_state_vector());
    state_vector.set_value(initial_state);

    // Sets context parameters.
    BasicVector<T>& parameter_vector =
        newly_allocated_context->get_mutable_numeric_parameter(0);
    parameter_vector.set_value(parameters);

    // Keeps track of current step size and accuracy settings.
    const T initial_step_size = integrator_->get_initial_step_size_target();
    const T max_step_size = integrator_->get_maximum_step_size();
    const T target_accuracy = integrator_->get_target_accuracy();

    // Resets the integrator internal state and context.
    integrator_->Reset();
    integrator_->reset_context(newly_allocated_context.get());

    // Reinitializes the integrator internal state and settings.
    integrator_->request_initial_step_size_target(initial_step_size);
    integrator_->set_maximum_step_size(max_step_size);
    integrator_->set_target_accuracy(target_accuracy);
    integrator_->Initialize();

    // Keeps track of the current initial conditions and parameters
    // for future context invalidation.
    current_initial_time_ = initial_time;
    current_initial_state_ = initial_state;
    current_parameters_ = parameters;

    // Takes ownership of the integration context.
    context_ = std::move(newly_allocated_context);
  }

  // Integrates up to the requested time.
  integrator_->IntegrateWithMultipleSteps(
      time - context_->get_time());

  // Retrieves the system's continuous state vector.
  const VectorBase<T>& state_vector =
      context_->get_continuous_state_vector();
  return state_vector.CopyToVector();
}

template <typename T>
template <typename I>
I* InitialValueProblem<T>::reset_integrator() {
  integrator_ = std::make_unique<I>(*system_);
  return static_cast<I*>(integrator_.get());
}

}  // namespace systems
}  // namespace drake
