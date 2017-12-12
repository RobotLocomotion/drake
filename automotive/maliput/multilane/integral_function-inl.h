#pragma once

#include <memory>
#include <utility>

#include "drake/automotive/maliput/multilane/integral_function.h"
#include "drake/systems/analysis/runge_kutta3_integrator-inl.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"

namespace drake {
namespace maliput {
namespace multilane {

/// A helper class used to describe general ODE systems i.e. ∂𝘆/∂x = F(x, 𝘆, 𝐩)
/// with F : 𝕊ⁿ⁺¹ → 𝕊ⁿ , x ∈  𝕊 , 𝘆 ∈  𝕊ⁿ, 𝐩 ∈ 𝕊ⁱ.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class AnySystem : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AnySystem);

  /// Transition function type in a general ODE system.
  ///
  /// @param x The independent variable scalar x ∈  𝕊.
  /// @param y The dependent variable vector 𝘆 ∈  𝕊ⁿ.
  /// @param p The parameter vector 𝐩 ∈ 𝕊ⁱ.
  /// @param dydx The derivative vector ∂𝘆/∂x.
  typedef std::function<void(const T x, const systems::VectorBase<T>& y,
                             const systems::BasicVector<T>& p,
                             systems::VectorBase<T>* dydx)> TransitionFunction;

  /// Constructs a system that will use the @p transition_function,
  /// parameterized as described by the @p param_vector_model, to compute the
  /// derivatives and advance the @p state_vector_model.
  ///
  /// @param transition_function The transition function F.
  /// @param state_vector_model The state model vector 𝘆₀ , with initial values.
  /// @param param_vector_model The parameter model vector 𝐩₀, with default
  /// values.
  AnySystem(const TransitionFunction& transition_function,
            const systems::BasicVector<T>& state_vector_model,
            const systems::BasicVector<T>& param_vector_model);

 protected:
  /// Calculates the time derivatives for this system.
  /// @remarks This is due to System semantics. In the context of this
  /// particular subclass, time is actually x.
  /// @param context The current Context under integration.
  /// @param derivatives The derivatives vector.
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

 private:
  /// Transition function F in ∂𝘆/∂x = F(x, 𝘆, 𝐩).
  const TransitionFunction transition_function_;
};

template <typename T>
AnySystem<T>::AnySystem(
    const typename AnySystem<T>::TransitionFunction& transition_function,
    const systems::BasicVector<T>& state_vector,
    const systems::BasicVector<T>& param_vector)
    : transition_function_(transition_function) {
  // Uses the given state vector as a model.
  this->DeclareContinuousState(state_vector);
  // Uses the given param vector as a model.
  this->DeclareNumericParameter(param_vector);
}

template <typename T>
void AnySystem<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Obtains the derivatives vector we need to write into.
  systems::VectorBase<T>& derivatives_vector =
      derivatives->get_mutable_vector();
  // Obtains the state vector we need to read from.
  const systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector();
  // Obtains the param vector we need to read from.
  const systems::BasicVector<T>& numeric_parameter_vector =
      context.get_numeric_parameter(0);
  // Computes the derivatives at the current time and state,
  // param
  transition_function_(context.get_time(), continuous_state_vector,
                       numeric_parameter_vector, &derivatives_vector);
}



template <typename T>
IntegralFunction<T>::IntegralFunction(
    const typename IntegralFunction<T>::IntegrandFunction& integrand_function,
    const T constant_of_integration, const VectorX<T>& parameters) {
  // Instantiates a single element state vector model using the given constant.
  systems::BasicVector<T> state_vector_model(
      VectorX<T>::Constant(1, constant_of_integration));
  // Instantiates a param vector model using the given parameters.
  systems::BasicVector<T> param_vector_model(parameters);
  // Generalizes the given scalar integrand function to build a system.
  typename AnySystem<T>::TransitionFunction scalar_transition_function =
      [integrand_function](const T var, const systems::VectorBase<T>& state,
                           const systems::BasicVector<T>& params,
                           systems::VectorBase<T>* derivatives) {
    // TODO(hidmic): Find a better way to pass the parameters' vector, with
    // less copy overhead.
    derivatives->SetAtIndex(0, integrand_function(var, state.GetAtIndex(0),
                                                  params.CopyToVector()));
  };
  // Instantiates the generic system.
  system_ = std::make_unique<AnySystem<T>>(
      scalar_transition_function, state_vector_model, param_vector_model);

  // Instantiates an explicit RK3 integrator by default.
  integrator_ = std::make_unique<systems::RungeKutta3Integrator<T>>(*system_);
}

template <typename T>
bool IntegralFunction<T>::IsContextValid(const systems::Context<T>& context,
                                         T upper_integration_bound,
                                         const VectorX<T>& parameters) const {
  const systems::BasicVector<T>& numeric_parameters_vector =
      context.get_numeric_parameter(0);
  return (context.get_time() < upper_integration_bound &&
          numeric_parameters_vector.get_value() == parameters);
}

template <typename T>
T IntegralFunction<T>::operator()(T b, const VectorX<T>& p) const {
  DRAKE_DEMAND(b >= 0.0);

  if (!context_ || !IsContextValid(*context_, b, p)) {
    // Step size and accuracy defaults that should
    // in general be reasonable.
    const double kDefaultAccuracy = 1e-4;
    const double kMaxStepSize = 1.;
    const double kInitialStepSize = 1e-4;

    // Allocates system's default context.
    std::unique_ptr<systems::Context<T>> context =
        system_->CreateDefaultContext();

    // Set lower integration bound.
    context->set_time(0.0);

    // Sets first parameter vector to the given value.
    systems::BasicVector<T>& numeric_parameters_vector =
        context->get_mutable_numeric_parameter(0);
    numeric_parameters_vector.SetFromVector(p);

    // Resets the integrator internal state and context.
    integrator_->Reset();
    integrator_->reset_context(context.get());

    // Reinitializes the integrator internal state and configuration.
    integrator_->request_initial_step_size_target(kInitialStepSize);
    integrator_->set_maximum_step_size(kMaxStepSize);
    integrator_->set_target_accuracy(kDefaultAccuracy);
    integrator_->Initialize();

    // Keep context for future reuse.
    context_ = std::move(context);
  }
  // Integrates up to the upper integration bound.
  integrator_->IntegrateWithMultipleSteps(b - context_->get_time());

  // Retrieves the first element of the system's state vector.
  const systems::VectorBase<T>& continuous_state_vector =
      context_->get_continuous_state_vector();
  return continuous_state_vector.GetAtIndex(0);
}

template <typename T>
template <typename I>
systems::IntegratorBase<T>* IntegralFunction<T>::reset_integrator() {
  integrator_ = std::make_unique<I>(*system_);
  return static_cast<I*>(integrator_.get());
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
