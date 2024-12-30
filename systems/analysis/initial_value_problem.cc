#include "drake/systems/analysis/initial_value_problem.h"

#include <stdexcept>

#include "drake/systems/analysis/hermitian_dense_output.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {

// A LeafSystem subclass used to describe parameterized ODE systems
// i.e. d𝐱/dt = f(t, 𝐱; 𝐤) where f : t ⨯ 𝐱 →  ℝⁿ, t ∈ ℝ , 𝐱 ∈ ℝⁿ, 𝐤 ∈ ℝᵐ.
// The vector variable 𝐱 corresponds to the system state that is evolved
// through time t by the function f, which is in turn parameterized by a
// vector 𝐤.
//
// @tparam T The ℝ domain scalar type, which must be a valid Eigen scalar.
template <typename T>
class OdeSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OdeSystem);

  typedef typename InitialValueProblem<T>::OdeFunction SystemFunction;

  // Constructs a system that will use the given @p system_function,
  // parameterized as described by the @p param_model, to compute the
  // derivatives and advance the @p state_model.
  //
  // @remarks Here, the 'model' term has been borrowed from LeafSystem
  // terminology, where these vectors are used both to provide initial
  // values and to convey information about the dimensionality of the
  // variables involved.
  //
  // @param system_function The system function f(t, 𝐱; 𝐤).
  // @param state_model The state model vector 𝐱₀, with initial values.
  // @param param_model The parameter model vector 𝐤₀, with default values.
  OdeSystem(const SystemFunction& system_function,
            const VectorX<T>& state_model, const VectorX<T>& param_model);

 protected:
  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override;

 private:
  // General ODE system d𝐱/dt = f(t, 𝐱; 𝐤) function.
  const SystemFunction system_function_;
};

template <typename T>
OdeSystem<T>::OdeSystem(
    const typename OdeSystem<T>::SystemFunction& system_function,
    const VectorX<T>& state_model, const VectorX<T>& param_model)
    : system_function_(system_function) {
  // Models system state after the given state model.
  this->DeclareContinuousState(BasicVector<T>(state_model));
  // Models system parameters after the given parameter model.
  this->DeclareNumericParameter(BasicVector<T>(param_model));
}

template <typename T>
void OdeSystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  // Retrieves the state vector. This cast is safe because the
  // ContinuousState<T> of a LeafSystem<T> is flat i.e. it is just
  // a BasicVector<T>, and the implementation deals with LeafSystem<T>
  // instances only by design.
  const BasicVector<T>& state_vector = dynamic_cast<const BasicVector<T>&>(
      context.get_continuous_state_vector());
  // Retrieves the parameter vector.
  const BasicVector<T>& parameter_vector = context.get_numeric_parameter(0);

  // Retrieves the derivatives vector. This cast is safe because the
  // ContinuousState<T> of a LeafSystem<T> is flat i.e. it is just
  // a BasicVector<T>, and the implementation deals with LeafSystem<T>
  // instances only by design.
  BasicVector<T>& derivatives_vector =
      dynamic_cast<BasicVector<T>&>(derivatives->get_mutable_vector());
  // Computes the derivatives vector using the given system function
  // for the given time and state and with the given parameterization.
  derivatives_vector.set_value(system_function_(context.get_time(),
                                                state_vector.get_value(),
                                                parameter_vector.get_value()));
}

}  // namespace

template <typename T>
const double InitialValueProblem<T>::kDefaultAccuracy = 1e-4;

template <typename T>
const double InitialValueProblem<T>::kInitialStepSize = 1e-4;

template <typename T>
const double InitialValueProblem<T>::kMaxStepSize = 1e-1;

template <typename T>
InitialValueProblem<T>::InitialValueProblem(
    const OdeFunction& ode_function, const Eigen::Ref<const VectorX<T>>& x0,
    const Eigen::Ref<const VectorX<T>>& k) {
  // Instantiates the system using the given initial conditions and parameters.
  system_ = std::make_unique<OdeSystem<T>>(ode_function, x0, k);

  // Allocates a new default integration context.
  context_ = system_->CreateDefaultContext();

  // Instantiates an explicit RK3 integrator by default.
  integrator_ =
      std::make_unique<RungeKutta3Integrator<T>>(*system_, context_.get());

  // Sets step size and accuracy defaults.
  integrator_->request_initial_step_size_target(
      InitialValueProblem<T>::kInitialStepSize);
  integrator_->set_maximum_step_size(InitialValueProblem<T>::kMaxStepSize);
  integrator_->set_target_accuracy(InitialValueProblem<T>::kDefaultAccuracy);
}

template <typename T>
VectorX<T> InitialValueProblem<T>::Solve(const T& t0, const T& tf) const {
  DRAKE_THROW_UNLESS(tf >= t0);
  context_->SetTime(t0);

  ResetState();

  // Initializes integrator if necessary.
  if (!integrator_->is_initialized()) {
    integrator_->Initialize();
  }

  // Integrates up to the requested time.
  integrator_->IntegrateWithMultipleStepsToTime(tf);

  // Retrieves the state vector. This cast is safe because the
  // ContinuousState<T> of a LeafSystem<T> is flat i.e. it is just
  // a BasicVector<T>, and the implementation deals with LeafSystem<T>
  // instances only by design.
  const BasicVector<T>& state_vector = dynamic_cast<const BasicVector<T>&>(
      context_->get_continuous_state_vector());
  return state_vector.get_value();
}

template <typename T>
void InitialValueProblem<T>::ResetState() const {
  system_->SetDefaultContext(context_.get());

  // Keeps track of current step size and accuracy settings (regardless
  // of whether these are actually used by the integrator instance or not).
  const T max_step_size = integrator_->get_maximum_step_size();
  const T initial_step_size = integrator_->get_initial_step_size_target();
  const double target_accuracy = integrator_->get_target_accuracy();

  // Resets the integrator internal state.
  integrator_->Reset();

  // Sets integrator settings again.
  integrator_->set_maximum_step_size(max_step_size);
  if (integrator_->supports_error_estimation()) {
    // Specifies initial step and accuracy setting only if necessary.
    integrator_->request_initial_step_size_target(initial_step_size);
    integrator_->set_target_accuracy(target_accuracy);
  }
}

template <typename T>
std::unique_ptr<DenseOutput<T>> InitialValueProblem<T>::DenseSolve(
    const T& t0, const T& tf) const {
  DRAKE_THROW_UNLESS(tf >= t0);
  context_->SetTime(t0);
  ResetState();

  // Unconditionally re-initialize integrator.
  integrator_->Initialize();

  // Starts dense integration to build a dense output.
  integrator_->StartDenseIntegration();

  // Steps the integrator through the entire interval.
  integrator_->IntegrateWithMultipleStepsToTime(tf);

  // Stops dense integration to prevent future updates to
  // the dense output just built and yields it to the caller.
  const std::unique_ptr<trajectories::PiecewisePolynomial<T>> traj =
      integrator_->StopDenseIntegration();

  return std::make_unique<HermitianDenseOutput<T>>(*traj);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::InitialValueProblem);
