#include "drake/systems/estimators/luenberger_observer.h"

#include <cmath>

namespace drake {
namespace systems {
namespace estimators {

template <typename T>
LuenbergerObserver<T>::LuenbergerObserver(
    std::unique_ptr<systems::System<T>> observed_system,
    std::unique_ptr<systems::Context<T>> observed_system_context,
    const Eigen::Ref<const Eigen::MatrixXd>& observer_gain)
    : observed_system_(std::move(observed_system)),
      observer_gain_(observer_gain),
      observed_system_context_(std::move(observed_system_context)),
      observed_system_output_(
          observed_system_->AllocateOutput(*observed_system_context_)),
      observed_system_derivatives_(
          observed_system_->AllocateTimeDerivatives()) {
  DRAKE_DEMAND(observed_system_ != nullptr);

  // Note: Could potentially extend this to MIMO systems.
  DRAKE_DEMAND(observed_system_->get_num_input_ports() <= 1);
  DRAKE_DEMAND(observed_system_->get_num_output_ports() == 1);

  DRAKE_DEMAND(observed_system_context_->has_only_continuous_state());
  // Otherwise there is nothing to estimate.

  // TODO(russt): Add support for discrete-time systems (should be easy enough
  // to support both discrete and continuous simultaneously).

  // Observer state is the (estimated) state of the observed system
  // (returned by overloaded AllocateContinuousState).
  auto x = observed_system_context_->get_continuous_state();

  // Output port is the (estimated) state of the observed system.
  this->DeclareOutputPort(systems::kVectorValued, x->size());
  // TODO(russt): Could overload AllocateContinuousState and AllocateOutput to
  // call AllocateContinuousState on the observed system so that I can use the
  // actual derived class types.

  // First input port is the output of the observed system.
  this->DeclareInputPort(systems::kVectorValued,
                         observed_system_->get_output_port(0).get_size());

  // Check the size of the gain matrix.
  DRAKE_DEMAND(observer_gain_.rows() == x->size());
  DRAKE_DEMAND(observer_gain_.cols() ==
               observed_system_->get_output_port(0).get_size());

  // Second input port is the input to the observed system (if it exists).
  if (observed_system_->get_num_input_ports() > 0) {
    this->DeclareInputPort(systems::kVectorValued,
                           observed_system_->get_input_port(0).get_size());
  }

  // State has the same dimensions as the system being observed.
  const ContinuousState<T>& xc =
      *observed_system_context_->get_continuous_state();
  const int num_q = xc.get_generalized_position().size();
  const int num_v = xc.get_generalized_velocity().size();
  const int num_z = xc.get_misc_continuous_state().size();
  this->DeclareContinuousState(num_q, num_v, num_z);
}

template <typename T>
void LuenbergerObserver<T>::DoCalcOutput(
    const systems::Context<T>& context,
    systems::SystemOutput<T>* output) const {
  output->GetMutableVectorData(0)->set_value(
      context.get_continuous_state_vector().CopyToVector());
}

template <typename T>
void LuenbergerObserver<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // (Note that all relevant data in the observed_system_context is set here
  // from the observer state/inputs.  The observer_context does not hold any
  // hidden undeclared state).

  // Set observed system input.
  if (observed_system_->get_num_input_ports() > 0) {
    // TODO(russt): Avoid this dynamic allocation by fixing the input port once
    // and updating it here.
    observed_system_context_->FixInputPort(
        0, this->EvalVectorInput(context, 1)->CopyToVector());
  }
  // Set observed system state.
  observed_system_context_->get_mutable_continuous_state_vector()->SetFrom(
      context.get_continuous_state_vector());

  // Evaluate the observed system.
  observed_system_->CalcOutput(*observed_system_context_,
                               observed_system_output_.get());
  observed_system_->CalcTimeDerivatives(*observed_system_context_,
                                        observed_system_derivatives_.get());

  // Get the measurements.
  auto y = this->EvalVectorInput(context, 0)->CopyToVector();
  auto yhat = observed_system_output_->GetMutableVectorData(0)->CopyToVector();

  // Add in the observed gain terms.
  auto xdothat = observed_system_derivatives_->get_mutable_vector();

  // xdothat = f(xhat,u) + L(y-yhat).
  derivatives->SetFromVector(
      xdothat->CopyToVector() + observer_gain_ * (y - yhat));
}

template class LuenbergerObserver<double>;
template class LuenbergerObserver<AutoDiffXd>;

}  // namespace estimators
}  // namespace systems
}  // namespace drake
