#include "drake/systems/estimators/luenberger_observer.h"

#include <cmath>
#include <set>
#include <utility>

namespace drake {
namespace systems {
namespace estimators {

template <typename T>
template <typename U>
LuenbergerObserver<T>::LuenbergerObserver(
    std::shared_ptr<const System<T>> observed_system,
    const Context<U>& observed_system_context_U,
    const Eigen::Ref<const Eigen::MatrixXd>& observer_gain, U)
    : LeafSystem<T>(SystemTypeTag<LuenbergerObserver>{}),
      observed_system_(std::move(observed_system)),
      observer_gain_(observer_gain) {
  DRAKE_THROW_UNLESS(observed_system_ != nullptr);

  this->get_mutable_system_scalar_converter().RemoveUnlessAlsoSupportedBy(
      observed_system_->get_system_scalar_converter());

  std::unique_ptr<Context<T>> observed_system_context_T;
  const Context<T>* observed_system_context;
  if constexpr (!std::is_same_v<T, U>) {
    observed_system_context_T = observed_system_->CreateDefaultContext();
    observed_system_context_T->SetTimeStateAndParametersFrom(
        observed_system_context_U);
    observed_system_context = observed_system_context_T.get();
  } else {
    observed_system_->ValidateContext(observed_system_context_U);
    observed_system_context = &observed_system_context_U;
  }

  // Note: Could potentially extend this to MIMO systems.
  const InputPort<T>* const observed_system_input =
      (observed_system_->num_input_ports() > 0)
          ? &observed_system_->get_input_port()
          : nullptr;
  const OutputPort<T>& observed_system_output =
      observed_system_->get_output_port();
  DRAKE_DEMAND(observed_system_output.get_data_type() == kVectorValued);

  DRAKE_DEMAND(observed_system_context->has_only_continuous_state() ||
               (observed_system_context->has_only_discrete_state() &&
                observed_system_context->num_discrete_state_groups() == 1));
  const bool is_continuous =
      observed_system_context->has_only_continuous_state();

  // Observer state is the (estimated) state of the observed system,
  // but the best we can do for now is to allocate a similarly-sized
  // BasicVector (see #6998).
  int num_states;
  if (is_continuous) {
    const auto& xc = observed_system_context->get_continuous_state();
    num_states = xc.size();
    const int num_q = xc.get_generalized_position().size();
    const int num_v = xc.get_generalized_velocity().size();
    const int num_z = xc.get_misc_continuous_state().size();
    ContinuousStateIndex state_index =
        this->DeclareContinuousState(num_q, num_v, num_z);
    this->DeclareStateOutputPort("estimated_state", state_index);
  } else {
    num_states = observed_system_context->get_discrete_state_vector().size();
    DiscreteStateIndex state_index = this->DeclareDiscreteState(num_states);
    this->DeclareStateOutputPort("estimated_state", state_index);

    DRAKE_THROW_UNLESS(
        observed_system_->GetUniquePeriodicDiscreteUpdateAttribute()
            .has_value());
    auto discrete_attr =
        observed_system_->GetUniquePeriodicDiscreteUpdateAttribute().value();
    this->DeclarePeriodicDiscreteUpdateEvent(
        discrete_attr.period_sec(), discrete_attr.offset_sec(),
        &LuenbergerObserver::DiscreteUpdate);
  }

  // First input port is the output of the observed system.
  const auto& y_input_port = this->DeclareVectorInputPort(
      "observed_system_output",
      *observed_system_->AllocateOutput()->get_vector_data(
          observed_system_output.get_index()));
  std::set<DependencyTicket> input_port_tickets{y_input_port.ticket()};

  // Check the size of the gain matrix.
  DRAKE_DEMAND(observer_gain_.rows() == num_states);
  DRAKE_DEMAND(observer_gain_.cols() == observed_system_output.size());

  // Second input port is the input to the observed system (if it exists).
  if (observed_system_input != nullptr) {
    DRAKE_DEMAND(observed_system_input->get_data_type() == kVectorValued);
    auto input_vec =
        observed_system_->AllocateInputVector(*observed_system_input);
    const auto& u_input_port =
        this->DeclareVectorInputPort("observed_system_input", *input_vec);
    input_port_tickets.emplace(u_input_port.ticket());
  }

  // Copy the observed system context into a cache entry where we can safely
  // modify it without runtime reallocation or a (non-thread-safe) mutable
  // member.
  observed_system_context_cache_entry_ = &this->DeclareCacheEntry(
      "observed system context", *observed_system_context,
      &LuenbergerObserver::UpdateObservedSystemContext, input_port_tickets);
}

template <typename T>
LuenbergerObserver<T>::LuenbergerObserver(
    std::shared_ptr<const System<T>> observed_system,
    const Context<T>& observed_system_context,
    const Eigen::Ref<const Eigen::MatrixXd>& observer_gain)
    : LuenbergerObserver(std::move(observed_system), observed_system_context,
                         observer_gain, T{}) {}

template <typename T>
LuenbergerObserver<T>::LuenbergerObserver(
    const System<T>& observed_system, const Context<T>& observed_system_context,
    const Eigen::Ref<const Eigen::MatrixXd>& observer_gain)
    : LuenbergerObserver(std::shared_ptr<const System<T>>(
                             /* managed object = */ std::shared_ptr<void>{},
                             /* stored pointer = */ &observed_system),
                         observed_system_context, observer_gain, T{}) {}

template <typename T>
template <typename U>
LuenbergerObserver<T>::LuenbergerObserver(const LuenbergerObserver<U>& other)
    : LuenbergerObserver(other.observed_system_->template ToScalarType<T>(),
                         other.observed_system_context_cache_entry_->Allocate()
                             ->template get_value<Context<U>>(),
                         other.observer_gain_, U{}) {}

template <typename T>
void LuenbergerObserver<T>::UpdateObservedSystemContext(
    const Context<T>& context, Context<T>* observed_system_context) const {
  // (Note that all relevant data in the observed_system_context is set here
  // from the observer state/inputs.  The observer_context does not hold any
  // hidden undeclared state).

  // Set observed system input.
  if (observed_system_->num_input_ports() > 0) {
    // TODO(russt): Avoid this dynamic allocation by fixing the input port once
    // and updating it here.
    if (this->get_observed_system_input_input_port().HasValue(context)) {
      observed_system_->get_input_port().FixValue(
          observed_system_context,
          this->get_observed_system_input_input_port().Eval(context));
    }
  }
  // Set observed system state.
  if (observed_system_context->has_only_continuous_state()) {
    observed_system_context->get_mutable_continuous_state_vector().SetFrom(
        context.get_continuous_state_vector());
  } else {
    observed_system_context->SetDiscreteState(
        context.get_discrete_state_vector().CopyToVector());
  }
  // Set time of observed system.
  observed_system_context->SetTime(context.get_time());
}

template <typename T>
void LuenbergerObserver<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  const Context<T>& observed_system_context =
      observed_system_context_cache_entry_->Eval<Context<T>>(context);

  // Evaluate the observed system.
  const VectorX<T>& yhat =
      observed_system_->get_output_port().Eval(observed_system_context);
  VectorX<T> xdothat =
      observed_system_->EvalTimeDerivatives(observed_system_context)
          .CopyToVector();

  // Get the measurements.
  const VectorX<T>& y =
      this->get_observed_system_output_input_port().Eval(context);

  // xdothat = f(xhat,u) + L(y-yhat).
  derivatives->SetFromVector(xdothat + observer_gain_ * (y - yhat));
}

template <typename T>
void LuenbergerObserver<T>::DiscreteUpdate(const Context<T>& context,
                                           DiscreteValues<T>* update) const {
  // Ensure observed system context is up to date.
  const Context<T>& observed_system_context =
      observed_system_context_cache_entry_->Eval<Context<T>>(context);

  // Evaluate the observed system.
  const VectorX<T>& yhat =
      observed_system_->get_output_port().Eval(observed_system_context);
  const VectorX<T> xhat_next =
      observed_system_
          ->EvalUniquePeriodicDiscreteUpdate(observed_system_context)
          .value();

  // Get the measurements.
  const VectorX<T>& y =
      this->get_observed_system_output_input_port().Eval(context);

  // x̂[n+1] = f(x̂[n],u) + L(y - g(x̂[n],u[n]))
  update->set_value(xhat_next + observer_gain_ * (y - yhat));
}

}  // namespace estimators
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::estimators::LuenbergerObserver);
