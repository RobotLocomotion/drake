#include "drake/systems/estimators/luenberger_observer.h"

#include <cmath>
#include <set>
#include <utility>

namespace drake {
namespace systems {
namespace estimators {

template <typename T>
LuenbergerObserver<T>::LuenbergerObserver(
    const System<T>* system, std::unique_ptr<System<T>> owned_system,
    const Context<T>& observed_system_context,
    const Eigen::Ref<const Eigen::MatrixXd>& observer_gain)
    : owned_system_(std::move(owned_system)),
      observed_system_(owned_system_ ? owned_system_.get() : system),
      observer_gain_(observer_gain) {
  DRAKE_DEMAND(observed_system_ != nullptr);
  observed_system_->ValidateContext(observed_system_context);

  // Note: Could potentially extend this to MIMO systems.
  DRAKE_DEMAND(observed_system_->num_input_ports() <= 1);
  DRAKE_DEMAND(observed_system_->num_output_ports() == 1);
  DRAKE_DEMAND(observed_system_->get_output_port(0).get_data_type() ==
               kVectorValued);

  DRAKE_DEMAND(observed_system_context.has_only_continuous_state());
  // Otherwise there is nothing to estimate.

  // TODO(russt): Add support for discrete-time systems (should be easy enough
  // to support both discrete and continuous simultaneously).

  // Observer state is the (estimated) state of the observed system,
  // but the best we can do for now is to allocate a similarly-sized
  // BasicVector (see #6998).
  const auto& xc = observed_system_context.get_continuous_state();
  const int num_q = xc.get_generalized_position().size();
  const int num_v = xc.get_generalized_velocity().size();
  const int num_z = xc.get_misc_continuous_state().size();
  this->DeclareContinuousState(num_q, num_v, num_z);

  // Output port is the (estimated) state of the observed system.
  // Note: Again, the derived type of the state vector is lost here, because xc
  // is only guaranteed to be a VectorBase, not a BasicVector.
  this->DeclareVectorOutputPort("estimated_state", xc.size(),
                                &LuenbergerObserver::CalcEstimatedState,
                                {this->xc_ticket()});

  // First input port is the output of the observed system.
  const auto& y_input_port = this->DeclareVectorInputPort(
      "observed_system_output",
      *observed_system_->AllocateOutput()->get_vector_data(0));
  std::set<DependencyTicket> input_port_tickets{y_input_port.ticket()};

  // Check the size of the gain matrix.
  DRAKE_DEMAND(observer_gain_.rows() == xc.size());
  DRAKE_DEMAND(observer_gain_.cols() ==
               observed_system_->get_output_port(0).size());

  // Second input port is the input to the observed system (if it exists).
  if (observed_system_->num_input_ports() > 0) {
    DRAKE_DEMAND(observed_system_->get_input_port(0).get_data_type() ==
                 kVectorValued);
    auto input_vec = observed_system_->AllocateInputVector(
        observed_system_->get_input_port(0));
    const auto& u_input_port =
        this->DeclareVectorInputPort("observed_system_input", *input_vec);
    input_port_tickets.emplace(u_input_port.ticket());
  }

  // Copy the observed system context into a cache entry where we can safely
  // modify it without runtime reallocation or a (non-thread-safe) mutable
  // member.
  observed_system_context_cache_entry_ = &this->DeclareCacheEntry(
      "observed system context", observed_system_context,
      &LuenbergerObserver::UpdateObservedSystemContext, input_port_tickets);
}

template <typename T>
LuenbergerObserver<T>::LuenbergerObserver(
    const System<T>& observed_system, const Context<T>& observed_system_context,
    const Eigen::Ref<const Eigen::MatrixXd>& observer_gain)
    : LuenbergerObserver(&observed_system, nullptr, observed_system_context,
                         observer_gain) {}

template <typename T>
LuenbergerObserver<T>::LuenbergerObserver(
    std::unique_ptr<System<T>> observed_system,
    const Context<T>& observed_system_context,
    const Eigen::Ref<const Eigen::MatrixXd>& observer_gain)
    : LuenbergerObserver(nullptr, std::move(observed_system),
                         observed_system_context, observer_gain) {}

template <typename T>
void LuenbergerObserver<T>::CalcEstimatedState(const Context<T>& context,
                                               BasicVector<T>* output) const {
  output->set_value(context.get_continuous_state_vector().CopyToVector());
}

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
    observed_system_->get_input_port(0).FixValue(
        observed_system_context,
        this->get_observed_system_input_input_port().Eval(context));
  }
  // Set observed system state.
  observed_system_context->get_mutable_continuous_state_vector().SetFrom(
      context.get_continuous_state_vector());
}

template <typename T>
void LuenbergerObserver<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  const Context<T>& observed_system_context =
      observed_system_context_cache_entry_->Eval<Context<T>>(context);

  // Evaluate the observed system.
  const VectorX<T>& yhat =
      observed_system_->get_output_port(0).Eval(observed_system_context);
  VectorX<T> xdothat =
      observed_system_->EvalTimeDerivatives(observed_system_context)
          .CopyToVector();

  // Get the measurements.
  const VectorX<T>& y =
      this->get_observed_system_output_input_port().Eval(context);

  // xdothat = f(xhat,u) + L(y-yhat).
  derivatives->SetFromVector(xdothat + observer_gain_ * (y - yhat));
}

}  // namespace estimators
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::estimators::LuenbergerObserver)
