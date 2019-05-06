#include "drake/systems/estimators/luenberger_observer.h"

#include <cmath>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace systems {
namespace estimators {

template <typename T>
LuenbergerObserver<T>::LuenbergerObserver(
    std::unique_ptr<const systems::System<T>> observed_system,
    std::unique_ptr<systems::Context<T>> observed_system_context,
    const Eigen::Ref<const Eigen::MatrixXd>& observer_gain)
    : observed_system_(std::move(observed_system)),
      L_(observer_gain),
      observed_context_(std::move(observed_system_context)) {
  DRAKE_DEMAND(observed_system_ != nullptr);
  DRAKE_DEMAND(observed_context_ != nullptr);

  // Note: Could potentially extend this to MIMO systems.
  DRAKE_THROW_UNLESS(observed_system_->num_input_ports() <= 1);
  DRAKE_THROW_UNLESS(observed_system_->num_output_ports() == 1);
  observed_output_port_ = &observed_system_->get_output_port(0);
  DRAKE_THROW_UNLESS(observed_output_port_->get_data_type() == kVectorValued);

  // TODO(russt): Add support for discrete-time systems (should be easy enough
  // to support both discrete and continuous simultaneously).
  DRAKE_THROW_UNLESS(observed_context_->has_only_continuous_state());

  // Check the shape of the gain matrix.
  DRAKE_THROW_UNLESS(L_.size() > 0);  // Otherwise there is nothing to estimate.
  DRAKE_THROW_UNLESS(L_.rows() == observed_context_->num_total_states());
  DRAKE_THROW_UNLESS(L_.cols() == observed_output_port_->size());

  // Observer state is the (estimated) state of the observed system,
  // but the best we can do for now is to allocate a similarly-sized
  // BasicVector (see #6998).
  const auto& xc = observed_context_->get_continuous_state();
  const int num_q = xc.get_generalized_position().size();
  const int num_v = xc.get_generalized_velocity().size();
  const int num_z = xc.get_misc_continuous_state().size();
  this->DeclareContinuousState(num_q, num_v, num_z);

  // Output port is the (estimated) state of the observed system.
  // Note: Again, the derived type of the state vector is lost here, because xc
  // is only guaranteed to be a VectorBase, not a BasicVector.
  this->DeclareVectorOutputPort(BasicVector<T>(xc.size()),
                                &LuenbergerObserver::CalcEstimatedState,
                                {this->xc_ticket()});

  // First input port is the output of the observed system.
  // Note: Throws bad_cast if the output is not vector-valued.
  this->DeclareVectorInputPort(
      "y",
      observed_output_port_->Allocate()->template get_value<BasicVector<T>>());

  // Second input port is the input to the observed system (if it exists).
  if (observed_system_->num_input_ports() > 0) {
    const auto& observed_input = observed_system_->get_input_port(0);
    auto input_vec = observed_system_->AllocateInputVector(observed_input);
    this->DeclareVectorInputPort("u", *input_vec);
    observed_fixed_input_ = &observed_context_->FixInputPort(
        observed_input.get_index(), *input_vec);
  }
}

template <typename T>
void LuenbergerObserver<T>::CalcEstimatedState(
    const systems::Context<T>& context, systems::BasicVector<T>* output) const {
  output->SetFrom(context.get_continuous_state_vector());
}

template <typename T>
void LuenbergerObserver<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // (Note that all relevant data in the observed_context is set here from the
  // observer state/inputs;it does not hold any hidden undeclared state.)

  // Set observed system input.
  if (observed_fixed_input_) {
    const auto& u = this->get_input_port(1).Eval(context);
    observed_fixed_input_->GetMutableVectorData<T>()->set_value(u);
  }

  // Set observed system state.
  observed_context_->get_mutable_continuous_state_vector().SetFrom(
      context.get_continuous_state_vector());

  // Get the measurements.
  const auto& y = this->get_input_port(0).Eval(context);
  const auto& yhat = observed_output_port_->Eval(*observed_context_);

  // Return xdothat = f(xhat,u) + L(y-yhat).
  const auto& f_xhat =
      observed_system_->EvalTimeDerivatives(*observed_context_).CopyToVector();
  derivatives->SetFromVector(f_xhat + L_ * (y - yhat));
}

}  // namespace estimators
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::estimators::LuenbergerObserver)
