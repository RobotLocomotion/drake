#include "drake/systems/controllers/pid_controller.h"

#include "drake/common/drake_export.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/diagram_builder.h"

using std::make_unique;

namespace drake {
namespace systems {

template <typename T>
PidController<T>::PidController(const T& Kp, const T& Ki, const T& Kd, int size)
    : PidController(VectorX<T>::Ones(size) * Kp, VectorX<T>::Ones(size) * Ki,
      VectorX<T>::Ones(size) * Kd) { }

template <typename T>
PidController<T>::PidController(const VectorX<T>& Kp, const VectorX<T>& Ki,
    const VectorX<T>& Kd) : Diagram<T>() {
  const int size = Kp.size();
  DRAKE_ASSERT(size > 0);

  DRAKE_ASSERT(Ki.size() == size);
  DRAKE_ASSERT(Kd.size() == size);

  for (int i = 0; i < size; ++i) {
    DRAKE_ASSERT(Kp(i) >= 0);
    DRAKE_ASSERT(Ki(i) >= 0);
    DRAKE_ASSERT(Kd(i) >= 0);
  }

  DiagramBuilder<T> builder;
  pass_through_ = builder.AddSystem(make_unique<PassThrough<T>>(size));
  proportional_gain_ = builder.AddSystem(make_unique<Gain<T>>(Kp));
  integral_gain_ = builder.AddSystem(make_unique<Gain<T>>(Ki));
  derivative_gain_ = builder.AddSystem(make_unique<Gain<T>>(Kd));
  integrator_ = builder.AddSystem(make_unique<Integrator<T>>(size));
  adder_ = builder.AddSystem(make_unique<Adder<T>>(3 /* inputs */, size));

  // Input 0 connects to the proportional and integral components.
  builder.ExportInput(pass_through_->get_input_port(0));
  // Input 1 connects directly to the derivative component.
  builder.ExportInput(derivative_gain_->get_input_port());
  builder.Connect(*pass_through_, *proportional_gain_);
  builder.Connect(*pass_through_, *integrator_);
  builder.Connect(*integrator_, *integral_gain_);
  builder.Connect(proportional_gain_->get_output_port(),
                  adder_->get_input_port(0));
  builder.Connect(integral_gain_->get_output_port(),
                  adder_->get_input_port(1));
  builder.Connect(derivative_gain_->get_output_port(),
                  adder_->get_input_port(2));
  builder.ExportOutput(adder_->get_output_port());
  builder.BuildInto(this);
}

template <typename T>
const T& PidController<T>::get_Kp() const {
  return proportional_gain_->get_gain();
}

template <typename T>
const T& PidController<T>::get_Ki() const {
  return integral_gain_->get_gain();
}

template <typename T>
const T& PidController<T>::get_Kd() const {
  return derivative_gain_->get_gain();
}

template <typename T>
const VectorX<T>& PidController<T>::get_Kp_vector() const {
  return proportional_gain_->get_gain_vector();
}

template <typename T>
const VectorX<T>& PidController<T>::get_Ki_vector() const {
  return integral_gain_->get_gain_vector();
}

template <typename T>
const VectorX<T>& PidController<T>::get_Kd_vector() const {
  return derivative_gain_->get_gain_vector();
}

template <typename T>
bool PidController<T>::has_any_direct_feedthrough() const {
  return !get_Kp_vector().isZero() || !get_Kd_vector().isZero();
}

template <typename T>
const SystemPortDescriptor<T>& PidController<T>::get_error_port() const {
  return Diagram<T>::get_input_port(0);
}

template <typename T>
const SystemPortDescriptor<T>&
PidController<T>::get_error_derivative_port() const {
  return Diagram<T>::get_input_port(1);
}


template <typename T>
const SystemPortDescriptor<T>&
PidController<T>::get_control_output_port() const {
  return System<T>::get_output_port(0);
}

template <typename T>
void PidController<T>::SetDefaultState(Context<T>* context) const {
  const int size = Diagram<T>::get_input_port(0).get_size();
  set_integral_value(context, VectorX<T>::Zero(size));
}

template <typename T>
void PidController<T>::set_integral_value(
    Context<T>* context, const Eigen::Ref<const VectorX<T>>& value) const {
  Context<T>* integrator_context =
      Diagram<T>::GetMutableSubsystemContext(context, integrator_);
  integrator_->set_integral_value(integrator_context, value);
}

template class DRAKE_EXPORT PidController<double>;
template class DRAKE_EXPORT PidController<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
