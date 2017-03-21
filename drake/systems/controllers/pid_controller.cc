#include "drake/systems/controllers/pid_controller.h"

#include <string>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/pass_through.h"

using std::make_unique;

namespace drake {
namespace systems {

/// A PID controller system. Given an error signal `e` and its time derivative
/// `edot` the output of this sytem is
/// <pre>
///     y = Kp * e + Ki integral(e,dt) + Kd * edot
/// </pre>
/// where `integral(e,dt)` is the time integral of `e`.
/// A PID controller directly feedthroughs the error signal to the output when
/// the proportional constant is non-zero. It feeds through the rate of change
/// of the error signal when the derivative constant is non-zero.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
/// @ingroup primitive_systems
template <typename T>
class PidControllerInternal : public Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PidControllerInternal)

  /// Constructs a %PidControllerInternal system where all of the gains are the
  /// same value.
  ///
  /// @param Kp the proportional constant.
  /// @param Ki the integral constant.
  /// @param Kd the derivative constant.
  /// @param size number of elements in the error signal to be processed.
  PidControllerInternal(const T& Kp, const T& Ki, const T& Kd, int size);

  /// Constructs a %PidControllerInternal system where each gain can have a
  /// different value.
  ///
  /// @param Kp the vector of proportional gain constants.
  /// @param Ki the vector of integral gain constants.
  /// @param Kd the vector of derivative gain constants.
  PidControllerInternal(const VectorX<T>& Kp, const VectorX<T>& Ki,
                const VectorX<T>& Kd);

  ~PidControllerInternal() override {}

  /// Returns the proportional gain constant. This method should only be called
  /// if the proportional gain can be represented as a scalar value, i.e., every
  /// element in the proportional gain vector is the same. It will throw a
  /// `std::runtime_error` if the proportional gain cannot be represented as a
  /// scalar value.
  const T& get_Kp_singleton() const;

  /// Returns the integral gain constant. This method should only be called if
  /// the integral gain can be represented as a scalar value, i.e., every
  /// element in the integral gain vector is the same. It will throw a
  /// `std::runtime_error` if the integral gain cannot be represented as a
  /// scalar value.
  const T& get_Ki_singleton() const;

  /// Returns the derivative gain constant. This method should only be called if
  /// the derivative gain can be represented as a scalar value, i.e., every
  /// element in the derivative gain vector is the same. It will throw a
  /// `std::runtime_error` if the derivative gain cannot be represented as a
  /// scalar value.
  const T& get_Kd_singleton() const;

  /// Returns the proportional vector constant.
  const VectorX<T>& get_Kp_vector() const;

  /// Returns the integral vector constant.
  const VectorX<T>& get_Ki_vector() const;

  /// Returns the derivative vector constant.
  const VectorX<T>& get_Kd_vector() const;

  // System<T> overrides
  /// A PID controller directly feedthroughs the error signal to the output when
  /// the proportional constant is non-zero. It feeds through the rate of change
  /// of the error signal when the derivative constant is non-zero.
  bool has_any_direct_feedthrough() const override;

  /// Sets the integral of the %PidControllerInternal to @p value.
  /// @p value must be a column vector of the appropriate size.
  void set_integral_value(Context<T>* context,
                          const Eigen::Ref<const VectorX<T>>& value) const;

  /// Returns the input port to the error signal.
  const InputPortDescriptor<T>& get_error_port() const;

  /// Returns the input port to the time derivative or rate of the error signal.
  const InputPortDescriptor<T>& get_error_derivative_port() const;

  /// Returns the output port to the control output.
  const OutputPortDescriptor<T>& get_control_output_port() const;

 private:
  Adder<T>* adder_ = nullptr;
  Integrator<T>* integrator_ = nullptr;
  PassThrough<T>* pass_through_ = nullptr;
  Gain<T>* proportional_gain_ = nullptr;
  Gain<T>* integral_gain_ = nullptr;
  Gain<T>* derivative_gain_ = nullptr;
};

template <typename T>
PidControllerInternal<T>::PidControllerInternal(
    const T& Kp, const T& Ki, const T& Kd, int size)
    : PidControllerInternal(
        VectorX<T>::Ones(size) * Kp,
        VectorX<T>::Ones(size) * Ki,
        VectorX<T>::Ones(size) * Kd) { }

template <typename T>
PidControllerInternal<T>::PidControllerInternal(
    const VectorX<T>& Kp, const VectorX<T>& Ki, const VectorX<T>& Kd)
    : Diagram<T>() {
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
  builder.ExportInput(pass_through_->get_input_port());
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
const T& PidControllerInternal<T>::get_Kp_singleton() const {
  return proportional_gain_->get_gain();
}

template <typename T>
const T& PidControllerInternal<T>::get_Ki_singleton() const {
  return integral_gain_->get_gain();
}

template <typename T>
const T& PidControllerInternal<T>::get_Kd_singleton() const {
  return derivative_gain_->get_gain();
}

template <typename T>
const VectorX<T>& PidControllerInternal<T>::get_Kp_vector() const {
  return proportional_gain_->get_gain_vector();
}

template <typename T>
const VectorX<T>& PidControllerInternal<T>::get_Ki_vector() const {
  return integral_gain_->get_gain_vector();
}

template <typename T>
const VectorX<T>& PidControllerInternal<T>::get_Kd_vector() const {
  return derivative_gain_->get_gain_vector();
}

template <typename T>
bool PidControllerInternal<T>::has_any_direct_feedthrough() const {
  return !get_Kp_vector().isZero() || !get_Kd_vector().isZero();
}

template <typename T>
const InputPortDescriptor<T>& PidControllerInternal<T>::get_error_port() const {
  return Diagram<T>::get_input_port(0);
}

template <typename T>
const InputPortDescriptor<T>&
PidControllerInternal<T>::get_error_derivative_port() const {
  return Diagram<T>::get_input_port(1);
}


template <typename T>
const OutputPortDescriptor<T>&
PidControllerInternal<T>::get_control_output_port() const {
  return System<T>::get_output_port(0);
}

template <typename T>
void PidControllerInternal<T>::set_integral_value(
    Context<T>* context, const Eigen::Ref<const VectorX<T>>& value) const {
  Context<T>* integrator_context =
      Diagram<T>::GetMutableSubsystemContext(context, integrator_);
  integrator_->set_integral_value(integrator_context, value);
}

template <typename T>
PidController<T>::PidController(
    const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd) {
  ConnectPorts(nullptr, kp, ki, kd);
}

template <typename T>
PidController<T>::PidController(
    std::unique_ptr<MatrixGain<T>> feedback_selector,
    const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd) {
  ConnectPorts(std::move(feedback_selector), kp, ki, kd);
}

template <typename T>
void PidController<T>::ConnectPorts(
    std::unique_ptr<MatrixGain<T>> feedback_selector,
    const VectorX<T>& kp, const VectorX<T>& ki,
    const VectorX<T>& kd) {
  DRAKE_DEMAND(kp.size() == kd.size());
  DRAKE_DEMAND(ki.size() == kd.size());

  DiagramBuilder<T> builder;

  if (feedback_selector == nullptr) {
    // No feedback selector was provided. Create a GainMatrix containing an
    // identity matrix, which results in every element of the plant's output
    // port zero being used as the feedback signal to the PID controller.
    feedback_selector = std::make_unique<MatrixGain<T>>(2 * kp.size());
  }
  auto feedback_selector_p =
    builder.template AddSystem(std::move(feedback_selector));

  DRAKE_DEMAND(2 * kp.size() == feedback_selector_p->get_output_port().size());

  const int num_effort_commands = kp.size();
  const int num_states = num_effort_commands * 2;

  DRAKE_DEMAND(feedback_selector_p->get_output_port().size() == num_states);

  auto state_minus_target =
    builder.template AddSystem<Adder<T>>(2, num_states);

  controller_ =
      builder.template AddSystem<PidControllerInternal<T>>(kp, ki, kd);

  // Split the input into two signals one with the positions and one
  // with the velocities.
  auto error_demux = builder.template AddSystem<Demultiplexer<T>>(
      num_states, num_effort_commands);

  auto controller_inverter =
    builder.template AddSystem<Gain<T>>(-1.0, num_effort_commands);
  auto error_inverter = builder.template AddSystem<Gain<T>>(-1.0, num_states);

  builder.Connect(error_inverter->get_output_port(),
      state_minus_target->get_input_port(0));
  builder.Connect(feedback_selector_p->get_output_port(),
      state_minus_target->get_input_port(1));

  // Splits the error signal into positions and velocities components.
  builder.Connect(state_minus_target->get_output_port(),
      error_demux->get_input_port(0));

  // Connects PID controller.
  builder.Connect(error_demux->get_output_port(0),
      controller_->get_error_port());
  builder.Connect(error_demux->get_output_port(1),
      controller_->get_error_derivative_port());
  // Adds feedback.
  builder.Connect(controller_->get_output_port(0),
      controller_inverter->get_input_port());

  // Expose state input
  int index = builder.ExportInput(feedback_selector_p->get_input_port());
  this->set_input_port_index_estimated_state(index);

  // Exposes desired state input
  index = builder.ExportInput(error_inverter->get_input_port());
  this->set_input_port_index_desired_state(index);

  // Exposes torque output
  index = builder.ExportOutput(controller_inverter->get_output_port());
  this->set_output_port_index_control(index);

  builder.BuildInto(this);
}

template <typename T>
const T& PidController<T>::get_Kp_singleton() const {
  return controller_->get_Kp_singleton();
}

template <typename T>
const T& PidController<T>::get_Ki_singleton() const {
  return controller_->get_Ki_singleton();
}

template <typename T>
const T& PidController<T>::get_Kd_singleton() const {
  return controller_->get_Kd_singleton();
}

template <typename T>
const VectorX<T>& PidController<T>::get_Kp_vector() const {
  return controller_->get_Kp_vector();
}

template <typename T>
const VectorX<T>& PidController<T>::get_Ki_vector() const {
  return controller_->get_Ki_vector();
}

template <typename T>
const VectorX<T>& PidController<T>::get_Kd_vector() const {
  return controller_->get_Kd_vector();
}

template <typename T>
void PidController<T>::set_integral_value(
    Context<T>* context, const Eigen::Ref<const VectorX<T>>& value) const {
  Context<T>* integrator_context =
      Diagram<T>::GetMutableSubsystemContext(context, controller_);
  // TODO(siyuanfeng): need to get rid of the - once we switch the integrator
  // to be int(q_d - q), right now it's (q - q_d).
  controller_->set_integral_value(integrator_context, -value);
}

template <typename T>
bool PidController<T>::has_any_direct_feedthrough() const {
  return !get_Kp_vector().isZero() || !get_Kd_vector().isZero();
}

// Adds a simple record-based representation of the PID controller to @p dot.
template <typename T>
void PidController<T>::GetGraphvizFragment(std::stringstream* dot) const {
  std::string name = this->get_name();
  if (name.empty()) {
    name = "PID Controller";
  }
  *dot << this->GetGraphvizId() << " [shape=record, label=\"" << name;
  *dot << " | { {<u0> q |<u1> q_d} |<y0> y}";
  *dot << "\"];" << std::endl;
}

template <typename T>
void PidController<T>::GetGraphvizInputPortToken(
    const InputPortDescriptor<T>& port, std::stringstream* dot) const {
  DRAKE_DEMAND(port.get_system() == this);
  *dot << this->GetGraphvizId() << ":u" << port.get_index();
}

template <typename T>
void PidController<T>::GetGraphvizOutputPortToken(
    const OutputPortDescriptor<T>& port, std::stringstream* dot) const {
  DRAKE_DEMAND(port.get_system() == this);
  *dot << this->GetGraphvizId() << ":y" << port.get_index();
}

template class PidControllerInternal<double>;
template class PidControllerInternal<AutoDiffXd>;

template class PidController<double>;
template class PidController<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
