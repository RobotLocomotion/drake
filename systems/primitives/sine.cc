#include "drake/systems/primitives/sine.h"

#include <sstream>

#include "drake/common/drake_throw.h"

namespace drake {
namespace systems {

template <typename T>
Sine<T>::Sine(double amplitude, double frequency, double phase, int size,
              bool is_time_based)
    : Sine(Eigen::VectorXd::Ones(size) * amplitude,
           Eigen::VectorXd::Ones(size) * frequency,
           Eigen::VectorXd::Ones(size) * phase, is_time_based) {}

template <typename T>
Sine<T>::Sine(const Eigen::VectorXd& amplitudes,
              const Eigen::VectorXd& frequencies,
              const Eigen::VectorXd& phases,
              bool is_time_based)
      : LeafSystem<T>(SystemTypeTag<Sine>{}),
      amplitude_(amplitudes), frequency_(frequencies), phase_(phases),
      is_time_based_(is_time_based) {
  // Ensure the incoming vectors are all the same size
  DRAKE_THROW_UNLESS(amplitudes.size() == frequencies.size());
  DRAKE_THROW_UNLESS(amplitudes.size() == phases.size());

  // Check each of the incoming vectors. For each vector, set a flag if every
  // element in that vector is the same.
  is_const_amplitude_ = amplitude_.isConstant(amplitude_[0]);
  is_const_frequency_ = frequency_.isConstant(frequency_[0]);
  is_const_phase_ = phase_.isConstant(phase_[0]);

  // If the Sine system is system time based, do not create an input port.
  // System time is used as the time variable in this case. If the Sine system
  // is not system time based, create an input port that contains the signal to
  // be used as the time variable.
  if (!is_time_based) {
    this->DeclareInputPort(kUseDefaultName, kVectorValued, amplitudes.size());
  }
  value_output_port_index_ =
      this->DeclareVectorOutputPort(kUseDefaultName, amplitudes.size(),
                                    &Sine::CalcValueOutput)
          .get_index();
  first_derivative_output_port_index_ =
      this->DeclareVectorOutputPort(kUseDefaultName, amplitudes.size(),
                                    &Sine::CalcFirstDerivativeOutput)
          .get_index();
  second_derivative_output_port_index_ =
      this->DeclareVectorOutputPort(kUseDefaultName, amplitudes.size(),
                                    &Sine::CalcSecondDerivativeOutput)
          .get_index();
}

template <typename T>
template <typename U>
Sine<T>::Sine(const Sine<U>& other)
    : Sine<T>(other.amplitude_vector(), other.frequency_vector(),
              other.phase_vector(), other.is_time_based()) {}

template <typename T>
double Sine<T>::amplitude() const {
  if (!is_const_amplitude_) {
    std::stringstream s;
    s << "The amplitude vector, [" << amplitude_ << "], cannot be represented "
      << "as a scalar value. Please use "
      << "drake::systems::Sine::amplitude_vector() instead.";
    throw std::logic_error(s.str());
  }
  return amplitude_[0];
}

template <typename T>
double Sine<T>::frequency() const {
  if (!is_const_frequency_) {
    std::stringstream s;
    s << "The frequency vector, [" << frequency_ << "], cannot be represented "
      << "as a scalar value. Please use "
      << "drake::systems::Sine::frequency_vector() instead.";
    throw std::logic_error(s.str());
  }
  return frequency_[0];
}

template <typename T>
double Sine<T>::phase() const {
  if (!is_const_phase_) {
    std::stringstream s;
    s << "The phase vector, [" << phase_ << "], cannot be represented as a "
      << "scalar value. Please use "
      << "drake::systems::Sine::phase_vector() instead.";
    throw std::logic_error(s.str().c_str());
  }
  return phase_[0];
}

template <typename T>
bool Sine<T>::is_time_based() const {
  return is_time_based_;
}

template <typename T>
const Eigen::VectorXd& Sine<T>::amplitude_vector() const {
  return amplitude_;
}

template <typename T>
const Eigen::VectorXd& Sine<T>::frequency_vector() const {
  return frequency_;
}

template <typename T>
const Eigen::VectorXd& Sine<T>::phase_vector() const {
  return phase_;
}

template <typename T>
void Sine<T>::CalcValueOutput(const Context<T>& context,
                              BasicVector<T>* output) const {
  VectorX<T> sine_arg;
  Sine::CalcArg(context, &sine_arg);

  Eigen::VectorBlock<VectorX<T>> output_block = output->get_mutable_value();
  output_block = amplitude_.array() * sine_arg.array().sin();
}

template <typename T>
void Sine<T>::CalcFirstDerivativeOutput(
    const Context<T>& context, BasicVector<T>* output) const {

  VectorX<T> cos_arg;
  Sine::CalcArg(context, &cos_arg);

  Eigen::VectorBlock<VectorX<T>> output_block = output->get_mutable_value();
  output_block =
      amplitude_.array() * frequency_.array() * cos_arg.array().cos();
}

template <typename T>
void Sine<T>::CalcSecondDerivativeOutput(
    const Context<T>& context, BasicVector<T>* output) const {

  VectorX<T> sine_arg;
  Sine::CalcArg(context, &sine_arg);

  Eigen::VectorBlock<VectorX<T>> output_block = output->get_mutable_value();
  output_block =
      - amplitude_.array() * frequency_.array().pow(2) * sine_arg.array().sin();
}

template <typename T>
void Sine<T>::CalcArg(
    const Context<T>& context, VectorX<T>* arg) const {

  if (is_time_based_) {
    VectorX<T> time_vec(amplitude_.size());
    time_vec.fill(context.get_time());
    *arg = frequency_.array() * time_vec.array() + phase_.array();
  } else {
    const VectorX<T>& input = this->get_input_port(0).Eval(context);
    *arg = frequency_.array() * input.array() + phase_.array();
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Sine)
