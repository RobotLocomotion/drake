#pragma once

/// @file
/// Template method implementations for sine.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include <sstream>

#include "drake/systems/primitives/sine.h"

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
      : LeafSystem<T>(SystemTypeTag<systems::Sine>{}),
      amplitude_(amplitudes), frequency_(frequencies), phase_(phases),
      is_time_based_(is_time_based) {
  // If the Sine system is system time based, do not create an input port.
  // System time is used as the time variable in this case. If the Sine system
  // is not system time based, create an input port that contains the signal to
  // be used as the time variable.
  if (!is_time_based) {
    this->DeclareInputPort(kVectorValued, amplitudes.size());
  }
  value_output_port_index_ =
      this->DeclareVectorOutputPort(
          BasicVector<T>(amplitudes.size()),
          &Sine::CalcValueOutput).get_index();
  first_derivative_output_port_index_ =
      this->DeclareVectorOutputPort(
          BasicVector<T>(amplitudes.size()),
          &Sine::CalcFirstDerivativeOutput).get_index();
  second_derivative_output_port_index_ =
      this->DeclareVectorOutputPort(
          BasicVector<T>(amplitudes.size()),
          &Sine::CalcSecondDerivativeOutput).get_index();
}

template <typename T>
template <typename U>
Sine<T>::Sine(const Sine<U>& other)
    : Sine<T>(other.get_amplitude_vector(), other.get_frequency_vector(),
              other.get_phase_vector(), other.is_time_based()) {}

template <typename T>
double Sine<T>::get_amplitude() const {
  if (!amplitude_.isConstant(amplitude_[0])) {
    std::stringstream s;
    s << "The amplitude vector, [" << amplitude_ << "], cannot be represented "
      << "as a scalar value. Please use "
      << "drake::systems::Sine::get_amplitude_vector() instead.";
    throw std::runtime_error(s.str().c_str());
  }
  return amplitude_[0];
}

template <typename T>
double Sine<T>::get_frequency() const {
  if (!frequency_.isConstant(frequency_[0])) {
    std::stringstream s;
    s << "The frequency vector, [" << frequency_ << "], cannot be represented "
      << "as a scalar value. Please use "
      << "drake::systems::Sine::get_frequency_vector() instead.";
    throw std::runtime_error(s.str().c_str());
  }
  return frequency_[0];
}

template <typename T>
double Sine<T>::get_phase() const {
  if (!phase_.isConstant(phase_[0])) {
    std::stringstream s;
    s << "The phase vector, [" << phase_ << "], cannot be represented as a "
      << "scalar value. Please use "
      << "drake::systems::Sine::get_phase_vector() instead.";
    throw std::runtime_error(s.str().c_str());
  }
  return phase_[0];
}

template <typename T>
bool Sine<T>::is_time_based() const {
  return is_time_based_;
}

template <typename T>
const Eigen::VectorXd& Sine<T>::get_amplitude_vector() const {
  return amplitude_;
}

template <typename T>
const Eigen::VectorXd& Sine<T>::get_frequency_vector() const {
  return frequency_;
}

template <typename T>
const Eigen::VectorXd& Sine<T>::get_phase_vector() const {
  return phase_;
}

template <typename T>
void Sine<T>::CalcValueOutput(const Context<T>& context,
                              BasicVector<T>* output) const {
  VectorX<T> sine_arg;
  if (is_time_based_) {
    VectorX<T> time_vec(amplitude_.size());
    time_vec.fill(context.get_time());
    sine_arg = frequency_.array() * time_vec.array() + phase_.array();
  } else {
    Eigen::VectorBlock<const VectorX<T>> input =
        this->EvalEigenVectorInput(context, 0);
    sine_arg = frequency_.array() * input.array() + phase_.array();
  }

  Eigen::VectorBlock<VectorX<T>> output_block = output->get_mutable_value();
  output_block = amplitude_.array() * sine_arg.array().sin();
}

template <typename T>
void Sine<T>::CalcFirstDerivativeOutput(
    const Context<T>& context, BasicVector<T>* output) const {

  VectorX<T> cos_arg;
  if (is_time_based_) {
    VectorX<T> time_vec(amplitude_.size());
    time_vec.fill(context.get_time());
    cos_arg = frequency_.array() * time_vec.array() + phase_.array();
  } else {
    Eigen::VectorBlock<const VectorX<T>> input =
        this->EvalEigenVectorInput(context, 0);
    cos_arg = frequency_.array() * input.array() + phase_.array();
  }

  Eigen::VectorBlock<VectorX<T>> output_block = output->get_mutable_value();
  output_block =
      amplitude_.array() * frequency_.array() * cos_arg.array().cos();
}

template <typename T>
void Sine<T>::CalcSecondDerivativeOutput(
    const Context<T>& context, BasicVector<T>* output) const {

  VectorX<T> sine_arg;
  if (is_time_based_) {
    VectorX<T> time_vec(amplitude_.size());
    time_vec.fill(context.get_time());
    sine_arg = frequency_.array() * time_vec.array() + phase_.array();
  } else {
    Eigen::VectorBlock<const VectorX<T>> input =
        this->EvalEigenVectorInput(context, 0);
    sine_arg = frequency_.array() * input.array() + phase_.array();
  }

  Eigen::VectorBlock<VectorX<T>> output_block = output->get_mutable_value();
  output_block =
      - amplitude_.array() * frequency_.array().pow(2) * sine_arg.array().sin();
}

}  // namespace systems
}  // namespace drake
