#pragma once

/// @file
/// Template method implementations for sine.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include <sstream>

#include "drake/common/unused.h"
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
    : VectorSystem<T>(SystemTypeTag<systems::Sine>{},
                      amplitudes.size() * !is_time_based,
                      amplitudes.size()),
      amplitude_(amplitudes), frequency_(frequencies), phase_(phases),
      is_time_based_(is_time_based) {}

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
void Sine<T>::DoCalcVectorOutput(
    const Context<T>& context,
    const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* output) const {
  unused(state);

  VectorX<T> sine_arg;
  if (is_time_based_) {
      VectorX<T> time_vec(amplitude_.size());
      time_vec.fill(context.get_time());
      sine_arg = frequency_.array() * time_vec.array() + phase_.array();
  } else {
      sine_arg = frequency_.array() * input.array() + phase_.array();
  }

  *output = amplitude_.array() * sine_arg.array().sin();
}

}  // namespace systems
}  // namespace drake
