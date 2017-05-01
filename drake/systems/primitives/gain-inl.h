#pragma once

/// @file
/// Template method implementations for gain.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/primitives/gain.h"

#include <sstream>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

namespace drake {
namespace systems {

// TODO(amcastro-tri): remove the size parameter from the constructor once
// #3109 supporting automatic sizes is resolved.
template <typename T>
Gain<T>::Gain(double k, int size) : Gain(Eigen::VectorXd::Ones(size) * k) {}

template <typename T>
Gain<T>::Gain(const Eigen::VectorXd& k)
    : SisoVectorSystem<T>(k.size(), k.size()), k_(k) {}

template <typename T>
double Gain<T>::get_gain() const {
  if (!k_.isConstant(k_[0])) {
    std::stringstream s;
    s << "The gain vector, [" << k_ << "], cannot be represented as a scalar "
      << "value. Please use drake::systems::Gain::get_gain_vector() instead.";
    throw std::runtime_error(s.str().c_str());
  }
  return k_[0];
}

template <typename T>
const Eigen::VectorXd& Gain<T>::get_gain_vector() const {
  return k_;
}

template <typename T>
void Gain<T>::DoCalcVectorOutput(
    const Context<T>&,
    const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* output) const {
  unused(state);
  *output = k_.array() * input.array();
}

template <typename T>
Gain<symbolic::Expression>* Gain<T>::DoToSymbolic() const {
  return new Gain<symbolic::Expression>(k_);
}

}  // namespace systems
}  // namespace drake
