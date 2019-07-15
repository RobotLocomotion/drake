#pragma once

#include <fmt/format.h>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {
namespace systems {

/// An element-wise gain block with input `u` and output `y = k * u` with `k` a
/// constant vector.  The input to this system directly feeds through to its
/// output.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to https://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following scalar types @p T are provided:
///
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// To use other specific scalar types see gain-inl.h.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// @ingroup primitive_systems
template <typename T>
class Gain final : public VectorSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Gain)

  /// Constructs a %Gain system where the same gain is applied to every input
  /// value.
  ///
  /// @param[in] k the gain constant so that `y = k * u`.
  /// @param[in] size number of elements in the signal to be processed.
  Gain(double k, int size);

  /// Constructs a %Gain system where different gains can be applied to each
  /// input value.
  ///
  /// @param[in] k the gain vector constants so that `y_i = k_i * u_i` where
  /// subscript `i` indicates the i-th element of the vector.
  explicit Gain(const Eigen::VectorXd& k);

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit Gain(const Gain<U>&);

  /// Returns the gain constant. This method should only be called if the gain
  /// can be represented as a scalar value, i.e., every element in the gain
  /// vector is the same. It will throw an exception if the gain cannot be
  /// represented as a single scalar value.
  double get_gain() const;

  /// Returns the gain vector constant.
  const Eigen::VectorXd& get_gain_vector() const;

 private:
  void DoCalcVectorOutput(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const final;

  const Eigen::VectorXd k_;
};

// TODO(amcastro-tri): remove the size parameter from the constructor once
// #3109 supporting automatic sizes is resolved.
template <typename T>
Gain<T>::Gain(double k, int size) : Gain(Eigen::VectorXd::Ones(size) * k) {}

template <typename T>
Gain<T>::Gain(const Eigen::VectorXd& k)
    : VectorSystem<T>(SystemTypeTag<systems::Gain>{}, k.size(), k.size()),
      k_(k) {}

template <typename T>
template <typename U>
Gain<T>::Gain(const Gain<U>& other)
    : Gain<T>(other.get_gain_vector()) {}

template <typename T>
double Gain<T>::get_gain() const {
  if (!k_.isConstant(k_[0])) {
    throw std::runtime_error(fmt::format(
        "The gain vector [{}] cannot be represented as a scalar value. "
        "Please use drake::systems::Gain::get_gain_vector() instead.",
        k_.transpose()));
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

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Gain)
