#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {
namespace systems {

/// An element-wise gain block with input `u` and output `y = k * u` with `k` a
/// constant vector.  The input to this system directly feeds through to its
/// output.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following scalar types @p T are provided:
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
  /// vector is the same. It will abort if the gain cannot be represented as a
  /// single scalar value.
  double get_gain() const;

  /// Returns the gain vector constant.
  const Eigen::VectorXd& get_gain_vector() const;

 protected:
  void DoCalcVectorOutput(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const override;

  const Eigen::VectorXd k_;
};

}  // namespace systems
}  // namespace drake
