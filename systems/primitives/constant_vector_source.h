#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/single_output_vector_source.h"

namespace drake {
namespace systems {

/// A source block with a constant output port at all times. The value of the
/// output port is a parameter of the system (see Parameters).
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// @ingroup primitive_systems
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class ConstantVectorSource final : public SingleOutputVectorSource<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConstantVectorSource)

  /// Constructs a system with a vector output that is constant and equals the
  /// supplied @p source_value at all times.
  /// @param source_value the constant value of the output so that
  /// `y = source_value` at all times.
  explicit ConstantVectorSource(
      const Eigen::Ref<const VectorX<T>>& source_value);

  /// Constructs a system with a scalar-valued output of type T that is constant
  /// and equals the supplied @p source_value at all times.
  /// @param source_value the constant value of the output so that
  /// `y = source_value` at all times.
  explicit ConstantVectorSource(const T& source_value);

  /// Constructs a system with a vector output that is constant, has the type of
  /// the @p source_value, and equals the @p source_value at all times.
  ///
  /// @note Objects created using this constructor overload do not support
  /// system scalar conversion.  See @ref system_scalar_conversion.
  explicit ConstantVectorSource(const BasicVector<T>& source_value);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit ConstantVectorSource(const ConstantVectorSource<U>& other);

  ~ConstantVectorSource() override;

  /// Return a read-only reference to the source value of this block in the
  /// given @p context.
  const BasicVector<T>& get_source_value(const Context<T>& context) const;

  /// Return a mutable reference to the source value of this block in the given
  /// @p context.
  BasicVector<T>& get_mutable_source_value(Context<T>* context);

 private:
  // Allow different specializations to access each other's private data.
  template <typename U> friend class ConstantVectorSource;

  // All other constructor overloads delegate to here.
  ConstantVectorSource(SystemScalarConverter, const BasicVector<T>&);

  // Outputs a signal with a fixed value as specified by the user.
  void DoCalcVectorOutput(
      const Context<T>& context,
      Eigen::VectorBlock<VectorX<T>>* output) const override;

  const int source_value_index_;
};

}  // namespace systems
}  // namespace drake
