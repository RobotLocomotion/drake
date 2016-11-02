#pragma once

#include <cstdint>
#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// A source block with a constant output port at all times.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// @ingroup primitive_systems
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
template <typename T>
class ConstantVectorSource : public LeafSystem<T> {
 public:
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

  /// Outputs a signal with a fixed value as specified by the user.
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;

  /// Returns the output port to the constant source.
  const SystemPortDescriptor<T>& get_output_port() const;

 private:
  // TODO(amcastro-tri): move source_value_ to the system's parameters.
  const VectorX<T> source_value_;
};

}  // namespace systems
}  // namespace drake
