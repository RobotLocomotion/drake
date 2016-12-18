#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// This system combines multiple vector-valued inputs into a vector-valued
/// output.  The input to this system directly feeds through to its output.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following `T` values are provided:
/// - double
///
/// They are already available to link against in `libdrakeSystemsFramework`.
/// Currently, no other values for `T` are supported.
///
/// @ingroup primitive_systems
template <typename T>
class Multiplexer : public LeafSystem<T> {
 public:
  /// Constructs a %Multiplexer with `num_scalar_inputs` scalar-valued input
  /// ports, and one vector-valued output port of size `num_scalar_inputs`.
  explicit Multiplexer(int num_scalar_inputs);

  /// Constructs a %Multiplexer with `input_sizes.size()` vector-valued input
  /// ports where the i-th input has size `input_sizes[i]`, and one vector-
  /// valued output port of size `sum(input_sizes)`.
  explicit Multiplexer(std::vector<int> input_sizes);

  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;

  // Non-copyable.
  Multiplexer(const Multiplexer<T>&) = delete;
  Multiplexer& operator=(const Multiplexer<T>&) = delete;

 private:
  const std::vector<int> input_sizes_;
};

}  // namespace systems
}  // namespace drake
