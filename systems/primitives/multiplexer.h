#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// This system combines multiple vector-valued inputs into a vector-valued
/// output.  The input to this system directly feeds through to its output.
///
/// @tparam_default_scalar
/// @ingroup primitive_systems
template <typename T>
class Multiplexer : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Multiplexer)

  /// Constructs a %Multiplexer with `num_scalar_inputs` scalar-valued input
  /// ports, and one vector-valued output port of size `num_scalar_inputs`.
  explicit Multiplexer(int num_scalar_inputs);

  /// Constructs a %Multiplexer with `input_sizes.size()` vector-valued input
  /// ports where the i-th input has size `input_sizes[i]`, and one vector-
  /// valued output port of size `sum(input_sizes)`.
  explicit Multiplexer(std::vector<int> input_sizes);

  /// Constructs a %Multiplexer with model_vector.size() scalar-valued inputs
  /// and one vector-valued output port whose size equals the size of
  /// `model_vector`.  In addition, the output type derives from that of
  /// `model_vector`.
  ///
  /// @note Objects created using this constructor overload do not support
  /// system scalar conversion.  See @ref system_scalar_conversion.
  explicit Multiplexer(const systems::BasicVector<T>& model_vector);

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit Multiplexer(const Multiplexer<U>&);

 private:
  template <typename> friend class Multiplexer;

  // All other constructors delegate to here.
  Multiplexer(SystemScalarConverter converter, std::vector<int> input_sizes,
              const systems::BasicVector<T>& model_vector);

  // This is the calculator for the output port.
  void CombineInputsToOutput(const Context<T>& context,
                             BasicVector<T>* output) const;

  const std::vector<int> input_sizes_;
};

}  // namespace systems
}  // namespace drake
