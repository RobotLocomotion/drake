#pragma once

#include <cstdint>
#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// A source block with a constant output port at all times.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class ConstantVectorSource : public System<T> {
 public:
  /// @param source_value the constant value of the output so that
  /// `y = source_value` at all times.
  explicit ConstantVectorSource(
      const Eigen::Ref<const VectorX<T>>& source_value);

  // Allocates the default context with no state.
  std::unique_ptr<ContextBase<T>> CreateDefaultContext() const override;

  // Allocates one output port with a length equal to the size of the
  // @p source_value specified in the constructor.
  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const ContextBase<T>& context) const override;

  /// Outputs a signal with a fixed value as specified by the user.
  void EvalOutput(const ContextBase<T>& context,
                  SystemOutput<T>* output) const override;

 private:
  // TODO(amcastro-tri): move source_value_ to the system's parameters.
  const VectorX<T> source_value_;
};

}  // namespace systems
}  // namespace drake
