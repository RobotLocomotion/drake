#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A source block that always outputs a constant value.
///
/// @tparam_default_scalar
/// @ingroup primitive_systems
template <typename T>
class ConstantValueSource final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConstantValueSource)

  /// @param value The constant value to emit which is copied by this system.
  explicit ConstantValueSource(const AbstractValue& value);

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit ConstantValueSource(const ConstantValueSource<U>&);

 private:
  template <typename> friend class ConstantValueSource;

  // TODO(david-german-tri): move source_value_ to the system's parameters.
  const std::unique_ptr<AbstractValue> source_value_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::ConstantValueSource)
