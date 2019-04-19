#pragma once

#include <memory>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A source block that always outputs a constant value.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// @ingroup primitive_systems
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to https://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
///
/// They are already available to link against in the containing library.
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

template <typename T>
ConstantValueSource<T>::ConstantValueSource(const AbstractValue& value)
    : LeafSystem<T>(SystemTypeTag<systems::ConstantValueSource>{}),
      source_value_(value.Clone()) {
  // Use the "advanced" method to provide explicit non-member functors here
  // since we already have AbstractValues.
  this->DeclareAbstractOutputPort(
      [this]() {
        return source_value_->Clone();
      },
      [this](const Context<T>&, AbstractValue* output) {
        output->SetFrom(*source_value_);
      });
}

template <typename T>
template <typename U>
ConstantValueSource<T>::ConstantValueSource(const ConstantValueSource<U>& other)
    : ConstantValueSource<T>(*other.source_value_) {}

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::ConstantValueSource)
