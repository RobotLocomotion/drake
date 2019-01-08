#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/value.h"

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
class ConstantValueSource : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConstantValueSource)

  /// @param value The constant value to emit which is copied by this system.
  explicit ConstantValueSource(const AbstractValue& value);

  // TODO(jwnimmer-tri) Remove this overload on or about 2018-04-01.
  DRAKE_DEPRECATED(
      "Use the ConstantValueSource(const AbstractValue&) constructor instead")
  explicit ConstantValueSource(std::unique_ptr<AbstractValue> value);

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
