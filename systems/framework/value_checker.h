#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include "drake/common/drake_throw.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/value.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace internal {

/// Checks some BasicVector invariants on @p basic_vector.
///
/// Because this function uses shady implementation tricks, it should ONLY be
/// called from within DRAKE_ASSERT_VOID or unit test code.
///
/// This function is likely to be expensive (on the order of a full copy), so
/// should be used sparingly.  In particular, only a few select locations
/// within the Systems Framework itself should likely call this function.
///
/// @throws std::exception if invariants are violated or basic_vector is nullptr
template <typename T>
void CheckBasicVectorInvariants(const BasicVector<T>* basic_vector) {
  DRAKE_THROW_UNLESS(basic_vector != nullptr);
  std::unique_ptr<BasicVector<T>> cloned_base = basic_vector->Clone();
  const BasicVector<T>* const cloned_vector = cloned_base.get();
  DRAKE_THROW_UNLESS(cloned_vector != nullptr);
  const auto& original_type = typeid(*basic_vector);
  const auto& cloned_type = typeid(*cloned_vector);
  if (original_type != cloned_type) {
    const std::string original_name = NiceTypeName::Get(*basic_vector);
    const std::string cloned_name = NiceTypeName::Get(*cloned_vector);
    throw std::runtime_error(
        "CheckBasicVectorInvariants failed: " + original_name + "::Clone "
        "produced a " + cloned_name + " object instead of the same type");
  }
}

/// If @p abstract_value is a Value<BasicVector<T>>, then checks some
/// BasicVector invariants.  Otherwise, does nothing.
///
/// Because this function uses shady implementation tricks, it should ONLY be
/// called from within DRAKE_ASSERT_VOID or unit test code.
///
/// This function is likely to be expensive (on the order of a full copy), so
/// should be used sparingly.  In particular, only a few select locations
/// within the Systems Framework itself should likely call this function.
///
/// @tparam T the supposed element type of the Value<BasicVector<T>> that has
/// been erased into an AbstractValue
///
/// @throws std::exception if invariants are violated or abstract_value is
/// nullptr
template <typename T>
void CheckVectorValueInvariants(const AbstractValue* abstract_value) {
  DRAKE_THROW_UNLESS(abstract_value != nullptr);
  const auto* const basic_vector =
      abstract_value->maybe_get_value<BasicVector<T>>();
  if (basic_vector != nullptr) {
    // We are a Value<BasicVector<T>>, so check the invariants.
    CheckBasicVectorInvariants<T>(basic_vector);
  }
}

}  // namespace internal
}  // namespace systems
}  // namespace drake
