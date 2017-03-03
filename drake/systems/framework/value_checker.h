#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include "drake/common/drake_throw.h"
#include "drake/common/nice_type_name.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {
namespace detail {

/// Checks some BasicVector invariants on @p basic_vector.
///
/// Because this function uses shady implementation tricks, it should ONLY be
/// called from within DRAKE_ASSERT_VOID or unit test code.
///
/// This function is likely to be expensive (on the order of a full copy), so
/// should be used sparingly.  In particular, only a few select locations
/// within the Systems Framework itself should likely call this function.
///
/// @throw exception if invariants are violated or abstract_value is nullptr
template <typename T>
void CheckBasicVectorInvariants(const BasicVector<T>* basic_vector) {
  DRAKE_THROW_UNLESS(basic_vector != nullptr);
  std::unique_ptr<BasicVector<T>> cloned_base = basic_vector->Clone();
  const BasicVector<T>* const cloned_vector = cloned_base.get();
  DRAKE_THROW_UNLESS(cloned_vector != nullptr);
  const auto& original_type = typeid(*basic_vector);
  const auto& cloned_type = typeid(*cloned_vector);
  if (original_type != cloned_type) {
    using N = NiceTypeName;
    const std::string original_name =
        N::Canonicalize(N::Demangle(original_type.name()));
    const std::string cloned_name =
        N::Canonicalize(N::Demangle(cloned_type.name()));
    throw std::runtime_error(
        "CheckVectorValueInvariants failed: " + original_name + "::Clone "
        "produced a " + cloned_name + " object instead of the same type");
  }
}

/// If @p abstract_value is a VectorValue<T>, then checks some BasicVector
/// invariants.  If this is not a VectorValue<Scalar>, does nothing.
///
/// Because this function uses shady implementation tricks, it should ONLY be
/// called from within DRAKE_ASSERT_VOID or unit test code.
///
/// This function is likely to be expensive (on the order of a full copy), so
/// should be used sparingly.  In particular, only a few select locations
/// within the Systems Framework itself should likely call this function.
///
/// @tparam T the supposed element type of the VectorValue<T> that has been
/// erased into an AbstractValue
///
/// @throw exception if invariants are violated or abstract_value is nullptr
template <typename T>
void CheckVectorValueInvariants(const AbstractValue* abstract_value) {
  DRAKE_THROW_UNLESS(abstract_value != nullptr);
  const VectorValue<T>* const vector_value =
      dynamic_cast<const VectorValue<T>*>(abstract_value);
  if (vector_value != nullptr) {
    // We are a VectorValue<T>, so check the invariants.
    CheckBasicVectorInvariants<T>(vector_value->get_value());
  }
}

}  // namespace detail
}  // namespace systems
}  // namespace drake
