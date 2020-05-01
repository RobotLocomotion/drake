#pragma once

/**
@file
(Advanced) Provides the ability to override NiceTypeName::Get(T*) so that
Python objects can have human-readable names.
*/

#include <functional>
#include <string>
#include <typeinfo>

namespace drake {
namespace internal {

// Structure for passing a type-erased pointer with RTTI.
struct type_erased_ptr {
  const void* raw{};
  const std::type_info& info;
};

// Callback for overriding an object's nice type name.
using NiceTypeNamePtrOverride =
    std::function<std::string(const type_erased_ptr&)>;

// Sets override for nice type names. This can only ever be set once, and
// must be given a non-empty function<> object.
void SetNiceTypeNamePtrOverride(NiceTypeNamePtrOverride new_ptr_override);

// Gets the override. If unset, it will be an empty function<> object.
const NiceTypeNamePtrOverride& GetNiceTypeNamePtrOverride();

}  // namespace internal
}  // namespace drake
