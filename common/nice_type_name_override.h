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
  bool is_polymorphic{};
};

// Callback for overriding an object's nice type name.
using NiceTypeNamePtrOverride =
    std::function<std::string(const type_erased_ptr&)>;

// Sets override for nice type names. This can only ever be set once, and
// must be given a non-empty function<> object.
void SetNiceTypeNamePtrOverride(NiceTypeNamePtrOverride new_ptr_override);

// Gets the override. If unset, it will be an empty function<> object.
const NiceTypeNamePtrOverride& GetNiceTypeNamePtrOverride();

// Adds an entry to the (global) type alias dictionary for use by pydrake
// bindings during the NiceTypeNamePtrOverride callback (for details, see
// bindings/pydrake/common/module_py.cc). The `bound_type` is retained by
// the dictionary and must survive indefinitely.
void AddTypeInfoAlias(const std::type_info& alias_type,
                      const ::std::type_info* bound_type);

// If `query` was previously passed as an `alias_type` to AddTypeInfoAlias,
// then returns its corresponding `bound_type`. Otherwise, returns `query`.
// The `query` must not be null, and the return value is never null.
const std::type_info* GetTypeInfoAlias(const std::type_info* query);

}  // namespace internal
}  // namespace drake
