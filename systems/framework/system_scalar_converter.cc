#include "drake/systems/framework/system_scalar_converter.h"

#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/default_scalars.h"
#include "drake/common/hash.h"
#include "drake/common/nice_type_name.h"

using std::pair;
using std::type_index;
using std::type_info;

namespace drake {
namespace systems {

SystemScalarConverter::Key::Key(
    const type_info& t_info, const type_info& u_info)
    : pair<type_index, type_index>(t_info, u_info) {}

size_t SystemScalarConverter::KeyHasher::operator()(const Key& key) const {
  drake::DefaultHasher hasher;
  using drake::hash_append;
  hash_append(hasher, std::hash<std::type_index>{}(key.first));
  hash_append(hasher, std::hash<std::type_index>{}(key.second));
  return static_cast<size_t>(hasher);
}

SystemScalarConverter::SystemScalarConverter() = default;

void SystemScalarConverter::Insert(
    const std::type_info& t_info, const std::type_info& u_info,
    const ErasedConverterFunc& converter) {
  const auto& key = Key{t_info, u_info};
  const auto& insert_result = funcs_.insert({key, converter});
  DRAKE_DEMAND(insert_result.second);
}

template <typename T, typename U>
void SystemScalarConverter::Remove() {
  funcs_.erase(Key(typeid(T), typeid(U)));
}

template <typename T, typename U>
bool SystemScalarConverter::IsConvertible() const {
  return IsConvertible(typeid(T), typeid(U));
}

bool SystemScalarConverter::IsConvertible(
    const std::type_info& t_info, const std::type_info& u_info) const {
  const auto* converter = Find(t_info, u_info);
  return (converter != nullptr);
}

const SystemScalarConverter::ErasedConverterFunc* SystemScalarConverter::Find(
    const std::type_info& t_info, const std::type_info& u_info) const {
  const auto& key = Key{t_info, u_info};
  auto iter = funcs_.find(key);
  if (iter != funcs_.end()) {
    return &(iter->second);
  } else {
    return nullptr;
  }
}

void SystemScalarConverter::RemoveUnlessAlsoSupportedBy(
    const SystemScalarConverter& other) {
  // Remove the items from `funcs_` whose key is absent from `other`.
  // (This would use erase_if, if we had it.)
  for (auto iter = funcs_.begin(); iter != funcs_.end(); ) {
    const Key& our_key = iter->first;
    if (other.funcs_.count(our_key) == 0) {
      iter = funcs_.erase(iter);
    } else {
      ++iter;
    }
  }
}

namespace system_scalar_converter_internal {

void ThrowConversionMismatch(
    const type_info& s_t_info, const type_info& s_u_info,
    const type_info& other_info) {
  throw std::runtime_error(fmt::format(
      "SystemScalarConverter was configured to convert a {} into a {}"
      " but was called with a {} at runtime",
      NiceTypeName::Get(s_u_info), NiceTypeName::Get(s_t_info),
      NiceTypeName::Get(other_info)));
}

template <typename T, typename U>
void AddPydrakeConverterFunction(
    SystemScalarConverter* converter,
    const std::function<System<T>* (const System<U>&)>& func) {
  DRAKE_DEMAND(converter != nullptr);
  DRAKE_DEMAND(func != nullptr);
  // Copy `func` into a lambda that ends up stored into `funcs_`.  The lambda
  // is typed as `void* => void*` in order to have a non-templated signature
  // and thus fit into a homogeneously-typed std::unordered_map.
  converter->Insert(typeid(T), typeid(U), [func](const void* const bare_u) {
    DRAKE_DEMAND(bare_u != nullptr);
    const System<U>& other = *static_cast<const System<U>*>(bare_u);
    // N.B. This returns a bare pointer, whose ownership is transferred to
    // the caller. We can't use unique_ptr to denote that, though, because
    // that would introduce a include file dependency cycle with System<T>'s
    // destructor's declaration.
    return func(other);
  });
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &AddPydrakeConverterFunction<T, U>
))

}  // namespace system_scalar_converter_internal

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &SystemScalarConverter::IsConvertible<T, U>,
    &SystemScalarConverter::Remove<T, U>
))

}  // namespace systems
}  // namespace drake
