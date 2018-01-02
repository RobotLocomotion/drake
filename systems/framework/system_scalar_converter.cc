#include "drake/systems/framework/system_scalar_converter.h"

#include "drake/common/hash.h"

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
  DRAKE_ASSERT(insert_result.second);
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

}  // namespace systems
}  // namespace drake
