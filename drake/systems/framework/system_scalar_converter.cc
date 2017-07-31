#include "drake/systems/framework/system_scalar_converter.h"

namespace drake {
namespace systems {

namespace {
std::pair<std::type_index, std::type_index> make_key(
    const std::type_info& t_info, const std::type_info& u_info) {
  return std::make_pair(std::type_index(t_info), std::type_index(u_info));
}
}  // namespace

SystemScalarConverter::SystemScalarConverter() = default;

void SystemScalarConverter::Insert(
    const std::type_info& t_info, const std::type_info& u_info,
    const ErasedConverterFunc& converter) {
  const auto& key = make_key(t_info, u_info);
  const auto& insert_result = funcs_.insert({key, converter});
  DRAKE_ASSERT(insert_result.second);
}

const SystemScalarConverter::ErasedConverterFunc* SystemScalarConverter::Find(
    const std::type_info& t_info, const std::type_info& u_info) const {
  const auto& key = make_key(t_info, u_info);
  auto iter = funcs_.find(key);
  if (iter != funcs_.end()) {
    return &(iter->second);
  } else {
    return nullptr;
  }
}

}  // namespace systems
}  // namespace drake
