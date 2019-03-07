#include "drake/common/value.h"

#include <fmt/format.h>

namespace drake {

AbstractValue::~AbstractValue() = default;

std::string AbstractValue::GetNiceTypeName() const {
  return NiceTypeName::Canonicalize(
      NiceTypeName::Demangle(type_info().name()));
}

void AbstractValue::ThrowCastError(const std::string& requested_type) const {
  throw std::logic_error(fmt::format(
      "AbstractValue: a request to cast to '{}' failed because "
      "the actual type was '{}'.", requested_type, GetNiceTypeName()));
}

}  // namespace drake
