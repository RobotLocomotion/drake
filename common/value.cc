#include "drake/common/value.h"

namespace drake {

AbstractValue::~AbstractValue() = default;

std::string AbstractValue::GetNiceTypeName() const {
  return NiceTypeName::Canonicalize(
      NiceTypeName::Demangle(type_info().name()));
}

}  // namespace drake
