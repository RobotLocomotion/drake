#include "drake/common/value.h"

#include <fmt/format.h>

namespace drake {

AbstractValue::~AbstractValue() {}

void AbstractValue::ThrowCastError(const std::string& requested_type) const {
  throw std::logic_error(fmt::format(
      "AbstractValue: a request to extract a value of type '{}' failed because "
      "the actual type was '{}'.", requested_type, GetNiceTypeName()));
}

}  // namespace drake
