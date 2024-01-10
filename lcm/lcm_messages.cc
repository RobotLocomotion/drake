#include "drake/lcm/lcm_messages.h"

#include <stdexcept>

#include <fmt/format.h>

namespace drake {
namespace lcm {
namespace internal {

void ThrowLcmEncodeDecodeError(const char* operation,
                               const std::type_info& message_type) {
  throw std::runtime_error(fmt::format("Error {} message of type '{}'",
                                       operation,
                                       NiceTypeName::Get(message_type)));
}

}  // namespace internal
}  // namespace lcm
}  // namespace drake
