#pragma once

#include <cstdint>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace lcm {

namespace internal {
[[noreturn]] void ThrowLcmEncodeDecodeError(const char* operation,
                                            const std::type_info& message_type);
}  // namespace internal

/**
 Encodes an LCM message to a series of bytes.
 */
template <typename Message>
std::vector<uint8_t> EncodeLcmMessage(const Message& message) {
  const int64_t num_bytes = message.getEncodedSize();
  DRAKE_THROW_UNLESS(num_bytes >= 0);
  std::vector<uint8_t> bytes(num_bytes);
  const int64_t used_bytes = message.encode(bytes.data(), 0, num_bytes);
  if (used_bytes != num_bytes) {
    // Any error here would be quite atypical: it would most likly indicate a
    // bug in the generated code for the message, rather than a user error.
    internal::ThrowLcmEncodeDecodeError("encoding", typeid(Message));
  }
  return bytes;
}

/**
 Decodes an LCM message from a series of bytes.
 @throws std::exception if decoding fails.
 */
template <typename Message>
Message DecodeLcmMessage(const std::vector<uint8_t>& bytes) {
  Message message{};
  const size_t size_decoded = message.decode(bytes.data(), 0, bytes.size());
  if (size_decoded != bytes.size()) {
    internal::ThrowLcmEncodeDecodeError("decoding", typeid(Message));
  }
  return message;
}

/**
 Compares two LCM messages of the same type to see if they are equal.
 */
template <typename Message>
bool AreLcmMessagesEqual(const Message& a, const Message& b) {
  return EncodeLcmMessage(a) == EncodeLcmMessage(b);
}

}  // namespace lcm
}  // namespace drake
