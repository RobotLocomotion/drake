#pragma once

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace lcm {

/**
 Encodes an LCM message to a series of bytes.
 */
template <typename Message>
std::vector<uint8_t> EncodeLcmMessage(const Message& message) {
  const int num_bytes = message.getEncodedSize();
  DRAKE_THROW_UNLESS(num_bytes >= 0);
  const size_t size_bytes = static_cast<size_t>(num_bytes);
  std::vector<uint8_t> bytes(size_bytes);
  message.encode(bytes.data(), 0, num_bytes);
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
    throw std::runtime_error(
        "Error decoding message of type '" + NiceTypeName::Get<Message>()
        + "'");
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
