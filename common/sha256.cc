#include "drake/common/sha256.h"

#include <istream>
#include <iterator>
#include <utility>

#include <picosha2.h>

#include "drake/common/drake_assert.h"

namespace drake {

Sha256 Sha256::Checksum(std::string_view data) {
  Sha256 result;
  static_assert(sizeof(bytes_) == picosha2::k_digest_size);
  picosha2::hash256(data, result.bytes_);
  return result;
}

Sha256 Sha256::Checksum(std::istream* stream) {
  DRAKE_THROW_UNLESS(stream != nullptr);
  Sha256 result;
  static_assert(sizeof(bytes_) == picosha2::k_digest_size);
  picosha2::hash256(std::istreambuf_iterator<char>(*stream),
                    std::istreambuf_iterator<char>(), result.bytes_);
  return result;
}

namespace {

/* Converts a hex char like '4' to its integer value 4.
On error, overwrites `success` with `false`; otherwise, leaves it unchanged. */
uint8_t ParseNibble(char nibble, bool* success) {
  DRAKE_ASSERT(success != nullptr);
  if (nibble >= '0' && nibble <= '9') {
    return nibble - '0';
  }
  if (nibble >= 'a' && nibble <= 'f') {
    return nibble - 'a' + 10;
  }
  if (nibble >= 'A' && nibble <= 'F') {
    return nibble - 'A' + 10;
  }
  *success = false;
  return 0;
}

/* Converts a two-digit hex string like "a0" to its integer value 0xa0.
On error, overwrites `success` with `false`; otherwise, leaves it unchanged. */
uint8_t ParseByte(std::string_view input, bool* success) {
  const uint8_t hi = ParseNibble(input[0], success);
  const uint8_t lo = ParseNibble(input[1], success);
  return (hi << 4) | lo;
}
}  // namespace

std::optional<Sha256> Sha256::Parse(std::string_view sha256) {
  std::optional<Sha256> result;
  constexpr int kNumBytes = sizeof(result->bytes_);
  constexpr int kNumHexDigits = kNumBytes * 2;
  if (sha256.size() == kNumHexDigits) {
    result.emplace();
    bool success = true;
    for (size_t i = 0; i < kNumBytes; ++i) {
      const size_t offset = 2 * i;
      result->bytes_[i] =
          ParseByte(sha256.substr(offset, offset + 2), &success);
    }
    if (!success) {
      result.reset();
    }
  }
  return result;
}

std::string Sha256::to_string() const {
  return picosha2::bytes_to_hex_string(bytes_);
}

}  // namespace drake
