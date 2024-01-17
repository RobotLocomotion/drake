#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <iosfwd>
#include <optional>
#include <string>

#include "drake/common/drake_copyable.h"

namespace drake {

/** Represents a SHA-256 cryptographic checksum.
See also https://en.wikipedia.org/wiki/SHA-2.

This class is not bound in pydrake, because Python programmers should prefer
using https://docs.python.org/3/library/hashlib.html instead. */
class Sha256 final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Sha256);

  /** Constructs an all-zero checksum.
  Note that this is NOT the same as the checksum of empty (zero-sized) data. */
  Sha256() { bytes_.fill(0); }

  /** Computes the checksum of the given `data` buffer. */
  static Sha256 Checksum(std::string_view data);

  /** Computes the checksum of the given `stream`.
  Does not check for istream errors; that is the responsibility of the caller.
  @pre stream != nullptr. */
  static Sha256 Checksum(std::istream* stream);

  /** Parses the 64-character ASCII hex representation of a SHA-256 checksum.
  Returns std::nullopt if the argument is an invalid checksum. */
  static std::optional<Sha256> Parse(std::string_view sha256);

  /** Returns the 64-character ASCII hex representation of this checksum. */
  std::string to_string() const;

  // Offer typical comparison operations.
  bool operator==(const Sha256& other) const { return bytes_ == other.bytes_; }
  bool operator!=(const Sha256& other) const { return bytes_ != other.bytes_; }
  bool operator<(const Sha256& other) const { return bytes_ < other.bytes_; }

 private:
  friend class std::hash<Sha256>;

  std::array<uint8_t, 32> bytes_;
};

}  // namespace drake

namespace std {
/** The STL container hash for Sha256 objects. This implementation avoids using
drake::hash_append for performance. The checksum should already be well-mixed,
so we can just squash it down into the required size. */
template <>
struct hash<drake::Sha256> {
  size_t operator()(const drake::Sha256& x) const noexcept {
    constexpr int kInputSize = sizeof(x.bytes_);
    constexpr int kWordSize = sizeof(size_t);
    static_assert(kInputSize % kWordSize == 0);
    constexpr int kNumSteps = kInputSize / kWordSize;
    size_t result = 0;
    for (int i = 0; i < kNumSteps; ++i) {
      // A memcpy is the canonical tool to convert raw memory between different
      // compile-time types -- compilers will always optimize it away in release
      // builds. The whole operator() implementation compiles down to just four
      // instructions (the xor'ing of four 64-bit words).
      size_t temp;
      std::memcpy(&temp, x.bytes_.data() + i * kWordSize, kWordSize);
      result = result ^ temp;
    }
    return result;
  }
};
}  // namespace std
