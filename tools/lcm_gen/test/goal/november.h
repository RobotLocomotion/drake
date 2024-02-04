#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace papa {

class november {
 public:
  papa::lima alpha;
  papa::lima bravo;
  int32_t charlie;

  // These functions match the expected API from the legacy lcm-gen tool,
  // but note that we use `int64_t` instead of `int` for byte counts.
  int64_t getEncodedSize() const { return 8 + _getEncodedSizeNoHash(); }
  int64_t _getEncodedSizeNoHash() const {
    int64_t _result = 0;
    _result += alpha._getEncodedSizeNoHash();
    _result += bravo._getEncodedSizeNoHash();
    _result += 4;  // charlie
    return _result;
  }
  template <bool with_hash = true>
  int64_t encode(void* buf, int64_t offset, int64_t maxlen) const {
    uint8_t* const _begin = static_cast<uint8_t*>(buf);
    uint8_t* const _start = _begin + offset;
    uint8_t* const _end = _begin + maxlen;
    uint8_t* _cursor = _start;
    return this->_encode<with_hash>(&_cursor, _end) ? (_cursor - _start) : -1;
  }
  int64_t _encodeNoHash(void* buf, int64_t offset, int64_t maxlen) const {
    return encode<false>(buf, offset, maxlen);
  }
  template <bool with_hash = true>
  int64_t decode(const void* buf, int64_t offset, int64_t maxlen) {
    const uint8_t* const _begin = static_cast<const uint8_t*>(buf);
    const uint8_t* const _start = _begin + offset;
    const uint8_t* const _end = _begin + maxlen;
    const uint8_t* _cursor = _start;
    return this->_decode<with_hash>(&_cursor, _end) ? (_cursor - _start) : -1;
  }
  int64_t _decodeNoHash(const void* buf, int64_t offset, int64_t maxlen) {
    return decode<false>(buf, offset, maxlen);
  }
  static constexpr int64_t getHash() {
    return static_cast<int64_t>(_get_hash_impl());
  }
  template <typename Parents>
  static uint64_t _computeHash(const Parents*) {
    return getHash();
  }

  // New-style (constexpr) hashing.
  template <size_t N = 0>
  static constexpr uint64_t _get_hash_impl(
      const std::array<uint64_t, N>& parents = {}) {
    const uint64_t base_hash = 0x86ad239bfc105cc3ull;
    std::array<uint64_t, N + 1> new_parents{base_hash};
    for (size_t n = 0; n < N; ++n) {
      if (parents[n] == base_hash) {
        // Special case for recursive message definition.
        return 0;
      }
      new_parents[n + 1] = parents[n];
    }
    const uint64_t composite_hash = base_hash
        + papa::lima::_get_hash_impl(new_parents)
        + papa::lima::_get_hash_impl(new_parents);
    return (composite_hash << 1) + ((composite_hash >> 63) & 1);
  }

  // New-style encoding.
  template <bool with_hash = true>
  bool _encode(uint8_t** _cursor, uint8_t* _end) const {
    constexpr int64_t _hash = _get_hash_impl();
    return  // true iff success
        (with_hash ? _encode_field(_hash, _cursor, _end) : true) &&
        _encode_field(alpha, _cursor, _end) &&
        _encode_field(bravo, _cursor, _end) &&
        _encode_field(charlie, _cursor, _end);
  }

  // New-style decoding.
  template <bool with_hash = true>
  bool _decode(const uint8_t** _cursor, const uint8_t* _end) {
    constexpr int64_t _expected_hash = _get_hash_impl();
    int64_t _hash = _expected_hash;
    return  // true iff success
        (with_hash ? _decode_field(&_hash, _cursor, _end) : true) &&
        (_hash == _expected_hash) &&
        _decode_field(&alpha, _cursor, _end) &&
        _decode_field(&bravo, _cursor, _end) &&
        _decode_field(&charlie, _cursor, _end);
  }

 private:
  // Given an N-byte integer at `_input` in network byte order, returns it as
  // a host unsigned integer using the matching unsigned integer type.
  template <size_t N>
  static auto _byteswap(const void* _input) {
    // clang-format off
    using result_t = std::conditional_t<
        N == 1, uint8_t, std::conditional_t<
        N == 2, uint16_t, std::conditional_t<
        N == 4, uint32_t, std::conditional_t<
        N == 8, uint64_t, void>>>>;
    // clang-format on
    result_t _result;
    std::memcpy(&_result, _input, N);
    // TODO(jwnimmer-tri) Don't bswap on PowerPC.
    if constexpr (N == 1) {
      return _result;
    } else if constexpr (N == 2) {
      return __builtin_bswap16(_result);
    } else if constexpr (N == 4) {
      return __builtin_bswap32(_result);
    } else if constexpr (N == 8) {
      return __builtin_bswap64(_result);
    }
  }

  // Given a field (or child element within a field), encodes it into the given
  // byte cursor and advances the cursor, returning true on success.
  template <typename T>
  static bool _encode_field(const T& _input, uint8_t** _cursor,
                            uint8_t* _end) {
    static_assert(!std::is_pointer_v<T>);
    if constexpr (std::is_fundamental_v<T>) {
      // POD input.
      constexpr size_t N = sizeof(T);
      if (*_cursor + N > _end) {
        return false;
      }
      auto _swapped = _byteswap<N>(&_input);
      std::memcpy(*_cursor, &_swapped, N);
      *_cursor += N;
      return true;
    } else {
      // Struct input.
      return _input.template _encode<false>(_cursor, _end);
    }
  }

  // Given a pointer to a field (or child element within a field), decodes it
  // from the given byte cursor and advances the cursor, returning true on
  // success.
  template <typename T>
  static bool _decode_field(T* _output, const uint8_t** _cursor,
                            const uint8_t* _end) {
    static_assert(!std::is_pointer_v<T>);
    if constexpr (std::is_fundamental_v<T>) {
      // POD output.
      constexpr size_t N = sizeof(T);
      if (*_cursor + N > _end) {
        return false;
      }
      auto _swapped = _byteswap<N>(*_cursor);
      std::memcpy(_output, &_swapped, N);
      *_cursor += N;
      return true;
    } else {
      // Struct output.
      return _output->template _decode<false>(_cursor, _end);
    }
  }
};

}  // namespace papa
