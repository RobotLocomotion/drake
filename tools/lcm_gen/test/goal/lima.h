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

class lima {
 public:
  static constexpr double charlie_delta = 3.25e1;
  static constexpr float charlie_foxtrot = 4.5e2;
  static constexpr int8_t charlie_india8 = 8;
  static constexpr int16_t charlie_india16 = 16;
  static constexpr int32_t charlie_india32 = 32;
  static constexpr int64_t charlie_india64 = 64;

  bool golf;
  uint8_t bravo;
  double delta;
  float foxtrot;
  int8_t india8;
  int16_t india16;
  int32_t india32;
  int64_t india64;

  // These functions match the expected API from the legacy lcm-gen tool.
  // Note that we use `int64_t` instead of `int` for byte counts.
  int64_t getEncodedSize() const { return 8 + _getEncodedSizeNoHash(); }
  int64_t _getEncodedSizeNoHash() const {
    int64_t _result = 0;
    _result += 1;  // golf
    _result += 1;  // bravo
    _result += 8;  // delta
    _result += 4;  // foxtrot
    _result += 1;  // india8
    _result += 2;  // india16
    _result += 4;  // india32
    _result += 8;  // india64
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
    const uint64_t base_hash = 0x35fef8dfc801b95eull;
    std::array<uint64_t, N + 1> new_parents{base_hash};
    for (size_t n = 0; n < N; ++n) {
      if (parents[n] == base_hash) {
        return 0;
      }
      new_parents[n + 1] = parents[n];
    }
    const uint64_t composite_hash = base_hash;
    return (composite_hash << 1) + ((composite_hash >> 63) & 1);
  }

  // New-style encoding.
  template <bool with_hash = true>
  bool _encode(uint8_t** _cursor, uint8_t* _end) const {
    constexpr int64_t _hash = getHash();
    constexpr int64_t _hash_size = with_hash ? 8 : 0;
    return  // true iff success
        (*_cursor + _hash_size + 1 + 1 + 8 + 4 + 1 + 2 + 4 + 8 <= _end) &&
        (with_hash ? _encode_field(_hash, _cursor, _end) : true) &&
        _encode_field(golf, _cursor, _end) &&
        _encode_field(bravo, _cursor, _end) &&
        _encode_field(delta, _cursor, _end) &&
        _encode_field(foxtrot, _cursor, _end) &&
        _encode_field(india8, _cursor, _end) &&
        _encode_field(india16, _cursor, _end) &&
        _encode_field(india32, _cursor, _end) &&
        _encode_field(india64, _cursor, _end);
  }

  // New-style decoding.
  template <bool with_hash = true>
  bool _decode(const uint8_t** _cursor, const uint8_t* _end) {
    constexpr int64_t _expected_hash = getHash();
    constexpr int64_t _hash_size = with_hash ? 8 : 0;
    int64_t _hash = _expected_hash;
    return  // true iff success
        (*_cursor + _hash_size + 1 + 1 + 8 + 4 + 1 + 2 + 4 + 8 <= _end) &&
        (with_hash ? _decode_field(&_hash, _cursor, _end) : true) &&
        (_hash == _expected_hash) &&
        _decode_field(&golf, _cursor, _end) &&
        _decode_field(&bravo, _cursor, _end) &&
        _decode_field(&delta, _cursor, _end) &&
        _decode_field(&foxtrot, _cursor, _end) &&
        _decode_field(&india8, _cursor, _end) &&
        _decode_field(&india16, _cursor, _end) &&
        _decode_field(&india32, _cursor, _end) &&
        _decode_field(&india64, _cursor, _end);
  }

 private:
  // Given N bytes at `_input`, returns a std::array in network byte order.
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
  template <typename T, typename... Ints>
  static bool _encode_field(const T& _input, uint8_t** _cursor, uint8_t* _end,
                            Ints... _dims) {
    static_assert(!std::is_pointer_v<T>);
    if constexpr (sizeof...(Ints) == 0) {
      // With no array dimensions, just decode the field directly.
      if constexpr (std::is_fundamental_v<T>) {
        // POD input.
        constexpr size_t N = sizeof(T);
        auto _swapped = _byteswap<N>(&_input);
        std::memcpy(*_cursor, &_swapped, N);
        *_cursor += N;
        return true;
      } else if constexpr (std::is_same_v<T, std::string>) {
        // String input.
        const int32_t _size = _input->size() + 1;
        const bool ok = (_input->size() < INT32_MAX) &&
                        (*_cursor + 4 + _size <= _end) &&
                        _encode_field(_size, _cursor, _end);
        if (ok) {
          std::memcpy(*_cursor, _input->c_str(), _size);
        }
        *_cursor += _size;
        return ok;
      } else {
        // Struct input.
        return _input.template _encode<false>(_cursor, _end);
      }
    } else {
      // In case of a variable-size dimension, cross-check vs the size field.
      const int64_t _dim = std::get<0>(std::make_tuple(_dims...));
      if (static_cast<int64_t>(_input.size()) != _dim) {
        return false;
      }
      // Encode each sub-item in turn.
      for (const auto& _child : _input) {
        const bool ok = [&](int /* _dim */, auto&&... _child_dims) {
          return _encode_field(_child, _cursor, _end, _child_dims...);
        }(_dims...);
        if (!ok) {
          return false;
        }
      }
      return true;
    }
  }

  // Given a pointer to a field (or child element within a field), decodes it
  // from the given byte cursor and advances the cursor, returning true on
  // success.
  template <typename T, typename... Ints>
  static bool _decode_field(T* _output, const uint8_t** _cursor,
                            const uint8_t* _end, Ints... _dims) {
    static_assert(!std::is_pointer_v<T>);
    if constexpr (sizeof...(Ints) == 0) {
      // With no array dimensions, just decode the field directly.
      if constexpr (std::is_fundamental_v<T>) {
        // POD output.
        constexpr size_t N = sizeof(T);
        auto _swapped = _byteswap<N>(*_cursor);
        std::memcpy(_output, &_swapped, N);
        *_cursor += N;
        // Overflow checking is the responsibility of the top-level decode.
        (void)(_end);
        return true;
      } else if constexpr (std::is_same_v<T, std::string>) {
        // String output.
        int32_t _size{};
        const bool ok = (*_cursor + 4 <= _end) &&
                        _decode_field(&_size, _cursor, _end) &&
                        (_size > 0) && (*_cursor + _size <= _end);
        if (ok) {
          _output->replace(_output->begin(), _output->end(), *_cursor,
                           *_cursor + _size - 1);
        }
        *_cursor += _size;
        return ok;
      } else {
        // Struct output.
        return _output->template _decode<false>(_cursor, _end);
      }
    } else {
      // In case of a variable-size dimension, resize our storage to match.
      if constexpr (std::is_same_v<T, std::vector<typename T::value_type>>) {
        const int64_t _dim = std::get<0>(std::make_tuple(_dims...));
        _output->resize(_dim);
      }
      // Decode each sub-item in turn.
      for (auto& _child : *_output) {
        const bool ok = [&](int /* _dim */, auto&&... _child_dims) {
          return _decode_field(&_child, _cursor, _end, _child_dims...);
        }(_dims...);
        if (!ok) {
          return false;
        }
      }
      return true;
    }
  }
};

}  // namespace papa
