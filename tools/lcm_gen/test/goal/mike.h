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

class mike {
 public:
  std::array<double, 3> delta;
  std::array<std::array<float, 5>, 4> foxtrot;
  papa::lima alpha;
  std::string sierra;
  int32_t rows;
  int32_t cols;
  std::vector<uint8_t> bravo;
  std::vector<std::vector<int8_t>> india8;
  std::array<std::vector<int16_t>, 7> india16;
  std::vector<std::array<int32_t, 11>> india32;
  std::array<papa::lima, 2> xray;
  std::vector<papa::lima> yankee;
  std::vector<std::array<papa::lima, 2>> zulu;

  // These functions match the expected API from the legacy lcm-gen tool,
  // but note that we use `int64_t` instead of `int` for byte counts.
  int64_t getEncodedSize() const { return 8 + _getEncodedSizeNoHash(); }
  int64_t _getEncodedSizeNoHash() const {
    int64_t _result = 0;
    if (rows < 0) {
      return _result;
    }
    if (cols < 0) {
      return _result;
    }
    _result += 8 * 3;  // delta
    _result += 4 * 4 * 5;  // foxtrot
    _result += alpha._getEncodedSizeNoHash();
    _result += 4 + sierra.size() + 1;
    _result += 4;  // rows
    _result += 4;  // cols
    _result += 1 * rows;  // bravo
    _result += 1 * rows * cols;  // india8
    _result += 2 * 7 * cols;  // india16
    _result += 4 * rows * 11;  // india32
    for (const auto& _xray_0 : xray) {
      _result += _xray_0._getEncodedSizeNoHash();
    }
    for (const auto& _yankee_0 : yankee) {
      _result += _yankee_0._getEncodedSizeNoHash();
    }
    for (const auto& _zulu_0 : zulu) {
      for (const auto& _zulu_1 : _zulu_0) {
        _result += _zulu_1._getEncodedSizeNoHash();
      }
    }
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
    const uint64_t base_hash = 0xd2dc16c61113f6b3ull;
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
        + papa::lima::_get_hash_impl(new_parents)
        + papa::lima::_get_hash_impl(new_parents)
        + papa::lima::_get_hash_impl(new_parents);
    return (composite_hash << 1) + ((composite_hash >> 63) & 1);
  }

  // New-style encoding.
  template <bool with_hash = true>
  bool _encode(uint8_t** _cursor, uint8_t* _end) const {
    constexpr int64_t _hash = _get_hash_impl();
    constexpr int64_t _hash_size = with_hash ? 8 : 0;
    return  // true iff success
        (rows >= 0) &&
        (cols >= 0) &&
        (*_cursor + _hash_size + 8 * 3 + 4 * 4 * 5 <= _end) &&
        (with_hash ? _encode_field(_hash, _cursor, _end) : true) &&
        _encode_field(delta, _cursor, _end, 3) &&
        _encode_field(foxtrot, _cursor, _end, 4, 5) &&
        _encode_field(alpha, _cursor, _end) &&
        _encode_field(sierra, _cursor, _end) &&
        (*_cursor + 4 + 4 + 1 * rows + 1 * rows * cols + 2 * 7 * cols + 4 * rows * 11 <= _end) &&
        _encode_field(rows, _cursor, _end) &&
        _encode_field(cols, _cursor, _end) &&
        _encode_field(bravo, _cursor, _end, rows) &&
        _encode_field(india8, _cursor, _end, rows, cols) &&
        _encode_field(india16, _cursor, _end, 7, cols) &&
        _encode_field(india32, _cursor, _end, rows, 11) &&
        _encode_field(xray, _cursor, _end, 2) &&
        _encode_field(yankee, _cursor, _end, rows) &&
        _encode_field(zulu, _cursor, _end, rows, 2);
  }

  // New-style decoding.
  template <bool with_hash = true>
  bool _decode(const uint8_t** _cursor, const uint8_t* _end) {
    constexpr int64_t _expected_hash = _get_hash_impl();
    constexpr int64_t _hash_size = with_hash ? 8 : 0;
    int64_t _hash = _expected_hash;
    return  // true iff success
        (*_cursor + _hash_size + 8 * 3 + 4 * 4 * 5 <= _end) &&
        (with_hash ? _decode_field(&_hash, _cursor, _end) : true) &&
        (_hash == _expected_hash) &&
        _decode_field(&delta, _cursor, _end, 3) &&
        _decode_field(&foxtrot, _cursor, _end, 4, 5) &&
        _decode_field(&alpha, _cursor, _end) &&
        _decode_field(&sierra, _cursor, _end) &&
        (*_cursor + 4 + 4 <= _end) &&
        _decode_field(&rows, _cursor, _end) &&
        (rows >= 0) &&
        _decode_field(&cols, _cursor, _end) &&
        (cols >= 0) &&
        (*_cursor + 1 * rows + 1 * rows * cols + 2 * 7 * cols + 4 * rows * 11 <= _end) &&
        _decode_field(&bravo, _cursor, _end, rows) &&
        _decode_field(&india8, _cursor, _end, rows, cols) &&
        _decode_field(&india16, _cursor, _end, 7, cols) &&
        _decode_field(&india32, _cursor, _end, rows, 11) &&
        _decode_field(&xray, _cursor, _end, 2) &&
        _decode_field(&yankee, _cursor, _end, rows) &&
        _decode_field(&zulu, _cursor, _end, rows, 2);
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
        const int32_t _size = _input.size() + 1;
        const bool ok = (_input.size() < INT32_MAX) &&
                        (*_cursor + 4 + _size <= _end) &&
                        _encode_field(_size, _cursor, _end);
        if (ok) {
          std::memcpy(*_cursor, _input.c_str(), _size);
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
