#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "papa/lima.hpp"

namespace papa {

class november {
 public:
  papa::lima alpha;
  papa::lima bravo;
  int32_t charlie;

  // These functions match the expected API from the legacy lcm-gen tool,
  // but note that we use `int64_t` instead of `int` for byte counts.
  //@{
  static const char* getTypeName() { return "november"; }
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
  //@}

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
  // a host unsigned integer using the matching unsigned integer type. (This
  // is also used to convert host to network order; it's the same operation.)
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

  // The dimensions of an array, for use during encoding / decoding, e.g., for
  // a message field `int8_t image[6][4]` we'd use `ArrayDims<2>{6, 4}`.
  template <size_t ndims>
  using ArrayDims = std::array<int64_t, ndims>;

  // Returns the second and following elements of _dims (i.e., _dims[1:]).
  // https://en.wikipedia.org/wiki/CAR_and_CDR
  template <size_t ndims>
  static ArrayDims<ndims - 1> _cdr(const std::array<int64_t, ndims>& _dims) {
    static_assert(ndims > 0);
    ArrayDims<ndims - 1> _result;
    for (size_t i = 1; i < ndims; ++i) {
      _result[i - 1] = _dims[i];
    }
    return _result;
  }

  // Given a field (or child element within a field), encodes it into the given
  // byte cursor and advances the cursor, returning true on success. Arrays are
  // passed with `_input` as vector-like container and `_dims` as the list of
  // multi-dimensional vector sizes, e.g., `int8_t image[6][4]` would be called
  // like `_encode_field(image.at(0), &cursor, end, ArrayDims<2>{6, 4})`. In
  // LCM messages, multi-dimensional arrays are encoded using C's memory layout
  // (i.e., with the last dimension as the most tightly packed.)
  template <typename T, size_t ndims = 0>
  static bool _encode_field(const T& _input, uint8_t** _cursor, uint8_t* _end,
                            const ArrayDims<ndims>& _dims = ArrayDims<0>{}) {
    static_assert(!std::is_pointer_v<T>);
    if constexpr (ndims == 0) {
      // With no array dimensions, just decode the field directly.
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
      } else if constexpr (std::is_same_v<T, std::string>) {
        // String input.
        const int32_t _size = _input.size() + 1;
        const bool ok = (_input.size() < INT32_MAX) &&
                        (*_cursor + sizeof(_size) + _size <= _end) &&
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
      // Cross-check the container size vs the size specified in the message's
      // size field. (For fixed-size containers this is a no-op.)
      if (static_cast<int64_t>(_input.size()) != _dims[0]) {
        return false;
      }
      // Encode each sub-item in turn, forwarding all the _dims but the first.
      for (const auto& _child : _input) {
        if (!_encode_field(_child, _cursor, _end, _cdr(_dims))) {
          return false;
        }
      }
      return true;
    }
  }

  // Given a pointer to a field (or child element within a field), decodes it
  // from the given byte cursor and advances the cursor, returning true on
  // success. The array `_dims` and storage order follow the same pattern as in
  // _encode_field(); refer to those docs for details.
  template <typename T, size_t ndims = 0>
  static bool _decode_field(T* _output, const uint8_t** _cursor,
                            const uint8_t* _end,
                            const ArrayDims<ndims>& _dims = {}) {
    static_assert(!std::is_pointer_v<T>);
    if constexpr (ndims == 0) {
      // With no array dimensions, just decode the field directly.
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
      } else if constexpr (std::is_same_v<T, std::string>) {
        // String output.
        int32_t _size{};
        const bool ok = _decode_field(&_size, _cursor, _end) &&
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
        _output->resize(_dims[0]);
      }
      // Decode each sub-item in turn.
      for (auto& _child : *_output) {
        if (!_decode_field(&_child, _cursor, _end, _cdr(_dims))) {
          return false;
        }
      }
      return true;
    }
  }
};

}  // namespace papa
