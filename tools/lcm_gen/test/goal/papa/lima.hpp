#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace papa {

class lima {
 public:
  static constexpr double charlie_delta = 3.25e1;
  static constexpr float charlie_foxtrot = 4.5e2;
  static constexpr int8_t charlie_india8 = -8;
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

  // These functions match the expected API from the legacy lcm-gen tool,
  // but note that we use `int64_t` instead of `int` for byte counts.
  //@{
  static const char* getTypeName() { return "lima"; }
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
    uint8_t* const _end = _start + maxlen;
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
    const uint8_t* const _end = _start + maxlen;
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
    const uint64_t base_hash = 0x35fef8dfc801b95eull;
    for (size_t n = 0; n < N; ++n) {
      if (parents[n] == base_hash) {
        // Special case for recursive message definition.
        return 0;
      }
    }
    const uint64_t composite_hash = base_hash;
    return (composite_hash << 1) + ((composite_hash >> 63) & 1);
  }

  // New-style encoding.
  template <bool with_hash = true>
  bool _encode(uint8_t** _cursor, uint8_t* _end) const {
    constexpr int64_t _hash = _get_hash_impl();
    return  // true iff success
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
    constexpr int64_t _expected_hash = _get_hash_impl();
    int64_t _hash = _expected_hash;
    return  // true iff success
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

  // Returns true iff T has a "slab" layout in memory, where all of its data
  // lives in one block of contiguous memory. The template arguments are the
  // same as _encode_field().
  template <typename T, int ndims>
  static constexpr bool _is_slab() {
    if constexpr (ndims == 0) {
      return false;
    } else {
      using Element = typename T::value_type;
      if constexpr (!std::is_trivial_v<Element>) {
        return false;
      } else if constexpr (ndims == 1) {
        return std::is_fundamental_v<Element>;
      } else {
        return _is_slab<Element, ndims - 1>();
      }
    }
  }

  // Returns the size of the ndims'th element type of the given containter T.
  // (This tells us how big of a byteswap we'll need while copying T's slab.)
  template <typename T, int ndims>
  static constexpr size_t _get_slab_step() {
    if constexpr (ndims > 0) {
      return _get_slab_step<typename T::value_type, ndims - 1>();
    }
    return sizeof(T);
  }

  // Copies _bytes amount of data from _src to _dst. While copying, each group
  // of N bytes is byteswapped. The number of _bytes must be a multiple of N.
  template <size_t N>
  static void _memcpy_byteswap(void* _dst, const void* _src, size_t _bytes) {
    if constexpr (N == 1) {
      if (_bytes > 0) [[likely]] {
        std::memcpy(_dst, _src, _bytes);
      }
    } else {
      for (size_t _i = 0; _i < _bytes; _i += N) {
        auto _swapped = _byteswap<N>(_src);
        std::memcpy(_dst, &_swapped, N);
        _dst = static_cast<uint8_t*>(_dst) + N;
        _src = static_cast<const uint8_t*>(_src) + N;
      }
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
      if constexpr (_is_slab<T, ndims>()){
        // Encode a slab of POD memory.
        const size_t _raw_size = _input.size() * sizeof(_input[0]);
        if ((*_cursor + _raw_size) > _end) {
          return false;
        }
        constexpr size_t N = _get_slab_step<T, ndims>();
        _memcpy_byteswap<N>(*_cursor, _input.data(), _raw_size);
        *_cursor += _raw_size;
      } else {
        // Encode each sub-item in turn, forwarding all _dims but the first.
        for (const auto& _child : _input) {
          if (!_encode_field(_child, _cursor, _end, _cdr(_dims))) {
            return false;
          }
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
      if constexpr (_is_slab<T, ndims>()) {
        // Decode a slab of POD memory.
        const size_t _raw_size = _dims[0] * sizeof((*_output)[0]);
        if ((*_cursor + _raw_size) > _end) {
          return false;
        }
        constexpr size_t N = _get_slab_step<T, ndims>();
        _memcpy_byteswap<N>(_output->data(), *_cursor, _raw_size);
        *_cursor += _raw_size;
      } else {
        // Decode each sub-item in turn.
        for (auto& _child : *_output) {
          if (!_decode_field(&_child, _cursor, _end, _cdr(_dims))) {
            return false;
          }
        }
      }
      return true;
    }
  }
};

}  // namespace papa
