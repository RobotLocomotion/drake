#pragma once

#if defined(__AVX2__) && defined(__FMA__)
#pragma GCC diagnostic push
// TODO(jwnimmer-tri) Ideally we would fix the old-style-cast warnings instead
// of suppressing them, perhaps by submitting a patch upstream.
#pragma GCC diagnostic ignored "-Wold-style-cast"
// By setting the next two HWY_... macros, we're saying that we don't want to
// generate any SIMD code for pre-AVX2 CPUs, and that we're okay not using the
// carry-less multiplication and crypto stuff.
#define HWY_BASELINE_TARGETS HWY_AVX2
#define HWY_DISABLE_PCLMUL_AES 1
#include "hwy/highway.h"
#pragma GCC diagnostic pop
#endif

#include <iostream>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

// The hn namespace holds the CPU-specific function overloads. By defining it
// using a substitute-able macro, we achieve per-CPU instruction selection.

#if defined(__AVX2__) && defined(__FMA__)

namespace hn = hwy::HWY_NAMESPACE;

/* @tparam float or double*/
template <typename T>
class SimdScalar {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SimdScalar);
  static const hn::ScalableTag<T> d;
  using V = decltype(hn::Zero(d));

  SimdScalar() : value_(hn::Zero(d)) {}

  // NOLINTNEXTLINE(runtime/explicit) to allow implicit conversion.
  SimdScalar(T value) : value_(hn::Set(d, value)) {}

  /* Compile is confused about implicit conversion when `value` is of type
   `int`. So we help it here*/
  template <typename U>
  SimdScalar(U value,
             typename std::enable_if<std::is_integral<U>::value, U>::type* = 0)
      : value_(hn::Set(d, static_cast<T>(value))) {}

  explicit SimdScalar(const T* input) : value_(hn::LoadU(d, input)) {}

  SimdScalar(const T* input, size_t lanes)
      : value_(hn::LoadN(d, input, lanes)) {
    DRAKE_ASSERT(lanes <= N);
  }

  void Write(T* output) const { hn::StoreU(value_, d, output); }
  void Write(T* output, size_t lanes) const {
    DRAKE_ASSERT(lanes <= N);
    hn::StoreN(value_, d, output, lanes);
  }

  T get_lane() const { return hn::GetLane(value_); }

  bool operator==(const SimdScalar& other) const {
    return hn::AllTrue(d, hn::Eq(value_, other.value_));
  }

  const V& value() const { return value_; }
  V& value() { return value_; }

  static size_t lanes() { return N; }

 private:
  static const size_t N = Lanes(d);

  V value_;
};

#else
template <typename T>
class SimdScalar {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SimdScalar);
  SimdScalar() = default;
  // NOLINTNEXTLINE(runtime/explicit) to allow implicit conversion.
  SimdScalar(T value) : value_(value) {}
  explicit SimdScalar(const T* input) : value_(*input) {}

  void Write(T* output) const { *output = value_; }

  static size_t lanes() { return 1; }

  T get_lane() const { return value_; }

 private:
  T value_{};
};
#endif

template <typename T>
Vector3<SimdScalar<T>> Load(const Vector3<T>* source, size_t size) {
  T data[SimdScalar<T>::lanes()];
  Vector3<SimdScalar<T>> result;
  for (int i = 0; i < 3; ++i) {
    for (size_t j = 0; j < size; ++j) {
      data[j] = source[j][i];
    }
    result[i] = SimdScalar<T>(data, size);
  }
  return result;
}

template <typename T>
Matrix3<SimdScalar<T>> Load(const Matrix3<T>* source, size_t size) {
  T data[SimdScalar<T>::lanes()];
  Matrix3<SimdScalar<T>> result;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (size_t k = 0; k < size; ++k) {
        data[k] = source[k](i, j);
      }
      result(i, j) = SimdScalar<T>(data, size);
    }
  }
  return result;
}

template <typename T>
void Store(const Vector3<SimdScalar<T>>& v, Vector3<T>* dest, size_t size) {
  T data[SimdScalar<T>::lanes()];
  for (int i = 0; i < 3; ++i) {
    v[i].Write(data);
    for (size_t j = 0; j < size; ++j) {
      dest[j][i] = data[j];
    }
  }
}

template <typename T>
void Store(const Matrix3<SimdScalar<T>>& m, Matrix3<T>* dest, size_t size) {
  T data[SimdScalar<T>::lanes()];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      m(i, j).Write(data);
      for (size_t k = 0; k < size; ++k) {
        dest[k](i, j) = data[k];
      }
    }
  }
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake

// These further refine our SimdScalar type and must appear in exactly this
// order.
// clang-format off
#include "drake/multibody/mpm/simd_operations.h"
#include "drake/multibody/mpm/eigen_specializations.h"
// clang-format on
