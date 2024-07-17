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
  using ValueType = decltype(hn::Zero(d));

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
    DRAKE_ASSERT(0 < lanes && lanes <= kN);
  }

  void Write(T* output) const { hn::StoreU(value_, d, output); }
  void Write(T* output, size_t lanes) const {
    DRAKE_ASSERT(lanes <= kN);
    hn::StoreN(value_, d, output, lanes);
  }

  T get_lane() const { return hn::GetLane(value_); }

  bool operator==(const SimdScalar& other) const {
    return hn::AllTrue(d, hn::Eq(value_, other.value_));
  }

  const ValueType& value() const { return value_; }
  ValueType& value() { return value_; }

  static size_t lanes() { return kN; }

 private:
  static const size_t kN = Lanes(d);
  ValueType value_;
};

#else
template <typename T>
class SimdScalar {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SimdScalar);

  using ValueType = T;

  SimdScalar() = default;
  // NOLINTNEXTLINE(runtime/explicit) to allow implicit conversion.
  SimdScalar(T value) : value_(value) {}

  /* Compile is confused about implicit conversion when `value` is of type
   `int`. So we help it here*/
  template <typename U>
  SimdScalar(U value,
             typename std::enable_if<std::is_integral<U>::value, U>::type* = 0)
      : value_(static_cast<T>(value)) {}

  explicit SimdScalar(const T* input) : value_(*input) {}

  SimdScalar(const T* input, size_t lanes) : value_(input[0]) {
    DRAKE_ASSERT(0 < lanes && lanes <= kN);
  }

  void Write(T* output) const { *output = value_; }
  void Write(T* output, size_t lanes) const {
    DRAKE_ASSERT(lanes <= kN);
    output[0] = value_;
  }

  T get_lane() const { return value_; }

  bool operator==(const SimdScalar<T>& other) const {
    return value_ == other.value_;
  }

  const ValueType& value() const { return value_; }
  ValueType& value() { return value_; }

  static size_t lanes() { return kN; }

 private:
  static const size_t kN = 1;
  T value_{};
};
#endif

template <typename T>
Vector3<SimdScalar<T>> Load(const std::vector<Vector3<T>>& source,
                            const std::vector<int>& indices) {
  const size_t size = indices.size();
  T data[size];
  Vector3<SimdScalar<T>> result;
  for (int i = 0; i < 3; ++i) {
    for (size_t j = 0; j < size; ++j) {
      data[j] = source[indices[j]][i];
    }
    result[i] = SimdScalar<T>(data, size);
  }
  return result;
}

template <typename T>
Matrix3<SimdScalar<T>> Load(const std::vector<Matrix3<T>>& source,
                            const std::vector<int>& indices) {
  const size_t size = indices.size();
  T data[size];
  Matrix3<SimdScalar<T>> result;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (size_t k = 0; k < size; ++k) {
        data[k] = source[indices[k]](i, j);
      }
      result(i, j) = SimdScalar<T>(data, size);
    }
  }
  return result;
}

template <typename T>
void Store(const Vector3<SimdScalar<T>>& v, std::vector<Vector3<T>>* dest,
           const std::vector<int>& indices) {
  const size_t size = indices.size();
  T data[SimdScalar<T>::lanes()];
  for (int i = 0; i < 3; ++i) {
    v[i].Write(data);
    for (size_t j = 0; j < size; ++j) {
      (*dest)[indices[j]][i] = data[j];
    }
  }
}

template <typename T>
void Store(const Matrix3<SimdScalar<T>>& m, std::vector<Matrix3<T>>* dest,
           const std::vector<int>& indices) {
  const size_t size = indices.size();
  T data[SimdScalar<T>::lanes()];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      m(i, j).Write(data);
      for (size_t k = 0; k < size; ++k) {
        (*dest)[indices[k]](i, j) = data[k];
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
