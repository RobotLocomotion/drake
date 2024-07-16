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

#include "drake/multibody/mpm/simd_scalar.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

#if defined(__AVX2__) && defined(__FMA__)

/** Standard compound addition and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
template <typename T>
inline SimdScalar<T>& operator+=(SimdScalar<T>& a, const SimdScalar<T>& b) {
  a.value() = hn::Add(a.value(), b.value());
  return a;
}

template <typename T>
inline SimdScalar<T> operator+(SimdScalar<T> a, const SimdScalar<T>& b) {
  a += b;
  return a;
}

/** Standard compound subtraction and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
template <typename T>
inline SimdScalar<T>& operator-=(SimdScalar<T>& a, SimdScalar<T> b) {
  a.value() = hn::Sub(a.value(), b.value());
  return a;
}

template <typename T>
inline SimdScalar<T> operator-(SimdScalar<T> a, SimdScalar<T> b) {
  a -= b;
  return a;
}

template <typename T>
inline SimdScalar<T> operator-(SimdScalar<T> a, T b) {
  a -= SimdScalar<T>(b);
  return a;
}

template <typename T>
inline SimdScalar<T> operator-(T a, SimdScalar<T> b) {
  SimdScalar<T> result(a);
  result -= b;
  return result;
}

/** Standard compound multiplication and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
template <typename T>
inline SimdScalar<T>& operator*=(SimdScalar<T>& a, const SimdScalar<T>& b) {
  a.value() = hn::Mul(a.value(), b.value());
  return a;
}

template <typename T>
inline SimdScalar<T>& operator*=(SimdScalar<T>& a, T b) {
  a *= SimdScalar<T>(b);
  return a;
}

template <typename T>
inline SimdScalar<T> operator*(SimdScalar<T> a, SimdScalar<T> b) {
  a *= b;
  return a;
}

template <typename T>
inline SimdScalar<T> operator*(SimdScalar<T> a, T b) {
  a *= b;
  return a;
}

template <typename T>
inline SimdScalar<T> operator*(T a, SimdScalar<T> b) {
  b *= a;
  return b;
}

/** Standard compound division and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
template <typename T>
inline SimdScalar<T>& operator/=(SimdScalar<T>& a, SimdScalar<T> b) {
  a.value() = hn::Div(a.value(), b.value());
  return a;
}

template <typename T>
inline SimdScalar<T> operator/(SimdScalar<T> a, SimdScalar<T> b) {
  a /= b;
  return a;
}

template <typename T>
inline SimdScalar<T> operator/(SimdScalar<T> a, T b) {
  a /= b;
  return a;
}

template <typename T>
inline SimdScalar<T> operator/(T a, SimdScalar<T> b) {
  SimdScalar<T> result(a);
  result /= b;
  return result;
}
#else

/** Standard compound addition and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
template <typename T>
inline SimdScalar<T>& operator+=(SimdScalar<T>& a, const SimdScalar<T>& b) {
  a.value() = a.value() + b.value();
  return a;
}

template <typename T>
inline SimdScalar<T> operator+(SimdScalar<T> a, const SimdScalar<T>& b) {
  a += b;
  return a;
}

/** Standard compound subtraction and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
template <typename T>
inline SimdScalar<T>& operator-=(SimdScalar<T>& a, SimdScalar<T> b) {
  a.value() = a.value() - b.value();
  return a;
}

template <typename T>
inline SimdScalar<T> operator-(SimdScalar<T> a, SimdScalar<T> b) {
  a -= b;
  return a;
}

template <typename T>
inline SimdScalar<T> operator-(SimdScalar<T> a, T b) {
  a -= SimdScalar<T>(b);
  return a;
}

template <typename T>
inline SimdScalar<T> operator-(T a, SimdScalar<T> b) {
  SimdScalar<T> result(a);
  result -= b;
  return result;
}

/** Standard compound multiplication and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
template <typename T>
inline SimdScalar<T>& operator*=(SimdScalar<T>& a, const SimdScalar<T>& b) {
  a.value() = a.value() * b.value();
  return a;
}

template <typename T>
inline SimdScalar<T>& operator*=(SimdScalar<T>& a, T b) {
  a *= SimdScalar<T>(b);
  return a;
}

template <typename T>
inline SimdScalar<T> operator*(SimdScalar<T> a, SimdScalar<T> b) {
  a *= b;
  return a;
}

template <typename T>
inline SimdScalar<T> operator*(SimdScalar<T> a, T b) {
  a *= b;
  return a;
}

template <typename T>
inline SimdScalar<T> operator*(T a, SimdScalar<T> b) {
  b *= a;
  return b;
}

/** Standard compound division and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
template <typename T>
inline SimdScalar<T>& operator/=(SimdScalar<T>& a, SimdScalar<T> b) {
  a.value() = a.value() / b.value();
  return a;
}

template <typename T>
inline SimdScalar<T> operator/(SimdScalar<T> a, SimdScalar<T> b) {
  a /= b;
  return a;
}

template <typename T>
inline SimdScalar<T> operator/(SimdScalar<T> a, T b) {
  a /= b;
  return a;
}

template <typename T>
inline SimdScalar<T> operator/(T a, SimdScalar<T> b) {
  SimdScalar<T> result(a);
  result /= b;
  return result;
}

#endif

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
