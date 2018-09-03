#pragma once

#include "drake/common/drake_deprecated.h"

/// @file
/// This file contains traits for number (scalar) types. Drake libraries that
/// are templated on a scalar type may consult these traits to perform
/// appropriate conditional compilation.
///
/// This file will also contain trait specializations for officially-supported
/// Drake scalar types, as we decide what those types are. Other scalar types,
/// such as experimental, test, and example types within Drake, or custom types
/// outside of Drake, may specialize the traits locally.

namespace drake {

/// is_numeric is true for types that are on the real line. The exact list
/// of operations that Drake requires numeric types to satisfy is not yet a
/// hard API commitment; we expect it to expand slowly over time. However, it
/// will only include operations that real numbers can implement.
///
/// By default, is_numeric is true. It should be specialized to false as
/// needed to avoid compiling Drake features that don't make sense for
/// non-real types.
///
/// Examples:
///
/// is_numeric should be true for types like double, int, and AutoDiffScalar.
///
/// is_numeric should be false for types like std::complex, Polynomial, and
/// symbolic::Expression.
template <typename T>
struct is_numeric {
  // TODO(jwnimmer-tri) Remove number_traits.h on or about 2018-12-01.
  DRAKE_DEPRECATED("This trait will be removed")
  static constexpr bool value = true;
};

}  // namespace drake
