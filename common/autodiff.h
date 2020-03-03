#pragma once
/// @file This header provides a single inclusion point for autodiff-related
/// header files in the `drake/common` directory. Users should include this
/// file. Including other individual headers such as `drake/common/autodiffxd.h`
/// will generate a compile-time warning.

// In each header included below, it asserts that this macro
// `DRAKE_COMMON_AUTODIFF_HEADER` is defined. If the macro is not defined, it
// generates diagnostic warning messages.
#define DRAKE_COMMON_AUTODIFF_HEADER

#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>

static_assert(EIGEN_VERSION_AT_LEAST(3, 3, 4),
              "Drake requires Eigen >= v3.3.4.");

// Do not alpha-sort the following block of hard-coded #includes, which is
// protected by `clang-format on/off`.
//
// Rationale: We want to maximize the use of this header, `autodiff.h`, even
// inside of the autodiff-related files to avoid any mistakes which might not be
// detected. By centralizing the list here, we make sure that everyone will see
// the correct order which respects the inter-dependencies of the autodiff
// headers. This shields us from triggering undefined behaviors due to
// order-of-specialization-includes-changed mistakes.
//
// clang-format off
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/autodiffxd.h"
#include "drake/common/autodiff_overloads.h"
// clang-format on
#undef DRAKE_COMMON_AUTODIFF_HEADER
