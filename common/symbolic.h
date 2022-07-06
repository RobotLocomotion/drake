#pragma once

/// @file
/// Provides public header files for the extended portion of Drake's symbolic
/// library that goes beyond the basic algebra (e.g., when using polynomials,
/// simplification, decomposition, etc.).
///
/// Many users will only need to use the "drake/common/symbolic/expression.h"
/// file instead of this one.
///
/// A user of the extended symbolic library should only include this header
/// file. Including other individual headers such as symbolic_polynomial.h will
/// generate a compile-time error.
///
/// Extended symbolic types are not closed under the defined operations. For
/// example, addition (+) over Monomial gives a Polynomial. If a user does not
/// include the necessary set of header files, he/she will get either
/// 1) incomprehensible c++ errors or 2) undefined runtime behaviors. The
/// problem is trickier if we use symbolic objects via Eigen.

// Add the basic algebra library, which is used by all of the below.
#include "drake/common/symbolic/expression.h"

// In each header included below, it asserts that this macro
// `DRAKE_COMMON_SYMBOLIC_HEADER` is defined. If the macro is not defined, it
// generates diagnostic error messages.
#define DRAKE_COMMON_SYMBOLIC_HEADER

// Do not alpha-sort the following block of hard-coded #includes, which is
// protected by `clang-format on/off`.
//
// Rationale: We want to maximize the use of this header, `symbolic.h`, even
// inside of the symbolic library files to avoid any mistakes which might not be
// detected. By centralizing the list here, we make sure that everyone will see
// the correct order which respects the inter-dependencies of the symbolic
// headers. This shields us from triggering undefined behaviors due to
// order-of-specialization-includes-changed mistakes.
//
// clang-format off
#include "drake/common/symbolic_monomial.h"
#include "drake/common/symbolic_monomial_util.h"
#include "drake/common/symbolic_polynomial.h"
#include "drake/common/symbolic_polynomial_basis_element.h"
#include "drake/common/symbolic_chebyshev_basis_element.h"
#include "drake/common/symbolic_monomial_basis_element.h"
#include "drake/common/symbolic_chebyshev_polynomial.h"
#include "drake/common/symbolic_polynomial_basis.h"
#include "drake/common/symbolic_generic_polynomial.h"
#include "drake/common/symbolic_rational_function.h"
#include "drake/common/symbolic_simplification.h"
#include "drake/common/symbolic_codegen.h"
// clang-format on
#undef DRAKE_COMMON_SYMBOLIC_HEADER

// TODO(#13833) We should split out this header into its own library,
// once we have the new subdirectory established.  In the meantime,
// we want to provide it "for free" here, but it is NOT part of the
// monolithic symbolic implementation components, so it should not
// appear within the preprocessor ifdef.
#include "drake/common/symbolic_decompose.h"
