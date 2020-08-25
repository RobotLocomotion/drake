#pragma once
/// @file
/// Provides public header files of Drake's symbolic library.
/// A user of the symbolic library should only include this header file.
/// Including other individual headers such as symbolic_expression.h will
/// generate a compile-time error.
///
/// Many symbolic types are not closed under the defined operations. For
/// example, relational operations (i.e. <) over symbolic::Expression produce
/// symbolic::Formula. Another example is addition (+) over Monomial which gives
/// Polynomial. If a user does not include the necessary set of header files,
/// he/she will get either 1) incomprehensible c++ errors or 2) undefined
/// runtime behaviors. The problem is trickier if we use symbolic objects via
/// Eigen.

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
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_expression_visitor.h"
#include "drake/common/symbolic_ldlt.h"
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
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_formula_visitor.h"
#include "drake/common/symbolic_simplification.h"
#include "drake/common/symbolic_codegen.h"
// clang-format on
#undef DRAKE_COMMON_SYMBOLIC_HEADER
