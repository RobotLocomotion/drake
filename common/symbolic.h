#pragma once

// NOLINTNEXTLINE(whitespace/line_length)
#warning This header is deprecated and will be removed from Drake on or after 2022-11-01. Use one of the more specific subdirectory include paths seen below.

// This order matches the prior contents of this file.
// clang-format off
#include "drake/common/symbolic/expression.h"
/* N.B. Don't include monomial.h here; it comes from polynomial.h. */
#include "drake/common/symbolic/monomial_util.h"
#include "drake/common/symbolic/polynomial.h"
#include "drake/common/symbolic/polynomial_basis_element.h"
#include "drake/common/symbolic/chebyshev_basis_element.h"
#include "drake/common/symbolic/monomial_basis_element.h"
#include "drake/common/symbolic/chebyshev_polynomial.h"
#include "drake/common/symbolic/polynomial_basis.h"
#include "drake/common/symbolic/generic_polynomial.h"
#include "drake/common/symbolic/rational_function.h"
#include "drake/common/symbolic/simplification.h"
#include "drake/common/symbolic/codegen.h"
#include "drake/common/symbolic/decompose.h"
// clang-format on
