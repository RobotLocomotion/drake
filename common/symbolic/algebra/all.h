#pragma once

// Users should not include this header file directly.  Instead, they should
// say `#include "drake/common/symbolic/algebra.h"` for brevity.

// Many symbolic types are not closed under their defined operations, e.g.,
// relational operations (i.e., ``<``) over Expression produce a Formula.
// To ensure that algebra's definitions are always consistent, we use this
// header to define the API as a single, consistent component. This macro
// provides a fail-fast at the all.h order is always obeyed.
#define DRAKE_COMMON_SYMBOLIC_ALGEBRA_ALL

// Do not alpha-sort the following block of hard-coded #includes, which is
// protected by `clang-format on/off`.
//
// This particular order ensures that everyone sees the order that respects the
// inter-dependencies of the symbolic headers. This shields us from triggering
// undefined behaviors due to varing the order of template specializations.
//
// clang-format off
#include "drake/common/symbolic/algebra/variable.h"
#include "drake/common/symbolic/algebra/variables.h"
#include "drake/common/symbolic/algebra/environment.h"
#include "drake/common/symbolic/algebra/expression.h"
#include "drake/common/symbolic/algebra/expression_visitor.h"
#include "drake/common/symbolic/algebra/ldlt.h"
#include "drake/common/symbolic/algebra/formula.h"
#include "drake/common/symbolic/algebra/formula_visitor.h"
// clang-format on

#undef DRAKE_COMMON_SYMBOLIC_ALGEBRA_ALL
