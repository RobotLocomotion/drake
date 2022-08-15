#include <gtest/gtest.h>

// #include "drake/common/symbolic.h"
#include "drake/common/symbolic_chebyshev_basis_element.h"
#include "drake/common/symbolic_chebyshev_polynomial.h"
#include "drake/common/symbolic_codegen.h"
#include "drake/common/symbolic_decompose.h"
#include "drake/common/symbolic_generic_polynomial.h"
#include "drake/common/symbolic_latex.h"
#include "drake/common/symbolic_monomial.h"
#include "drake/common/symbolic_monomial_basis_element.h"
#include "drake/common/symbolic_monomial_util.h"
#include "drake/common/symbolic_polynomial.h"
#include "drake/common/symbolic_polynomial_basis.h"
#include "drake/common/symbolic_polynomial_basis_element.h"
#include "drake/common/symbolic_rational_function.h"
#include "drake/common/symbolic_simplification.h"
#include "drake/common/symbolic_trigonometric_polynomial.h"
#include "drake/common/unused.h"

namespace drake {
namespace symbolic {
namespace {

// Spot check for symbolic.h
GTEST_TEST(DeprecatedSymbolicHeaderTest, Monolith) {
  RationalFunction x;
  unused(x);
}

// Spot check for symbolic_latex.h
GTEST_TEST(DeprecatedSymbolicHeaderTest, Latex) {
  EXPECT_EQ(ToLatex(Variable{"x"}), "x");
}

// Spot check for symbolic_trigonometric_polynomial.h.
GTEST_TEST(DeprecatedSymbolicHeaderTest, TrigonometricPolynomial) {
  SinCosSubstitution x;
  unused(x);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
