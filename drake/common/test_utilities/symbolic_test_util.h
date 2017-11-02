#pragma once
/// @file
/// This file provides a set of predicates which can be used with GTEST's
/// ASSERT/EXPECT_PRED{n} macros. The motivation is to provide better diagnostic
/// information when the assertions fail. Please consider a scenario where a
/// user wants to assert that two symbolic expressions, `e1` and `e2`, are
/// structurally identical.
///
/// @code
/// // The following does not work because `operator==(const Expression& e1,
/// // const Expression& e2)` does not return a Boolean value. We need to use
/// // Expression::EqualTo() method instead.
/// ASSERT_EQ(e1, e2);
///
/// // The following compiles, but it does not provide enough information when
/// // the assertion fails. It merely reports that `e1.EqualTo(e2)` is evaluated
/// // to `false`, not to `true`.
/// ASSERT_TRUE(e1.EqualTo(e2));
///
/// // When the following assertion fails, it reports the value of `e1` and `e2`
/// // which should help debugging.
/// ASSERT_PRED2(ExprEqual, e1, e2);
/// @endcode
#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
namespace test {

inline bool VarEqual(const Variable& v1, const Variable& v2) {
  return v1.equal_to(v2);
}

inline bool VarNotEqual(const Variable& v1, const Variable& v2) {
  return !VarEqual(v1, v2);
}

inline bool VarLess(const Variable& v1, const Variable& v2) {
  return v1.less(v2);
}

inline bool VarNotLess(const Variable& v1, const Variable& v2) {
  return !VarLess(v1, v2);
}

inline bool ExprEqual(const Expression& e1, const Expression& e2) {
  return e1.EqualTo(e2);
}

inline bool ExprNotEqual(const Expression& e1, const Expression& e2) {
  return !ExprEqual(e1, e2);
}

inline bool ExprLess(const Expression& e1, const Expression& e2) {
  return e1.Less(e2);
}

inline bool ExprNotLess(const Expression& e1, const Expression& e2) {
  return !ExprLess(e1, e2);
}

inline bool PolyEqual(const Polynomial& p1, const Polynomial& p2) {
  return p1.EqualTo(p2);
}

inline bool PolyNotEqual(const Polynomial& p1, const Polynomial& p2) {
  return !PolyEqual(p1, p2);
}

template <typename F>
bool all_of(const std::vector<Formula>& formulas, const F& f) {
  return std::all_of(formulas.begin(), formulas.end(), f);
}

template <typename F>
bool any_of(const std::vector<Formula>& formulas, const F& f) {
  return std::any_of(formulas.begin(), formulas.end(), f);
}

inline bool FormulaEqual(const Formula& f1, const Formula& f2) {
  return f1.EqualTo(f2);
}

inline bool FormulaNotEqual(const Formula& f1, const Formula& f2) {
  return !FormulaEqual(f1, f2);
}

inline bool FormulaLess(const Formula& f1, const Formula& f2) {
  return f1.Less(f2);
}

inline bool FormulaNotLess(const Formula& f1, const Formula& f2) {
  return !FormulaLess(f1, f2);
}

/**
 * Compare if two polynomials p1 and p2 are the same, by checking if all the
 * coefficients in their difference p1 - p2 is no larger than tol.
 * @param p1 A polynomial.
 * @param p2 A polynomial.
 * @param tol The tolerance on the coefficients of p1 - p2.
 */
::testing::AssertionResult PolynomialEqual(const symbolic::Polynomial& p1,
                     const symbolic::Polynomial& p2, double tol) {
  const symbolic::Polynomial diff = p1 - p2;
  // Check if the absolute value of the coefficient for each monomial is less
  // than tol.
  const symbolic::Polynomial::MapType& map = diff.monomial_to_coefficient_map();
  for (const auto& p : map) {
    if (std::abs(get_constant_value(p.second)) > tol) {
      return ::testing::AssertionFailure()
             << "The coefficient for " << p.first << " is " << p.second
             << ", exceed tolerance " << tol << "\n";
    }
  }
  return ::testing::AssertionSuccess();
}
}  // namespace test
}  // namespace symbolic
}  // namespace drake
