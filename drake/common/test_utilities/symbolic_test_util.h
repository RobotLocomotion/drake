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

}  // namespace test
}  // namespace symbolic
}  // namespace drake
