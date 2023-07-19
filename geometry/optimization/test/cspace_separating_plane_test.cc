#include "drake/geometry/optimization/cspace_separating_plane.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace geometry {
namespace optimization {
GTEST_TEST(CalcPlane, TestAllSymbolic) {
  symbolic::Variable s("s");
  Eigen::Matrix<symbolic::Variable, 8, 1> decision_vars;
  for (int i = 0; i < 8; ++i) {
    decision_vars(i) = symbolic::Variable("plane_var" + std::to_string(i));
  }
  Vector3<symbolic::Polynomial> a;
  symbolic::Polynomial b;
  int plane_degree = 1;
  CalcPlane<symbolic::Variable, symbolic::Variable, symbolic::Polynomial>(
      decision_vars, Vector1<symbolic::Variable>(s), plane_degree, &a, &b);
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(symbolic::test::PolyEqual, a(i),
                 symbolic::Polynomial(
                     decision_vars(2 * i) * s + decision_vars(2 * i + 1),
                     symbolic::Variables({s})));
  }
  EXPECT_PRED2(symbolic::test::PolyEqual, b,
               symbolic::Polynomial(decision_vars(6) * s + decision_vars(7),
                                    symbolic::Variables({s})));
}

// Test decision_vars taking double values and s takes symbolic values.
GTEST_TEST(CalcPlane, TestDoubleDecisionVariableSymbolicS) {
  symbolic::Variable s("s");
  Eigen::Matrix<double, 8, 1> decision_var_vals;
  for (int i = 0; i < 8; ++i) {
    decision_var_vals(i) = i + 1;
  }
  const int plane_degree = 1;
  Vector3<symbolic::Polynomial> a;
  symbolic::Polynomial b;
  CalcPlane<double, symbolic::Variable, symbolic::Polynomial>(
      decision_var_vals, Vector1<symbolic::Variable>(s), plane_degree, &a, &b);
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(symbolic::test::PolyEqual, a(i),
                 symbolic::Polynomial(decision_var_vals(2 * i) * s +
                                      decision_var_vals(2 * i + 1)));
  }
  EXPECT_PRED2(
      symbolic::test::PolyEqual, b,
      symbolic::Polynomial(decision_var_vals(6) * s + decision_var_vals(7)));
}

// Test with both decision variable and s taking double values.
GTEST_TEST(CalcPlane, TestDoubleDecisionVariableDoubleS) {
  const double s = 2;
  Eigen::Matrix<double, 8, 1> decision_var_vals;
  for (int i = 0; i < 8; ++i) {
    decision_var_vals(i) = i + 1;
  }
  const int plane_degree = 1;
  Eigen::Vector3d a;
  double b;
  CalcPlane<double, double, double>(decision_var_vals, Vector1d(s),
                                    plane_degree, &a, &b);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(a(i),
              decision_var_vals(2 * i) * s + decision_var_vals(2 * i + 1));
  }
  EXPECT_EQ(b, decision_var_vals(6) * s + decision_var_vals(7));
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
