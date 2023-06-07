#include "drake/geometry/optimization/dev/cspace_free_path_separating_plane.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace geometry {
namespace optimization {

GTEST_TEST(CalcPlane, TestAllSymbolic) {
  std::cout << "Test var var poly" << std::endl;
  symbolic::Variable mu("mu");
  for (int plane_degree = 1; plane_degree < 3; ++plane_degree) {
    const int num_coeffs_per_poly = plane_degree + 1;
    const int num_decision_vars = 4 * num_coeffs_per_poly;
    Eigen::Matrix<symbolic::Variable, Eigen::Dynamic, 1> decision_vars{
        num_decision_vars};
    for (int i = 0; i < num_decision_vars; ++i) {
      decision_vars(i) = symbolic::Variable("plane_var" + std::to_string(i));
    }
    Vector3<symbolic::Polynomial> a;
    symbolic::Polynomial b;
    CalcPlane<symbolic::Variable, symbolic::Variable, symbolic::Polynomial>(
        decision_vars, mu, plane_degree, &a, &b);
    int decision_var_ctr = 0;
    for (int i = 0; i < 3; ++i) {
      symbolic::Polynomial::MapType expected_poly_map;
      for (int j = 0; j < num_coeffs_per_poly; ++j) {
        expected_poly_map.insert({symbolic::Monomial{mu, plane_degree - j},
                                  decision_vars(decision_var_ctr)});
        ++decision_var_ctr;
      }
      EXPECT_PRED2(symbolic::test::PolyEqual, a(i),
                   symbolic::Polynomial(expected_poly_map));
    }
    symbolic::Polynomial::MapType expected_poly_map;
    for (int j = 0; j < num_coeffs_per_poly; ++j) {
      expected_poly_map.insert({symbolic::Monomial{mu, plane_degree - j},
                                decision_vars(decision_var_ctr)});
      ++decision_var_ctr;
    }
    EXPECT_PRED2(symbolic::test::PolyEqual, b,
                 symbolic::Polynomial(expected_poly_map));
  }
}

// Test decision_vars taking double values and mu takes symbolic values.
GTEST_TEST(CalcPlane, TestDoubleDecisionVariableSymbolicMu) {
  std::cout << "Test double sym poly" << std::endl;
  symbolic::Variable mu("mu");
  for (int plane_degree = 1; plane_degree < 3; ++plane_degree) {
    const int num_coeffs_per_poly = plane_degree + 1;
    const int num_decision_vars = 4 * num_coeffs_per_poly;
    Eigen::Matrix<double, Eigen::Dynamic, 1> decision_vars{num_decision_vars};
    for (int i = 0; i < num_decision_vars; ++i) {
      decision_vars(i) = i + 2;
    }
    Vector3<symbolic::Polynomial> a;
    symbolic::Polynomial b;
    CalcPlane<double, symbolic::Variable, symbolic::Polynomial>(
        decision_vars, mu, plane_degree, &a, &b);
    int decision_var_ctr = 0;
    for (int i = 0; i < 3; ++i) {
      symbolic::Polynomial::MapType expected_poly_map;
      for (int j = 0; j < num_coeffs_per_poly; ++j) {
        expected_poly_map.insert({symbolic::Monomial{mu, plane_degree - j},
                                  decision_vars(decision_var_ctr)});
        ++decision_var_ctr;
      }
      EXPECT_PRED2(symbolic::test::PolyEqual, a(i),
                   symbolic::Polynomial(expected_poly_map));
    }
    symbolic::Polynomial::MapType expected_poly_map;
    for (int j = 0; j < num_coeffs_per_poly; ++j) {
      expected_poly_map.insert({symbolic::Monomial{mu, plane_degree - j},
                                decision_vars(decision_var_ctr)});
      ++decision_var_ctr;
    }
    EXPECT_PRED2(symbolic::test::PolyEqual, b,
                 symbolic::Polynomial(expected_poly_map));
  }
}

// Test with both decision variable and mu taking double values.
GTEST_TEST(CalcPlane, TestDoubleDecisionVariableDoubleMu) {
  std::cout << "Test double all" << std::endl;
  const double mu = 3;
  for (int plane_degree = 1; plane_degree < 3; ++plane_degree) {
    const int num_coeffs_per_poly = plane_degree + 1;
    const int num_decision_vars = 4 * num_coeffs_per_poly;
    Eigen::Matrix<double, Eigen::Dynamic, 1> decision_vars_double{
        num_decision_vars};
    Eigen::Matrix<symbolic::Variable, Eigen::Dynamic, 1> decision_vars{
        num_decision_vars};
    symbolic::Environment env;
    for (int i = 0; i < num_decision_vars; ++i) {
      decision_vars_double(i) = i + 2;
      decision_vars(i) = symbolic::Variable("plane_var" + std::to_string(i));
      env.insert(decision_vars(i), decision_vars_double(i));
    }
    Eigen::Vector3d a_double = Eigen::Vector3d::Zero();
    double b_double{0};
    CalcPlane<double, double, double>(decision_vars_double, mu, plane_degree,
                                      &a_double, &b_double);
    Vector3<symbolic::Polynomial> a;
    symbolic::Polynomial b;
    CalcPlane<symbolic::Variable, symbolic::Variable, symbolic::Polynomial>(
        decision_vars_double, mu, plane_degree, &a, &b);
    int decision_var_ctr = 0;
    for (int i = 0; i < 3; ++i) {
      double expected_ret{0};
      for (int j = 0; j < num_coeffs_per_poly; ++j) {
        expected_ret += std::pow(mu, plane_degree - j) *
                        decision_vars_double(decision_var_ctr);
        ++decision_var_ctr;
      }
      EXPECT_EQ(a_double(i), expected_ret);
      EXPECT_EQ(a_double(i), a(i).Evaluate(env));
    }
    double expected_ret{0};
    for (int j = 0; j < num_coeffs_per_poly; ++j) {
      expected_ret += std::pow(mu, plane_degree - j) *
                      decision_vars_double(decision_var_ctr);
      ++decision_var_ctr;
    }
    EXPECT_EQ(b_double, expected_ret);
    EXPECT_EQ(b_double, b.Evaluate(env));
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
