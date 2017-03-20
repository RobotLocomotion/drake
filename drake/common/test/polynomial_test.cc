#include "drake/common/polynomial.h"

#include <cstddef>
#include <map>
#include <sstream>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/test/random_polynomial_matrix.h"

using Eigen::VectorXd;
using std::default_random_engine;
using std::uniform_int_distribution;
using std::uniform_real_distribution;

namespace drake {
namespace {

template <typename CoefficientType>
void testIntegralAndDerivative() {
  VectorXd coefficients = VectorXd::Random(5);
  Polynomial<CoefficientType> poly(coefficients);

  EXPECT_TRUE(CompareMatrices(poly.GetCoefficients(),
                              poly.Derivative(0).GetCoefficients(), 1e-14,
                              MatrixCompareType::absolute));

  Polynomial<CoefficientType> third_derivative = poly.Derivative(3);

  Polynomial<CoefficientType> third_derivative_check =
      poly.Derivative().Derivative().Derivative();

  EXPECT_TRUE(CompareMatrices(third_derivative.GetCoefficients(),
                              third_derivative_check.GetCoefficients(), 1e-14,
                              MatrixCompareType::absolute));

  Polynomial<CoefficientType> tenth_derivative = poly.Derivative(10);

  EXPECT_TRUE(CompareMatrices(tenth_derivative.GetCoefficients(),
                              VectorXd::Zero(1), 1e-14,
                              MatrixCompareType::absolute));

  Polynomial<CoefficientType> integral = poly.Integral(0.0);
  Polynomial<CoefficientType> poly_back = integral.Derivative();

  EXPECT_TRUE(CompareMatrices(poly_back.GetCoefficients(),
                              poly.GetCoefficients(), 1e-14,
                              MatrixCompareType::absolute));
}

template <typename CoefficientType>
void testOperators() {
  int max_num_coefficients = 6;
  int num_tests = 10;
  default_random_engine generator;
  std::uniform_int_distribution<> int_distribution(1, max_num_coefficients);
  uniform_real_distribution<double> uniform;

  for (int i = 0; i < num_tests; ++i) {
    VectorXd coeff1 = VectorXd::Random(int_distribution(generator));
    Polynomial<CoefficientType> poly1(coeff1);

    VectorXd coeff2 = VectorXd::Random(int_distribution(generator));
    Polynomial<CoefficientType> poly2(coeff2);

    double scalar = uniform(generator);

    Polynomial<CoefficientType> sum = poly1 + poly2;
    Polynomial<CoefficientType> difference = poly2 - poly1;
    Polynomial<CoefficientType> product = poly1 * poly2;
    Polynomial<CoefficientType> poly1_plus_scalar = poly1 + scalar;
    Polynomial<CoefficientType> poly1_minus_scalar = poly1 - scalar;
    Polynomial<CoefficientType> poly1_scaled = poly1 * scalar;
    Polynomial<CoefficientType> poly1_div = poly1 / scalar;
    Polynomial<CoefficientType> poly1_times_poly1 = poly1;
    const Polynomial<CoefficientType> pow_poly1_3{pow(poly1, 3)};
    const Polynomial<CoefficientType> pow_poly1_4{pow(poly1, 4)};
    const Polynomial<CoefficientType> pow_poly1_10{pow(poly1, 10)};
    poly1_times_poly1 *= poly1_times_poly1;

    double t = uniform(generator);
    EXPECT_NEAR(sum.EvaluateUnivariate(t),
                poly1.EvaluateUnivariate(t) + poly2.EvaluateUnivariate(t),
                1e-8);
    EXPECT_NEAR(difference.EvaluateUnivariate(t),
                poly2.EvaluateUnivariate(t) - poly1.EvaluateUnivariate(t),
                1e-8);
    EXPECT_NEAR(product.EvaluateUnivariate(t),
                poly1.EvaluateUnivariate(t) * poly2.EvaluateUnivariate(t),
                1e-8);
    EXPECT_NEAR(poly1_plus_scalar.EvaluateUnivariate(t),
                poly1.EvaluateUnivariate(t) + scalar, 1e-8);
    EXPECT_NEAR(poly1_minus_scalar.EvaluateUnivariate(t),
                poly1.EvaluateUnivariate(t) - scalar, 1e-8);
    EXPECT_NEAR(poly1_scaled.EvaluateUnivariate(t),
                poly1.EvaluateUnivariate(t) * scalar, 1e-8);
    EXPECT_NEAR(poly1_div.EvaluateUnivariate(t),
                poly1.EvaluateUnivariate(t) / scalar, 1e-8);
    EXPECT_NEAR(poly1_times_poly1.EvaluateUnivariate(t),
                poly1.EvaluateUnivariate(t) * poly1.EvaluateUnivariate(t),
                1e-8);
    EXPECT_NEAR(pow_poly1_3.EvaluateUnivariate(t),
                pow(poly1.EvaluateUnivariate(t), 3), 1e-8);
    EXPECT_NEAR(pow_poly1_4.EvaluateUnivariate(t),
                pow(poly1.EvaluateUnivariate(t), 4), 1e-8);
    EXPECT_NEAR(pow_poly1_10.EvaluateUnivariate(t),
                pow(poly1.EvaluateUnivariate(t), 10), 1e-8);
    EXPECT_NEAR(pow_poly1_3.EvaluateUnivariate(t) *
                    pow_poly1_4.EvaluateUnivariate(t) *
                    pow_poly1_3.EvaluateUnivariate(t),
                pow_poly1_10.EvaluateUnivariate(t), 1e-8);

    // Check the '==' operator.
    EXPECT_TRUE(poly1 + poly2 == sum);
    EXPECT_FALSE(poly1 == sum);
  }
}

template <typename CoefficientType>
void testRoots() {
  int max_num_coefficients = 6;
  default_random_engine generator;
  std::uniform_int_distribution<> int_distribution(1, max_num_coefficients);

  int num_tests = 50;
  for (int i = 0; i < num_tests; ++i) {
    VectorXd coeffs = VectorXd::Random(int_distribution(generator));
    Polynomial<CoefficientType> poly(coeffs);
    auto roots = poly.Roots();
    EXPECT_EQ(roots.rows(), poly.GetDegree());
    for (int k = 0; k < roots.size(); k++) {
      auto value = poly.EvaluateUnivariate(roots[k]);
      EXPECT_NEAR(std::abs(value), 0.0, 1e-8);
    }
  }
}

void testEvalType() {
  int max_num_coefficients = 6;
  default_random_engine generator;
  std::uniform_int_distribution<> int_distribution(1, max_num_coefficients);
  VectorXd coeffs = VectorXd::Random(int_distribution(generator));
  Polynomial<double> poly(coeffs);

  auto valueIntInput = poly.EvaluateUnivariate(1);
  EXPECT_EQ(typeid(decltype(valueIntInput)), typeid(double));

  auto valueComplexInput =
      poly.EvaluateUnivariate(std::complex<double>(1.0, 2.0));
  EXPECT_EQ(typeid(decltype(valueComplexInput)), typeid(std::complex<double>));
}

template <typename CoefficientType>
void testPolynomialMatrix() {
  int max_matrix_rows_cols = 7;
  int num_coefficients = 6;
  default_random_engine generator;

  uniform_int_distribution<> matrix_size_distribution(1, max_matrix_rows_cols);
  int rows_A = matrix_size_distribution(generator);
  int cols_A = matrix_size_distribution(generator);
  int rows_B = cols_A;
  int cols_B = matrix_size_distribution(generator);

  auto A = test::RandomPolynomialMatrix<CoefficientType>(num_coefficients,
                                                         rows_A, cols_A);
  auto B = test::RandomPolynomialMatrix<CoefficientType>(num_coefficients,
                                                         rows_B, cols_B);
  auto C = test::RandomPolynomialMatrix<CoefficientType>(num_coefficients,
                                                         rows_A, cols_A);
  auto product = A * B;
  auto sum = A + C;

  uniform_real_distribution<double> uniform;
  for (int row = 0; row < A.rows(); ++row) {
    for (int col = 0; col < A.cols(); ++col) {
      double t = uniform(generator);
      EXPECT_NEAR(sum(row, col).evaluateUnivariate(t),
                  A(row, col).evaluateUnivariate(t) +
                  C(row, col).evaluateUnivariate(t), 1e-8);

      double expected_product = 0.0;
      for (int i = 0; i < A.cols(); ++i) {
        expected_product += A(row, i).evaluateUnivariate(t) *
                            B(i, col).evaluateUnivariate(t);
      }
      EXPECT_NEAR(product(row, col).evaluateUnivariate(t),
                  expected_product, 1e-8);
    }
  }

  C.setZero();  // this was a problem before
}

GTEST_TEST(PolynomialTest, IntegralAndDerivative) {
  testIntegralAndDerivative<double>();
}

GTEST_TEST(PolynomialTest, TestMakeMonomialsUnique) {
  Eigen::Vector2d coefficients(1, 2);
  Polynomial<double> poly(coefficients);
  const auto poly_squared = poly * poly;
  EXPECT_EQ(poly_squared.GetNumberOfCoefficients(), 3);
}

GTEST_TEST(PolynomialTest, Operators) { testOperators<double>(); }

GTEST_TEST(PolynomialTest, Roots) { testRoots<double>(); }

GTEST_TEST(PolynomialTest, EvalType) { testEvalType(); }

GTEST_TEST(PolynomialTest, IsAffine) {
  Polynomiald x("x");
  Polynomiald y("y");

  EXPECT_TRUE(x.IsAffine());
  EXPECT_TRUE(y.IsAffine());
  EXPECT_TRUE((2 + x).IsAffine());
  EXPECT_TRUE((2 * x).IsAffine());
  EXPECT_FALSE((x * x).IsAffine());
  EXPECT_FALSE((x * y).IsAffine());
  EXPECT_TRUE((x + y).IsAffine());
  EXPECT_TRUE((2 + x + y).IsAffine());
  EXPECT_TRUE((2 + (2 * x) + y).IsAffine());
  EXPECT_FALSE((2 + (y * x) + y).IsAffine());
}

GTEST_TEST(PolynomialTest, VariableIdGeneration) {
  // Probe the outer edge cases of variable ID generation.

  // There is no documented maximum ID, but empirically it is 2325.  What we
  // really care about here is just that there is some value below which it
  // succeeds and above which it fails.
  static const int kMaxId = 2325;

  EXPECT_NO_THROW(Polynomial<double>("x", kMaxId));
  EXPECT_NO_THROW(Polynomial<double>("zzzz", 1));
  EXPECT_NO_THROW(Polynomial<double>("zzzz", kMaxId));
  EXPECT_THROW(Polynomial<double>("!"),
               std::runtime_error);  // Illegal character.
  EXPECT_THROW(Polynomial<double>("zzzz@"),
               std::runtime_error);  // Illegal length.
  EXPECT_THROW(Polynomial<double>("z", 0),
               std::runtime_error);  // Illegal ID.
  EXPECT_THROW(Polynomial<double>("z", kMaxId + 1),
               std::runtime_error);  // Illegal ID.

  // Test that ID generation round-trips correctly.
  std::stringstream test_stream;
  test_stream << Polynomial<double>("x", 1);
  std::string result;
  test_stream >> result;
  EXPECT_EQ(result, "x1");
}

GTEST_TEST(PolynomialTest, GetVariables) {
  Polynomiald x = Polynomiald("x");
  Polynomiald::VarType x_var = x.GetSimpleVariable();
  Polynomiald y = Polynomiald("y");
  Polynomiald::VarType y_var = y.GetSimpleVariable();
  Polynomiald z = Polynomiald("z");
  Polynomiald::VarType z_var = z.GetSimpleVariable();

  EXPECT_TRUE(x.GetVariables().count(x_var));
  EXPECT_FALSE(x.GetVariables().count(y_var));

  EXPECT_FALSE(Polynomiald().GetVariables().count(x_var));

  EXPECT_TRUE((x + x).GetVariables().count(x_var));

  EXPECT_TRUE((x + y).GetVariables().count(x_var));
  EXPECT_TRUE((x + y).GetVariables().count(y_var));

  EXPECT_TRUE((x * y * y + z).GetVariables().count(x_var));
  EXPECT_TRUE((x * y * y + z).GetVariables().count(y_var));
  EXPECT_TRUE((x * y * y + z).GetVariables().count(z_var));

  EXPECT_FALSE(x.Derivative().GetVariables().count(x_var));
}

// TODO(ggould-tri) -- This test does not pass, which is a misfeature or
// bug in Polynomial.
GTEST_TEST(PolynomialTest, DISABLED_Simplification) {
  Polynomiald x = Polynomiald("x");
  Polynomiald y = Polynomiald("y");

  {  // Test duplicate monomials.
    std::stringstream test_stream;
    test_stream << ((x * y) + (x * y));
    std::string result;
    test_stream >> result;
    EXPECT_EQ(result, "2 * x1 * y1");
  }

  {  // Test monomials that are duplicates under commutativity.
    std::stringstream test_stream;
    test_stream << ((x * y) + (y * x));
    std::string result;
    test_stream >> result;
    EXPECT_EQ(result, "2 * x1 * y1");
  }
}

GTEST_TEST(PolynomialTest, MonomialFactor) {
  Polynomiald x = Polynomiald("x");
  Polynomiald y = Polynomiald("y");

  // "m_" prefix denotes monomial.
  Polynomiald::Monomial m_one = Polynomiald(1).GetMonomials()[0];
  Polynomiald::Monomial m_two = Polynomiald(2).GetMonomials()[0];
  Polynomiald::Monomial m_x = x.GetMonomials()[0];
  Polynomiald::Monomial m_y = y.GetMonomials()[0];
  Polynomiald::Monomial m_2x = (x * 2).GetMonomials()[0];
  Polynomiald::Monomial m_x2 = (x * x).GetMonomials()[0];
  Polynomiald::Monomial m_x2y = (x * x * y).GetMonomials()[0];

  // Expect failures
  EXPECT_EQ(m_x.Factor(m_y).coefficient, 0);
  EXPECT_EQ(m_x.Factor(m_x2).coefficient, 0);

  // Expect successes
  EXPECT_EQ(m_x.Factor(m_x), m_one);
  EXPECT_EQ(m_2x.Factor(m_x), m_two);
  EXPECT_EQ(m_x2.Factor(m_x), m_x);
  EXPECT_EQ(m_x2y.Factor(m_x2), m_y);
  EXPECT_EQ(m_x2y.Factor(m_y), m_x2);
}

GTEST_TEST(PolynomialTest, MultivariateValue) {
  Polynomiald x = Polynomiald("x");
  Polynomiald y = Polynomiald("y");
  const Polynomiald p{x * x + 2 * x * y + y * y + 2};
  const Polynomiald pow_p_2{pow(p, 2)};
  const Polynomiald pow_p_3{pow(p, 3)};
  const Polynomiald pow_p_7{pow(p, 7)};
  const std::map<Polynomiald::VarType, double> eval_point = {
    {x.GetSimpleVariable(), 1},
    {y.GetSimpleVariable(), 2}};
  EXPECT_EQ((x * x + y).EvaluateMultivariate(eval_point), 3);
  EXPECT_EQ((2 * x * x + y).EvaluateMultivariate(eval_point), 4);
  EXPECT_EQ((x * x + 2 * y).EvaluateMultivariate(eval_point), 5);
  EXPECT_EQ((x * x + x * y).EvaluateMultivariate(eval_point), 3);
  EXPECT_NEAR(pow_p_2.EvaluateMultivariate(eval_point),
              pow(p.EvaluateMultivariate(eval_point), 2), 1e-8);
  EXPECT_NEAR(pow_p_3.EvaluateMultivariate(eval_point),
              pow(p.EvaluateMultivariate(eval_point), 3), 1e-8);
  EXPECT_NEAR(pow_p_7.EvaluateMultivariate(eval_point),
              pow(p.EvaluateMultivariate(eval_point), 7), 1e-8);
  EXPECT_NEAR((pow_p_2 * pow_p_3 * pow_p_2).EvaluateMultivariate(eval_point),
              pow_p_7.EvaluateMultivariate(eval_point), 1e-8);
}

GTEST_TEST(PolynomialTest, Conversion) {
  // Confirm that these conversions compile okay.
  Polynomial<double> x(1.0);
  Polynomial<double> y = 2.0;
  Polynomial<double> z = 3;
}

GTEST_TEST(PolynomialTest, EvaluatePartial) {
  Polynomiald x = Polynomiald("x");
  Polynomiald y = Polynomiald("y");
  Polynomiald dut = (5 * x * x * x) + (3 * x * y) + (2 * y) + 1;

  const std::map<Polynomiald::VarType, double> eval_point_null;
  const std::map<Polynomiald::VarType, double> eval_point_x = {
    {x.GetSimpleVariable(), 7}};
  const std::map<Polynomiald::VarType, double> eval_point_y = {
    {y.GetSimpleVariable(), 11}};
  const std::map<Polynomiald::VarType, double> eval_point_xy = {
    {x.GetSimpleVariable(), 7},
    {y.GetSimpleVariable(), 11}};

  // Test a couple of straightforward explicit cases.
  EXPECT_EQ(dut.EvaluatePartial(eval_point_null).GetMonomials(),
            dut.GetMonomials());
  // TODO(#2216) These fail due to a known drake bug:
#if 0
  EXPECT_EQ(dut.evaluatePartial(eval_point_x).getMonomials(),
            ((23 * y) + 1716).getMonomials());
  EXPECT_EQ(dut.evaluatePartial(eval_point_y).getMonomials(),
            ((5 * x * x * x) + (33 * x) + 23).getMonomials());
#endif

  // Test that every order of partial and then complete evaluation gives the
  // same answer.
  const double expected_result = dut.EvaluateMultivariate(eval_point_xy);
  EXPECT_EQ(
      dut.EvaluatePartial(eval_point_null).EvaluateMultivariate(eval_point_xy),
      expected_result);
  EXPECT_EQ(
      dut.EvaluatePartial(eval_point_xy).EvaluateMultivariate(eval_point_null),
      expected_result);
  EXPECT_EQ(
      dut.EvaluatePartial(eval_point_x).EvaluateMultivariate(eval_point_y),
      expected_result);
  EXPECT_EQ(
      dut.EvaluatePartial(eval_point_y).EvaluateMultivariate(eval_point_x),
      expected_result);

  // Test that zeroing out one term gives a sensible result.
  EXPECT_EQ(dut.EvaluatePartial(
      std::map<Polynomiald::VarType, double>{{x.GetSimpleVariable(), 0}}),
            (2 * y) + 1);
  EXPECT_EQ(dut.EvaluatePartial(
      std::map<Polynomiald::VarType, double>{{y.GetSimpleVariable(), 0}}),
            (5 * x * x * x) + 1);
}

}  // anonymous namespace
}  // namespace drake
