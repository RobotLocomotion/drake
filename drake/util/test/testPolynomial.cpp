#include "drake/util/Polynomial.h"

#include <cstddef>
#include <sstream>
#include <map>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/util/eigen_matrix_compare.h"
#include "drake/util/testUtil.h"

using drake::util::MatrixCompareType;
using Eigen::VectorXd;
using std::default_random_engine;
using std::uniform_int_distribution;
using std::uniform_real_distribution;

namespace drake {
namespace util {
namespace {

template <typename CoefficientType>
void testIntegralAndDerivative() {
  VectorXd coefficients = VectorXd::Random(5);
  Polynomial<CoefficientType> poly(coefficients);

  Polynomial<CoefficientType> third_derivative = poly.derivative(3);

  Polynomial<CoefficientType> third_derivative_check =
      poly.derivative().derivative().derivative();

  EXPECT_TRUE(CompareMatrices(third_derivative.getCoefficients(),
                              third_derivative_check.getCoefficients(), 1e-14,
                              MatrixCompareType::absolute));

  Polynomial<CoefficientType> tenth_derivative = poly.derivative(10);

  EXPECT_TRUE(CompareMatrices(tenth_derivative.getCoefficients(),
                              VectorXd::Zero(1), 1e-14,
                              MatrixCompareType::absolute));

  Polynomial<CoefficientType> integral = poly.integral(0.0);
  Polynomial<CoefficientType> poly_back = integral.derivative();

  EXPECT_TRUE(CompareMatrices(poly_back.getCoefficients(),
                              poly.getCoefficients(), 1e-14,
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
    poly1_times_poly1 *= poly1_times_poly1;

    double t = uniform(generator);
    valuecheck(sum.evaluateUnivariate(t),
               poly1.evaluateUnivariate(t) + poly2.evaluateUnivariate(t), 1e-8);
    valuecheck(difference.evaluateUnivariate(t),
               poly2.evaluateUnivariate(t) - poly1.evaluateUnivariate(t), 1e-8);
    valuecheck(product.evaluateUnivariate(t),
               poly1.evaluateUnivariate(t) * poly2.evaluateUnivariate(t), 1e-8);
    valuecheck(poly1_plus_scalar.evaluateUnivariate(t),
               poly1.evaluateUnivariate(t) + scalar, 1e-8);
    valuecheck(poly1_minus_scalar.evaluateUnivariate(t),
               poly1.evaluateUnivariate(t) - scalar, 1e-8);
    valuecheck(poly1_scaled.evaluateUnivariate(t),
               poly1.evaluateUnivariate(t) * scalar, 1e-8);
    valuecheck(poly1_div.evaluateUnivariate(t),
               poly1.evaluateUnivariate(t) / scalar, 1e-8);
    valuecheck(poly1_times_poly1.evaluateUnivariate(t),
               poly1.evaluateUnivariate(t) * poly1.evaluateUnivariate(t), 1e-8);

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
    auto roots = poly.roots();
    valuecheck<Eigen::Index>(roots.rows(), poly.getDegree());
    for (int i = 0; i < roots.size(); i++) {
      auto value = poly.evaluateUnivariate(roots[i]);
      valuecheck(std::abs(value), 0.0, 1e-8);
    }
  }
}

void testEvalType() {
  int max_num_coefficients = 6;
  default_random_engine generator;
  std::uniform_int_distribution<> int_distribution(1, max_num_coefficients);
  VectorXd coeffs = VectorXd::Random(int_distribution(generator));
  Polynomial<double> poly(coeffs);

  auto valueIntInput = poly.evaluateUnivariate(1);
  const auto& double_type = typeid(double);  // NOLINT(readability/function)
  valuecheck(typeid(decltype(valueIntInput)) == double_type, true);

  auto valueComplexInput =
      poly.evaluateUnivariate(std::complex<double>(1.0, 2.0));
  valuecheck(
      typeid(decltype(valueComplexInput)) == typeid(std::complex<double>),
      true);
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

  auto A = Polynomial<CoefficientType>::randomPolynomialMatrix(num_coefficients,
                                                               rows_A, cols_A);
  auto B = Polynomial<CoefficientType>::randomPolynomialMatrix(num_coefficients,
                                                               rows_B, cols_B);
  auto C = Polynomial<CoefficientType>::randomPolynomialMatrix(num_coefficients,
                                                               rows_A, cols_A);
  auto product = A * B;
  auto sum = A + C;

  uniform_real_distribution<double> uniform;
  for (std::ptrdiff_t row = 0; row < A.rows(); ++row) {
    for (std::ptrdiff_t col = 0; col < A.cols(); ++col) {
      double t = uniform(generator);
      EXPECT_NEAR(sum(row, col).evaluateUnivariate(t),
                  A(row, col).evaluateUnivariate(t) +
                  C(row, col).evaluateUnivariate(t), 1e-8);

      double expected_product = 0.0;
      for (std::ptrdiff_t i = 0; i < A.cols(); ++i) {
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

GTEST_TEST(PolynomialTest, Operators) { testOperators<double>(); }

GTEST_TEST(PolynomialTest, Roots) { testRoots<double>(); }

GTEST_TEST(PolynomialTest, EvalType) { testEvalType(); }

GTEST_TEST(PolynomialTest, PolynomialMatrix) { testPolynomialMatrix<double>(); }

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
  Polynomiald::VarType x_var = x.getSimpleVariable();
  Polynomiald y = Polynomiald("y");
  Polynomiald::VarType y_var = y.getSimpleVariable();
  Polynomiald z = Polynomiald("z");
  Polynomiald::VarType z_var = z.getSimpleVariable();

  EXPECT_TRUE(x.getVariables().count(x_var));
  EXPECT_FALSE(x.getVariables().count(y_var));

  EXPECT_FALSE(Polynomiald().getVariables().count(x_var));

  EXPECT_TRUE((x + x).getVariables().count(x_var));

  EXPECT_TRUE((x + y).getVariables().count(x_var));
  EXPECT_TRUE((x + y).getVariables().count(y_var));

  EXPECT_TRUE((x * y * y + z).getVariables().count(x_var));
  EXPECT_TRUE((x * y * y + z).getVariables().count(y_var));
  EXPECT_TRUE((x * y * y + z).getVariables().count(z_var));

  EXPECT_FALSE(x.derivative().getVariables().count(x_var));
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
  Polynomiald::Monomial m_one = Polynomiald(1).getMonomials()[0];
  Polynomiald::Monomial m_two = Polynomiald(2).getMonomials()[0];
  Polynomiald::Monomial m_x = x.getMonomials()[0];
  Polynomiald::Monomial m_y = y.getMonomials()[0];
  Polynomiald::Monomial m_2x = (x * 2).getMonomials()[0];
  Polynomiald::Monomial m_x2 = (x * x).getMonomials()[0];
  Polynomiald::Monomial m_x2y = (x * x * y).getMonomials()[0];

  // Expect failures
  EXPECT_EQ(m_x.factor(m_y).coefficient, 0);
  EXPECT_EQ(m_x.factor(m_x2).coefficient, 0);

  // Expect successes
  EXPECT_EQ(m_x.factor(m_x), m_one);
  EXPECT_EQ(m_2x.factor(m_x), m_two);
  EXPECT_EQ(m_x2.factor(m_x), m_x);
  EXPECT_EQ(m_x2y.factor(m_x2), m_y);
  EXPECT_EQ(m_x2y.factor(m_y), m_x2);
}

GTEST_TEST(PolynomialTest, MultivariateValue) {
  Polynomiald x = Polynomiald("x");
  Polynomiald y = Polynomiald("y");
  const std::map<Polynomiald::VarType, double> eval_point = {
    {x.getSimpleVariable(), 1},
    {y.getSimpleVariable(), 2}};
  EXPECT_EQ((x * x + y).evaluateMultivariate(eval_point), 3);
  EXPECT_EQ((2 * x * x + y).evaluateMultivariate(eval_point), 4);
  EXPECT_EQ((x * x + 2 * y).evaluateMultivariate(eval_point), 5);
  EXPECT_EQ((x * x + x * y).evaluateMultivariate(eval_point), 3);
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
    {x.getSimpleVariable(), 7}};
  const std::map<Polynomiald::VarType, double> eval_point_y = {
    {y.getSimpleVariable(), 11}};
  const std::map<Polynomiald::VarType, double> eval_point_xy = {
    {x.getSimpleVariable(), 7},
    {y.getSimpleVariable(), 11}};

  // Test a couple of straightforward explicit cases.
  EXPECT_EQ(dut.evaluatePartial(eval_point_null).getMonomials(),
            dut.getMonomials());
  // TODO(#2216) These fail due to a known drake bug:
#if 0
  EXPECT_EQ(dut.evaluatePartial(eval_point_x).getMonomials(),
            ((23 * y) + 1716).getMonomials());
  EXPECT_EQ(dut.evaluatePartial(eval_point_y).getMonomials(),
            ((5 * x * x * x) + (33 * x) + 23).getMonomials());
#endif

  // Test that every order of partial and then complete evaluation gives the
  // same answer.
  const double expected_result = dut.evaluateMultivariate(eval_point_xy);
  EXPECT_EQ(
      dut.evaluatePartial(eval_point_null).evaluateMultivariate(eval_point_xy),
      expected_result);
  EXPECT_EQ(
      dut.evaluatePartial(eval_point_xy).evaluateMultivariate(eval_point_null),
      expected_result);
  EXPECT_EQ(
      dut.evaluatePartial(eval_point_x).evaluateMultivariate(eval_point_y),
      expected_result);
  EXPECT_EQ(
      dut.evaluatePartial(eval_point_y).evaluateMultivariate(eval_point_x),
      expected_result);

  // Test that zeroing out one term gives a sensible result.
  EXPECT_EQ(dut.evaluatePartial(
      std::map<Polynomiald::VarType, double>{{x.getSimpleVariable(), 0}}),
            (2 * y) + 1);
  EXPECT_EQ(dut.evaluatePartial(
      std::map<Polynomiald::VarType, double>{{y.getSimpleVariable(), 0}}),
            (5 * x * x * x) + 1);
}

}  // anonymous namespace
}  // namespace test
}  // namespace drake
