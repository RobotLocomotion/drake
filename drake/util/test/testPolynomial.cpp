#include <iostream>
#include "drake/util/Polynomial.h"
#include "drake/util/eigen_matrix_compare.h"
#include "drake/util/testUtil.h"
#include "gtest/gtest.h"

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
    valuecheck(sum.value(t), poly1.value(t) + poly2.value(t), 1e-8);
    valuecheck(difference.value(t), poly2.value(t) - poly1.value(t), 1e-8);
    valuecheck(product.value(t), poly1.value(t) * poly2.value(t), 1e-8);
    valuecheck(poly1_plus_scalar.value(t), poly1.value(t) + scalar, 1e-8);
    valuecheck(poly1_minus_scalar.value(t), poly1.value(t) - scalar, 1e-8);
    valuecheck(poly1_scaled.value(t), poly1.value(t) * scalar, 1e-8);
    valuecheck(poly1_div.value(t), poly1.value(t) / scalar, 1e-8);
    valuecheck(poly1_times_poly1.value(t), poly1.value(t) * poly1.value(t),
               1e-8);
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
      auto value = poly.value(roots[i]);
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

  auto valueIntInput = poly.value(1);
  valuecheck(typeid(decltype(valueIntInput)) == typeid(double), true);

  auto valueComplexInput = poly.value(std::complex<double>(1.0, 2.0));
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
  auto product = A * B;  // just verify that this is possible without crashing
  auto sum = A + C;

  uniform_real_distribution<double> uniform;
  for (int row = 0; row < A.rows(); ++row) {
    for (int col = 0; col < A.cols(); ++col) {
      double t = uniform(generator);
      valuecheck(sum(row, col).value(t),
                 A(row, col).value(t) + C(row, col).value(t), 1e-8);
    }
  }

  C.setZero();  // this was a problem before
}

TEST(PolynomialTest, IntegralAndDerivative) {
  testIntegralAndDerivative<double>();
}

TEST(PolynomialTest, Operators) { testOperators<double>(); }

TEST(PolynomialTest, Roots) { testRoots<double>(); }

TEST(PolynomialTest, EvalType) { testEvalType(); }

TEST(PolynomialTest, PolynomialMatrix) { testPolynomialMatrix<double>(); }

TEST(PolynomialTest, VariableIdGeneration) {
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

}
}  // namespace test
}  // namespace drake
