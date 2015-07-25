#include "Polynomial.h"
#include <Eigen/Core>
#include <random>
#include "testUtil.h"
#include <iostream>
#include <typeinfo>

using namespace Eigen;
using namespace std;

template <typename CoefficientType>
void testIntegralAndDerivative() {
  VectorXd coefficients = VectorXd::Random(5);
  Polynomial<CoefficientType> poly(coefficients);

  cout << poly << endl;

  cout << "derivative: " << poly.derivative(1) << endl;
  
  Polynomial<CoefficientType> third_derivative = poly.derivative(3);
  
  cout << "third derivative: " << third_derivative << endl;
  Polynomial<CoefficientType> third_derivative_check = poly.derivative().derivative().derivative();
  valuecheckMatrix(third_derivative.getCoefficients(), third_derivative_check.getCoefficients(), 1e-14);

  Polynomial<CoefficientType> tenth_derivative = poly.derivative(10);
  valuecheckMatrix(tenth_derivative.getCoefficients(), VectorXd::Zero(1), 1e-14);

  Polynomial<CoefficientType> integral = poly.integral(0.0);
  cout << "integral: " << integral << endl;
  Polynomial<CoefficientType> poly_back = integral.derivative();
  valuecheckMatrix(poly_back.getCoefficients(), poly.getCoefficients(), 1e-14);
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

    cout << "-------" << endl;
    cout << "p1 = " << poly1 << endl;
    cout << "p2 = " << poly2 << endl;
    cout << "c = " << scalar << endl;
    
    Polynomial<CoefficientType> sum = poly1 + poly2;
    cout << "p1+p2: " << sum << endl;
    Polynomial<CoefficientType> difference = poly2 - poly1;
    cout << "p2-p1: " << difference << endl;
    Polynomial<CoefficientType> product = poly1 * poly2;
    cout << "p1*p2: " << product << endl;
    Polynomial<CoefficientType> poly1_plus_scalar = poly1 + scalar;
    cout << "p1+c: " << poly1_plus_scalar << endl;
    Polynomial<CoefficientType> poly1_minus_scalar = poly1 - scalar;
    cout << "p1-c: " << poly1_minus_scalar << endl;
    Polynomial<CoefficientType> poly1_scaled = poly1 * scalar;
    cout << "p1*c: " << poly1_scaled << endl;
    Polynomial<CoefficientType> poly1_div = poly1 / scalar;
    cout << "p1/c: " << poly1_div << endl;
    Polynomial<CoefficientType> poly1_times_poly1 = poly1;
    poly1_times_poly1 *= poly1_times_poly1;
    cout << "p1*p1" << poly1_times_poly1 << endl;

    double t = uniform(generator);
    valuecheck(sum.value(t), poly1.value(t) + poly2.value(t), 1e-8);
    valuecheck(difference.value(t), poly2.value(t) - poly1.value(t), 1e-8);
    valuecheck(product.value(t), poly1.value(t) * poly2.value(t), 1e-8);
    valuecheck(poly1_plus_scalar.value(t), poly1.value(t) + scalar, 1e-8);
    valuecheck(poly1_minus_scalar.value(t), poly1.value(t) - scalar, 1e-8);
    valuecheck(poly1_scaled.value(t), poly1.value(t) * scalar, 1e-8);
    valuecheck(poly1_div.value(t), poly1.value(t) / scalar, 1e-8);
    valuecheck(poly1_times_poly1.value(t), poly1.value(t) * poly1.value(t), 1e-8);
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
    valuecheck<DenseIndex>(roots.rows(), poly.getDegree());
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
  valuecheck(typeid(decltype(valueComplexInput)) == typeid(std::complex<double>), true);
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

  auto A = Polynomial<CoefficientType>::randomPolynomialMatrix(num_coefficients, rows_A, cols_A);
  auto B = Polynomial<CoefficientType>::randomPolynomialMatrix(num_coefficients, rows_B, cols_B);
  auto C = Polynomial<CoefficientType>::randomPolynomialMatrix(num_coefficients, rows_A, cols_A);
  auto product = A * B; // just verify that this is possible without crashing
  auto sum = A + C;

  uniform_real_distribution<double> uniform;
  for (int row = 0; row < A.rows(); ++row) {
    for (int col = 0; col < A.cols(); ++col) {
      double t = uniform(generator);
      valuecheck(sum(row, col).value(t), A(row, col).value(t) + C(row, col).value(t), 1e-8);
    }
  }

  C.setZero(); // this was a problem before
}

int main(int argc, char **argv) {
  testIntegralAndDerivative<double>();
  testOperators<double>();
  testRoots<double>();
  testEvalType();
  testPolynomialMatrix<double>();
  return 0;
}
