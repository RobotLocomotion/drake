#include "Polynomial.h"
#include <Eigen/Core>
#include <random>
#include "testUtil.h"

using namespace Eigen;
using namespace std;

void testIntegralAndDerivative() {
  VectorXd coefficients = VectorXd::Random(5);
  Polynomial poly(coefficients);
  Polynomial third_derivative = poly.derivative(3);
  Polynomial third_derivative_check = poly.derivative().derivative().derivative();
  valuecheck(third_derivative.getCoefficients(), third_derivative_check.getCoefficients(), 1e-14);

  Polynomial tenth_derivative = poly.derivative(10);
  valuecheck(tenth_derivative.getCoefficients(), VectorXd::Zero(1), 1e-14);

  Polynomial integral = poly.integral(0.0);
  Polynomial poly_back = integral.derivative();
  valuecheck(poly_back.getCoefficients(), poly.getCoefficients(), 1e-14);
}

void testOperators() {
  int max_num_coefficients = 6;
  int num_tests = 10;
  default_random_engine generator;
  std::uniform_int_distribution<> int_distribution(1, max_num_coefficients);
  uniform_real_distribution<double> uniform;

  for (int i = 0; i < num_tests; ++i) {
    VectorXd coeff1 = VectorXd::Random(int_distribution(generator));
    Polynomial poly1(coeff1);

    VectorXd coeff2 = VectorXd::Random(int_distribution(generator));
    Polynomial poly2(coeff2);

    double scalar = uniform(generator);

    Polynomial sum = poly1 + poly2;
    Polynomial difference = poly2 - poly1;
    Polynomial product = poly1 * poly2;
    Polynomial poly1_plus_scalar = poly1 + scalar;
    Polynomial poly1_minus_scalar = poly1 - scalar;
    Polynomial poly1_scaled = poly1 * scalar;
    Polynomial poly1_div = poly1 / scalar;

    double t = uniform(generator);
    valuecheck(sum.value(t), poly1.value(t) + poly2.value(t), 1e-8);
    valuecheck(difference.value(t), poly2.value(t) - poly1.value(t), 1e-8);
    valuecheck(product.value(t), poly1.value(t) * poly2.value(t), 1e-8);
    valuecheck(poly1_plus_scalar.value(t), poly1.value(t) + scalar, 1e-8);
    valuecheck(poly1_minus_scalar.value(t), poly1.value(t) - scalar, 1e-8);
    valuecheck(poly1_scaled.value(t), poly1.value(t) * scalar, 1e-8);
    valuecheck(poly1_div.value(t), poly1.value(t) / scalar, 1e-8);
  }
}

void testRoots() {
  int max_num_coefficients = 6;
  int num_tests = 50;
  default_random_engine generator;
  std::uniform_int_distribution<> int_distribution(1, max_num_coefficients);

  for (int i = 0; i < num_tests; ++i) {
    VectorXd coeffs = VectorXd::Random(int_distribution(generator));
    Polynomial poly(coeffs);
    auto roots = poly.roots();
    valuecheck(roots.rows(), poly.getDegree());
    for (int i = 0; i < roots.size(); i++) {
      auto value = poly.value(roots[i]);
      valuecheck(std::abs(value), 0.0, 1e-8);
    }
  }
}

int main(int argc, char **argv) {

  testIntegralAndDerivative();
  testOperators();
  testRoots();
  cout << "test passed" << endl;
  return 0;
}
