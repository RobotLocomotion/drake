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

void testMultiplication() {
  VectorXd coeff1 = VectorXd::Random(3);
  Polynomial poly1(coeff1);

  VectorXd coeff2 = VectorXd::Random(4);
  Polynomial poly2(coeff2);

  Polynomial product = poly1 * poly2;

  default_random_engine generator;
  uniform_real_distribution<double> uniform;

  double t = uniform(generator);
  valuecheck(product.value(t), poly1.value(t) * poly2.value(t), 1e-8);
}

int main(int argc, char **argv) {

  testIntegralAndDerivative();
  testMultiplication();
  cout << "test passed" << endl;
  return 0;
}
