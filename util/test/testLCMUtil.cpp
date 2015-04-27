#include "lcmUtil.h"
#include <random>
#include <iostream>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
  default_random_engine generator;

  int max_num_coefficients = 5;
  uniform_int_distribution<> int_distribution(1, max_num_coefficients);
  int num_coefficients = int_distribution(generator);
  VectorXd coefficients = VectorXd::Random(num_coefficients);
  Polynomial<double> poly(coefficients);

  drake::lcmt_polynomial msg;
  encodePolynomial(poly, msg);

  std::cout << "test passed";
}

