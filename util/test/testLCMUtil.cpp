#include "lcmUtil.h"
#include <random>
#include <iostream>
#include "testUtil.h"

using namespace std;
using namespace Eigen;

default_random_engine generator;

void testPolynomial()
{
  int max_num_coefficients = 5;
  uniform_int_distribution<> int_distribution(1, max_num_coefficients);
  int num_coefficients = int_distribution(generator);
  VectorXd coefficients = VectorXd::Random(num_coefficients);
  Polynomial<double> poly(coefficients);
  drake::lcmt_polynomial msg;
  encodePolynomial(poly, msg);
  auto poly_back = decodePolynomial(msg);
  valuecheck(poly.isApprox(poly_back, 1e-8), true);
}

void testPolynomialMatrix()
{
  auto poly_matrix = Polynomial<double>::randomPolynomialMatrix(6, 5, 8);
  drake::lcmt_polynomial_matrix msg;
  encodePolynomialMatrix<Eigen::Dynamic,Eigen::Dynamic>(poly_matrix, msg);
  valuecheck(static_cast<int>(msg.rows), static_cast<int>(poly_matrix.rows()));
  valuecheck(static_cast<int>(msg.cols), static_cast<int>(poly_matrix.cols()));
  auto poly_matrix_back = decodePolynomialMatrix<Dynamic, Dynamic>(msg);
  valuecheck(poly_matrix.rows(), poly_matrix_back.rows());
  valuecheck(poly_matrix.cols(), poly_matrix_back.cols());
  for (int row = 0; row < msg.rows; ++row) {
    for (int col = 0; col < msg.cols; ++col) {
      valuecheck(poly_matrix(row, col).isApprox(poly_matrix_back(row, col), 1e-8), true);
    }
  }
}

void testPiecewisePolynomial()
{
  int num_segments = 6;
  int rows = 4;
  int cols = 7;
  int num_coefficients = 3;
  std::vector<double> segment_times = PiecewiseFunction::randomSegmentTimes(num_segments, generator);
  PiecewisePolynomial<double> piecewise_polynomial = PiecewisePolynomial<double>::random(rows, cols, num_coefficients, segment_times);
  drake::lcmt_piecewise_polynomial msg;
  encodePiecewisePolynomial(piecewise_polynomial, msg);
  auto piecewise_polynomial_back = decodePiecewisePolynomial(msg);
  valuecheck(piecewise_polynomial_back.isApprox(piecewise_polynomial, 1e-10), true);
}

int main(int argc, char **argv) {
  testPolynomial();
  testPolynomialMatrix();
  testPiecewisePolynomial();
  std::cout << "test passed";
}
