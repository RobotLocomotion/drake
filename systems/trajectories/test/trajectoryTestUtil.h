#ifndef SYSTEMS_TRAJECTORIES_TEST_TRAJECTORYTESTUTIL_H_
#define SYSTEMS_TRAJECTORIES_TEST_TRAJECTORYTESTUTIL_H_

#include <Eigen/Core>
#include <vector>
#include <random>
#include "PiecewisePolynomial.h"

template<typename CoefficientType>
PiecewisePolynomial<CoefficientType> generateRandomPiecewisePolynomial(
    Eigen::DenseIndex rows, Eigen::DenseIndex cols, Eigen::DenseIndex num_coefficients,
    const std::vector<double>& segment_times)
{
  typedef PiecewisePolynomial<CoefficientType> PiecewisePolynomialType;
  typedef typename PiecewisePolynomialType::PolynomialType PolynomialType;
  typedef typename PiecewisePolynomialType::PolynomialMatrix PolynomialMatrix;
  typedef typename PiecewisePolynomialType::CoefficientMatrix CoefficientMatrix;
  typedef typename PolynomialType::CoefficientsType CoefficientsType;

  size_t num_segments = segment_times.size() - 1;
  std::vector<PolynomialMatrix> polynomials;
  Eigen::DenseIndex size = rows * cols;
  for (Eigen::DenseIndex segment_index = 0; segment_index < num_segments; ++segment_index) {
    PolynomialMatrix matrix(rows, cols);
    for (Eigen::DenseIndex i = 0; i < size; i++) {
      CoefficientsType coefficients = CoefficientsType::Random(num_coefficients);
      matrix(i) = PolynomialType(coefficients);
    }
    polynomials.push_back(matrix);
  }
  return PiecewisePolynomial<CoefficientType>(polynomials, segment_times);
}

std::vector<double> generateRandomSegmentTimes(int num_segments, std::default_random_engine& generator);


#endif /* SYSTEMS_TRAJECTORIES_TEST_TRAJECTORYTESTUTIL_H_ */
