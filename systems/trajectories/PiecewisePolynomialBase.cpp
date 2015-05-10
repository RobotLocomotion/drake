#include "PiecewisePolynomialBase.h"
#include <stdexcept>
#include <sstream>
#include <cmath>

PiecewisePolynomialBase::PiecewisePolynomialBase(std::vector<double> const & segment_times) :
  PiecewiseFunction(segment_times)
{
  // empty
}

PiecewisePolynomialBase::PiecewisePolynomialBase() {
  // empty
}

PiecewisePolynomialBase::~PiecewisePolynomialBase()
{
  // empty
}

int PiecewisePolynomialBase::getNumberOfCoefficients(int segment_number, Eigen::DenseIndex row, Eigen::DenseIndex col) const {
  return getSegmentPolynomialDegree(segment_number, row, col) + 1;
}

int PiecewisePolynomialBase::getTotalNumberOfCoefficients(Eigen::DenseIndex row, Eigen::DenseIndex col) const {
  int ret = 0;

  for (int i = 0; i < getNumberOfSegments(); i++) {
    ret += getNumberOfCoefficients(i, row, col);
  }
  return ret;
}
