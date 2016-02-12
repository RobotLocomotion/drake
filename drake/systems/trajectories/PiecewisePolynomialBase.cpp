#include "drake/systems/trajectories/PiecewisePolynomialBase.h"
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

int PiecewisePolynomialBase::getNumberOfCoefficients(int segment_number, Eigen::Index row, Eigen::Index col) const {
  return getSegmentPolynomialDegree(segment_number, row, col) + 1;
}

int PiecewisePolynomialBase::getTotalNumberOfCoefficients(Eigen::Index row, Eigen::Index col) const {
  int ret = 0;

  for (int i = 0; i < getNumberOfSegments(); i++) {
    ret += getNumberOfCoefficients(i, row, col);
  }
  return ret;
}
