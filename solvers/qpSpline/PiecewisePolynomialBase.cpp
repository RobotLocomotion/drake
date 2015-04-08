#include "PiecewisePolynomialBase.h"

PiecewisePolynomialBase::PiecewisePolynomialBase(std::vector<double> const & segment_times) :
  segment_times(segment_times)
{
  for (int i = 0; i < getNumberOfSegments(); i++) {
    if (segment_times[i + 1] < segment_times[i])
      throw std::runtime_error("times must be increasing");
  }
}

PiecewisePolynomialBase::~PiecewisePolynomialBase()
{
  // empty
}

int PiecewisePolynomialBase::getNumberOfSegments() const {
  return static_cast<int>(segment_times.size() - 1);
}

int PiecewisePolynomialBase::getNumberOfCoefficients(int segment_number) const {
  return getSegmentPolynomialOrder(segment_number) + 1;
}

double PiecewisePolynomialBase::getStartTime(int segment_number) const {
  return segment_times[segment_number];
}

double PiecewisePolynomialBase::getEndTime(int segment_number) const {
  return segment_times[segment_number + 1];
}

double PiecewisePolynomialBase::getStartTime() const {
  return getStartTime(0);
}

double PiecewisePolynomialBase::getEndTime() const {
  return getEndTime(getNumberOfSegments() - 1);
}

int PiecewisePolynomialBase::getTotalNumberOfCoefficients() const {
  int ret = 0;

  for (int i = 0; i < getNumberOfSegments(); i++) {
    ret += getNumberOfCoefficients(i);
  }
  return ret;
}

