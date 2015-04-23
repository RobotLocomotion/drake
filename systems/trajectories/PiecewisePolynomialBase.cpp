#include "PiecewisePolynomialBase.h"
#include <stdexcept>
#include <sstream>
#include <cmath>

PiecewisePolynomialBase::PiecewisePolynomialBase(std::vector<double> const & segment_times) :
  segment_times(segment_times)
{
  for (int i = 0; i < getNumberOfSegments(); i++) {
    if (segment_times[i + 1] < segment_times[i])
      throw std::runtime_error("times must be increasing");
  }
}

PiecewisePolynomialBase::PiecewisePolynomialBase() {
  // empty
}

PiecewisePolynomialBase::~PiecewisePolynomialBase()
{
  // empty
}

int PiecewisePolynomialBase::getNumberOfSegments() const {
  return static_cast<int>(segment_times.size() - 1);
}

int PiecewisePolynomialBase::getNumberOfCoefficients(int segment_number, Eigen::DenseIndex row, Eigen::DenseIndex col) const {
  return getSegmentPolynomialDegree(segment_number, row, col) + 1;
}

double PiecewisePolynomialBase::getStartTime(int segment_number) const {
  segmentNumberRangeCheck(segment_number);
  return segment_times[segment_number];
}

double PiecewisePolynomialBase::getEndTime(int segment_number) const {
  segmentNumberRangeCheck(segment_number);
  return segment_times[segment_number + 1];
}

double PiecewisePolynomialBase::getDuration(int segment_number) const {
  return getEndTime(segment_number) - getStartTime(segment_number);
}

double PiecewisePolynomialBase::getStartTime() const {
  return getStartTime(0);
}

double PiecewisePolynomialBase::getEndTime() const {
  return getEndTime(getNumberOfSegments() - 1);
}

int PiecewisePolynomialBase::getTotalNumberOfCoefficients(Eigen::DenseIndex row, Eigen::DenseIndex col) const {
  int ret = 0;

  for (int i = 0; i < getNumberOfSegments(); i++) {
    ret += getNumberOfCoefficients(i, row, col);
  }
  return ret;
}

int PiecewisePolynomialBase::getSegmentIndex(double t) const {
  // clip to min/max times
  if (t < getStartTime())
    t = getStartTime();

  if (t > getEndTime())
    t = getEndTime();

  int segment_index = 0;
  while (t >= getEndTime(segment_index) && segment_index < getNumberOfSegments() - 1)
    segment_index++;

  return segment_index;
}

void PiecewisePolynomialBase::segmentNumberRangeCheck(int segment_number) const {
  if (segment_number < 0 || segment_number >= getNumberOfSegments()) {
    std::stringstream msg;
    msg << "Segment number " << segment_number << " out of range [" << 0 << ", " << getNumberOfSegments() - 1 << "]" << std::endl;
    throw std::runtime_error(msg.str().c_str());
  }
}

bool PiecewisePolynomialBase::segmentTimesEqual(const PiecewisePolynomialBase& other, double tol) const {
  if (segment_times.size() != other.segment_times.size())
    return false;
  for (int i = 0; i < segment_times.size(); i++) {
    if (std::abs(segment_times[i] - other.segment_times[i]) > tol)
      return false;
  }
  return true;
}

void PiecewisePolynomialBase::checkScalarValued() const
{
  if (rows() != 1 || cols() != 1) {
    throw std::runtime_error("Not scalar valued.");
  }
}
