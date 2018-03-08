#include "drake/common/trajectories/piecewise_function.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "drake/common/drake_assert.h"

using std::uniform_real_distribution;
using std::vector;

namespace drake {
namespace trajectories {

PiecewiseFunction::PiecewiseFunction(
    std::vector<double> const& breaks)
    : breaks_(breaks) {
  for (int i = 1; i < getNumberOfSegments() + 1; i++) {
    if (breaks_[i] - breaks_[i - 1] < kEpsilonTime)
      throw std::runtime_error("times must be increasing.");
  }
}

PiecewiseFunction::PiecewiseFunction() {
  // empty
}

PiecewiseFunction::~PiecewiseFunction() {
  // empty
}

bool PiecewiseFunction::isTimeInRange(double time) const {
  return (time >= getStartTime() && time <= getEndTime());
}

int PiecewiseFunction::getNumberOfSegments() const {
  return static_cast<int>(breaks_.size() - 1);
}

double PiecewiseFunction::getStartTime(int segment_number) const {
  segmentNumberRangeCheck(segment_number);
  return breaks_[segment_number];
}

double PiecewiseFunction::getEndTime(int segment_number) const {
  segmentNumberRangeCheck(segment_number);
  return breaks_[segment_number + 1];
}

double PiecewiseFunction::getDuration(int segment_number) const {
  return getEndTime(segment_number) - getStartTime(segment_number);
}

double PiecewiseFunction::getStartTime() const { return getStartTime(0); }

double PiecewiseFunction::getEndTime() const {
  return getEndTime(getNumberOfSegments() - 1);
}

int PiecewiseFunction::GetSegmentIndexRecursive(
    double time, int start, int end) const {
  DRAKE_DEMAND(end >= start);
  DRAKE_DEMAND(end < static_cast<int>(breaks_.size()));
  DRAKE_DEMAND(start >= 0);
  DRAKE_DEMAND(time <= breaks_[end] && time >= breaks_[start]);

  int mid = (start + end) / 2;

  // one or two numbers
  if (end - start <= 1)
    return start;

  if (time < breaks_[mid])
    return GetSegmentIndexRecursive(time, start, mid);
  else if (time > breaks_[mid])
    return GetSegmentIndexRecursive(time, mid, end);
  else
    return mid;
}

int PiecewiseFunction::getSegmentIndex(double t) const {
  if (breaks_.empty())
    return 0;
  // clip to min/max times
  t = std::min(std::max(t, getStartTime()), getEndTime());
  return GetSegmentIndexRecursive(
      t, 0, static_cast<int>(breaks_.size() - 1));
}

const std::vector<double>& PiecewiseFunction::getSegmentTimes() const {
  return breaks_;
}

void PiecewiseFunction::segmentNumberRangeCheck(int segment_number) const {
  if (segment_number < 0 || segment_number >= getNumberOfSegments()) {
    std::stringstream msg;
    msg << "Segment number " << segment_number << " out of range [" << 0 << ", "
        << getNumberOfSegments() << ")" << std::endl;
    throw std::runtime_error(msg.str().c_str());
  }
}

std::vector<double> PiecewiseFunction::randomSegmentTimes(
    int num_segments, std::default_random_engine& generator) {
  vector<double> breaks;
  uniform_real_distribution<double> uniform;
  double t0 = uniform(generator);
  breaks.push_back(t0);
  for (int i = 0; i < num_segments; ++i) {
    double duration = uniform(generator);
    breaks.push_back(breaks[i] + duration);
  }
  return breaks;
}

bool PiecewiseFunction::segmentTimesEqual(const PiecewiseFunction& other,
                                          double tol) const {
  if (breaks_.size() != other.breaks_.size()) return false;
  for (size_t i = 0; i < breaks_.size(); i++) {
    if (std::abs(breaks_[i] - other.breaks_[i]) > tol) return false;
  }
  return true;
}

void PiecewiseFunction::checkScalarValued() const {
  if (rows() != 1 || cols() != 1) {
    throw std::runtime_error("Not scalar valued.");
  }
}

}  // namespace trajectories
}  // namespace drake
