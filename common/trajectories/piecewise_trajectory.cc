#include "drake/common/trajectories/piecewise_trajectory.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "drake/common/drake_assert.h"

using std::uniform_real_distribution;
using std::vector;

namespace drake {
namespace trajectories {

template <typename T>
PiecewiseTrajectory<T>::PiecewiseTrajectory(const std::vector<T>& breaks)
    : Trajectory<T>(), breaks_(breaks) {
  for (int i = 1; i < get_number_of_segments() + 1; i++) {
    DRAKE_DEMAND(breaks_[i] - breaks_[i - 1] >= kEpsilonTime);
  }
}

template <typename T>
boolean<T> PiecewiseTrajectory<T>::is_time_in_range(const T& time) const {
  return (time >= start_time() && time <= end_time());
}

template <typename T>
int PiecewiseTrajectory<T>::get_number_of_segments() const {
  return static_cast<int>(breaks_.size() > 0 ? breaks_.size() - 1 : 0);
}

template <typename T>
T PiecewiseTrajectory<T>::start_time(int segment_number) const {
  segment_number_range_check(segment_number);
  return breaks_[segment_number];
}

template <typename T>
T PiecewiseTrajectory<T>::end_time(int segment_number) const {
  segment_number_range_check(segment_number);
  return breaks_[segment_number + 1];
}

template <typename T>
T PiecewiseTrajectory<T>::duration(int segment_number) const {
  return end_time(segment_number) - start_time(segment_number);
}

template <typename T>
T PiecewiseTrajectory<T>::start_time() const {
  return start_time(0);
}

template <typename T>
T PiecewiseTrajectory<T>::end_time() const {
  return end_time(get_number_of_segments() - 1);
}

template <typename T>
int PiecewiseTrajectory<T>::GetSegmentIndexRecursive(const T& time, int start,
                                                     int end) const {
  DRAKE_DEMAND(end >= start);
  DRAKE_DEMAND(end < static_cast<int>(breaks_.size()));
  DRAKE_DEMAND(start >= 0);
  DRAKE_DEMAND(time <= breaks_[end] && time >= breaks_[start]);

  int mid = (start + end) / 2;

  // one or two numbers
  if (end - start <= 1) return start;

  if (time < breaks_[mid])
    return GetSegmentIndexRecursive(time, start, mid);
  else if (time > breaks_[mid])
    return GetSegmentIndexRecursive(time, mid, end);
  else
    return mid;
}

template <typename T>
int PiecewiseTrajectory<T>::get_segment_index(const T& t) const {
  if (breaks_.empty()) return 0;
  // clip to min/max times
  using std::min;
  using std::max;
  T time = min(max(t, start_time()), end_time());
  return GetSegmentIndexRecursive(time, 0,
                                  static_cast<int>(breaks_.size() - 1));
}

template <typename T>
const std::vector<T>& PiecewiseTrajectory<T>::get_segment_times() const {
  return breaks_;
}

template <typename T>
void PiecewiseTrajectory<T>::segment_number_range_check(
    int segment_number) const {
  if (segment_number < 0 || segment_number >= get_number_of_segments()) {
    std::stringstream msg;
    msg << "Segment number " << segment_number << " out of range [" << 0 << ", "
        << get_number_of_segments() << ")" << std::endl;
    throw std::runtime_error(msg.str().c_str());
  }
}

template <typename T>
std::vector<T> PiecewiseTrajectory<T>::RandomSegmentTimes(
    // TODO(#2274) Fix this NOLINTNEXTLINE(runtime/references)
    int num_segments, std::default_random_engine& generator) {
  vector<T> breaks;
  uniform_real_distribution<double> uniform(kEpsilonTime, 1);
  T t0 = uniform(generator);
  breaks.push_back(t0);
  for (int i = 0; i < num_segments; ++i) {
    T duration = uniform(generator);
    breaks.push_back(breaks[i] + duration);
  }
  return breaks;
}

template <typename T>
bool PiecewiseTrajectory<T>::SegmentTimesEqual(
    const PiecewiseTrajectory<T>& other, double tol) const {
  if (breaks_.size() != other.breaks_.size()) return false;
  using std::abs;
  for (size_t i = 0; i < breaks_.size(); i++) {
    if (abs(breaks_[i] - other.breaks_[i]) > tol) return false;
  }
  return true;
}

}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PiecewiseTrajectory)
