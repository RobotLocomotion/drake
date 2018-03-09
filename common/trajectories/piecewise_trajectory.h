#pragma once

#include <random>
#include <vector>

#include <Eigen/Core>

#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace trajectories {

/// Minimum delta quantity used for comparing time.
static constexpr double kEpsilonTime = 1e-10;

// @tparam T is the scalar type.  Explicit instantions are provided for
//  - double
template <typename T>
class PiecewiseTrajectory : public Trajectory<T> {
 public:
  /// @throws std::runtime_exception if `breaks` increments are
  /// smaller than kEpsilonTime.
  explicit PiecewiseTrajectory(std::vector<double> const& breaks);

  virtual ~PiecewiseTrajectory();

  int get_number_of_segments() const;

  double start_time(int segment_number) const;

  double end_time(int segment_number) const;

  double duration(int segment_number) const;

  double start_time() const override;

  double end_time() const override;

  /**
   * Returns true iff `t >= getStartTime() && t <= getEndTime()`.
   */
  bool is_time_in_range(double t) const;

  int get_segment_index(double t) const;

  const std::vector<double>& get_segment_times() const;

  void segment_number_range_check(int segment_number) const;

  static std::vector<double> RandomSegmentTimes(
      // TODO(#2274) Fix this NOLINTNEXTLINE(runtime/references)
      int num_segments, std::default_random_engine &generator);

 protected:
  bool SegmentTimesEqual(const PiecewiseTrajectory &b,
                         double tol = kEpsilonTime) const;

  PiecewiseTrajectory();

  const std::vector<double>& breaks() const { return breaks_; }
  std::vector<double>& get_mutable_breaks() { return breaks_; }

 private:
  int GetSegmentIndexRecursive(double time, int start, int end) const;

  std::vector<double> breaks_;
};

}  // namespace trajectories
}  // namespace drake
