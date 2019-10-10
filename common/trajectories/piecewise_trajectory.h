#pragma once

#include <memory>
#include <random>
#include <vector>

#include <Eigen/Core>

#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace trajectories {

/// Abstract class that implements the basic logic of maintaining consequent
/// segments of time (delimited by `breaks`) to implement a trajectory that
/// is represented by simpler logic in each segment or "piece".
///
/// @tparam T is the scalar type.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
template <typename T>
class PiecewiseTrajectory : public Trajectory<T> {
 public:
  // TODO(ggould-tri) This quantity is surprisingly large and never justified.
  /// Minimum delta quantity used for comparing time.
  static constexpr double kEpsilonTime = 1e-10;

  ~PiecewiseTrajectory() override = default;

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
  // Final subclasses are allowed to make copy/move/assign public.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PiecewiseTrajectory)
  PiecewiseTrajectory() = default;

  /// @p breaks increments must be greater or equal to kEpsilonTime.
  explicit PiecewiseTrajectory(const std::vector<double>& breaks);

  bool SegmentTimesEqual(const PiecewiseTrajectory& b,
                         double tol = kEpsilonTime) const;

  const std::vector<double>& breaks() const { return breaks_; }
  std::vector<double>& get_mutable_breaks() { return breaks_; }

 private:
  int GetSegmentIndexRecursive(double time, int start, int end) const;

  std::vector<double> breaks_;
};

}  // namespace trajectories
}  // namespace drake
