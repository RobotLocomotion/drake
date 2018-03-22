#include "drake/common/trajectories/piecewise_trajectory.h"

#include <random>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace trajectories {

namespace {

// Dummy implementation of PiecewiseTrajectory to test the basic indexing
// functions.
class PiecewiseTrajectoryTester : public PiecewiseTrajectory<double> {
 public:
  explicit PiecewiseTrajectoryTester(const std::vector<double>& times)
      : PiecewiseTrajectory<double>(times) { }
  Eigen::Index rows() const override { return 0; }
  Eigen::Index cols() const override { return 0; }
  Eigen::MatrixXd value(double t) const override {
    return Eigen::MatrixXd(0, 0);
  }
  std::unique_ptr<Trajectory<double>> Clone() const override {
    return nullptr;
  }
  std::unique_ptr<Trajectory<double>> MakeDerivative(int) const override {
    return nullptr;
  }
};

void TestPiecewiseTrajectoryTimeRelatedGetters(
    const PiecewiseTrajectoryTester& traj,
    const std::vector<double>& time) {
  EXPECT_EQ(traj.get_number_of_segments(), static_cast<int>(time.size()) - 1);

  EXPECT_EQ(traj.start_time(), time.front());
  EXPECT_EQ(traj.end_time(), time.back());

  // Check start / end / duration for each segment.
  for (int i = 0; i < static_cast<int>(time.size()) - 1; i++) {
    EXPECT_EQ(traj.start_time(i), time[i]);
    EXPECT_EQ(traj.end_time(i), time[i + 1]);
    EXPECT_EQ(traj.duration(i), time[i + 1] - time[i]);
  }

  // Check returned segment times == given.
  const std::vector<double>& ret_seg_times = traj.get_segment_times();
  EXPECT_EQ(ret_seg_times.size(), time.size());
  for (int i = 0; i < static_cast<int>(time.size()); i++) {
    EXPECT_EQ(time[i], ret_seg_times[i]);
  }

  // Check get_segment_index works at the breaks, also epsilon from the
  // breaks.
  for (int i = 0; i < static_cast<int>(time.size()); i++) {
    int idx = traj.get_segment_index(time[i]);
    if (i == static_cast<int>(time.size()) - 1) {
      EXPECT_EQ(idx, i - 1);
    } else {
      EXPECT_EQ(idx, i);
    }

    if (i != 0) {
      const double t_minus_eps = time[i] - 1e-10;
      idx = traj.get_segment_index(t_minus_eps);
      EXPECT_EQ(idx, i - 1);
      EXPECT_TRUE(traj.start_time(idx) < t_minus_eps);
    }
  }

  // Dense sample the time, and make sure the returned index is valid.
  for (double t = time.front() - 0.1; t < time.back() + 0.1; t += 0.01) {
    const int idx = traj.get_segment_index(t);
    EXPECT_GE(idx, 0);
    EXPECT_LT(idx, traj.get_number_of_segments());

    if (t >= traj.start_time() && t <= traj.end_time()) {
      EXPECT_TRUE(t >= traj.start_time(idx));
      EXPECT_TRUE(t <= traj.end_time(idx));
    }
  }
}

GTEST_TEST(PiecewiseTrajectoryTest, RandomizedGetIndexTest) {
  int N = 100000;
  std::default_random_engine generator(123);
  std::vector<double> time =
      PiecewiseTrajectory<double>::RandomSegmentTimes(N - 1, generator);

  PiecewiseTrajectoryTester traj(time);

  TestPiecewiseTrajectoryTimeRelatedGetters(traj, time);
}

GTEST_TEST(PiecewiseTrajectoryTest, GetIndexTest) {
  std::vector<double> time = {0, 1, 2, 3.5};

  PiecewiseTrajectoryTester traj(time);

  TestPiecewiseTrajectoryTimeRelatedGetters(traj, time);
}

}  // namespace
}  // namespace trajectories
}  // namespace drake
