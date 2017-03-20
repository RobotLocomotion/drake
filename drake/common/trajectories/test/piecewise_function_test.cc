#include "drake/common/trajectories/piecewise_function.h"

#include <random>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"

// Dummy implementation of PiecewiseFunction to test the basic indexing
// functions.
class PiecewiseFunctionTester : public PiecewiseFunction {
 public:
  explicit PiecewiseFunctionTester(const std::vector<double>& times)
      : PiecewiseFunction(times) { }
  Eigen::Index rows() const { return 0; }
  Eigen::Index cols() const { return 0; }
};

void TestPiecewiseFunctionTimeRelatedGetters(
    const PiecewiseFunctionTester& traj,
    const std::vector<double>& time) {
  EXPECT_EQ(traj.getNumberOfSegments(), static_cast<int>(time.size()) - 1);

  EXPECT_EQ(traj.getStartTime(), time.front());
  EXPECT_EQ(traj.getEndTime(), time.back());

  // Check start / end / duration for each segment.
  for (size_t i = 0; i < time.size() - 1; i++) {
    EXPECT_EQ(traj.getStartTime(i), time[i]);
    EXPECT_EQ(traj.getEndTime(i), time[i + 1]);
    EXPECT_EQ(traj.getDuration(i), time[i + 1] - time[i]);
  }

  // Check returned segment times == given.
  const std::vector<double>& ret_seg_times = traj.getSegmentTimes();
  EXPECT_EQ(ret_seg_times.size(), time.size());
  for (size_t i = 0; i < time.size(); i++) {
    EXPECT_EQ(time[i], ret_seg_times[i]);
  }

  // Check getSegmentIndex works at the breaks, also epsilon from the
  // breaks.
  for (int i = 0; i < static_cast<int>(time.size()); i++) {
    int idx = traj.getSegmentIndex(time[i]);
    if (i == static_cast<int>(time.size()) - 1) {
      EXPECT_EQ(idx, i - 1);
    } else {
      EXPECT_EQ(idx, i);
    }

    if (i != 0) {
      double t_minus_eps = time[i] - 1e-10;
      idx = traj.getSegmentIndex(t_minus_eps);
      EXPECT_TRUE(t_minus_eps <= time[i]);
      EXPECT_EQ(idx, i - 1);
      EXPECT_TRUE(traj.getStartTime(idx) < t_minus_eps);
    }
  }

  // Dense sample the time, and make sure the returned index is valid.
  for (double t = time.front() - 0.1; t < time.back() + 0.1; t += 0.01) {
    int idx = traj.getSegmentIndex(t);
    EXPECT_GE(idx, 0);
    EXPECT_LT(idx, traj.getNumberOfSegments());

    if (t >= traj.getStartTime() && t <= traj.getEndTime()) {
      EXPECT_TRUE(t >= traj.getStartTime(idx));
      EXPECT_TRUE(t <= traj.getEndTime(idx));
    }
  }
}

GTEST_TEST(PiecewiseFunctionTest, RandomizedGetIndexTest) {
  int N = 100000;
  std::default_random_engine generator(123);
  std::vector<double> time =
      PiecewiseFunction::randomSegmentTimes(N - 1, generator);

  PiecewiseFunctionTester traj(time);

  TestPiecewiseFunctionTimeRelatedGetters(traj, time);
}

GTEST_TEST(PiecewiseFunctionTest, GetIndexTest) {
  std::vector<double> time = {0, 1, 2, 3.5};

  PiecewiseFunctionTester traj(time);

  TestPiecewiseFunctionTimeRelatedGetters(traj, time);
}
