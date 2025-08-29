#include "drake/systems/analysis/realtime_rate_calculator.h"

#include <ios>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace internal {

using UpdateResult = RealtimeRateCalculator::UpdateResult;

std::ostream& operator<<(std::ostream& out, const UpdateResult& report) {
  out << std::boolalpha << "{" << report.initialized << ", "
      << report.period_count << ", " << report.rate << "}";
  return out;
}

namespace {

// Matches UpdateResult `arg` with UpdateResult `ref` if their values match
// (with rate values considered "matching" within the specified tolerance).
MATCHER_P2(UpdateResultNear, tolerance, ref, "") {
  return arg.initialized == ref.initialized &&
         arg.period_count == ref.period_count &&
         std::abs(arg.rate - ref.rate) <= tolerance;
}

GTEST_TEST(RealtimeRateCalculatorTest, Calculation) {
  const double kPeriod = 1;
  RealtimeRateCalculator calculator(kPeriod);
  auto timer_ptr = std::make_unique<ManualTimer>();
  ManualTimer& timer = *timer_ptr;
  calculator.InjectMockTimer(std::move(timer_ptr));

  struct Sample {
    // Called t_s, below.
    double sim_time{};
    // Called t_w, below.
    double wall_time{};
    UpdateResult expected_report;
    std::string_view description;
  };

  // Note: the test below are articulated based on the report period being
  // 1 (it keeps the specification simpler). Each literal value is implicitly
  // that quantity times kPeriod.
  DRAKE_DEMAND(kPeriod == 1);

  // clang-format off

  // A progression of invocations to UpdateAndRecalculate. It includes:
  //
  // t_s   t_w     expected rate report     description
  const std::vector<Sample> samples{
    // First invocation, at garbage wall time, is initialization. It resets
    // timer.Tick() back to zero. So, the next wall time will be measured
    // relative to zero.
    {1,    123,   UpdateResult::Initialized(), "Initialization"},
    // "Repeated" invocation is no-op; t_w = 0 equals the reset timer value.
    {1,    0,     UpdateResult::Incomplete(), "Repeat sim time"},
    // Sim time advanced, but not enough wall time.
    {2,    0.75,    UpdateResult::Incomplete(), "Not enough wall time"},
    // Passed boundary at t_w = 1. rate = Δt_s / Δt_w = (3 - 1) / 1.5.
    {3,    1.5,     UpdateResult::Computed(1, 2 / 1.5), "Past first boundary"},
    // Advancing multiple periods will calculate a rate and indicate it applies
    // to multiple periods.
    //
    // The previous invocation advanced sim time t_s = 3 s at wall time
    // t_w = 1.5 s. The latest report period boundary is at t_w = 1 s. We clip
    // the sim and wall time to the portion in that full period interval,
    // giving residual times of 2/3 sim second and 0.5 wall seconds.
    //
    // t_w advances by 3.5 s (from 1.5 to 5.0). However, with the 0.5 s
    // residual, the total advancement is 4.0, so we should get exactly four
    // full periods. t_s advances 3 seconds (with 1/3 s residual), giving us a
    // rate of Δt_s / Δt_w = 3 1/3 / 4 = 11 / 12.
    {6,    5,       UpdateResult::Computed(4, 11 / 12.0), "Four boundaries"},
    // Another advancement with insufficient wall time computes no rate.
    {6.01, 5.11,    UpdateResult::Incomplete(), "Infinitesimal step"},
    // Multi-period report doesn't interfere with the next boundary.
    {7,    6,       UpdateResult::Computed(1, 1), "On report boundary exactly"},
    // Implicit reset; previous t_s = 2, t_w gets reset to 0.
    {2,    10.5,    UpdateResult::Initialized(), "Implicit initialize"},
    // t_w = 0.99 is less than the previous wall time. Ordinarily, that would
    // be "bad", but it's fine with the reset.
    {2.9,  0.99,    UpdateResult::Incomplete(), "Not far enough after re-init"},
    // Advance to the boundary.
    {3,    1.0,     UpdateResult::Computed(1, 1), "New boundary after re-init"},
  };
  // clang-format on

  for (const auto& sample : samples) {
    timer.set_tick(sample.wall_time);
    auto report = calculator.UpdateAndRecalculate(sample.sim_time);
    EXPECT_THAT(report, UpdateResultNear(1e-14, sample.expected_report));
    if (sample.expected_report.initialized) {
      // Initialization should always call Timer::Start().
      EXPECT_EQ(timer.Tick(), 0);
    }
  }
}

// Confirm that setting the time to negative infinity "works" in the sense that
// -infinity counts as a time earlier than previous times for the purpose of
// re-initializing the calculator, but subsequent calls with finite, increasing
// times recalibrate to meaningful rate values.
GTEST_TEST(RealtimeRateCalculatorTest, NegativeInfinity) {
  const double kPeriod = 1;
  RealtimeRateCalculator calculator(kPeriod);
  auto timer_ptr = std::make_unique<ManualTimer>();
  ManualTimer& timer = *timer_ptr;
  calculator.InjectMockTimer(std::move(timer_ptr));

  // A reality check that we can advance time and get a rate.
  timer.set_tick(0);
  auto report = calculator.UpdateAndRecalculate(0);
  ASSERT_TRUE(report.initialized);
  // Advance wall clock and sim time in lock step; real time rate = 1.
  timer.set_tick(kPeriod);
  report = calculator.UpdateAndRecalculate(kPeriod);
  ASSERT_FALSE(report.initialized);
  EXPECT_DOUBLE_EQ(report.rate, 1.0);
  EXPECT_EQ(report.period_count, 1);

  // No set time to infinity; this should be an initialization event and a
  // subsequent elapsed period should produce an infinite rate.
  timer.set_tick(timer.Tick() + kPeriod);
  report =
      calculator.UpdateAndRecalculate(-std::numeric_limits<double>::infinity());
  ASSERT_TRUE(report.initialized);

  // After two elapsed periods, we should report two infinite rates.
  timer.set_tick(timer.Tick() + 2 * kPeriod);
  report = calculator.UpdateAndRecalculate(0);
  ASSERT_FALSE(report.initialized);
  EXPECT_DOUBLE_EQ(report.rate, std::numeric_limits<double>::infinity());
  EXPECT_EQ(report.period_count, 2);

  // After another period, we should be back to real rate measurements.
  timer.set_tick(timer.Tick() + kPeriod);
  report = calculator.UpdateAndRecalculate(kPeriod);
  ASSERT_FALSE(report.initialized);
  EXPECT_DOUBLE_EQ(report.rate, 1);
  EXPECT_EQ(report.period_count, 1);
}

// Confirms that when Nan is passed, an empty result is reported.
GTEST_TEST(RealtimeRateCalculatorTest, NanNonsense) {
  const double kPeriod = 1;
  RealtimeRateCalculator calculator(kPeriod);
  auto timer_ptr = std::make_unique<ManualTimer>();
  ManualTimer& timer = *timer_ptr;
  calculator.InjectMockTimer(std::move(timer_ptr));

  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

  // Initializing with NaN does *not* initialize.
  auto report = calculator.UpdateAndRecalculate(kNan);
  ASSERT_FALSE(report.initialized);
  ASSERT_EQ(report.period_count, 0);

  // We likewise get an empty report if we're already initialized and pass NaN.
  report = calculator.UpdateAndRecalculate(0);
  ASSERT_TRUE(report.initialized);

  timer.set_tick(kPeriod * 2);
  report = calculator.UpdateAndRecalculate(kNan);
  ASSERT_FALSE(report.initialized);
  ASSERT_EQ(report.period_count, 0);
}

}  // namespace
}  // namespace internal
}  // namespace systems
}  // namespace drake
