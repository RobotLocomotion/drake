#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput_railcar.h"
#include "drake/common/call_matlab.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

namespace drake {
namespace automotive {
namespace {

class TrajectoryOptimizationRailcarTest : public ::testing::Test {
 protected:
  void SetUp() {
    // Defines the dragway's parameters.
    const int kNumLanes{1};
    const double kDragwayLength{50};
    const double kDragwayLaneWidth{0.5};
    const double kDragwayShoulderWidth{0.25};
    road_.reset(new maliput::dragway::RoadGeometry(
        maliput::api::RoadGeometryId({"Dragway"}), kNumLanes, kDragwayLength,
        kDragwayLaneWidth, kDragwayShoulderWidth));
    const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
    plant_.reset(new MaliputRailcar<double>(
        LaneDirection(lane, true /* with_s */), true /* has_single_lane */));
    context_ = plant_->CreateDefaultContext();
  }

  std::unique_ptr<const maliput::api::RoadGeometry> road_;
  std::unique_ptr<MaliputRailcar<double>> plant_;
  std::unique_ptr<systems::Context<double>> context_;
};

// Sets up a simple trajectory optimization problem uses MaliputRailcar .....
TEST_F(TrajectoryOptimizationRailcarTest, SimpleCarDircolTest) {
  EXPECT_TRUE(context_->has_only_continuous_state());

  MaliputRailcarState<double> x0;
  MaliputRailcarState<double> xf;

  x0.set_s(0.0);
  x0.set_speed(15.0);  // m/s = ~ 33mph

  const double initial_duration = 30.0;  // seconds
  xf.set_s(x0.s() + initial_duration * x0.speed());
  xf.set_speed(x0.speed());

  const int kNumTimeSamples = 10;

  // The solved trajectory may deviate from the initial guess at a reasonable
  // duration.
  const double kTrajectoryTimeLowerBound = 0.8 * initial_duration;
  const double kTrajectoryTimeUpperBound = 1.2 * initial_duration;

  systems::DircolTrajectoryOptimization prog(plant_.get(), *context_,
                                             kNumTimeSamples,
                                             kTrajectoryTimeLowerBound,
                                             kTrajectoryTimeUpperBound);

  // Limits on the acceleration input value.
  auto lower_limit = systems::BasicVector<double>::Make(
      -std::numeric_limits<double>::infinity());
  auto upper_limit = systems::BasicVector<double>::Make(
      std::numeric_limits<double>::infinity());
  prog.AddInputBounds(lower_limit->get_value(), upper_limit->get_value());

  // Ensure that time intervals are (relatively) evenly spaced.
  prog.AddTimeIntervalBounds(kTrajectoryTimeLowerBound / (kNumTimeSamples - 1),
                             kTrajectoryTimeUpperBound / (kNumTimeSamples - 1));

  // Fix initial conditions.
  prog.AddLinearConstraint(prog.initial_state() == x0.get_value());

  // Fix final conditions.
  prog.AddLinearConstraint(prog.final_state() == xf.get_value());

  // Cost function: int_0^T [ u'u ] dt.
  prog.AddRunningCost(prog.input().transpose() * prog.input());

  // Initial guess is a straight line from the initial state to the final state.
  auto initial_state_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      {0, initial_duration}, {x0.get_value(), xf.get_value()});

  solvers::SolutionResult result =
      prog.SolveTraj(initial_duration, PiecewisePolynomial<double>(),
                     initial_state_trajectory);

  solvers::SolverType solver;
  int solver_result;
  prog.GetSolverResult(&solver, &solver_result);

  if (solver == solvers::SolverType::kIpopt) {
    EXPECT_EQ(result,
              solvers::SolutionResult::kIterationLimit);  // TODO(russt): Tune
                                                          // Ipopt for this
                                                          // example.
  } else {
    EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound);
  }

  // Plot the solution.
  // Note: see lcm_call_matlab.h for instructions on viewing the plot.
  Eigen::MatrixXd inputs;
  Eigen::MatrixXd states;
  std::vector<double> times_out;
  prog.GetResultSamples(&inputs, &states, &times_out);
  // common::CallMatlab("plot", states.row(MaliputRailcarStateIndices::kX),
  //                   states.row(MaliputRailcarStateIndices::kY));
  // common::CallMatlab("xlabel", "x (m)");
  // common::CallMatlab("ylabel", "y (m)");

  // Checks that the input commands found are not too large.
  EXPECT_LE(inputs.row(0).lpNorm<1>(), 0.1);
  EXPECT_LE(inputs.row(1).lpNorm<1>(), 1);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
