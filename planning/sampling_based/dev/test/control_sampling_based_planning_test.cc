#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <common_robotics_utilities/math.hpp>
#include <gtest/gtest.h>

#include "drake/common/text_logging.h"
#include "drake/planning/sampling_based/dev/asymmetric_planning_space.h"
#include "drake/planning/sampling_based/dev/goal_checker.h"
#include "drake/planning/sampling_based/dev/goal_sampler.h"
#include "drake/planning/sampling_based/dev/parallel_rrt_planner.h"
#include "drake/planning/sampling_based/dev/rrt_planner.h"

namespace drake {
namespace planning {
namespace {
using common_robotics_utilities::math::ContinuousRevoluteDistance;
using common_robotics_utilities::math::EnforceContinuousRevoluteBounds;
using common_robotics_utilities::math::Interpolate;
using CarPlanningState = ControlPlanningState<Eigen::Vector3d>;

// Define a (very basic) planning space for car control planning. Only methods
// required for basic control RRT planning are implemented.
// TODO(calderpg) augment this with an example using Drake simulation for
// forward propagation.
class CarControlPlanningSpace
    : public AsymmetricPlanningSpace<CarPlanningState> {
 public:
  // The copy constructor is private for use in implementing Clone().
  // Does not allow copy, move, or assignment.
  CarControlPlanningSpace(CarControlPlanningSpace&&) = delete;
  CarControlPlanningSpace& operator=(const CarControlPlanningSpace&) = delete;
  CarControlPlanningSpace& operator=(CarControlPlanningSpace&&) = delete;

  explicit CarControlPlanningSpace(uint64_t seed)
      : AsymmetricPlanningSpace<CarPlanningState>(seed, Parallelism::Max()) {}

 private:
  CarControlPlanningSpace(const CarControlPlanningSpace& other) = default;

  std::unique_ptr<PlanningSpace<CarPlanningState>> DoClone() const override {
    return std::unique_ptr<CarControlPlanningSpace>(
        new CarControlPlanningSpace(*this));
  }

  double DoNearestNeighborDistanceForwards(
      const CarPlanningState& from, const CarPlanningState& to) const override {
    return ApproximateStateDistance(from.state(), to.state());
  }

  double DoNearestNeighborDistanceBackwards(
      const CarPlanningState& from, const CarPlanningState& to) const override {
    throw std::runtime_error("CarControlPlanningSpace is forwards-only");
  }

  double DoStateDistanceForwards(
      const CarPlanningState& from, const CarPlanningState& to) const override {
    // Note: only an approximate distance function is required for goal checks.
    return ApproximateStateDistance(from.state(), to.state());
  }

  double DoStateDistanceBackwards(
      const CarPlanningState& from, const CarPlanningState& to) const override {
    throw std::runtime_error("CarControlPlanningSpace is forwards-only");
  }

  CarPlanningState DoInterpolateForwards(
      const CarPlanningState& from, const CarPlanningState& to,
      double ratio) const override {
    throw std::runtime_error(
        "CarControlPlanningSpace doesn't support forwards interpolation");
  }

  CarPlanningState DoInterpolateBackwards(
      const CarPlanningState& from, const CarPlanningState& to,
      double ratio) const override {
    throw std::runtime_error(
        "CarControlPlanningSpace doesn't support backwards interpolation");
  }

  std::vector<CarPlanningState> DoPropagateForwards(
      const CarPlanningState& from, const CarPlanningState& to,
      std::map<std::string, double>* propagation_statistics,
      int thread_number) override {
    // Sample a control input.
    const Eigen::VectorXd control = SampleControl(thread_number);

    // This is a very simple iterative approximation of car kinematics.
    // Vehicle and propagation parameters.
    const double step_size = 0.005;
    const double wheelbase = 0.025;
    Eigen::Vector3d propagated_xytheta = from.state();
    const int32_t num_steps =
        static_cast<int32_t>(std::ceil(std::abs(control(1)) / step_size));
    const double step_distance = control(1) / static_cast<double>(num_steps);
    // Since steering angle is fixed for the propagation, the orientation change
    // is the same at every step.
    double thetadot = 0.0;
    if (control(0) > 0.0) {
      const double radius = wheelbase / std::tan(std::abs(control(0)));
      const double total_angle_turned = control(1) / radius;
      thetadot = total_angle_turned / static_cast<double>(num_steps);
    } else if (control(0) < 0.0) {
      const double radius = wheelbase / std::tan(std::abs(control(0)));
      const double total_angle_turned = control(1) / radius;
      thetadot = -(total_angle_turned / static_cast<double>(num_steps));
    }
    bool propagation_valid = true;
    for (int32_t step = 1; step < num_steps; step++) {
      Eigen::Vector3d step_dot;
      step_dot(0) = step_distance * std::cos(propagated_xytheta(2));
      step_dot(1) = step_distance * std::sin(propagated_xytheta(2));
      step_dot(2) = thetadot;
      propagated_xytheta += step_dot;
      propagated_xytheta(2) =
          EnforceContinuousRevoluteBounds(propagated_xytheta(2));
      if (!DoCheckCarStateValidity(propagated_xytheta, thread_number)) {
        propagation_valid = false;
        break;
      }
    }
    if (propagation_valid) {
      return {CarPlanningState(propagated_xytheta, control)};
    } else {
      return {};
    }
  }

  std::vector<CarPlanningState> DoPropagateBackwards(
      const CarPlanningState& from, const CarPlanningState& to,
      std::map<std::string, double>* propagation_statistics,
      int thread_number) override {
    throw std::runtime_error("CarControlPlanningSpace is forwards-only");
  }

  double DoMotionCostForwards(
      const CarPlanningState& from, const CarPlanningState& to) const override {
    throw std::runtime_error(
        "CarControlPlanningSpace doesn't support forwards motion cost");
  }

  double DoMotionCostBackwards(
      const CarPlanningState& from, const CarPlanningState& to) const override {
    throw std::runtime_error(
        "CarControlPlanningSpace doesn't support backwards motion cost");
  }

  bool DoCheckStateValidity(
      const CarPlanningState& state, int thread_number) const override {
    return DoCheckCarStateValidity(state.state(), thread_number);
  }

  bool DoCheckEdgeValidity(
      const CarPlanningState& from, const CarPlanningState& to,
      int thread_number) const override {
    throw std::runtime_error(
        "CarControlPlanningSpace doesn't support edge validity");
  }

  CarPlanningState DoSampleState(int thread_number) override {
    Eigen::Vector3d sampled_state;
    for (int index = 0; index < sampled_state.size(); ++index) {
      const double lower = lower_bounds()(index);
      const double upper = upper_bounds()(index);
      const double ratio = random_source().DrawUniformUnitReal(thread_number);
      sampled_state(index) = Interpolate(lower, upper, ratio);
    }
    return CarPlanningState(sampled_state);
  }

  double ApproximateStateDistance(
      const Eigen::Vector3d& from, const Eigen::Vector3d& to) const {
    const double x_dist = std::abs(from(0) - to(0));
    const double y_dist = std::abs(from(1) - to(1));
    const double theta_dist = ContinuousRevoluteDistance(from(2), to(2));
    const double translation_distance =
        std::sqrt(std::pow(x_dist, 2.0) + std::pow(y_dist, 2.0));
    return translation_distance + (theta_dist * 0.1);
  }

  Eigen::VectorXd SampleControl(int thread_number) {
    Eigen::VectorXd sampled_control(2);
    for (int index = 0; index < sampled_control.size(); ++index) {
      const double lower = control_lower_bounds()(index);
      const double upper = control_upper_bounds()(index);
      const double ratio = random_source().DrawUniformUnitReal(thread_number);
      sampled_control(index) = Interpolate(lower, upper, ratio);
    }
    return sampled_control;
  }

  bool DoCheckCarStateValidity(
      const Eigen::Vector3d& xytheta, int) const {
    // Enforce bounds.
    if (xytheta(0) < lower_bounds()(0) || xytheta(0) > upper_bounds()(0)) {
      return false;
    } else if (xytheta(1) < lower_bounds()(1) ||
               xytheta(1) > upper_bounds()(1)) {
      return false;
    }
    // Obstacle in the middle.
    if (xytheta(0) > 0.2 && xytheta(0) < 0.8 &&
        xytheta(1) > 0.2 && xytheta(1) < 0.8) {
      return false;
    }
    return true;
  }

  const Eigen::Vector3d& lower_bounds() const { return lower_bounds_; }

  const Eigen::Vector3d& upper_bounds() const { return upper_bounds_; }

  const Eigen::VectorXd& control_lower_bounds() const {
    return control_lower_bounds_;
  }

  const Eigen::VectorXd& control_upper_bounds() const {
    return control_upper_bounds_;
  }

  // Bounds are for x, y, θ of a planar vehicle in a 1m x 1m x 1m environment.
  const Eigen::Vector3d lower_bounds_ = Eigen::Vector3d(0.0, 0.0, -M_PI);
  const Eigen::Vector3d upper_bounds_ = Eigen::Vector3d(1.0, 1.0, M_PI);

  // Control is steering angle and distance.
  const Eigen::VectorXd control_lower_bounds_ =
      (Eigen::VectorXd(2) << -M_PI_4, 0.001).finished();
  const Eigen::VectorXd control_upper_bounds_ =
      (Eigen::VectorXd(2) << M_PI_4, 0.2).finished();
};

// Define a basic goal checker.
class CarGoalChecker final : public GoalChecker<CarPlanningState> {
 public:
  CarGoalChecker() = default;

 private:
  bool DoCheckGoalReached(const CarPlanningState& candidate, int) const final {
    // Goal region is x in [0.8, 1.0], y in [0.8, 1.0].
    return (candidate.state()(0) >= 0.8) && (candidate.state()(0) <= 1.0) &&
           (candidate.state()(1) >= 0.8) && (candidate.state()(1) <= 1.0);
  }
};

// Define a basic goal sampler.
class CarGoalSampler final : public GoalSampler<CarPlanningState> {
 public:
  CarGoalSampler() = default;

 private:
  CarPlanningState DoSample(std::mt19937_64* generator, int) const final {
    Eigen::Vector3d sampled_goal_state;
    for (int index = 0; index < sampled_goal_state.size(); ++index) {
      const double lower = goal_lower_bounds_(index);
      const double upper = goal_upper_bounds_(index);
      const double ratio = DrawUniformUnitReal(generator);
      sampled_goal_state(index) = Interpolate(lower, upper, ratio);
    }
    return CarPlanningState(sampled_goal_state);
  }

  const Eigen::Vector3d goal_lower_bounds_ = Eigen::Vector3d(0.8, 0.8, -M_PI);
  const Eigen::Vector3d goal_upper_bounds_ = Eigen::Vector3d(1.0, 1.0, M_PI);
};

// TODO(calderpg) Replace/augment with an example task that uses drake.
GTEST_TEST(ControlSamplingBasedPlanningTest, StartAndGoalTest) {
  // Create planning space.
  const uint64_t prng_seed = 42;
  CarControlPlanningSpace planning_space(prng_seed);

  // Start and goal states.
  const CarPlanningState start(Eigen::Vector3d(0.1, 0.2, 0.0));
  const CarPlanningState goal(Eigen::Vector3d(0.9, 0.9, 0.0));

  // Planner parameters.
  RRTPlanner<CarPlanningState>::Parameters parameters;
  parameters.goal_sampling_bias = 0.15;
  parameters.p_goal_sample_is_new = 0.0;  // Not used with fixed goal states.
  parameters.goal_tolerance = 0.025;
  parameters.nearest_neighbor_parallelism = Parallelism::None();
  parameters.time_limit = 300.0;
  parameters.tree_growth_limit = 50000;
  parameters.planner_log_level = spdlog::level::info;

  // Plan a path.
  const auto result = RRTPlanner<CarPlanningState>::Plan(
      start, goal, parameters, &planning_space);
  ASSERT_TRUE(result.has_solution());
}

GTEST_TEST(ControlSamplingBasedPlanningTest, StartAndGoalSamplingTest) {
  // Create planning space.
  const uint64_t prng_seed = 42;
  CarControlPlanningSpace planning_space(prng_seed);

  // Start state.
  const CarPlanningState start(Eigen::Vector3d(0.1, 0.2, 0.0));

  // Goal sampler.
  const CarGoalSampler goal_sampler;

  // Planner parameters.
  RRTPlanner<CarPlanningState>::Parameters parameters;
  parameters.goal_sampling_bias = 0.15;
  parameters.p_goal_sample_is_new = 0.5;
  parameters.goal_tolerance = 0.025;
  parameters.nearest_neighbor_parallelism = Parallelism::None();
  parameters.time_limit = 300.0;
  parameters.tree_growth_limit = 1000;
  parameters.planner_log_level = spdlog::level::info;

  // Plan a path.
  const auto result = RRTPlanner<CarPlanningState>::PlanGoalSampling(
      start, goal_sampler, parameters, &planning_space);
  ASSERT_TRUE(result.has_solution());
}

GTEST_TEST(ControlSamplingBasedPlanningTest, StartAndGoalCheckTest) {
  // Create planning space.
  const uint64_t prng_seed = 42;
  CarControlPlanningSpace planning_space(prng_seed);

  // Start state.
  const CarPlanningState start(Eigen::Vector3d(0.1, 0.2, 0.0));

  // Goal check function.
  const CarGoalChecker goal_checker;

  // Planner parameters.
  RRTPlanner<CarPlanningState>::Parameters parameters;
  parameters.goal_sampling_bias = 0.0;  // Not used with a goal check.
  parameters.p_goal_sample_is_new = 0.0;  // Not used with a goal check.
  parameters.goal_tolerance = 0.0;  // Not used with a goal check.
  parameters.nearest_neighbor_parallelism = Parallelism::None();
  parameters.time_limit = 300.0;
  parameters.tree_growth_limit = 1000;
  parameters.planner_log_level = spdlog::level::info;

  // Plan a path.
  const auto result = RRTPlanner<CarPlanningState>::PlanGoalCheck(
      start, goal_checker, parameters, &planning_space);
  ASSERT_TRUE(result.has_solution());
}

GTEST_TEST(ControlSamplingBasedPlanningTest, ParallelStartAndGoalTest) {
  // Create planning space.
  const uint64_t prng_seed = 42;
  CarControlPlanningSpace planning_space(prng_seed);

  // Start and goal states.
  const CarPlanningState start(Eigen::Vector3d(0.1, 0.2, 0.0));
  const CarPlanningState goal(Eigen::Vector3d(0.9, 0.9, 0.0));

  // Planner parameters.
  ParallelRRTPlanner<CarPlanningState>::Parameters parameters;
  parameters.goal_sampling_bias = 0.15;
  parameters.p_goal_sample_is_new = 0.0;  // Not used with fixed goal states.
  parameters.goal_tolerance = 0.025;
  parameters.num_workers = 2;
  // This is an unusually low value to exercise tree swapping in test.
  parameters.initial_tree_capacity = 10;
  parameters.nearest_neighbor_parallelism = Parallelism::None();
  parameters.time_limit = 300.0;
  parameters.tree_growth_limit = 50000;
  parameters.planner_log_level = spdlog::level::info;

  // Plan a path.
  const auto result = ParallelRRTPlanner<CarPlanningState>::Plan(
      start, goal, parameters, &planning_space);
  ASSERT_TRUE(result.has_solution());
}

GTEST_TEST(ControlSamplingBasedPlanningTest, ParallelStartAndGoalSamplingTest) {
  // Create planning space.
  const uint64_t prng_seed = 42;
  CarControlPlanningSpace planning_space(prng_seed);

  // Start state.
  const CarPlanningState start(Eigen::Vector3d(0.1, 0.2, 0.0));

  // Goal sampler.
  const CarGoalSampler goal_sampler;

  // Planner parameters.
  ParallelRRTPlanner<CarPlanningState>::Parameters parameters;
  parameters.goal_sampling_bias = 0.15;
  parameters.p_goal_sample_is_new = 0.5;
  parameters.goal_tolerance = 0.025;
  parameters.num_workers = 2;
  // This is an unusually low value to exercise tree swapping in test.
  parameters.initial_tree_capacity = 10;
  parameters.nearest_neighbor_parallelism = Parallelism::None();
  parameters.time_limit = 300.0;
  parameters.tree_growth_limit = 1000;
  parameters.planner_log_level = spdlog::level::info;

  // Plan a path.
  const auto result = ParallelRRTPlanner<CarPlanningState>::PlanGoalSampling(
      start, goal_sampler, parameters, &planning_space);
  ASSERT_TRUE(result.has_solution());
}

GTEST_TEST(ControlSamplingBasedPlanningTest, ParallelStartAndGoalCheckTest) {
  // Create planning space.
  const uint64_t prng_seed = 42;
  CarControlPlanningSpace planning_space(prng_seed);

  // Start state.
  const CarPlanningState start(Eigen::Vector3d(0.1, 0.2, 0.0));

  // Goal check function.
  const CarGoalChecker goal_checker;

  // Planner parameters.
  ParallelRRTPlanner<CarPlanningState>::Parameters parameters;
  parameters.goal_sampling_bias = 0.0;  // Not used with a goal check.
  parameters.p_goal_sample_is_new = 0.0;  // Not used with a goal check.
  parameters.goal_tolerance = 0.0;  // Not used with a goal check.
  parameters.num_workers = 2;
  // This is an unusually low value to exercise tree swapping in test.
  parameters.initial_tree_capacity = 10;
  parameters.nearest_neighbor_parallelism = Parallelism::None();
  parameters.time_limit = 300.0;
  parameters.tree_growth_limit = 1000;
  parameters.planner_log_level = spdlog::level::info;

  // Plan a path.
  const auto result = ParallelRRTPlanner<CarPlanningState>::PlanGoalCheck(
      start, goal_checker, parameters, &planning_space);
  ASSERT_TRUE(result.has_solution());
}

}  // namespace
}  // namespace planning
}  // namespace drake
