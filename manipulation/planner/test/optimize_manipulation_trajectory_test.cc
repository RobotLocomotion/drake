#include "drake/manipulation/planner/optimize_manipulation_trajectory.h"

#include <limits>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/planner/kinematic_tree.h"
#include "drake/manipulation/planner/rigid_body_tree_wrapper.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/math/bspline_curve.h"
#include "drake/math/transform.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"

using drake::examples::kuka_iiwa_arm::get_iiwa_max_joint_velocities;
using drake::math::BsplineCurve;
using drake::math::Transform;
using drake::trajectories::PiecewisePolynomial;
using drake::systems::DrakeVisualizer;

DEFINE_bool(visualize, false, "Publish visualization messages over LCM.");

namespace drake {
namespace manipulation {
namespace planner {
namespace {
std::unique_ptr<RigidBodyTree<double>> MakeRigidBodyTreeForTesting() {
  drake::manipulation::util::WorldSimTreeBuilder<double> tree_builder;
  tree_builder.StoreDrakeModel(
      "iiwa",
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_spheres_collision.urdf");
  tree_builder.StoreDrakeModel("environment",
                               "drake/manipulation/planner/test/"
                               "optimize_manipulation_trajectory_test_"
                               "environment.sdf");
  tree_builder.AddFixedModelInstance("iiwa", {0, 0, 0});
  tree_builder.AddFixedModelInstance("environment", {0, 0, 0});
  return tree_builder.Build();
}
class OptimizeManipulationTrajectoryTests
    : public ::testing::TestWithParam<
          test::OptimizeManipulationTrajectoryTestParameter> {
 public:
  OptimizeManipulationTrajectoryTests() {
    tree_ = MakeRigidBodyTreeForTesting();
    tool_frame_ = std::make_shared<RigidBodyFrame<double>>(
        "tool_frame", tree_->FindBody(GetParam().tool_frame_body_name),
        GetParam().X_BT);
    tree_->addFrame(tool_frame_);
    kinematic_tree_ = std::make_unique<RigidBodyTreeWrapper>(tree_.get());
    kinematic_tree_->SetJointVelocityLimits(-get_iiwa_max_joint_velocities(),
                                            get_iiwa_max_joint_velocities());
    visualize_ = FLAGS_visualize;
    if (visualize_) {
      drake::log()->info("Constructing visualizer ...");
      lcm_ = std::make_unique<drake::lcm::DrakeLcm>();
      visualizer_ = std::make_unique<DrakeVisualizer>(*tree_, lcm_.get(), true);
      visualizer_->PublishLoadRobot();
    }
  }

 protected:
  bool visualize_{false};
  OptimizeManipulationTrajectoryParameters optimization_parameters_{};
  std::unique_ptr<drake::systems::DrakeVisualizer> visualizer_;
  std::unique_ptr<drake::lcm::DrakeLcm> lcm_;
  std::unique_ptr<RigidBodyTree<double>> tree_;
  std::unique_ptr<KinematicTree> kinematic_tree_;
  std::shared_ptr<RigidBodyFrame<double>> camera_frame_;
  std::shared_ptr<RigidBodyFrame<double>> tool_frame_;

  bool CollisionCheckWaypoints(const std::vector<Eigen::VectorXd>& waypoints) {
    std::shared_ptr<solvers::Constraint> collision_avoidance_constraint =
        kinematic_tree_->MakeCollisionAvoidanceConstraint(
            optimization_parameters_.collision_avoidance_threshold);
    for (const auto& waypoint : waypoints) {
      if (!collision_avoidance_constraint->CheckSatisfied(waypoint)) {
        return false;
      }
    }
    return true;
  }

  bool CollisionCheckTrajectory(const PiecewisePolynomial<double>& traj) {
    constexpr int num_validation_points = 1e3;
    auto t = Eigen::VectorXd::LinSpaced(num_validation_points,
                                        traj.start_time(), traj.end_time());
    std::vector<Eigen::VectorXd> waypoints{};
    for (int i = 0; i < num_validation_points; ++i) {
      waypoints.push_back(traj.value(t(i)));
    }
    return CollisionCheckWaypoints(waypoints);
  }
};

std::vector<test::OptimizeManipulationTrajectoryTestParameter>
MakeOptimizeManipulationTrajectoryTestParameters() {
  return {
#include "manipulation/planner/test/optimize_manipulation_trajectory_test_parameters.inc"
  };
}

TEST_P(OptimizeManipulationTrajectoryTests,
       OptimizeTrajectoryThroughDesiredConfigurationsTest) {
  // Verify that OptimizeTrajectoryThroughDesiredConfigurations() successfully
  // generates a collision-free trajectory.
  std::vector<VectorX<double>> desired_configurations{GetParam().q0};
  for (const auto q : GetParam().q_des) {
    desired_configurations.push_back(q);
  }
  for (const auto duration : GetParam().durations) {
    drake::log()->debug("duration: {}", duration);
  }
  optional<BsplineCurve<double>> position_curve =
      OptimizeTrajectoryThroughDesiredConfigurations(
          *kinematic_tree_, desired_configurations, GetParam().durations,
          optimization_parameters_);
  ASSERT_TRUE(position_curve);
  EXPECT_TRUE(
      CollisionCheckTrajectory(position_curve->piecewise_polynomial().value()));
  if (visualize_) {
    visualizer_->PlaybackTrajectory(
        position_curve->piecewise_polynomial().value());
  }
}

TEST_P(OptimizeManipulationTrajectoryTests,
       OptimizeTrajectoryThroughDesiredToolPoses) {
  // Verify that GenerateJointTrajectory() successfully generates a
  // collision-free trajectory when called with end-effector poses.
  std::vector<Transform<double>> X_RT_desired_sequence;
  for (const auto q : GetParam().q_des) {
    X_RT_desired_sequence.push_back(
        kinematic_tree_->CalcRelativeTransform(q, "world", "tool_frame"));
  }
  optional<BsplineCurve<double>> position_curve =
      OptimizeTrajectoryThroughDesiredToolPoses(
          *kinematic_tree_, X_RT_desired_sequence, "tool_frame", GetParam().q0,
          GetParam().durations, {} /* reference_frame_name */,
          optimization_parameters_);
  ASSERT_TRUE(position_curve);
  EXPECT_TRUE(
      CollisionCheckTrajectory(position_curve->piecewise_polynomial().value()));
  if (visualize_) {
    visualizer_->PlaybackTrajectory(
        position_curve->piecewise_polynomial().value());
  }
}

INSTANTIATE_TEST_CASE_P(
    OptimizeManipulationTrajectoryTestParameters,
    OptimizeManipulationTrajectoryTests,
    ::testing::ValuesIn(MakeOptimizeManipulationTrajectoryTestParameters()));

}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
