#include "drake/manipulation/planner/robot_plan_interpolator.h"

#include <gtest/gtest.h>
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace {

static const int kNumJoints = 7;
const char* const kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
const char* const kDualIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/"
    "dual_iiwa14_polytope_collision.urdf";

GTEST_TEST(RobotPlanInterpolatorTest, InstanceTest) {
  // Test that the constructor works and that the expected ports are
  // present.
  RobotPlanInterpolator dut(FindResourceOrThrow(kIiwaUrdf));
  EXPECT_EQ(dut.get_plan_input_port().get_data_type(),
            systems::kAbstractValued);
  EXPECT_EQ(dut.get_state_input_port().get_data_type(), systems::kVectorValued);
  EXPECT_EQ(dut.get_state_input_port().size(), kNumJoints * 2);
  EXPECT_EQ(dut.get_state_output_port().get_data_type(),
            systems::kVectorValued);
  EXPECT_EQ(dut.get_state_output_port().size(), kNumJoints * 2);
  EXPECT_EQ(dut.get_acceleration_output_port().get_data_type(),
            systems::kVectorValued);
  EXPECT_EQ(dut.get_acceleration_output_port().size(), kNumJoints);
}

GTEST_TEST(RobotPlanInterpolatorTest, DualInstanceTest) {
  // Check that the port sizes come out appropriately for a dual armed
  // model.
  RobotPlanInterpolator dut(FindResourceOrThrow(kDualIiwaUrdf));
  EXPECT_EQ(dut.tree().get_num_positions(), kNumJoints * 2);
  EXPECT_EQ(dut.tree().get_num_velocities(), kNumJoints * 2);

  EXPECT_EQ(dut.get_plan_input_port().get_data_type(),
            systems::kAbstractValued);
  EXPECT_EQ(dut.get_state_input_port().get_data_type(), systems::kVectorValued);
  EXPECT_EQ(dut.get_state_input_port().size(), kNumJoints * 4);
  EXPECT_EQ(dut.get_state_output_port().get_data_type(),
            systems::kVectorValued);
  EXPECT_EQ(dut.get_state_output_port().size(), kNumJoints * 4);
  EXPECT_EQ(dut.get_acceleration_output_port().get_data_type(),
            systems::kVectorValued);
  EXPECT_EQ(dut.get_acceleration_output_port().size(), kNumJoints * 2);
}

struct TrajectoryTestCase {
  TrajectoryTestCase(double time_in, double position_in, double velocity_in,
                     double accel_in)
      : time(time_in),
        position(position_in),
        velocity(velocity_in),
        accel(accel_in) {}

  const double time{};
  const double position{};
  const double velocity{};
  const double accel{};
};

void DoTrajectoryTest(InterpolatorType interp_type) {
  RobotPlanInterpolator dut(FindResourceOrThrow(kIiwaUrdf), interp_type);

  std::vector<double> t{0, 1, 2, 3, 4};
  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(kNumJoints, t.size());
  // Only bother with one joint.
  q(0, 1) = 1;
  q(0, 2) = 1.5;
  q(0, 3) = 1.5;
  q(0, 4) = 1;

  std::vector<int> info(t.size(), 1);

  const int num_time_steps = q.cols();

  // Encode into a robot_plan_t structure.
  robotlocomotion::robot_plan_t plan{};
  plan.num_states = num_time_steps;
  const bot_core::robot_state_t default_robot_state{};
  plan.plan.resize(num_time_steps, default_robot_state);
  plan.plan_info.resize(num_time_steps, 0);
  for (int i = 0; i < num_time_steps; i++) {
    bot_core::robot_state_t& step = plan.plan[i];
    step.utime = t[i] * 1e6;
    step.num_joints = q.rows();
    for (int j = 0; j < step.num_joints; j++) {
      step.joint_name.push_back(dut.tree().get_position_name(j));
      step.joint_position.push_back(q(j, i));
      step.joint_velocity.push_back(0);
      step.joint_effort.push_back(0);
    }
  }

  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut.AllocateOutput(*context);
  context->FixInputPort(
      dut.get_state_input_port().get_index(),
      Eigen::VectorXd::Zero(kNumJoints * 2));
  context->FixInputPort(dut.get_plan_input_port().get_index(),
                        systems::AbstractValue::Make(plan));
  dut.Initialize(0, Eigen::VectorXd::Zero(kNumJoints),
                 &context->get_mutable_state());

  dut.CalcUnrestrictedUpdate(*context, &context->get_mutable_state());

  // Test we're running the plan through time by watching the
  // positions, velocities, and acceleration change.
  std::vector<TrajectoryTestCase> cases;
  std::string interp_str;
  switch (interp_type) {
    case InterpolatorType::ZeroOrderHold :
      interp_str = "Zero Order Hold";
      cases.push_back(TrajectoryTestCase{0.5, 0, 0, 0});
      cases.push_back(TrajectoryTestCase{1.5, 1, 0, 0});
      cases.push_back(TrajectoryTestCase{2.7, 1.5, 0, 0});
      cases.push_back(TrajectoryTestCase{3.5, 1.5, 0, 0});
      break;
    case InterpolatorType::FirstOrderHold :
      interp_str = "First Order Hold";
      cases.push_back(TrajectoryTestCase{0.5, 0.5, 1, 0});
      cases.push_back(TrajectoryTestCase{1.5, 1.25, 0.5, 0});
      cases.push_back(TrajectoryTestCase{2.7, 1.5, 0, 0});
      cases.push_back(TrajectoryTestCase{3.5, 1.25, -0.5, 0});
      break;
    case InterpolatorType::Pchip :
      interp_str = "Pchip";
      cases.push_back(TrajectoryTestCase{0.5, 0.417, 1.333, 0.666});
      cases.push_back(TrajectoryTestCase{1.5, 1.333, 0.583, -0.666});
      cases.push_back(TrajectoryTestCase{2.7, 1.5, 0, 0});
      cases.push_back(TrajectoryTestCase{3.5, 1.250, -0.75, 0});
      break;
    case InterpolatorType::Cubic :
      interp_str = "Cubic";
      cases.push_back(TrajectoryTestCase{0.5, 0.3661, 1.232, 1.071});
      cases.push_back(TrajectoryTestCase{1.5, 1.357, 0.429, -0.857});
      cases.push_back(TrajectoryTestCase{2.7, 1.577, -0.101, -0.900});
      cases.push_back(TrajectoryTestCase{3.5, 1.196, -0.642, 0.429});
      break;
  }

  for (const TrajectoryTestCase& kase : cases) {
    context->set_time(kase.time);
    dut.CalcUnrestrictedUpdate(*context, &context->get_mutable_state());
    dut.CalcOutput(*context, output.get());
    const double position =
        output->get_vector_data(dut.get_state_output_port().get_index())
            ->GetAtIndex(0);
    const double velocity =
        output->get_vector_data(dut.get_state_output_port().get_index())
            ->GetAtIndex(kNumJoints);
    const double accel =
        output->get_vector_data(dut.get_acceleration_output_port().get_index())
            ->GetAtIndex(0);
    const double err_tol = 1e-3;
    EXPECT_NEAR(position, kase.position, err_tol)
              << "Failed at interpolator type: " << interp_str;
    EXPECT_NEAR(velocity, kase.velocity, err_tol)
              << "Failed at interpolator type: " << interp_str;
    EXPECT_NEAR(accel, kase.accel, err_tol)
              << "Failed at interpolator type: " << interp_str;
  }

  // Check that the final knot point has zero acceleration and
  // velocity.
  if (interp_type == InterpolatorType::Cubic ||
      interp_type == InterpolatorType::ZeroOrderHold ||
      interp_type == InterpolatorType::Pchip) {
    context->set_time(t.back() + 0.01);
    dut.CalcUnrestrictedUpdate(*context, &context->get_mutable_state());
    dut.CalcOutput(*context, output.get());
    const double velocity =
        output->get_vector_data(dut.get_state_output_port().get_index())
            ->GetAtIndex(kNumJoints);
    const double accel =
        output->get_vector_data(dut.get_acceleration_output_port().get_index())
            ->GetAtIndex(0);
    EXPECT_FLOAT_EQ(velocity, 0)
              << "Failed at interpolator type: " << interp_str;
    EXPECT_FLOAT_EQ(accel, 0)
              << "Failed at interpolator type: " << interp_str;
  }

  // Check that sending an empty plan causes us to continue to output
  // the same commanded position.
  context->set_time(1);
  dut.CalcUnrestrictedUpdate(*context, &context->get_mutable_state());
  dut.CalcOutput(*context, output.get());
  double position =
      output->get_vector_data(dut.get_state_output_port().get_index())
          ->GetAtIndex(0);
  EXPECT_DOUBLE_EQ(1, position)
            << "Failed at interpolator type: " << interp_str;

  plan.num_states = 0;
  plan.plan.clear();
  context->FixInputPort(dut.get_plan_input_port().get_index(),
                        systems::AbstractValue::Make(plan));
  dut.CalcUnrestrictedUpdate(*context, &context->get_mutable_state());
  dut.CalcOutput(*context, output.get());
  position = output->get_vector_data(
      dut.get_state_output_port().get_index())->GetAtIndex(0);
  EXPECT_DOUBLE_EQ(1, position)
            << "Failed at interpolator type: " << interp_str;
}


class TrajectoryTestClass : public testing::TestWithParam<InterpolatorType> {
 public:
  virtual void SetUp() {}
  virtual void TearDown() {}
};

INSTANTIATE_TEST_CASE_P(InstantiationName, TrajectoryTestClass,
                        ::testing::Values(InterpolatorType::ZeroOrderHold,
                                          InterpolatorType::FirstOrderHold,
                                          InterpolatorType::Pchip,
                                          InterpolatorType::Cubic));

TEST_P(TrajectoryTestClass, TrajectoryTest) {
  DoTrajectoryTest(GetParam());
}

}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
