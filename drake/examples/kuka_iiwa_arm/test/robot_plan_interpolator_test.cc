#include "drake/examples/kuka_iiwa_arm/robot_plan_interpolator.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

static const int kNumJoints = 7;
const char* const kIiwaUrdf =
    "/manipulation/models/iiwa_description/urdf/"
        "iiwa14_polytope_collision.urdf";

GTEST_TEST(RobotPlanInterpolatorTest, InstanceTest) {
  // Test that the constructor works and that the expected ports are
  // present.
  RobotPlanInterpolator dut(GetDrakePath() + kIiwaUrdf);
  EXPECT_EQ(dut.get_plan_input_port().get_data_type(),
            systems::kAbstractValued);
  EXPECT_EQ(dut.get_state_input_port().get_data_type(),
            systems::kVectorValued);
  EXPECT_EQ(dut.get_state_input_port().size(), kNumJoints * 2);
  EXPECT_EQ(dut.get_state_output_port().get_data_type(),
            systems::kVectorValued);
  EXPECT_EQ(dut.get_state_output_port().size(), kNumJoints * 2);
  EXPECT_EQ(dut.get_acceleration_output_port().get_data_type(),
            systems::kVectorValued);
  EXPECT_EQ(dut.get_acceleration_output_port().size(), kNumJoints);
}

struct TrajectoryTestCase {
  TrajectoryTestCase(double time_in, double velocity_sign_in,
                     double accel_sign_in)
      : time(time_in),
        velocity_sign(velocity_sign_in),
        accel_sign(accel_sign_in) {}

  const double time{};
  const double velocity_sign{};
  const double accel_sign{};
};


GTEST_TEST(RobotPlanInterpolatorTest, TrajectoryTest) {
  RobotPlanInterpolator dut(GetDrakePath() + kIiwaUrdf);

  std::vector<double> t{0, 1, 2, 3, 4};
  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(kNumJoints, t.size());
  // Only bother with one joint.
  q(0, 1) = 1;
  q(0, 2) = 1.5;
  q(0, 3) = 1.5;
  q(0, 4) = 1;

  std::vector<int> info(t.size(), 1);

  robotlocomotion::robot_plan_t plan = EncodeKeyFrames(dut.tree(), t, info, q);

  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      dut.AllocateOutput(*context);
  context->FixInputPort(dut.get_state_input_port().get_index(),
                        Eigen::VectorXd::Zero(kNumJoints * 2));
  context->FixInputPort(
      dut.get_plan_input_port().get_index(),
      systems::AbstractValue::Make(plan));
  dut.Initialize(0, Eigen::VectorXd::Zero(kNumJoints),
                 context->get_mutable_state());

  systems::DiscreteEvent<double> event;
  event.action = systems::DiscreteEvent<double>::kUnrestrictedUpdateAction;
  dut.CalcUnrestrictedUpdate(*context, event,
                             context->get_mutable_state());

  // Test we're running the plan through time by watching the
  // velocities and acceleraiton change.
  std::vector<TrajectoryTestCase> cases;
  cases.push_back(TrajectoryTestCase{0.5, 1, 1});
  cases.push_back(TrajectoryTestCase{1.5, 1, -1});
  cases.push_back(TrajectoryTestCase{2.7, -1, -1});
  cases.push_back(TrajectoryTestCase{3.5, -1, 1});

  for (const TrajectoryTestCase& kase : cases) {
    context->set_time(kase.time);
    dut.CalcUnrestrictedUpdate(*context, event,
                               context->get_mutable_state());
    dut.CalcOutput(*context, output.get());
    const double velocity = output->get_vector_data(
        dut.get_state_output_port().get_index())->GetAtIndex(kNumJoints);
    const double accel = output->get_vector_data(
        dut.get_acceleration_output_port().get_index())->GetAtIndex(0);
    EXPECT_GT(velocity * kase.velocity_sign, 0);
    EXPECT_GT(accel * kase.accel_sign, 0);
  }

  // Check that sending an empty plan causes a replan to the current
  // measured position.

  context->set_time(1);
  dut.CalcUnrestrictedUpdate(*context, event,
                             context->get_mutable_state());
  dut.CalcOutput(*context, output.get());
  double position = output->get_vector_data(
        dut.get_state_output_port().get_index())->GetAtIndex(0);
  EXPECT_DOUBLE_EQ(1, position);

  plan.num_states = 0;
  plan.plan.clear();
  context->FixInputPort(
      dut.get_plan_input_port().get_index(),
      systems::AbstractValue::Make(plan));
  dut.CalcUnrestrictedUpdate(*context, event,
                             context->get_mutable_state());
  dut.CalcOutput(*context, output.get());
  position = output->get_vector_data(
        dut.get_state_output_port().get_index())->GetAtIndex(0);
  EXPECT_DOUBLE_EQ(0, position);
}

}  // namespace

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
