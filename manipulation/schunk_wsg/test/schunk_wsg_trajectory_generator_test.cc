#include "drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/primitives/vector_log_sink.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
namespace {

GTEST_TEST(SchunkWsgTrajectoryGeneratorTest, DefaultPorts) {
  SchunkWsgTrajectoryGenerator dut(1, 0);
  EXPECT_EQ(dut.num_input_ports(), 3);
  EXPECT_EQ(dut.num_output_ports(), 2);

  EXPECT_EQ(dut.get_desired_position_input_port().get_name(),
            "desired_position");
  EXPECT_EQ(dut.get_force_limit_input_port().get_name(), "force_limit");
  EXPECT_EQ(dut.get_state_input_port().get_name(), "u2");
  EXPECT_EQ(dut.get_target_output_port().get_name(), "y0");
  EXPECT_EQ(dut.get_max_force_output_port().get_name(), "y1");
}

GTEST_TEST(SchunkWsgTrajectoryGeneratorTest, NonForceLimitPorts) {
  SchunkWsgTrajectoryGenerator dut(1, 0, /* use_force_limit = */ false);
  EXPECT_EQ(dut.num_input_ports(), 2);
  EXPECT_EQ(dut.num_output_ports(), 1);

  EXPECT_EQ(dut.get_desired_position_input_port().get_name(),
            "desired_position");
  EXPECT_THROW(dut.get_force_limit_input_port(), std::exception);
  EXPECT_EQ(dut.get_state_input_port().get_name(), "u2");
  EXPECT_EQ(dut.get_target_output_port().get_name(), "y0");
  EXPECT_THROW(dut.get_max_force_output_port(), std::exception);
}

GTEST_TEST(SchunkWsgTrajectoryGeneratorTest, BasicTest) {
  SchunkWsgTrajectoryGenerator dut(1, 0);
  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output = dut.AllocateOutput();

  // Start off with the gripper closed (zero) and a command to open to
  // 100mm.
  dut.get_desired_position_input_port().FixValue(context.get(), 0.05);
  dut.get_force_limit_input_port().FixValue(context.get(), 40.);
  dut.get_state_input_port().FixValue(context.get(), 0.0);

  // Step a little bit. We should be commanding a point on the
  // trajectory wider than zero, but not to the target yet.
  const double expected_target = -0.05;  // 50mm
  systems::Simulator<double> simulator(dut, std::move(context));
  simulator.AdvanceTo(0.1);
  dut.CalcOutput(simulator.get_context(), output.get());
  EXPECT_LT(output->get_vector_data(0)->GetAtIndex(0), 0);
  EXPECT_GT(output->get_vector_data(0)->GetAtIndex(0), expected_target);

  // Step quite a bit longer and see that we get there.
  simulator.AdvanceTo(2.0);
  dut.CalcOutput(simulator.get_context(), output.get());
  EXPECT_FLOAT_EQ(output->get_vector_data(0)->GetAtIndex(0), expected_target);
}

// Test the specific case when the last command does not match the current
// command but the difference between the measured position and the desired
// position is zero.
GTEST_TEST(SchunkWsgTrajectoryGeneratorTest, DeltaEqualsZero) {
  SchunkWsgTrajectoryGenerator dut(1, 0);
  auto context = dut.CreateDefaultContext();

  SchunkWsgTrajectoryGeneratorStateVector<double> state;
  state.set_last_target_position(0.0);
  context->SetDiscreteState(state.value());

  const double pos = 0.06;
  dut.get_desired_position_input_port().FixValue(context.get(), pos);
  dut.get_force_limit_input_port().FixValue(context.get(), 40.0);
  dut.get_state_input_port().FixValue(context.get(), -pos / 2.0);

  systems::Simulator<double> simulator(dut, std::move(context));
  simulator.AdvanceTo(0.1);
  Eigen::Vector2d target =
      dut.get_target_output_port().Eval(simulator.get_context());
  EXPECT_EQ(target[0], -pos);
}

// If we reset the diagram containing the ScunkWsgTrajectoryGenerator, we
// should obtain the same output.
GTEST_TEST(SchunkWsgTrajectoryGeneratorTest, ResetSim) {
  systems::DiagramBuilder<double> builder;
  auto dut = builder.AddSystem<SchunkWsgTrajectoryGenerator>(1, 0);
  auto target_logger = LogVectorOutput(dut->get_target_output_port(), &builder);
  auto max_force_logger =
      LogVectorOutput(dut->get_max_force_output_port(), &builder);
  auto diagram = builder.Build();

  auto diagram_context = diagram->CreateDefaultContext();
  auto& dut_context = dut->GetMyMutableContextFromRoot(diagram_context.get());
  const auto reset_context = diagram_context->Clone();

  const double pos = 0.06;
  dut->get_desired_position_input_port().FixValue(&dut_context, pos);
  dut->get_force_limit_input_port().FixValue(&dut_context, 40.0);
  dut->get_state_input_port().FixValue(&dut_context, -pos / 2.0);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.AdvanceTo(0.1);

  auto get_and_clear_logger = [](const systems::VectorLogSink<double>& sink,
                                 systems::Context<double>* root_context) {
    auto& log_context = sink.GetMyMutableContextFromRoot(root_context);
    systems::VectorLog<double>& log = sink.GetMutableLog(&log_context);
    const Eigen::MatrixXd data = log.data();
    log.Clear();
    return data;
  };
  const Eigen::MatrixXd target_data_expected =
      get_and_clear_logger(*target_logger, &(simulator.get_mutable_context()));

  const Eigen::MatrixXd max_force_data_expected = get_and_clear_logger(
      *max_force_logger, &(simulator.get_mutable_context()));

  // Simulate the diagram again with reset_context.
  simulator.get_mutable_context().SetTimeStateAndParametersFrom(*reset_context);
  simulator.Initialize();
  simulator.AdvanceTo(0.1);
  const Eigen::MatrixXd target_data =
      get_and_clear_logger(*target_logger, &(simulator.get_mutable_context()));
  const Eigen::MatrixXd max_force_data = get_and_clear_logger(
      *max_force_logger, &(simulator.get_mutable_context()));
  EXPECT_TRUE(CompareMatrices(target_data, target_data_expected));
  EXPECT_TRUE(CompareMatrices(max_force_data, max_force_data_expected));
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
