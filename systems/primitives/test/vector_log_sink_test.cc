#include "drake/systems/primitives/vector_log_sink.h"

#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(TestVectorLogSink, LogGetters) {
  VectorLogSink<double> dut(11);
  auto context = dut.CreateDefaultContext();
  EXPECT_EQ(&(dut.GetLog(*context)), &(dut.GetMutableLog(context.get())));
}

GTEST_TEST(TestVectorLogSink, LogFinders) {
  DiagramBuilder<double> builder;
  builder.AddSystem<ConstantVectorSource<double>>(22.0);
  auto dut = builder.AddSystem<VectorLogSink<double>>(1);
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  auto& logger_context =
      dut->GetMyMutableContextFromRoot(diagram_context.get());

  EXPECT_EQ(&(dut->GetLog(logger_context)),
            &(dut->FindLog(*diagram_context)));
  EXPECT_EQ(&(dut->FindLog(*diagram_context)),
            &(dut->FindMutableLog(diagram_context.get())));

  DRAKE_EXPECT_THROWS_MESSAGE(dut->GetLog(*diagram_context),
                              ".*Context was not created for.*");
  DRAKE_EXPECT_THROWS_MESSAGE(dut->FindLog(logger_context),
                              ".*given context must be a root context.*");
}

GTEST_TEST(TestVectorLogSink, LogFindersWrongContext) {
  // Log sink is in *a* diagram, so something working is plausible.
  DiagramBuilder<double> builder;
  auto dut = builder.AddSystem<VectorLogSink<double>>(1);
  auto diagram = builder.Build();

  // Some unrelated wrong diagram exists to provide bad input for the test.
  DiagramBuilder<double> wrong_builder;
  wrong_builder.AddSystem<ConstantVectorSource<double>>(22.0);
  auto wrong_diagram = wrong_builder.Build();
  auto wrong_diagram_context = wrong_diagram->CreateDefaultContext();

  // Both access methods fail context validation.
  DRAKE_EXPECT_THROWS_MESSAGE(dut->GetLog(*wrong_diagram_context),
                              ".*Context was not created for.*");
  DRAKE_EXPECT_THROWS_MESSAGE(dut->FindLog(*wrong_diagram_context),
                              ".*Context was not created for.*");
}

// Log the output of a simple linear system (with a known solution).
GTEST_TEST(TestVectorLogSink, LinearSystemTest) {
  using std::exp;
  DiagramBuilder<double> builder;

  // xdot = -x.  y = x.  (No inputs).
  auto plant = builder.AddSystem<LinearSystem<double>>(
      Vector1d::Constant(-1.0),      // A.
      Eigen::MatrixXd::Zero(1, 0),   // B.
      Vector1d::Constant(1.0),       // C.
      Eigen::MatrixXd::Zero(1, 0));  // D.
  plant->set_name("plant");

  auto logger = builder.AddSystem<VectorLogSink<double>>(1);
  logger->set_name("logger");
  builder.Cascade(*plant, *logger);

  // Test the AddVectorLogSink helper method, too.
  auto logger2 = LogVectorOutput(plant->get_output_port(), &builder);

  auto diagram = builder.Build();

  // Simulate the simple system from x(0) = 1.0.
  Simulator<double> simulator(*diagram);
  Context<double>& context = simulator.get_mutable_context();
  const auto& log = logger->FindLog(context);
  const auto& log2 = logger2->FindLog(context);
  context.get_mutable_continuous_state_vector().SetAtIndex(0, 1.0);

  // Make the integrator tolerance sufficiently tight for the test to pass.
  simulator.get_mutable_integrator().set_target_accuracy(1e-4);

  simulator.Initialize();
  // The Simulator schedules VectorLogSink's default per-step event, which
  // performs data logging.
  simulator.AdvanceTo(3);

  // Gets the time stamps when each data point is saved.
  const auto& t = log.sample_times();

  // Gets the logged data.
  const auto& x = log.data();
  EXPECT_EQ(x.cols(), t.size());

  // Check that num_samples() makes sense.
  EXPECT_EQ(log.num_samples(), t.size());

  // Now check the data (against the known solution to the diff eq).
  const Eigen::MatrixXd expected_x = exp(-t.transpose().array());

  // Tolerance allows the default RK3 integrator to just squeak by.
  double tol = 1e-5;
  EXPECT_TRUE(CompareMatrices(expected_x, x, tol));

  // Confirm that both loggers acquired the same data.
  EXPECT_TRUE(CompareMatrices(log.sample_times(), log2.sample_times()));
  EXPECT_TRUE(CompareMatrices(log.data(), log2.data()));

  // Test that Clear makes everything empty.
  logger->FindMutableLog(&context).Clear();
  EXPECT_EQ(log.num_samples(), 0);
  EXPECT_EQ(log.sample_times().size(), 0);
  EXPECT_EQ(log.data().cols(), 0);

  simulator.AdvanceTo(4.);
  EXPECT_EQ(log.data().cols(), log.num_samples());
}

// Test that periodic publishing causes correct triggering even for logging
// a constant signal.
GTEST_TEST(TestVectorLogSink, PeriodicPublish) {
  // Add System and Connect
  DiagramBuilder<double> builder;
  auto system = builder.AddSystem<ConstantVectorSource<double>>(2.0);
  auto logger = LogVectorOutput(system->get_output_port(), &builder, 0.125);
  auto diagram = builder.Build();

  // Construct Simulator
  Simulator<double> simulator(*diagram);

  // Run simulation
  simulator.AdvanceTo(1);

  const auto& log = logger->FindLog(simulator.get_context());
  EXPECT_EQ(log.num_samples(), 9);
}

// Test that setting forced-publish-only triggers on an explicit Publish().
GTEST_TEST(TestVectorLogSink, ForcedPublishOnly) {
  // Add System and Connect
  DiagramBuilder<double> builder;
  auto system = builder.AddSystem<ConstantVectorSource<double>>(2.0);
  auto logger = LogVectorOutput(system->get_output_port(), &builder,
                                {TriggerType::kForced});
  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  const auto& log = logger->FindLog(*context);

  EXPECT_EQ(log.num_samples(), 0);

  diagram->Publish(*context);
  EXPECT_EQ(log.num_samples(), 1);
}

// Test that setting no forced publish triggers does not log on Publish().
GTEST_TEST(TestVectorLogSink, NoForcedPublish) {
  // Add System and Connect
  DiagramBuilder<double> builder;
  auto system = builder.AddSystem<ConstantVectorSource<double>>(2.0);
  auto logger = LogVectorOutput(system->get_output_port(), &builder,
                                {TriggerType::kPeriodic}, 0.125);
  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  const auto& log = logger->FindLog(*context);

  EXPECT_EQ(log.num_samples(), 0);

  diagram->Publish(*context);
  EXPECT_EQ(log.num_samples(), 0);
}

GTEST_TEST(TestVectorLogSink, ScalarConversion) {
  VectorLogSink<double> dut_per_step_publish(2);
  VectorLogSink<double> dut_forced_publish(2, {TriggerType::kForced});
  VectorLogSink<double> dut_periodic_publish(2, 0.25);
  for (const auto* dut : {
      &dut_per_step_publish, &dut_forced_publish, &dut_periodic_publish}) {
    EXPECT_TRUE(is_autodiffxd_convertible(*dut, [&](const auto& converted) {
      ASSERT_EQ(converted.num_input_ports(), 1);
      EXPECT_EQ(converted.get_input_port().size(), 2);
    }));
    EXPECT_TRUE(is_symbolic_convertible(*dut, [&](const auto& converted) {
      ASSERT_EQ(converted.num_input_ports(), 1);
      EXPECT_EQ(converted.get_input_port().size(), 2);
    }));
  }
}

GTEST_TEST(TestVectorLogSink, DiagramToAutoDiff) {
  DiagramBuilder<double> builder;
  auto system = builder.AddSystem<ConstantVectorSource<double>>(2.0);
  LogVectorOutput(system->get_output_port(), &builder);
  auto diagram = builder.Build();
  EXPECT_TRUE(is_autodiffxd_convertible(*diagram));
}

}  // namespace
}  // namespace systems
}  // namespace drake
