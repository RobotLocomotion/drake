#include "drake/systems/primitives/signal_logger.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace {

// Log the output of a simple linear system (with a known solution).
GTEST_TEST(TestSignalLogger, LinearSystemTest) {
  systems::DiagramBuilder<double> builder;

  // xdot = -x.  y = x.  (No inputs).
  auto plant = builder.AddSystem<systems::LinearSystem<double>>(
      Vector1d::Constant(-1.0),      // A.
      Eigen::MatrixXd::Zero(1, 0),   // B.
      Vector1d::Constant(1.0),       // C.
      Eigen::MatrixXd::Zero(1, 0));  // D.
  plant->set_name("plant");

  auto logger = builder.AddSystem<systems::SignalLogger<double>>(1);
  logger->set_name("logger");
  builder.Cascade(*plant, *logger);

  // Test the AddSignalLogger helper method, too.
  auto logger2 = LogOutput(plant->get_output_port(), &builder);

  auto diagram = builder.Build();

  // Simulate the simple system from x(0) = 1.0.
  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& context = simulator.get_mutable_context();
  context.get_mutable_continuous_state_vector().SetAtIndex(0, 1.0);

  // Make the integrator tolerance sufficiently tight for the test to pass.
  simulator.get_mutable_integrator()->set_target_accuracy(1e-4);

  simulator.Initialize();
  // The Simulator internally calls Publish(), which triggers data logging.
  simulator.StepTo(3);

  // Gets the time stamps when each data point is saved.
  const auto& t = logger->sample_times();
  EXPECT_EQ(t.size(), simulator.get_num_publishes());

  // Gets the logged data.
  const auto& x = logger->data();
  EXPECT_EQ(x.cols(), t.size());

  // Now check the data (against the known solution to the diff eq).
  const Eigen::MatrixXd expected_x = exp(-t.transpose().array());

  double tol = 1e-6;  // Not bad for numerical integration!
  EXPECT_TRUE(CompareMatrices(expected_x, x, tol));

  // Confirm that both loggers acquired the same data.
  EXPECT_TRUE(CompareMatrices(logger->sample_times(), logger2->sample_times()));
  EXPECT_TRUE(CompareMatrices(logger->data(), logger2->data()));
}

}  // namespace
}  // namespace drake
