#include "drake/systems/primitives/diagram_logger.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace {

// Adds two systems (a linear system and a gain block with gain = 0.5) to a
// diagram, log both systems' outputs using a DiagramLogger.
GTEST_TEST(TestDiagramLogger, Test) {
  systems::DiagramBuilder<double> builder;

  auto plant = builder.AddSystem<systems::LinearSystem<double>>(
      Vector1d::Constant(-1.0),      // A.
      Eigen::MatrixXd::Zero(1, 0),   // B.
      Vector1d::Constant(1.0),       // C.
      Eigen::MatrixXd::Zero(1, 0));  // D.

  auto gain = builder.AddSystem<systems::Gain<double>>(0.5, 1);
  builder.Connect(plant->get_output_port(), gain->get_input_port());

  DiagramLogger<double> loggers;
  loggers.LogOutputPort("lin_sys", plant->get_output_port(), &builder);
  loggers.LogOutputPort("gain", gain->get_output_port(), &builder);

  auto diagram = builder.Build();

  // Simulate the simple system from x(0) = 1.0.
  Simulator<double> simulator(*diagram);
  Context<double>* context = simulator.get_mutable_context();
  context->get_mutable_continuous_state_vector()->SetAtIndex(0, 1.0);

  simulator.Initialize();
  simulator.StepTo(3);

  const SignalLogger<double>* logger;
  {
    logger = loggers.get_logger("lin_sys");
    const auto& t = logger->sample_times();
    EXPECT_EQ(t.size(), simulator.get_num_publishes());

    // Now check the data (against the known solution to the diff eq).
    const auto& x = logger->data();
    EXPECT_EQ(x.cols(), t.size());

    const Eigen::MatrixXd desired_x = exp(-t.transpose().array());

    double tol = 1e-6;  // Not bad for numerical integration!
    EXPECT_TRUE(CompareMatrices(desired_x, x, tol));
  }

  {
    logger = loggers.get_logger("gain");
    const auto& t = logger->sample_times();
    EXPECT_EQ(t.size(), simulator.get_num_publishes());

    // Now check the data (against the known solution to the diff eq).
    const auto& x = logger->data();
    EXPECT_EQ(x.cols(), t.size());

    const Eigen::MatrixXd desired_x = exp(-t.transpose().array());

    double tol = 1e-6;  // Not bad for numerical integration!
    EXPECT_TRUE(CompareMatrices(0.5 * desired_x, x, tol));
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
