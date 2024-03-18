#include "drake/systems/analysis/batch_eval.h"

#include <cmath>
#include <thread>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/symbolic_vector_system.h"

namespace drake {
namespace systems {
namespace analysis {
namespace {

using symbolic::Expression;
using symbolic::Variable;
using systems::LinearSystem;

GTEST_TEST(BatchEvalUniquePeriodicDiscreteUpdate, BasicTest) {
  Eigen::Matrix2d A, B;
  // clang-format off
  A << 1, 2,
       3, 4;
  B << 5, 6,
       7, 8;
  // clang-format on
  const Eigen::MatrixXd C(0, 2), D(0, 2);
  const double time_step = 0.1;
  LinearSystem<double> system(A, B, C, D, time_step);
  auto context = system.CreateDefaultContext();

  Parallelism parallelize = Parallelism::Max();

  const Eigen::RowVector3d times{0, 1, 2};
  Eigen::MatrixXd x0(2, 3);
  Eigen::MatrixXd inputs(2, 3);
  // clang-format off
  x0 << 0.1, 0.2, 0.3,
        0.4, 0.5, 0.6;
  inputs << -0.12, -0.34,  0.45,
             0.32,  0.14, -0.65;
  // clang-format on

  const Eigen::MatrixXd x1_expected = A * x0 + B * inputs;
  const Eigen::MatrixXd x2_expected = A * x1_expected + B * inputs;

  int num_time_steps = 1;
  InputPortIndex input_port_index = system.get_input_port().get_index();

  const Eigen::MatrixXd x1 = BatchEvalUniquePeriodicDiscreteUpdate<double>(
      system, *context, times, x0, inputs, num_time_steps, input_port_index,
      parallelize);

  EXPECT_TRUE(CompareMatrices(x1, x1_expected, 1e-14));

  const Eigen::RowVector3d times1 = (times.array() + time_step).matrix();
  const Eigen::MatrixXd x2 = BatchEvalUniquePeriodicDiscreteUpdate<double>(
      system, *context, times1, x1, inputs, num_time_steps, input_port_index,
      parallelize);
  EXPECT_TRUE(CompareMatrices(x2, x2_expected, 1e-14));

  // Compare the results from doing 2 steps one at a time vs two steps via
  // num_time_steps = 2.
  EXPECT_TRUE(CompareMatrices(
      x2_expected,
      BatchEvalUniquePeriodicDiscreteUpdate<double>(
          system, *context, times, x0, inputs, 2 /* num_time_steps */,
          input_port_index, parallelize),
      1e-14));
}

// Confirm that time-varying dynamics are handled correctly (with multiple time
// steps, etc.)
GTEST_TEST(BatchEvalUniquePeriodicDiscreteUpdate, TimeVaryingTest) {
  const Variable t("t");
  const Vector1<Variable> x{Variable("x")};
  const double time_period = 0.12;
  // Make a simple time-varying system
  // x[n+1] = x[n] + t
  const auto system = SymbolicVectorSystemBuilder()
                          .time(t)
                          .state(x)
                          .dynamics(Vector1<Expression>{x[0] + t})
                          .time_period(time_period)
                          .Build();
  auto context = system->CreateDefaultContext();

  const Eigen::RowVector3d times{0, 1, 2};
  Eigen::MatrixXd x0(1, 3);
  x0 << 0.1, 0.2, 0.3;
  // No inputs are needed here, since there is no input port.
  Eigen::MatrixXd inputs(0, 3);

  int num_time_steps = 2;
  // x1 = x0 + t0
  // x2 = x0 + t0 + (t0 + time_step) = x0 + 2*t0 + time_step
  const Eigen::MatrixXd x2 = BatchEvalUniquePeriodicDiscreteUpdate<double>(
      *system, *context, times, x0, inputs, num_time_steps);

  Eigen::MatrixXd x2_expected =
      x0 + 2 * times + Eigen::RowVector3d::Constant(time_period);
  EXPECT_TRUE(CompareMatrices(x2, x2_expected, 1e-14));
}

GTEST_TEST(BatchEvalTimeDerivatives, BasicTest) {
  Eigen::Matrix2d A, B;
  // clang-format off
  A << 1, 2,
       3, 4;
  B << 5, 6,
       7, 8;
  // clang-format on
  const Eigen::MatrixXd C(0, 2), D(0, 2);
  LinearSystem<double> system(A, B, C, D);
  auto context = system.CreateDefaultContext();

  Parallelism parallelize = Parallelism::Max();

  const Eigen::RowVector3d times{0, 1, 2};
  Eigen::MatrixXd states(2, 3);
  Eigen::MatrixXd inputs(2, 3);
  // clang-format off
  states << 0.1, 0.2, 0.3,
            0.4, 0.5, 0.6;
  inputs << -0.12, -0.34,  0.45,
             0.32,  0.14, -0.65;
  // clang-format on

  InputPortIndex input_port_index = system.get_input_port().get_index();

  const Eigen::MatrixXd xdot = BatchEvalTimeDerivatives<double>(
      system, *context, times, states, inputs, input_port_index, parallelize);

  const Eigen::MatrixXd xdot_expected = A * states + B * inputs;
  EXPECT_TRUE(CompareMatrices(xdot, xdot_expected, 1e-14));
}

}  // namespace
}  // namespace analysis
}  // namespace systems
}  // namespace drake
