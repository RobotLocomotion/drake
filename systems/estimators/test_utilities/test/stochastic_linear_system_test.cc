#include "drake/systems/estimators/test_utilities/stochastic_linear_system.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace systems {
namespace estimators_test {
namespace {

GTEST_TEST(TestStochasticLinearSystem, InputOutputPort) {
  const int ns = 2, ni = 1, no = 1, nw = 1, nv = 1;
  // clang-format off
  Eigen::MatrixXd A(ns, ns), B(ns, ni), G(ns, nw),
                  C(no, ns), D(no, ni), H(no, nv);
  // clang-format on
  A << 1.0, 2.0, 3.0, 4.0;
  B << 5.0, 6.0;
  G << 7.0, 8.0;
  C << 9.0, 10.0;
  D << 11.0;
  H << 12.0;
  StochasticLinearSystem<double> sys(A, B, G, C, D, H);

  EXPECT_EQ(sys.GetInputPort("u").get_index(),
            sys.get_u_input_port().get_index());
  EXPECT_EQ(sys.GetInputPort("w").get_index(),
            sys.get_w_input_port().get_index());
  EXPECT_EQ(sys.GetInputPort("v").get_index(),
            sys.get_v_input_port().get_index());
  EXPECT_EQ(sys.GetOutputPort("y").get_index(),
            sys.get_y_output_port().get_index());
}

GTEST_TEST(TestContinuousTimeStochasticLinearSystem, test) {
  const int ns = 2, ni = 1, no = 1, nw = 1, nv = 1;
  // clang-format off
  Eigen::MatrixXd A(ns, ns), B(ns, ni), G(ns, nw),
                  C(no, ns), D(no, ni), H(no, nv);
  // clang-format on
  A << 1.0, 2.0, 3.0, 4.0;
  B << 5.0, 6.0;
  G << 7.0, 8.0;
  C << 9.0, 10.0;
  D << 11.0;
  H << 12.0;
  StochasticLinearSystem<double> sys(A, B, G, C, D, H);
  EXPECT_TRUE(sys.IsDifferentialEquationSystem());

  StochasticLinearSystem<double> system(LinearSystem<double>(A, B, C, D), G, H);
  EXPECT_TRUE(system.IsDifferentialEquationSystem());

  EXPECT_TRUE(CompareMatrices(system.A(), A));
  EXPECT_TRUE(CompareMatrices(system.B(), B));
  EXPECT_TRUE(CompareMatrices(system.G(), G));
  EXPECT_TRUE(CompareMatrices(system.C(), C));
  EXPECT_TRUE(CompareMatrices(system.D(), D));
  EXPECT_TRUE(CompareMatrices(system.H(), H));

  Eigen::VectorXd x(ns), u(ni), w(nw), v(nv);
  x << 13.0, 14.0;
  u << 15.0;
  w << 16.0;
  v << 17.0;

  auto context = system.CreateDefaultContext();
  system.get_u_input_port().FixValue(context.get(), u);
  system.get_w_input_port().FixValue(context.get(), w);
  system.get_v_input_port().FixValue(context.get(), v);

  context->SetContinuousState(x);
  Eigen::VectorXd xdot =
      system.EvalTimeDerivatives(*context).get_vector().CopyToVector();
  Eigen::VectorXd y = system.get_output_port().Eval(*context);

  EXPECT_TRUE(CompareMatrices(xdot, A * x + B * u + G * w));
  EXPECT_TRUE(CompareMatrices(y, C * x + D * u + H * v));

  EXPECT_NO_THROW(system.ToAutoDiffXd());
  EXPECT_NO_THROW(system.ToSymbolic());
}

GTEST_TEST(TestDiscreteTimeStochasticLinearSystem, test) {
  const int ns = 2, ni = 1, no = 1, nw = 1, nv = 1;
  // clang-format off
  Eigen::MatrixXd A(ns, ns), B(ns, ni), G(ns, nw),
                  C(no, ns), D(no, ni), H(no, nv);
  // clang-format on
  A << 1.0, 2.0, 3.0, 4.0;
  B << 5.0, 6.0;
  G << 7.0, 8.0;
  C << 9.0, 10.0;
  D << 11.0;
  H << 12.0;
  const double time_period = 0.1;
  StochasticLinearSystem<double> sys(A, B, G, C, D, H, time_period);
  EXPECT_TRUE(sys.IsDifferenceEquationSystem());

  StochasticLinearSystem<double> system(
      LinearSystem<double>(A, B, C, D, time_period), G, H);
  EXPECT_TRUE(system.IsDifferenceEquationSystem());

  EXPECT_TRUE(CompareMatrices(system.A(), A));
  EXPECT_TRUE(CompareMatrices(system.B(), B));
  EXPECT_TRUE(CompareMatrices(system.G(), G));
  EXPECT_TRUE(CompareMatrices(system.C(), C));
  EXPECT_TRUE(CompareMatrices(system.D(), D));
  EXPECT_TRUE(CompareMatrices(system.H(), H));

  Eigen::VectorXd x(ns), u(ni), w(nw), v(nv);
  x << 13.0, 14.0;
  u << 15.0;
  w << 16.0;
  v << 17.0;

  auto context = system.CreateDefaultContext();
  system.get_u_input_port().FixValue(context.get(), u);
  system.get_w_input_port().FixValue(context.get(), w);
  system.get_v_input_port().FixValue(context.get(), v);

  context->SetDiscreteState(x);
  Eigen::VectorXd x_next =
      system.EvalUniquePeriodicDiscreteUpdate(*context).value();
  Eigen::VectorXd y = system.get_output_port().Eval(*context);

  EXPECT_TRUE(CompareMatrices(x_next, A * x + B * u + G * w));
  EXPECT_TRUE(CompareMatrices(y, C * x + D * u + H * v));

  EXPECT_NO_THROW(system.ToAutoDiffXd());
  EXPECT_NO_THROW(system.ToSymbolic());
}

}  // namespace
}  // namespace estimators_test
}  // namespace systems
}  // namespace drake
