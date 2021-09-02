#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace multibody {
GTEST_TEST(UnitQuaternionConstraintTest, Test) {
  UnitQuaternionConstraint dut{};
  EXPECT_EQ(dut.num_constraints(), 1);
  EXPECT_EQ(dut.num_vars(), 4);
  EXPECT_TRUE(CompareMatrices(dut.lower_bound(), Vector1d(1)));
  EXPECT_TRUE(CompareMatrices(dut.upper_bound(), Vector1d(1)));
  Eigen::VectorXd y_double;
  Eigen::Vector4d x_val(1, 2, 3, 4);
  dut.Eval(x_val, &y_double);
  EXPECT_TRUE(CompareMatrices(y_double, Vector1d(30)));

  Eigen::Matrix<double, 4, Eigen::Dynamic> x_grad(4, 2);
  x_grad << 1, 2, 3, 4, 5, 6, 7, 8;
  const AutoDiffVecXd x_autodiff =
      math::InitializeAutoDiff(x_val, x_grad);
  AutoDiffVecXd y_autodiff;
  dut.Eval(x_autodiff, &y_autodiff);
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(y_autodiff), Vector1d(30)));
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y_autodiff),
                              2 * x_val.transpose() * x_grad));

  const symbolic::Variable z0("z0");
  const symbolic::Variable z1("z1");
  const symbolic::Variable z2("z2");
  const symbolic::Variable z3("z3");
  Vector4<symbolic::Variable> z(z0, z1, z2, z3);
  VectorX<symbolic::Expression> y_symbolic;
  dut.Eval(z, &y_symbolic);
  EXPECT_PRED2(symbolic::test::ExprEqual, y_symbolic(0),
               z0 * z0 + z1 * z1 + z2 * z2 + z3 * z3);
}

TEST_F(TwoFreeBodiesConstraintTest, AddUnitQuaternionConstraintOnPlant) {
  solvers::MathematicalProgram prog;
  auto q = prog.NewContinuousVariables(plant_->num_positions());
  AddUnitQuaternionConstraintOnPlant(*plant_, q, &prog);
  EXPECT_EQ(prog.generic_constraints().size(), 2);
  Eigen::VectorXd q_val = Eigen::VectorXd::Zero(14);
  q_val.head<4>() << 0.5, -0.5, 0.5, -1.;
  q_val.segment<4>(7) << 1.0 / 3, 2.0 / 3, 2.0 / 3, 0.5;
  EXPECT_EQ(prog.generic_constraints()[0].variables().rows(), 4);
  EXPECT_EQ(prog.generic_constraints()[1].variables().rows(), 4);
  const auto result = solvers::Solve(prog, q_val);
  EXPECT_TRUE(result.is_success());
  EXPECT_NEAR(result.GetSolution(q.head<4>()).norm(), 1, 1E-5);
  EXPECT_NEAR(result.GetSolution(q.segment<4>(7)).norm(), 1, 1E-5);
}
}  // namespace multibody
}  // namespace drake
