#include "drake/solvers/rotation.h"

#include <random>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

void AddObjective(MathematicalProgram* prog, const DecisionVariableMatrixX& R,
                  const Eigen::Ref<const Eigen::Matrix3d>& R_desired) {
  DecisionVariableMatrix<9, 1> error =
      prog->AddContinuousVariables<9, 1>("error");

  Eigen::Map<const Eigen::Matrix<double, 9, 1>> vecR_desired(R_desired.data(),
                                                             R_desired.size());

  // error - vec(R) = vec(R_desired)
  Eigen::Matrix<double, 9, 18> A;
  A << Eigen::Matrix<double, 9, 9>::Identity(),
      -Eigen::Matrix<double, 9, 9>::Identity();
  prog->AddLinearEqualityConstraint(A, vecR_desired,
                                    {error, R.col(0), R.col(1), R.col(2)});

  // sigma >= |error|_2
  DecisionVariableMatrix<1, 1> sigma =
      prog->AddContinuousVariables<1, 1>("sigma");
  prog->AddLorentzConeConstraint({sigma, error});

  // min sigma
  prog->AddLinearCost(Vector1d::Ones(), {sigma});
}

/*
void RotationMatrixCheck(const Eigen::Ref<const Eigen::Matrix3d>& R)
{
  const double tol = 1e-3;
  EXPECT_NEAR(R.col(0).lpNorm<2>(),1,tol);
  EXPECT_NEAR(R.col(1).lpNorm<2>(),1,tol);
  EXPECT_NEAR(R.col(2).lpNorm<2>(),1,tol);
  EXPECT_NEAR(R.col(0).dot(R.col(1)),0,tol);
  EXPECT_NEAR(R.col(0).dot(R.col(2)),0,tol);
  EXPECT_NEAR(R.col(1).dot(R.col(2)),0,tol);

  const double det = R.determinant();
  bool is_invertible = std::abs(det)>tol;
  if (!is_invertible) {
    std::cout << "R is not invertible:" << std::endl;
    std::cout << R << std::endl;
  }
  ASSERT_TRUE(is_invertible);
  EXPECT_TRUE(CompareMatrices(R.inverse(),R.transpose()));
  EXPECT_NEAR(det,1,tol);
}
*/

GTEST_TEST(RotationTest, TestRPYLimits) {
  for (int limits = (1 << 1); limits < (1 << 7); limits += 2) {
    MathematicalProgram prog;
    auto Rvar = NewRotationMatrixVars(&prog);
    AddRollPitchYawLimitBoundingBoxConstraints(
        &prog, Rvar, static_cast<RollPitchYawLimits>(limits));
    auto bb_constraints = prog.bounding_box_constraints();

    // Bounds are loose, so just test that feasible points are indeed feasible.
    const double rmin =
        (limits & kRoll_0_to_PI)
            ? 0
            : (limits & kRoll_NegPI_2_to_PI_2) ? -M_PI_2 : -M_PI;
    const double rmax = (limits & kRoll_NegPI_2_to_PI_2) ? M_PI_2 : M_PI;
    const double pmin =
        (limits & kPitch_0_to_PI)
            ? 0
            : (limits & kPitch_NegPI_2_to_PI_2) ? -M_PI_2 : -M_PI;
    const double pmax = (limits & kPitch_NegPI_2_to_PI_2) ? M_PI_2 : M_PI;
    const double ymin = (limits & kYaw_0_to_PI)
                            ? 0
                            : (limits & kYaw_NegPI_2_to_PI_2) ? -M_PI_2 : -M_PI;
    const double ymax = (limits & kYaw_NegPI_2_to_PI_2) ? M_PI_2 : M_PI;

    for (double roll = rmin; roll <= rmax; roll += M_PI / 6) {
      for (double pitch = pmin; pitch <= pmax; pitch += M_PI / 6) {
        for (double yaw = ymin; yaw <= ymax; yaw += M_PI / 6) {
          Eigen::Matrix3d R =
              drake::math::rpy2rotmat(Eigen::Vector3d(roll, pitch, yaw));
          Eigen::Map<Eigen::Matrix<double, 9, 1>> vecR(R.data(), R.size());
          prog.SetDecisionVariableValues(vecR);
          for (const auto& b : bb_constraints) {
            Eigen::VectorXd x = b.VariableListToVectorXd();
            const Eigen::VectorXd& lb = b.constraint()->lower_bound();
            const Eigen::VectorXd& ub = b.constraint()->upper_bound();
            for (int i = 0; i < x.size(); i++) {
              EXPECT_GE(x(i), lb(i));
              EXPECT_LE(x(i), ub(i));
            }
          }
        }
      }
    }
  }
}

GTEST_TEST(RotationTest, TestSpectralPsd) {
  MathematicalProgram prog;
  auto Rvar = NewRotationMatrixVars(&prog);

  // R_desired is outside the unit ball.
  AddObjective(&prog, Rvar, 2 * Eigen::Matrix<double, 3, 3>::Ones());
  AddRotationMatrixSpectrahedralSdpConstraint(&prog, Rvar);
  ASSERT_EQ(prog.Solve(), kSolutionFound);

  Eigen::Matrix3d R = GetSolution(Rvar);

  // Check eq 10 in https://arxiv.org/pdf/1403.4914.pdf
  Eigen::Matrix4d U;
  // clang-format off
  // NOLINTNEXTLINE(whitespace/comma)
  U << 1-R(0,0)-R(1,1)+R(2,2), R(0,2)+R(2,0), R(0,1)-R(1,0), R(1,2)+R(2,1),
  // NOLINTNEXTLINE(whitespace/comma)
       R(0,2)+R(2,0), 1+R(0,0)-R(1,1)-R(2,2), R(1,2)-R(2,1), R(0,1)+R(1,0),
  // NOLINTNEXTLINE(whitespace/comma)
       R(0,1)-R(1,0), R(1,2)-R(2,1), 1+R(0,0)+R(1,1)+R(2,2), R(2,0)-R(0,2),
  // NOLINTNEXTLINE(whitespace/comma)
       R(1,2)+R(2,1), R(0,1)+R(1,0), R(2,0)-R(0,2), 1-R(0,0)+R(1,1)-R(2,2);
  // clang-format on

  auto lambda_mag = U.eigenvalues().array().abs();
  for (int i = 0; i < 4; i++) EXPECT_GE(lambda_mag(i), 0);
}

GTEST_TEST(RotationTest, TestOrthonormal) {
  MathematicalProgram prog;
  auto Rvar = NewRotationMatrixVars(&prog);

  // R_desired is outside the unit ball.
  AddObjective(&prog, Rvar, 2 * Eigen::Matrix<double, 3, 3>::Ones());
  AddRotationMatrixOrthonormalSocpConstraint(&prog, Rvar);
  ASSERT_EQ(prog.Solve(), kSolutionFound);

  Eigen::Matrix3d R = GetSolution(Rvar);

  double tol = 1e-4;
  EXPECT_LE(R.col(0).lpNorm<2>(), 1 + tol);
  EXPECT_LE(R.col(1).lpNorm<2>(), 1 + tol);
  EXPECT_LE(R.col(2).lpNorm<2>(), 1 + tol);
  EXPECT_LE(std::abs(R.col(0).dot(R.col(1))),
            2 - R.col(0).lpNorm<2>() - R.col(1).lpNorm<2>() + tol);
  EXPECT_LE(std::abs(R.col(1).dot(R.col(2))),
            2 - R.col(1).lpNorm<2>() - R.col(2).lpNorm<2>() + tol);
  EXPECT_LE(std::abs(R.col(0).dot(R.col(2))),
            2 - R.col(0).lpNorm<2>() - R.col(2).lpNorm<2>() + tol);
}

GTEST_TEST(RotationTest, TestL1Norm) {
  MathematicalProgram prog;
  auto Rvar = NewRotationMatrixVars(&prog);

  // R_desired is inside the unit ball.
  AddObjective(&prog, Rvar, .1 * Eigen::Matrix<double, 3, 3>::Ones());
  AddRotationMatrixL1NormMilpConstraint(&prog, Rvar);
  ASSERT_EQ(prog.Solve(), kSolutionFound);

  Eigen::Matrix3d R = GetSolution(Rvar);

  double tol = 1e-4;
  EXPECT_GE(R.col(0).lpNorm<1>(), 1 - tol);
  EXPECT_GE(R.col(1).lpNorm<1>(), 1 - tol);
  EXPECT_GE(R.col(2).lpNorm<1>(), 1 - tol);
}

}  // namespace solvers
}  // namespace drake
