#include "drake/solvers/rotation_constraint.h"

#include <random>

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rotation_matrix.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solve.h"

using Eigen::Vector3d;
using Eigen::Matrix3d;

using drake::symbolic::Expression;

using std::sqrt;

namespace drake {
namespace solvers {
namespace {
void AddObjective(MathematicalProgram* prog,
                  const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R,
                  const Eigen::Ref<const Matrix3d>& R_desired) {
  const auto R_error = R - R_desired;

  // sigma >= |error|_2
  MatrixDecisionVariable<1, 1> sigma =
      prog->NewContinuousVariables<1, 1>("sigma");
  // trace(R_errorᵀ * R_error) = sum_{i,j} R_error(i,j)²
  prog->AddLorentzConeConstraint(
      sigma(0), (R_error.transpose() * R_error).trace(), 1E-15);

  // min sigma
  prog->AddCost(sigma(0));
}

// Iterates over possible setting of the RPY limits flag, and for each setting
// evaluates a mesh of points within those limits.  This test confirms that
// of the rotation matrices generated from rotations with those limits are
// still feasible after the RPY limits constraints have been applied.
class TestRpyLimitsFixture : public ::testing::TestWithParam<int> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestRpyLimitsFixture)
  TestRpyLimitsFixture() = default;
};

TEST_P(TestRpyLimitsFixture, TestRpyLimits) {
  const int limits = GetParam();
  // Add brace scope to avoid reflowing all of this code.
  {
    MathematicalProgram prog;
    auto Rvar = NewRotationMatrixVars(&prog);
    AddBoundingBoxConstraintsImpliedByRollPitchYawLimits(
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
          const drake::math::RollPitchYaw<double> rpy(roll, pitch, yaw);
          Matrix3d R = rpy.ToMatrix3ViaRotationMatrix();
          Eigen::Map<Eigen::Matrix<double, 9, 1>> vecR(R.data(), R.size());
          prog.SetInitialGuessForAllVariables(vecR);
          for (const auto& b : bb_constraints) {
            const Eigen::VectorXd x = prog.EvalBindingAtInitialGuess(b);
            const Eigen::VectorXd& lb = b.evaluator()->lower_bound();
            const Eigen::VectorXd& ub = b.evaluator()->upper_bound();
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

INSTANTIATE_TEST_SUITE_P(
    RotationTest, TestRpyLimitsFixture,
    ::testing::Range(1 << 1, 1 << 7, 2));

// Sets up and solves an optimization:
// <pre>
//    min_R  sum_{i,j} |R(i,j) - R_desired(i,j)|^2
// </pre>
// where the columans (and rows) of R_desired are outside the unit ball.
// Confirms that the SpectralPSD constraint results in a matrix with columns
// and rows of unit length (or less), and that the actual PSD constraint (typed
// in a very different way here) was satisfied.
GTEST_TEST(RotationTest, TestSpectralPsd) {
  MathematicalProgram prog;
  auto Rvar = NewRotationMatrixVars(&prog);

  // R_desired is outside the unit ball.
  AddObjective(&prog, Rvar, 2 * Eigen::Matrix<double, 3, 3>::Ones());
  AddRotationMatrixSpectrahedralSdpConstraint(&prog, Rvar);
  MathematicalProgramResult result = Solve(prog);
  ASSERT_TRUE(result.is_success());

  Matrix3d R = result.GetSolution(Rvar);

  double tol = 1e-6;
  EXPECT_LE(R.col(0).lpNorm<2>(), 1 + tol);
  EXPECT_LE(R.col(1).lpNorm<2>(), 1 + tol);
  EXPECT_LE(R.col(2).lpNorm<2>(), 1 + tol);
  EXPECT_LE(R.row(0).lpNorm<2>(), 1 + tol);
  EXPECT_LE(R.row(1).lpNorm<2>(), 1 + tol);
  EXPECT_LE(R.row(2).lpNorm<2>(), 1 + tol);

  // Check eq 10 in https://arxiv.org/pdf/1403.4914.pdf
  Eigen::Matrix4d U;
  // clang-format off
  // NOLINTNEXTLINE(whitespace/comma)
  U << 1 - R(0, 0) - R(1, 1) + R(2, 2), R(0, 2) + R(2, 0), R(0, 1) - R(1, 0),
      R(1, 2) + R(2, 1),
      // NOLINTNEXTLINE(whitespace/comma)
      R(0, 2) + R(2, 0), 1 + R(0, 0) - R(1, 1) - R(2, 2), R(1, 2) - R(2, 1),
      R(0, 1) + R(1, 0),
      // NOLINTNEXTLINE(whitespace/comma)
      R(0, 1) - R(1, 0), R(1, 2) - R(2, 1), 1 + R(0, 0) + R(1, 1) + R(2, 2),
      R(2, 0) - R(0, 2),
      // NOLINTNEXTLINE(whitespace/comma)
      R(1, 2) + R(2, 1), R(0, 1) + R(1, 0), R(2, 0) - R(0, 2), 1 - R(0, 0)
      + R(1, 1) - R(2, 2);
  // clang-format on

  const Eigen::Array4d lambda_mag{U.eigenvalues().array().real()};
  for (int i = 0; i < 4; i++) EXPECT_GE(lambda_mag(i), -tol);
}

// Sets up and solves an optimization:
// <pre>
//    min_R  sum_{i,j} |R(i,j) - R_desired(i,j)|^2
// </pre>
// where the columns (and rows) of R_desired are outside the unit ball.
// Confirms that the Orthonormal SOCP constraints result in a solution matrix
// with columns and rows of unit length or less, and that the specific
// orthogonality relaxation implemented by the routine is satisfied.
GTEST_TEST(RotationTest, TestOrthonormal) {
  MathematicalProgram prog;
  auto Rvar = NewRotationMatrixVars(&prog);

  // R_desired is outside the unit ball.
  AddObjective(&prog, Rvar, 2 * Eigen::Matrix<double, 3, 3>::Ones());
  AddRotationMatrixOrthonormalSocpConstraint(&prog, Rvar);
  MathematicalProgramResult result = Solve(prog);
  ASSERT_TRUE(result.is_success());

  Matrix3d R = result.GetSolution(Rvar);

  double tol = 1e-4;
  EXPECT_LE(R.col(0).lpNorm<2>(), 1 + tol);
  EXPECT_LE(R.col(1).lpNorm<2>(), 1 + tol);
  EXPECT_LE(R.col(2).lpNorm<2>(), 1 + tol);
  EXPECT_LE(2 * std::abs(R.col(0).dot(R.col(1))),
            2 - R.col(0).dot(R.col(0)) - R.col(1).dot(R.col(1)) + tol);
  EXPECT_LE(2 * std::abs(R.col(1).dot(R.col(2))),
            2 - R.col(1).dot(R.col(1)) - R.col(2).dot(R.col(2)) + tol);
  EXPECT_LE(2 * std::abs(R.col(0).dot(R.col(2))),
            2 - R.col(0).dot(R.col(0)) - R.col(2).dot(R.col(2)) + tol);

  EXPECT_LE(R.row(0).lpNorm<2>(), 1 + tol);
  EXPECT_LE(R.row(1).lpNorm<2>(), 1 + tol);
  EXPECT_LE(R.row(2).lpNorm<2>(), 1 + tol);
  EXPECT_LE(2 * std::abs(R.row(0).dot(R.row(1))),
            2 - R.row(0).dot(R.row(0)) - R.row(1).dot(R.row(1)) + tol);
  EXPECT_LE(2 * std::abs(R.row(1).dot(R.row(2))),
            2 - R.row(0).dot(R.row(0)) - R.row(1).dot(R.row(1)) + tol);
  EXPECT_LE(2 * std::abs(R.row(0).dot(R.row(2))),
            2 - R.row(0).dot(R.row(0)) - R.row(1).dot(R.row(1)) + tol);
}
}  // namespace
}  // namespace solvers
}  // namespace drake

int main(int argc, char** argv) {
  // Ensure that we have the MOSEK license for the entire duration of this test,
  // so that we do not have to release and re-acquire the license for every
  // test.
  auto mosek_license = drake::solvers::MosekSolver::AcquireLicense();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
