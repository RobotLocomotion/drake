#include "drake/solvers/rotation_constraint.h"
#include "drake/solvers/rotation_constraint_internal.h"

#include <random>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/random_rotation.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/math/rotation_matrix.h"
#include "drake/solvers/mathematical_program.h"

using Eigen::Vector3d;
using Eigen::Matrix3d;

namespace drake {
namespace solvers {

void AddObjective(MathematicalProgram* prog,
                  const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R,
                  const Eigen::Ref<const Matrix3d>& R_desired) {
  MatrixDecisionVariable<9, 1> error =
      prog->NewContinuousVariables<9, 1>("error");

  Eigen::Map<const Eigen::Matrix<double, 9, 1>> vecR_desired(R_desired.data(),
                                                             R_desired.size());

  // error - vec(R) = vec(R_desired)
  Eigen::Matrix<double, 9, 18> A;
  A << Eigen::Matrix<double, 9, 9>::Identity(),
      -Eigen::Matrix<double, 9, 9>::Identity();
  prog->AddLinearEqualityConstraint(A, vecR_desired,
                                    {error, R.col(0), R.col(1), R.col(2)});

  // sigma >= |error|_2
  MatrixDecisionVariable<1, 1> sigma =
      prog->NewContinuousVariables<1, 1>("sigma");
  prog->AddLorentzConeConstraint({sigma, error});

  // min sigma
  prog->AddLinearCost(Vector1d::Ones(), sigma);
}

// Iterates over possible setting of the RPY limits flag, and for each setting
// evaluates a mesh of points within those limits.  This test confirms that
// of the rotation matrices generated from rotations with those limits are
// still feasible after the RPY limits constraints have been applied.
GTEST_TEST(RotationTest, TestRPYLimits) {
  for (int limits = (1 << 1); limits < (1 << 7); limits += 2) {
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
          Matrix3d R = drake::math::rpy2rotmat(Vector3d(roll, pitch, yaw));
          Eigen::Map<Eigen::Matrix<double, 9, 1>> vecR(R.data(), R.size());
          prog.SetDecisionVariableValues(vecR);
          for (const auto& b : bb_constraints) {
            Eigen::VectorXd x = prog.EvalBindingAtSolution(b);
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
  ASSERT_EQ(prog.Solve(), kSolutionFound);

  Matrix3d R = prog.GetSolution(Rvar);

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
  U << 1-R(0,0)-R(1,1)+R(2,2), R(0,2)+R(2,0), R(0,1)-R(1,0), R(1,2)+R(2,1),
  // NOLINTNEXTLINE(whitespace/comma)
       R(0,2)+R(2,0), 1+R(0,0)-R(1,1)-R(2,2), R(1,2)-R(2,1), R(0,1)+R(1,0),
  // NOLINTNEXTLINE(whitespace/comma)
       R(0,1)-R(1,0), R(1,2)-R(2,1), 1+R(0,0)+R(1,1)+R(2,2), R(2,0)-R(0,2),
  // NOLINTNEXTLINE(whitespace/comma)
       R(1,2)+R(2,1), R(0,1)+R(1,0), R(2,0)-R(0,2), 1-R(0,0)+R(1,1)-R(2,2);
  // clang-format on

  auto lambda_mag = U.eigenvalues().array().real();
  for (int i = 0; i < 4; i++) EXPECT_GE(lambda_mag(i), -tol);
}

// Sets up and solves an optimization:
// <pre>
//    min_R  sum_{i,j} |R(i,j) - R_desired(i,j)|^2
// </pre>
// where the columans (and rows) of R_desired are outside the unit ball.
// Confirms that the Orthonormal SOCP constraints result in a solution matrix
// with columns and rows of unit length or less, and that the specific
// orthogonality relaxation implemented by the routine is satisfied.
GTEST_TEST(RotationTest, TestOrthonormal) {
  MathematicalProgram prog;
  auto Rvar = NewRotationMatrixVars(&prog);

  // R_desired is outside the unit ball.
  AddObjective(&prog, Rvar, 2 * Eigen::Matrix<double, 3, 3>::Ones());
  AddRotationMatrixOrthonormalSocpConstraint(&prog, Rvar);
  ASSERT_EQ(prog.Solve(), kSolutionFound);

  Matrix3d R = prog.GetSolution(Rvar);

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

void CompareIntersectionResults(std::vector<Vector3d> desired,
                                std::vector<Vector3d> actual) {
  EXPECT_EQ(desired.size(), actual.size());
  Eigen::Matrix<bool, Eigen::Dynamic, 1> used =
      Eigen::Matrix<bool, Eigen::Dynamic, 1>::Constant(desired.size(), false);
  double tol = 1e-8;
  for (int i = 0; i < static_cast<int>(desired.size()); i++) {
    // need not be in the same order.
    bool found_match = false;
    for (int j = 0; j < static_cast<int>(desired.size()); j++) {
      if (used(j)) continue;
      if ((desired[i] - actual[j]).lpNorm<2>() < tol) {
        used(j) = true;
        found_match = true;
        continue;
      }
    }
    EXPECT_TRUE(found_match);
  }
}

// Test a number of closed-form solutions for the intersection of a box in the
// positive orthant with the unit circle.
GTEST_TEST(RotationTest, TestIntersectBoxWithCircle) {
  std::vector<Vector3d> desired;

  // Entire first octant.
  Vector3d box_min(0, 0, 0);
  Vector3d box_max(1, 1, 1);
  desired.push_back(Vector3d(1, 0, 0));
  desired.push_back(Vector3d(0, 1, 0));
  desired.push_back(Vector3d(0, 0, 1));
  CompareIntersectionResults(
      desired, internal::IntersectBoxWUnitCircle(box_min, box_max));

  // Lifts box bottom (in z).  Still has 3 solutions.
  box_min << 0, 0, 1.0 / 3.0;
  desired[0] << std::sqrt(8) / 3.0, 0, 1.0 / 3.0;
  desired[1] << 0, std::sqrt(8) / 3.0, 1.0 / 3.0;
  CompareIntersectionResults(
      desired, internal::IntersectBoxWUnitCircle(box_min, box_max));

  // Lowers box top (in z).  Now we have four solutions.
  box_max << 1, 1, 2.0 / 3.0;
  desired[2] << std::sqrt(5) / 3.0, 0, 2.0 / 3.0;
  desired.push_back(Vector3d(0, std::sqrt(5) / 3.0, 2.0 / 3.0));
  CompareIntersectionResults(
      desired, internal::IntersectBoxWUnitCircle(box_min, box_max));

  // Gets a different four edges by shortening the box (in x).
  box_max(0) = .5;
  desired[0] << .5, std::sqrt(23.0) / 6.0, 1.0 / 3.0;
  desired[2] << .5, std::sqrt(11.0) / 6.0, 2.0 / 3.0;
  CompareIntersectionResults(
      desired, internal::IntersectBoxWUnitCircle(box_min, box_max));

  // Now three edges again as we shorten the box (in y).
  box_max(1) = .6;
  desired.pop_back();
  desired[0] << .5, std::sqrt(11.0) / 6.0, 2.0 / 3.0;
  desired[1] << 2 * std::sqrt(11.0) / 15.0, .6, 2.0 / 3.0;
  desired[2] << .5, .6, std::sqrt(39.0) / 10.0;
  CompareIntersectionResults(
      desired, internal::IntersectBoxWUnitCircle(box_min, box_max));

  // All four intersections are on the vertical edges.
  box_min << 1.0 / 3.0, 1.0 / 3.0, 0;
  box_max << 2.0 / 3.0, 2.0 / 3.0, 1;
  desired[0] << 1.0 / 3.0, 1.0 / 3.0, std::sqrt(7.0) / 3.0;
  desired[1] << 2.0 / 3.0, 1.0 / 3.0, 2.0 / 3.0;
  desired[2] << 1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0;
  desired.push_back(Vector3d(2.0 / 3.0, 2.0 / 3.0, 1.0 / 3.0));
  CompareIntersectionResults(
      desired, internal::IntersectBoxWUnitCircle(box_min, box_max));

  // Last case is box_max right on the unit sphere.
  box_max << 1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0;
  // Should return just the single point.
  desired.erase(desired.begin() + 1, desired.end());
  desired[0] = box_max;
  CompareIntersectionResults(
      desired, internal::IntersectBoxWUnitCircle(box_min, box_max));
}

bool IsFeasibleCheck(
    MathematicalProgram* prog,
    const std::shared_ptr<LinearEqualityConstraint>& feasibility_constraint,
    const Eigen::Ref<const Matrix3d>& R_sample) {
  Eigen::Map<const Eigen::Matrix<double, 9, 1>> R_sample_vec(R_sample.data());
  feasibility_constraint->UpdateLowerBound(R_sample_vec);
  feasibility_constraint->UpdateUpperBound(R_sample_vec);

  return (prog->Solve() == kSolutionFound);
}

// Tests that a number of feasible rotation matrices are indeed feasible for
// McCormick envelopes with 1 or 2 bins per axis.  Also checks that some
// specific matrices that should be ruled out by the orthogonality and cross-
// product constraints are indeed infeasible.  Finally, checks a few points
// that we expect to be reported as feasible for the loose envelope, but are
// then correctly reported as infeasible given the tighter envelope.
GTEST_TEST(RotationTest, TestMcCormick) {
  std::mt19937 generator(41);
  std::normal_distribution<double> randn;
  std::uniform_int_distribution<> rand(0, 1 << 6);

  for (int num_bins = 1; num_bins < 3; num_bins++) {
    MathematicalProgram prog;
    MatrixDecisionVariable<3, 3> R = NewRotationMatrixVars(&prog);

    AddRotationMatrixMcCormickEnvelopeMilpConstraints(&prog, R, num_bins);

    // Feasibility constraint
    std::shared_ptr<LinearEqualityConstraint> feasibility_constraint =
        prog.AddLinearEqualityConstraint(
            Eigen::Matrix<double, 9, 9>::Identity(),
            Eigen::Matrix<double, 9, 1>::Zero(),
            {R.col(0), R.col(1), R.col(2)});

    // Use a simple lambda to make the tests more readable below.
    auto IsFeasible = [&](Matrix3d R_to_check) -> bool {
      return IsFeasibleCheck(&prog, feasibility_constraint, R_to_check);
    };

    // Test a few valid rotation matrices.
    Matrix3d R_test = Matrix3d::Identity();
    EXPECT_TRUE(IsFeasible(R_test));

    R_test = math::ZRotation(M_PI_4) * R_test;
    EXPECT_TRUE(IsFeasible(R_test));

    R_test = math::YRotation(M_PI_4) * R_test;
    EXPECT_TRUE(IsFeasible(R_test));

    // This one caught a bug (in the loop finding the most conservative linear
    // constraint for a given region) during random testing.
    R_test << 0.17082017792981191, 0.65144498431260445, -0.73921573253413542,
        -0.82327804434149443, -0.31781600529013027, -0.47032568342231595,
        -0.54132589862048197, 0.68892119955432829, 0.48203096610835455;
    EXPECT_TRUE(IsFeasible(R_test));

    for (int i = 0; i < 20; i++) {
      R_test = math::UniformlyRandomRotmat(generator);
      EXPECT_TRUE(IsFeasible(R_test));
    }

    // Checks the dot product constraints.
    R_test = Matrix3d::Constant(1.0 / sqrt(3.0));  // All rows and columns are
                                                   // on the unit sphere.
    EXPECT_FALSE(IsFeasible(R_test));
    // All in different octants, all unit length, but not orthogonal.
    // R.col(0).dot(R.col(1)) = 1/3;
    R_test(0, 1) *= -1.0;
    R_test(2, 1) *= -1.0;
    R_test(0, 2) *= -1.0;
    R_test(1, 2) *= -1.0;
    // Requires 2 bins to catch.
    if (num_bins == 1)
      EXPECT_TRUE(IsFeasible(R_test));
    else
      EXPECT_FALSE(IsFeasible(R_test));

    // Checks the det(R)=-1 case.
    // (only ruled out by the cross-product constraint).
    R_test = Matrix3d::Identity();
    R_test(2, 2) = -1;
    EXPECT_FALSE(IsFeasible(R_test));

    R_test = math::ZRotation(M_PI_4) * R_test;
    EXPECT_FALSE(IsFeasible(R_test));

    R_test = math::YRotation(M_PI_4) * R_test;
    EXPECT_FALSE(IsFeasible(R_test));

    // Checks a few cases just outside the L1 ball.  Should be feasible for
    // num_bins=1, but infeasible for num_bins>1.
    R_test = math::YRotation(M_PI_4);
    R_test(2, 0) -= 0.2;
    EXPECT_GT(R_test.col(0).lpNorm<1>(), 1.0);
    EXPECT_GT(R_test.row(2).lpNorm<1>(), 1.0);
    if (num_bins == 1)
      EXPECT_TRUE(IsFeasible(R_test));
    else
      EXPECT_FALSE(IsFeasible(R_test));
  }
}

}  // namespace solvers
}  // namespace drake
