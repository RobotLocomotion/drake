#include "drake/solvers/rotation_constraint.h"

#include <random>

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/random_rotation.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/math/rotation_matrix.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"

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
          for (const auto &b : bb_constraints) {
            Eigen::VectorXd x = prog.EvalBindingAtSolution(b);
            const Eigen::VectorXd &lb = b.constraint()->lower_bound();
            const Eigen::VectorXd &ub = b.constraint()->upper_bound();
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

bool IsFeasibleCheck(
    MathematicalProgram *prog,
    const std::shared_ptr<LinearEqualityConstraint> &feasibility_constraint,
    const Eigen::Ref<const Matrix3d> &R_sample) {
  Eigen::Map<const Eigen::Matrix<double, 9, 1>> R_sample_vec(R_sample.data());
  feasibility_constraint->UpdateLowerBound(R_sample_vec);
  feasibility_constraint->UpdateUpperBound(R_sample_vec);

  return (prog->Solve() == kSolutionFound);
}

GTEST_TEST(RotationConstraint, TestAddStaticSizeNumIntervalsPerHalfAxis) {
  MathematicalProgram prog;
  auto R = NewRotationMatrixVars(&prog);
  auto ret1 = AddRotationMatrixBilinearMcCormickMilpConstraints<1>(&prog, R);
  static_assert(
      std::is_same<
          decltype(ret1),
          std::pair<std::array<std::array<VectorDecisionVariable<1>, 3>, 3>,
                    Eigen::Matrix<double, 3, 1>>>::value,
      "Incorrect type.");

  auto ret2 = AddRotationMatrixBilinearMcCormickMilpConstraints<2>(&prog, R);
  static_assert(
      std::is_same<
          decltype(ret2),
          std::pair<std::array<std::array<VectorDecisionVariable<2>, 3>, 3>,
                    Eigen::Matrix<double, 5, 1>>>::value,
      "Incorrect type.");

  auto ret3 = AddRotationMatrixBilinearMcCormickMilpConstraints<3>(&prog, R);
  static_assert(
      std::is_same<
          decltype(ret3),
          std::pair<std::array<std::array<VectorDecisionVariable<3>, 3>, 3>,
                    Eigen::Matrix<double, 7, 1>>>::value,
      "Incorrect type.");

  auto ret4 = AddRotationMatrixBilinearMcCormickMilpConstraints<4>(&prog, R);
  static_assert(
      std::is_same<
          decltype(ret4),
          std::pair<std::array<std::array<VectorDecisionVariable<3>, 3>, 3>,
                    Eigen::Matrix<double, 9, 1>>>::value,
      "Incorrect type.");
}

class TestMcCormick : public ::testing::TestWithParam<std::tuple<bool, int>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestMcCormick)

  TestMcCormick()
      : prog_(),
        R_(NewRotationMatrixVars(&prog_)),
        replace_bilinear_product_(std::get<0>(GetParam())),
        num_intervals_per_half_axis_(std::get<1>(GetParam())),
  feasibility_constraint_{prog_.AddLinearEqualityConstraint(
      Eigen::Matrix<double, 9, 9>::Identity(),
      Eigen::Matrix<double, 9, 1>::Zero(),
      {R_.col(0), R_.col(1), R_.col(2)}).constraint()} {
    if (replace_bilinear_product_) {
      AddRotationMatrixBilinearMcCormickMilpConstraints(
          &prog_, R_, num_intervals_per_half_axis_);
    } else {
      AddRotationMatrixMcCormickEnvelopeMilpConstraints(
          &prog_, R_, num_intervals_per_half_axis_);
    }
  }

  bool IsFeasible(const Eigen::Ref<const Eigen::Matrix3d>& R_to_check) {
    return IsFeasibleCheck(&prog_, feasibility_constraint_, R_to_check);
  }

  ~TestMcCormick() override {};

 protected:
  MathematicalProgram prog_;
  MatrixDecisionVariable<3, 3> R_;
  bool replace_bilinear_product_{};  // If true, replace the bilinear product
  // with another varaible in the McCormick envelope. Otherwise, relax the
  // surface ofthe unit sphere to its convex hull.
  int num_intervals_per_half_axis_{};
  std::shared_ptr<LinearEqualityConstraint> feasibility_constraint_;
};

TEST_P(TestMcCormick, TestExactRotationMatrix) {
  // If R is exactly on SO(3), test whether it also satisfies our relaxation.

  // Test a few valid rotation matrices.
  Matrix3d R_test = Matrix3d::Identity();
  EXPECT_TRUE(IsFeasible(R_test));

  R_test = math::ZRotation(M_PI_4) * R_test;
  EXPECT_TRUE(IsFeasible(R_test));

  R_test = math::YRotation(M_PI_4) * R_test;
  EXPECT_TRUE(IsFeasible(R_test));

  R_test = math::ZRotation(M_PI_2);
  EXPECT_TRUE(IsFeasible(R_test));

  R_test = math::ZRotation(-M_PI_2);
  EXPECT_TRUE(IsFeasible(R_test));

  R_test = math::YRotation(M_PI_2);
  EXPECT_TRUE(IsFeasible(R_test));

  R_test = math::YRotation(-M_PI_2);
  EXPECT_TRUE(IsFeasible(R_test));

  // This one caught a bug (in the loop finding the most conservative linear
  // constraint for a given region) during random testing.
  R_test << 0.17082017792981191, 0.65144498431260445, -0.73921573253413542,
      -0.82327804434149443, -0.31781600529013027, -0.47032568342231595,
      -0.54132589862048197, 0.68892119955432829, 0.48203096610835455;
  EXPECT_TRUE(IsFeasible(R_test));

  std::mt19937 generator(41);
  for (int i = 0; i < 40; i++) {
    R_test = math::UniformlyRandomRotmat(generator);
    EXPECT_TRUE(IsFeasible(R_test));
  }
}

TEST_P(TestMcCormick, TestInexactRotationMatrix) {
  // If R is not exactly on SO(3), test whether it is infeasible for our SO(3)
  // relaxation.

  // Checks the dot product constraints.
  Eigen::Matrix3d R_test =
      Matrix3d::Constant(1.0 / sqrt(3.0));  // All rows and columns are
  // on the unit sphere.
  EXPECT_FALSE(IsFeasible(R_test));
  // All in different octants, all unit length, but not orthogonal.
  // R.col(0).dot(R.col(1)) = 1/3;
  R_test(0, 1) *= -1.0;
  R_test(2, 1) *= -1.0;
  R_test(0, 2) *= -1.0;
  R_test(1, 2) *= -1.0;
  // Requires 2 intervals per half axis to catch.
  if (num_intervals_per_half_axis_ == 1 && !replace_bilinear_product_)
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

  // Checks a few cases just outside the L1 ball. If we use the formulation that
  // replaces the bilinear term with another variable in the McCormick envelope,
  // then it should always be infeasible. Otherwise should be feasible for
  // num_intervals_per_half_axis_=1, but infeasible for
  // num_intervals_per_half_axis_>1.
  R_test = math::YRotation(M_PI_4);
  R_test(2, 0) -= 0.1;
  EXPECT_GT(R_test.col(0).lpNorm<1>(), 1.0);
  EXPECT_GT(R_test.row(2).lpNorm<1>(), 1.0);
  if (num_intervals_per_half_axis_ == 1 && !replace_bilinear_product_)
    EXPECT_TRUE(IsFeasible(R_test));
  else
    EXPECT_FALSE(IsFeasible(R_test));
}

INSTANTIATE_TEST_CASE_P(
    RotationTest, TestMcCormick,
    ::testing::Combine(::testing::ValuesIn<std::vector<bool>>({false, true}),
                       ::testing::ValuesIn<std::vector<int>>({1, 2})));

// Test some corner cases of McCormick envelope.
// The corner cases happens when either the innermost or the outermost corner
// of the box bmin <= x <= bmax lies on the surface of the unit sphere.
class TestMcCormickCorner
    : public ::testing::TestWithParam<std::tuple<int, bool, int>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestMcCormickCorner)

  TestMcCormickCorner()
    : prog_(),
      R_(NewRotationMatrixVars(&prog_)),
      Cpos_(),
      Cneg_(),
      orthant_(std::get<0>(GetParam())),
      is_bmin_(std::get<1>(GetParam())),
      col_idx_(std::get<2>(GetParam())) {
    DRAKE_DEMAND(orthant_ >= 0);
    DRAKE_DEMAND(orthant_ <= 7);
    std::tie(Cpos_, Cneg_, std::ignore, std::ignore) =
        AddRotationMatrixMcCormickEnvelopeMilpConstraints(&prog_, R_, 3);
  }

  ~TestMcCormickCorner() override {}

 protected:
  MathematicalProgram prog_;
  MatrixDecisionVariable<3, 3> R_;
  std::vector<Eigen::Matrix<Expression, 3, 3>> Cpos_;
  std::vector<Eigen::Matrix<Expression, 3, 3>> Cneg_;
  int orthant_;  // Index of the orthant that R_.col(col_idx_) is in.
  bool is_bmin_;  // If true, then the box bmin <= x <= bmax intersects with the
                  // surface of the unit sphere at the unique point bmin;
                  // otherwise it intersects at the unique point bmax;
  int col_idx_;  // R_.col(col_idx_) will be fixed to a vertex of the box, and
                 // also this point is on the surface of the unit sphere.
};

TEST_P(TestMcCormickCorner, TestOrthogonal) {
  // box_pt is a vertex of the box, and also lies exactly on the surface of
  // the unit sphere.
  Eigen::Vector3d box_pt(1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0);
  for (int axis = 0; axis < 3; ++axis) {
    if (orthant_ & 1 << axis) {
      box_pt(axis) *= -1;
    }
  }

  int free_axis0 = (col_idx_ + 1) % 3;
  int free_axis1 = (col_idx_ + 2) % 3;
  // If R_.col(i) == box_pt, and box_pt is on the surface of the unit sphere,
  // while also being either bmin or bmax of the box bmin <= x <= bmax, then
  // the solution should satisfy R.col(j) ⊥ box_pt and R.col(k) ⊥ box_pt
  prog_.AddBoundingBoxConstraint(box_pt, box_pt, R_.col(col_idx_));

  // If bmin is the unique intersection point, here we document when the box is
  // in the first orthant (+++)
  // Cpos[1](0, col_idx_) = 1 => 1 / 3 <= R_(0, col_idx_) <= 2 / 3
  // Cpos[2](1, col_idx_) = 1 => 2 / 3 <= R_(1, col_idx_) <= 1
  // Cpos[2](2, col_idx_) = 1 => 2 / 3 <= R_(2, col_idx_) <= 1
  // If bmax is the unique intersection point, here we document when the box is
  // in the first orthant (+++)
  // Cpos[0](0, col_idx_) = 1 => 0 <= R_(0, col_idx_) <= 1 / 3
  // Cpos[1](1, col_idx_) = 1 => 1 / 3 <= R_(1, col_idx_) <= 2 / 3
  // Cpos[1](2, col_idx_) = 1 => 1 / 3 <= R_(2, col_idx_) <= 2 / 3

  // orthant_C[i](j) is either Cpos[i](j, col_idx_) or Cneg[i](j, col_idx_),
  // depending on the orthant.
  std::array<Eigen::Matrix<Expression, 3, 1>, 3> orthant_C;

  for (int i = 0; i < 3; ++i) {
    for (int axis = 0; axis < 3; ++axis) {
      if (orthant_ & 1 << axis) {
        orthant_C[i](axis) = Cneg_[i](axis, col_idx_);
      } else {
        orthant_C[i](axis) = Cpos_[i](axis, col_idx_);
      }
    }
  }
  if (is_bmin_) {
    prog_.AddLinearConstraint(1 == orthant_C[1](0));
    prog_.AddLinearConstraint(Eigen::Vector2d::Ones() ==
      orthant_C[2].block<2, 1>(1, 0));
  } else {
    prog_.AddLinearConstraint(1 == orthant_C[0](0));
    prog_.AddLinearConstraint(Eigen::Vector2d::Ones() ==
      orthant_C[1].block<2, 1>(1, 0));
  }

  // Add a cost function to try to make the column of R not perpendicular.
  prog_.AddLinearCost(R_.col(free_axis0).dot(box_pt) +
                      R_.col(free_axis1).dot(box_pt));

  SolutionResult sol_result = prog_.Solve();
  EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);
  const auto R_val = prog_.GetSolution(R_);
  std::vector<Eigen::Matrix3d> Bpos_val(3);
  std::vector<Eigen::Matrix3d> Bneg_val(3);
  EXPECT_NEAR(R_val.col(free_axis0).dot(box_pt), 0, 1E-4);
  EXPECT_NEAR(R_val.col(free_axis1).dot(box_pt), 0, 1E-4);
  EXPECT_TRUE(CompareMatrices(box_pt.cross(R_val.col(free_axis0)),
                              R_val.col(free_axis1), 1E-4,
                              MatrixCompareType::absolute));
}

INSTANTIATE_TEST_CASE_P(
    RotationTest, TestMcCormickCorner,
    ::testing::Combine(
        ::testing::ValuesIn<std::vector<int>>({0, 1, 2, 3, 4, 5, 6,
                                               7}),             // Orthant
        ::testing::ValuesIn<std::vector<bool>>({true, false}),  // bmin or bmax
        ::testing::ValuesIn<std::vector<int>>({0, 1, 2})));     // column index

// Make sure that no two row or column vectors in R, which satisfies the
// McCormick relaxation, can lie in either the same or the opposite orthant.
class TestMcCormickOrthant
    : public ::testing::TestWithParam<
          std::tuple<int, int, bool, std::pair<int, int>, bool, bool>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestMcCormickOrthant)

  TestMcCormickOrthant()
      : prog_(),
        R_(NewRotationMatrixVars(&prog_)) {
    const int num_bin = std::get<0>(GetParam());
    const int orthant = std::get<1>(GetParam());
    const bool is_row_vector = std::get<2>(GetParam());
    const int idx0 = std::get<3>(GetParam()).first;
    const int idx1 = std::get<3>(GetParam()).second;
    const bool is_same_orthant = std::get<4>(GetParam());
    const bool replace_bilinear = std::get<5>(GetParam());
    DRAKE_DEMAND(idx0 != idx1);
    DRAKE_DEMAND(idx0 >= 0);
    DRAKE_DEMAND(idx1 >= 0);
    DRAKE_DEMAND(idx0 <= 2);
    DRAKE_DEMAND(idx1 <= 2);

    if (replace_bilinear) {
      AddRotationMatrixBilinearMcCormickMilpConstraints(&prog_, R_, num_bin);
    } else {
      AddRotationMatrixMcCormickEnvelopeMilpConstraints(&prog_, R_, num_bin);
    }

    MatrixDecisionVariable<3, 3> R_hat = R_;
    if (is_row_vector) {
      R_hat = R_.transpose();
    }
    Eigen::Vector3d vec0_lb =
        Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
    Eigen::Vector3d vec0_ub =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    // positive or negative x axis?
    if (orthant & (1 << 2)) {
      vec0_lb(0) = 1E-3;
    } else {
      vec0_ub(0) = -1E-3;
    }
    // positive or negative y axis?
    if (orthant & (1 << 1)) {
      vec0_lb(1) = 1E-3;
    } else {
      vec0_ub(1) = -1E-3;
    }
    // positive or negative z axis?
    if (orthant & (1 << 0)) {
      vec0_lb(2) = 1E-3;
    } else {
      vec0_ub(2) = -1E-3;
    }
    // If we want to verify vec1 and vec2 cannot be in the SAME orthant,
    // then set vec1_lb = vec0_lb, vec1_ub = vec0_ub;
    // otherwise if we want to verify vec1 and vec2 cannot be in the OPPOSITE
    // orthant, then set vec1_lb = -vec1_ub, vec1_ub = -vec0_lb.
    Eigen::Vector3d vec1_lb = vec0_lb;
    Eigen::Vector3d vec1_ub = vec0_ub;
    if (!is_same_orthant) {
      vec1_lb = -vec0_ub;
      vec1_ub = -vec0_lb;
    }
    prog_.AddBoundingBoxConstraint(vec0_lb, vec0_ub, R_hat.col(idx0));
    prog_.AddBoundingBoxConstraint(vec1_lb, vec1_ub, R_hat.col(idx1));
  }

  ~TestMcCormickOrthant() override {}

 protected:
  MathematicalProgram prog_;
  MatrixDecisionVariable<3, 3> R_;
};

TEST_P(TestMcCormickOrthant, test) {
  GurobiSolver gurobi_solver;
  if (gurobi_solver.available()) {
    SolutionResult sol_result = gurobi_solver.Solve(prog_);
    // Since no two row or column vectors in R can lie in either the same of the
    // opposite orthant, the program should be infeasible.
    EXPECT_TRUE(sol_result == SolutionResult::kInfeasible_Or_Unbounded ||
        sol_result == SolutionResult::kInfeasibleConstraints);
  }
}

std::array<std::pair<int, int>, 3> vector_indices() {
  std::array<std::pair<int, int>, 3> idx = {{{0, 1}, {0, 2}, {1, 2}}};
  return idx;
}

INSTANTIATE_TEST_CASE_P(
    RotationTest, TestMcCormickOrthant,
    ::testing::Combine(::testing::ValuesIn<std::vector<int>>(
                           {1}),  // # of intervals per half axis
                       ::testing::ValuesIn<std::vector<int>>(
                           {0, 1, 2, 3, 4, 5, 6, 7}),  // orthant index
                       ::testing::ValuesIn<std::vector<bool>>(
                           {false, true}),  // row vector or column vector
                       ::testing::ValuesIn<std::array<std::pair<int, int>, 3>>(
                           vector_indices()),  // vector indices
                       ::testing::ValuesIn<std::vector<bool>>(
                           {false, true}),  // same of opposite orthant
                       ::testing::ValuesIn<std::vector<bool>>(
                           {false, true})));  // replace bilinear or not.
}  // namespace
}  // namespace solvers
}  // namespace drake
