/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mixed_integer_rotation_constraint.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/gray_code.h"
#include "drake/solvers/integer_optimization_util.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/rotation_constraint.h"
#include "drake/solvers/solve.h"

using drake::symbolic::Expression;
namespace drake {
namespace solvers {
namespace {
enum RotationMatrixIntervalBinning {
  kLinear,        ///< Same as IntervalBining::kLinear, used by
                  /// MixedIntegerRotationMatrixGenerator.
  kLogarithmic,   ///< Same as IntervalBinning::kLogarithmic, used by
                  /// MixedIntegerRotationMatrixGenerator.
  kPosNegLinear,  ///< Used by AddRotationMatrixBoxSphereIntersection. It uses
                  /// linear binning for positive and negative axis
                  /// respectively.
};

// Test some corner cases of box-sphere intersection.
// The corner cases happens when either the innermost or the outermost corner
// of the box bmin <= x <= bmax lies on the surface of the unit sphere.
class TestBoxSphereCorner
    : public ::testing::TestWithParam<
          std::tuple<int, bool, int, RotationMatrixIntervalBinning>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestBoxSphereCorner)

  TestBoxSphereCorner()
      : prog_(),
        R_(NewRotationMatrixVars(&prog_)),
        Cpos_(),
        Cneg_(),
        orthant_(std::get<0>(GetParam())),
        is_bmin_(std::get<1>(GetParam())),
        col_idx_(std::get<2>(GetParam())),
        r_interval_binning_(std::get<3>(GetParam())) {
    DRAKE_DEMAND(orthant_ >= 0);
    DRAKE_DEMAND(orthant_ <= 7);
    const int N = 3;  // num_interval_per_half_axis = 3
    if (r_interval_binning_ == RotationMatrixIntervalBinning::kLinear ||
        r_interval_binning_ == RotationMatrixIntervalBinning::kLogarithmic) {
      const IntervalBinning interval_binning =
          r_interval_binning_ == RotationMatrixIntervalBinning::kLinear
              ? IntervalBinning::kLinear
              : IntervalBinning::kLogarithmic;
      const MixedIntegerRotationConstraintGenerator rotation_generator(
          MixedIntegerRotationConstraintGenerator::Approach::
              kBoxSphereIntersection,
          N, interval_binning);
      const auto ret = rotation_generator.AddToProgram(R_, &prog_);
      Cpos_.resize(N);
      Cneg_.resize(N);
      const auto gray_codes = math::CalculateReflectedGrayCodes<3>();

      for (int k = 0; k < N; ++k) {
        if (interval_binning == IntervalBinning::kLogarithmic) {
          Cpos_[k] = prog_.NewContinuousVariables<3, 3>().cast<Expression>();
          Cneg_[k] = prog_.NewContinuousVariables<3, 3>().cast<Expression>();
        }
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            if (interval_binning == IntervalBinning::kLinear) {
              Cpos_[k](i, j) = ret.B_[i][j](N + k);
              Cneg_[k](i, j) = ret.B_[i][j](N - k - 1);
            } else {
              // logarithmic binning.
              prog_.AddConstraint(CreateBinaryCodeMatchConstraint(
                  ret.B_[i][j].cast<Expression>(),
                  gray_codes.row(N + k).transpose(), Cpos_[k](i, j)));
              prog_.AddConstraint(CreateBinaryCodeMatchConstraint(
                  ret.B_[i][j].cast<Expression>(),
                  gray_codes.row(N - k - 1).transpose(), Cneg_[k](i, j)));
            }
          }
        }
      }
    } else {
      const auto ret =
          AddRotationMatrixBoxSphereIntersectionMilpConstraints(R_, N, &prog_);
      Cpos_ = ret.CRpos;
      Cneg_ = ret.CRneg;
    }
  }

  ~TestBoxSphereCorner() override {}

 protected:
  MathematicalProgram prog_;
  MatrixDecisionVariable<3, 3> R_;
  std::vector<Eigen::Matrix<Expression, 3, 3>> Cpos_;
  std::vector<Eigen::Matrix<Expression, 3, 3>> Cneg_;
  int orthant_;   // Index of the orthant that R_.col(col_idx_) is in.
  bool is_bmin_;  // If true, then the box bmin <= x <= bmax intersects with the
                  // surface of the unit sphere at the unique point bmin;
                  // otherwise it intersects at the unique point bmax;
  int col_idx_;   // R_.col(col_idx_) will be fixed to a vertex of the box, and
                  // also this point is on the surface of the unit sphere.
  RotationMatrixIntervalBinning r_interval_binning_;
};

TEST_P(TestBoxSphereCorner, TestOrthogonal) {
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

  const auto result = Solve(prog_);
  EXPECT_TRUE(result.is_success());
  const auto R_val = result.GetSolution(R_);
  std::vector<Eigen::Matrix3d> Bpos_val(3);
  std::vector<Eigen::Matrix3d> Bneg_val(3);
  EXPECT_NEAR(R_val.col(free_axis0).dot(box_pt), 0, 1E-4);
  EXPECT_NEAR(R_val.col(free_axis1).dot(box_pt), 0, 1E-4);
  EXPECT_TRUE(CompareMatrices(box_pt.cross(R_val.col(free_axis0)),
                              R_val.col(free_axis1), 1E-4,
                              MatrixCompareType::absolute));
}

// It takes too long time to run the test under debug mode.
#ifdef DRAKE_ASSERT_IS_ARMED
INSTANTIATE_TEST_SUITE_P(
    RotationTest, TestBoxSphereCorner,
    ::testing::Combine(
        ::testing::ValuesIn<std::vector<int>>({0}),      // Orthant
        ::testing::ValuesIn<std::vector<bool>>({true}),  // bmin or bmax
        ::testing::ValuesIn<std::vector<int>>({0}),      // column index
        ::testing::ValuesIn<std::vector<RotationMatrixIntervalBinning>>(
            {RotationMatrixIntervalBinning::kPosNegLinear})));
#else
INSTANTIATE_TEST_SUITE_P(
    RotationTest, TestBoxSphereCorner,
    ::testing::Combine(
        ::testing::ValuesIn<std::vector<int>>({0, 1, 2, 3, 4, 5, 6,
                                               7}),             // Orthant
        ::testing::ValuesIn<std::vector<bool>>({true, false}),  // bmin or bmax
        ::testing::ValuesIn<std::vector<int>>({0, 1, 2}),       // column index
        ::testing::ValuesIn<std::vector<RotationMatrixIntervalBinning>>(
            {RotationMatrixIntervalBinning::kLinear,
             RotationMatrixIntervalBinning::kLogarithmic,
             RotationMatrixIntervalBinning::kPosNegLinear})));
#endif
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
