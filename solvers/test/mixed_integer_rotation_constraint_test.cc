#include "drake/solvers/mixed_integer_rotation_constraint.h"

#include <random>

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rotation_matrix.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/rotation_constraint.h"

using Eigen::Vector3d;
using Eigen::Matrix3d;

using drake::symbolic::Expression;

using std::sqrt;

namespace drake {
namespace solvers {
namespace {
bool IsFeasibleCheck(
    MathematicalProgram* prog,
    const std::shared_ptr<LinearEqualityConstraint>& feasibility_constraint,
    const Eigen::Ref<const Matrix3d>& R_sample) {
  Eigen::Map<const Eigen::Matrix<double, 9, 1>> R_sample_vec(R_sample.data());
  feasibility_constraint->UpdateLowerBound(R_sample_vec);
  feasibility_constraint->UpdateUpperBound(R_sample_vec);

  return (prog->Solve() == kSolutionFound);
}

class TestMixedIntegerRotationConstraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestMixedIntegerRotationConstraint)

  TestMixedIntegerRotationConstraint(
      MixedIntegerRotationConstraintType constraint_type,
      int num_intervals_per_half_axis, IntervalBinning interval_binning)
      : prog_(),
        R_(NewRotationMatrixVars(&prog_)),
        constraint_type_(constraint_type),
        num_intervals_per_half_axis_(num_intervals_per_half_axis),
        interval_binning_(interval_binning),
        feasibility_constraint_{prog_
                                    .AddLinearEqualityConstraint(
                                        Eigen::Matrix<double, 9, 9>::Identity(),
                                        Eigen::Matrix<double, 9, 1>::Zero(),
                                        {R_.col(0), R_.col(1), R_.col(2)})
                                    .evaluator()} {}

  bool IsFeasible(const Eigen::Ref<const Eigen::Matrix3d>& R_to_check) {
    return IsFeasibleCheck(&prog_, feasibility_constraint_, R_to_check);
  }

  virtual ~TestMixedIntegerRotationConstraint() = default;

  void TestExactRotationMatrix() {
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
      R_test = math::UniformlyRandomRotationMatrix(&generator).matrix();
      EXPECT_TRUE(IsFeasible(R_test));
    }
  }

  void TestInexactRotationMatrix() {
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
    if (num_intervals_per_half_axis_ == 1 &&
        constraint_type_ ==
            MixedIntegerRotationConstraintType::kBoxSphereIntersection)
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

    // Checks a few cases just outside the L1 ball. If we use the formulation
    // that
    // replaces the bilinear term with another variable in the McCormick
    // envelope,
    // then it should always be infeasible. Otherwise should be feasible for
    // num_intervals_per_half_axis_=1, but infeasible for
    // num_intervals_per_half_axis_>1.
    R_test = math::YRotation(M_PI_4);
    R_test(2, 0) -= 0.1;
    EXPECT_GT(R_test.col(0).lpNorm<1>(), 1.0);
    EXPECT_GT(R_test.row(2).lpNorm<1>(), 1.0);
    if (num_intervals_per_half_axis_ == 1 &&
        constraint_type_ ==
            MixedIntegerRotationConstraintType::kBoxSphereIntersection)
      EXPECT_TRUE(IsFeasible(R_test));
    else
      EXPECT_FALSE(IsFeasible(R_test));
  }

 protected:
  MathematicalProgram prog_;
  MatrixDecisionVariable<3, 3> R_;
  MixedIntegerRotationConstraintType constraint_type_;
  int num_intervals_per_half_axis_;
  IntervalBinning interval_binning_;
  std::shared_ptr<LinearEqualityConstraint> feasibility_constraint_;
};

class TestMixedIntegerRotationConstraintBilinearMcCormick
    : public TestMixedIntegerRotationConstraint,
      public ::testing::TestWithParam<std::tuple<int, IntervalBinning>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(
      TestMixedIntegerRotationConstraintBilinearMcCormick)

  TestMixedIntegerRotationConstraintBilinearMcCormick()
      : TestMixedIntegerRotationConstraint(
            MixedIntegerRotationConstraintType::kBilinearMcCormick,
            std::get<0>(GetParam()), std::get<1>(GetParam())),
        rotation_generator_(num_intervals_per_half_axis_, interval_binning_),
        B_{rotation_generator_.AddToProgram(&prog_, R_)} {}

  ~TestMixedIntegerRotationConstraintBilinearMcCormick() = default;

 protected:
  MixedIntegerRotationConstraintGenerator<
      MixedIntegerRotationConstraintType::kBilinearMcCormick>
      rotation_generator_;
  std::array<std::array<VectorXDecisionVariable, 3>, 3> B_;
};

TEST_P(TestMixedIntegerRotationConstraintBilinearMcCormick, TestConstructor) {
  EXPECT_TRUE(CompareMatrices(
      rotation_generator_.phi(),
      Eigen::VectorXd::LinSpaced(2 * num_intervals_per_half_axis_ + 1, -1, 1),
      1E-12));
  EXPECT_TRUE(CompareMatrices(
      rotation_generator_.phi_nonnegative(),
      Eigen::VectorXd::LinSpaced(num_intervals_per_half_axis_ + 1, 0, 1),
      1E-12));
}

TEST_P(TestMixedIntegerRotationConstraintBilinearMcCormick,
       TestBinaryAssignment) {
  const Eigen::Matrix3d R_test =
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto b_constraint = prog_.AddBoundingBoxConstraint(0, 0, B_[0][0]);
  auto UpdateBConstraint =
      [&b_constraint](const Eigen::Ref<const Eigen::VectorXd>& b_val) {
        b_constraint.evaluator()->UpdateLowerBound(b_val);
        b_constraint.evaluator()->UpdateUpperBound(b_val);
      };
  switch (interval_binning_) {
    case IntervalBinning::kLinear: {
      switch (num_intervals_per_half_axis_) {
        case 1:
          UpdateBConstraint(Eigen::Vector2d(0, 1));
          EXPECT_TRUE(IsFeasible(R_test));
          UpdateBConstraint(Eigen::Vector2d(1, 0));
          EXPECT_FALSE(IsFeasible(R_test));
          break;
        case 2:
          UpdateBConstraint(Eigen::Vector4d(0, 0, 0, 1));
          EXPECT_TRUE(IsFeasible(R_test));
          UpdateBConstraint(Eigen::Vector4d(0, 0, 1, 0));
          EXPECT_FALSE(IsFeasible(R_test));
          break;
        default:
          throw std::runtime_error("Unsuppored num_intervals_per_half_axis_.");
      }
      break;
    }
    case IntervalBinning::kLogarithmic: {
      switch (num_intervals_per_half_axis_) {
        case 1: {
          UpdateBConstraint(Vector1d(1));
          EXPECT_TRUE(IsFeasible(R_test));
          UpdateBConstraint(Vector1d(0));
          EXPECT_FALSE(IsFeasible(R_test));
          break;
        }
        case 2: {
          UpdateBConstraint(Eigen::Vector2d(1, 0));
          EXPECT_TRUE(IsFeasible(R_test));
          UpdateBConstraint(Eigen::Vector2d(0, 1));
          EXPECT_FALSE(IsFeasible(R_test));
          break;
        }
        default:
          throw std::runtime_error("Unsupported num_intervals_per_half_axis_.");
      }
      break;
    }
  }
}

TEST_P(TestMixedIntegerRotationConstraintBilinearMcCormick,
       ExactRotationMatrix) {
  TestExactRotationMatrix();
}

TEST_P(TestMixedIntegerRotationConstraintBilinearMcCormick,
       InexactRotationMatrix) {
  TestInexactRotationMatrix();
}

INSTANTIATE_TEST_CASE_P(
    RotationTest, TestMixedIntegerRotationConstraintBilinearMcCormick,
    ::testing::Combine(::testing::ValuesIn<std::vector<int>>({1, 2}),
                       ::testing::ValuesIn<std::vector<IntervalBinning>>(
                           {IntervalBinning::kLinear,
                            IntervalBinning::kLogarithmic})));
/*
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
  int orthant_;   // Index of the orthant that R_.col(col_idx_) is in.
  bool is_bmin_;  // If true, then the box bmin <= x <= bmax intersects with the
                  // surface of the unit sphere at the unique point bmin;
                  // otherwise it intersects at the unique point bmax;
  int col_idx_;   // R_.col(col_idx_) will be fixed to a vertex of the box, and
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

  TestMcCormickOrthant() : prog_(), R_(NewRotationMatrixVars(&prog_)) {
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
                           {false, true})));  // replace bilinear or not.*/
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
