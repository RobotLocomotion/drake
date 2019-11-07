#include "drake/solvers/mixed_integer_rotation_constraint.h"

#include <random>

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/gray_code.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rotation_matrix.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/integer_optimization_util.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/rotation_constraint.h"
#include "drake/solvers/solve.h"

using Eigen::Vector3d;
using Eigen::Matrix3d;

using drake::math::RotationMatrixd;
using drake::symbolic::Expression;

using std::sqrt;

namespace drake {
namespace solvers {
namespace {
bool IsFeasibleCheck(
    const MathematicalProgram& prog,
    const std::shared_ptr<LinearEqualityConstraint>& feasibility_constraint,
    const Eigen::Ref<const Matrix3d>& R_sample) {
  Eigen::Map<const Eigen::Matrix<double, 9, 1>> R_sample_vec(R_sample.data());
  feasibility_constraint->UpdateLowerBound(R_sample_vec);
  feasibility_constraint->UpdateUpperBound(R_sample_vec);

  return Solve(prog).is_success();
}

class TestMixedIntegerRotationConstraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestMixedIntegerRotationConstraint)

  TestMixedIntegerRotationConstraint(
      MixedIntegerRotationConstraintGenerator::Approach approach,
      int num_intervals_per_half_axis)
      : prog_(),
        R_(NewRotationMatrixVars(&prog_)),
        approach_(approach),
        num_intervals_per_half_axis_(num_intervals_per_half_axis),
        feasibility_constraint_{prog_
                                    .AddLinearEqualityConstraint(
                                        Eigen::Matrix<double, 9, 9>::Identity(),
                                        Eigen::Matrix<double, 9, 1>::Zero(),
                                        {R_.col(0), R_.col(1), R_.col(2)})
                                    .evaluator()} {}

  bool IsFeasible(const Eigen::Ref<const Eigen::Matrix3d>& R_to_check) {
    return IsFeasibleCheck(prog_, feasibility_constraint_, R_to_check);
  }

  bool IsFeasible(const RotationMatrixd& R_to_check) {
    return IsFeasible(R_to_check.matrix());
  }

  void TestExactRotationMatrix() {
    // If R is exactly on SO(3), test whether it also satisfies our relaxation.

    // Test a few valid rotation matrices.
    RotationMatrixd R_test;  // Identity matrix.
    EXPECT_TRUE(IsFeasible(R_test));

    R_test = RotationMatrixd::MakeZRotation(M_PI_4) * R_test;
    EXPECT_TRUE(IsFeasible(R_test));

    R_test = RotationMatrixd::MakeYRotation(M_PI_4) * R_test;
    EXPECT_TRUE(IsFeasible(R_test));

    R_test = RotationMatrixd::MakeZRotation(M_PI_2);
    EXPECT_TRUE(IsFeasible(R_test));

    R_test = RotationMatrixd::MakeZRotation(-M_PI_2);
    EXPECT_TRUE(IsFeasible(R_test));

    R_test = RotationMatrixd::MakeYRotation(M_PI_2);
    EXPECT_TRUE(IsFeasible(R_test));

    R_test = RotationMatrixd::MakeYRotation(-M_PI_2);
    EXPECT_TRUE(IsFeasible(R_test));

    // This one caught a bug (in the loop finding the most conservative linear
    // constraint for a given region) during random testing.
    Matrix3d R_check;
    R_check << 0.17082017792981191, 0.65144498431260445, -0.73921573253413542,
              -0.82327804434149443, -0.31781600529013027, -0.47032568342231595,
              -0.54132589862048197, 0.68892119955432829, 0.48203096610835455;
    EXPECT_TRUE(IsFeasible(R_check));

    std::mt19937 generator(41);
    for (int i = 0; i < 40; i++) {
      R_test = math::UniformlyRandomRotationMatrix(&generator);
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
        approach_ == MixedIntegerRotationConstraintGenerator::Approach::
                         kBoxSphereIntersection)
      EXPECT_TRUE(IsFeasible(R_test));
    else
      EXPECT_FALSE(IsFeasible(R_test));

    // Checks the det(R)=-1 case.
    // (only ruled out by the cross-product constraint).
    R_test = Matrix3d::Identity();
    R_test(2, 2) = -1;
    EXPECT_FALSE(IsFeasible(R_test));

    R_test = RotationMatrixd::MakeZRotation(M_PI_4).matrix() * R_test;
    EXPECT_FALSE(IsFeasible(R_test));

    R_test = RotationMatrixd::MakeYRotation(M_PI_4).matrix() * R_test;
    EXPECT_FALSE(IsFeasible(R_test));

    // Checks a few cases just outside the L1 ball. If we use the formulation
    // that replaces the bilinear term with another variable in the McCormick
    // envelope, then it should always be infeasible. Otherwise should be
    // feasible for num_intervals_per_half_axis_=1, but infeasible for
    // num_intervals_per_half_axis_>1.
    R_test = RotationMatrixd::MakeYRotation(M_PI_4).matrix();
    R_test(2, 0) -= 0.1;
    EXPECT_GT(R_test.col(0).lpNorm<1>(), 1.0);
    EXPECT_GT(R_test.row(2).lpNorm<1>(), 1.0);
    if (num_intervals_per_half_axis_ == 1 &&
        approach_ == MixedIntegerRotationConstraintGenerator::Approach::
                         kBoxSphereIntersection)
      EXPECT_TRUE(IsFeasible(R_test));
    else
      EXPECT_FALSE(IsFeasible(R_test));
  }
  virtual ~TestMixedIntegerRotationConstraint() = default;

 protected:
  MathematicalProgram prog_;
  MatrixDecisionVariable<3, 3> R_;
  MixedIntegerRotationConstraintGenerator::Approach approach_;
  int num_intervals_per_half_axis_;
  std::shared_ptr<LinearEqualityConstraint> feasibility_constraint_;
};

class TestMixedIntegerRotationConstraintGenerator
    : public TestMixedIntegerRotationConstraint,
      public ::testing::TestWithParam<
          std::tuple<MixedIntegerRotationConstraintGenerator::Approach, int,
                     IntervalBinning>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestMixedIntegerRotationConstraintGenerator)

  TestMixedIntegerRotationConstraintGenerator()
      : TestMixedIntegerRotationConstraint(std::get<0>(GetParam()),
                                           std::get<1>(GetParam())),
        interval_binning_(std::get<2>(GetParam())),
        rotation_generator_(approach_, num_intervals_per_half_axis_,
                            interval_binning_),
        ret(rotation_generator_.AddToProgram(R_, &prog_)) {}

  ~TestMixedIntegerRotationConstraintGenerator() = default;

 protected:
  IntervalBinning interval_binning_;
  MixedIntegerRotationConstraintGenerator rotation_generator_;
  MixedIntegerRotationConstraintGenerator::ReturnType ret;
};

TEST_P(TestMixedIntegerRotationConstraintGenerator, TestConstructor) {
  EXPECT_TRUE(CompareMatrices(
      rotation_generator_.phi(),
      Eigen::VectorXd::LinSpaced(2 * num_intervals_per_half_axis_ + 1, -1, 1),
      1E-12));
  EXPECT_TRUE(CompareMatrices(
      rotation_generator_.phi_nonnegative(),
      Eigen::VectorXd::LinSpaced(num_intervals_per_half_axis_ + 1, 0, 1),
      1E-12));
}

TEST_P(TestMixedIntegerRotationConstraintGenerator, TestBinaryAssignment) {
  const RotationMatrixd R_test = RotationMatrixd::MakeZRotation(0.1);
  auto b_constraint = prog_.AddBoundingBoxConstraint(0, 0, ret.B_[0][0]);
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
          GTEST_FAIL() << "Unsuppored num_intervals_per_half_axis_.";
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

TEST_P(TestMixedIntegerRotationConstraintGenerator, ExactRotationMatrix) {
  TestExactRotationMatrix();
}

TEST_P(TestMixedIntegerRotationConstraintGenerator, InexactRotationMatrix) {
  TestInexactRotationMatrix();
}

INSTANTIATE_TEST_SUITE_P(
    RotationTest, TestMixedIntegerRotationConstraintGenerator,
    ::testing::Combine(
        ::testing::ValuesIn<
            std::vector<MixedIntegerRotationConstraintGenerator::Approach>>(
            {MixedIntegerRotationConstraintGenerator::Approach::
                 kBilinearMcCormick,
             MixedIntegerRotationConstraintGenerator::Approach::
                 kBoxSphereIntersection,
             MixedIntegerRotationConstraintGenerator::Approach::kBoth}),
        ::testing::ValuesIn<std::vector<int>>({1, 2}),
        ::testing::ValuesIn<std::vector<IntervalBinning>>(
            {IntervalBinning::kLinear, IntervalBinning::kLogarithmic})));

class TestRotationMatrixBoxSphereIntersection
    : public TestMixedIntegerRotationConstraint,
      public ::testing::TestWithParam<int> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestRotationMatrixBoxSphereIntersection)

  TestRotationMatrixBoxSphereIntersection()
      : TestMixedIntegerRotationConstraint(
            MixedIntegerRotationConstraintGenerator::Approach::
                kBoxSphereIntersection,
            GetParam()) {
    AddRotationMatrixBoxSphereIntersectionMilpConstraints(
        R_, num_intervals_per_half_axis_, &prog_);
  }

  ~TestRotationMatrixBoxSphereIntersection() override {}
};

TEST_P(TestRotationMatrixBoxSphereIntersection, ExactRotationMatrix) {
  TestExactRotationMatrix();
}

TEST_P(TestRotationMatrixBoxSphereIntersection, InexactRotationMatrix) {
  TestInexactRotationMatrix();
}

INSTANTIATE_TEST_SUITE_P(RotationTest, TestRotationMatrixBoxSphereIntersection,
                        ::testing::ValuesIn<std::vector<int>>({1, 2}));

// Make sure that no two row or column vectors in R, which satisfies the
// mixed-integer relaxation, can lie in either the same or the opposite orthant.
class TestOrthant
    : public ::testing::TestWithParam<
          std::tuple<int, int, bool, std::pair<int, int>, bool,
                     MixedIntegerRotationConstraintGenerator::Approach>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestOrthant)

  TestOrthant() : prog_(), R_(NewRotationMatrixVars(&prog_)) {
    const int num_bin = std::get<0>(GetParam());
    const int orthant = std::get<1>(GetParam());
    const bool is_row_vector = std::get<2>(GetParam());
    const int idx0 = std::get<3>(GetParam()).first;
    const int idx1 = std::get<3>(GetParam()).second;
    const bool is_same_orthant = std::get<4>(GetParam());
    const auto approach = std::get<5>(GetParam());
    DRAKE_DEMAND(idx0 != idx1);
    DRAKE_DEMAND(idx0 >= 0);
    DRAKE_DEMAND(idx1 >= 0);
    DRAKE_DEMAND(idx0 <= 2);
    DRAKE_DEMAND(idx1 <= 2);

    MixedIntegerRotationConstraintGenerator rotation_generator(
        approach, num_bin, IntervalBinning::kLinear);
    rotation_generator.AddToProgram(R_, &prog_);

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

  ~TestOrthant() override {}

 protected:
  MathematicalProgram prog_;
  MatrixDecisionVariable<3, 3> R_;
};

TEST_P(TestOrthant, test) {
  GurobiSolver gurobi_solver;
  if (gurobi_solver.available()) {
    MathematicalProgramResult result;
    gurobi_solver.Solve(prog_, {}, {}, &result);
    SolutionResult sol_result = result.get_solution_result();
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

INSTANTIATE_TEST_SUITE_P(
    RotationTest, TestOrthant,
    ::testing::Combine(
        ::testing::ValuesIn<std::vector<int>>(
            {1}),  // # of intervals per half axis
        ::testing::ValuesIn<std::vector<int>>({0, 1, 2, 3, 4, 5, 6,
                                               7}),  // orthant index
        ::testing::ValuesIn<std::vector<bool>>(
            {false, true}),  // row vector or column vector
        ::testing::ValuesIn<std::array<std::pair<int, int>, 3>>(
            vector_indices()),  // vector indices
        ::testing::ValuesIn<std::vector<bool>>(
            {false, true}),  // same or opposite orthant
        ::testing::ValuesIn<
            std::vector<MixedIntegerRotationConstraintGenerator::Approach>>(
            {MixedIntegerRotationConstraintGenerator::Approach::
                 kBoxSphereIntersection,
             MixedIntegerRotationConstraintGenerator::Approach::
                 kBoxSphereIntersection})));  // box-sphere intersection or
                                              // bilinear McCormick.
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
