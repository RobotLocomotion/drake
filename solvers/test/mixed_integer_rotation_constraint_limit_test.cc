/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mixed_integer_rotation_constraint.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/rotation_constraint.h"

namespace drake {
namespace solvers {
// The goal of this class is to measure how well we can approximate the
// constraint on SO(3). To do so, we choose to compute the closest distance
// between R.col(0) and R.col(1), where `R` satisfies our relaxation.
// If `R` satisfies the SO(3) constraint exactly, then the closest distance
// is sqrt(2). Due to the relaxation, we should see the closest distance
// being smaller than sqrt(2).
// This test records how well we can approximate the rotation matrix on SO(3).
// If in the future we improved our relaxation and get a larger minimal
// distance, please update this test.
class TestMinimumDistance
    : public testing::TestWithParam<
          std::tuple<MixedIntegerRotationConstraintGenerator::Approach, int>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestMinimumDistance)

  TestMinimumDistance()
      : prog_(),
        R_(NewRotationMatrixVars(&prog_)),
        d_(prog_.NewContinuousVariables<1>("d")),
        approach_(std::get<0>(GetParam())),
        num_intervals_per_half_axis_(std::get<1>(GetParam())),
        minimal_distance_expected_(0) {
    MixedIntegerRotationConstraintGenerator rotation_generator(
        approach_, num_intervals_per_half_axis_, IntervalBinning::kLogarithmic);
    rotation_generator.AddToProgram(R_, &prog_);

    // Add the constraint that d_ >= |R_.col(0) - R_.col(1)|
    Vector4<symbolic::Expression> s;
    s << d_(0), R_.col(0) - R_.col(1);
    prog_.AddLorentzConeConstraint(s);

    // Miminize the distance.
    prog_.AddCost(d_(0));
  }

  ~TestMinimumDistance() override {}

  void SetMinimumDistanceExpected() { DoSetMinimumDistanceExpected(); }

  void SolveAndCheckSolution() {
    GurobiSolver gurobi_solver;
    if (gurobi_solver.available()) {
      prog_.SetSolverOption(GurobiSolver::id(), "OutputFlag", true);
      auto result = gurobi_solver.Solve(prog_, {}, {});
      EXPECT_TRUE(result.is_success());
      double d_val = result.GetSolution(d_(0));
      EXPECT_NEAR(d_val, minimal_distance_expected_, 1E-2);
    }
  }

 protected:
  MathematicalProgram prog_;
  MatrixDecisionVariable<3, 3> R_;
  VectorDecisionVariable<1> d_;
  MixedIntegerRotationConstraintGenerator::Approach approach_;
  int num_intervals_per_half_axis_;
  double minimal_distance_expected_;

 private:
  virtual void DoSetMinimumDistanceExpected() {
    // Update the expected minimal distance, when we improve the relaxation on
    // SO(3).
    std::array<double, 3> min_distance;  // Record the global minimal for
                                         // different number of intervals per
                                         // half axis {1, 2, 3}.
    switch (approach_) {
      case MixedIntegerRotationConstraintGenerator::Approach::
          kBoxSphereIntersection: {
        min_distance = {{0.069166, 0.974, 1.0823199}};
        break;
      }
      case MixedIntegerRotationConstraintGenerator::Approach::
          kBilinearMcCormick: {
        min_distance = {{0.60229, 1.22474, 1.32667}};
        break;
      }
      case MixedIntegerRotationConstraintGenerator::Approach::kBoth: {
        min_distance = {{0.60302, 1.25649, 1.33283}};
        break;
      }
    }
    minimal_distance_expected_ = min_distance[num_intervals_per_half_axis_ - 1];
  }
};

class TestMinimumDistanceWOrthonormalSocp : public TestMinimumDistance {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestMinimumDistanceWOrthonormalSocp)

  TestMinimumDistanceWOrthonormalSocp() : TestMinimumDistance() {
    AddRotationMatrixOrthonormalSocpConstraint(&prog_, R_);
  }

  ~TestMinimumDistanceWOrthonormalSocp() override {}

 private:
  void DoSetMinimumDistanceExpected() override {
    // Update the expected minimal distance, when we improve the relaxation on
    // SO(3).
    switch (num_intervals_per_half_axis_) {
      case 1: {
        minimal_distance_expected_ = 0.06916;
        break;
      }
      case 2: {
        minimal_distance_expected_ = 1.19452;
        break;
      }
      case 3: {
        minimal_distance_expected_ = 1.3056;
        break;
      }
      default: {
        throw std::runtime_error(
            "Have not attempted this number of binary variables yet.");
      }
    }
  }
};

TEST_P(TestMinimumDistance, Test) {
  SetMinimumDistanceExpected();
  SolveAndCheckSolution();
}

TEST_P(TestMinimumDistanceWOrthonormalSocp, Test) {
  SetMinimumDistanceExpected();
  SolveAndCheckSolution();
}

INSTANTIATE_TEST_SUITE_P(
    RotationTest, TestMinimumDistance,
    ::testing::Combine(
        ::testing::ValuesIn<
            std::vector<MixedIntegerRotationConstraintGenerator::Approach>>(
            {MixedIntegerRotationConstraintGenerator::Approach::
                 kBoxSphereIntersection,
             MixedIntegerRotationConstraintGenerator::Approach::
                 kBilinearMcCormick}),
        ::testing::ValuesIn<std::vector<int>>(
            {1, 2, 3})));  // number of binary variables per half axis

INSTANTIATE_TEST_SUITE_P(
    RotationTest, TestMinimumDistanceWOrthonormalSocp,
    ::testing::Combine(
        ::testing::ValuesIn<
            std::vector<MixedIntegerRotationConstraintGenerator::Approach>>(
            {MixedIntegerRotationConstraintGenerator::Approach::
                 kBoxSphereIntersection}),
        ::testing::ValuesIn<std::vector<int>>(
            {1, 2, 3})));  // number of binary variables per half axis
}  // namespace solvers
}  // namespace drake
