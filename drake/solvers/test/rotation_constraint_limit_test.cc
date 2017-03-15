#include "drake/solvers/rotation_constraint.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
// Compute the minimum distance of R.col(0) and R.col(1), if R satisfies
// the McCormick envelope constraint. This test records how well we can
// approximate the rotation matrix on SO(3). If in the future we improved
// our relaxation and get a larger minimal distance, please update this test.
static Eigen::Matrix3d R_test;
class TestMinimumDistance : public testing::TestWithParam<int> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestMinimumDistance)

  TestMinimumDistance()
      : prog_(),
        R_(NewRotationMatrixVars(&prog_)),
        d_(prog_.NewContinuousVariables<1>("d")),
        Bpos_(GetParam()),
        Bneg_(GetParam()),
        num_binary_vars_per_half_axis_(GetParam()),
        minimal_distance_expected_(0) {
    const auto p = AddRotationMatrixMcCormickEnvelopeMilpConstraints(
        &prog_,
        R_,
        num_binary_vars_per_half_axis_);
    Bpos_ = p.first;
    Bneg_ = p.second;
    // Add the constraint that d_ >= |R_.col(0) - R_.col(1)|
    Vector4<symbolic::Expression> s;
    s << d_(0), R_.col(0) - R_.col(1);
    prog_.AddLorentzConeConstraint(s);

    // Miminize the distance.
    prog_.AddCost(d_(0));

    if (num_binary_vars_per_half_axis_ == 2) {
      for(int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          if (i == 2 && j == 0) {
            prog_.AddBoundingBoxConstraint(0.6, 0.7, R_(2, 0));
          } else if (i == 0 && j == 2) {
            prog_.AddBoundingBoxConstraint(0.6, 0.7, R_(0, 2));
          } else if( i == 2 && j == 1) {
            prog_.AddBoundingBoxConstraint(0.6, 0.7, R_(2, 1));
          } else if (i == 1 && j == 2) {
            prog_.AddBoundingBoxConstraint(0.6, 0.7, R_(1, 2));
          }
          else {
            prog_.AddBoundingBoxConstraint(R_test(i, j), R_test(i, j), R_(i, j));
          }
        }

      }

    }
  }

  ~TestMinimumDistance() override {}

  void SetMinimumDistanceExpected() { DoSetMinimumDistanceExpected(); }

  void SolveAndCheckSolution() {
    GurobiSolver gurobi_solver;
    if (gurobi_solver.available()) {
      prog_.SetSolverOption(SolverType::kGurobi, "OutputFlag", true);
      SolutionResult sol_result = gurobi_solver.Solve(prog_);

      EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);
      double d_val = prog_.GetSolution(d_(0));
      EXPECT_NEAR(d_val, minimal_distance_expected_, 1E-2);
      std::cout << " R:\n" << prog_.GetSolution(R_) << std::endl;
      for (int i = 0; i < num_binary_vars_per_half_axis_; ++i) {
        std::cout << "Bpos[" << i << "]:\n" << prog_.GetSolution(Bpos_[i]) << std::endl;
        std::cout << "Bneg[" << i << "]:\n" << prog_.GetSolution(Bneg_[i]) << std::endl;
      }
      if (num_binary_vars_per_half_axis_ == 3) {
        R_test = prog_.GetSolution(R_);
      }
    }
  }

 protected:
  MathematicalProgram prog_;
  MatrixDecisionVariable<3, 3> R_;
  VectorDecisionVariable<1> d_;
  std::vector<MatrixDecisionVariable<3, 3>> Bpos_;
  std::vector<MatrixDecisionVariable<3, 3>> Bneg_;
  int num_binary_vars_per_half_axis_;
  double minimal_distance_expected_;

 private:
  virtual void DoSetMinimumDistanceExpected() {
    // Update the expected minimal distance, when we improve the relaxation on
    // SO(3).
    switch (num_binary_vars_per_half_axis_) {
      case 1 : {
        // TODO(hongkai.dai): The minimum distance should not be zero. Add some
        // constraint to prevent this, such as no two columns/rows can be in the
        // same bin.
        minimal_distance_expected_ = 0;
        break;
      }
      case 2 : {
        minimal_distance_expected_ = 0.974;
        break;
      }
      case 3 : {
        // TODO(hongkai.dai): Figure out why 3 binary variables give closest
        // distance smaller than 2 binary variables case.
        minimal_distance_expected_ = 0.38;
        break;
      }
      default : {
        throw std::runtime_error(
            "Have not attempted this number of binary variables yet.");
      }
    }
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
    switch (num_binary_vars_per_half_axis_) {
      case 1 : {
        minimal_distance_expected_ = 0;
        break;
      }
      case 2 : {
        minimal_distance_expected_ = 1.1928;
        break;
      }
      case 3 : {
        minimal_distance_expected_ = 1.3056;
        break;
      }
      default : {
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

INSTANTIATE_TEST_CASE_P(RotationTest, TestMinimumDistance,
    ::testing::ValuesIn({3, 2})
);
/*
INSTANTIATE_TEST_CASE_P(RotationTest, TestMinimumDistanceWOrthonormalSocp,
    ::testing::ValuesIn({1, 2, 3})
);*/
}  // namespace solvers
}  // namespace drake
