#include "drake/multibody/fem/dirichlet_boundary_condition.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/fem/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {
namespace {

/* An arbitrary number of degree of freedom made up for testing purpose. */
static constexpr int kNumDofs = 6;
using DenseMatrix = Eigen::Matrix<double, kNumDofs, kNumDofs>;
using contact_solvers::internal::BlockSparseSymmetricMatrix3d;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using std::make_unique;
using std::unique_ptr;

// clang-format off
const Matrix3d A00 = (Eigen::Matrix3d() <<  3,   1,    0.5,
                                            1,   2,    0.25,
                                            0.5, 0.25, 3).finished();
const Matrix3d A11 = (Eigen::Matrix3d() <<  0.3,  0.1,    0.05,
                                            0.1,  0.2,    0.025,
                                            0.05, 0.025,  0.3).finished();
// clang-format on

class DirichletBoundaryConditionTest : public ::testing::Test {
 protected:
  void SetUp() {
    const Vector3<double> bc_q(1, 2, 3);
    const Vector3<double> bc_v(4, 5, 6);
    const Vector3<double> bc_a(7, 8, 9);
    bc_.AddBoundaryCondition(FemNodeIndex(0), {bc_q, bc_v, bc_a});
    /* Makes an arbitrary compatible model state. */
    Vector6<double> q, v, a;
    q << 0.1, 0.2, 0.3, 1.1, 1.2, 1.3;
    v << 0.5, 0.6, 0.7, 0.8, 0.9, 1.0;
    a << 0.9, 1.0, 1.1, 1.2, 1.4, 1.6;
    fem_state_system_ = make_unique<FemStateSystem<double>>(q, v, a);
    fem_state_ = make_unique<FemState<double>>(fem_state_system_.get());
  }

  /* Makes an arbitrary residual vector with appropriate size. */
  static VectorXd MakeResidual() {
    Vector<double, kNumDofs> residual;
    residual << 1.1, 2.2, 3.3, 4.4, 5.5, 6.6;
    return residual;
  }

  /* Makes a block diagonal tangent matrix with
    [ A00   0;
      0    A11]. */
  static unique_ptr<BlockSparseSymmetricMatrix3d> MakeTangentMatrix() {
    const std::vector<std::vector<int>> sparsity = {std::vector<int>({0}),
                                                    std::vector<int>({1})};
    contact_solvers::internal::BlockSparsityPattern block_pattern(
        std::vector<int>(2, 3), sparsity);
    auto A = make_unique<BlockSparseSymmetricMatrix3d>(block_pattern);
    A->SetBlock(0, 0, A00);
    A->SetBlock(1, 1, A11);
    return A;
  }

  /* The DirichletBoundaryCondition under test. */
  DirichletBoundaryCondition<double> bc_;
  unique_ptr<FemStateSystem<double>> fem_state_system_;
  unique_ptr<FemState<double>> fem_state_;
};

/* Tests that the DirichletBoundaryCondition under test successfully modifies
 a given state. */
TEST_F(DirichletBoundaryConditionTest, ApplyBoundaryConditionToState) {
  bc_.ApplyBoundaryConditionToState(fem_state_.get());
  Vector<double, kNumDofs> expected_q, expected_v, expected_a;
  expected_q << 1, 2, 3, 1.1, 1.2, 1.3;
  expected_v << 4, 5, 6, 0.8, 0.9, 1.0;
  expected_a << 7, 8, 9, 1.2, 1.4, 1.6;
  EXPECT_TRUE(CompareMatrices(fem_state_->GetPositions(), expected_q));
  EXPECT_TRUE(CompareMatrices(fem_state_->GetVelocities(), expected_v));
  EXPECT_TRUE(CompareMatrices(fem_state_->GetAccelerations(), expected_a));
}

/* Tests that the DirichletBoundaryCondition under test successfully modifies
 a given residual. */
TEST_F(DirichletBoundaryConditionTest, ApplyHomogeneousBoundaryCondition) {
  VectorXd b = MakeResidual();
  bc_.ApplyHomogeneousBoundaryCondition(&b);
  Vector<double, kNumDofs> expected_residual;
  expected_residual << 0, 0, 0, 4.4, 5.5, 6.6;
  EXPECT_TRUE(CompareMatrices(b, expected_residual));
}

/* Tests that the DirichletBoundaryCondition under test successfully modifies a
 given tangent matrix. */
TEST_F(DirichletBoundaryConditionTest,
       ApplyBoundaryConditionResidualAndTangentMatrix) {
  DenseMatrix expected_tangent_matrix;
  // clang-format off
  expected_tangent_matrix << 3, 0, 0,    0,    0,     0,
                             0, 2, 0,    0,    0,     0,
                             0, 0, 3,    0,    0,     0,

                             0, 0, 0,    0.3,  0.1,   0.05,
                             0, 0, 0,    0.1,  0.2,   0.025,
                             0, 0, 0,    0.05, 0.025, 0.3;
  // clang-format on

  auto A = MakeTangentMatrix();
  bc_.ApplyBoundaryConditionToTangentMatrix(A.get());
  EXPECT_TRUE(CompareMatrices(A->MakeDenseMatrix(), expected_tangent_matrix));
}

/* Tests out-of-bound boundary conditions throw an exception. */
TEST_F(DirichletBoundaryConditionTest, OutOfBound) {
  /* Put a dof that is out-of-bound under boundary condition. */
  bc_.AddBoundaryCondition(FemNodeIndex(100),
                           {Vector3<double>::Zero(), Vector3<double>::Zero(),
                            Vector3<double>::Zero()});
  DRAKE_EXPECT_THROWS_MESSAGE(
      bc_.ApplyBoundaryConditionToState(fem_state_.get()),
      "An index of the Dirichlet boundary condition is out of range.");
  VectorXd b = MakeResidual();
  DRAKE_EXPECT_THROWS_MESSAGE(
      bc_.ApplyHomogeneousBoundaryCondition(&b),
      "An index of the Dirichlet boundary condition is out of range.");
  auto A = MakeTangentMatrix();
  DRAKE_EXPECT_THROWS_MESSAGE(
      bc_.ApplyBoundaryConditionToTangentMatrix(A.get()),
      "An index of the Dirichlet boundary condition is out of range.");
}

/* Tests merging  */
TEST_F(DirichletBoundaryConditionTest, Merge) {
  /* Disjoint BCs. */
  {
    DirichletBoundaryCondition<double> another_bc;
    const Vector3<double> bc_q(10, 20, 30);
    const Vector3<double> bc_v(40, 50, 60);
    const Vector3<double> bc_a(70, 80, 90);
    another_bc.AddBoundaryCondition(FemNodeIndex(1), {bc_q, bc_v, bc_a});
    bc_.Merge(another_bc);
    Vector<double, kNumDofs> expected_q, expected_v, expected_a;
    expected_q << 1, 2, 3, 10, 20, 30;
    expected_v << 4, 5, 6, 40, 50, 60;
    expected_a << 7, 8, 9, 70, 80, 90;
    bc_.ApplyBoundaryConditionToState(fem_state_.get());
    EXPECT_TRUE(CompareMatrices(fem_state_->GetPositions(), expected_q));
    EXPECT_TRUE(CompareMatrices(fem_state_->GetVelocities(), expected_v));
    EXPECT_TRUE(CompareMatrices(fem_state_->GetAccelerations(), expected_a));
  }
  {
    /* Overlapping BCs. */
    DirichletBoundaryCondition<double> yet_another_bc;
    const Vector3<double> bc_q(100, 200, 300);
    const Vector3<double> bc_v(400, 500, 600);
    const Vector3<double> bc_a(700, 800, 900);
    yet_another_bc.AddBoundaryCondition(FemNodeIndex(0), {bc_q, bc_v, bc_a});
    bc_.Merge(yet_another_bc);
    Vector<double, kNumDofs> expected_q, expected_v, expected_a;
    expected_q << 100, 200, 300, 10, 20, 30;
    expected_v << 400, 500, 600, 40, 50, 60;
    expected_a << 700, 800, 900, 70, 80, 90;
    bc_.ApplyBoundaryConditionToState(fem_state_.get());
    EXPECT_TRUE(CompareMatrices(fem_state_->GetPositions(), expected_q));
    EXPECT_TRUE(CompareMatrices(fem_state_->GetVelocities(), expected_v));
    EXPECT_TRUE(CompareMatrices(fem_state_->GetAccelerations(), expected_a));
  }
  {
    /* Empty BC. */
    DirichletBoundaryCondition<double> empty_bc;
    bc_.Merge(empty_bc);
    Vector<double, kNumDofs> expected_q, expected_v, expected_a;
    expected_q << 100, 200, 300, 10, 20, 30;
    expected_v << 400, 500, 600, 40, 50, 60;
    expected_a << 700, 800, 900, 70, 80, 90;
    bc_.ApplyBoundaryConditionToState(fem_state_.get());
    EXPECT_TRUE(CompareMatrices(fem_state_->GetPositions(), expected_q));
    EXPECT_TRUE(CompareMatrices(fem_state_->GetVelocities(), expected_v));
    EXPECT_TRUE(CompareMatrices(fem_state_->GetAccelerations(), expected_a));
  }
  {
    /* Merge with self. */
    bc_.Merge(bc_);
    Vector<double, kNumDofs> expected_q, expected_v, expected_a;
    expected_q << 100, 200, 300, 10, 20, 30;
    expected_v << 400, 500, 600, 40, 50, 60;
    expected_a << 700, 800, 900, 70, 80, 90;
    bc_.ApplyBoundaryConditionToState(fem_state_.get());
    EXPECT_TRUE(CompareMatrices(fem_state_->GetPositions(), expected_q));
    EXPECT_TRUE(CompareMatrices(fem_state_->GetVelocities(), expected_v));
    EXPECT_TRUE(CompareMatrices(fem_state_->GetAccelerations(), expected_a));
  }
}

}  // namespace
}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
