#include "drake/multibody/fem/dirichlet_boundary_condition.h"

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
static constexpr int kNumDofs = 4;
using DenseMatrix = Eigen::Matrix<double, kNumDofs, kNumDofs>;
using Eigen::VectorXd;
using std::make_unique;
using std::unique_ptr;
using std::vector;

class DirichletBoundaryConditionTest : public ::testing::Test {
 protected:
  void SetUp() {
    bc_.AddBoundaryCondition(0, Vector3<double>(3, 2, 1));
    bc_.AddBoundaryCondition(2, Vector3<double>(-3, -2, 0));
    /* Makes an arbitrary compatible model state. */
    fem_state_system_ = make_unique<FemStateSystem<double>>(
        Vector<double, kNumDofs>(0.1, 0.2, 0.3, 0.4),
        Vector<double, kNumDofs>(0.5, 0.6, 0.7, 0.8),
        Vector<double, kNumDofs>(0.9, 1.0, 1.1, 1.2));
    fem_state_ = make_unique<FemState<double>>(fem_state_system_.get());
  }

  /* Makes an arbitrary residual vector with appropriate size. */
  static VectorXd MakeResidual() {
    return Vector<double, kNumDofs>(1.1, 2.2, 3.3, 4.4);
  }

  /* Makes an arbitrary SPD tangent matrix with appropriate size in PETSc
   format. The PETSc matrix contains a single block matrix of size 4-by-4.*/
  static unique_ptr<PetscSymmetricBlockSparseMatrix> MakePetscTangentMatrix() {
    const vector<int> num_upper_triangular_blocks_per_row = {1};
    auto A = make_unique<PetscSymmetricBlockSparseMatrix>(
        kNumDofs, kNumDofs, num_upper_triangular_blocks_per_row);
    const Vector1<int> index(0);
    DenseMatrix A_block;
    // clang-format off
    A_block << 3,   1,     0.5,  -1,
               1,   2,     0.25,  0.125,
               0.5, 0.25,  3,     0.25,
              -1,   0.125, 0.25,  4;
    // clang-format on
    A->AddToBlock(index, A_block);
    A->AssembleIfNecessary();
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
  EXPECT_TRUE(CompareMatrices(fem_state_->GetPositions(),
                              Vector<double, kNumDofs>{3, 0.2, -3, 0.4}));
  EXPECT_TRUE(CompareMatrices(fem_state_->GetVelocities(),
                              Vector<double, kNumDofs>{2, 0.6, -2, 0.8}));
  EXPECT_TRUE(CompareMatrices(fem_state_->GetAccelerations(),
                              Vector<double, kNumDofs>{1, 1.0, 0, 1.2}));
}

/* Tests that the DirichletBoundaryCondition under test successfully modifies
 a given residual. */
TEST_F(DirichletBoundaryConditionTest, ApplyBoundaryConditionToResidual) {
  VectorXd b = MakeResidual();
  bc_.ApplyBoundaryConditionToResidual(&b);
  const Vector<double, kNumDofs> b_expected(0, 2.2, 0, 4.4);
  EXPECT_TRUE(CompareMatrices(b, b_expected));
}

/* Tests that the DirichletBoundaryCondition under test successfully modifies a
 given tangent matrix. */
TEST_F(DirichletBoundaryConditionTest,
       ApplyBoundaryConditionResidualAndTangentMatrix) {
  DenseMatrix A_expected;
  // clang-format off
  A_expected << 1, 0,     0, 0,
                0, 2,     0, 0.125,
                0, 0,     1, 0,
                0, 0.125, 0, 4;
  // clang-format on

  auto A_petsc = MakePetscTangentMatrix();
  bc_.ApplyBoundaryConditionToTangentMatrix(A_petsc.get());
  EXPECT_TRUE(CompareMatrices(A_petsc->MakeDenseMatrix(), A_expected));
}

/* Tests out-of-bound boundary conditions throw an exception. */
TEST_F(DirichletBoundaryConditionTest, OutOfBound) {
  /* Put a dof that is out-of-bound under boundary condition. */
  bc_.AddBoundaryCondition(4, Vector3<double>(9, 1, 1));
  DRAKE_EXPECT_THROWS_MESSAGE(
      bc_.ApplyBoundaryConditionToState(fem_state_.get()),
      "An index of the Dirichlet boundary condition is out of range.");
  VectorXd b = MakeResidual();
  DRAKE_EXPECT_THROWS_MESSAGE(
      bc_.ApplyBoundaryConditionToResidual(&b),
      "An index of the Dirichlet boundary condition is out of range.");
  auto A_petsc = MakePetscTangentMatrix();
  DRAKE_EXPECT_THROWS_MESSAGE(
      bc_.ApplyBoundaryConditionToTangentMatrix(A_petsc.get()),
      "An index of the Dirichlet boundary condition is out of range.");
}

}  // namespace
}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
