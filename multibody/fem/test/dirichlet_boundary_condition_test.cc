#include "drake/multibody/fem/dirichlet_boundary_condition.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/fem/fem_state.h"
#include "drake/multibody/fem/test/dummy_element.h"

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
using SparseMatrix = Eigen::SparseMatrix<double>;
using Element = DummyElement;
using State = FemStateImpl<Element>;
using std::make_unique;
using std::unique_ptr;
using std::vector;

class DirichletBoundaryConditionTest : public ::testing::Test {
 protected:
  void SetUp() {
    bc_.AddBoundaryCondition(0, Vector3<double>(3, 2, 1));
    bc_.AddBoundaryCondition(2, Vector3<double>(-3, -2, 0));
  }

  /* Makes an arbitrary residual vector with appropriate size. */
  static VectorXd MakeResidual() {
    return Vector<double, kNumDofs>(1.1, 2.2, 3.3, 4.4);
  }

  /* Makes an arbitrary SPD tangent matrix with appropriate size. */
  static SparseMatrix MakeEigenTangentMatrix() {
    DenseMatrix A;
    // clang-format off
    A << 3,   1,     0.5, -1,
         1,   2,     0.25, 0.125,
         0.5, 0.25,  3,    0.25,
        -1,   0.125, 0.25, 4;
    // clang-format on
    SparseMatrix A_eigen_sparse = A.sparseView();
    return A_eigen_sparse;
  }

  /* Returns the same matrix as in MakeEigenTangentMatrix(), but in PETSc
   format. The PETSc matrix contains a single block matrix of size 4-by-4.*/
  static unique_ptr<PetscSymmetricBlockSparseMatrix> MakePetscTangentMatrix() {
    const vector<int> num_upper_triangular_blocks_per_row = {1};
    auto A = make_unique<PetscSymmetricBlockSparseMatrix>(
        kNumDofs, kNumDofs, num_upper_triangular_blocks_per_row);
    const Vector1<int> index(0);
    A->AddToBlock(index, DenseMatrix(MakeEigenTangentMatrix()));
    A->AssembleIfNecessary();
    return A;
  }

  /* Makes an arbitrary compatible FemStateImpl with appropriate size. */
  static State MakeState() {
    State state{Vector<double, kNumDofs>(0.1, 0.2, 0.3, 0.4),
                Vector<double, kNumDofs>(0.5, 0.6, 0.7, 0.8),
                Vector<double, kNumDofs>(0.9, 1.0, 1.1, 1.2)};
    return state;
  }

  /* The DirichletBoundaryCondition under test. */
  DirichletBoundaryCondition<double> bc_;
};

/* Tests that the DirichletBoundaryCondition under test successfully modifies
 a given state. */
TEST_F(DirichletBoundaryConditionTest, ApplyBoundaryConditionToState) {
  State s = MakeState();
  bc_.ApplyBoundaryConditionToState(&s);
  EXPECT_TRUE(CompareMatrices(s.GetPositions(),
                              Vector<double, kNumDofs>{3, 0.2, -3, 0.4}));
  EXPECT_TRUE(CompareMatrices(s.GetVelocities(),
                              Vector<double, kNumDofs>{2, 0.6, -2, 0.8}));
  EXPECT_TRUE(CompareMatrices(s.GetAccelerations(),
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
  /* Test for the Eigen::SparseMatrix overload. */
  SparseMatrix A_eigen_sparse = MakeEigenTangentMatrix();
  const int nnz = A_eigen_sparse.nonZeros();
  bc_.ApplyBoundaryConditionToTangentMatrix(&A_eigen_sparse);
  /* The number of nonzeros should not change. */
  EXPECT_EQ(A_eigen_sparse.nonZeros(), nnz);
  const DenseMatrix A(A_eigen_sparse);
  DenseMatrix A_expected;
  // clang-format off
  A_expected << 1, 0,     0, 0,
                0, 2,     0, 0.125,
                0, 0,     1, 0,
                0, 0.125, 0, 4;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(A, A_expected));

  /* Test for the PetscSymmetricBlockSparseMatrix overload. */
  auto A_petsc = MakePetscTangentMatrix();
  bc_.ApplyBoundaryConditionToTangentMatrix(A_petsc.get());
  EXPECT_TRUE(CompareMatrices(A_petsc->MakeDenseMatrix(), A_expected));
}

/* Tests out-of-bound boundary conditions throw an exception. */
TEST_F(DirichletBoundaryConditionTest, OutOfBound) {
  /* Put a dof that is out-of-bound under boundary condition. */
  bc_.AddBoundaryCondition(4, Vector3<double>(9, 1, 1));
  State state = MakeState();
  DRAKE_EXPECT_THROWS_MESSAGE(
      bc_.ApplyBoundaryConditionToState(&state),
      "An index of the Dirichlet boundary condition is out of the range.");
  VectorXd b = MakeResidual();
  DRAKE_EXPECT_THROWS_MESSAGE(
      bc_.ApplyBoundaryConditionToResidual(&b),
      "An index of the Dirichlet boundary condition is out of the range.");
  SparseMatrix A_eigen_sparse = MakeEigenTangentMatrix();
  DRAKE_EXPECT_THROWS_MESSAGE(
      bc_.ApplyBoundaryConditionToTangentMatrix(&A_eigen_sparse),
      "An index of the Dirichlet boundary condition is out of the range.");
  auto A_petsc = MakePetscTangentMatrix();
  DRAKE_EXPECT_THROWS_MESSAGE(
      bc_.ApplyBoundaryConditionToTangentMatrix(A_petsc.get()),
      "An index of the Dirichlet boundary condition is out of the range.");
}

}  // namespace
}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
