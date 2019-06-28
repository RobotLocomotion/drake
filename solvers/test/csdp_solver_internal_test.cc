#include "drake/solvers/csdp_solver_internal.h"

#include <limits>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/test/csdp_test_examples.h"

namespace drake {
namespace solvers {
namespace internal {

void CompareBlockrec(const csdp::blockrec& block, csdp::blockcat blockcategory,
                     int blocksize, const std::vector<double>& value,
                     double tol) {
  EXPECT_EQ(block.blockcategory, blockcategory);
  EXPECT_EQ(block.blocksize, blocksize);
  if (blockcategory == csdp::blockcat::MATRIX) {
    for (int j = 0; j < blocksize; ++j) {
      for (int i = 0; i < blocksize; ++i) {
        EXPECT_NEAR(block.data.mat[ijtok(i + 1, j + 1, blocksize)],
                    value[j * blocksize + i], tol);
      }
    }
  } else if (blockcategory == csdp::blockcat::DIAG) {
    for (int i = 0; i < blocksize; ++i) {
      EXPECT_NEAR(block.data.vec[i + 1], value[i], tol);
    }
  } else {
    throw std::invalid_argument("Unknown block category.");
  }
}

void CheckSparseblock(const csdp::sparseblock& block,
                      const std::vector<Eigen::Triplet<double>>& block_entries,
                      int blocknum, int blocksize, int constraintnum) {
  for (int i = 0; i < static_cast<int>(block_entries.size()); ++i) {
    EXPECT_EQ(block.entries[i + 1], block_entries[i].value());
    EXPECT_EQ(block.iindices[i + 1], block_entries[i].row() + 1);
    EXPECT_EQ(block.jindices[i + 1], block_entries[i].col() + 1);
    EXPECT_EQ(block.numentries, static_cast<int>(block_entries.size()));
    EXPECT_EQ(block.blocknum, blocknum);
    EXPECT_EQ(block.blocksize, blocksize);
    EXPECT_EQ(block.constraintnum, constraintnum);
  }
}

TEST_F(SDPwithOverlappingVariables,
       GenerateCsdpProblemDataWithoutFreeVariables) {
  const SdpaFreeFormat dut(*prog_);
  struct csdp::blockmatrix C_csdp;
  double* rhs_csdp;
  struct csdp::constraintmatrix* constraints{nullptr};
  GenerateCsdpProblemDataWithoutFreeVariables(dut, &C_csdp, &rhs_csdp,
                                              &constraints);

  // Check the cost min 2 * x0 + x2
  EXPECT_EQ(C_csdp.nblocks, 2);
  CompareBlockrec(C_csdp.blocks[1], csdp::MATRIX, 2, {-2, 0, 0, 0}, 0);
  CompareBlockrec(C_csdp.blocks[2], csdp::MATRIX, 2, {0, -0.5, -0.5, 0}, 0);

  // Check the equality constraint from
  // [x0 x1] is psd
  // [x1 x0]
  struct csdp::sparseblock* blockptr = constraints[1].blocks;
  std::vector<Eigen::Triplet<double>> block_entries;
  block_entries.emplace_back(0, 0, 1);
  block_entries.emplace_back(1, 1, -1);
  CheckSparseblock(*blockptr, block_entries, 1, 2, 1);
  EXPECT_EQ(blockptr->next, nullptr);
  EXPECT_EQ(rhs_csdp[1], 0);

  // Check the equality constraint from
  // [x0 x2] is psd
  // [x2 x0]
  // The equality constraint X(0, 0) - X(2, 2) = 0
  blockptr = constraints[2].blocks;
  block_entries.clear();
  block_entries.emplace_back(0, 0, 1);
  CheckSparseblock(*blockptr, block_entries, 1, 2, 2);
  blockptr = blockptr->next;
  block_entries.clear();
  block_entries.emplace_back(0, 0, -1);
  CheckSparseblock(*blockptr, block_entries, 2, 2, 2);
  EXPECT_EQ(blockptr->next, nullptr);
  EXPECT_EQ(rhs_csdp[2], 0);

  // The equality constraint X(0, 0) - X(3, 3) = 0
  blockptr = constraints[3].blocks;
  block_entries.clear();
  block_entries.emplace_back(0, 0, 1);
  CheckSparseblock(*blockptr, block_entries, 1, 2, 3);
  blockptr = blockptr->next;
  block_entries.clear();
  block_entries.emplace_back(1, 1, -1);
  CheckSparseblock(*blockptr, block_entries, 2, 2, 3);
  EXPECT_EQ(blockptr->next, nullptr);
  EXPECT_EQ(rhs_csdp[3], 0);

  // The equality constraint x1 = 1
  blockptr = constraints[4].blocks;
  block_entries.clear();
  block_entries.emplace_back(0, 1, 0.5);
  CheckSparseblock(*blockptr, block_entries, 1, 2, 4);
  EXPECT_EQ(blockptr->next, nullptr);
  EXPECT_EQ(rhs_csdp[4], 1);

  FreeCsdpProblemData(4, C_csdp, rhs_csdp, constraints);
}

TEST_F(CsdpDocExample, GenerateCsdpProblemDataWithoutFreeVariables) {
  const SdpaFreeFormat dut(*prog_);
  struct csdp::blockmatrix C_csdp;
  double* rhs_csdp;
  struct csdp::constraintmatrix* constraints{nullptr};
  GenerateCsdpProblemDataWithoutFreeVariables(dut, &C_csdp, &rhs_csdp,
                                              &constraints);

  // Check the cost.
  EXPECT_EQ(C_csdp.nblocks, 3);
  CompareBlockrec(C_csdp.blocks[1], csdp::MATRIX, 2, {2, 1, 1, 2}, 0);
  CompareBlockrec(C_csdp.blocks[2], csdp::MATRIX, 3,
                  {3, 0, 1, 0, 2, 0, 1, 0, 3}, 0);
  CompareBlockrec(C_csdp.blocks[3], csdp::DIAG, 2, {0, 0}, 0);

  // Check constraints.
  // Constraint 1.
  struct csdp::sparseblock* blockptr = constraints[1].blocks;
  std::vector<Eigen::Triplet<double>> block_entries;
  block_entries.emplace_back(0, 0, 3);
  block_entries.emplace_back(0, 1, 1);
  block_entries.emplace_back(1, 1, 3);
  CheckSparseblock(*blockptr, block_entries, 1, 2, 1);
  blockptr = blockptr->next;
  block_entries.clear();
  block_entries.emplace_back(0, 0, 1);
  CheckSparseblock(*blockptr, block_entries, 3, 2, 1);
  EXPECT_EQ(blockptr->next, nullptr);
  EXPECT_EQ(rhs_csdp[1], 1);

  // Constraint 2.
  blockptr = constraints[2].blocks;
  block_entries.clear();
  block_entries.emplace_back(0, 0, 3);
  block_entries.emplace_back(1, 1, 4);
  block_entries.emplace_back(0, 2, 1);
  block_entries.emplace_back(2, 2, 5);
  CheckSparseblock(*blockptr, block_entries, 2, 3, 2);
  blockptr = blockptr->next;
  block_entries.clear();
  CheckSparseblock(*blockptr, block_entries, 3, 2, 2);
  EXPECT_EQ(blockptr->next, nullptr);
  block_entries.emplace_back(1, 1, 1);
  EXPECT_EQ(rhs_csdp[2], 2);

  FreeCsdpProblemData(2, C_csdp, rhs_csdp, constraints);
}

TEST_F(TrivialSDP1, GenerateCsdpProblemDataWithoutFreeVariables) {
  const SdpaFreeFormat dut(*prog_);
  struct csdp::blockmatrix C_csdp;
  double* rhs_csdp;
  struct csdp::constraintmatrix* constraints{nullptr};
  GenerateCsdpProblemDataWithoutFreeVariables(dut, &C_csdp, &rhs_csdp,
                                              &constraints);
  /**
   * A trivial SDP
   * max X1(0, 1) + X1(1, 2)
   * s.t X1 ∈ ℝ³ˣ³ is psd
   *     X1(0, 0) + X1(1, 1) + X1(2, 2) = 1
   *     X1(0, 1) + X1(1, 2) - 2 * X1(0, 2) <= 0
   */
  // Check the cost
  EXPECT_EQ(C_csdp.nblocks, 2);
  CompareBlockrec(C_csdp.blocks[1], csdp::MATRIX, 3,
                  {0, 0.5, 0, 0.5, 0, 0.5, 0, 0.5, 0}, 0);
  CompareBlockrec(C_csdp.blocks[2], csdp::DIAG, 1, {0}, 0);
  // Check the constraint X1(0, 0) + X1(1, 1) + X1(2, 2) = 1
  struct csdp::sparseblock* blockptr = constraints[1].blocks;
  std::vector<Eigen::Triplet<double>> block_entries;
  block_entries.emplace_back(0, 0, 1);
  block_entries.emplace_back(1, 1, 1);
  block_entries.emplace_back(2, 2, 1);
  CheckSparseblock(*blockptr, block_entries, 1, 3, 1);
  EXPECT_EQ(blockptr->next, nullptr);
  EXPECT_EQ(rhs_csdp[1], 1);

  // Check the constraint X1(0, 1) + X1(1, 2) - 2 * X1(0, 2) <= 0
  blockptr = constraints[2].blocks;
  block_entries.clear();
  block_entries.emplace_back(0, 1, 0.5);
  block_entries.emplace_back(0, 2, -1);
  block_entries.emplace_back(1, 2, 0.5);
  CheckSparseblock(*blockptr, block_entries, 1, 3, 2);
  blockptr = blockptr->next;
  block_entries.clear();
  block_entries.emplace_back(0, 0, 1);
  CheckSparseblock(*blockptr, block_entries, 2, 1, 2);
  EXPECT_EQ(blockptr->next, nullptr);
  EXPECT_EQ(rhs_csdp[2], 0);

  FreeCsdpProblemData(2, C_csdp, rhs_csdp, constraints);
}

TEST_F(SDPwithOverlappingVariables, Solve) {
  const SdpaFreeFormat dut(*prog_);
  csdp::blockmatrix C;
  double* rhs;
  csdp::constraintmatrix* constraints{nullptr};
  GenerateCsdpProblemDataWithoutFreeVariables(dut, &C, &rhs, &constraints);

  struct csdp::blockmatrix X, Z;
  double* y;
  csdp::initsoln(dut.num_X_rows(), dut.g().rows(), C, rhs, constraints, &X, &y,
                 &Z);
  double pobj, dobj;
  const int ret =
      csdp::easy_sdp(dut.num_X_rows(), dut.g().rows(), C, rhs, constraints,
                     -dut.constant_min_cost_term(), &X, &y, &Z, &pobj, &dobj);
  EXPECT_EQ(ret, 0 /* 0 is for success */);
  const double tol = 1E-7;
  EXPECT_NEAR(pobj, -1, tol);
  EXPECT_NEAR(dobj, -1, tol);
  // Check the value of X
  EXPECT_EQ(X.nblocks, 2);
  CompareBlockrec(X.blocks[1], csdp::MATRIX, 2, {1, 1, 1, 1}, tol);
  CompareBlockrec(X.blocks[2], csdp::MATRIX, 2, {1, -1, -1, 1}, tol);

  csdp::free_prob(dut.num_X_rows(), dut.g().rows(), C, rhs, constraints, X, y,
                  Z);
}

TEST_F(CsdpDocExample, Solve) {
  const SdpaFreeFormat dut(*prog_);
  csdp::blockmatrix C;
  double* rhs;
  csdp::constraintmatrix* constraints{nullptr};
  GenerateCsdpProblemDataWithoutFreeVariables(dut, &C, &rhs, &constraints);

  struct csdp::blockmatrix X, Z;
  double* y;
  csdp::initsoln(dut.num_X_rows(), dut.g().rows(), C, rhs, constraints, &X, &y,
                 &Z);
  double pobj, dobj;
  const int ret =
      csdp::easy_sdp(dut.num_X_rows(), dut.g().rows(), C, rhs, constraints,
                     -dut.constant_min_cost_term(), &X, &y, &Z, &pobj, &dobj);
  EXPECT_EQ(ret, 0 /* 0 is for success */);

  const double tol = 5E-7;
  EXPECT_NEAR(pobj, 2.75, tol);
  EXPECT_NEAR(y[1], 0.75, tol);
  EXPECT_NEAR(y[2], 1, tol);

  EXPECT_EQ(X.nblocks, 3);
  CompareBlockrec(X.blocks[1], csdp::MATRIX, 2, {0.125, 0.125, 0.125, 0.125},
                  tol);
  CompareBlockrec(X.blocks[2], csdp::MATRIX, 3,
                  {2.0 / 3, 0, 0, 0, 0, 0, 0, 0, 0}, tol);
  CompareBlockrec(X.blocks[3], csdp::DIAG, 2, {0, 0}, tol);

  EXPECT_EQ(Z.nblocks, 3);
  CompareBlockrec(Z.blocks[1], csdp::MATRIX, 2, {0.25, -0.25, -0.25, 0.25},
                  tol);
  CompareBlockrec(Z.blocks[2], csdp::MATRIX, 3, {0, 0, 0, 0, 2, 0, 0, 0, 2},
                  tol);
  CompareBlockrec(Z.blocks[3], csdp::DIAG, 2, {0.75, 1}, tol);

  csdp::free_prob(dut.num_X_rows(), dut.g().rows(), C, rhs, constraints, X, y,
                  Z);
}

TEST_F(TrivialSDP1, Solve) {
  const SdpaFreeFormat dut(*prog_);
  csdp::blockmatrix C;
  double* rhs;
  csdp::constraintmatrix* constraints{nullptr};
  GenerateCsdpProblemDataWithoutFreeVariables(dut, &C, &rhs, &constraints);

  struct csdp::blockmatrix X, Z;
  double* y;
  csdp::initsoln(dut.num_X_rows(), dut.g().rows(), C, rhs, constraints, &X, &y,
                 &Z);
  double pobj, dobj;
  const int ret =
      csdp::easy_sdp(dut.num_X_rows(), dut.g().rows(), C, rhs, constraints,
                     -dut.constant_min_cost_term(), &X, &y, &Z, &pobj, &dobj);
  EXPECT_EQ(ret, 0 /* 0 is for success */);

  csdp::free_prob(dut.num_X_rows(), dut.g().rows(), C, rhs, constraints, X, y,
                  Z);
}

void CheckRemoveFreeVariableByNullspaceApproach(
    const SdpaFreeFormat& dut, const Eigen::SparseMatrix<double>& C_hat,
    const std::vector<Eigen::SparseMatrix<double>>& A_hat,
    const Eigen::VectorXd& rhs_hat, const Eigen::VectorXd& y_hat,
    const Eigen::SparseQR<Eigen::SparseMatrix<double>,
                          Eigen::COLAMDOrdering<int>>& QR_B,
    double tol) {
  EXPECT_EQ(y_hat.rows(), static_cast<int>(dut.A().size()));
  // Check Bᵀ * ŷ = d
  EXPECT_TRUE(CompareMatrices(Eigen::VectorXd(dut.B().transpose() * y_hat),
                              Eigen::VectorXd(dut.d()), tol));
  // Check Ĉ = C -∑ᵢ ŷᵢAᵢ
  Eigen::SparseMatrix<double> C_hat_expected = dut.C();
  for (int i = 0; i < y_hat.rows(); ++i) {
    C_hat_expected -= y_hat(i) * dut.A()[i];
  }
  EXPECT_TRUE(CompareMatrices(Eigen::MatrixXd(C_hat),
                              Eigen::MatrixXd(C_hat_expected), tol));
  // N is the null space of Bᵀ. Namely if we do a QR decomposition on B, then
  // N = Q₂.
  Eigen::SparseMatrix<double> Q;
  Q = QR_B.matrixQ();
  const Eigen::SparseMatrix<double> N =
      Q.rightCols(dut.B().rows() - QR_B.rank());
  EXPECT_TRUE(CompareMatrices(Eigen::MatrixXd(dut.B().transpose() * N),
                              Eigen::MatrixXd::Zero(dut.B().cols(), N.cols()),
                              tol));
  // Check rhs_hat = Nᵀ * rhs
  EXPECT_TRUE(
      CompareMatrices(rhs_hat, Eigen::VectorXd(N.transpose() * dut.g()), tol));
  // Check Âᵢ = ∑ⱼNⱼᵢAⱼ
  EXPECT_EQ(static_cast<int>(A_hat.size()), N.cols());
  for (int i = 0; i < N.cols(); ++i) {
    Eigen::SparseMatrix<double> A_hat_expected(dut.num_X_rows(),
                                               dut.num_X_rows());
    A_hat_expected.setZero();
    for (int j = 0; j < static_cast<int>(dut.A().size()); ++j) {
      A_hat_expected += N.coeff(j, i) * dut.A()[j];
    }
    EXPECT_TRUE(CompareMatrices(Eigen::MatrixXd(A_hat[i]),
                                Eigen::MatrixXd(A_hat_expected), tol));
  }
}

void TestRemoveFreeVariableByNullspaceApproach(
    const MathematicalProgram& prog) {
  const SdpaFreeFormat dut(prog);
  Eigen::SparseMatrix<double> C_hat;
  std::vector<Eigen::SparseMatrix<double>> A_hat;
  Eigen::VectorXd rhs_hat, y_hat;
  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> QR_B;
  RemoveFreeVariableByNullspaceApproach(dut, &C_hat, &A_hat, &rhs_hat, &y_hat,
                                        &QR_B);
  CheckRemoveFreeVariableByNullspaceApproach(dut, C_hat, A_hat, rhs_hat, y_hat,
                                             QR_B, 1E-10);
}

TEST_F(LinearProgramBoundingBox1, RemoveFreeVariableByNullspaceApproach) {
  const SdpaFreeFormat dut(*prog_);
  Eigen::SparseMatrix<double> C_hat;
  std::vector<Eigen::SparseMatrix<double>> A_hat;
  Eigen::VectorXd rhs_hat, y_hat;
  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> QR_B;
  RemoveFreeVariableByNullspaceApproach(dut, &C_hat, &A_hat, &rhs_hat, &y_hat,
                                        &QR_B);
  CheckRemoveFreeVariableByNullspaceApproach(dut, C_hat, A_hat, rhs_hat, y_hat,
                                             QR_B, 1E-10);

  // Now try to call CSDP to solve this problem.
  csdp::blockmatrix C_csdp;
  double* rhs_csdp{nullptr};
  csdp::constraintmatrix* constraints_csdp{nullptr};
  ConvertSparseMatrixFormatToCsdpProblemData(dut.X_blocks(), C_hat, A_hat,
                                              rhs_hat, &C_csdp, &rhs_csdp,
                                              &constraints_csdp);
  struct csdp::blockmatrix X_csdp, Z;
  double* y{nullptr};
  csdp::initsoln(dut.num_X_rows(), rhs_hat.rows(), C_csdp, rhs_csdp,
                 constraints_csdp, &X_csdp, &y, &Z);
  double pobj{0};
  double dobj{0};
  const int ret = csdp::easy_sdp(
      dut.num_X_rows(), rhs_hat.rows(), C_csdp, rhs_csdp, constraints_csdp,
      -dut.constant_min_cost_term() + dut.g().dot(y_hat), &X_csdp, &y, &Z,
      &pobj, &dobj);
  EXPECT_EQ(ret, 0 /* 0 is for success */);
  Eigen::SparseMatrix<double> X_hat(dut.num_X_rows(), dut.num_X_rows());
  ConvertCsdpBlockMatrixtoEigen(X_csdp, &X_hat);
  // Now compute the free variable values.
  Eigen::VectorXd AX(dut.A().size());
  for (int i = 0; i < AX.rows(); ++i) {
    AX(i) = (dut.A()[i].cwiseProduct(X_hat)).sum();
  }
  Eigen::VectorXd s_val;
  s_val = QR_B.solve(dut.g() - AX);
  const double tol = 1E-6;
  EXPECT_NEAR(pobj, 43, tol);
  EXPECT_EQ(X_csdp.nblocks, 1);
  CompareBlockrec(X_csdp.blocks[1], csdp::DIAG, 7, {0, 5, 0, 0, 0, 7, 0}, tol);
  csdp::free_prob(dut.num_X_rows(), rhs_hat.rows(), C_csdp, rhs_csdp,
                  constraints_csdp, X_csdp, y, Z);
}

TEST_F(CsdpLinearProgram2, RemoveFreeVariableByNullspaceApproach) {
  TestRemoveFreeVariableByNullspaceApproach(*prog_);
}

TEST_F(TrivialSDP2, RemoveFreeVariableByNullspaceApproach) {
  TestRemoveFreeVariableByNullspaceApproach(*prog_);
}

TEST_F(TrivialSOCP1, RemoveFreeVariableByNullspaceApproach) {
  TestRemoveFreeVariableByNullspaceApproach(*prog_);
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake
