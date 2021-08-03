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

TEST_F(SDPwithOverlappingVariables1,
       GenerateCsdpProblemDataWithoutFreeVariables) {
  const SdpaFreeFormat dut(*prog_);
  struct csdp::blockmatrix C_csdp;
  double* rhs_csdp;
  struct csdp::constraintmatrix* constraints{nullptr};
  GenerateCsdpProblemDataWithoutFreeVariables(dut, &C_csdp, &rhs_csdp,
                                              &constraints);

  // Check the cost min 2 * x0 + x2
  EXPECT_EQ(C_csdp.nblocks, 3);
  CompareBlockrec(C_csdp.blocks[1], csdp::MATRIX, 2, {-2, 0, 0, 0}, 0);
  CompareBlockrec(C_csdp.blocks[2], csdp::MATRIX, 2, {0, -0.5, -0.5, 0}, 0);
  CompareBlockrec(C_csdp.blocks[3], csdp::DIAG, 2, {0, 0}, 0);

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

  // The equality constraint x0 - X(4, 4) = 0.5
  blockptr = constraints[4].blocks;
  block_entries.clear();
  block_entries.emplace_back(0, 0, 1);
  CheckSparseblock(*blockptr, block_entries, 1, 2, 4);
  blockptr = blockptr->next;
  block_entries.clear();
  block_entries.emplace_back(0, 0, -1);
  CheckSparseblock(*blockptr, block_entries, 3, 2, 4);
  EXPECT_EQ(blockptr->next, nullptr);
  EXPECT_EQ(rhs_csdp[4], 0.5);

  // The equality constraint x1 = 1
  blockptr = constraints[5].blocks;
  block_entries.clear();
  block_entries.emplace_back(0, 1, 0.5);
  CheckSparseblock(*blockptr, block_entries, 1, 2, 5);
  EXPECT_EQ(blockptr->next, nullptr);
  EXPECT_EQ(rhs_csdp[5], 1);

  // To impose the inequality constraint x2 <= 2, we add the equality constraint
  // x2 + X(5, 5) = 2
  blockptr = constraints[6].blocks;
  block_entries.clear();
  block_entries.emplace_back(0, 1, 0.5);
  CheckSparseblock(*blockptr, block_entries, 2, 2, 6);
  blockptr = blockptr->next;
  block_entries.clear();
  block_entries.emplace_back(1, 1, 1);
  CheckSparseblock(*blockptr, block_entries, 3, 2, 6);
  EXPECT_EQ(blockptr->next, nullptr);
  EXPECT_EQ(rhs_csdp[6], 2);

  FreeCsdpProblemData(6, C_csdp, rhs_csdp, constraints);
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

TEST_F(SDPwithOverlappingVariables1, Solve) {
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
      csdp::easy_sdp(nullptr,
                     dut.num_X_rows(), dut.g().rows(), C, rhs, constraints,
                     -dut.constant_min_cost_term(), &X, &y, &Z, &pobj, &dobj);
  EXPECT_EQ(ret, 0 /* 0 is for success */);
  const double tol = 1E-7;
  EXPECT_NEAR(pobj, -1, tol);
  EXPECT_NEAR(dobj, -1, tol);
  // Check the value of X
  EXPECT_EQ(X.nblocks, 3);
  CompareBlockrec(X.blocks[1], csdp::MATRIX, 2, {1, 1, 1, 1}, tol);
  CompareBlockrec(X.blocks[2], csdp::MATRIX, 2, {1, -1, -1, 1}, tol);
  CompareBlockrec(X.blocks[3], csdp::DIAG, 2, {0.5, 3}, tol);

  csdp::free_prob(dut.num_X_rows(), dut.g().rows(), C, rhs, constraints, X, y,
                  Z);
}

TEST_F(SDPwithOverlappingVariables2, Solve) {
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
      csdp::easy_sdp(nullptr,
                     dut.num_X_rows(), dut.g().rows(), C, rhs, constraints,
                     -dut.constant_min_cost_term(), &X, &y, &Z, &pobj, &dobj);
  EXPECT_EQ(ret, 0 /* 0 is for success */);
  const double tol = 1E-7;
  EXPECT_NEAR(pobj, -5, tol);
  EXPECT_NEAR(dobj, -5, tol);
  // Check the value of X
  EXPECT_EQ(X.nblocks, 2);
  CompareBlockrec(X.blocks[1], csdp::MATRIX, 2, {2, 1, 1, 2}, tol);
  CompareBlockrec(X.blocks[2], csdp::DIAG, 3, {0, 1, 0}, tol);

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
      csdp::easy_sdp(nullptr,
                     dut.num_X_rows(), dut.g().rows(), C, rhs, constraints,
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
      csdp::easy_sdp(nullptr,
                     dut.num_X_rows(), dut.g().rows(), C, rhs, constraints,
                     -dut.constant_min_cost_term(), &X, &y, &Z, &pobj, &dobj);
  EXPECT_EQ(ret, 0 /* 0 is for success */);

  csdp::free_prob(dut.num_X_rows(), dut.g().rows(), C, rhs, constraints, X, y,
                  Z);
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake
