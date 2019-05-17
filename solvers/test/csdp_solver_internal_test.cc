#include "drake/solvers/csdp_solver_internal.h"

#include <limits>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/test/csdp_test_examples.h"

namespace drake {
namespace solvers {
namespace internal {

void CompareEntryInX(const EntryInX& entry1, const EntryInX& entry2) {
  EXPECT_EQ(entry1.block_index, entry2.block_index);
  EXPECT_EQ(entry1.row_index_in_block, entry2.row_index_in_block);
  EXPECT_EQ(entry1.column_index_in_block, entry2.column_index_in_block);
  EXPECT_EQ(entry1.X_start_row, entry2.X_start_row);
}

void CompareBlockInX(const BlockInX& block1, const BlockInX& block2) {
  EXPECT_EQ(block1.blockcategory, block2.blockcategory);
  EXPECT_EQ(block1.num_rows, block2.num_rows);
}

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

void CompareTriplets(const std::vector<Eigen::Triplet<double>>& triplets1,
                     const std::vector<Eigen::Triplet<double>>& triplets2,
                     int matrix_rows, int matrix_cols) {
  EXPECT_EQ(triplets1.size(), triplets2.size());
  Eigen::SparseMatrix<double> mat1(matrix_rows, matrix_cols);
  mat1.setFromTriplets(triplets1.begin(), triplets1.end());
  Eigen::SparseMatrix<double> mat2(matrix_rows, matrix_cols);
  mat2.setFromTriplets(triplets2.begin(), triplets2.end());
  EXPECT_TRUE(
      CompareMatrices(Eigen::MatrixXd(mat1), Eigen::MatrixXd(mat2), 1E-12));
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

TEST_F(SDPwithOverlappingVariables, TestSdpaFreeFormatConstructor) {
  const SdpaFreeFormat dut(*prog_);
  EXPECT_EQ(dut.num_X_rows(), 4);
  EXPECT_EQ(dut.num_free_variables(), 0);
  EXPECT_EQ(dut.X_blocks().size(), 2);
  CompareBlockInX(dut.X_blocks()[0], BlockInX(csdp::MATRIX, 2));
  CompareBlockInX(dut.X_blocks()[1], BlockInX(csdp::MATRIX, 2));
  EXPECT_EQ(dut.map_prog_var_index_to_entry_in_X().size(), 3);
  CompareEntryInX(dut.map_prog_var_index_to_entry_in_X().at(
                      prog_->FindDecisionVariableIndex(x_(0))),
                  EntryInX(0, 0, 0, 0));
  CompareEntryInX(dut.map_prog_var_index_to_entry_in_X().at(
                      prog_->FindDecisionVariableIndex(x_(1))),
                  EntryInX(0, 0, 1, 0));
  CompareEntryInX(dut.map_prog_var_index_to_entry_in_X().at(
                      prog_->FindDecisionVariableIndex(x_(2))),
                  EntryInX(1, 0, 1, 2));
  EXPECT_EQ(dut.map_prog_var_index_to_s_index().size(), 0);

  // Check A_triplets.
  EXPECT_EQ(dut.A_triplets().size(), 4);
  std::vector<Eigen::Triplet<double>> A0_triplets_expected;
  A0_triplets_expected.emplace_back(0, 0, 1.0);
  A0_triplets_expected.emplace_back(1, 1, -1.0);
  CompareTriplets(dut.A_triplets()[0], A0_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  std::vector<Eigen::Triplet<double>> A1_triplets_expected;
  A1_triplets_expected.emplace_back(0, 0, 1.0);
  A1_triplets_expected.emplace_back(2, 2, -1.0);
  CompareTriplets(dut.A_triplets()[1], A1_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  std::vector<Eigen::Triplet<double>> A2_triplets_expected;
  A2_triplets_expected.emplace_back(0, 0, 1.0);
  A2_triplets_expected.emplace_back(3, 3, -1.0);
  CompareTriplets(dut.A_triplets()[2], A2_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  std::vector<Eigen::Triplet<double>> A3_triplets_expected;
  A3_triplets_expected.emplace_back(0, 1, 0.5);
  A3_triplets_expected.emplace_back(1, 0, 0.5);
  CompareTriplets(dut.A_triplets()[3], A3_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());

  EXPECT_TRUE(CompareMatrices(dut.g(), Eigen::Vector4d(0, 0, 0, 1)));
  EXPECT_EQ(dut.B_triplets().size(), 0);

  // Now check the cost.
  std::vector<Eigen::Triplet<double>> C_triplets_expected;
  C_triplets_expected.emplace_back(0, 0, -2);
  C_triplets_expected.emplace_back(2, 3, -0.5);
  C_triplets_expected.emplace_back(3, 2, -0.5);
  CompareTriplets(dut.C_triplets(), C_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  EXPECT_EQ(dut.d_triplets().size(), 0);
  EXPECT_EQ(dut.constant_min_cost_term(), 0);
}

TEST_F(CsdpDocExample, TestSdpaFreeFormatConstructor) {
  SdpaFreeFormat dut(*prog_);
  EXPECT_EQ(dut.num_X_rows(), 7);
  EXPECT_EQ(dut.num_free_variables(), 0);
  EXPECT_EQ(dut.X_blocks().size(), 3);

  // Check the cost.
  std::vector<Eigen::Triplet<double>> C_triplets_expected;
  C_triplets_expected.emplace_back(0, 0, 2);
  C_triplets_expected.emplace_back(0, 1, 1);
  C_triplets_expected.emplace_back(1, 0, 1);
  C_triplets_expected.emplace_back(1, 1, 2);
  C_triplets_expected.emplace_back(2, 2, 3);
  C_triplets_expected.emplace_back(3, 3, 2);
  C_triplets_expected.emplace_back(2, 4, 1);
  C_triplets_expected.emplace_back(4, 2, 1);
  C_triplets_expected.emplace_back(4, 4, 3);
  CompareTriplets(dut.C_triplets(), C_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  EXPECT_EQ(dut.d_triplets().size(), 0);
  EXPECT_EQ(dut.constant_min_cost_term(), 0);
}

TEST_F(LinearProgram1, TestSdpaFreeFormatConstructor) {
  // Test if we can correctly register decision variables with bounding box
  // constraints.
  SdpaFreeFormat dut(*prog_);
  EXPECT_EQ(dut.num_X_rows(), 8);
  EXPECT_EQ(dut.num_free_variables(), 4);

  EXPECT_EQ(dut.X_blocks().size(), 1);
  CompareBlockInX(dut.X_blocks()[0], BlockInX(csdp::DIAG, 8));
  EXPECT_EQ(dut.map_prog_var_index_to_entry_in_X().size(), 3);
  CompareEntryInX(dut.map_prog_var_index_to_entry_in_X().at(0),
                  EntryInX(0, 0, 0, 0));
  CompareEntryInX(dut.map_prog_var_index_to_entry_in_X().at(1),
                  EntryInX(0, 1, 1, 0));
  CompareEntryInX(dut.map_prog_var_index_to_entry_in_X().at(5),
                  EntryInX(0, 7, 7, 0));
  EXPECT_EQ(dut.map_prog_var_index_to_s_index().size(), 4);
  EXPECT_EQ(dut.map_prog_var_index_to_s_index().at(
                SdpaFreeFormat::DecisionVariableIndex(2)),
            0);
  EXPECT_EQ(dut.map_prog_var_index_to_s_index().at(
                SdpaFreeFormat::DecisionVariableIndex(3)),
            1);
  EXPECT_EQ(dut.map_prog_var_index_to_s_index().at(
                SdpaFreeFormat::DecisionVariableIndex(4)),
            2);
  EXPECT_EQ(dut.map_prog_var_index_to_s_index().at(
                SdpaFreeFormat::DecisionVariableIndex(6)),
            3);

  // Check the linear constraint.
  EXPECT_EQ(dut.A_triplets().size(), 7);
  std::vector<Eigen::Triplet<double>> B_triplets_expected;
  Eigen::Matrix<double, 7, 1> g_expected;
  // The first constraint is for x(1) <= 5
  std::vector<Eigen::Triplet<double>> A0_triplets_expected;
  A0_triplets_expected.emplace_back(1, 1, 1.0);
  A0_triplets_expected.emplace_back(2, 2, 1.0);
  CompareTriplets(dut.A_triplets()[0], A0_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  g_expected(0) = 5;
  // The second constraint is for x(2) >= -1
  std::vector<Eigen::Triplet<double>> A1_triplets_expected;
  A1_triplets_expected.emplace_back(3, 3, -1.0);
  CompareTriplets(dut.A_triplets()[1], A1_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  B_triplets_expected.emplace_back(
      1,
      dut.map_prog_var_index_to_s_index().at(
          SdpaFreeFormat::DecisionVariableIndex(2)),
      1.0);
  g_expected(1) = -1;
  // The third constrait is for x(3) <= 10
  std::vector<Eigen::Triplet<double>> A2_triplets_expected;
  A2_triplets_expected.emplace_back(4, 4, 1.0);
  CompareTriplets(dut.A_triplets()[2], A2_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  B_triplets_expected.emplace_back(
      2,
      dut.map_prog_var_index_to_s_index().at(
          SdpaFreeFormat::DecisionVariableIndex(3)),
      1.0);
  g_expected(2) = 10;
  // The forth constraint is for x(4) >= -2
  std::vector<Eigen::Triplet<double>> A3_triplets_expected;
  A3_triplets_expected.emplace_back(5, 5, -1.0);
  CompareTriplets(dut.A_triplets()[3], A3_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  B_triplets_expected.emplace_back(
      3,
      dut.map_prog_var_index_to_s_index().at(
          SdpaFreeFormat::DecisionVariableIndex(4)),
      1.0);
  g_expected(3) = -2;
  // The fifth constraint is for x(4) <= 5.
  std::vector<Eigen::Triplet<double>> A4_triplets_expected;
  A4_triplets_expected.emplace_back(6, 6, 1.0);
  CompareTriplets(dut.A_triplets()[4], A4_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  B_triplets_expected.emplace_back(
      4,
      dut.map_prog_var_index_to_s_index().at(
          SdpaFreeFormat::DecisionVariableIndex(4)),
      1.0);
  g_expected(4) = 5;
  // The sixth constraint is for x(5) == 0.
  std::vector<Eigen::Triplet<double>> A5_triplets_expected;
  A5_triplets_expected.emplace_back(7, 7, 1.0);
  CompareTriplets(dut.A_triplets()[5], A5_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  g_expected(5) = 0;
  // The seventh constraint is for x(6) == 1.
  EXPECT_EQ(dut.A_triplets()[6].size(), 0);
  B_triplets_expected.emplace_back(
      6,
      dut.map_prog_var_index_to_s_index().at(
          SdpaFreeFormat::DecisionVariableIndex(6)),
      1.0);
  g_expected(6) = 1.0;
  // Compare B_triplets and g.
  CompareTriplets(dut.B_triplets(), B_triplets_expected, 7,
                  dut.num_free_variables());
  EXPECT_TRUE(CompareMatrices(dut.g(), g_expected));

  // Check for the cost max -x(0) + x(1) - 2 *x(2) + 3 * x(3) + x(4)
  std::vector<Eigen::Triplet<double>> C_triplets_expected;
  C_triplets_expected.emplace_back(0, 0, -1);
  C_triplets_expected.emplace_back(1, 1, 1);
  CompareTriplets(dut.C_triplets(), C_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  std::vector<Eigen::Triplet<double>> d_triplets_expected;
  d_triplets_expected.emplace_back(
      dut.map_prog_var_index_to_s_index().at(
          SdpaFreeFormat::DecisionVariableIndex(2)),
      0, -2);
  d_triplets_expected.emplace_back(
      dut.map_prog_var_index_to_s_index().at(
          SdpaFreeFormat::DecisionVariableIndex(3)),
      0, 3);
  d_triplets_expected.emplace_back(
      dut.map_prog_var_index_to_s_index().at(
          SdpaFreeFormat::DecisionVariableIndex(4)),
      0, 1);
  CompareTriplets(dut.d_triplets(), d_triplets_expected,
                  dut.num_free_variables(), 1);
  EXPECT_EQ(dut.constant_min_cost_term(), -1);
}

TEST_F(LinearProgram2, TestSdpaFreeFormatConstructor) {
  // This tests adding linear constraint.
  const SdpaFreeFormat dut(*prog_);
  EXPECT_EQ(dut.num_X_rows(), 4);
  EXPECT_EQ(dut.num_free_variables(), 3);

  EXPECT_EQ(dut.X_blocks().size(), 1);
  CompareBlockInX(dut.X_blocks()[0], BlockInX(csdp::DIAG, 4));
  EXPECT_EQ(dut.map_prog_var_index_to_entry_in_X().size(), 0);
  EXPECT_EQ(dut.map_prog_var_index_to_s_index().size(), 3);
  EXPECT_EQ(dut.map_prog_var_index_to_s_index().at(
                SdpaFreeFormat::DecisionVariableIndex(0)),
            0);
  EXPECT_EQ(dut.map_prog_var_index_to_s_index().at(
                SdpaFreeFormat::DecisionVariableIndex(1)),
            1);
  EXPECT_EQ(dut.map_prog_var_index_to_s_index().at(
                SdpaFreeFormat::DecisionVariableIndex(2)),
            2);

  // Check the linear constraints.
  EXPECT_EQ(dut.A_triplets().size(), 6);
  std::vector<Eigen::Triplet<double>> B_triplets_expected;
  Vector6<double> g_expected;
  // 2 * x(0) + 3 * x(1) + x(2) = 1
  B_triplets_expected.emplace_back(0, 0, 2);
  B_triplets_expected.emplace_back(0, 1, 3);
  B_triplets_expected.emplace_back(0, 2, 1);
  g_expected(0) = 1;
  // -2 * x(2) + x(0) <= -1
  EXPECT_EQ(dut.A_triplets()[0].size(), 0);
  std::vector<Eigen::Triplet<double>> A1_triplets_expected;
  A1_triplets_expected.emplace_back(0, 0, 1);
  CompareTriplets(dut.A_triplets()[1], A1_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  B_triplets_expected.emplace_back(1, 0, 1);
  B_triplets_expected.emplace_back(1, 2, -2);
  g_expected(1) = -1;
  // 2 * x(1) + x(0) >= -2
  std::vector<Eigen::Triplet<double>> A2_triplets_expected;
  A2_triplets_expected.emplace_back(1, 1, -1);
  CompareTriplets(dut.A_triplets()[2], A2_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  B_triplets_expected.emplace_back(2, 0, 1);
  B_triplets_expected.emplace_back(2, 1, 2);
  g_expected(2) = -2;
  // -2 <= -x(0) + 3 * x(2) <= 3
  std::vector<Eigen::Triplet<double>> A3_triplets_expected;
  A3_triplets_expected.emplace_back(2, 2, -1);
  CompareTriplets(dut.A_triplets()[3], A3_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  B_triplets_expected.emplace_back(3, 0, -1);
  B_triplets_expected.emplace_back(3, 2, 3);
  g_expected(3) = -2;
  std::vector<Eigen::Triplet<double>> A4_triplets_expected;
  A4_triplets_expected.emplace_back(3, 3, 1);
  CompareTriplets(dut.A_triplets()[4], A4_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  B_triplets_expected.emplace_back(4, 0, -1);
  B_triplets_expected.emplace_back(4, 2, 3);
  g_expected(4) = 3;
  // x(0) + x(1) + 4 * x(2) = 3
  EXPECT_EQ(dut.A_triplets()[5].size(), 0);
  B_triplets_expected.emplace_back(5, 0, 1);
  B_triplets_expected.emplace_back(5, 1, 1);
  B_triplets_expected.emplace_back(5, 2, 4);
  g_expected(5) = 3;

  CompareTriplets(dut.B_triplets(), B_triplets_expected, 6,
                  dut.num_free_variables());
  EXPECT_TRUE(CompareMatrices(dut.g(), g_expected));

  // Check the cost min x(0) + 2 * x(1) +  3 * x(2)
  EXPECT_EQ(dut.C_triplets().size(), 0);
  std::vector<Eigen::Triplet<double>> d_triplets_expected;
  d_triplets_expected.emplace_back(0, 0, -1);
  d_triplets_expected.emplace_back(1, 0, -2);
  d_triplets_expected.emplace_back(2, 0, -3);
  CompareTriplets(dut.d_triplets(), d_triplets_expected, 3, 1);
}

TEST_F(TrivialSDP1, TestSdpaFreeFormatConstructor) {
  // Test SdpaFreeFormat constructor with both PSD constraint and linear
  // constraint.
  const SdpaFreeFormat dut(*prog_);
  EXPECT_EQ(dut.num_X_rows(), 4);
  EXPECT_EQ(dut.num_free_variables(), 0);

  EXPECT_EQ(dut.X_blocks().size(), 2);
  CompareBlockInX(dut.X_blocks()[0], BlockInX(csdp::MATRIX, 3));
  CompareBlockInX(dut.X_blocks()[1], BlockInX(csdp::DIAG, 1));
  CompareEntryInX(dut.map_prog_var_index_to_entry_in_X().at(0),
                  EntryInX(0, 0, 0, 0));
  CompareEntryInX(dut.map_prog_var_index_to_entry_in_X().at(1),
                  EntryInX(0, 0, 1, 0));
  CompareEntryInX(dut.map_prog_var_index_to_entry_in_X().at(2),
                  EntryInX(0, 0, 2, 0));
  CompareEntryInX(dut.map_prog_var_index_to_entry_in_X().at(3),
                  EntryInX(0, 1, 1, 0));
  CompareEntryInX(dut.map_prog_var_index_to_entry_in_X().at(4),
                  EntryInX(0, 1, 2, 0));
  CompareEntryInX(dut.map_prog_var_index_to_entry_in_X().at(5),
                  EntryInX(0, 2, 2, 0));
  EXPECT_EQ(dut.map_prog_var_index_to_s_index().size(), 0);

  EXPECT_EQ(dut.A_triplets().size(), 2);
  Eigen::Vector2d g_expected(1, 0);
  // X1(0, 0) + X1(1, 1) + X1(2, 2) = 1
  std::vector<Eigen::Triplet<double>> A0_triplets_expected;
  A0_triplets_expected.emplace_back(0, 0, 1);
  A0_triplets_expected.emplace_back(1, 1, 1);
  A0_triplets_expected.emplace_back(2, 2, 1);
  CompareTriplets(dut.A_triplets()[0], A0_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());

  // X1(0, 1) + X1(1, 2) - 2 * X1(0, 2) <= 0
  std::vector<Eigen::Triplet<double>> A1_triplets_expected;
  A1_triplets_expected.emplace_back(0, 1, 0.5);
  A1_triplets_expected.emplace_back(1, 0, 0.5);
  A1_triplets_expected.emplace_back(1, 2, 0.5);
  A1_triplets_expected.emplace_back(2, 1, 0.5);
  A1_triplets_expected.emplace_back(0, 2, -1);
  A1_triplets_expected.emplace_back(2, 0, -1);
  A1_triplets_expected.emplace_back(3, 3, 1);
  CompareTriplets(dut.A_triplets()[1], A1_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  EXPECT_TRUE(CompareMatrices(dut.g(), g_expected));
  EXPECT_EQ(dut.B_triplets().size(), 0);

  // Cost max X1(0, 1) + X1(1, 2)
  std::vector<Eigen::Triplet<double>> C_triplets_expected;
  C_triplets_expected.emplace_back(0, 1, 0.5);
  C_triplets_expected.emplace_back(1, 0, 0.5);
  C_triplets_expected.emplace_back(1, 2, 0.5);
  C_triplets_expected.emplace_back(2, 1, 0.5);
  CompareTriplets(dut.C_triplets(), C_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  EXPECT_EQ(dut.d_triplets().size(), 0);
}

TEST_F(TrivialSDP2, TestSdpaFreeFormatConstructor) {
  // Test constructor with linear matrix inequality constraint.
  const SdpaFreeFormat dut(*prog_);
  EXPECT_EQ(dut.num_X_rows(), 4);
  EXPECT_EQ(dut.num_free_variables(), 1);

  EXPECT_EQ(dut.X_blocks().size(), 2);
  CompareBlockInX(dut.X_blocks()[0], BlockInX(csdp::MATRIX, 2));
  CompareBlockInX(dut.X_blocks()[1], BlockInX(csdp::MATRIX, 2));
  CompareEntryInX(dut.map_prog_var_index_to_entry_in_X().at(
                      prog_->FindDecisionVariableIndex(X1_(0, 0))),
                  EntryInX(0, 0, 0, 0));
  CompareEntryInX(dut.map_prog_var_index_to_entry_in_X().at(
                      prog_->FindDecisionVariableIndex(X1_(0, 1))),
                  EntryInX(0, 0, 1, 0));
  CompareEntryInX(dut.map_prog_var_index_to_entry_in_X().at(
                      prog_->FindDecisionVariableIndex(X1_(1, 1))),
                  EntryInX(0, 1, 1, 0));
  EXPECT_EQ(dut.map_prog_var_index_to_s_index().at(
                SdpaFreeFormat::DecisionVariableIndex(
                    prog_->FindDecisionVariableIndex(y_))),
            0);

  // Check linear constraint
  EXPECT_EQ(dut.A_triplets().size(), 4);
  std::vector<Eigen::Triplet<double>> B_triplets_expected;
  Eigen::Vector4d g_expected;
  // X1(0, 0) + 2 * X1(1, 1) + 3 * y = 1
  std::vector<Eigen::Triplet<double>> A0_triplets_expected;
  A0_triplets_expected.emplace_back(0, 0, 1);
  A0_triplets_expected.emplace_back(1, 1, 2);
  CompareTriplets(dut.A_triplets()[0], A0_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  B_triplets_expected.emplace_back(0, 0, 3);
  g_expected(0) = 1;

  // I + F1 * y + F2 * X1(0, 0) is psd.
  // where F1 = [1 2; 2 3], F2 = [2 0; 0 4]
  // 1 + y + 2 * X1(0, 0) = X_slack(0, 0)
  std::vector<Eigen::Triplet<double>> A1_triplets_expected;
  A1_triplets_expected.emplace_back(0, 0, 2);
  A1_triplets_expected.emplace_back(2, 2, -1);
  CompareTriplets(dut.A_triplets()[1], A1_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  B_triplets_expected.emplace_back(1, 0, 1);
  g_expected(1) = -1;

  // 2*y = X_slack(0, 1)
  std::vector<Eigen::Triplet<double>> A2_triplets_expected;
  A2_triplets_expected.emplace_back(2, 3, -0.5);
  A2_triplets_expected.emplace_back(3, 2, -0.5);
  CompareTriplets(dut.A_triplets()[2], A2_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  B_triplets_expected.emplace_back(2, 0, 2);
  g_expected(2) = 0;

  // 1 + 3*y + 4*X1_(0, 0) = X_slack(1, 1)
  std::vector<Eigen::Triplet<double>> A3_triplets_expected;
  A3_triplets_expected.emplace_back(0, 0, 4);
  A3_triplets_expected.emplace_back(3, 3, -1);
  CompareTriplets(dut.A_triplets()[3], A3_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  B_triplets_expected.emplace_back(3, 0, 3);
  g_expected(3) = -1;
  CompareTriplets(dut.B_triplets(), B_triplets_expected, 4, 1);
  EXPECT_TRUE(CompareMatrices(dut.g(), g_expected));

  // Check the cost max y
  EXPECT_EQ(dut.C_triplets().size(), 0);
  std::vector<Eigen::Triplet<double>> d_triplets_expected;
  d_triplets_expected.emplace_back(0, 0, 1);
  CompareTriplets(dut.d_triplets(), d_triplets_expected, 1, 1);
}

TEST_F(SDPwithOverlappingVariables,
       GenerateCsdpProblemDataWithoutFreeVariables) {
  const SdpaFreeFormat dut(*prog_);
  struct csdp::blockmatrix C_csdp;
  double* rhs_csdp;
  struct csdp::constraintmatrix* constraints{nullptr};
  dut.GenerateCsdpProblemDataWithoutFreeVariables(&C_csdp, &rhs_csdp,
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
  //     [x0 x2] is psd
  //     [x2 x0]
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
  dut.GenerateCsdpProblemDataWithoutFreeVariables(&C_csdp, &rhs_csdp,
                                                  &constraints);

  // Check the cost
  EXPECT_EQ(C_csdp.nblocks, 3);
  CompareBlockrec(C_csdp.blocks[1], csdp::MATRIX, 2, {2, 1, 1, 2}, 0);
  CompareBlockrec(C_csdp.blocks[2], csdp::MATRIX, 3,
                  {3, 0, 1, 0, 2, 0, 1, 0, 3}, 0);
  CompareBlockrec(C_csdp.blocks[3], csdp::DIAG, 2, {0, 0}, 0);

  // Check constraints.
  // constraint 1
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

  // constraint 2.
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
  dut.GenerateCsdpProblemDataWithoutFreeVariables(&C_csdp, &rhs_csdp,
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
  dut.GenerateCsdpProblemDataWithoutFreeVariables(&C, &rhs, &constraints);

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
  dut.GenerateCsdpProblemDataWithoutFreeVariables(&C, &rhs, &constraints);

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
  dut.GenerateCsdpProblemDataWithoutFreeVariables(&C, &rhs, &constraints);

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
}  // namespace internal
}  // namespace solvers
}  // namespace drake
