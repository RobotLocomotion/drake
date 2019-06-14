#include "drake/solvers/sdpa_free_format.h"

#include <fstream>
#include <limits>
#include <utility>

#include <gtest/gtest.h>
#include <spruce.hh>

#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/test/csdp_test_examples.h"

namespace drake {
namespace solvers {
const double kInf = std::numeric_limits<double>::infinity();
namespace internal {

void CompareProgVarInSdpa(const SdpaFreeFormat& sdpa_free_format,
                          int variable_index, double val_expected) {
  const double val =
      get<double>(sdpa_free_format.prog_var_in_sdpa()[variable_index]);
  EXPECT_EQ(val, val_expected);
}

void CompareProgVarInSdpa(const SdpaFreeFormat& sdpa_free_format,
                          int variable_index,
                          SdpaFreeFormat::FreeVariableIndex s_index_expected) {
  const SdpaFreeFormat::FreeVariableIndex s_index =
      get<SdpaFreeFormat::FreeVariableIndex>(
          sdpa_free_format.prog_var_in_sdpa()[variable_index]);
  EXPECT_EQ(s_index, s_index_expected);
}

void CompareProgVarInSdpa(const SdpaFreeFormat& sdpa_free_format,
                          int variable_index,
                          const DecisionVariableInSdpaX& val_expected) {
  const auto val = get<DecisionVariableInSdpaX>(
      sdpa_free_format.prog_var_in_sdpa()[variable_index]);
  EXPECT_EQ(val.coeff_sign, val_expected.coeff_sign);
  EXPECT_EQ(val.offset, val_expected.offset);
  EXPECT_EQ(val.entry_in_X.block_index, val_expected.entry_in_X.block_index);
  EXPECT_EQ(val.entry_in_X.row_index_in_block,
            val_expected.entry_in_X.row_index_in_block);
  EXPECT_EQ(val.entry_in_X.column_index_in_block,
            val_expected.entry_in_X.column_index_in_block);
  EXPECT_EQ(val.entry_in_X.X_start_row, val_expected.entry_in_X.X_start_row);
}

void CompareBlockInX(const BlockInX& block1, const BlockInX& block2) {
  EXPECT_EQ(block1.block_type, block2.block_type);
  EXPECT_EQ(block1.num_rows, block2.num_rows);
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

TEST_F(SDPwithOverlappingVariables, TestSdpaFreeFormatConstructor) {
  const SdpaFreeFormat dut(*prog_);
  ASSERT_EQ(dut.num_X_rows(), 4);
  ASSERT_EQ(dut.num_free_variables(), 0);
  ASSERT_EQ(dut.X_blocks().size(), 2);
  CompareBlockInX(dut.X_blocks()[0], BlockInX(BlockType::kMatrix, 2));
  CompareBlockInX(dut.X_blocks()[1], BlockInX(BlockType::kMatrix, 2));
  ASSERT_EQ(dut.prog_var_in_sdpa().size(), 3);
  CompareProgVarInSdpa(dut, prog_->FindDecisionVariableIndex(x_(0)),
                       DecisionVariableInSdpaX(Sign::kPositive, 0, 0, 0, 0, 0));
  CompareProgVarInSdpa(dut, prog_->FindDecisionVariableIndex(x_(1)),
                       DecisionVariableInSdpaX(Sign::kPositive, 0, 0, 0, 1, 0));
  CompareProgVarInSdpa(dut, prog_->FindDecisionVariableIndex(x_(2)),
                       DecisionVariableInSdpaX(Sign::kPositive, 0, 1, 0, 1, 2));

  // Check A_triplets.
  ASSERT_EQ(dut.A_triplets().size(), 4);
  CompareTriplets(
      dut.A_triplets()[0],
      {Eigen::Triplet<double>(0, 0, 1.0), Eigen::Triplet<double>(1, 1, -1.0)},
      dut.num_X_rows(), dut.num_X_rows());
  CompareTriplets(
      dut.A_triplets()[1],
      {Eigen::Triplet<double>(0, 0, 1.0), Eigen::Triplet<double>(2, 2, -1.0)},
      dut.num_X_rows(), dut.num_X_rows());
  CompareTriplets(
      dut.A_triplets()[2],
      {Eigen::Triplet<double>(0, 0, 1.0), Eigen::Triplet<double>(3, 3, -1.0)},
      dut.num_X_rows(), dut.num_X_rows());
  // The constraint that x1 = 1
  CompareTriplets(
      dut.A_triplets()[3],
      {Eigen::Triplet<double>(0, 1, 0.5), Eigen::Triplet<double>(1, 0, 0.5)},
      dut.num_X_rows(), dut.num_X_rows());

  EXPECT_TRUE(CompareMatrices(dut.g(), Eigen::Vector4d(0, 0, 0, 1)));
  EXPECT_EQ(dut.B_triplets().size(), 0);

  // Now check the cost.
  CompareTriplets(
      dut.C_triplets(),
      {Eigen::Triplet<double>(0, 0, -2), Eigen::Triplet<double>(2, 3, -0.5),
       Eigen::Triplet<double>(3, 2, -0.5)},
      dut.num_X_rows(), dut.num_X_rows());
  EXPECT_EQ(dut.d_triplets().size(), 0);
  EXPECT_EQ(dut.constant_min_cost_term(), 0);
}

TEST_F(CsdpDocExample, TestSdpaFreeFormatConstructor) {
  SdpaFreeFormat dut(*prog_);
  EXPECT_EQ(dut.num_X_rows(), 7);
  EXPECT_EQ(dut.num_free_variables(), 0);
  EXPECT_EQ(dut.X_blocks().size(), 3);
  CompareBlockInX(dut.X_blocks()[0], BlockInX(BlockType::kMatrix, 2));
  CompareBlockInX(dut.X_blocks()[1], BlockInX(BlockType::kMatrix, 3));
  CompareBlockInX(dut.X_blocks()[2], BlockInX(BlockType::kDiagonal, 2));
  EXPECT_EQ(dut.prog_var_in_sdpa().size(), prog_->num_vars());
  CompareProgVarInSdpa(dut, prog_->FindDecisionVariableIndex(X1_(0, 0)),
                       DecisionVariableInSdpaX(Sign::kPositive, 0, 0, 0, 0, 0));
  CompareProgVarInSdpa(dut, prog_->FindDecisionVariableIndex(X1_(1, 0)),
                       DecisionVariableInSdpaX(Sign::kPositive, 0, 0, 0, 1, 0));
  CompareProgVarInSdpa(dut, prog_->FindDecisionVariableIndex(X1_(1, 1)),
                       DecisionVariableInSdpaX(Sign::kPositive, 0, 0, 1, 1, 0));
  for (int j = 0; j < 3; ++j) {
    for (int i = 0; i <= j; ++i) {
      CompareProgVarInSdpa(
          dut, prog_->FindDecisionVariableIndex(X2_(i, j)),
          DecisionVariableInSdpaX(Sign::kPositive, 0, 1, i, j, 2));
    }
  }
  for (int i = 0; i < 2; ++i) {
    CompareProgVarInSdpa(
        dut, prog_->FindDecisionVariableIndex(y_(i)),
        DecisionVariableInSdpaX(Sign::kPositive, 0, 2, i, i, 5));
  }

  // Check the cost.
  CompareTriplets(
      dut.C_triplets(),
      {Eigen::Triplet<double>(0, 0, 2), Eigen::Triplet<double>(0, 1, 1),
       Eigen::Triplet<double>(1, 0, 1), Eigen::Triplet<double>(1, 1, 2),
       Eigen::Triplet<double>(2, 2, 3), Eigen::Triplet<double>(3, 3, 2),
       Eigen::Triplet<double>(2, 4, 1), Eigen::Triplet<double>(4, 2, 1),
       Eigen::Triplet<double>(4, 4, 3)},
      dut.num_X_rows(), dut.num_X_rows());
  EXPECT_EQ(dut.d_triplets().size(), 0);
  EXPECT_EQ(dut.constant_min_cost_term(), 0);

  // Check the constraints.
  EXPECT_EQ(dut.A_triplets().size(), 2);
  CompareTriplets(
      dut.A_triplets()[0],
      {Eigen::Triplet<double>(0, 0, 3), Eigen::Triplet<double>(0, 1, 1),
       Eigen::Triplet<double>(1, 0, 1), Eigen::Triplet<double>(1, 1, 3),
       Eigen::Triplet<double>(5, 5, 1)},
      dut.num_X_rows(), dut.num_X_rows());
  CompareTriplets(
      dut.A_triplets()[1],
      {Eigen::Triplet<double>(2, 2, 3), Eigen::Triplet<double>(3, 3, 4),
       Eigen::Triplet<double>(2, 4, 1), Eigen::Triplet<double>(4, 2, 1),
       Eigen::Triplet<double>(4, 4, 5), Eigen::Triplet<double>(6, 6, 1)},
      dut.num_X_rows(), dut.num_X_rows());
  EXPECT_EQ(dut.B_triplets().size(), 0);
  EXPECT_TRUE(CompareMatrices(dut.g(), Eigen::Vector2d(1, 2)));
}

TEST_F(LinearProgramBoundingBox1, TestSdpaFreeFormatConstructor) {
  // Test if we can correctly register decision variables with bounding box
  // constraints.
  SdpaFreeFormat dut(*prog_);
  EXPECT_EQ(dut.num_X_rows(), 7);
  EXPECT_EQ(dut.num_free_variables(), 1);

  EXPECT_EQ(dut.X_blocks().size(), 1);
  CompareBlockInX(dut.X_blocks()[0], BlockInX(BlockType::kDiagonal, 7));
  EXPECT_EQ(dut.prog_var_in_sdpa().size(), prog_->num_vars());
  CompareProgVarInSdpa(dut, 0,
                       DecisionVariableInSdpaX(Sign::kPositive, 0, 0, 0, 0, 0));
  CompareProgVarInSdpa(dut, 1,
                       DecisionVariableInSdpaX(Sign::kPositive, 0, 0, 1, 1, 0));
  CompareProgVarInSdpa(
      dut, 2, DecisionVariableInSdpaX(Sign::kPositive, -1, 0, 3, 3, 0));
  CompareProgVarInSdpa(
      dut, 3, DecisionVariableInSdpaX(Sign::kNegative, 10, 0, 4, 4, 0));
  CompareProgVarInSdpa(
      dut, 4, DecisionVariableInSdpaX(Sign::kPositive, -2, 0, 5, 5, 0));
  CompareProgVarInSdpa(dut, 5, 0.0);
  CompareProgVarInSdpa(dut, 6, 1.0);
  CompareProgVarInSdpa(dut, 7, SdpaFreeFormat::FreeVariableIndex(0));

  // Constraining 0 <= x(1) <= 5
  Eigen::Vector2d g_expected;
  CompareTriplets(
      dut.A_triplets()[0],
      {Eigen::Triplet<double>(1, 1, 1), Eigen::Triplet<double>(2, 2, 1)},
      dut.num_X_rows(), dut.num_X_rows());
  g_expected(0) = 5;
  // Constraining -2 <= x(4) <= 5
  CompareTriplets(
      dut.A_triplets()[1],
      {Eigen::Triplet<double>(5, 5, 1), Eigen::Triplet<double>(6, 6, 1)},
      dut.num_X_rows(), dut.num_X_rows());
  g_expected(1) = 7;
  EXPECT_EQ(dut.B_triplets().size(), 0);
  EXPECT_TRUE(CompareMatrices(dut.g(), g_expected));

  // Check the cost max -x(0) + x(1) - 2 * x(2) + 3 * x(3) +  x(4) + 1
  CompareTriplets(
      dut.C_triplets(),
      {Eigen::Triplet<double>(0, 0, -1), Eigen::Triplet<double>(1, 1, 1),
       Eigen::Triplet<double>(3, 3, -2), Eigen::Triplet<double>(4, 4, -3),
       Eigen::Triplet<double>(5, 5, 1)},
      dut.num_X_rows(), dut.num_X_rows());
  EXPECT_EQ(dut.constant_min_cost_term(), -31);
}

TEST_F(CsdpLinearProgram2, TestSdpaFreeFormatConstructor) {
  // This tests adding linear constraint.
  const SdpaFreeFormat dut(*prog_);
  EXPECT_EQ(dut.num_X_rows(), 4);
  EXPECT_EQ(dut.num_free_variables(), 3);
  EXPECT_EQ(dut.prog_var_in_sdpa().size(), 3);
  for (int i = 0; i < 3; ++i) {
    CompareProgVarInSdpa(dut, i, SdpaFreeFormat::FreeVariableIndex(i));
  }

  EXPECT_EQ(dut.X_blocks().size(), 1);
  CompareBlockInX(dut.X_blocks()[0], BlockInX(BlockType::kDiagonal, 4));

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

TEST_F(CsdpLinearProgram3, TestSdpaFreeFormatConstructor) {
  const SdpaFreeFormat dut(*prog_);
  EXPECT_EQ(dut.num_X_rows(), 7);
  EXPECT_EQ(dut.num_free_variables(), 1);
  EXPECT_EQ(dut.prog_var_in_sdpa().size(), 3);
  CompareProgVarInSdpa(
      dut, 0, DecisionVariableInSdpaX(Sign::kPositive, -1, 0, 0, 0, 0));
  CompareProgVarInSdpa(dut, 1,
                       DecisionVariableInSdpaX(Sign::kNegative, 8, 0, 2, 2, 0));
  CompareProgVarInSdpa(dut, 2, SdpaFreeFormat::FreeVariableIndex(0));

  EXPECT_EQ(dut.X_blocks().size(), 2);

  CompareBlockInX(dut.X_blocks()[0], BlockInX(BlockType::kDiagonal, 3));
  CompareBlockInX(dut.X_blocks()[1], BlockInX(BlockType::kDiagonal, 4));

  // Check the cost 2x(0) + 3x(1) + 4x(2) + 3
  CompareTriplets(
      dut.C_triplets(),
      {Eigen::Triplet<double>(0, 0, 2), Eigen::Triplet<double>(2, 2, -3)},
      dut.num_X_rows(), dut.num_X_rows());
  CompareTriplets(dut.d_triplets(), {Eigen::Triplet<double>(0, 0, 4)}, 1, 1);
  EXPECT_EQ(dut.constant_min_cost_term(), -25);
  EXPECT_EQ(dut.A_triplets().size(), 6);
  Vector6<double> g_expected;
  std::vector<Eigen::Triplet<double>> B_triplets_expected;
  // Check the constraint x(0) <= 10
  CompareTriplets(
      dut.A_triplets()[0],
      {Eigen::Triplet<double>(0, 0, 1), Eigen::Triplet<double>(1, 1, 1)},
      dut.num_X_rows(), dut.num_X_rows());
  g_expected(0) = 11;

  // Check the constraint x(0) + 2x(1) + 3x(2) = 3
  CompareTriplets(
      dut.A_triplets()[1],
      {Eigen::Triplet<double>(0, 0, 1), Eigen::Triplet<double>(2, 2, -2)},
      dut.num_X_rows(), dut.num_X_rows());
  B_triplets_expected.emplace_back(1, 0, 3);
  g_expected(1) = -12;

  // Check the constraint 2x(0) - x(2)>= -1
  CompareTriplets(
      dut.A_triplets()[2],
      {Eigen::Triplet<double>(0, 0, 2), Eigen::Triplet<double>(3, 3, -1)},
      dut.num_X_rows(), dut.num_X_rows());
  B_triplets_expected.emplace_back(2, 0, -1);
  g_expected(2) = 1;

  // Check the constraint x(1) - 3x(2) <= 5
  CompareTriplets(
      dut.A_triplets()[3],
      {Eigen::Triplet<double>(2, 2, -1), Eigen::Triplet<double>(4, 4, 1)},
      dut.num_X_rows(), dut.num_X_rows());
  B_triplets_expected.emplace_back(3, 0, -3);
  g_expected(3) = -3;

  // Check the constraint -4 <= x(0) + x(2) <= 9
  CompareTriplets(
      dut.A_triplets()[4],
      {Eigen::Triplet<double>(0, 0, 1), Eigen::Triplet<double>(5, 5, -1)},
      dut.num_X_rows(), dut.num_X_rows());
  B_triplets_expected.emplace_back(4, 0, 1);
  g_expected(4) = -3;

  CompareTriplets(
      dut.A_triplets()[5],
      {Eigen::Triplet<double>(0, 0, 1), Eigen::Triplet<double>(6, 6, 1)},
      dut.num_X_rows(), dut.num_X_rows());
  B_triplets_expected.emplace_back(5, 0, 1);
  g_expected(5) = 10;
  CompareTriplets(dut.B_triplets(), B_triplets_expected, dut.g().rows(),
                  dut.num_free_variables());
  EXPECT_TRUE(CompareMatrices(dut.g(), g_expected));
}

TEST_F(TrivialSDP1, TestSdpaFreeFormatConstructor) {
  // Test SdpaFreeFormat constructor with both PSD constraint and linear
  // constraint.
  const SdpaFreeFormat dut(*prog_);
  EXPECT_EQ(dut.num_X_rows(), 4);
  EXPECT_EQ(dut.num_free_variables(), 0);

  EXPECT_EQ(dut.X_blocks().size(), 2);
  CompareBlockInX(dut.X_blocks()[0], BlockInX(BlockType::kMatrix, 3));
  CompareBlockInX(dut.X_blocks()[1], BlockInX(BlockType::kDiagonal, 1));
  for (int j = 0; j < 3; ++j) {
    for (int i = 0; i <= j; ++i) {
      CompareProgVarInSdpa(
          dut, prog_->FindDecisionVariableIndex(X1_(i, j)),
          DecisionVariableInSdpaX(Sign::kPositive, 0, 0, i, j, 0));
    }
  }

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
  CompareBlockInX(dut.X_blocks()[0], BlockInX(BlockType::kMatrix, 2));
  CompareBlockInX(dut.X_blocks()[1], BlockInX(BlockType::kMatrix, 2));
  for (int j = 0; j < 2; ++j) {
    for (int i = 0; i <= j; ++i) {
      CompareProgVarInSdpa(
          dut, prog_->FindDecisionVariableIndex(X1_(i, j)),
          DecisionVariableInSdpaX(Sign::kPositive, 0, 0, i, j, 0));
    }
  }
  CompareProgVarInSdpa(dut, prog_->FindDecisionVariableIndex(y_),
                       SdpaFreeFormat::FreeVariableIndex(0));

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

TEST_F(TrivialSOCP1, TestSdpaFreeFormatConstructor) {
  const SdpaFreeFormat dut(*prog_);
  EXPECT_EQ(dut.num_X_rows(), 5);
  EXPECT_EQ(dut.num_free_variables(), 1);

  EXPECT_EQ(dut.X_blocks().size(), 2);
  CompareBlockInX(dut.X_blocks()[0], BlockInX(BlockType::kDiagonal, 2));
  CompareBlockInX(dut.X_blocks()[1], BlockInX(BlockType::kMatrix, 3));

  CompareProgVarInSdpa(dut, 0, SdpaFreeFormat::FreeVariableIndex(0));
  for (int i = 0; i < 2; ++i) {
    CompareProgVarInSdpa(
        dut, i + 1, DecisionVariableInSdpaX(Sign::kPositive, 0, 0, i, i, 0));
  }

  // Check the cost max x(0).
  EXPECT_EQ(dut.C_triplets().size(), 0);
  EXPECT_TRUE(CompareMatrices(Eigen::VectorXd(dut.d()), Vector1d(1)));

  EXPECT_EQ(dut.A_triplets().size(), 7);
  std::vector<Eigen::Triplet<double>> B_triplets_expected;
  Eigen::Matrix<double, 7, 1> g_expected;
  // Check the first constraint x(0) + x(1) + x(2) = 10
  std::vector<Eigen::Triplet<double>> A0_triplets_expected;
  A0_triplets_expected.emplace_back(0, 0, 1);
  A0_triplets_expected.emplace_back(1, 1, 1);
  CompareTriplets(dut.A_triplets()[0], A0_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  B_triplets_expected.emplace_back(0, 0, 1);
  g_expected(0) = 10;

  // The equality constraints arising from the Lorentz cone. 2x(0) + 1 ==
  // X_blocks()[1](i, i) for i = 0, 1, 2.
  std::vector<Eigen::Triplet<double>> A_diagonal_triplets_expected;
  for (int i = 0; i < 3; ++i) {
    if (!A_diagonal_triplets_expected.empty()) {
      A_diagonal_triplets_expected.pop_back();
    }
    A_diagonal_triplets_expected.emplace_back(2 + i, 2 + i, -1);
    CompareTriplets(dut.A_triplets()[1 + i], A_diagonal_triplets_expected,
                    dut.num_X_rows(), dut.num_X_rows());
    B_triplets_expected.emplace_back(1 + i, 0, 2);
    g_expected(1 + i) = -1;
  }

  // The equality constraint arising from the Lorentz cone. 3x(1) + 2 =
  // X_blocks()[1](0, 1).
  std::vector<Eigen::Triplet<double>> A4_triplets_expected;
  A4_triplets_expected.emplace_back(0, 0, 3);
  A4_triplets_expected.emplace_back(2, 3, -0.5);
  A4_triplets_expected.emplace_back(3, 2, -0.5);
  CompareTriplets(dut.A_triplets()[4], A4_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  g_expected(4) = -2;

  // The equality constraint arising from the Lorentz cone. x(0) + x(2) + 3 =
  // X_blocks()[1](0, 2).
  std::vector<Eigen::Triplet<double>> A5_triplets_expected;
  A5_triplets_expected.emplace_back(1, 1, 1);
  A5_triplets_expected.emplace_back(2, 4, -0.5);
  A5_triplets_expected.emplace_back(4, 2, -0.5);
  CompareTriplets(dut.A_triplets()[5], A5_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  B_triplets_expected.emplace_back(5, 0, 1);
  g_expected(5) = -3;

  // The equality constraint arising from the Lorentz cone X_blocks()[1](1, 2) =
  // 0.
  std::vector<Eigen::Triplet<double>> A6_triplets_expected;
  A6_triplets_expected.emplace_back(3, 4, 0.5);
  A6_triplets_expected.emplace_back(4, 3, 0.5);
  CompareTriplets(dut.A_triplets()[6], A6_triplets_expected, dut.num_X_rows(),
                  dut.num_X_rows());
  g_expected(6) = 0;

  CompareTriplets(dut.B_triplets(), B_triplets_expected, dut.g().rows(),
                  dut.num_free_variables());
  EXPECT_TRUE(CompareMatrices(dut.g(), g_expected));
}

TEST_F(TrivialSOCP2, TestSdpaFreeFormatConstructor) {
  const SdpaFreeFormat dut(*prog_);
  EXPECT_EQ(dut.num_X_rows(), 3);
  EXPECT_EQ(dut.num_free_variables(), 2);

  EXPECT_EQ(dut.X_blocks().size(), 1);
  CompareBlockInX(dut.X_blocks()[0], BlockInX(BlockType::kMatrix, 3));
  for (int i = 0; i < 2; ++i) {
    CompareProgVarInSdpa(dut, i, SdpaFreeFormat::FreeVariableIndex(i));
  }

  // Check the cost.
  EXPECT_EQ(dut.C_triplets().size(), 0);
  CompareTriplets(dut.d_triplets(), {Eigen::Triplet<double>(1, 0, 1)}, 2, 1);

  EXPECT_EQ(dut.A_triplets().size(), 6);
  std::vector<Eigen::Triplet<double>> B_triplets_expected;
  Vector6<double> g_expected;
  // Check the linear equality constraint arising from Lorentz cone. X(i, i) =
  // x(0) + 2 for i = 0, 1, 2.
  for (int i = 0; i < 3; ++i) {
    CompareTriplets(dut.A_triplets()[i], {Eigen::Triplet<double>(i, i, -1)},
                    dut.num_X_rows(), dut.num_X_rows());
    B_triplets_expected.emplace_back(i, 0, 1);
    g_expected(i) = -2;
  }
  // Check the linear equality constraint arising from Lorentz cone, X(0, 1) =
  // x(0) + x(1) + 1.
  CompareTriplets(
      dut.A_triplets()[3],
      {Eigen::Triplet<double>(0, 1, -0.5), Eigen::Triplet<double>(1, 0, -0.5)},
      dut.num_X_rows(), dut.num_X_rows());
  B_triplets_expected.emplace_back(3, 0, 1);
  B_triplets_expected.emplace_back(3, 1, 1);
  g_expected(3) = -1;
  // Check the linear equality constraint arising from Lorentz cone, X(0, 2) =
  // x(0) - x(1) + 1.
  CompareTriplets(
      dut.A_triplets()[4],
      {Eigen::Triplet<double>(0, 2, -0.5), Eigen::Triplet<double>(2, 0, -0.5)},
      dut.num_X_rows(), dut.num_X_rows());
  B_triplets_expected.emplace_back(4, 0, 1);
  B_triplets_expected.emplace_back(4, 1, -1);
  g_expected(4) = -1;
  // Check the linear equality constraint arising from Lorentz cone, X(1, 2) =
  // 0.
  CompareTriplets(
      dut.A_triplets()[5],
      {Eigen::Triplet<double>(1, 2, 0.5), Eigen::Triplet<double>(2, 1, 0.5)},
      dut.num_X_rows(), dut.num_X_rows());
  g_expected(5) = 0;

  CompareTriplets(dut.B_triplets(), B_triplets_expected, dut.g().rows(),
                  dut.num_free_variables());
  EXPECT_TRUE(CompareMatrices(dut.g(), g_expected));
}

TEST_F(TrivialSOCP3, TestSdpaFreeFormatConstructor) {
  const SdpaFreeFormat dut(*prog_);
  EXPECT_EQ(dut.num_X_rows(), 3);
  EXPECT_EQ(dut.num_free_variables(), 2);

  EXPECT_EQ(dut.X_blocks().size(), 1);
  CompareBlockInX(dut.X_blocks()[0], BlockInX(BlockType::kMatrix, 3));

  for (int i = 0; i < 2; ++i) {
    CompareProgVarInSdpa(dut, i, SdpaFreeFormat::FreeVariableIndex(i));
  }

  // Check the cost max -x(1).
  EXPECT_EQ(dut.C_triplets().size(), 0);
  CompareTriplets(dut.d_triplets(), {Eigen::Triplet<double>(1, 0, -1)}, 2, 1);

  EXPECT_EQ(dut.A_triplets().size(), 6);
  std::vector<Eigen::Triplet<double>> B_triplets_expected;
  Vector6<double> g_expected;
  // Check the equality constraint arising from rotated Lorentz cone
  // 2x(0) + 2 = X(0, 0).
  CompareTriplets(dut.A_triplets()[0], {Eigen::Triplet<double>(0, 0, -1)},
                  dut.num_X_rows(), dut.num_X_rows());
  B_triplets_expected.emplace_back(0, 0, 2);
  g_expected(0) = -2;

  // Check the equality constraint arising from rotated Lorentz cone
  // x(0) + 2 = X(0, 1).
  CompareTriplets(
      dut.A_triplets()[1],
      {Eigen::Triplet<double>(0, 1, -0.5), Eigen::Triplet<double>(1, 0, -0.5)},
      dut.num_X_rows(), dut.num_X_rows());
  B_triplets_expected.emplace_back(1, 0, 1);
  g_expected(1) = -2;

  // Check the equality constraint arising from rotated Lorentz cone
  // 3x(0) + x(1) + 1 = X(0, 2).
  CompareTriplets(
      dut.A_triplets()[2],
      {Eigen::Triplet<double>(0, 2, -0.5), Eigen::Triplet<double>(2, 0, -0.5)},
      dut.num_X_rows(), dut.num_X_rows());
  B_triplets_expected.emplace_back(2, 0, 3);
  B_triplets_expected.emplace_back(2, 1, 1);
  g_expected(2) = -1;

  // Check the equality constraint arising from rotated Lorentz cone
  // 3x(1) + 4 = X(1, 1) and 3x(1) + 4 = X(2, 2).
  for (int i = 0; i < 2; ++i) {
    CompareTriplets(dut.A_triplets()[3 + i],
                    {Eigen::Triplet<double>(i + 1, i + 1, -1)},
                    dut.num_X_rows(), dut.num_X_rows());
    B_triplets_expected.emplace_back(3 + i, 1, 3);
    g_expected(3 + i) = -4;
  }

  // Check the equality constraint arising from rotated Lorentz cone
  // X(1, 2) = 0.
  CompareTriplets(
      dut.A_triplets()[5],
      {Eigen::Triplet<double>(1, 2, 0.5), Eigen::Triplet<double>(2, 1, 0.5)},
      dut.num_X_rows(), dut.num_X_rows());
  g_expected(5) = 0;
  CompareTriplets(dut.B_triplets(), B_triplets_expected, dut.g().rows(),
                  dut.num_X_rows());
  EXPECT_TRUE(CompareMatrices(dut.g(), g_expected));
}

}  // namespace internal
GTEST_TEST(SdpaFreeFormatTest, GenerateSDPA1) {
  // This is the sample program from http://plato.asu.edu/ftp/sdpa_format.txt
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto X = prog.NewSymmetricContinuousVariables<2>();
  prog.AddBoundingBoxConstraint(0, kInf, x);
  prog.AddPositiveSemidefiniteConstraint(X);
  prog.AddLinearCost(-x(0) - 2 * x(1) - 3 * X(0, 0) - 4 * X(1, 1));
  prog.AddLinearEqualityConstraint(x(0) + x(1), 10);
  prog.AddLinearEqualityConstraint(
      x(1) + 5 * X(0, 0) + 4 * X(0, 1) + 6 * X(1, 1), 20);

  const std::string file_name = temp_directory() + "/sdpa";
  std::cout << file_name << "\n";
  EXPECT_TRUE(GenerateSDPA(prog, file_name));
  EXPECT_TRUE(spruce::path(file_name + ".dat-s").exists());

  std::ifstream infile(file_name + ".dat-s");
  ASSERT_TRUE(infile.is_open());
  std::string line;
  std::getline(infile, line);
  EXPECT_EQ(line, "2");
  std::getline(infile, line);
  EXPECT_EQ(line, "2");
  std::getline(infile, line);
  EXPECT_EQ(line, "2 -2 ");
  std::getline(infile, line);
  EXPECT_EQ(line, "10 20");
  std::getline(infile, line);
  EXPECT_EQ(line, "0 1 1 1 3");
  std::getline(infile, line);
  EXPECT_EQ(line, "0 1 2 2 4");
  std::getline(infile, line);
  EXPECT_EQ(line, "0 2 1 1 1");
  std::getline(infile, line);
  EXPECT_EQ(line, "0 2 2 2 2");
  std::getline(infile, line);
  EXPECT_EQ(line, "1 2 1 1 1");
  std::getline(infile, line);
  EXPECT_EQ(line, "1 2 2 2 1");
  std::getline(infile, line);
  EXPECT_EQ(line, "2 1 1 1 5");
  std::getline(infile, line);
  EXPECT_EQ(line, "2 1 1 2 2");
  std::getline(infile, line);
  EXPECT_EQ(line, "2 1 2 2 6");
  std::getline(infile, line);
  EXPECT_EQ(line, "2 2 2 2 1");
  EXPECT_FALSE(std::getline(infile, line));

  infile.close();
}

GTEST_TEST(SdpaFreeFormatTest, GenerateInvalidSDPA) {
  // Test the program that cannot be formulated in SDPA format.
  MathematicalProgram prog1;
  prog1.NewBinaryVariables<2>();
  EXPECT_THROW(GenerateSDPA(prog1, "tmp"), std::invalid_argument);

  // program with unbounded variables.
  MathematicalProgram prog2;
  auto x = prog2.NewContinuousVariables<2>();
  prog2.AddLinearEqualityConstraint(x(0) + x(1), 1);
  EXPECT_FALSE(GenerateSDPA(prog2, "tmp"));
}

}  // namespace solvers
}  // namespace drake
