#include "drake/solvers/sdpa_free_format.h"

#include <fstream>
#include <limits>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/csdp_solver_internal.h"
#include "drake/solvers/test/csdp_test_examples.h"

namespace drake {
namespace solvers {
const double kInf = std::numeric_limits<double>::infinity();
namespace internal {

void CompareProgVarInSdpa(const SdpaFreeFormat& sdpa_free_format,
                          int variable_index, double val_expected) {
  const double val =
      std::get<double>(sdpa_free_format.prog_var_in_sdpa()[variable_index]);
  EXPECT_EQ(val, val_expected);
}

void CompareProgVarInSdpa(const SdpaFreeFormat& sdpa_free_format,
                          int variable_index,
                          SdpaFreeFormat::FreeVariableIndex s_index_expected) {
  const SdpaFreeFormat::FreeVariableIndex s_index =
      std::get<SdpaFreeFormat::FreeVariableIndex>(
          sdpa_free_format.prog_var_in_sdpa()[variable_index]);
  EXPECT_EQ(s_index, s_index_expected);
}

void CompareProgVarInSdpa(const SdpaFreeFormat& sdpa_free_format,
                          int variable_index,
                          const DecisionVariableInSdpaX& val_expected) {
  const auto val = std::get<DecisionVariableInSdpaX>(
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

TEST_F(SDPwithOverlappingVariables1, TestSdpaFreeFormatConstructor) {
  const SdpaFreeFormat dut(*prog_);
  ASSERT_EQ(dut.num_X_rows(), 6);
  ASSERT_EQ(dut.num_free_variables(), 0);
  ASSERT_EQ(dut.X_blocks().size(), 3);
  CompareBlockInX(dut.X_blocks()[0], BlockInX(BlockType::kMatrix, 2));
  CompareBlockInX(dut.X_blocks()[1], BlockInX(BlockType::kMatrix, 2));
  CompareBlockInX(dut.X_blocks()[2], BlockInX(BlockType::kDiagonal, 2));
  ASSERT_EQ(dut.prog_var_in_sdpa().size(), 3);
  CompareProgVarInSdpa(dut, prog_->FindDecisionVariableIndex(x_(0)),
                       DecisionVariableInSdpaX(Sign::kPositive, 0, 0, 0, 0, 0));
  CompareProgVarInSdpa(dut, prog_->FindDecisionVariableIndex(x_(1)),
                       DecisionVariableInSdpaX(Sign::kPositive, 0, 0, 0, 1, 0));
  CompareProgVarInSdpa(dut, prog_->FindDecisionVariableIndex(x_(2)),
                       DecisionVariableInSdpaX(Sign::kPositive, 0, 1, 0, 1, 2));

  // Check A_triplets.
  ASSERT_EQ(dut.A_triplets().size(), 6);
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
  // The constraint x0 >= 0.5, we add a slack variable y0 >= 0 with constraint
  // x0 - y0 = 0.5
  CompareTriplets(dut.A_triplets()[3],
                  {Eigen::Triplet<double>(0, 0, 1.), Eigen::Triplet(4, 4, -1.)},
                  dut.num_X_rows(), dut.num_X_rows());
  // The constraint that x1 = 1
  CompareTriplets(
      dut.A_triplets()[4],
      {Eigen::Triplet<double>(0, 1, 0.5), Eigen::Triplet<double>(1, 0, 0.5)},
      dut.num_X_rows(), dut.num_X_rows());
  // The constraint x2 <= 2. We add a slack variable y2 >= 0 with the constraint
  // x2 + y2 = 2
  CompareTriplets(
      dut.A_triplets()[5],
      {Eigen::Triplet<double>(2, 3, 0.5), Eigen::Triplet<double>(3, 2, 0.5),
       Eigen::Triplet<double>(5, 5, 1)},
      dut.num_X_rows(), dut.num_X_rows());

  Vector6d g_expected;
  g_expected << 0, 0, 0, 0.5, 1, 2;
  EXPECT_TRUE(CompareMatrices(dut.g(), g_expected));
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

TEST_F(SDPwithOverlappingVariables2, TestSdpaFreeFormatConstructor) {
  const SdpaFreeFormat dut(*prog_);
  ASSERT_EQ(dut.num_X_rows(), 5);
  ASSERT_EQ(dut.num_free_variables(), 0);
  ASSERT_EQ(dut.X_blocks().size(), 2);
  CompareBlockInX(dut.X_blocks()[0], BlockInX(BlockType::kMatrix, 2));
  CompareBlockInX(dut.X_blocks()[1], BlockInX(BlockType::kDiagonal, 3));
  ASSERT_EQ(dut.prog_var_in_sdpa().size(), 2);
  CompareProgVarInSdpa(dut, prog_->FindDecisionVariableIndex(x_(0)),
                       DecisionVariableInSdpaX(Sign::kPositive, 0, 0, 0, 0, 0));
  CompareProgVarInSdpa(dut, prog_->FindDecisionVariableIndex(x_(1)),
                       DecisionVariableInSdpaX(Sign::kPositive, 0, 0, 0, 1, 0));

  // Check A_triplets.
  ASSERT_EQ(dut.A_triplets().size(), 4);
  CompareTriplets(
      dut.A_triplets()[0],
      {Eigen::Triplet<double>(0, 0, 1.0), Eigen::Triplet<double>(1, 1, -1.0)},
      dut.num_X_rows(), dut.num_X_rows());
  // The constraint 2 <= x0 <= 3, we add a slack variable y0 >= 0 with
  // constraint x0 - y0 = 2, and slack variable y1 >= 0 with constraint x0 + y1
  // = 3
  CompareTriplets(
      dut.A_triplets()[1],
      {Eigen::Triplet<double>(0, 0, 1.), Eigen::Triplet<double>(2, 2, -1.)},
      dut.num_X_rows(), dut.num_X_rows());
  CompareTriplets(
      dut.A_triplets()[2],
      {Eigen::Triplet<double>(0, 0, 1.), Eigen::Triplet<double>(3, 3, 1.)},
      dut.num_X_rows(), dut.num_X_rows());
  // The constraint that x1 >= 1
  CompareTriplets(
      dut.A_triplets()[3],
      {Eigen::Triplet<double>(0, 1, 0.5), Eigen::Triplet<double>(1, 0, 0.5),
       Eigen::Triplet<double>(4, 4, -1)},
      dut.num_X_rows(), dut.num_X_rows());

  Eigen::Vector4d g_expected(0, 2, 3, 1);
  EXPECT_TRUE(CompareMatrices(dut.g(), g_expected));
  EXPECT_EQ(dut.B_triplets().size(), 0);

  // Now check the cost.
  CompareTriplets(
      dut.C_triplets(),
      {Eigen::Triplet<double>(0, 0, -2), Eigen::Triplet<double>(0, 1, -0.5),
       Eigen::Triplet<double>(1, 0, -0.5)},
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
  dut.RemoveFreeVariableByNullspaceApproach(&C_hat, &A_hat, &rhs_hat, &y_hat,
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
  dut.RemoveFreeVariableByNullspaceApproach(&C_hat, &A_hat, &rhs_hat, &y_hat,
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
      nullptr,
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
  EXPECT_EQ(X_csdp.blocks[1].blockcategory, csdp::blockcat::DIAG);
  EXPECT_EQ(X_csdp.blocks[1].blocksize, 7);
  std::vector<double> block_val({0, 5, 0, 0, 0, 7, 0});
  for (int i = 0; i < 7; ++i) {
    EXPECT_NEAR(X_csdp.blocks[1].data.vec[i + 1], block_val[i], tol);
  }

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

void TestRemoveFreeVariableByTwoSlackVariablesApproach(
    const MathematicalProgram& prog) {
  const SdpaFreeFormat dut(prog);
  std::vector<internal::BlockInX> X_hat_blocks;
  std::vector<Eigen::SparseMatrix<double>> A_hat;
  Eigen::SparseMatrix<double> C_hat;
  dut.RemoveFreeVariableByTwoSlackVariablesApproach(&X_hat_blocks, &A_hat,
                                                    &C_hat);
  EXPECT_EQ(X_hat_blocks.size(), dut.X_blocks().size() + 1);
  for (int i = 0; i < static_cast<int>(dut.X_blocks().size()); ++i) {
    EXPECT_EQ(X_hat_blocks[i].block_type, dut.X_blocks()[i].block_type);
    EXPECT_EQ(X_hat_blocks[i].num_rows, dut.X_blocks()[i].num_rows);
  }
  EXPECT_EQ(X_hat_blocks[X_hat_blocks.size() - 1].block_type,
            BlockType::kDiagonal);
  EXPECT_EQ(X_hat_blocks[X_hat_blocks.size() - 1].num_rows,
            dut.num_free_variables() * 2);
  EXPECT_EQ(A_hat.size(), dut.A().size());
  const int num_X_hat_rows = dut.num_X_rows() + 2 * dut.num_free_variables();
  for (int i = 0; i < static_cast<int>(A_hat.size()); ++i) {
    Eigen::MatrixXd A_hat_expected =
        Eigen::MatrixXd::Zero(num_X_hat_rows, num_X_hat_rows);
    A_hat_expected.block(0, 0, dut.A()[i].rows(), dut.A()[i].cols()) =
        dut.A()[i];
    Eigen::VectorXd bi = dut.B().row(i).transpose();
    A_hat_expected.block(dut.A()[i].rows(), dut.A()[i].cols(), dut.B().cols(),
                         dut.B().cols()) = bi.asDiagonal();
    A_hat_expected.bottomRightCorner(dut.num_free_variables(),
                                     dut.num_free_variables()) =
        (-bi).asDiagonal();
    EXPECT_TRUE(CompareMatrices(Eigen::MatrixXd(A_hat[i]), A_hat_expected));
  }

  Eigen::MatrixXd C_hat_expected =
      Eigen::MatrixXd::Zero(num_X_hat_rows, num_X_hat_rows);
  C_hat_expected.topLeftCorner(dut.num_X_rows(), dut.num_X_rows()) = dut.C();
  C_hat_expected.block(dut.num_X_rows(), dut.num_X_rows(),
                       dut.num_free_variables(), dut.num_free_variables()) =
      Eigen::VectorXd(dut.d()).asDiagonal();
  C_hat_expected.bottomRightCorner(dut.num_free_variables(),
                                   dut.num_free_variables()) =
      Eigen::VectorXd(-dut.d()).asDiagonal();
  EXPECT_TRUE(CompareMatrices(Eigen::MatrixXd(C_hat), C_hat_expected));
}

TEST_F(LinearProgramBoundingBox1,
       RemoveFreeVariableByTwoSlackVariablesApproach) {
  TestRemoveFreeVariableByTwoSlackVariablesApproach(*prog_);
}

TEST_F(CsdpLinearProgram2, RemoveFreeVariableByTwoSlackVariablesApproach) {
  TestRemoveFreeVariableByTwoSlackVariablesApproach(*prog_);
}

TEST_F(TrivialSDP2, RemoveFreeVariableByTwoSlackVariablesApproach) {
  TestRemoveFreeVariableByTwoSlackVariablesApproach(*prog_);
}

TEST_F(TrivialSOCP1, RemoveFreeVariableByTwoSlackVariablesApproach) {
  TestRemoveFreeVariableByTwoSlackVariablesApproach(*prog_);
}

void TestRemoveFreeVariableByLorentzConeSlackApproach(
    const MathematicalProgram& prog) {
  const SdpaFreeFormat dut(prog);
  std::vector<internal::BlockInX> X_hat_blocks;
  std::vector<Eigen::SparseMatrix<double>> A_hat;
  Eigen::VectorXd rhs_hat;
  Eigen::SparseMatrix<double> C_hat;
  dut.RemoveFreeVariableByLorentzConeSlackApproach(&X_hat_blocks, &A_hat,
                                                   &rhs_hat, &C_hat);
  EXPECT_EQ(X_hat_blocks.size(), dut.X_blocks().size() + 1);
  for (int i = 0; i < static_cast<int>(dut.X_blocks().size()); ++i) {
    EXPECT_EQ(X_hat_blocks[i].block_type, dut.X_blocks()[i].block_type);
    EXPECT_EQ(X_hat_blocks[i].num_rows, dut.X_blocks()[i].num_rows);
  }
  EXPECT_EQ(X_hat_blocks[X_hat_blocks.size() - 1].block_type,
            BlockType::kMatrix);
  EXPECT_EQ(X_hat_blocks[X_hat_blocks.size() - 1].num_rows,
            dut.num_free_variables() + 1);
  EXPECT_EQ(A_hat.size(), dut.A().size() + dut.num_free_variables() *
                                               (dut.num_free_variables() + 1) /
                                               2);
  EXPECT_EQ(rhs_hat.rows(), A_hat.size());
  Eigen::VectorXd rhs_hat_expected = Eigen::VectorXd::Zero(A_hat.size());
  rhs_hat_expected.head(dut.A().size()) = dut.g();
  EXPECT_TRUE(CompareMatrices(rhs_hat, rhs_hat_expected));
  const int num_X_hat_rows = dut.num_X_rows() + dut.num_free_variables() + 1;
  for (int i = 0; i < static_cast<int>(dut.A().size()); ++i) {
    Eigen::MatrixXd A_hat_expected =
        Eigen::MatrixXd::Zero(num_X_hat_rows, num_X_hat_rows);
    A_hat_expected.block(0, 0, dut.A()[i].rows(), dut.A()[i].cols()) =
        dut.A()[i];
    const Eigen::VectorXd bi = dut.B().row(i).transpose();
    A_hat_expected.block(dut.A()[i].rows(), dut.A()[i].cols() + 1, 1,
                         dut.B().cols()) = bi.transpose() / 2;
    A_hat_expected.block(dut.A()[i].rows() + 1, dut.A()[i].cols(),
                         dut.B().cols(), 1) = bi / 2;
    EXPECT_TRUE(CompareMatrices(Eigen::MatrixXd(A_hat[i]), A_hat_expected));
  }
  // Now check the newly added linear equality constraint.
  // Y(i, i) = Y(0, 0) and Y(i, j) = 0 for j > i >= 1
  int A_hat_count = dut.A().size();
  for (int i = 1; i < dut.num_free_variables() + 1; ++i) {
    Eigen::MatrixXd A_hat_expected =
        Eigen::MatrixXd::Zero(num_X_hat_rows, num_X_hat_rows);
    A_hat_expected(dut.num_X_rows() + i, dut.num_X_rows() + i) = 1;
    A_hat_expected(dut.num_X_rows(), dut.num_X_rows()) = -1;
    EXPECT_TRUE(
        CompareMatrices(Eigen::MatrixXd(A_hat[A_hat_count++]), A_hat_expected));
    for (int j = i + 1; j < dut.num_free_variables() + 1; ++j) {
      A_hat_expected.setZero();
      A_hat_expected(dut.num_X_rows() + i, dut.num_X_rows() + j) = 0.5;
      A_hat_expected(dut.num_X_rows() + j, dut.num_X_rows() + i) = 0.5;
      EXPECT_TRUE(CompareMatrices(Eigen::MatrixXd(A_hat[A_hat_count++]),
                                  A_hat_expected));
    }
  }

  Eigen::MatrixXd C_hat_expected =
      Eigen::MatrixXd::Zero(num_X_hat_rows, num_X_hat_rows);
  C_hat_expected.topLeftCorner(dut.num_X_rows(), dut.num_X_rows()) = dut.C();
  C_hat_expected.block(dut.num_X_rows(), dut.num_X_rows() + 1, 1,
                       dut.num_free_variables()) =
      Eigen::VectorXd(dut.d()).transpose() / 2;
  C_hat_expected.block(dut.num_X_rows() + 1, dut.num_X_rows(),
                       dut.num_free_variables(), 1) =
      Eigen::VectorXd(dut.d()) / 2;
  EXPECT_TRUE(CompareMatrices(Eigen::MatrixXd(C_hat), C_hat_expected));
}

TEST_F(LinearProgramBoundingBox1,
       RemoveFreeVariableByLorentzConeSlackApproach) {
  TestRemoveFreeVariableByLorentzConeSlackApproach(*prog_);
}

TEST_F(CsdpLinearProgram2, RemoveFreeVariableByLorentzConeSlackApproach) {
  TestRemoveFreeVariableByLorentzConeSlackApproach(*prog_);
}

TEST_F(TrivialSDP2, RemoveFreeVariableByLorentzConeSlackApproach) {
  TestRemoveFreeVariableByLorentzConeSlackApproach(*prog_);
}

TEST_F(TrivialSOCP1, RemoveFreeVariableByLorentzConeSlackApproach) {
  TestRemoveFreeVariableByLorentzConeSlackApproach(*prog_);
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
  EXPECT_TRUE(GenerateSDPA(prog, file_name));
  EXPECT_TRUE(filesystem::exists({file_name + ".dat-s"}));

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
}

GTEST_TEST(SdpaFreeFormatTest, GenerateSDPA_remove_free_variables_two_slack) {
  // Test GenerateSDPA with prog that has free variables.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto X = prog.NewSymmetricContinuousVariables<2>();
  prog.AddPositiveSemidefiniteConstraint(X);
  prog.AddLinearEqualityConstraint(X(0, 0) + x(0) + x(1), 1);
  prog.AddLinearCost(X(0, 1) + 2 * x(1));

  internal::SdpaFreeFormat dut(prog);
  EXPECT_GT(dut.num_free_variables(), 0);
  const std::string file_name = temp_directory() + "/sdpa_free1";
  EXPECT_TRUE(GenerateSDPA(prog, file_name,
                           RemoveFreeVariableMethod::kTwoSlackVariables));
  EXPECT_TRUE(filesystem::exists({file_name + ".dat-s"}));

  std::ifstream infile(file_name + ".dat-s");
  ASSERT_TRUE(infile.is_open());
  std::string line;
  std::getline(infile, line);
  EXPECT_EQ(line, "1");
  std::getline(infile, line);
  EXPECT_EQ(line, "2");
  std::getline(infile, line);
  EXPECT_EQ(line, "2 -4 ");
  std::getline(infile, line);
  EXPECT_EQ(line, "1");
  std::getline(infile, line);
  EXPECT_EQ(line, "0 1 1 2 -0.5");
  std::getline(infile, line);
  EXPECT_EQ(line, "0 2 2 2 -2");
  std::getline(infile, line);
  EXPECT_EQ(line, "0 2 4 4 2");
  std::getline(infile, line);
  EXPECT_EQ(line, "1 1 1 1 1");
  std::getline(infile, line);
  EXPECT_EQ(line, "1 2 1 1 1");
  std::getline(infile, line);
  EXPECT_EQ(line, "1 2 2 2 1");
  std::getline(infile, line);
  EXPECT_EQ(line, "1 2 3 3 -1");
  std::getline(infile, line);
  EXPECT_EQ(line, "1 2 4 4 -1");
  EXPECT_FALSE(std::getline(infile, line));

  infile.close();
}

GTEST_TEST(SdpaFreeFormatTest, GenerateSDPA_remove_free_variables_null_space) {
  // Test GenerateSDPA with prog that has free variables.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto X = prog.NewSymmetricContinuousVariables<2>();
  prog.AddPositiveSemidefiniteConstraint(X);
  prog.AddLinearEqualityConstraint(X(0, 0) + x(0) + x(1), 1);

  internal::SdpaFreeFormat dut(prog);
  EXPECT_GT(dut.num_free_variables(), 0);
  const std::string file_name = temp_directory() + "/sdpa_free2";
  EXPECT_TRUE(GenerateSDPA(prog, file_name,
                           RemoveFreeVariableMethod::kNullspace));
  EXPECT_TRUE(filesystem::exists({file_name + ".dat-s"}));
  std::ifstream infile(file_name + ".dat-s");
  ASSERT_TRUE(infile.is_open());
  std::string line;
  std::getline(infile, line);
  // The null space approach completely removes the constraint.
  EXPECT_EQ(line, "0");
  std::getline(infile, line);
  EXPECT_EQ(line, "1");
  std::getline(infile, line);
  EXPECT_EQ(line, "2 ");
  std::getline(infile, line);
  // The constraint is empty.
  EXPECT_EQ(line, "");

  infile.close();
}

GTEST_TEST(SdpaFreeFormatTest,
           GenerateSDPA_remove_free_variables_lorentz_slack) {
  // Test GenerateSDPA with prog that has free variables.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto X = prog.NewSymmetricContinuousVariables<2>();
  prog.AddPositiveSemidefiniteConstraint(X);
  prog.AddLinearEqualityConstraint(X(0, 0) + x(0) + x(1), 1);
  prog.AddLinearCost(X(0, 1) + 2 * x(1));

  internal::SdpaFreeFormat dut(prog);
  EXPECT_GT(dut.num_free_variables(), 0);
  const std::string file_name = temp_directory() + "/sdpa_free3";
  EXPECT_TRUE(GenerateSDPA(prog, file_name,
                           RemoveFreeVariableMethod::kLorentzConeSlack));
  EXPECT_TRUE(filesystem::exists({file_name + ".dat-s"}));

  std::ifstream infile(file_name + ".dat-s");
  ASSERT_TRUE(infile.is_open());
  std::string line;
  std::getline(infile, line);
  // number of constraints.
  EXPECT_EQ(line, "4");
  std::getline(infile, line);
  // nblocks
  EXPECT_EQ(line, "2");
  std::getline(infile, line);
  // block sizes
  EXPECT_EQ(line, "2 3 ");
  std::getline(infile, line);
  // constraint rhs
  EXPECT_EQ(line, "1 0 0 0");
  // Each non-zero entry in C
  std::getline(infile, line);
  EXPECT_EQ(line, "0 1 1 2 -0.5");
  std::getline(infile, line);
  EXPECT_EQ(line, "0 2 1 3 -1");
  // Each non-zero entry in Ai
  std::getline(infile, line);
  EXPECT_EQ(line, "1 1 1 1 1");
  std::getline(infile, line);
  EXPECT_EQ(line, "1 2 1 2 0.5");
  std::getline(infile, line);
  EXPECT_EQ(line, "1 2 1 3 0.5");
  std::getline(infile, line);
  EXPECT_EQ(line, "2 2 1 1 -1");
  std::getline(infile, line);
  EXPECT_EQ(line, "2 2 2 2 1");
  std::getline(infile, line);
  EXPECT_EQ(line, "3 2 2 3 0.5");
  std::getline(infile, line);
  EXPECT_EQ(line, "4 2 1 1 -1");
  std::getline(infile, line);
  EXPECT_EQ(line, "4 2 3 3 1");
  EXPECT_FALSE(std::getline(infile, line));

  infile.close();
}

}  // namespace solvers
}  // namespace drake
