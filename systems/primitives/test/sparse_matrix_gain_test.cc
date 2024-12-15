#include "drake/systems/primitives/sparse_matrix_gain.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace {

using Eigen::SparseMatrix;

// Tests that the outputs are correctly computed.
GTEST_TEST(MatrixGainTest, BasicTest) {
  SparseMatrix<double> D = (Eigen::Matrix<double, 2, 3>() << 0, 1, 2, 3, 4, 0)
                               .finished()
                               .sparseView();
  SparseMatrixGain<double> dut(D);

  EXPECT_EQ(dut.get_input_port().size(), 3);
  EXPECT_EQ(dut.get_output_port().size(), 2);
  EXPECT_TRUE(CompareMatrices(dut.D().toDense(), D.toDense()));

  auto context = dut.CreateDefaultContext();

  const Eigen::Vector3d u(2.17, 5.99, 1.32);
  dut.get_input_port().FixValue(context.get(), u);
  EXPECT_TRUE(CompareMatrices(dut.get_output_port().Eval(*context), D * u));

  SparseMatrix<double> D2 = (Eigen::Matrix<double, 2, 3>() << 1, 3, 0, 0, 3, 0)
                                .finished()
                                .sparseView();
  dut.set_D(D2);
  EXPECT_TRUE(CompareMatrices(dut.D().toDense(), D2.toDense()));
}

GTEST_TEST(MatrixGainTest, EmptyMatrix) {
  // Intentionally include (0, 0), (0, 1), and (1, 0) matrices to make sure
  // empty matrices are handled correctly.
  for (int rows = 0; rows < 2; ++rows) {
    for (int cols = 0; cols < 2; ++cols) {
      SparseMatrix<double> D(rows, cols);
      SparseMatrixGain<double> dut(D);

      EXPECT_EQ(dut.get_input_port().size(), cols);
      EXPECT_EQ(dut.get_output_port().size(), rows);

      auto context = dut.CreateDefaultContext();
      const Eigen::VectorXd u = Eigen::VectorXd::Ones(cols);
      dut.get_input_port().FixValue(context.get(), u);
      EXPECT_TRUE(CompareMatrices(dut.get_output_port().Eval(*context),
                                  D.toDense() * u));
    }
  }
}

// Tests converting to different scalar types.
GTEST_TEST(SparseMatrixGainTest, ConvertScalarType) {
  SparseMatrix<double> D = (Eigen::Matrix<double, 2, 3>() << 0, 1, 2, 3, 4, 0)
                               .finished()
                               .sparseView();
  SparseMatrixGain<double> dut(D);

  EXPECT_TRUE(is_autodiffxd_convertible(dut, [&](const auto& converted) {
    EXPECT_EQ(converted.D().toDense(), D.toDense());
  }));
  EXPECT_TRUE(is_symbolic_convertible(dut, [&](const auto& converted) {
    EXPECT_EQ(converted.D().toDense(), D.toDense());
  }));
}

}  // namespace
}  // namespace systems
}  // namespace drake
