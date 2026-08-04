#include "drake/systems/estimators/test_utilities/sum_matrix_columns_system.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace systems {
namespace estimators_test {
namespace {

template <typename T>

class SumMatrixColumnsSystemTest : public ::testing::Test {};

using DefaultScalars =
    ::testing::Types<double, AutoDiffXd, symbolic::Expression>;
TYPED_TEST_SUITE(SumMatrixColumnsSystemTest, DefaultScalars);

TYPED_TEST(SumMatrixColumnsSystemTest, Output) {
  using T = TypeParam;
  SumMatrixColumnsSystem<T> system(3, 4);
  auto context = system.CreateDefaultContext();

  Eigen::MatrixX<T> mat(3, 4);
  // clang-format off
  mat << 1,  2,  3,  4,
         5,  6,  7,  8,
         9, 10, 11, 12;
  // clang-format on

  system.get_input_port().FixValue(context.get(), Value(mat));

  Eigen::VectorX<T> result(3);
  // clang-format off
  result << 10,
            26,
            42;
  // clang-format on

  EXPECT_TRUE(CompareMatrices(system.get_output_port().Eval(*context), result));
}

TYPED_TEST(SumMatrixColumnsSystemTest, ScalarConversion) {
  using T = TypeParam;
  SumMatrixColumnsSystem<T> system(3, 4);

  if constexpr (std::is_same_v<T, double>) {
    EXPECT_NO_THROW(system.ToAutoDiffXd());
  } else {
    EXPECT_NO_THROW(system.template ToScalarType<double>());
  }
}

}  // namespace
}  // namespace estimators_test
}  // namespace systems
}  // namespace drake
