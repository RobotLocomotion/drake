#include "drake/common/ad/internal/static_unit_vector.h"

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace ad {
namespace internal {
namespace {

using Eigen::VectorXd;

constexpr int N = kMaxStaticVectorSize;

GTEST_TEST(StaticUnitVectorTest, SpotChecks) {
  for (int i : {0, 100, kMaxStaticVectorSize - 1}) {
    const double* data = GetStaticUnitVector(i);
    EXPECT_TRUE(CompareMatrices(Eigen::Map<const VectorXd>(data, N),
                                VectorXd::Unit(N, i)));
  }
}

}  // namespace
}  // namespace internal
}  // namespace ad
}  // namespace drake
