#include "drake/multibody/mpm/bspline.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

template <typename T>
class BsplineWeightsTest : public ::testing::Test {
 protected:
  std::unique_ptr<BsplineWeights<T>> dut_;
};

using ScalarTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(BsplineWeightsTest, ScalarTypes);

/* BsplineWeights should create a partition of unity; i.e. All weights should
 sum up to 1.0. */
TYPED_TEST(BsplineWeightsTest, PartitionOfUnity) {
  const Vector3<TypeParam> x = Vector3<TypeParam>(0.1, 0.2, 0.3);
  const TypeParam dx = 0.5;
  this->dut_ = std::make_unique<BsplineWeights<TypeParam>>(x, dx);
  TypeParam total_weight = 0.0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        total_weight += this->dut_->weight(i, j, k);
      }
    }
  }
  EXPECT_NEAR(total_weight, 1.0,
              4.0 * std::numeric_limits<TypeParam>::epsilon());
}

/* Compare the computed result with hand computed result. */
TYPED_TEST(BsplineWeightsTest, AnalyticResult) {
  const Vector3<TypeParam> x = Vector3<TypeParam>::Zero();
  const TypeParam dx = 1.0;
  this->dut_ = std::make_unique<BsplineWeights<TypeParam>>(x, dx);
  Vector3<TypeParam> expected_1d_result(0.125, 0.75, 0.125);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        const TypeParam weight = this->dut_->weight(i, j, k);
        EXPECT_EQ(weight, expected_1d_result(i) * expected_1d_result(j) *
                              expected_1d_result(k));
      }
    }
  }
}

/* Smoke test for the factory methods. */
GTEST_TEST(BsplineWeightsTest, MakeBsplineWeights) {
  EXPECT_NO_THROW(MakeBsplineWeights<float>(Vector3<float>::Zero(), 1.0));
  EXPECT_NO_THROW(MakeBsplineWeights<double>(Vector3<double>::Zero(), 1.0));
  EXPECT_NO_THROW(MakeBsplineWeights(Vector3<AutoDiffXd>::Zero(), 1.0));
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
