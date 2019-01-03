#include "drake/systems/framework/vector_base.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace systems {
namespace {
template <typename T>
class DummyVector : public VectorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyVector)

  DummyVector() : VectorBase<T>(3) {
    this->SetUpperBound(0, 10);
    this->SetLowerBound(1, -10);
    this->SetBounds(2, -5, 20);
  }

  T& GetAtIndex(int index) override { return val_[index]; }

  const T& GetAtIndex(int index) const override { return val_[index]; }

 private:
  Vector3<T> val_;
};

template <typename T>
void TestBounds() {
  const double kInf = std::numeric_limits<double>::infinity();
  DummyVector<T> dummy;
  EXPECT_TRUE(
      CompareMatrices(dummy.lower_bound(), Vector3<double>(-kInf, -10, -5)));
  EXPECT_TRUE(
      CompareMatrices(dummy.upper_bound(), Vector3<double>(10, kInf, 20)));
  EXPECT_EQ(dummy.size(), 3);
}

GTEST_TEST(VectorBaseTest, TestBounds) {
  TestBounds<double>();
  TestBounds<AutoDiffXd>();
  TestBounds<symbolic::Expression>();
}
}  // namespace
}  // namespace systems
}  // namespace drake
