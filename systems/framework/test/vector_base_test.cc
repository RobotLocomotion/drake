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

  DummyVector() {
    this->AppendInequalityConstraintUpperBound(10);
    this->AppendInequalityConstraintLowerBound(-10);
    this->AppendInequalityConstraintBounds(-5, 20);
  }

  int size() const override { return 2; }

  T& GetAtIndex(int index) override { return val_[index]; }

  const T& GetAtIndex(int index) const override { return val_[index]; }

 private:
  Vector2<T> val_;
};

template <typename T>
void TestBounds() {
  const double kInf = std::numeric_limits<double>::infinity();
  DummyVector<T> dummy;
  EXPECT_TRUE(CompareMatrices(dummy.inequality_constraint_lower_bound(),
                              Vector3<double>(-kInf, -10, -5)));
  EXPECT_TRUE(CompareMatrices(dummy.inequality_constraint_upper_bound(),
                              Vector3<double>(10, kInf, 20)));
  VectorX<T> inequality_val;
  dummy.CalcInequalityConstraint(&inequality_val);
  EXPECT_EQ(inequality_val.size(), 3);
}

GTEST_TEST(VectorBaseTest, TestBounds) {
  TestBounds<double>();
  TestBounds<AutoDiffXd>();
  TestBounds<symbolic::Expression>();
}
}  // namespace
}  // namespace systems
}  // namespace drake
