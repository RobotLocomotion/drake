#include "drake/multibody/mpm/grid_data.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

using IndexTypes = ::testing::Types<int32_t, int64_t>;

template <typename T>
class IndexOrFlagTest : public ::testing::Test {};

TYPED_TEST_SUITE(IndexOrFlagTest, IndexTypes);

TYPED_TEST(IndexOrFlagTest, Basic) {
  using T = TypeParam;
  IndexOrFlag<T> dut;
  EXPECT_FALSE(dut.is_index());
  EXPECT_FALSE(dut.is_flag());
  EXPECT_TRUE(dut.is_inactive());

  dut.set_index(123);
  EXPECT_EQ(dut.index(), 123);
  dut.set_inactive();
  EXPECT_TRUE(dut.is_inactive());
  dut.set_flag();
  EXPECT_TRUE(dut.is_flag());
  /* Setting flag twice is allowed. */
  dut.set_flag();
  EXPECT_TRUE(dut.is_flag());
}

TYPED_TEST(IndexOrFlagTest, StateTransition) {
  using T = TypeParam;
  IndexOrFlag<T> dut(123);
  EXPECT_TRUE(dut.is_index());
  EXPECT_FALSE(dut.is_flag());
  EXPECT_FALSE(dut.is_inactive());

  /* Active -> Inactive */
  dut.set_inactive();
  EXPECT_FALSE(dut.is_index());
  EXPECT_FALSE(dut.is_flag());
  EXPECT_TRUE(dut.is_inactive());

  /* Inactive -> Flag */
  dut.set_flag();
  EXPECT_FALSE(dut.is_index());
  EXPECT_TRUE(dut.is_flag());
  EXPECT_FALSE(dut.is_inactive());

  /* Flag -> Inactive */
  dut.set_inactive();
  EXPECT_FALSE(dut.is_index());
  EXPECT_FALSE(dut.is_flag());
  EXPECT_TRUE(dut.is_inactive());

  /* Inactive -> Active */
  dut.set_index(123);
  EXPECT_TRUE(dut.is_index());
  EXPECT_FALSE(dut.is_flag());
  EXPECT_FALSE(dut.is_inactive());
  EXPECT_EQ(dut.index(), 123);

  /* Additional scenario: Flag -> Active */
  IndexOrFlag<T> another_dut;
  another_dut.set_flag();
  another_dut.set_index(123);
  EXPECT_TRUE(another_dut.is_index());
  EXPECT_EQ(another_dut.index(), 123);
  EXPECT_FALSE(another_dut.is_flag());
  EXPECT_FALSE(another_dut.is_inactive());
}

using FloatingPointTypes = ::testing::Types<float, double, AutoDiffXd>;

template <typename T>
class GridDataTest : public ::testing::Test {};

TYPED_TEST_SUITE(GridDataTest, FloatingPointTypes);

TYPED_TEST(GridDataTest, Reset) {
  using T = TypeParam;
  GridData<T> data;
  data.index_or_flag.set_index(123);
  data.scratch = Vector3<T>::Ones();
  data.v = Vector3<T>::Ones();
  data.m = 1;

  data.reset();
  EXPECT_TRUE(data.index_or_flag.is_inactive());
  EXPECT_TRUE(data.is_inactive());
  EXPECT_EQ(data.scratch, Vector3<T>::Zero());
  EXPECT_EQ(data.v, Vector3<T>::Zero());
  EXPECT_EQ(data.m, 0);
}

TYPED_TEST(GridDataTest, Equality) {
  using T = TypeParam;

  T one{1.0};
  if constexpr (std::is_same_v<T, AutoDiffXd>) {
    one = AutoDiffXd(1.0, 1, 0);
  }

  GridData<T> data1;
  data1.index_or_flag.set_index(123);
  data1.scratch = Vector3<T>::Constant(one);
  data1.v = Vector3<T>::Constant(one);
  data1.m = 1;

  GridData<T> data2;
  data2.index_or_flag.set_index(123);
  data2.scratch = Vector3<T>::Constant(one);
  data2.v = Vector3<T>::Constant(one);
  data2.m = 1;

  EXPECT_EQ(data1, data2);
  data2.index_or_flag.set_inactive();
  EXPECT_NE(data1, data2);
  data1.index_or_flag.set_inactive();
  EXPECT_EQ(data1, data2);
  data1.reset();
  EXPECT_NE(data1, data2);
  data2.reset();
  EXPECT_EQ(data1, data2);
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
