#include "drake/multibody/mpm/grid_data.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

using IndexTypes = ::testing::Types<int32_t, int64_t>;

template <typename T>
class GridNodeIndexTest : public ::testing::Test {};

TYPED_TEST_SUITE(GridNodeIndexTest, IndexTypes);

TYPED_TEST(GridNodeIndexTest, Basic) {
  GridNodeIndex<TypeParam> index;
  EXPECT_TRUE(index.is_valid());

  index.set_value(123);
  EXPECT_TRUE(index.is_valid());
  EXPECT_EQ(index.value(), 123);
  index.reset();
  EXPECT_TRUE(index.is_valid());
  index.set_flag();
  EXPECT_FALSE(index.is_valid());
  /* Setting participation twice in a row is fine. */
  index.set_flag();
  EXPECT_FALSE(index.is_valid());
}

TYPED_TEST(GridNodeIndexTest, Equality) {
  GridNodeIndex<TypeParam> index1(123);
  GridNodeIndex<TypeParam> index2(123);
  EXPECT_EQ(index1, index2);
  index2.set_flag();
  EXPECT_NE(index1, index2);
  index1.set_flag();
  EXPECT_EQ(index1, index2);
  index2.reset();
  EXPECT_NE(index1, index2);
  index1.reset();
  EXPECT_EQ(index1, index2);
  index2.set_value(123);
  EXPECT_NE(index1, index2);
  index1.set_value(123);
  EXPECT_EQ(index1, index2);
}

using FloatingPointTypes = ::testing::Types<float, double>;

template <typename T>
class GridDataTest : public ::testing::Test {};

TYPED_TEST_SUITE(GridDataTest, FloatingPointTypes);

TYPED_TEST(GridDataTest, Reset) {
  GridData<TypeParam> data;
  data.index.set_value(123);
  data.scratch = Vector3<TypeParam>::Ones();
  data.v = Vector3<TypeParam>::Ones();
  data.m = 1;

  data.reset();
  EXPECT_EQ(data.index.value(), 0);
  EXPECT_EQ(data.scratch, Vector3<TypeParam>::Zero());
  EXPECT_EQ(data.v, Vector3<TypeParam>::Zero());
  EXPECT_EQ(data.m, 0);
}

TYPED_TEST(GridDataTest, Equality) {
  GridData<TypeParam> data1;
  data1.index.set_value(123);
  data1.scratch = Vector3<TypeParam>::Ones();
  data1.v = Vector3<TypeParam>::Ones();
  data1.m = 1;

  GridData<TypeParam> data2;
  data2.index.set_value(123);
  data2.scratch = Vector3<TypeParam>::Ones();
  data2.v = Vector3<TypeParam>::Ones();
  data2.m = 1;

  EXPECT_EQ(data1, data2);
  data2.index.set_flag();
  EXPECT_NE(data1, data2);
  data1.index.set_flag();
  EXPECT_EQ(data1, data2);
  data2.reset();
  EXPECT_NE(data1, data2);
  data1.reset();
  EXPECT_EQ(data1, data2);
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
