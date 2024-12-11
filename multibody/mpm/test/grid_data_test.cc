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
  using T = TypeParam;
  GridNodeIndex<T> index;
  EXPECT_FALSE(index.is_index());
  EXPECT_FALSE(index.is_participating());
  EXPECT_TRUE(index.is_inactive());

  index.set_value(123);
  EXPECT_EQ(index.value(), 123);
  index.set_inactive();
  EXPECT_TRUE(index.is_inactive());
  index.set_participating();
  EXPECT_TRUE(index.is_participating());
  /* Setting participation twice is allowed. */
  index.set_participating();
  EXPECT_TRUE(index.is_participating());
}

TYPED_TEST(GridNodeIndexTest, StateTransition) {
  using T = TypeParam;
  GridNodeIndex<T> index(123);
  EXPECT_TRUE(index.is_index());
  EXPECT_FALSE(index.is_participating());
  EXPECT_FALSE(index.is_inactive());

  /* Active -> Inactive */
  index.set_inactive();
  EXPECT_FALSE(index.is_index());
  EXPECT_FALSE(index.is_participating());
  EXPECT_TRUE(index.is_inactive());

  /* Inactive -> Participating */
  index.set_participating();
  EXPECT_FALSE(index.is_index());
  EXPECT_TRUE(index.is_participating());
  EXPECT_FALSE(index.is_inactive());

  /* Participating -> Inactive */
  index.set_inactive();
  EXPECT_FALSE(index.is_index());
  EXPECT_FALSE(index.is_participating());
  EXPECT_TRUE(index.is_inactive());

  /* Inactive -> Active */
  index.set_value(123);
  EXPECT_TRUE(index.is_index());
  EXPECT_FALSE(index.is_participating());
  EXPECT_FALSE(index.is_inactive());
  EXPECT_EQ(index.value(), 123);

  /* Additional scenario: Participating -> Active */
  GridNodeIndex<T> another_index;
  another_index.set_participating();
  another_index.set_value(123);
  EXPECT_TRUE(another_index.is_index());
  EXPECT_EQ(another_index.value(), 123);
  EXPECT_FALSE(another_index.is_participating());
  EXPECT_FALSE(another_index.is_inactive());
}

using FloatingPointTypes = ::testing::Types<float, double>;

template <typename T>
class GridDataTest : public ::testing::Test {};

TYPED_TEST_SUITE(GridDataTest, FloatingPointTypes);

TYPED_TEST(GridDataTest, Reset) {
  using T = TypeParam;
  GridData<T> data;
  data.index.set_value(123);
  data.scratch = Vector3<T>::Ones();
  data.v = Vector3<T>::Ones();
  data.m = 1;

  data.reset();
  EXPECT_TRUE(data.index.is_inactive());
  EXPECT_NE(data.scratch, data.scratch);
  EXPECT_NE(data.v, data.v);
  EXPECT_TRUE(std::isnan(data.m));
}

TYPED_TEST(GridDataTest, Equality) {
  using T = TypeParam;
  GridData<T> data1;
  data1.index.set_value(123);
  data1.scratch = Vector3<T>::Ones();
  data1.v = Vector3<T>::Ones();
  data1.m = 1;

  GridData<T> data2;
  data2.index.set_value(123);
  data2.scratch = Vector3<T>::Ones();
  data2.v = Vector3<T>::Ones();
  data2.m = 1;

  EXPECT_EQ(data1, data2);
  data2.index.set_participating();
  EXPECT_NE(data1, data2);
  data1.index.set_participating();
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
