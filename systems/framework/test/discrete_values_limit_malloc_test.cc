#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/discrete_values.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace systems {
namespace {

template <typename T>
class DiscreteValuesLimitMallocFixture :public testing::Test {
 protected:
  void SetUp() override {
    // Build a first object with some arbitrary data.
    a_.AppendGroup(BasicVector<T>::Make({1.0, 2.0}));
    a_.AppendGroup(BasicVector<T>::Make({3.0, 4.0}));

    // Make a second object with the same "data shape".
    b_.AppendGroup(std::make_unique<BasicVector<T>>(2));
    b_.AppendGroup(std::make_unique<BasicVector<T>>(2));
  }

  DiscreteValues<T> a_;
  DiscreteValues<T> b_;
};

using ScalarTypes = ::testing::Types<double, AutoDiffXd, symbolic::Expression>;

TYPED_TEST_SUITE(DiscreteValuesLimitMallocFixture, ScalarTypes);

// Ensure we avoid the heap when performing SetFrom() without changing scalar
// type. See #14802.
TYPED_TEST(DiscreteValuesLimitMallocFixture, NoHeapAllocsInNonConvertSetFrom) {
  test::LimitMalloc guard({.max_num_allocations = 0});
  this->b_.SetFrom(this->a_);
}

}  // namespace
}  // namespace systems
}  // namespace drake
