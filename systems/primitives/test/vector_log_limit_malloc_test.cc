#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/systems/primitives/vector_log.h"

namespace drake {
namespace systems {
namespace {

template <typename T>
class VectorLogLimitMallocFixture : public testing::Test {
 protected:
  int max_num_allocations() const {
    if constexpr (std::is_same_v<T, symbolic::Expression>) {
      // symbolic:: generates heap traffic on its own. The specific limit here
      // is not very important, as long as it doesn't grow without bound.
      return 2999;
    } else {
      return 0;
    }
    DRAKE_UNREACHABLE();
  }

  static constexpr int64_t kBigCapacity_ = 3 * VectorLog<T>::kDefaultCapacity;
  VectorLog<T> log_{3};
  VectorX<T> record_{Vector3<T>{1.1, 2.2, 3.3}};
};

using ScalarTypes = ::testing::Types<double, AutoDiffXd, symbolic::Expression>;

TYPED_TEST_SUITE(VectorLogLimitMallocFixture, ScalarTypes);

// Ensure that reserving log storage can avoid heap operations when adding
// data. See #10228.
TYPED_TEST(VectorLogLimitMallocFixture, Reserve) {
  // Reserve more than the default capacity.
  this->log_.Reserve(this->kBigCapacity_);

  // Store records to the full capacity, allowing allocations as determined by
  // scalar type.
  test::LimitMalloc guard({.max_num_allocations = this->max_num_allocations()});
  for (int k = 0; k < this->kBigCapacity_; k++) {
    this->log_.AddData(0.01 * k, this->record_);
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
