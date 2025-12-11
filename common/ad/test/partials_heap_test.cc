#include <gtest/gtest.h>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace ad {
namespace internal {
namespace {

using Eigen::VectorXd;
using test::LimitMalloc;

// An empty fixture for later expansion.
class PartialsHeapTest : public ::testing::Test {};

// Neither the constructor nor GetRawStorageMutable allocate when size() == 0.
TEST_F(PartialsHeapTest, GetRawStorageMutableEmpty) {
  LimitMalloc guard;
  Partials empty;
  empty.GetRawStorageMutable();
  empty.GetRawStorageMutable();
}

// Between the full-degree `value` constructor and GetRawStorageMutable, at
// most one allocation (for the returned storage) is allowed.
TEST_F(PartialsHeapTest, GetRawStorageMutableGeneric) {
  const Eigen::Vector3d value{1.0, 2.0, 3.0};
  LimitMalloc guard({1});
  Partials full(value);
  full.GetRawStorageMutable();
  full.GetRawStorageMutable();
}

// Between the unit-vector constructor and GetRawStorageMutable, at
// most one allocation (for the returned storage) is allowed.
TEST_F(PartialsHeapTest, GetRawStorageMutableUnit) {
  LimitMalloc guard({1});
  Partials unit(10, 0);
  unit.GetRawStorageMutable();
  unit.GetRawStorageMutable();
}

}  // namespace
}  // namespace internal
}  // namespace ad
}  // namespace drake
