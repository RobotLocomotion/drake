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

// Neither the constructor nor MakeMutableXpr allocate when size() == 0.
TEST_F(PartialsHeapTest, MakeMutableXprEmpty) {
  LimitMalloc guard;
  Partials empty;
  empty.MakeMutableXpr();
  empty.MakeMutableXpr();
}

// Between the full-degree `value` constructor and MakeMutableXpr, at
// most one allocation (for the returned storage) is allowed.
TEST_F(PartialsHeapTest, MakeMutableXprGeneric) {
  const Eigen::Vector3d value{1.0, 2.0, 3.0};
  LimitMalloc guard({1});
  Partials full(value);
  full.MakeMutableXpr();
  full.MakeMutableXpr();
}

// Between the unit-vector constructor and MakeMutableXpr, at
// most one allocation (for the returned storage) is allowed.
TEST_F(PartialsHeapTest, MakeMutableXprUnit) {
  LimitMalloc guard({1});
  Partials unit(10, 0);
  unit.MakeMutableXpr();
  unit.MakeMutableXpr();
}

// Copy-assignment of equal sizes doesn't allocate.
TEST_F(PartialsHeapTest, CopyAssign) {
  Partials foo(Eigen::Vector3d{1.0, 2.0, 3.0});
  const Partials bar(Eigen::Vector3d{4.0, 5.0, 6.0});
  LimitMalloc guard;
  foo = bar;
}

}  // namespace
}  // namespace internal
}  // namespace ad
}  // namespace drake
