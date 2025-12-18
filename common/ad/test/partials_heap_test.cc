#include <gtest/gtest.h>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/internal/static_unit_vector.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace ad {
namespace internal {
namespace {

using Eigen::VectorXd;
using test::LimitMalloc;

// Provide some autodiff variables that don't expire at scope end for test
// cases.
class PartialsHeapTest : public ::testing::Test {
 public:
  PartialsHeapTest() {
    // Prime the global storage so that it doesn't get billed to the test case.
    internal::GetStaticUnitVector(0);
  }
};

// Neither the constructor nor MakeMutableXpr allocate when size() == 0.
TEST_F(PartialsHeapTest, MutableEmpty) {
  LimitMalloc guard;
  Partials empty;
  empty.MakeMutableXpr();
  empty.MakeMutableXpr();
}

// Neither the constructor nor MakeMutableXpr allocates when size() == 1.
TEST_F(PartialsHeapTest, MutableUnit) {
  LimitMalloc guard;
  Partials unit{1, 0};
  unit.MakeMutableXpr();
  unit.MakeMutableXpr();
}

// Between the full-degree `value` constructor and MakeMutableXpr, at most one
// allocation (for the returned storage) is allowed.
TEST_F(PartialsHeapTest, MutableGeneric) {
  const Eigen::Vector3d value{1.0, 2.0, 3.0};
  LimitMalloc guard({1});
  Partials full(value);
  full.MakeMutableXpr();
  full.MakeMutableXpr();
}

}  // namespace
}  // namespace internal
}  // namespace ad
}  // namespace drake
