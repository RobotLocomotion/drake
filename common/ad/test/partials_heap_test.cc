#include <gtest/gtest.h>

#include "drake/common/ad/internal/static_unit_vector.h"
#include "drake/common/autodiff.h"
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

// MakeMutableXpr doesn't allocate when size() == 0.
TEST_F(PartialsHeapTest, MakeMutableXprEmpty) {
  LimitMalloc guard;
  Partials empty;
  empty.MakeMutableXpr();
}

// MakeMutableXpr doesn't allocate when size() == 1.
TEST_F(PartialsHeapTest, MakeMutableXprSize1) {
  LimitMalloc guard;
  Partials unit{1, 0};
  unit.MakeMutableXpr();
}

// MakeMutableXpr is allowed a single allocation (for the mutable storage).
TEST_F(PartialsHeapTest, MakeMutableXprGeneric) {
  LimitMalloc guard({1});
  Partials unit(10, 0);
  unit.MakeMutableXpr();
  unit.MakeMutableXpr();
}

}  // namespace
}  // namespace internal
}  // namespace ad
}  // namespace drake
