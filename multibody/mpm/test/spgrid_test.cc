#include "drake/multibody/mpm/spgrid.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

struct GridData {
  using Scalar = double;
  Eigen::Vector4<double> data;
  void set_zero() { data.setZero(); }
};

GTEST_TEST(SpGridTest, Constructor) {
  /* Constructor itself shouldn't allocate memory; memory is only allocated when
   Allocate() is called. */
  test::LimitMalloc guard;
  SpGrid<GridData> grid;
}

GTEST_TEST(SpGridTest, Allocate) {
  SpGrid<GridData> grid;
  using Offset = SpGrid<GridData>::Offset;
  /* Given the size of GridData (4 * 8 = 32 bytes), each block in SpGrid is of
   size 4 * 4 * 8. We choose offsets such that offset0 and offset1 belong to the
   same block, but offset2 belongs to a separate block. */
  Offset offset0 = grid.CoordinateToOffset(0, 0, 0);
  Offset offset1 = grid.CoordinateToOffset(1, 0, 0);
  Offset offset2 = grid.CoordinateToOffset(5, 0, 0);
  {
    std::vector<Offset> offsets = {offset0};
    grid.Allocate(offsets);
    EXPECT_EQ(grid.num_blocks(), 1);
  }

  {
    std::vector<Offset> offsets = {offset0, offset1};
    grid.Allocate(offsets);
    EXPECT_EQ(grid.num_blocks(), 1);
  }

  {
    std::vector<Offset> offsets = {offset0, offset2};
    grid.Allocate(offsets);
    EXPECT_EQ(grid.num_blocks(), 2);
  }
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
