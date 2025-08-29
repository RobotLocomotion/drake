#include "drake/multibody/mpm/spgrid.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

using Vector4d = Eigen::Vector4d;

struct GridData {
  Vector4d data;
  void reset() { data.setZero(); }
};

bool operator==(const GridData& lhs, const GridData& rhs) {
  return lhs.data == rhs.data;
}

using Offset = SpGrid<GridData>::Offset;

/* Helper function to help cutting down the amount of boilerplates when looping
 over pads. */
void TripleLoop(const std::function<void(int, int, int)>& func) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        func(i, j, k);
      }
    }
  }
}

GTEST_TEST(SpGridTest, Constructor) {
  /* Constructor itself shouldn't allocate memory; memory is only allocated when
   Allocate() is called. */
  test::LimitMalloc guard;
  SpGrid<GridData> grid;
}

GTEST_TEST(SpGridTest, Allocate) {
  SpGrid<GridData> grid;
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

GTEST_TEST(SpGridTest, CoordinateOffsetConversion) {
  SpGrid<GridData> grid;

  /* Test that different coordinates yield different offsets */
  Offset offset0 = grid.CoordinateToOffset(0, 0, 0);
  Offset offset1 = grid.CoordinateToOffset(10, 0, 0);
  Offset offset2 = grid.CoordinateToOffset(0, 10, 0);

  EXPECT_NE(offset0, offset1);
  EXPECT_NE(offset0, offset2);
  EXPECT_NE(offset1, offset2);

  /* Test with negative coordinates. */
  Offset negative_offset = grid.CoordinateToOffset(-1, -1, -1);
  EXPECT_NE(negative_offset, offset0);

  /* Verify that the conversion is reversible */
  Vector3<int> coordinate0 = grid.OffsetToCoordinate(offset0);
  EXPECT_EQ(coordinate0, Vector3<int>(0, 0, 0));
  Vector3<int> coordinate1 = grid.OffsetToCoordinate(offset1);
  EXPECT_EQ(coordinate1, Vector3<int>(10, 0, 0));
  Vector3<int> coordinate2 = grid.OffsetToCoordinate(offset2);
  EXPECT_EQ(coordinate2, Vector3<int>(0, 10, 0));

  /* Test the two flavors of CoordinateToOffset. */
  EXPECT_EQ(grid.CoordinateToOffset(Vector3<int>(1, 2, 3)),
            grid.CoordinateToOffset(1, 2, 3));
}

GTEST_TEST(SpGridTest, SetPadDataAndGetPadData) {
  SpGrid<GridData> grid;

  /* Allocate memory for a specific offset to ensure that the pads around it are
   active. */
  Offset center_offset = grid.CoordinateToOffset(9, 1, 1);
  std::vector<Offset> offsets = {center_offset};
  grid.Allocate(offsets);

  /* Create pad data to set at the center offset. */
  Pad<GridData> pad_to_set;
  auto func = [&](int i, int j, int k) {
    pad_to_set[i][j][k].data = Vector4d(i, j, k, i + j + k);
  };
  TripleLoop(func);

  /* Set the pad data at the specified offset. */
  grid.SetPadData(center_offset, pad_to_set);

  /* Retrieve the pad data at the same offset. */
  Pad<GridData> pad_retrieved = grid.GetPadData(center_offset);

  /* Check if the retrieved data matches the set data. */
  auto check_func = [&](int i, int j, int k) {
    EXPECT_EQ(pad_retrieved[i][j][k].data, pad_to_set[i][j][k].data);
  };
  TripleLoop(check_func);
}

GTEST_TEST(SpGridTest, IterateGrid) {
  SpGrid<GridData> grid;
  std::vector<Offset> offsets = {grid.CoordinateToOffset(0, 0, 0),
                                 grid.CoordinateToOffset(1, 1, 1),
                                 grid.CoordinateToOffset(2, 2, 2)};
  grid.Allocate(offsets);

  /* Use IterateGrid to modify all grid nodes. */
  grid.IterateGrid([](GridData* node_data) {
    node_data->data = Vector4d(1.0, 2.0, 3.0, 4.0);
  });

  /* Verify that data has been modified using GetPadData that we tested
   elsewhere. */
  for (const auto& offset : offsets) {
    Pad<GridData> pad = grid.GetPadData(offset);
    auto check_func = [&](int i, int j, int k) {
      EXPECT_EQ(pad[i][j][k].data, Vector4d(1.0, 2.0, 3.0, 4.0));
    };
    TripleLoop(check_func);
  }
}

GTEST_TEST(SpGridTest, IterateGridWithOffset) {
  SpGrid<GridData> grid;
  std::vector<Offset> offsets = {grid.CoordinateToOffset(0, 0, 0),
                                 grid.CoordinateToOffset(1, 1, 1),
                                 grid.CoordinateToOffset(2, 2, 2)};
  grid.Allocate(offsets);

  /* Use IterateGridWithOffset to modify all grid nodes. */
  grid.IterateGridWithOffset([](Offset offset, GridData* node_data) {
    node_data->data =
        Vector4d(offset * 1.0, offset * 2.0, offset * 3.0, offset * 4.0);
  });

  /* Verify data using GetPadData that we tested elsewhere. */
  for (const auto& offset : offsets) {
    const Pad<GridData> pad = grid.GetPadData(offset);
    auto func = [&](int i, int j, int k) {
      const Vector3<int> coord =
          grid.OffsetToCoordinate(offset) + Vector3<int>(i - 1, j - 1, k - 1);
      const Offset adjusted_offset =
          grid.CoordinateToOffset(coord.x(), coord.y(), coord.z());
      Vector4d expected_value(adjusted_offset * 1.0, adjusted_offset * 2.0,
                              adjusted_offset * 3.0, adjusted_offset * 4.0);
      EXPECT_EQ(pad[i][j][k].data, expected_value);
    };
    TripleLoop(func);
  }

  // Use IterateGridWithOffset to ensure const access does not modify data
  grid.IterateGridWithOffset([](Offset offset, const GridData& node_data) {
    Vector4d expected_value(offset * 1.0, offset * 2.0, offset * 3.0,
                            offset * 4.0);
    EXPECT_EQ(node_data.data, expected_value);
  });
}

GTEST_TEST(SpGridTest, SetFrom) {
  SpGrid<GridData> grid1;
  SpGrid<GridData> grid2;

  /* Allocate memory for grid1 with some offsets and set data */
  std::vector<Offset> offsets = {grid1.CoordinateToOffset(0, 0, 0),
                                 grid1.CoordinateToOffset(1, 1, 1),
                                 grid1.CoordinateToOffset(2, 2, 2)};
  grid1.Allocate(offsets);

  grid1.IterateGridWithOffset([](Offset offset, GridData* node_data) {
    node_data->data = Eigen::Vector4<double>(offset * 1.0, offset * 2.0,
                                             offset * 3.0, offset * 4.0);
  });

  /* Set grid2 from grid1 and confirm they are the same. */
  grid2.SetFrom(grid1);
  EXPECT_EQ(grid1.num_blocks(), grid2.num_blocks());

  /* Verify data consistency using GetPadData for each allocated offset. */
  for (const auto& offset : offsets) {
    Pad<GridData> pad1 = grid1.GetPadData(offset);
    Pad<GridData> pad2 = grid2.GetPadData(offset);
    EXPECT_EQ(pad1, pad2);
  }

  /* Offset computations are also the same. */
  EXPECT_EQ(grid1.CoordinateToOffset(8, 4, 7),
            grid2.CoordinateToOffset(8, 4, 7));
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
