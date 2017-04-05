#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/rndf/branch_point.h"
#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"

namespace drake {
namespace maliput {
namespace rndf {

const double kLinearTolerance = 1e-12;
const double kAngularTolerance = 1e-12;

// The following test the Junction relative API
GTEST_TEST(RNDFRoadGeometryTest, JunctionTest) {
  RoadGeometry rg({"RG-Junction"}, kLinearTolerance, kAngularTolerance);

  EXPECT_EQ(rg.num_junctions(), 0);

  Junction *j1 = rg.NewJunction({"j:1"});
  EXPECT_EQ(rg.num_junctions(), 1);
  EXPECT_EQ(rg.junction(0), j1);

  Junction *j2 = rg.NewJunction({"j:2"});
  EXPECT_EQ(rg.num_junctions(), 2);
  EXPECT_EQ(rg.junction(1), j2);

  EXPECT_THROW(rg.junction(-1), std::runtime_error);
  EXPECT_THROW(rg.junction(3), std::runtime_error);
}

// The following test the BranchPoint relative API
GTEST_TEST(RNDFRoadGeometryTest, BranchPointTest) {
  RoadGeometry rg({"RG-BranchPoint"}, kLinearTolerance, kAngularTolerance);

  EXPECT_EQ(rg.num_branch_points(), 0);

  BranchPoint *bp1 = rg.NewBranchPoint({"bp:1"});
  EXPECT_EQ(rg.num_branch_points(), 1);
  EXPECT_EQ(rg.branch_point(0), bp1);

  BranchPoint *bp2 = rg.NewBranchPoint({"bp:2"});
  EXPECT_EQ(rg.num_branch_points(), 2);
  EXPECT_EQ(rg.branch_point(1), bp2);

  EXPECT_THROW(rg.junction(-1), std::runtime_error);
  EXPECT_THROW(rg.junction(3), std::runtime_error);
}

// The following test the Getters of tolerances relative API
GTEST_TEST(RNDFRoadGeometryTest, GettersTest) {
  RoadGeometry rg({"RG-Tolerances"}, kLinearTolerance, kAngularTolerance);
  EXPECT_EQ(rg.linear_tolerance(), kLinearTolerance);
  EXPECT_EQ(rg.angular_tolerance(), kAngularTolerance);
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
