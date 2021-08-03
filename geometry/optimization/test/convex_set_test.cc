#include "drake/geometry/optimization/convex_set.h"

#include <gtest/gtest.h>

#include "drake/common/is_approx_equal_abstol.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/point.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;

GTEST_TEST(ConvexSetsTest, BasicTest) {
  ConvexSets sets;

  const ConvexSet& a =
      *sets.emplace_back(Point(Vector2d{1., 2.}));
  const Vector3d b_point{3., 4., 5.};
  std::unique_ptr<Point> b_original =
      std::make_unique<Point>(b_point);
  Point* b_pointer = b_original.get();
  const ConvexSet& b = *sets.emplace_back(std::move(b_original));

  EXPECT_EQ(a.ambient_dimension(), 2);
  EXPECT_EQ(b.ambient_dimension(), 3);

  EXPECT_EQ(sets.size(), 2);
  EXPECT_EQ(sets[0]->ambient_dimension(), 2);
  EXPECT_EQ(sets[1]->ambient_dimension(), 3);

  // Confirm that a const reference to the container provides only const access
  // to the set.
  const ConvexSets& const_sets = sets;
  static_assert(std::is_same_v<const ConvexSet&, decltype(*const_sets[0])>);

  // Confirm that I can move sets without copying the underlying data.
  // Note: jwnimmer-tri argued that this should not be a strong requirement.
  // Derived ConvexSets with substantial memory footprint could implement
  // Clone() using a shared_ptr on their data.  It may be fine to remove this if
  // a different pattern requires it.
  ConvexSets moved = std::move(sets);
  EXPECT_EQ(moved.size(), 2);
  EXPECT_EQ(moved[0]->ambient_dimension(), 2);
  EXPECT_EQ(moved[1]->ambient_dimension(), 3);
  EXPECT_TRUE(moved[1]->PointInSet(b_point));
  const Vector3d new_point{6., 7., 8.};
  EXPECT_FALSE(moved[1]->PointInSet(new_point));
  b_pointer->set_x(new_point);
  EXPECT_TRUE(moved[1]->PointInSet(new_point));
}


GTEST_TEST(ConvexSetTest, IntersectsWithTest) {
/* Test that IntersectsWith() yields correct results for the following
arrangement of boxes:
     5                ┏━━━━━━━━━┓
                      ┃      C  ┃
     4      ┏━━━━━━━━━┃━━━━┓    ┃
            ┃         ┃    ┃    ┃
     3      ┃         ┗━━━━━━━━━┛
            ┃      B       ┃
     2 ┏━━━━┃━━━━┓         ┃
       ┃    ┃    ┃         ┃
     1 ┃    ┗━━━━━━━━━━━━━━┛
       ┃  A      ┃
     0 ┗━━━━━━━━━┛
       0    1    2    3    4    5
*/
  HPolyhedron set_A = HPolyhedron::MakeBox(Vector2d(0, 0), Vector2d(2, 2));
  HPolyhedron set_B = HPolyhedron::MakeBox(Vector2d(1, 1), Vector2d(4, 4));
  HPolyhedron set_C = HPolyhedron::MakeBox(Vector2d(3, 3), Vector2d(5, 5));
  EXPECT_TRUE(set_A.IntersectsWith(set_B));
  EXPECT_TRUE(set_B.IntersectsWith(set_A));
  EXPECT_TRUE(set_B.IntersectsWith(set_C));
  EXPECT_TRUE(set_C.IntersectsWith(set_B));
  EXPECT_FALSE(set_A.IntersectsWith(set_C));
  EXPECT_FALSE(set_C.IntersectsWith(set_A));
}
}  // namespace

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
