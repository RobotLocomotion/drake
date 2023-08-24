#include "drake/geometry/hydroelastize.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(HydroelastizeTest, Trivial) {
  // Feeding in an a empty scene graph does not crash.
  SceneGraph<double> scene_graph;
  EXPECT_NO_THROW(Hydroelastize(&scene_graph));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
