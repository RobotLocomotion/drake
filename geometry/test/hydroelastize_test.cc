#include "drake/geometry/hydroelastize.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace {

// GTEST_TEST(HydroelastizeTest, TrivialSceneGraph) {
//   // Feeding in an a empty scene graph does not crash.
//   SceneGraph<double> scene_graph;
//   EXPECT_NO_THROW(Hydroelastize(&scene_graph));
// }

GTEST_TEST(HydroelastizeTest, TrivialGeometryState) {
  // Feeding in an a empty scene graph does not crash.
  GeometryState<double> geometry_state;
  EXPECT_NO_THROW(internal::Hydroelastize(&geometry_state));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
