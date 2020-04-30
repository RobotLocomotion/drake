#include "drake/geometry/render/gl_renderer/dev/shape_meshes.h"

#include <sstream>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {
namespace {

GTEST_TEST(LoadMeshFromObjTest, ErrorModes) {
  {
    // Case: Vertices only reports no faces found.
    std::stringstream in_stream("v 1 2 3");
    DRAKE_EXPECT_THROWS_MESSAGE(LoadMeshFromObj(&in_stream), std::runtime_error,
                                "The OBJ data appears to have no faces.*");
  }
  {
    // Case: Not an obj in any way reports as no faces.
    std::stringstream in_stream("Not an obj\njust some\nmeaningles text.\n");
    DRAKE_EXPECT_THROWS_MESSAGE(
        LoadMeshFromObj(&in_stream), std::runtime_error,
        "The OBJ data appears to have no faces.* might not be an OBJ file");
  }
}

// Simply confirms that the filename variant of LoadMeshFromObj successfully
// dispatches files or errors, as appropriate. The actual parsing functionality
// is parsed via the stream interface.
GTEST_TEST(LoadMeshFromObjTest, ReadingFile) {
  const std::string filename =
      FindResourceOrThrow("drake/systems/sensors/test/models/meshes/box.obj");

  auto [vertices, indices] = LoadMeshFromObj(filename);
  EXPECT_EQ(vertices.rows(), 8);
  EXPECT_EQ(indices.rows(), 12);

  DRAKE_EXPECT_THROWS_MESSAGE(LoadMeshFromObj("Bad file name"),
                              std::runtime_error,
                              "Cannot load the obj file 'Bad file name'");
}

// Confirms that non-triangular faces get triangulated.
GTEST_TEST(LoadMeshFromObjTest, TriangulatePolygons) {
/*
             o 4
            ╱  ╲            A five-sided polygon and a four-sided polygon
           ╱    ╲           should be triangulated into three and two
          ╱      ╲          triangles respectively. But with the same
         ╱        ╲         vertices.
        ╱          ╲
     5 o            o 3
        ╲          ╱
         ╲        ╱
       1  o──────o 2
          │      │
          o──────o
          6      7
*/
  std::stringstream in_stream(R"_(
v -1 -1 0
v 1 -1 0
v 2 1 0
v 0 2 0
v -2 1 0
v -1 -2 0
v 1 -2 0
f 1 2 3 4 5
f 6 7 2 1
  )_");
  auto [vertices, indices] = LoadMeshFromObj(&in_stream);
  EXPECT_EQ(vertices.rows(), 7);
  EXPECT_EQ(indices.rows(), 5);
}

// Geometry already triangulated gets preserved.
GTEST_TEST(LoadMeshFromObjTest, PreserveTriangulation) {
/*
             o 4
            ╱  ╲
           ╱    ╲
          ╱      ╲
         ╱        ╲
        ╱          ╲       Note: Ascii art makes drawing the following edges
     5 o────────────o 3    nearly impossible.
        ╲          ╱
         ╲        ╱        Connect 2 to 5 to form two triangles.
       1  o──────o 2
          │      │         Connect 1 to 7 to form two triangles.
          o──────o
          6      7
*/
  std::stringstream in_stream(R"_(
v -1 -1 0
v 1 -1 0
v 2 1 0
v 0 2 0
v -2 1 0
v -1 -2 0
v 1 -2 0
f 1 2 5
f 2 3 5
f 3 4 5
f 1 6 7
f 1 7 2
  )_");
  auto [vertices, indices] = LoadMeshFromObj(&in_stream);
  EXPECT_EQ(vertices.rows(), 7);
  EXPECT_EQ(indices.rows(), 5);
}

// Confirms that no mesh optimization takes place, e.g., unreferenced vertices
// are still included and duplicate vertices are not merged.
GTEST_TEST(LoadMeshFromObjTest, NoMeshOptimization) {
/*

        4 o───────o 3
          │     ╱ │
          │   ╱   │   o 5
          │ ╱     │
    1, 6  o───────o 2

*/
  std::stringstream in_stream(R"_(
v -1 -1 0  # First four corners form a box around the origin.
v 1 -1 0
v 1 1 0
v -1 1 0
v 3 0 0    # Unreferenced vertex to the right of the box
v -1 -1 0  # Duplpicate of vertex 1
f 1 2 3
f 6 3 4
  )_");
  auto [vertices, indices] = LoadMeshFromObj(&in_stream);
  EXPECT_EQ(vertices.rows(), 6);
  EXPECT_EQ(indices.rows(), 2);
}

}  // namespace
}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
