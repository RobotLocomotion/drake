#include "drake/geometry/render_gl/internal_shape_meshes.h"

#include <sstream>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {
namespace {

using Eigen::AngleAxisf;
using Vector2f = Vector2<GLfloat>;
using Vector3f = Vector3<GLfloat>;
using std::vector;

GTEST_TEST(LoadMeshFromObjTest, ErrorModes) {
  {
    // Case: Vertices only reports no faces found.
    std::stringstream in_stream("v 1 2 3");
    DRAKE_EXPECT_THROWS_MESSAGE(LoadMeshFromObj(&in_stream),
                                "The OBJ data appears to have no faces.*");
  }
  {
    // Case: Not an obj in any way reports as no faces.
    std::stringstream in_stream("Not an obj\njust some\nmeaningles text.\n");
    DRAKE_EXPECT_THROWS_MESSAGE(
        LoadMeshFromObj(&in_stream),
        "The OBJ data appears to have no faces.* might not be an OBJ file.+");
  }
  {
    // Case: The obj has no normals. Note: the face specification is otherwise
    // invalid in that it references vertex positions that don't exist.
    std::stringstream in_stream("v 1 2 3\nf 1 2 3\n");
    DRAKE_EXPECT_THROWS_MESSAGE(
        LoadMeshFromObj(&in_stream),
        "OBJ has no normals; RenderEngineGl requires OBJs with normals.+");
  }
  {
    // Case: Not all faces reference normals. Note: the face specification is
    // otherwise invalid in that it references vertex positions that don't
    // exist.
    std::stringstream in_stream(R"""(
v 1 2 3
vn 0 0 1
vt 0 1
f 1 2 3
)""");
    DRAKE_EXPECT_THROWS_MESSAGE(LoadMeshFromObj(&in_stream),
                                "Not all faces reference normals.+");
  }
  {
    // Case: Not all faces reference uvs. Note: the face specification is
    // otherwise invalid in that it references vertex positions that don't
    // exist.
    std::stringstream in_stream(R"""(
v 1 2 3
vn 0 0 1
vt 0 0
f 1//1 2//1 3//1
)""");
    DRAKE_EXPECT_THROWS_MESSAGE(LoadMeshFromObj(&in_stream),
                                "Not all faces reference texture.+");
  }
}

GTEST_TEST(LoadMeshFromObjTest, NoMeshUvs) {
  // Update: This OBJ has no UVs, but we have now updated this to accept it.
  std::stringstream in_stream(R"""(
v 0.1 0.2 0.3
v 0.4 0.5 0.6
v -0.1 -0.2 -0.3
vn 0 0 1
f 1//1 2//1 3//1
)""");
  EXPECT_NO_THROW(LoadMeshFromObj(&in_stream));
}

// Simply confirms that the filename variant of LoadMeshFromObj successfully
// dispatches files or errors, as appropriate. The actual parsing functionality
// is parsed via the stream interface.
GTEST_TEST(LoadMeshFromObjTest, ReadingFile) {
  const std::string filename =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj");

  MeshData mesh_data = LoadMeshFromObj(filename);
  EXPECT_EQ(mesh_data.positions.rows(), 24);
  EXPECT_EQ(mesh_data.normals.rows(), 24);
  EXPECT_EQ(mesh_data.uvs.rows(), 24);
  EXPECT_EQ(mesh_data.indices.rows(), 12);

  DRAKE_EXPECT_THROWS_MESSAGE(LoadMeshFromObj("Bad file name"),
                              "Cannot load the obj file 'Bad file name'");
}

// Helpful struct for specifying multiple cases in the TriangluatePolygons test.
struct TriangluateTestParmas {
  std::string name;
  std::string obj_spec;
  int position_count;
  // The expected normal and texture coordinate for the vertices in face 0 (f0).
  Vector3f n0;
  Vector2f uv0;
  // The expected normal and texture coordinate for the vertices in face 1 (f1).
  Vector3f n1;
  Vector2f uv1;
};

// Confirms that non-triangular faces get triangulated. This also confirms that
// duplication occurs due to associations of vertex positions with multiple
// normals or texture coordinates.
GTEST_TEST(LoadMeshFromObjTest, TriangulatePolygons) {
  /*
             o 4
            ╱  ╲            A five-sided polygon and a four-sided polygon
           ╱    ╲           should be triangulated into three and two
          ╱      ╲          triangles respectively. But with the same
         ╱        ╲         vertices.
        ╱    f0    ╲
     5 o            o 3     The upper and lower polygons have *different*
        ╲          ╱        normals. So, vertices 1 & 2 will be copied.
         ╲        ╱         All vertices have the same texture coordinate, so
       1  o──────o 2        its value does not trigger duplication.
          │  f1  │
          o──────o
          6      7
  */
  constexpr char positions[] = R"""(
  v -1 -1 0
  v 1 -1 0
  v 2 1 0
  v 0 2 0
  v -2 1 0
  v -1 -2 0
  v 1 -2 0)""";

  const Vector3f unit3_z = Vector3f::UnitZ();
  const Vector3f unit3_y = Vector3f::UnitY();
  const Vector2f zero2 = Vector2f::Zero();
  const Vector2f ones2(1, 1);

  vector<TriangluateTestParmas> test_params{
      // Every vertex is associated with a single normal and texture coordinate.
      {"Unique vertices", R"""(
      vn 0 0 1
      vt 0 0
      f 1/1/1 2/1/1 3/1/1 4/1/1 5/1/1
      f 6/1/1 7/1/1 2/1/1 1/1/1)""",
       7, unit3_z, zero2, unit3_z, zero2},
      // Upper and lower faces have different normals; vertices 1 & 2 will be
      // duplicated.
      {"Vertices with multiple normals", R"""(
      vn 0 0 1
      vn 0 1 0
      vt 0 0
      f 1/1/1 2/1/1 3/1/1 4/1/1 5/1/1
      f 6/1/2 7/1/2 2/1/2 1/1/2)""",
       9, unit3_z, zero2, unit3_y, zero2},
      // Upper and lower faces have different uvs; vertices 1 & 2 will be
      // duplicated.
      {"Vertices with multiple uvs", R"""(
      vn 0 0 1
      vt 0 0
      vt 1 1
      f 1/1/1 2/1/1 3/1/1 4/1/1 5/1/1
      f 6/2/1 7/2/1 2/2/1 1/2/1)""",
       9, unit3_z, zero2, unit3_z, ones2},
  };

  for (const auto& params : test_params) {
    SCOPED_TRACE(params.name);
    std::stringstream in_stream;
    in_stream << positions;
    in_stream << params.obj_spec;
    MeshData mesh_data = LoadMeshFromObj(&in_stream);
    EXPECT_EQ(mesh_data.positions.rows(), params.position_count);
    EXPECT_EQ(mesh_data.normals.rows(), params.position_count);
    EXPECT_EQ(mesh_data.uvs.rows(), params.position_count);
    // The two faces will always be triangulated into five triangles.
    EXPECT_EQ(mesh_data.indices.rows(), 5);

    // The first three triangles come from f0, and all have the expected normal
    // and texture coordinate.
    for (int t = 0; t < 3; ++t) {
      for (int i = 0; i < 3; ++i) {
        const int index = mesh_data.indices(t, i);
        EXPECT_TRUE(CompareMatrices(mesh_data.normals.row(index),
                                    params.n0.transpose()));
        EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(index),
                                    params.uv0.transpose()));
      }
    }
    // The last two triangles come from f1 and all have the expected normal and
    // texture coordinate.
    for (int t = 3; t < 5; ++t) {
      for (int i = 0; i < 3; ++i) {
        const int index = mesh_data.indices(t, i);
        EXPECT_TRUE(CompareMatrices(mesh_data.normals.row(index),
                                    params.n1.transpose()));
        EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(index),
                                    params.uv1.transpose()));
      }
    }
  }
}

// Geometry already triangulated gets preserved. This doesn't do variations on
// whether vertex positions need copying due to associations with multiple
// normals/uvs. It relies on `TriangulatePolygons` to handle that.
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
  std::stringstream in_stream(R"""(
  v -1 -1 0
  v 1 -1 0
  v 2 1 0
  v 0 2 0
  v -2 1 0
  v -1 -2 0
  v 1 -2 0
  vn 0 0 1
  vt 0 0
  f 1/1/1 2/1/1 5/1/1
  f 2/1/1 3/1/1 5/1/1
  f 3/1/1 4/1/1 5/1/1
  f 1/1/1 6/1/1 7/1/1
  f 1/1/1 7/1/1 2/1/1
  )""");
  MeshData mesh_data = LoadMeshFromObj(&in_stream);
  EXPECT_EQ(mesh_data.positions.rows(), 7);
  EXPECT_EQ(mesh_data.normals.rows(), 7);
  EXPECT_EQ(mesh_data.uvs.rows(), 7);
  EXPECT_EQ(mesh_data.indices.rows(), 5);
}

// The MeshData produces *new* vertices from what was in the OBJ based on what
// vertex gets referenced by which faces. A vertex that doesn't get referenced
// gets omitted.
GTEST_TEST(LoadMeshFromObjTest, RemoveUnreferencedVertices) {
  /*

        4 o───────o 3
          │     ╱ │
          │   ╱   │   o 5
          │ ╱     │
    1, 6  o───────o 2

  */
  std::stringstream in_stream(R"""(
  v -1 -1 0  # First four corners form a box around the origin.
  v 1 -1 0
  v 1 1 0
  v -1 1 0
  v 3 0 0    # Unreferenced vertex to the right of the box (omitted).
  v -1 -1 0  # Duplicate of vertex 1 but propagated through.
  vn 0 0 1
  vt 0 0
  f 1/1/1 2/1/1 3/1/1
  f 6/1/1 3/1/1 4/1/1
  )""");
  MeshData mesh_data = LoadMeshFromObj(&in_stream);
  EXPECT_EQ(mesh_data.positions.rows(), 5);
  EXPECT_EQ(mesh_data.normals.rows(), 5);
  EXPECT_EQ(mesh_data.uvs.rows(), 5);
  EXPECT_EQ(mesh_data.indices.rows(), 2);
}

}  // namespace
}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
