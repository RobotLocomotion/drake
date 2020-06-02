#include "drake/geometry/render/gl_renderer/shape_meshes.h"

#include <limits>
#include <sstream>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {
namespace {

using Vector3f = Vector3<GLfloat>;

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

// Computes the normal to the indicated triangle whose magnitude is twice the
// triangle's area. I.e., for triangle (A, B, C), computes: (B - A) X (C - A).
Vector3f CalcTriNormal(const VertexBuffer& vertices, const IndexBuffer& tris,
                       int tri_index) {
  const auto& a = vertices.block<1, 3>(tris(tri_index, 0), 0);
  const auto& b = vertices.block<1, 3>(tris(tri_index, 1), 0);
  const auto& c = vertices.block<1, 3>(tris(tri_index, 2), 0);
  return (b - a).cross(c - a);
}

// Computes the area of the indicated triangle.
GLfloat CalcTriArea(const VertexBuffer& vertices, const IndexBuffer& tris,
                    int tri_index) {
  return CalcTriNormal(vertices, tris, tri_index).norm() * 0.5;
}

// Computes the total area of the given triangles.
GLfloat CalcTotalArea(const VertexBuffer& vertices, const IndexBuffer& tris) {
  float total_area = 0;
  for (int t = 0; t < tris.rows(); ++t) {
    const auto a = CalcTriArea(vertices, tris, t);
    total_area += a;
  }
  return total_area;
}

// Computes the normal for the triangle (implied by its winding).
Vector3f CalcTriUnitNormal(const VertexBuffer& vertices,
                           const IndexBuffer& tris, int tri_index) {
  return CalcTriNormal(vertices, tris, tri_index).normalized();
}

// Computes the centroid of the indicated triangle.
Vector3f CalcTriCentroid(const VertexBuffer& vertices, const IndexBuffer& tris,
                         int tri_index) {
  return (vertices.block<1, 3>(tris(tri_index, 0), 0) +
          vertices.block<1, 3>(tris(tri_index, 1), 0) +
          vertices.block<1, 3>(tris(tri_index, 2), 0)) /
         3.f;
}

// Simply confirm that the utility functions above report good values.
GTEST_TEST(ShapeMeshesTest, ConfirmUtilities) {
  VertexBuffer vertices(3, 3);
  /*
            │╲ (0, 1.5, 0)        Right isosceles triangle with legs of length
            │ ╲                   1.5, lying on the z = 0 plane.
            │  ╲                     area = 1.5 * 1.5 / 2 = 2.25 / 2 = 1.125
            │   ╲                    unit normal = <0, 0, 1>
            │    ╲                   normal = <0, 0, 2.25>
   (0, 0, 0)└─────  (1.5, 0, 0)      centroid = (0.5, 0.5, 0)
  */
  vertices.block<1, 3>(0, 0) << 0, 0, 0;
  vertices.block<1, 3>(1, 0) << 1.5, 0, 0;
  vertices.block<1, 3>(2, 0) << 0, 1.5, 0;
  IndexBuffer indices(1, 3);
  indices.block<1, 3>(0, 0) << 0, 1, 2;
  EXPECT_EQ(CalcTriNormal(vertices, indices, 0), Vector3f(0, 0, 2.25));
  EXPECT_EQ(CalcTriUnitNormal(vertices, indices, 0), Vector3f(0, 0, 1));
  EXPECT_EQ(CalcTriArea(vertices, indices, 0), 1.125);
  EXPECT_EQ(CalcTriCentroid(vertices, indices, 0), Vector3f(0.5f, 0.5f, 0));
}

/* The tests for these tessellated primitives are merely suggestive; they don't
 guarantee correct meshes. We *might* consider confirming there are no holes in
 the mesh. In practice that may not be necessary; those kinds of artifacts would
 be immediately apparent in rendered images. Defer those heavyweight types of
 tests until there's a proven bug. */

GTEST_TEST(PrimitiveMeshTests, MakeLongLatUnitSphere) {
  const GLfloat kEps = std::numeric_limits<GLfloat>::epsilon();

  // Closed form solution for surface area of unit sphere.
  const GLfloat kIdealArea = static_cast<GLfloat>(4 * M_PI);  // 4πR², R = 1.

  float prev_area = 0;
  for (int resolution : {3, 10, 20, 40}) {
    auto [vertices, indices] = MakeLongLatUnitSphere(resolution, resolution);

    // Confirm area converges towards (but not above) ideal area.
    float curr_area = CalcTotalArea(vertices, indices);
    EXPECT_GT(curr_area, prev_area) << "for resolution " << resolution;
    EXPECT_LE(curr_area, kIdealArea) << "for resolution " << resolution;
    prev_area = curr_area;

    // All vertices lie on the unit sphere.
    for (int v_i = 0; v_i < vertices.rows(); ++v_i) {
      const auto& v = vertices.block<1, 3>(v_i, 0);
      EXPECT_NEAR(v.norm(), 1.f, kEps)
          << "for resolution: " << resolution << " and vertex " << v_i;
    }

    // All face normals point outward
    for (int t = 0; t < indices.rows(); ++t) {
      Vector3f c = CalcTriCentroid(vertices, indices, t);
      Vector3f n = CalcTriNormal(vertices, indices, t);
      // If the winding were backwards, this dot product would be negative.
      EXPECT_GT(n.dot(c), 0)
          << "for resolution: " << resolution << " and triangle " << t;
    }
  }
}

GTEST_TEST(PrimitiveMeshTests, MakeUnitCylinder) {
  const GLfloat kEps = std::numeric_limits<GLfloat>::epsilon();

  // Closed form solution for surface area of unit cylinder: H = 1, R = 1.
  //  Total cap area: 2 * πR² = 2π
  //  Barrel area: 2πRH = 2π
  //  Total area = 2π + 2π = 4π
  const GLfloat kIdealArea = static_cast<GLfloat>(4 * M_PI);

  float prev_area = 0;
  for (int resolution : {3, 10, 20, 40}) {
    auto [vertices, indices] = MakeUnitCylinder(resolution, resolution);

    // Confirm area converges towards (but not above) ideal area.
    float curr_area = CalcTotalArea(vertices, indices);
    EXPECT_GT(curr_area, prev_area) << "for resolution " << resolution;
    EXPECT_LE(curr_area, kIdealArea) << "for resolution " << resolution;
    prev_area = curr_area;

    // All vertices (except first and last) are 1 unit away from z-axis.
    for (int v_i = 1; v_i < vertices.rows() - 1; ++v_i) {
      Vector3f v = vertices.block<1, 3>(v_i, 0);
      v(2) = 0;
      EXPECT_NEAR(v.norm(), 1.f, kEps)
          << "for resolution: " << resolution << " and vertex " << v_i;
    }
    // First and last vertices are *on* the z axis.
    for (int v_i : {0, static_cast<int>(vertices.rows() - 1)}) {
      Vector3f v = vertices.block<1, 3>(v_i, 0);
      v(2) = 0;
      EXPECT_NEAR(v.norm(), 0.f, kEps)
          << "for resolution: " << resolution << " and vertex " << v_i;
    }

    // The first resolution + 1 vertices should be at z = 0.5. The last
    // resolution + 1 vertices should be at z = -0.5.
    for (int v_i = 0; v_i < resolution + 1; ++v_i) {
      EXPECT_EQ(vertices(v_i, 2), 0.5);
    }
    for (int v_i = vertices.rows() - resolution - 1; v_i < vertices.rows();
         ++v_i) {
      EXPECT_EQ(vertices(v_i, 2), -0.5);
    }

    // All face normals point outward
    for (int t = 0; t < indices.rows(); ++t) {
      Vector3f c = CalcTriCentroid(vertices, indices, t);
      Vector3f n = CalcTriNormal(vertices, indices, t);
      // If the winding were backwards, this dot product would be negative.
      EXPECT_GT(n.dot(c), 0)
          << "for resolution: " << resolution << " and triangle " << t;
    }
  }
}

GTEST_TEST(PrimitiveMeshTests, MakeSquarePatch) {
  const GLfloat kEps = std::numeric_limits<GLfloat>::epsilon();
  const GLfloat kMax = std::numeric_limits<GLfloat>::max();
  const GLfloat kMeasure = 25;
  const GLfloat kArea = kMeasure * kMeasure;

  for (int resolution : {1, 4, 15}) {
    auto [vertices, indices] = MakeSquarePatch(kMeasure, resolution);

    // The tolerance we select is going to be a scaled machine epsilon. The
    // first scale factor is the actual expected area; this makes our tolerance
    // a _relative_ tolerance. There is a second contribution to error:
    // the sum of many small real values each contribute round-off error. The
    // more we sum, the more round off error we introduce. We scale by the
    // number of triangles (representative of the number of small additions).
    const GLfloat area_epsilon = kEps * kArea * resolution * resolution;
    EXPECT_NEAR(CalcTotalArea(vertices, indices), kArea, area_epsilon);
    EXPECT_EQ(vertices.rows(), (resolution + 1) * (resolution + 1));
    EXPECT_EQ(indices.rows(), 2 * resolution * resolution);
    for (int t = 0; t < indices.rows(); ++t) {
      EXPECT_TRUE(CompareMatrices(CalcTriUnitNormal(vertices, indices, t),
                                  Vector3f(0, 0, 1), kEps));
    }

    // Confirm the bounding box is what we would expected.
    Vector3f min_pt{kMax, kMax, kMax};
    Vector3f max_pt = -min_pt;
    for (int v = 0; v < vertices.rows(); ++v) {
      Vector3f vertex = vertices.block<1, 3>(v, 0);
      min_pt = min_pt.cwiseMin(vertex);
      max_pt = max_pt.cwiseMax(vertex);
    }
    const Vector3f corner = Vector3f(0.5f, 0.5f, 0.0) * kMeasure;
    EXPECT_TRUE(CompareMatrices(min_pt, -corner, kEps));
    EXPECT_TRUE(CompareMatrices(max_pt, corner, kEps));
  }
}

GTEST_TEST(PrimitiveMeshTests, MakeUnitBox) {
  const GLfloat kEps = std::numeric_limits<GLfloat>::epsilon();
  const GLfloat kMax = std::numeric_limits<GLfloat>::max();

  auto [vertices, indices] = MakeUnitBox();

  // In computing total area, we scale epsilon by the expected area and also
  // the number of triangles, as per-triangle area gets magnified.
  EXPECT_NEAR(CalcTotalArea(vertices, indices), 6.f, kEps);
  EXPECT_EQ(vertices.rows(), 8);
  EXPECT_EQ(indices.rows(), 12);

  // All face normals point outward
  for (int t = 0; t < indices.rows(); ++t) {
    Vector3f c = CalcTriCentroid(vertices, indices, t);
    Vector3f n = CalcTriNormal(vertices, indices, t);
    // If the winding were backwards, this dot product would be negative.
    EXPECT_GT(n.dot(c), 0);
  }

  // Confirm the bounding box is what we would expected.
  Vector3f min_pt{kMax, kMax, kMax};
  Vector3f max_pt = -min_pt;
  for (int v = 0; v < vertices.rows(); ++v) {
    Vector3f vertex = vertices.block<1, 3>(v, 0);
    min_pt = min_pt.cwiseMin(vertex);
    max_pt = max_pt.cwiseMax(vertex);
  }
  const Vector3f corner{0.5f, 0.5f, 0.5};
  EXPECT_TRUE(CompareMatrices(min_pt, -corner, kEps));
  EXPECT_TRUE(CompareMatrices(max_pt, corner, kEps));
}

}  // namespace
}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
