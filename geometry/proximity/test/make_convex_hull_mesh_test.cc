#include "drake/geometry/proximity/make_convex_hull_mesh.h"

#include <algorithm>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;
using PolyMesh = PolygonSurfaceMesh<double>;
using std::vector;

/* To compare polygon meshes we will produce "canonical" representations. The
 canonical mesh is the same manifold as the input mesh but has the following
 properties:

   - The vertices are deterministically ordered (basis of ordering is
     irrelevant).
   - Each face is defined with index sequence [v0, v1, v2, ...] such that v0 is
     guaranteed to be the smallest vertex index (but the relative ordering is
     the same as the original mesh).
   - The collection of faces are also sorted (ordering basis is, again,
     irrelevant). */
class CanonicalMesh {
 public:
  /* Constructs a canonical mesh from a polygon mesh. */
  explicit CanonicalMesh(const PolyMesh& mesh) {
    const vector<int> vertex_mesh_to_canon = CanonicalizeVertices(mesh);
    CanonicalizeFaces(mesh, vertex_mesh_to_canon);
  }

  /* Construct a bespoke canonical mesh. Used to test the previous constructor.
   The correctness (in the canonical sense) of vertices and faces is *not*
   tested. */
  CanonicalMesh(vector<Vector3d> vertices, vector<vector<int>> faces)
      : vertices_(std::move(vertices)), faces_(std::move(faces)) {}

  const vector<Vector3d>& vertices() const { return vertices_; }

  const vector<vector<int>> faces() const { return faces_; }

 private:
  /* Builds the list of canonicalized vertices and returns the mapping from the
   input mesh vertex index to the canonicalized vertex index. */
  vector<int> CanonicalizeVertices(const PolyMesh& mesh) {
    /* The vector key is the *ordered* vertex index, the value is the input mesh
     index. */
    vector<int> canon_to_mesh(mesh.num_vertices());
    std::iota(canon_to_mesh.begin(), canon_to_mesh.end(), 0);
    std::sort(canon_to_mesh.begin(), canon_to_mesh.end(),
              [&mesh](int a, int b) {
                const Vector3d& va = mesh.vertex(a);
                const Vector3d& vb = mesh.vertex(b);
                return std::lexicographical_compare(va.data(), va.data() + 3,
                                                    vb.data(), vb.data() + 3);
              });
    for (int v = 0; v < mesh.num_vertices(); ++v) {
      vertices_.push_back(mesh.vertex(canon_to_mesh[v]));
    }

    /* We need the map's inverse to update faces: mesh index -> canon index. */
    vector<int> mesh_to_canon(mesh.num_vertices());
    for (int i = 0; i < mesh.num_vertices(); ++i) {
      mesh_to_canon[canon_to_mesh[i]] = i;
    }
    return mesh_to_canon;
  }

  /* Builds the list of canonicalized faces. */
  void CanonicalizeFaces(const PolyMesh& mesh,
                         const vector<int> vertex_mesh_to_canon) {
    faces_.reserve(mesh.num_elements());
    for (int f = 0; f < mesh.num_faces(); ++f) {
      faces_.push_back({});
      vector<int>& canon_face = faces_.back();
      const SurfacePolygon& face = mesh.element(f);
      // Rebuild the face based on the canonicalized vertex indices.
      for (int vi = 0; vi < face.num_vertices(); ++vi) {
        canon_face.push_back(vertex_mesh_to_canon[face.vertex(vi)]);
      }
      // Rotate the face so the first vertex has lowest index.
      auto min_iter = std::min_element(canon_face.begin(), canon_face.end());
      std::rotate(canon_face.begin(), min_iter, canon_face.end());
    }
    // Now sort the faces.
    std::sort(faces_.begin(), faces_.end(), [](const auto& f1, const auto& f2) {
      return std::lexicographical_compare(f1.begin(), f1.end(), f2.begin(),
                                          f2.end());
    });
  }

  vector<Vector3d> vertices_;
  vector<vector<int>> faces_;
};

MATCHER_P(NearVertex, tolerance, "") {
  // `arg` is a 2-tuple of Vector3d.
  return (std::get<0>(arg) - std::get<1>(arg)).norm() <= tolerance;
}

void MeshesAreEquivalent(const CanonicalMesh& dut,
                         const CanonicalMesh& expected, double tolerance) {
  ASSERT_THAT(dut.vertices(),
              testing::Pointwise(NearVertex(tolerance), expected.vertices()));
  ASSERT_THAT(dut.faces(), testing::Eq(expected.faces()));
}

/* Confirm that the constructor is working correctly.

 We'll build the following tet:

                    |
                 v3 o
                    |    /
                    |   o v2
                    |  /
                    | /
                    |/
   -----------------o-------o---------
                 v0 |       v1

   The faces would be defined as:

     | f0 | v1 v2 v3 |
     | f1 | v0 v1 v3 |
     | f2 | v0 v2 v1 |
     | f3 | v0 v3 v2 |
 */
GTEST_TEST(CanonicalMeshTest, Constructor) {
  const Vector3d v0{0, 0, 0};
  const Vector3d v1{1, 0, 0};
  const Vector3d v2{0, 1, 0};
  const Vector3d v3{0, 0, 1};

  /* Mapping from the picture to the canonicalized and perturbed vertex lists.

         | picture | canon | perturbed |
         |:-------:|:-----:|:---------:|
         |   v0    |   0   |     3     |
         |   v1    |   3   |     0     |
         |   v2    |   2   |     1     |
         |   v3    |   1   |     2     |

                  Vertex indices  */
  const vector<Vector3d> canon_vertices{v0, v3, v2, v1};
  const vector<Vector3d> perturbed_vertices{v1, v2, v3, v0};

  /* Mapping from the picture to the canonicalized and perturbed faces. The
   table shows the face index and face definition as illustrated in the picture.

     - mapped: replacing picture vertex indices with canonical vertex indices.
     - rotated: rotate canonical vertex indices so that the smallest index
                is first.
     - face order: the expected order of the canonical faces.

     | pic |    pic   | mapped | rotated | face order |
     |:---:|:--------:|:------:|:-------:|:----------:|
     | f0  | v1 v2 v3 |  3 2 1 |  1 3 2  |     3      |
     | f1  | v0 v1 v3 |  0 3 1 |  0 3 1  |     2      |
     | f2  | v0 v2 v1 |  0 2 3 |  0 2 3  |     1      |
     | f3  | v0 v3 v2 |  0 1 2 |  0 1 2  |     0      | */
  const vector<vector<int>> canon_faces{
      {0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {1, 3, 2}};

  /* A similar table for perturbed vertex indices. We don't have to worry about
   rotating or ordering the faces.

     | pic |    pic   | mapped |
     |:---:|:--------:|:------:|
     | f0  | v1 v2 v3 | 0 1 2  |
     | f1  | v0 v1 v3 | 3 0 2  |
     | f2  | v0 v2 v1 | 3 1 0  |
     | f3  | v0 v3 v2 | 3 2 1  | */
  const vector<vector<int>> perturbed_faces{
      {0, 1, 2}, {3, 0, 2}, {3, 1, 0}, {3, 2, 1}};
  vector<int> perturbed_face_data;
  for (const auto& face : perturbed_faces) {
    perturbed_face_data.push_back(ssize(face));
    for (int v : face) {
      perturbed_face_data.push_back(v);
    }
  }

  const CanonicalMesh dut(
      PolygonSurfaceMesh<double>(perturbed_face_data, perturbed_vertices));
  const CanonicalMesh expected(canon_vertices, canon_faces);

  MeshesAreEquivalent(dut, expected, 0.0);
}

/* Expects that the meshes are equivalent. Equivalence is defined as follows:
  - Same number of elements (vertices and polygons)
  - There exists a bijection between the vertices of `dut` and `expected` such
    that for a mapped pair, the Euclidian distance is less than `tolerance`.
  - There exists a bijection between the polygons of the two meshes such that
    for a mapped pair:
      - */
void MeshesAreEquivalent(const PolyMesh& dut_mesh,
                         const PolyMesh& expected_mesh, double tolerance) {
  const CanonicalMesh dut(dut_mesh);
  const CanonicalMesh expected(expected_mesh);
  MeshesAreEquivalent(dut, expected, tolerance);
}

GTEST_TEST(CompareTest, Compare) {
  vector<Vector3d> ordered{Vector3d(-1, -1, -1), Vector3d(-1, -1, 1),
                           Vector3d(-1, 1, -1),  Vector3d(-1, 1, 1),
                           Vector3d(1, -1, -1),  Vector3d(1, -1, 1),
                           Vector3d(1, 1, -1),   Vector3d(1, 1, 1)};

  for (int i = 0; i < ssize(ordered); ++i) {
    const Vector3d& a = ordered[i];
    for (int j = i + 1; j < ssize(ordered); ++j) {
      const Vector3d& b = ordered[j];
      ASSERT_TRUE(std::lexicographical_compare(a.data(), a.data() + 3, b.data(),
                                               b.data() + 3))
          << fmt::format("{}", fmt_eigen(a.transpose())) << " < "
          << fmt::format("{}", fmt_eigen(b.transpose()));
    }
  }
}

GTEST_TEST(MakeConvexHullMeshTest, RejectBadShapes) {
  DRAKE_EXPECT_THROWS_MESSAGE(MakeConvexHull(Box(1, 1, 1)),
                              ".* only applies to Mesh and Convex .* Box.");
  DRAKE_EXPECT_THROWS_MESSAGE(MakeConvexHull(Capsule(1, 1)),
                              ".* only applies to .* Capsule.");
  DRAKE_EXPECT_THROWS_MESSAGE(MakeConvexHull(Cylinder(1, 1)),
                              ".* only applies .*");
  DRAKE_EXPECT_THROWS_MESSAGE(MakeConvexHull(Ellipsoid(1, 1, 1)),
                              ".* only applies .*");
  DRAKE_EXPECT_THROWS_MESSAGE(MakeConvexHull(HalfSpace()),
                              ".* only applies .*");
  DRAKE_EXPECT_THROWS_MESSAGE(MakeConvexHull(MeshcatCone(1)),
                              ".* only applies .*");
  DRAKE_EXPECT_THROWS_MESSAGE(MakeConvexHull(Sphere(1)), ".* only applies .*");
}

/* Creates a poly mesh cube that should be equivalent to the various geometries
 used in these tests. They should all be cubes, centered at the origin with
 edge length equal to two (based on the scale factor). */
PolyMesh MakeCube(double scale) {
  /* This is a transcription of the data in the obj referenced below with the
   scale factor pre-multiplied. */
  // clang-format off
  return PolyMesh({
      4, 0, 1, 3, 2,
      4, 0, 2, 6, 4,
      4, 0, 4, 5, 1,
      4, 7, 5, 4, 6,
      4, 7, 6, 2, 3,
      4, 7, 3, 1, 5
  }, {
      Vector3d(-scale, -scale, -scale),
      Vector3d(-scale, scale, -scale),
      Vector3d(scale, -scale, -scale),
      Vector3d(scale, scale, -scale),
      Vector3d(-scale, -scale, scale),
      Vector3d(-scale, scale, scale),
      Vector3d(scale, -scale, scale),
      Vector3d(scale, scale, scale)
  });
  // clang-format on
}

/* This tests the case where the obj is its own convex hull. */
GTEST_TEST(MakeConvexHullMeshTest, MeshIsHull) {
  const double scale = 2.0;
  const PolyMesh expected = MakeCube(scale);

  const PolyMesh dut = MakeConvexHull(Mesh(
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj"), scale));

  MeshesAreEquivalent(dut, expected, 1e-14);
}

/* A more elaborate non-convex obj - a cube with a hole punched through. */
GTEST_TEST(MakeConvexHullMesh, HullIsSubset) {
  const double scale = 2.0;
  const PolyMesh expected = MakeCube(scale);

  const PolyMesh dut = MakeConvexHull(Mesh(
      FindResourceOrThrow("drake/geometry/test/cube_with_hole.obj"), scale));

  MeshesAreEquivalent(dut, expected, 1e-14);
}

/* A multiple disjoint pieces have a cubical convex hull. */
GTEST_TEST(MakeConvexHullMesh, DisjointMesh) {
  const double scale = 2.0;
  const PolyMesh expected = MakeCube(scale);

  const PolyMesh dut = MakeConvexHull(
      Mesh(FindResourceOrThrow("drake/geometry/test/cube_corners.obj"), scale));

  MeshesAreEquivalent(dut, expected, 1e-14);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
