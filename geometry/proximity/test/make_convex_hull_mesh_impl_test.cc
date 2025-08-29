#include "drake/geometry/proximity/make_convex_hull_mesh_impl.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/memory_file.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using PolyMesh = PolygonSurfaceMesh<double>;
using std::vector;

namespace fs = std::filesystem;

fs::path FindPathOrThrow(const std::string& resource_file) {
  return FindResourceOrThrow(resource_file);
}

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

  const vector<vector<int>>& faces() const { return faces_; }

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
                /* N.B. We're ordering the vertices using a lexicographical sort
                 with *tolerance*. Previously, we used a lexicographical sort on
                 the exact measure values -- this worked for straightforward
                 convex hull computations because the hull's vertex measures are
                 bit identical to the input mesh's. However, we needed to add
                 this tolerance to handle rounding error that *can* be
                 introduced when inflating the hull (the process of transforming
                 vertices twice is the source of the rounding error).

                 It does mean that equality is no longer transitive. That means
                 the final order can depend on the initial order of the
                 measures. We don't expect for this to be a problem in practice,
                 but if, for example, a change in qhull version causes an
                 otherwise inexplicable failure in an inflated convex hull test,
                 *this* might be the reason. */
                constexpr double kTolerance =
                    4 * std::numeric_limits<double>::epsilon();
                const Vector3d& va = mesh.vertex(a);
                const Vector3d& vb = mesh.vertex(b);

                auto almost_equal = [](double x, double y) {
                  return std::abs(x - y) < kTolerance;
                };

                if (almost_equal(va[0], vb[0])) {
                  if (almost_equal(va[1], vb[1])) {
                    return va[2] < vb[2];
                  } else {
                    return va[1] < vb[1];
                  }
                } else {
                  return va[0] < vb[0];
                }
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
  // The Pointwise matcher compares dut and expected element-wise. The
  // comparison operator is the NearVertex matcher that requires the distance
  // between the two points to be less than tolerance.
  EXPECT_THAT(dut.vertices(),
              testing::Pointwise(NearVertex(tolerance), expected.vertices()));
  EXPECT_THAT(dut.faces(), testing::Eq(expected.faces()));
}

/* Confirm that the constructor is working correctly.

 We'll build the following tet:

                    Z
                    |
                 v3 o     Y
                    |    /
                    |   o v2
                    |  /
                    | /
                    |/
   -----------------o-------o--------- X
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

         | picture | index in canon | index in perturbed |
         |:-------:|:--------------:|:------------------:|
         |   v0    |       0        |         3          |
         |   v1    |       3        |         0          |
         |   v2    |       2        |         1          |
         |   v3    |       1        |         2          |

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
  - There exists a bijection between the vertices of `dut_mesh` and
    `expected_mesh` such that for a mapped pair, the Euclidian distance is less
    than `tolerance`.
  - There exists a bijection between the polygons of the two meshes such that
    for a mapped pair:
      - the polygons have the same number of vertices
      - there exists a rotation of one polygon such that its ith vertex is
        mapped to the ith vertex of the other polygon, for all vertices. */
void MeshesAreEquivalent(const PolyMesh& dut_mesh,
                         const PolyMesh& expected_mesh, double tolerance) {
  const CanonicalMesh dut(dut_mesh);
  const CanonicalMesh expected(expected_mesh);
  MeshesAreEquivalent(dut, expected, tolerance);
}

/* Creates a poly mesh box that should be equivalent to the various geometries
 used in these tests. They should all be boxes, centered at the origin with
 edge length equal to two times the indicated scale factors. */
PolyMesh MakeBox(const Vector3d& scale) {
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
      Vector3d(-scale.x(), -scale.y(), -scale.z()),
      Vector3d(-scale.x(), scale.y(), -scale.z()),
      Vector3d(scale.x(), -scale.y(), -scale.z()),
      Vector3d(scale.x(), scale.y(), -scale.z()),
      Vector3d(-scale.x(), -scale.y(), scale.z()),
      Vector3d(-scale.x(), scale.y(), scale.z()),
      Vector3d(scale.x(), -scale.y(), scale.z()),
      Vector3d(scale.x(), scale.y(), scale.z())
  });
  // clang-format on
}

/* This tests the case where the obj is its own convex hull. */
GTEST_TEST(MakeConvexHullMeshTest, MeshIsHull) {
  const Vector3d scale(2, 3, 4);
  const PolyMesh expected = MakeBox(scale);

  const PolyMesh dut = MakeConvexHull(
      FindPathOrThrow("drake/geometry/render/test/meshes/box.obj"), scale);

  MeshesAreEquivalent(dut, expected, 1e-14);
}

/* A more elaborate non-convex obj - a cube with a hole punched through. */
GTEST_TEST(MakeConvexHullMeshTest, HullIsSubset) {
  const Vector3d scale(2, 2, 2);
  const PolyMesh expected = MakeBox(scale);

  const PolyMesh dut = MakeConvexHull(
      FindPathOrThrow("drake/geometry/test/cube_with_hole.obj"), scale);

  MeshesAreEquivalent(dut, expected, 1e-14);
}

/* A multiple disjoint pieces have a cubical convex hull. */
GTEST_TEST(MakeConvexHullMeshTest, DisjointMesh) {
  const Vector3d scale(2, 2, 2);
  const PolyMesh expected = MakeBox(scale);

  const PolyMesh dut = MakeConvexHull(
      FindPathOrThrow("drake/geometry/test/cube_corners.obj"), scale);

  MeshesAreEquivalent(dut, expected, 1e-14);
}

/* A reality check that it also works on VTK volume mesh files. */
GTEST_TEST(MakeConvexHullMeshTest, VolumeMesh) {
  const Vector3d scale(2, 3, 4);
  // Create a surface mesh corresponding to the tet in one_tetrahedron.vtk.
  // clang-format off
  const PolyMesh expected({
      3, 0, 1, 3,
      3, 0, 2, 1,
      3, 0, 3, 2,
      3, 1, 2, 3
    }, {
      Vector3d(0, 0, 0),
      Vector3d(scale.x(), 0, 0),
      Vector3d(0, scale.y(), 0),
      Vector3d(0, 0, scale.z())
    });
  // clang-format on

  const PolyMesh dut = MakeConvexHull(
      FindPathOrThrow("drake/geometry/test/one_tetrahedron.vtk"), scale);

  MeshesAreEquivalent(dut, expected, 1e-14);
}

/* A reality check that it also works on glTF mesh files. */
GTEST_TEST(MakeConvexHullMeshTest, GltfMesh) {
  const Vector3d scale(2, 3, 4);
  /* The glTF's bin file contains a cube with a hole in the center. We'll
   apply a transform to the glTF node so it's no longer centered so we can
   confirm that we're handling the y-up vs z-up transformation.

   However, we're also applying a Drake scale factor to it. So, the expected
   cube is the unit cube, first offset and then scaled. So, we'll construct
   that expected cube here. */
  const PolyMesh bin_cube = MakeBox(Vector3d::Ones());
  const Vector3d p_WC(1, 2, 3);
  vector<int> face_data = bin_cube.face_data();
  vector<Vector3d> vertices;
  for (int vi = 0; vi < bin_cube.num_vertices(); ++vi) {
    vertices.push_back(scale.cwiseProduct(bin_cube.vertex(vi) + p_WC));
  }
  const PolyMesh expected(std::move(face_data), std::move(vertices));

  /* Create a custom version of cube_with_hole.gltf that is displaced by
   p_WC. Simply copy the .bin and write out a modified .gltf file. */
  const fs::path dir_path(temp_directory());

  const fs::path bin_source =
      FindPathOrThrow("drake/geometry/test/cube_with_hole.bin");
  const fs::path bin_target = dir_path / bin_source.filename();
  fs::copy_file(bin_source, bin_target);

  const fs::path gltf_source =
      FindPathOrThrow("drake/geometry/test/cube_with_hole.gltf");
  const fs::path gltf_target = dir_path / gltf_source.filename();
  {
    std::ifstream in_gltf(gltf_source);
    DRAKE_DEMAND(in_gltf.is_open());
    nlohmann::json cube = nlohmann::json::parse(in_gltf);
    DRAKE_DEMAND(cube["nodes"].size() == 1);
    // p_WC is expressed in Drake's z-up world. So, when we set it in the file,
    // it needs to be glTF's y-up world.
    cube["nodes"][0]["translation"] = {p_WC.x(), p_WC.z(), -p_WC.y()};
    std::ofstream out_gltf(gltf_target);
    DRAKE_DEMAND(out_gltf.is_open());
    out_gltf << cube;
  }

  const PolyMesh dut = MakeConvexHull(gltf_target, scale);

  MeshesAreEquivalent(dut, expected, 1e-14);
}

/* Tests what happens when a mesh that is strictly a 2D surface. */
GTEST_TEST(MakeConvexHullMeshTest, PlanarSurface) {
  const fs::path dir_path(temp_directory());
  const fs::path obj_path = dir_path / "one_square.obj";
  {
    std::ofstream file(obj_path);
    DRAKE_DEMAND(file.is_open());
    file << R"""(# Generated by MakeConvexHullMesh.PlanarSurface test.
v -1.0 -1.0 0.0
v -1.0 1.0 0.0
v 1.0 -1.0 0.0
v 1.0 1.0 0.0
vn 0 0 -1
f 1//1 2//1 4//1 3//1
)""";
  }

  // The convex hull of the above flat quad is, conceptually, two polygons built
  // on the same four vertices, one wound for a normal in the Wz direction, the
  // other in the -Wz direction.
  //
  // In practice, one face *is* a polygon. The face on the on the other side is
  // actually decomposed into a triangle fan (due to how we handle planar
  // geometry).
  //
  // The fan is centered around the mean vertex position: [0, 0, 0].
  //
  // The polygonal facet has a normal pointing upwards. The triangle fan has
  // normals pointing downward (defining the winding). Note: this winding is
  // simply a function of the vertex ordering in the original mesh. If we were
  // to perturb the order, we'd have to reverse this.
  // clang-format off
  const PolyMesh expected({
      4, 1, 2, 3, 4,
      3, 0, 2, 1,
      3, 0, 3, 2,
      3, 0, 4, 3,
      3, 0, 1, 4
    }, {
      Vector3d(0, 0, 0),
      Vector3d(-1, -1, 0),
      Vector3d(1, -1, 0),
      Vector3d(1, 1, 0),
      Vector3d(-1, 1, 0)
    });
  // clang-format on

  const PolyMesh dut = MakeConvexHull(obj_path, Vector3d::Constant(1));

  MeshesAreEquivalent(dut, expected, 1e-14);
}

// There are some mesh cases that we simply throw; they are and should be
// considered degenerate.
GTEST_TEST(MakeConvexHullMeshTest, DegenerateMeshes) {
  const fs::path dir_path(temp_directory());
  auto make_obj = [&dir_path](std::string_view name,
                              std::string_view contents) {
    fs::path obj_path = dir_path / name;
    std::ofstream file(obj_path);
    DRAKE_DEMAND(file.is_open());
    file << contents;
    return obj_path;
  };

  // Too few vertices
  const fs::path too_few_obj = make_obj("too_few.obj", R"""(# Generated
  v 0 0 0
  v 0 1 1
  f 1 1 2
  )""");
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeConvexHull(too_few_obj, Vector3d::Constant(1.0)),
      ".*fewer than three vertices; found 2 .*too_few.obj.");

  // Coincident points
  const fs::path coincident_obj = make_obj("coincident.obj", R"""(# Generated
  v 0 0 0
  v 9e-13 0 0
  v 0 9e-13 0
  f 1 2 3
  )""");
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeConvexHull(coincident_obj, Vector3d::Constant(1.0)),
      ".*all vertices in the mesh were within a sphere.*coincident.obj.");
  // However, scaling it up puts us outside the threshold.
  EXPECT_NO_THROW(MakeConvexHull(coincident_obj, Vector3d::Constant(2.0)));

  // Colinear points.
  const fs::path colinear_obj = make_obj("colinear.obj", R"""(# Generated
  v 0 0 0
  v -1 0 0
  v 1 0 0
  f 1 2 3
  )""");
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeConvexHull(colinear_obj, Vector3d::Constant(1.0)),
      ".*all vertices in the mesh appear to be co-linear.*colinear.obj.");
}

// We unit test code paths specific to making an inflated convex hull. More
// precisely, the computation of an inflated convex hull requires three convex
// hull computations. The first one, corresponds to a code path already tested
// in the unit tests in this file for zero margin (with various model
// permutations). This one unit test focuses on testing code paths that exercise
// the second and third convex hull computations needed to obtain an inflated
// convex hull. Therefore there is no need to test this computation with
// different models, since we can safely assumed the second and third
// computations start from a well tested first convex hull.
GTEST_TEST(MakeConvexHullMeshTest, NonZeroMargin) {
  const double margin = 0.01;
  const Vector3d scale(2, 2, 2);
  const PolyMesh expected = MakeBox(scale + Vector3d::Constant(margin));

  const PolyMesh dut = MakeConvexHull(
      FindPathOrThrow("drake/geometry/test/cube_with_hole.obj"), scale, margin);

  MeshesAreEquivalent(dut, expected, 1e-14);
}

// Create a polygon mesh which is the equivalent of the tet defined in
// one_tetrahedron.vtk, but with the faces offset by the given margin.
PolyMesh GetTetrahedronWithMargin(const Vector3d& scale, double margin) {
  // We look at the one tilted face on the original mesh.
  Vector3d c = scale / 3;             // Face's centroid.
  const double d = c.norm();          // Distance to the origin.
  const Vector3d n = c.normalized();  // Face's normal.

  // We take a look at the original vertex with coordinates p = (0, 0, L). By
  // symmetry we know that the other two are (L, 0, 0) and (0, L, 0) (plus the
  // origin). We then work with pz. All faces move margin along their normal.
  // Then the inflated point pz moves to p̃ = (-δ, -δ, L̃). The equation of the
  // tilted plane is n⋅p=d+δ, with normal n and distance d computed above.
  // Substituting p̃ = (-δ, -δ, L̃) allows us to compute L̃:
  //  L̃ = (d + δ⋅(nx+ny)) / nz.
  const double length = (d + margin * (1 + n(0) + n(1))) / n(2);
  // Create an inflated surface mesh corresponding to the tet in
  // one_tetrahedron.vtk.

  // clang-format off
  return PolyMesh({
      3, 0, 1, 3,
      3, 0, 2, 1,
      3, 0, 3, 2,
      3, 1, 2, 3
    }, {
      Vector3d(-margin, -margin, -margin),
      Vector3d(length, -margin, -margin),
      Vector3d(-margin,  length, -margin),
      Vector3d(-margin, -margin,  length)
    });
  // clang-format on
}

// This test is sensitive to the OrderPolyVertices() function in ways the
// previous tests are not, therefore providing greater test coverage.
GTEST_TEST(MakeConvexHullMeshTest, TetrahedronWithMargin) {
  const double kMargin = 0.01;
  const Vector3d kScale(2, 2, 2);

  // Create an inflated surface mesh corresponding to the tet in
  // one_tetrahedron.vtk.
  const PolyMesh expected = GetTetrahedronWithMargin(kScale, kMargin);

  const PolyMesh dut =
      MakeConvexHull(FindPathOrThrow("drake/geometry/test/one_tetrahedron.vtk"),
                     kScale, kMargin);

  MeshesAreEquivalent(dut, expected, 1e-14);
}

/* Simple regression test against passing a MeshSource to MakeConvexHull
 directly. The core functionality has already been tested above. */
GTEST_TEST(MakeConvexHullMeshTest, MakeFromMeshSource) {
  const Vector3d kScale(2, 2, 2);
  const double kMargin = 1.0;
  // The box in box.obj has edge length of 2 m. We'll scale it by s = kScale and
  // then inflate it δ = kMargin. The effective size will be 2s + 2δ. The cube
  // is a scaled unit cube; so we need to scale by (2s + 2δ) / 2 = s + δ.
  const fs::path box_path =
      FindPathOrThrow("drake/geometry/render/test/meshes/box.obj");
  const MeshSource obj_source(InMemoryMesh{MemoryFile::Make(box_path)});
  const PolyMesh expected_box = MakeBox(kScale + Vector3d::Constant(kMargin));

  // The tet in one_tetrahedron.vtk has vertices at origin and unit positions
  // along all axes.
  const fs::path tet_path =
      FindPathOrThrow("drake/geometry/test/one_tetrahedron.vtk");
  const MeshSource vtk_source(InMemoryMesh{MemoryFile::Make(tet_path)});
  const PolyMesh expected_tet = GetTetrahedronWithMargin(kScale, kMargin);

  // The rainbow_box.gltf has embedded data *and* has a non-trivial hierarchy
  // with transformations. The hull of the gltf box does not exactly match the
  // hull of the obj box on mac, so we need its own reference mesh. So, we'll
  // simply compare it against the file path's version.
  const fs::path embedded_gltf_path =
      FindResourceOrThrow("drake/geometry/render/test/meshes/rainbow_box.gltf");
  const MeshSource gltf_embedded_source(
      InMemoryMesh{MemoryFile::Make(embedded_gltf_path)});
  const PolyMesh expected_gltf_box =
      MakeConvexHull(MeshSource(embedded_gltf_path), kScale, kMargin);

  // The fully_textured_pyramid.gltf references external files. Specifically,
  // the .bin file is necessary to know vertex positions.
  const fs::path pyramid_path = FindResourceOrThrow(
      "drake/geometry/render/test/meshes/fully_textured_pyramid.gltf");
  const fs::path pyramid_bin_path = FindResourceOrThrow(
      "drake/geometry/render/test/meshes/fully_textured_pyramid.bin");
  const MeshSource gltf_pyramid_source(InMemoryMesh{
      MemoryFile::Make(pyramid_path),
      {{"fully_textured_pyramid.bin", MemoryFile::Make(pyramid_bin_path)}}});
  // The convex hull of the in-memory version should match that from disk.
  const PolyMesh expected_pyramid =
      MakeConvexHull(pyramid_path, kScale, kMargin);

  struct TestCase {
    const MeshSource* mesh_source{};
    const PolyMesh* expected_mesh{};
    std::string_view description;
  };

  std::vector<TestCase> test_cases{
      {&obj_source, &expected_box, "Valid obj"},
      {&vtk_source, &expected_tet, "Valid vtk"},
      {&gltf_embedded_source, &expected_gltf_box, "Valid embedded gltf"},
      {&gltf_pyramid_source, &expected_pyramid, "Distributed gltf"}};
  for (const TestCase& test_case : test_cases) {
    SCOPED_TRACE(test_case.description);
    const PolyMesh dut =
        MakeConvexHull(*test_case.mesh_source, kScale, kMargin);
    MeshesAreEquivalent(dut, *test_case.expected_mesh, 1e-14);
  }

  // Unsupported extension.
  {
    SCOPED_TRACE("Unsupported extension");
    const MeshSource bad_source(InMemoryMesh{MemoryFile::Make(
        FindPathOrThrow("drake/geometry/render/test/meshes/box.obj.mtl"))});
    DRAKE_EXPECT_THROWS_MESSAGE(MakeConvexHull(bad_source, kScale, kMargin),
                                ".*unsupported extension '.mtl'.*");
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
