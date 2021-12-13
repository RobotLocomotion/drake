#include "drake/geometry/proximity/mesh_plane_intersection.h"

#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;
using math::RotationMatrix;
using math::RotationMatrixd;
using std::make_unique;
using std::pair;
using std::set;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

// TODO(SeanCurtis-TRI): I originally copied this from
//  mesh_intersection_test.cc. Perhaps this extended version should be
//  refactored to be shared across mesh tests.
/* Generates a trivial volume mesh in Frame F consisting of two tetrahedra with
 a shared face. The mesh is illustrated here in Frame M and then transformed to
 Frame F (via X_FM). The shared face in the tetrahedron lies on the Mz = 0
 plane. Each tetrahedron extends along the Mz axis in opposite directions as
 shown:

      +Z
       |
       v3
       |
       |
     v0+------v2---+Y
      /|
     / |
   v1  v4
   /   |
 +X    |
      -Z

 @param X_FM                The relative pose between Frames F and M.
 @param minimum_vertices    If true, there will be five unique vertices.
                            Otherwise, the vertices for the shared face will
                            be duplicated (i.e., the mesh will have eight
                            vertices).
*/
template <typename T>
VolumeMesh<T> TrivialVolumeMesh(
    const RigidTransform<T>& X_FM = RigidTransform<T>::Identity(),
    bool minimum_vertices = true) {
  vector<VolumeElement> elements;
  vector<Vector3<T>> vertices;
  if (minimum_vertices) {
    const int element_data[2][4] = {{0, 1, 2, 3}, {0, 2, 1, 4}};
    for (const auto& element : element_data) {
      elements.emplace_back(element);
    }
    const Vector3<T> vertex_data[5] = {
        Vector3<T>::Zero(),
        Vector3<T>::UnitX(),
        Vector3<T>::UnitY(),
        Vector3<T>::UnitZ(),
        -Vector3<T>::UnitZ()
    };
    for (auto& vertex : vertex_data) {
      vertices.emplace_back(X_FM * vertex);
    }
  } else {
    const int element_data[2][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}};
    for (const auto& element : element_data) {
      elements.emplace_back(element);
    }
    const Vector3<T> vertex_data[8] = {
        Vector3<T>::Zero(),
        Vector3<T>::UnitX(),
        Vector3<T>::UnitY(),
        Vector3<T>::UnitZ(),
        Vector3<T>::Zero(),
        Vector3<T>::UnitY(),
        Vector3<T>::UnitX(),
        -Vector3<T>::UnitZ()
    };
    for (auto& vertex : vertex_data) {
      vertices.emplace_back(X_FM * vertex);
    }
  }
  return {std::move(elements), std::move(vertices)};
}

/* Infrastructure for testing SliceTetWithPlane(). These tests involve three
 frames: M, F, and W. The trivial mesh (see above) is defined in its frame M.
 However, we transform that trivial mesh into some arbitrary frame F (so that
 the mesh vertices don't have a bunch of trivial zeros and ones). The
 intersecting plane is likewise defined in that same frame F. Finally, the
 result of slicing the tet with the plane produces results in the *world* frame
 W (via the X_WF_ transform). */
class SliceTetWithPlaneTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // An arbitrary transform that doesn't contain any identities (although it
    // will require a larger epsilon in the tests due to compounding rounding
    // error).
    X_WF_ = RigidTransformd{
        RotationMatrixd{AngleAxisd{M_PI / 3, Vector3d{-1, 1, 1}.normalized()}},
        Vector3d{1.25, 2.5, -3.75}};
  }

  /* Invokes SliceTetWithPlane() on a trivial mesh in frame F. This method
   arbitrarily uses a PolygonMeshBuilder (the choice doesn't matter, but we
   choose it because it leads to simpler mesh analysis). The mesh and field
   created by building the mesh after this single slicing operation are
   returned. The mesh *may* be empty.

   As a side effect, member fields volume_mesh_F_ and field_F_ are written to;
   they are the mesh and field used in the query (posed in frame F via the given
   X_FM).

   @param plane_F       The slicing plane (measured and expressed in Frame F).
   @param X_FM          The relative pose between the trivial mesh's canonical
                        frame M and the query frame F.

   @returns The resulting mesh and field expressed in the world frame. */
  pair<unique_ptr<PolygonSurfaceMesh<double>>,
       unique_ptr<PolygonSurfaceMeshFieldLinear<double, double>>>
  CallSliceTetWithPlane(const Plane<double>& plane_F,
                        const RigidTransformd& X_FM) {
    volume_mesh_F_ = make_unique<VolumeMesh<double>>(TrivialVolumeMesh(X_FM));
    // Make an arbitrary mesh field with heterogeneous values.
    vector<double> values{0.25, 0.5, 0.75, 1, -1};
    field_F_ = make_unique<VolumeMeshFieldLinear<double, double>>(
        move(values), volume_mesh_F_.get());

    cut_edges_.clear();
    const int tet_index = 0;
    // Every invocation gets a new builder instance.
    builder_W_ = PolyMeshBuilder<double>();
    SliceTetWithPlane(tet_index, *field_F_, plane_F, X_WF_, &builder_W_,
                      &cut_edges_);
    return builder_W_.MakeMeshAndField();
  }

  /* Reports `true` if mesh and field have a single face and the indicated
   number of vertices and field values. */
  ::testing::AssertionResult HasNVertices(
      const PolygonSurfaceMesh<double>& mesh_W,
      const PolygonSurfaceMeshFieldLinear<double, double>& field_W,
      size_t num_vertices) const {
    bool error = false;
    ::testing::AssertionResult failure = ::testing::AssertionFailure();
    auto test_features = [&failure, &error](size_t test_count,
                                            size_t expected_count,
                                            const char* label) {
      if (test_count != expected_count) {
        error = true;
        failure << "Wrong number of " << label
                << ".\n Expected: " << expected_count
                << "\n Found: " << test_count << "\n";
      }
    };
    const int num_faces = num_vertices > 0 ? 1 : 0;
    test_features(mesh_W.num_elements(), num_faces, "faces");
    test_features(mesh_W.num_vertices(), num_vertices, "vertices");
    test_features(field_W.values().size(), num_vertices, "pressure values");
    // Each vertex in the mesh arose from slicing one tet edge.
    test_features(cut_edges_.size(), num_vertices, "cut edges");

    return error ? failure : ::testing::AssertionSuccess();
  }

  /* Struct for associating a vertex produced from a slice with the mesh edge
   that got sliced. It includes the resultant _surface_ vertex index, the
   indices of the vertices in the volume_ mesh that define the edge, and a
   weight value such that the position of the slice vertex is:
   weight * P(edge.first) + (1 - weight) * P(edge.second), where P(v) is the
   position of the volume vertex with index v.  */
  struct EdgeVertex {
    int slice_vertex;
    SortedPair<int> edge;
    double weight{};
  };

  /* For each vertex in the "slice" mesh polygon, determine which edge in the
   tet it lies on and its "weight" along that edge. If the classification fails
   for any vertex, report failure (without edge data). */
  pair<vector<EdgeVertex>, ::testing::AssertionResult> CharacterizeEdgeVertices(
      const PolygonSurfaceMesh<double>& slice_mesh_W, int tet_index,
      const VolumeMesh<double>& volume_mesh_F) const {
    // This test is designed to work with a *single* slice polygon.
    DRAKE_DEMAND(slice_mesh_W.num_elements() == 1);

    constexpr double kEps = 2 * std::numeric_limits<double>::epsilon();
    const VolumeElement& tet = volume_mesh_F.element(tet_index);

    // Enumeration of all tetrahedron edges defined by vertex pairs referenced
    // by tet-local index values.
    const int tet_edges[6][2] = {{0, 1}, {0, 2}, {0, 3},
                                 {1, 2}, {1, 3}, {2, 3}};

    const RigidTransformd X_FW = X_WF_.inverse();

    vector<EdgeVertex> edge_vertices;
    // The sliced mesh vertices we've classified.
    set<int> slice_vertex_indices;
    const auto& poly = slice_mesh_W.element(0);
    for (int i = 0; i < poly.num_vertices(); ++i) {
      const int v = poly.vertex(i);
      if (slice_vertex_indices.count(v) > 0) continue;
      slice_vertex_indices.insert(v);
      // The slice polygon vertex S in the mesh frame F.
      const Vector3d p_FS = X_FW * slice_mesh_W.vertex(v);
      for (int e = 0; e < 6; ++e) {
        const int V0 = tet.vertex(tet_edges[e][0]);
        const int V1 = tet.vertex(tet_edges[e][1]);
        const Vector3d p_FV0 = volume_mesh_F.vertex(V0);
        const Vector3d p_FV1 = volume_mesh_F.vertex(V1);
        const Vector3d p_V0V1_F = p_FV1 - p_FV0;
        const double d_V0V1 = p_V0V1_F.norm();
        const Vector3d p_V0S_F = p_FS - p_FV0;
        const double d_V0S = p_V0S_F.norm();
        // Test that if S lies on the line segment spanned by V0V1.
        //  - it is colinear with the _line_ passing through V0V1 and
        //  - its "weight" (i.e., interpolation between V0 and V1) is in the
        //    interval [0, 1].
        //
        // We can implicitly test collinearity (and part of the weight value)
        // by comparing the vector V0S with V0V1. A negative dot product
        // indicates that S *cannot* lie between V0 and V1 (colinear or not)
        // and a positive value equal to |V0S| * |V0V1| indicates
        // colinearity (it can still lie beyond the edge's vertices).
        if (p_V0V1_F.dot(p_V0S_F) - d_V0V1 * d_V0S <= -kEps) continue;
        // Define weight w such that: S = w * V0 + (1 - w) * V1.
        double w = 1.0 - d_V0S / d_V0V1;
        // The previous test already confirmed colinearity and w >= 0.
        if (w > 1 + kEps) {
          return {{},
                  ::testing::AssertionFailure()
                      << "Vertex " << v << " is co-linear with edge " << e
                      << " but doesn't lie on the edge"};
        }
        edge_vertices.push_back(EdgeVertex{v, {V0, V1}, w});
      }
    }
    // With a single element, we expect all vertices to be classified.
    if (static_cast<int>(edge_vertices.size()) != slice_mesh_W.num_vertices()) {
      return {{},
              ::testing::AssertionFailure()
                  << "Only " << edge_vertices.size() << " slice vertices were "
                  << "mapped to tet edges; expected "
                  << slice_mesh_W.num_vertices()};
    }
    // Every edge I've identified should be located in the clipping algorithm's
    // cache: cut_edges_. Make sure they line up.
    for (const auto& edge_vertex : edge_vertices) {
      const SortedPair<int>& computed_edge = edge_vertex.edge;
      if (cut_edges_.count(computed_edge) == 0) {
        return {{},
                ::testing::AssertionFailure()
                    << "We matched surface vertex " << edge_vertex.slice_vertex
                    << " to edge " << computed_edge
                    << ", but that can't be found in the cut edge cache"};
      }
      // The matched volume edge appears in the cache; confirm the cache
      // associates it with the surface vertex index that this test matched.
      if (cut_edges_.at(computed_edge) != edge_vertex.slice_vertex) {
        return {{},
                ::testing::AssertionFailure()
                    << "We matched surface vertex " << edge_vertex.slice_vertex
                    << " to edge " << computed_edge
                    << ", but the cut edge cache has surface vertex "
                    << cut_edges_.at(computed_edge)};
      }
    }
    return {edge_vertices, ::testing::AssertionSuccess()};
  }

  /* Confirms that the pressures stored in surface_pressure_ can be reproduced
   by linearly interpolating pressure values on the `tet` based on the
   data in `edge_vertices`. */
  ::testing::AssertionResult PressuresMatchVertices(
      const PolygonSurfaceMeshFieldLinear<double, double>& field_W,
      int tet_index,
      const vector<EdgeVertex>& edge_vertices,
      const VolumeMeshFieldLinear<double, double>& field_F) const {
    constexpr double kEps = 32 * std::numeric_limits<double>::epsilon();
    for (const auto& edge_vertex : edge_vertices) {
      // Compute expected pressure.
      const auto [V0, V1] = edge_vertex.edge;
      const double p0 = field_F.EvaluateAtVertex(V0);
      const double p1 = field_F.EvaluateAtVertex(V1);
      // Combine using the weight as documented in EdgeVertex.
      const double expected_pressure =
          p0 * edge_vertex.weight + (1 - edge_vertex.weight) * p1;
      const int test_vertex_index = edge_vertex.slice_vertex;
      const double pressure = field_W.EvaluateAtVertex(test_vertex_index);
      if (std::abs(pressure - expected_pressure) > kEps) {
        return ::testing::AssertionFailure()
               << "\nSurface vertex " << test_vertex_index
               << " has wrong pressure\n"
               << " Found    " << pressure << "\n"
               << " Expected " << expected_pressure;
      }
    }

    return ::testing::AssertionSuccess();
  }

  /* This tests the mesh created after a _single_ call to SliceTetWithPlane()
   against the given _volume_ mesh and confirms they are consistent.

   Which mesh builder is used is arbitrary, but this test code needs to know so
   that it can recognize successful properties. In this case, the resulting mesh
   data will include a single polygon, wholly contained by the tet indicated by
   tet_index.

   The following is tested:

      1. Proper intersection:
         - Each vertex lies on an *edge* of the tet.
      3. Pressure evaluation:
         - Pressure at each vertex is the linear interpolation of the values
           at the vertices for that edge (using the same weight). */
  ::testing::AssertionResult SliceIsConsistent(
      const PolygonSurfaceMesh<double>& mesh_W,
      const PolygonSurfaceMeshFieldLinear<double, double>& field_W,
      int tet_index,
      const VolumeMeshFieldLinear<double, double>& field_F) const {
    // Confirms that each perimeter vertex lies on an edge of the tetrahedron.
    const auto [edge_vertices, result] =
        CharacterizeEdgeVertices(mesh_W, tet_index, field_F.mesh());
    if (!result) return result;

    return PressuresMatchVertices(field_W, tet_index, edge_vertices, field_F);
  }

  /* Computes the centroid of the indicated tet in the given mesh. */
  static Vector3d ComputeTetCentroid(const VolumeMesh<double>& mesh_M,
                                     const VolumeElement& tet) {
    // Accumulator for the position of the centroid C in the mesh frame M.
    Vector3d p_MC = Vector3d::Zero();
    for (int i = 0; i < 4; ++i) {
      const Vector3d& p_MVi = mesh_M.vertex(tet.vertex(i));
      p_MC += p_MVi;
    }
    return p_MC / 4.0;
  }

  /* Confirms that the normal implied by the face indicated by `face_index`
   matches the given expected normal. */
  ::testing::AssertionResult FaceNormalMatches(
      const PolygonSurfaceMesh<double>& mesh_W, int face_index,
      const Vector3d& expected_n_W) const {
    const Vector3d& n_W = mesh_W.face_normal(face_index);
    constexpr double kEps = 32 * std::numeric_limits<double>::epsilon();
    return CompareMatrices(expected_n_W, n_W, kEps);
  }

  /* The operands for the SliceTetWithPlane() method. */
  PolyMeshBuilder<double> builder_W_;
  unique_ptr<VolumeMesh<double>> volume_mesh_F_;
  unique_ptr<VolumeMeshFieldLinear<double, double>> field_F_;
  RigidTransformd X_WF_;
  unordered_map<SortedPair<int>, int> cut_edges_;
};

/* This tests the *boundaries* of intersection. Confirms that a tet lying
 completely on either side of the plane produces no mesh artifacts. Then, with
 a small perturbation confirms that artifacts *are* created, properly bounding
 the sensitivity between intersecting and not intersecting. The correctness of
 the values of those artifacts can be found in subsequent tests.

 We confirm intersection/separation by confirming that we have zero faces
 when separated and a non-zero number of faces when intersecting. The exact
 value of that non-zero value is not important to the test -- merely that it is
 non-zero, indicating intersection. */
TEST_F(SliceTetWithPlaneTest, NonIntersectingConfiguration) {
  // This uses a plane whose normal is aligned with the Mz axis. To assess the
  // thresholds of detecting intersection and not, we need to know the extrema
  // of the tet in that direction (its minimum and maximum points). We'll call
  // those points Max and Min, respectively. By definition we know that
  // the maximum point is at <0, 0, 1> (vertex 3) and the minimum point is any
  // point on the bottom face (we'll pick <0, 0, 0>).
  const Vector3d p_MMax{0, 0, 1};
  const Vector3d p_MMin{0, 0, 0};
  constexpr double kEps = 4 * std::numeric_limits<double>::epsilon();

  vector<RigidTransformd> X_FMs{
      RigidTransformd::Identity(),
      RigidTransformd{RotationMatrixd{AngleAxisd{
                          8 * M_PI / 7, Vector3d{1, 2, 3}.normalized()}},
                      Vector3d{-2.3, -4.2, 3.7}}};
  for (const auto& X_FM : X_FMs) {
    const Vector3d& Mz_F = X_FM.rotation().col(2);
    // The minimum and maximum points of the tet in frame F.
    const Vector3d p_FMax = X_FM * p_MMax;
    const Vector3d p_FMin = X_FM * p_MMin;
    // A small offset in the normal direction, expressed in Frame F.
    const Vector3d offset_F = kEps * Mz_F;

    // Case: Plane lies completely above the tet (just beyond V3)
    {
      const auto [mesh_W, field_W] =
          CallSliceTetWithPlane(Plane<double>{Mz_F, p_FMax + offset_F}, X_FM);
      EXPECT_TRUE(HasNVertices(*mesh_W, *field_W, 0));
    }

    // Case: Plane lies _almost_ completely above the tet (V3 penetrates the
    // plane).
    {
      const auto [mesh_W, field_W] =
          CallSliceTetWithPlane(Plane<double>{Mz_F, p_FMax - offset_F}, X_FM);
      EXPECT_TRUE(HasNVertices(*mesh_W, *field_W, 3));
    }

    // Case: Plane lies completely below the tet (the bottom faces lies on the
    // z = 0 plane in Frame M).
    {
      const auto [mesh_W, field_W] =
          CallSliceTetWithPlane(Plane<double>{Mz_F, p_FMin - offset_F}, X_FM);
      EXPECT_TRUE(HasNVertices(*mesh_W, *field_W, 0));
    }

    // Case: Plane lies _almost_ completely below the tet.
    {
      const auto [mesh_W, field_W] =
          CallSliceTetWithPlane(Plane<double>{Mz_F, p_FMin + offset_F}, X_FM);
      EXPECT_TRUE(HasNVertices(*mesh_W, *field_W, 3));
    }

    // Tests in which the plane normal is _not_ the Mz axis. We'll align it so
    // that it is normal to the v1, v2, v3 plane. We happen to know that
    // that normal (expressed in M) points in the <1, 1, 1> direction.
    const Vector3d normal_F = X_FM.rotation() * Vector3d{1, 1, 1}.normalized();

    // Case: Plane is just beyond the v1, v2, v3, plane; it passes through a
    // point _near_ V3, but just offset in the normal direction.
    {
      const auto [mesh_W, field_W] = CallSliceTetWithPlane(
          Plane<double>{normal_F, p_FMax + kEps * normal_F}, X_FM);
      EXPECT_TRUE(HasNVertices(*mesh_W, *field_W, 0));
    }

    // Case: Plane intersects the tet near the v1, v2, v3, plane; it passes
    // through a point _near_ V3, but offset in the _negative_ normal direction.
    {
      const auto [mesh_W, field_W] = CallSliceTetWithPlane(
          Plane<double>{normal_F, p_FMax - 4 * kEps * normal_F}, X_FM);
      EXPECT_TRUE(HasNVertices(*mesh_W, *field_W, 3));
    }
  }
}

/* This tests the cases where the plane intersects with the tet, creating a
 triangle. It confirms the triangular polygon produced, confirms face normal
 direction, and uses the test infrastructure to evaluate the consistency of the
 set of polygons. */
TEST_F(SliceTetWithPlaneTest, TriangleIntersections) {
  // There are eight unique configurations for a plane-tet intersection to be
  // a triangle. If a single vertex of the tet lies on one side of the plane
  // and the other three lie on the opposite side, that forms a triangle. There
  // are four such permutations. We double that to eight by reversing the plane
  // normal.
  //
  // The mesh vertices are "nicely" positioned along frame M's axes. Because
  // the calculations are performed in the mesh's frame, we want to make sure
  // that the plane normal is not *also* perfectly aligned with those axes --
  // we want to avoid a proliferation of zeros in that vector. We can avoid this
  // by computing the centroid for the tet. It will be located inside the tet's
  // volume and vectors pointing from that centroid to the tet vertices will
  // *not* be axis aligned (at least for the test mesh, if not generally). Then
  // we simply need to position the plane such that we partition the vertices as
  // expected.

  const int tet_index{0};

  vector<RigidTransformd> X_FMs{
      RigidTransformd::Identity(),
      RigidTransformd{RotationMatrixd{AngleAxisd{
                          9 * M_PI / 7, Vector3d{1, 2, 3}.normalized()}},
                      Vector3d{-2.3, -4.2, 3.7}}};
  for (const auto& X_FM : X_FMs) {
    VolumeMesh<double> mesh_F = TrivialVolumeMesh(X_FM);
    const VolumeElement& tet = mesh_F.element(tet_index);

    // Position of the tet centroid C, measured and expressed in frame F.
    const Vector3d p_FC = ComputeTetCentroid(mesh_F, tet);

    for (int i = 0; i < 4; ++i) {
      // The position of the "isolated" vertex -- the lone vertex lying on one
      // side of the plane.
      const Vector3d& p_FVi = mesh_F.vertex(tet.vertex(i));
      const Vector3d p_CVi_F = p_FVi - p_FC;
      const Vector3d nhat_F = p_CVi_F.normalized();
      // Define a point P halfway between isolated vertex Vi and centroid; the
      // plane will pass through this point. Measure and express it in frame F.
      const Vector3d p_FP = p_FC + 0.5 * p_CVi_F;

      // We consider both cases for this plane -- where we reverse the
      // definition of the plane so that once the isolated vertex has a
      // negative signed distance, and once a positive distance.
      for (const auto plane_sign : {1.0, -1.0}) {
        Plane<double> plane_F{plane_sign * nhat_F, p_FP};

        // Confirm configuration.
        // The isolated vertex is on the side of the plane expected.
        EXPECT_GT(plane_sign * plane_F.CalcHeight(p_FVi), 0.0);
        for (int j = 0; j < 4; ++j) {
          if (i != j) {
            const Vector3d& p_FVj = mesh_F.vertex(tet.vertex(j));
            // All other vertices are on the opposite side of the plane.
            EXPECT_LT(plane_sign * plane_F.CalcHeight(p_FVj), 0);
          }
        }
        const auto [mesh_W, field_W] = CallSliceTetWithPlane(plane_F, X_FM);
        ASSERT_TRUE(SliceIsConsistent(*mesh_W, *field_W, 0, *field_F_));

        // Further consistency analysis.
        ASSERT_TRUE(HasNVertices(*mesh_W, *field_W, 3));
        const Vector3d nhat_W = X_WF_.rotation() * plane_F.normal();
        EXPECT_TRUE(FaceNormalMatches(*mesh_W, 0 /* poly index */, nhat_W));
      }
    }
  }
}

/* Similar for the TriangleIntersections test, but for intersections that form
 quads. */
TEST_F(SliceTetWithPlaneTest, QuadIntersections) {
  /* A plane forms a quad when intersecting a tet when two vertices are on one
   side and two are on the other. There are three ways to partition the four
   vertices in this manner.

       (0, 1) | (2, 3)
       (0, 2) | (1, 3)
       (0, 3) | (1, 2)

   Each row in the table leads to two configurations: one where the left-hand
   column is above the plane, and one where it is below the plane. Therefore,
   there are a total of six configurations which intersect as a quad and we can
   iterate through them by iterating through the entries of the left-hand
   column and perform the slicing operation twice. Each slice uses a plane that
   spans the same space, but has normals pointing in opposite directions.

   Similar to how we did it in the triangle intersection, we'll compute the
   tet centroid. The plane normal will point from centroid to the *nearest*
   point on the corresponding edge and the plane will pass through a point
   midway between centroid and the nearest point on the edge. */

  const int tet_index{0};

  vector<RigidTransformd> X_FMs{
      RigidTransformd::Identity(), RigidTransformd{Vector3d{1.5, 2.5, 3.5}},
      RigidTransformd{RotationMatrixd{AngleAxisd{
          9 * M_PI / 7, Vector3d{1, 2, 3}.normalized()}},
                      Vector3d{-2.3, -4.2, 3.7}}};
  for (const auto& X_FM : X_FMs) {
    VolumeMesh<double> mesh_F = TrivialVolumeMesh(X_FM);
    const VolumeElement& tet = mesh_F.element(tet_index);

    // Position of the tet centroid C, measured and expressed in frame F.
    const Vector3d p_FC = ComputeTetCentroid(mesh_F, tet);

    // Given an edge defined by vertices A and B, defines the point on that
    // edge closest to Q. This *assumes* that Q projects onto the line segment
    // spanned by A and B -- valid for this context; every point internal to
    // a tet in the trivial mesh projects onto all six edges.
    auto nearest_point_to_edge = [](const Vector3d& p_FQ, const Vector3d& p_FA,
        const Vector3d& p_FB) -> Vector3d {
      const Vector3d p_AQ_F = p_FQ - p_FA;
      const Vector3d p_AB_F = p_FB - p_FA;
      return p_FA + p_AB_F.dot(p_AQ_F) / p_AB_F.squaredNorm() * p_AB_F;
    };

    // (0, 1), (0, 2), (0, 3) are the "right" vertex pairs such that we can
    // iterate through all six permutations. For example, we don't list the pair
    // (1, 2) because it is the "complement" of (0, 3). Same argument applies
    // to every other unique pair being a complement of one of those three.
    for (const auto& [v0, v1] :
         vector<pair<int, int>>{{0, 1}, {0, 2}, {0, 3}}) {
      const Vector3d& p_FV0 = mesh_F.vertex(tet.vertex(v0));
      const Vector3d& p_FV1 = mesh_F.vertex(tet.vertex(v1));
      // Position on edge E nearest the centroid.
      const Vector3d p_FE = nearest_point_to_edge(p_FC, p_FV0, p_FV1);
      const Vector3d p_CE_F = p_FE - p_FC;
      const Vector3d nhat_F = p_CE_F.normalized();
      // Define a point halfway between isolated vertex Vi and centroid; the
      // plane will pass through this point. Measure and express it in frame F.
      const Vector3d p_FP = p_FC + 0.5 * p_CE_F;

      // We consider both cases for this plane -- where we reverse the
      // definition of the  plane so that once the isolated vertex has a
      // negative signed distance, and once a positive distance.
      for (const auto plane_sign : {-1.0, 1.0}) {
        Plane<double> plane_F{plane_sign * nhat_F, p_FP};

        // Confirm configuration; all four vertices are on the expected side
        // of the plane.
        for (int i = 0; i < 4; ++i) {
          const Vector3d& p_FVi = mesh_F.vertex(tet.vertex(i));
          if (i == v0 || i == v1) {
            EXPECT_GT(plane_sign * plane_F.CalcHeight(p_FVi), 0.0);
          } else {
            EXPECT_LT(plane_sign * plane_F.CalcHeight(p_FVi), 0.0);
          }
        }

        const auto [mesh_W, field_W] = CallSliceTetWithPlane(plane_F, X_FM);
        ASSERT_TRUE(SliceIsConsistent(*mesh_W, *field_W, 0, *field_F_));

        // Further consistency analysis.
        ASSERT_TRUE(HasNVertices(*mesh_W, *field_W, 4));
        const Vector3d nhat_W = X_WF_.rotation() * plane_F.normal();
        FaceNormalMatches(*mesh_W, 0 /* poly index */, nhat_W);
      }
    }
  }
}

/* Confirms that unique vertices on the VolumeMesh produce unique vertices on
 the output (and conversely duplicate input vertices lead to duplicate output
 vertices). */
TEST_F(SliceTetWithPlaneTest, DuplicateOutputFromDuplicateInput) {
  // We create the mesh twice; once with duplicates, once without. The plane
  // will slice through *both* tets simultaneously. In both cases we should
  // end up with six triangles (one triangle per tet divided into three
  // sub-triangles each). One resulting surface will have six vertices
  // and one will have eight. We confirm that the triangles occupy the
  // same space -- i.e., the only difference is duplication of vertices.

  // We're going to do this with a single X_FM; we've already evaluated the
  // correctness of the per-tet calculation. This is about checking the
  // bookkeeping via feature counts.
  const RigidTransformd X_FM;  // Identity.

  // The infrastructure for the mesh with the minimum number of vertices.
  VolumeMesh<double> min_mesh_F =
      TrivialVolumeMesh(X_FM, true /* min_vertices */);
  VolumeMeshFieldLinear<double, double> min_field_F{
      vector<double>{0, 0, 0, 1, -1}, &min_mesh_F};
  unordered_map<SortedPair<int>, int> min_cut_edges;

  // The infrastructure for the mesh with duplicate vertices.
  VolumeMesh<double> dupe_mesh_F =
      TrivialVolumeMesh(X_FM, false /* min_vertices */);
  VolumeMeshFieldLinear<double, double> dupe_field_F{
      vector<double>{0, 0, 0, 1, 0, 0, 0, -1}, &dupe_mesh_F};
  unordered_map<SortedPair<int>, int> dupe_cut_edges;

  // The common slicing plane.
  Plane<double> plane_F{Vector3d::UnitX(), Vector3d{0.5, 0, 0}};

  TriMeshBuilder<double> min_builder_W;
  TriMeshBuilder<double> dupe_builder_W;
  for (int i = 0; i < 2; ++i) {
    SliceTetWithPlane(i, min_field_F, plane_F, X_WF_, &min_builder_W,
                      &min_cut_edges);
    SliceTetWithPlane(i, dupe_field_F, plane_F, X_WF_, &dupe_builder_W,
                      &dupe_cut_edges);
  }

  const auto [min_mesh_W, min_field_W] = min_builder_W.MakeMeshAndField();
  const auto [dupe_mesh_W, dupe_field_W] = dupe_builder_W.MakeMeshAndField();

  // We're not examining the fields.
  unused(min_field_W, dupe_field_W);

  // Both have the expected number (6) of faces.
  ASSERT_EQ(dupe_mesh_W->num_triangles(), 6);
  ASSERT_EQ(min_mesh_W->num_triangles(), 6);
  // Different vertex counts.
  ASSERT_EQ(dupe_mesh_W->num_vertices(), 8);
  ASSERT_EQ(min_mesh_W->num_vertices(), 6);
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  // We exploit knowledge of the algorithm to assume that the faces in both
  // results are in the exact same order with the constituent vertices also in
  // same order; the jth vertex of the ith triangle in each mesh should be in
  // the same location.
  for (int f = 0; f < 6; ++f) {
    const SurfaceTriangle& min_face = min_mesh_W->element(f);
    const SurfaceTriangle& dupe_face = dupe_mesh_W->element(f);
    for (int v = 0; v < 3; ++v) {
      const Vector3d& p_FVm = min_mesh_W->vertex(min_face.vertex(v));
      const Vector3d& p_FVd = dupe_mesh_W->vertex(dupe_face.vertex(v));
      EXPECT_NEAR((p_FVm - p_FVd).norm(), 0, kEps);
    }
  }

  // Confirm that all of the vertices in the duplicate mesh are referenced.
  set<int> vertex_indices;
  for (const auto& tri : dupe_mesh_W->triangles()) {
    for (int i = 0; i < 3; ++i) vertex_indices.insert(tri.vertex(i));
  }
  ASSERT_EQ(static_cast<int>(vertex_indices.size()),
            dupe_mesh_W->num_vertices());
}

/* This confirms that the structure of the query precludes "double counting". If
 the slicing plane is co-planar with a face shared by two tets, only
 intersection with _one_ of those tets will produce a result. */
TEST_F(SliceTetWithPlaneTest, NoDoubleCounting) {
  /* Arranging the tet face to be _perfectly_ coplanar with the slicing plane
   is subject to all sorts of numerical rounding issues. The simplest way to
   confirm implicit handling of this scenario is to do the test in a regime
   where all the values are perfectly represented. So, in this case, we are
   doing all of the computations in the mesh frame where the zeros and ones
   give us that perfect precision for free.

   We use a PolyMeshBuilder because we can build an "empty" mesh. */

  const Plane<double> plane_M{Vector3d::UnitZ(), Vector3d::Zero()};
  const RigidTransformd I;
  VolumeMesh<double> mesh_M = TrivialVolumeMesh(I);
  // Make an arbitrary mesh field with heterogeneous values.
  vector<double> values{0.25, 0.5, 0.75, 1, -1};
  VolumeMeshFieldLinear<double, double> field_M{move(values), &mesh_M};

  {
    // Slicing against tet 0 should intersect and produce the three faces.
    PolyMeshBuilder<double> builder_W;
    unordered_map<SortedPair<int>, int> cut_edges;
    const int tet_index = 0;
    SliceTetWithPlane(tet_index, field_M, plane_M, I, &builder_W, &cut_edges);
    const auto [mesh_W, field_W] = builder_W.MakeMeshAndField();
    // We're not examining the fields.
    unused(field_W);
    EXPECT_EQ(mesh_W->num_elements(), 1);
  }

  {
    // Slicing against tet 1 should produce no intersection.
    PolyMeshBuilder<double> builder_W;
    unordered_map<SortedPair<int>, int> cut_edges;
    const int tet_index = 1;
    SliceTetWithPlane(tet_index, field_M, plane_M, I, &builder_W, &cut_edges);
    const auto [mesh_W, field_W] = builder_W.MakeMeshAndField();
    // We're not examining the fields.
    unused(field_W);
    EXPECT_EQ(mesh_W->num_elements(), 0);
  }
}

/* Test of ComputeContactSurface(). Most of the hard-core math is contained in
 SliceTetWithPlane (and is covered by its unit tests). This function has the
 following unique responsibilities:

  1. No intersection returns nullptr.
  2. The resulting contact surface is a function of the tets provided; they
     are all considered and only those that actually intersect contribute to
     the final output.
  3. Duplicates handled correctly; if the input mesh has no duplicates, the
     result has none. Alternatively, if the input has duplicates, the output
     does as well.
  4. Confirm that the surface mesh field references the surface mesh in the
     resultant ContactSurface.
  5. Mesh normals point out of plane and into soft mesh.
  6. Report the pressure gradient of the soft mesh on the contact surface.
  7. The mesh representation of the contact surface is consistent with the
     type of MeshBuilder passed.

 These tests are done in a single, non-trivial frame F. The robustness of the
 numerics is covered in the SliceTetWithPlaneTest and this just needs to
 account for data tracking.

 ComputeContactSurface() is *largely* agnostic of the type of mesh
 representation being requested. Only responsibility 7 directly depends on that
 type and so will be tested in its own test. All other tests simply use one
 of the MeshBuilder types, inferring success for the other. */
class ComputeContactSurfaceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    X_WF_ = RigidTransformd{
        RotationMatrixd{AngleAxisd{M_PI / 2, Vector3d::UnitX()}},
        Vector3d{1.25, 2.5, -3.75}};
    X_FM_ = RigidTransformd{RotationMatrixd{AngleAxisd{
                                9 * M_PI / 7, Vector3d{1, 2, 3}.normalized()}},
                            Vector3d{-2.3, -4.2, 3.7}};

    // NOTE: all tests assume the minimum number of vertices in the mesh; no
    // duplicates.
    mesh_F_ = make_unique<VolumeMesh<double>>(
        TrivialVolumeMesh(X_FM_, true /* minimum_vertices */));
    field_F_ = make_unique<VolumeMeshFieldLinear<double, double>>(
        vector<double>{0, 0, 0, 1, -1}, mesh_F_.get());
    mesh_id_ = GeometryId::get_new_id();
    plane_id_ = GeometryId::get_new_id();
  }

  // The soft mesh and the pressure field on that mesh.
  unique_ptr<VolumeMesh<double>> mesh_F_;
  unique_ptr<VolumeMeshFieldLinear<double, double>> field_F_;
  GeometryId mesh_id_;
  GeometryId plane_id_;

  // Vectors of tet indices; to facilitate calls to ComputeContactSurface.
  const vector<int> both_tets_{0, 1};
  const vector<int> only_tet_0_{0};
  const vector<int> only_tet_1_{1};

  // Three frames:
  //   - the query frame F
  //   - the mesh frame M
  //   - the world frame W.
  // The poses X_FM and X_WF are arbitrary non-identity transforms. The full
  // robustness of the math based on these transforms is captured in the tests
  // for SliceTetWithPlane().
  RigidTransformd X_FM_;
  RigidTransformd X_WF_;
};

/* Tests responsibility 1 (listed above): if there is no intersection, nullptr
 is returned. This is tested in several ways:
   1. A scenario in which there is literally no intersection.
   2. A scenario in which there is no intersection with the tets actually
      provided. */
TEST_F(ComputeContactSurfaceTest, NoIntersectionReturnsNullPtr) {
  const Vector3d& Mz_F = X_FM_.rotation().col(2);
  {
    // Case: plane slices through *nothing*; but we'll test all tets.
    const Vector3d p_FB = X_FM_.translation() + 1.5 * Mz_F;
    const Plane<double> plane_F{Mz_F, p_FB};
    EXPECT_EQ(ComputeContactSurface<TriMeshBuilder<double>>(
                  mesh_id_, *field_F_, plane_id_, plane_F, both_tets_, X_WF_),
              nullptr);
  }

  {
    // Case: plane slices through tet 0, but we omit tet 0 from the test case
    // therefore no intersection is detected and nullptr is returned.
    const Vector3d p_FB = X_FM_.translation() + 0.5 * Mz_F;
    const Plane<double> plane_F{Mz_F, p_FB};
    EXPECT_EQ(ComputeContactSurface<TriMeshBuilder<double>>(
                  mesh_id_, *field_F_, plane_id_, plane_F, only_tet_1_, X_WF_),
              nullptr);
  }
}

/* Tests responsibility 2 (listed above): all tets are considered. This is
 tested by configuring a plane that intersects with both tets and show that
 calls which enumerate one tet, the other, or both, produce unique results. */
TEST_F(ComputeContactSurfaceTest, AllTetsAreConsidered) {
  const Vector3d& Mx_F = X_FM_.rotation().col(0);
  // The plane's normal aligns with Mx; slices through both tets.
  const Vector3d p_FB = X_FM_.translation() + 0.5 * Mx_F;
  const Plane<double> plane_F{Mx_F, p_FB};

  // Passing in all tet indices produces the full intersection: 6 triangles.
  unique_ptr<ContactSurface<double>> contact_surface =
      ComputeContactSurface<TriMeshBuilder<double>>(
          mesh_id_, *field_F_, plane_id_, plane_F, both_tets_, X_WF_);
  ASSERT_NE(contact_surface, nullptr);
  EXPECT_EQ(contact_surface->tri_mesh_W().num_elements(), 6);
  EXPECT_EQ(contact_surface->tri_mesh_W().num_vertices(), 6);

  // Passing just one or the other produces only one tet's worth of triangles.
  auto contact_surface_0 = ComputeContactSurface<TriMeshBuilder<double>>(
      mesh_id_, *field_F_, plane_id_, plane_F, only_tet_0_, X_WF_);
  ASSERT_NE(contact_surface_0, nullptr);
  EXPECT_EQ(contact_surface_0->tri_mesh_W().num_elements(), 3);
  EXPECT_EQ(contact_surface_0->tri_mesh_W().num_vertices(), 4);
  const RigidTransformd X_MW = X_FM_.InvertAndCompose(X_WF_.inverse());
  const int last_vertex = 3;
  {
    // For tet 0, the z-value of the last vertex should be > 0.
    const Vector3d& p_WV = contact_surface_0->tri_mesh_W().vertex(last_vertex);
    const Vector3d& p_MV = X_MW * p_WV;
    EXPECT_GT(p_MV(2), 0);
  }

  auto contact_surface_1 = ComputeContactSurface<TriMeshBuilder<double>>(
      mesh_id_, *field_F_, plane_id_, plane_F, only_tet_1_, X_WF_);
  ASSERT_NE(contact_surface_1, nullptr);
  EXPECT_EQ(contact_surface_1->tri_mesh_W().num_elements(), 3);
  EXPECT_EQ(contact_surface_1->tri_mesh_W().num_vertices(), 4);
  {
    // For tet 1, the z-value of the last vertex should be < 0.
    const Vector3d& p_WV = contact_surface_1->tri_mesh_W().vertex(last_vertex);
    const Vector3d& p_MV = X_MW * p_WV;
    EXPECT_LT(p_MV(2), 0);
  }
}

/* Tests responsibility 3: no duplicates are introduced. While this is implied
 by the *counts* of vertices observed in other tests, this one explicitly
 compares vertex values to make sure they are reasonably distant. */
TEST_F(ComputeContactSurfaceTest, DuplicatesHandledProperly) {
  const Vector3d& Mx_F = X_FM_.rotation().col(0);
  // The plane's normal aligns with Mx; slices through both tets.
  const Vector3d p_FB = X_FM_.translation() + 0.5 * Mx_F;
  const Plane<double> plane_F{Mx_F, p_FB};

  /* Contact surface would look like this:

          z
          ┆
          ○
          ┃╲                 The two triangles are formed by the slicing plane.
          ┃ ╲                They are subdivided into three triangles each using
          ┃○ ╲               each triangle's centroid. The two vertices on the
          ┃   ╲              y axis (marked with ●) are shared when the input
   ┄┄┄┄┄┄┄●┄┄┄┄●┄┄┄ y        mesh has no duplicates, and are duplicated when
          ┃   ╱              the input mesh has duplicates, leading to an
          ┃○ ╱               expected 6 or 8 vertices, respectively.
          ┃ ╱
          ┃╱
          ○
          ┆
  */
  {
    // Case 1: we operate on a mesh without duplicates; we get a contact surface
    // without duplicates.

    // Passing in all tet indices produces the full intersection: 6 triangles.
    unique_ptr<ContactSurface<double>> contact_surface =
        ComputeContactSurface<TriMeshBuilder<double>>(
            mesh_id_, *field_F_, plane_id_, plane_F, both_tets_, X_WF_);
    ASSERT_NE(contact_surface, nullptr);
    const TriangleSurfaceMesh<double>& contact_mesh_W =
        contact_surface->tri_mesh_W();
    EXPECT_EQ(contact_mesh_W.num_elements(), 6);
    EXPECT_EQ(contact_mesh_W.num_vertices(), 6);

    // O(N^2) test comparing all vertex distances; report the minimum distance.
    double min_distance = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 5; ++i) {
      const Vector3d& p_WVi = contact_mesh_W.vertex(i);
      for (int j = i + 1; j < 6; ++j) {
        const Vector3d& p_WVj = contact_mesh_W.vertex(j);
        min_distance = std::min(min_distance, (p_WVj - p_WVi).norm());
      }
    }
    // We pick an arbitrary number related to the scale of the tetrahedra and
    // where the plane slices through the mesh; the point is that it is much,
    // much larger than "zero".
    EXPECT_GT(min_distance, 0.1);
  }

  {
    // Case 2: we operate on a mesh _with_ duplicates; we get a contact surface
    // with duplicates.
    const VolumeMesh<double> dupe_mesh_F = TrivialVolumeMesh(X_FM_, false);
    VolumeMeshFieldLinear<double, double> dupe_field_F{
        vector<double>{0, 0, 0, 1, 0, 0, 0, -1}, &dupe_mesh_F};

    // Passing in all tet indices produces the full intersection: 6 triangles.
    unique_ptr<ContactSurface<double>> contact_surface =
        ComputeContactSurface<TriMeshBuilder<double>>(
            mesh_id_, dupe_field_F, plane_id_, plane_F, both_tets_, X_WF_);
    ASSERT_NE(contact_surface, nullptr);
    const TriangleSurfaceMesh<double>& contact_mesh_W =
        contact_surface->tri_mesh_W();
    EXPECT_EQ(contact_mesh_W.num_elements(), 6);
    EXPECT_EQ(contact_mesh_W.num_vertices(), 8);

    // We'll count the number of duplicates (based on distance between
    // vertices). We expect two vertices to be duplicated.
    constexpr double kEps = std::numeric_limits<double>::epsilon();
    int duplicate_count = 0;
    for (int i = 0; i < 5; ++i) {
      const Vector3d& p_WVi = contact_mesh_W.vertex(i);
      for (int j = i + 1; j < 6; ++j) {
        const Vector3d& p_WVj = contact_mesh_W.vertex(j);
        if ((p_WVj - p_WVi).norm() < kEps) ++duplicate_count;
      }
    }
    EXPECT_EQ(duplicate_count, 2);
  }
}

/* Tests responsibility 4: the returned contact surface has a pressure field
 that references its own mesh. */
TEST_F(ComputeContactSurfaceTest, ContactSurfaceFieldReferencesMesh) {
  // The plane slices through tet 0 creating a single triangle (decomposed into
  // three in the output).
  const Vector3d& Mz_F = X_FM_.rotation().col(2);
  const Vector3d p_FB = X_FM_.translation() + 0.5 * Mz_F;
  const Plane<double> plane_F{Mz_F, p_FB};
  unique_ptr<ContactSurface<double>> contact =
      ComputeContactSurface<TriMeshBuilder<double>>(
          mesh_id_, *field_F_, plane_id_, plane_F, both_tets_, X_WF_);
  const auto& mesh_W = contact->tri_mesh_W();
  const auto& field_W = contact->tri_e_MN();
  EXPECT_EQ(&mesh_W, &field_W.mesh());
}

/* Tests responsibility 5: the normals point out of the plane and into the
 volume mesh. */
TEST_F(ComputeContactSurfaceTest, NormalsInPlaneDirection) {
  // The plane slices through tet 0 creating a single triangle (decomposed into
  // three in the output).
  const Vector3d& Mz_F = X_FM_.rotation().col(2);
  const Vector3d p_FB = X_FM_.translation() + 0.5 * Mz_F;
  const Plane<double> plane_F{Mz_F, p_FB};

  // We'll evaluate it with two id configurations. We want to make sure that
  // the normals are correct, regardless of relationship between ids.
  vector<pair<GeometryId, GeometryId>> ids{{mesh_id_, plane_id_},
                                           {plane_id_, mesh_id_}};
  for (const auto& [id_A, id_B] : ids) {
    unique_ptr<ContactSurface<double>> contact =
        ComputeContactSurface<TriMeshBuilder<double>>(
            id_A, *field_F_, id_B, plane_F, both_tets_, X_WF_);

    const double normal_sign = contact->id_N() == id_B ? 1.0 : -1.0;
    const Vector3d nhat_W = X_WF_.rotation() * (normal_sign * Mz_F);
    // NOTE: When we set the normals directly from the plane, this precision
    // will improve.
    constexpr double kEps = 64 * std::numeric_limits<double>::epsilon();
    const TriangleSurfaceMesh<double>& mesh_W = contact->tri_mesh_W();
    for (int f = 0; f < mesh_W.num_elements(); ++f) {
      EXPECT_TRUE(CompareMatrices(mesh_W.face_normal(f), nhat_W, kEps));
    }
  }
}

/* Tests responsibility 6: the gradient of the mesh's pressure field is supplied
 in the resulting contact surface. We make sure there's contact across multiple
 tetrahedra so that we can confirm that unique gradients are reported.  */
TEST_F(ComputeContactSurfaceTest, GradientConstituentPressure) {
  ASSERT_LT(mesh_id_, plane_id_);

  const Vector3d& Mx_F = X_FM_.rotation().col(0);
  // The plane's normal aligns with Mx; slices through both tets.
  const Vector3d p_FB = X_FM_.translation() + 0.5 * Mx_F;
  const Plane<double> plane_F{Mx_F, p_FB};

  const Vector3d& grad_eMesh_W_expected0 =
      X_WF_.rotation() * field_F_->EvaluateGradient(0);
  const Vector3d& grad_eMesh_W_expected1 =
      X_WF_.rotation() * field_F_->EvaluateGradient(1);

  // Case: plane slices through both tets. The resulting surface should have
  // six triangles. The gradient for pressure field on M should be defined, N
  // is not. The gradient reported on the first three triangles is that of tet
  // 0, and for the last three should be tet 1.
  {
    unique_ptr<ContactSurface<double>> contact_surface =
        ComputeContactSurface<TriMeshBuilder<double>>(
            mesh_id_, *field_F_, plane_id_, plane_F, both_tets_, X_WF_);
    ASSERT_NE(contact_surface, nullptr);
    EXPECT_EQ(contact_surface->tri_mesh_W().num_elements(), 6);
    EXPECT_EQ(contact_surface->tri_mesh_W().num_vertices(), 6);
    EXPECT_TRUE(contact_surface->HasGradE_M());
    EXPECT_FALSE(contact_surface->HasGradE_N());
    for (int f = 0; f < 3; ++f) {
      EXPECT_TRUE(CompareMatrices(contact_surface->EvaluateGradE_M_W(f),
                                  grad_eMesh_W_expected0));
    }
    for (int f = 3; f < 6; ++f) {
      EXPECT_TRUE(CompareMatrices(contact_surface->EvaluateGradE_M_W(f),
                                  grad_eMesh_W_expected1));
    }
  }

  // Case: Reversing the ids will swap M and N. Otherwise all results should
  // be the same.
  {
    unique_ptr<ContactSurface<double>> contact_surface =
        ComputeContactSurface<TriMeshBuilder<double>>(
            plane_id_, *field_F_, mesh_id_, plane_F, both_tets_, X_WF_);
    ASSERT_NE(contact_surface, nullptr);
    EXPECT_EQ(contact_surface->tri_mesh_W().num_elements(), 6);
    EXPECT_EQ(contact_surface->tri_mesh_W().num_vertices(), 6);
    EXPECT_FALSE(contact_surface->HasGradE_M());
    EXPECT_TRUE(contact_surface->HasGradE_N());
    for (int f = 0; f < 3; ++f) {
      EXPECT_TRUE(CompareMatrices(contact_surface->EvaluateGradE_N_W(f),
                                  grad_eMesh_W_expected0));
    }
    for (int f = 3; f < 6; ++f) {
      EXPECT_TRUE(CompareMatrices(contact_surface->EvaluateGradE_N_W(f),
                                  grad_eMesh_W_expected1));
    }
  }
}

/* Tests responsibility 7: confirms that the mesh type respects the requested
 type: poly or tri. */
TEST_F(ComputeContactSurfaceTest, MeshRepresentation) {
  // The plane slices through tet 0 creating a single triangular polygon.
  const Vector3d& Mz_F = X_FM_.rotation().col(2);
  const Vector3d p_FB = X_FM_.translation() + 0.5 * Mz_F;
  const Plane<double> plane_F{Mz_F, p_FB};

  unique_ptr<ContactSurface<double>> tri_contact =
      ComputeContactSurface<TriMeshBuilder<double>>(
          mesh_id_, *field_F_, plane_id_, plane_F, both_tets_, X_WF_);
  unique_ptr<ContactSurface<double>> poly_contact =
      ComputeContactSurface<PolyMeshBuilder<double>>(
          mesh_id_, *field_F_, plane_id_, plane_F, both_tets_, X_WF_);

  EXPECT_EQ(tri_contact->representation(),
            HydroelasticContactRepresentation::kTriangle);
  EXPECT_EQ(poly_contact->representation(),
            HydroelasticContactRepresentation::kPolygon);
}

/* Test of ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(). This function
 has the following unique responsibilities:

  1. Simply that all of the values contribute to the output (poses, ids, etc.)
  2. Representation request is respected. */
GTEST_TEST(MeshPlaneIntersectionTest, SoftVolumeRigidHalfSpace) {
  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  // Create mesh and volume mesh.
  const VolumeMesh<double> mesh_F = TrivialVolumeMesh(RigidTransformd{});
  const VolumeMeshFieldLinear<double, double> field_F{
      vector<double>{0.25, 0.5, 0.75, 1, -1}, &mesh_F};
  const Bvh<Obb, VolumeMesh<double>> bvh_F(mesh_F);

  // We'll pose the plane in the soft mesh's frame S and then transform the
  // whole system.
  const RigidTransformd X_WS{
      RotationMatrixd{AngleAxisd{M_PI / 3, Vector3d{-1, 1, 1}.normalized()}},
      Vector3d{1.25, 2.5, -3.75}};

  // We want the plane to cut through both tets simultaneously. So, orient its
  // normal Rz in the Sx (via rotation around Sy). We then position it 0.5 units
  // in the Sx direction (displacement in the Sy and Sz directions have no
  // affect on the final answer as it is displacement on the plane).
  const RigidTransformd X_SR{
      RotationMatrixd{AngleAxisd{M_PI / 2, Vector3d::UnitY()}},
      Vector3d{0.5, 2.25, -1.25}};
  const RigidTransformd X_WR = X_WS * X_SR;

  constexpr double kEps = 16 * std::numeric_limits<double>::epsilon();

  {
    // Case 1: in initial configuration, we get a contact surface with
    // appropriate contact info. We won't exhaustively test the data, relying
    // on previous tests to prove correctness. We're just looking for positive
    // indicators.
    auto contact_surface = ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(
        id_A, field_F, bvh_F, X_WS, id_B, X_WR,
        HydroelasticContactRepresentation::kTriangle);
    EXPECT_TRUE(contact_surface->HasGradE_M());
    EXPECT_FALSE(contact_surface->HasGradE_N());
    ASSERT_NE(contact_surface, nullptr);
    // We exploit the knowledge that id_M < id_N and id_A < id_B.
    ASSERT_LT(id_A, id_B);
    EXPECT_EQ(contact_surface->id_M(), id_A);
    EXPECT_EQ(contact_surface->id_N(), id_B);
    EXPECT_EQ(contact_surface->tri_mesh_W().num_elements(), 6);
    // Sample the face normals.
    const Vector3d& norm_W =
        contact_surface->tri_mesh_W().face_normal(0);
    const Vector3d& Sx_W = X_WS.rotation().col(0);
    EXPECT_TRUE(CompareMatrices(norm_W, Sx_W, kEps));
    // Sample the vertex positions: in the S frame they should all have x = 0.5.
    const Vector3d& p_WV =
        contact_surface->tri_mesh_W().vertex(0);
    const Vector3d p_SV = X_WS.inverse() * p_WV;
    EXPECT_NEAR(p_SV(0), 0.5, kEps);
  }

  // Subsequent tests will not test all the properties of the resulting contact
  // surface. We assume that the initial test indicates the values come through.

  {
    // Case 2: Move the mesh out of intersection.
    const RigidTransformd X_SM{Vector3d{-0.51, 0, 0}};
    auto contact_surface = ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(
        id_A, field_F, bvh_F, X_WS * X_SM, id_B, X_WR,
        HydroelasticContactRepresentation::kTriangle);
    ASSERT_EQ(contact_surface, nullptr);
  }

  {
    // Case 3: Move the plane out of intersection.
    const RigidTransformd X_SR2{X_SR.rotation(), Vector3d{1.01, 0, 0}};
    auto contact_surface = ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(
        id_A, field_F, bvh_F, X_WS, id_B, X_WS * X_SR2,
        HydroelasticContactRepresentation::kTriangle);
    ASSERT_EQ(contact_surface, nullptr);
  }

  {
    // Case 4: Reverse GeometryIds.
    auto contact_surface = ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(
        id_B, field_F, bvh_F, X_WS, id_A, X_WR,
        HydroelasticContactRepresentation::kTriangle);
    ASSERT_NE(contact_surface, nullptr);
    EXPECT_EQ(contact_surface->id_M(), id_A);
    EXPECT_EQ(contact_surface->id_N(), id_B);
    // The effect of reversing the labels reverses the normals, so repeat the
    // normal test, but in the opposite direction.
    const Vector3d& norm_W =
        contact_surface->tri_mesh_W().face_normal(0);
    const Vector3d& Sx_W = X_WS.rotation().col(0);
    EXPECT_TRUE(CompareMatrices(norm_W, -Sx_W, kEps));
  }

  {
    // Case 5: Request polygon mesh.
    auto contact_surface = ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(
        id_A, field_F, bvh_F, X_WS, id_B, X_WR,
        HydroelasticContactRepresentation::kPolygon);
    EXPECT_EQ(contact_surface->representation(),
              HydroelasticContactRepresentation::kPolygon);
  }
}

/* This test fixture enables some limited testing of the autodiff-valued contact
 surface. It computes the intersection between a plane and simple tetrahedral
 mesh (single tet).

 We define the plane to be arbitrarily oriented and positioned in the world
 frame.

 The volume mesh is a single tetrahedron with vertices at (0, 0, 0),
 (1, 0, 0), (0, 1, 0), and (0, 0, 1) in the soft mesh frame S.

              Sz
              ┆   ╱
           v3 ●  ╱
              ┆ ╱
           v0 ┆╱    v2
     ┄┄┄┄┄┄┄┄┄●┄┄┄┄┄┄┄●┄┄┄ Sy
             ╱┆
            ╱ ┆
        v1 ●  ┆
          ╱   ┆
        Sx

 We will create a number of fixed poses of the tet w.r.t. the plane:

   - horizontal slice:
   - triangle slice:
   - quad slice:

 The function TestPositionDerivative() will pose the tetrahedral mesh and
 compute a contact surface. It invokes a provided functor to assess the reported
 derivatives of some arbitrary quantity of the contact surface with respect to
 the position of the origin of frame S.

 These tests actively use the triangle mesh because the position and value of
 the polygon centroid provides a further basis for reasoning about derivatives.
 */
class MeshPlaneDerivativesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    id_S_ = GeometryId::get_new_id();
    /* The pressure field is arbitrary, but
      1. its orientation doesn't lead to culling of intersection polygons, and
      2. validation is expressed in terms of that gradient. */
    vector<VolumeElement> elements({VolumeElement(0, 1, 2, 3)});
    vector<Vector3d> vertices_S({Vector3d::Zero(), Vector3d::UnitX(),
                                 Vector3d::UnitY(), Vector3d::UnitZ()});
    mesh_S_ = make_unique<VolumeMesh<double>>(std::move(elements),
                                              std::move(vertices_S));
    field_S_ = make_unique<VolumeMeshFieldLinear<double, double>>(
        vector<double>{0.25, 0.5, 0.75, 1}, mesh_S_.get());
    bvh_S_ = std::make_unique<Bvh<Obb, VolumeMesh<double>>>(*mesh_S_);

    /* Rigid plane; tilt and offset the plane so things are interesting. */
    X_WR_ = HalfSpace::MakePose(Vector3d{1, 2, 3}.normalized(),
                                Vector3d{0.25, 0.1, -0.2})
                .cast<AutoDiffXd>();
    id_R_ = GeometryId::get_new_id();
  }

  /* Creates a rotation to align two vectors: `s` and `t` such that t = Rs.

   Note: If we want to differentiate w.r.t. orientation, it is imperative that
   `s` not point in the `t` or `-t` directions.

   @pre |s| = |t| = 1. */
  static RotationMatrixd OrientTetrahedron(const Vector3d& s_F,
                                           const Vector3d& t_F) {
    // TODO(SeanCurtis-TRI) I stole this code from characterization_utilities.cc
    //  One more re-use and we have the opportunity to refactor.

    const double cos_theta = s_F.dot(t_F);
    constexpr double kAlmostOne = 1 - std::numeric_limits<double>::epsilon();

    /* Default to identity if s and t are already aligned. */
    RotationMatrixd R_FA;
    if (cos_theta < kAlmostOne) {
      /* They aren't already parallel. */
      if (cos_theta < -kAlmostOne) {
        /* They are anti-parallel. We need a normal perpendicular to s_F;
         extract it from a valid basis. */
        const math::RotationMatrix<double> basis =
            math::RotationMatrix<double>::MakeFromOneVector(s_F, 2);
        const Vector3d rhat = basis.col(0);
        R_FA = RotationMatrixd(AngleAxisd(M_PI, rhat));
      } else {
        const Vector3d rhat = s_F.cross(t_F).normalized();
        R_FA = RotationMatrixd(AngleAxisd(acos(cos_theta), rhat));
      }
    }
    return R_FA;
  }

  /* Indicator for the relative pose of the tet relative to the plane. See
   class documentation for details. */
  enum TetPose { kHorizontalSlice, kTriangleSlice, kQuadSlice };

  /* Tests for an arbitrary quantity of the contact surface against multiple
   relative poses between plane and tetrahedron. We evaluate three different
   configurations (as documented in the function). For each configuration, we
   invoke evaluate_quantity().

   @param evaluate_quantity  A function that assess some aspect of the contact
                             surface and its derivatives. It must be written
                             to account for the documented relative poses
                             between plane and mesh. The provided function
                             should make use of googletest EXPECT_* macros to
                             perform the assessment. */
  void TestPositionDerivative(
      const std::function<void(const ContactSurface<AutoDiffXd>&,
                               const RigidTransform<AutoDiffXd>&, TetPose)>
          evaluate_quantity) const {
    struct Configuration {
      std::string name;
      Vector3d p_RS_d;
      RotationMatrixd R_RS_d;
      int num_triangles{};
      TetPose pose;
    };
    vector<Configuration> configurations;

    const RigidTransformd X_WR_d = convert_to_double(X_WR_);
    const Vector3d n_R{0, 0, 1};
    // We want to make sure that p_SoRo is completely non-zero. So, we pick
    // a point N on the plane offset from its origin and position the tet
    // with respect to that point.
    const Vector3d p_RN{0.25, -0.3, 0};
    const Vector3d p_RS_d(p_RN - kDepth * n_R);

    {
      /* Leave the tetrahedron unrotated, i.e. R_RS = I. The contact mesh will
       be a "horizontal" slice of the tet: a right isosceles triangle. */
      configurations.push_back(
          {"Horizontal slice", p_RS_d, RotationMatrixd{}, 3, kHorizontalSlice});
    }

    {
      /* Vertex 0 (<0, 0, 0> in S) lies kDepth units on the negative side of the
       plane, all other vertices lie on the other side. The vector
       Sx + Sy + Sz = <1, 1, 1> points out of vertex 0 and into the opposing
       tetrahedral face. We'll align it with the plane normal. The contact
       surface will be an equilateral triangle. */
      const RotationMatrixd R_RS_d =
          OrientTetrahedron(Vector3d{1, 1, 1}.normalized(), n_R);
      configurations.push_back(
          {"Single vertex penetration", p_RS_d, R_RS_d, 3, kTriangleSlice});
    }

    {
      /* Vertices 0 and 3 (<0, 0, 1>) lie kDepth units on the negative side of
       the plane, vertices 1 & 2 on the positive side. The vector
       Sx + Sy = <1, 1, 0> is perpendicular to the edge spanned by V0 and V3
       (pointing into the tetrahedron). We'll align that with the plane normal.
       This should leave the edges (0, 3) and (1, 2) both parallel with the
       plane. */
      const RotationMatrixd R_RS_d =
          OrientTetrahedron(Vector3d{1, 1, 0}.normalized(), n_R);

      /* Reality check: confirm the edges *are* parallel with the plane. */
      const RotationMatrixd R_WR_d = convert_to_double(X_WR_).rotation();
      const RotationMatrixd R_WS_d = R_WR_d * R_RS_d;
      const Vector3d& v0_S = mesh_S_->vertex(0);
      const Vector3d& v1_S = mesh_S_->vertex(1);
      const Vector3d& v2_S = mesh_S_->vertex(2);
      const Vector3d& v3_S = mesh_S_->vertex(3);
      const Vector3d e03_S = v3_S - v0_S;
      const Vector3d e12_S = v2_S - v1_S;
      const Vector3d e03_W = R_WS_d * e03_S;
      const Vector3d e12_W = R_WS_d * e12_S;
      const Vector3d n_W_d = R_WR_d * n_R;
      EXPECT_NEAR(n_W_d.dot(e03_W), 0, 1e-15);
      EXPECT_NEAR(n_W_d.dot(e12_W), 0, 1e-15);
      configurations.push_back(
          {"Two vertex penetration", p_RS_d, R_RS_d, 4, kQuadSlice});
    }

    for (const auto& config : configurations) {
      const RotationMatrixd R_WS_d = X_WR_d.rotation() * config.R_RS_d;
      const Vector3d p_WS_d = X_WR_d * config.p_RS_d;
      const Vector3<AutoDiffXd> p_WS = math::InitializeAutoDiff(p_WS_d);
      const RigidTransform<AutoDiffXd> X_WS(R_WS_d.cast<AutoDiffXd>(), p_WS);

      auto surface = ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(
          id_S_, *field_S_, *bvh_S_, X_WS, id_R_, X_WR_,
          HydroelasticContactRepresentation::kTriangle);

      SCOPED_TRACE(config.name);
      ASSERT_NE(surface, nullptr);
      ASSERT_EQ(surface->tri_mesh_W().num_triangles(), config.num_triangles);

      evaluate_quantity(*surface, X_WS, config.pose);
    }
  }

  /* Given the point E which purports to lie on an edge of the tetrahedral mesh,
   finds the edge it lies on (spanning vertices A and B) and returns p_AB_S. */
  Vector3d GetEdgeDirInS(const Vector3d& p_SE) const {
    // We determine the edge that E lies on with this simple metric. If E lies
    // on edge AB, then AB⋅AE = |AB|⋅|AE|, or, to avoid square roots:
    // (AB⋅AE)² = |AB|²⋅|AE|².
    // This wouldn't be sufficient generally. But for this test we can make
    // two simplifying assumptions:
    //   1. E actually does lie on *one* of the mesh edges.
    //   2. The edges are easily distinguishable by direction.
    const vector<Vector3d>& verts_S = field_S_->mesh().vertices();
    const vector<pair<int, int>> edges{{0, 1}, {0, 2}, {0, 3},
                                       {1, 2}, {1, 3}, {2, 3}};
    for (const auto& [a, b] : edges) {
      const Vector3d p_AB_S = verts_S[b] - verts_S[a];
      const Vector3d p_AE_S = p_SE - verts_S[a];
      const double lhs = std::pow(p_AB_S.dot(p_AE_S), 2);
      const double rhs = p_AB_S.squaredNorm() * p_AE_S.squaredNorm();
      if (std::abs(lhs - rhs) < 1e-15) {
        return p_AB_S;
      }
    }
    throw std::logic_error(
        "Querying for point E that isn't actually on a tet edge");
  }

  /* Soft volume mesh. */
  GeometryId id_S_;
  unique_ptr<VolumeMesh<double>> mesh_S_;
  unique_ptr<VolumeMeshFieldLinear<double, double>> field_S_;
  unique_ptr<Bvh<Obb, VolumeMesh<double>>> bvh_S_;

  /* Rigid plane. No geometry is required; it's implied by the pose. */
  RigidTransform<AutoDiffXd> X_WR_;
  GeometryId id_R_;

  /* The amount I penetrate plane into the tet.  */
  static constexpr double kDepth = 0.25;
};

TEST_F(MeshPlaneDerivativesTest, Area) {
  /* We'll compute the expected contact surface area (and its derivatives) by
   decomposing it into triangles. For a triangle defined by vertices A, B, and
   C, with triangle normal n̂ (and assuming that the triangle winding is
   consistent with the normal direction):

      Area = 0.5 * |(B - A) × (C - A)|₂
           = 0.5 * [(B - A) × (C - A)]ᵀn̂
           = 0.5 * [skew_sym(B - A) (C-A)]ᵀn̂

        where, a x b = skew_sym(a) b and

                          │  0 -a3  a2│
            skew_sym(a) = │ a3   0 -a1│
                          │-a2  a1   0│

    ∂Area          ∂[skew_sym(B - A) (C-A)]ᵀ
    ────── = 0.5 * ─────────────────── n̂
      ∂So                ∂So

                   │                  ∂(B - A)                    ∂(C - A) │ᵀ
           = 0.5 * │ -skew_sym(C - A) ────────  + skew_sym(B - A) ──────── │ n̂
                   │                    ∂So                          ∂So   │

   We are *given* the quantities ∂A/∂So, ∂B/∂So, and ∂C/∂So which have been
   independently validated by the VertexPosition test. */
  auto evaluate_area = [X_WR = convert_to_double(this->X_WR_)](
                           const ContactSurface<AutoDiffXd>& surface,
                           const RigidTransform<AutoDiffXd>& X_WS_ad,
                           TetPose pose) {
    const auto& mesh_W = surface.tri_mesh_W();

    // For v × A, this makes a matrix V such that VA = v × A.
    auto skew_matrix = [](const Vector3d& v) {
      Matrix3<double> result;
      // clang-format off
      result <<     0, -v(2),  v(1),
                 v(2),     0, -v(0),
                -v(1),  v(0),   0;
      // clang-format on
      return result;
    };

    /* We don't want to compute the areas of the *individual* triangles in the
     contact surface because it is triangle fan around a centroid. The fan and
     centroid contribute nothing to the area. So, as an independent witness,
     we'll compute the area of the *polygon* based on the vertices on the
     perimeter. The triangles defined below are based on a priori knowledge
     of the behavior of the mesh-plane intersection algorithm. If it changes,
     these hard-coded indices could become invalid. A more robust solution would
     be to *infer* the boundary polygon edges from the contact surface, but
     that's a lot of effort for little present value. */
    vector<vector<int>> triangles;
    switch (pose) {
      case TetPose::kHorizontalSlice:
      case TetPose::kTriangleSlice:
        triangles.emplace_back(vector<int>{0, 1, 2});
        break;
      case TetPose::kQuadSlice:
        /* This is a bit brittle and is predicated on knowledge of how
         the intersection algorithm processes the particular geometry. If that
         proves to be too brittle, we'll need to reconstruct this by looking
         at the provided mesh. */
        triangles.emplace_back(vector<int>{0, 1, 2});
        triangles.emplace_back(vector<int>{2, 3, 1});
        break;
    }

    /* The normal for the contact surface is simply the plane normal: Rz (here,
     expressed in world). */
    const Vector3d n_W = X_WR.rotation().col(2);

    double area_expected = 0;
    Vector3d dArea_dSo_expected = Vector3d::Zero();
    for (const auto& tri : triangles) {
      const auto& p_WA_ad = mesh_W.vertex(tri[0]);
      const auto& p_WB_ad = mesh_W.vertex(tri[1]);
      const auto& p_WC_ad = mesh_W.vertex(tri[2]);
      const Vector3d p_WA = convert_to_double(p_WA_ad);
      const Vector3d p_WB = convert_to_double(p_WB_ad);
      const Vector3d p_WC = convert_to_double(p_WC_ad);
      const Matrix3<double> dA_dSo = math::ExtractGradient(p_WA_ad);
      const Matrix3<double> dB_dSo = math::ExtractGradient(p_WB_ad);
      const Matrix3<double> dC_dSo = math::ExtractGradient(p_WC_ad);

      const Vector3d p_AB_W = p_WB - p_WA;
      const Vector3d p_AC_W = p_WC - p_WA;
      const double tri_area = 0.5 * p_AB_W.cross(p_AC_W).dot(n_W);
      area_expected += tri_area;

      const Matrix3<double> left = -skew_matrix(p_AC_W) * (dB_dSo - dA_dSo);
      const Matrix3<double> right = skew_matrix(p_AB_W) * (dC_dSo - dA_dSo);
      dArea_dSo_expected += 0.5 * ((left + right).transpose() * n_W);
    }

    constexpr double kEps = std::numeric_limits<double>::epsilon();
    const AutoDiffXd& total_area = mesh_W.total_area();
    EXPECT_NEAR(total_area.value(), area_expected, 4 * kEps);
    EXPECT_TRUE(CompareMatrices(total_area.derivatives(), dArea_dSo_expected,
                                4 * kEps));
  };

  TestPositionDerivative(evaluate_area);
}

TEST_F(MeshPlaneDerivativesTest, VertexPosition) {
  /* The vertices of the contact surface *always* lie on the plane. Some of the
   vertices come from intersecting tet edges with the plane. The remaining are
   centroids of polygons formed by the intersection. We'll deal with those
   vertices differently.

   - For vertices at the intersection of plane and tet edge, the derivative of
     the vertex position w.r.t. the position of the tetrahedron origin (So) is
     (expressed in the rigid plane's frame R):

                         | 1  0  -p.x |
          ∂p_RV/∂p_RSo = | 0  1  -p.y |
                         | 0  0   0   |

     In the rigid plane's frame, the vertices can never move *off* the plane, so
     ∂V.z/∂So_R must always be zero. ∂V.x/∂So_R and ∂V.y/∂So_R are 100% coupled
     with the movement of So in the Rx and Ry directions, respectively. So's
     movement in the Rz direction affects vertex position in the Rx and Ry
     directions based on the angle between the edge and the plane normal. The
     intersecting edge e has a component parallel to the plane and a component
     perpendicular to the plane. We define the vector p as the ratio of movement
     on the plane versus motion perpendicular to the plane:

        p = (e − (e⋅n̂)n̂)/(e⋅n̂)

     However, the derivatives reported in the test are ∂p_WV/∂p_WSo, so we have
     to transform the expected result from the rigid plane frame to world.

          ∂p_WV    ∂(R_WR⋅p_RV + p_WRo)
         ------- = --------------------          // Expand p_WV = X_WR * p_RV.
          ∂p_WSo         ∂p_WSo

                   ∂(R_WR⋅p_RV)
                 = -------------                 // p_WRo doesn't depend on So.
                      ∂p_WSo

                   ∂(R_WR⋅p_RV)     ∂p_RSo
                 = ------------- * --------      // Chain rule.
                      ∂p_RSo        ∂p_WSo

                   ∂(R_WR⋅p_RV)
                 = ------------- * R_RW          // Change of So in R is related
                      ∂p_RSo                     // to change in W by R_RW.

                           ∂p_RV
                 = R_WR * -------- * R_RW        // R_WR doesn't depend on So.
                           ∂p_RSo

                          | 1 0 -p.x |
                 = R_WR * | 0 1 -p.y | * R_RW    // Definition of ∂p_RV/∂p_RSo.
                          | 0 0  0   |

   - We treat centroid vertices differently. For centroids of triangles (where
     the intersecting polygon between tet and plane is a triangle), the centroid
     position and its derivative are simply:

                 C = (v0 + v1 + v2) / 3, and
            ∂C/∂So = (∂v0/∂So + ∂v1/∂So + ∂v2/∂So) / 3

     We skip the centroid for polygons with four or more vertices. Those
     centroids are computed by decomposing the polygon into triangles. For each
     triangle we compute centroid and area and then define the polygon centroid
     as a weighted average of the triangle centroids. There is no simple way to
     validate the derivatives of this vertex, so we'll skip it for now.

   Finally, this test exploits special knowledge that when a polygon with N
   vertices is produced by intersection, N + 1 vertices are added to the contact
   surface mesh: the polygon's N vertices followed by the centroid. In this
   test, we can use that to implicitly recognize which vertices come from edge
   intersections and which are centroids (combined with the fact that there's
   only a single polygon in the contact surface mesh). */
  auto evalute_position = [this](const ContactSurface<AutoDiffXd>& surface,
                                 const RigidTransform<AutoDiffXd>& X_WS_ad,
                                 TetPose pose) {
    constexpr double kEps = 5 * std::numeric_limits<double>::epsilon();

    /* The test is set up so there is only ever a single intersecting polygon.
     So, there is *one* centroid (the last vertex). All other vertices come
     from intersecting a tet edge with the plane. We'll evaluate all of those
     and then handle the centroid specially. */
    const Vector3d n_R{0, 0, 1};
    const TriangleSurfaceMesh<AutoDiffXd>& mesh_W = surface.tri_mesh_W();
    const RotationMatrixd R_WR = convert_to_double(this->X_WR_).rotation();
    const RotationMatrixd R_RW = R_WR.inverse();
    const RigidTransformd X_WS = convert_to_double(X_WS_ad);
    const RotationMatrixd& R_WS = X_WS.rotation();
    for (int v = 0; v < mesh_W.num_vertices() - 1; ++v) {
      const Vector3<AutoDiffXd>& p_WV_ad = mesh_W.vertex(v);
      const Vector3d& p_WV = convert_to_double(p_WV_ad);
      const Vector3d e_R = R_RW * R_WS * GetEdgeDirInS(X_WS.inverse() * p_WV);
      const double in_normal_dir = e_R.dot(n_R);
      // Reality check: We don't have edges lying on the plane.
      DRAKE_DEMAND(std::abs(in_normal_dir) > 1e-10);
      const Vector3d p = (e_R - in_normal_dir * n_R) / in_normal_dir;
      Matrix3<double> expected_J_R;
      // clang-format off
      expected_J_R << 1,  0, -p.x(),
                      0,  1, -p.y(),
                      0,  0,  0;
      // clang-format on
      const Matrix3<double> expected_J_W =
          R_WR * (expected_J_R * R_RW.matrix());
      const Matrix3<double> J_W = math::ExtractGradient(p_WV_ad);
      ASSERT_TRUE(CompareMatrices(J_W, expected_J_W, kEps));
    }

    /* Now handle the centroid. */
    switch (pose) {
      case kHorizontalSlice:
      case kTriangleSlice: {
        /* The derivative should simply be the mean of the first three. */
        Matrix3<double> expected_J_W = Matrix3<double>::Zero();
        for (int v = 0; v < 3; ++v) {
          expected_J_W += math::ExtractGradient(mesh_W.vertex(v));
        }
        expected_J_W /= 3;
        const Vector3<AutoDiffXd>& p_WC = mesh_W.vertex(3);
        const Matrix3<double> J_W = math::ExtractGradient(p_WC);
        EXPECT_TRUE(CompareMatrices(J_W, expected_J_W, kEps));
        break;
      }
      case kQuadSlice:
        /* We skip the centroid for the quad case. */
        break;
    }
  };

  TestPositionDerivative(evalute_position);
}

TEST_F(MeshPlaneDerivativesTest, FaceNormalsWrtPosition) {
  /* Face normals should always be parallel with the plane normal. */
  auto evaluate_normals = [X_WR = convert_to_double(this->X_WR_)](
                              const ContactSurface<AutoDiffXd>& surface,
                              const RigidTransform<AutoDiffXd>& X_WS, TetPose) {
    constexpr double kEps = std::numeric_limits<double>::epsilon();
    const auto& mesh_W = surface.tri_mesh_W();
    const Vector3d plane_n_W = X_WR.rotation().col(2);
    const Matrix3<double> zeros = Matrix3<double>::Zero();
    for (int f = 0; f < mesh_W.num_elements(); ++f) {
      const Vector3<AutoDiffXd>& tri_n_W = mesh_W.face_normal(f);
      EXPECT_TRUE(
          CompareMatrices(math::ExtractValue(tri_n_W), plane_n_W, 2 * kEps));
      EXPECT_TRUE(
          CompareMatrices(math::ExtractGradient(tri_n_W), zeros, 10 * kEps));
    }
  };

  TestPositionDerivative(evaluate_normals);
}

TEST_F(MeshPlaneDerivativesTest, FaceNormalsWrtOrientation) {
  /* Even if the soft frame is rotated w.r.t. the rigid frame, the contact
   surface normals should always be parallel with the plane's normal and,
   therefore, should have zero derivatives.

   This test does *not* use the TestPositionDerivative() API because that
   differentiates with respect to p_WSo and makes assumptions about the
   resulting mesh. For this test, we need a different derivative and different
   assumptions, so we'll simply duplicate that portion of
   TestPositionDerivative() that is relevant for this test. */

  /* For simplicity, we'll simply differentiate w.r.t. a single scalar: a
   rotation of θ radians around an arbitrary axis. We'll sample a few values,
   but make sure that the mesh normal and field gradient remain sufficiently
   aligned that we don't lose the triangle due to "backface culling".  */
  const Vector3d v_W = Vector3d{-1, 2, -3}.normalized();
  // Rather than aligning the tet mesh with the *origin* of the rigid frame,
  // we pick some point (N) away from the origin, but still on the plane.
  const Vector3<AutoDiffXd> p_WN =
      this->X_WR_ * Vector3<AutoDiffXd>{0.25, -0.3, 0};
  const Vector3<AutoDiffXd> plane_n_W_ad = this->X_WR_.rotation().col(2);
  const Vector3d plane_n_W = math::ExtractValue(plane_n_W_ad);
  const Vector3<AutoDiffXd> p_WS = p_WN - this->kDepth * plane_n_W_ad;
  for (const double theta : {0.0, M_PI / 6, M_PI / 2 * 0.9, M_PI / 2 * 0.99}) {
    AutoDiffXd theta_ad = theta;
    theta_ad.derivatives().resize(1);
    theta_ad.derivatives() << 1;
    RigidTransform<AutoDiffXd> X_WS{
        RotationMatrix<AutoDiffXd>(AngleAxis<AutoDiffXd>(theta_ad, v_W)), p_WS};

    auto surface = ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(
        this->id_S_, *this->field_S_, *this->bvh_S_, X_WS, this->id_R_,
        this->X_WR_, HydroelasticContactRepresentation::kTriangle);

    SCOPED_TRACE(fmt::format("theta = {:.5f} radians", theta));
    ASSERT_NE(surface, nullptr);
    /* Make sure the test doesn't pass simply because we have no triangles. */
    const auto& mesh_W = surface->tri_mesh_W();
    ASSERT_GT(mesh_W.num_elements(), 0);

    constexpr double kEps = std::numeric_limits<double>::epsilon();
    for (int f = 0; f < mesh_W.num_elements(); ++f) {
      const Vector3<AutoDiffXd>& tri_n_W = mesh_W.face_normal(f);
      /* Confirm the normal direction. */
      EXPECT_TRUE(
          CompareMatrices(math::ExtractValue(tri_n_W), plane_n_W, 5 * kEps));
      /* Confirm the normal gradient w.r.t. theta. */
      EXPECT_TRUE(CompareMatrices(math::ExtractGradient(tri_n_W),
                                  Vector3d::Zero(), 10 * kEps));
    }
  }
}

TEST_F(MeshPlaneDerivativesTest, Pressure) {
  /* Because the field is a linear field, we can think of the pressure function,
   evaluated at point Q as p(Q) = ∇pᵀ(Q - E), such that ∇p is the gradient of
   the pressure field and E is a point at which the pressure field is zero.
   We'll define p_W(p_WQ) = ∇p_Wᵀ(p_WQ - p_WE) as the pressure evaluated on
   points measured and expressed in the world frame. So,

                    ∂[∇p_Wᵀ(p_WQ - p_WE)]
      ∂p_WQ/∂p_WSo = ─────────────────────
                           ∂p_WSo

                            ∂[(p_WQ - p_WE)]
                   = ∇p_Wᵀ ──────────────────      // ∇p_W const w.r.t. p_WSo.
                                 ∂p_WSo

                           ┌                   ┐
                           │ ∂p_WQ      ∂p_WE  │
                   = ∇p_Wᵀ │──────── - ────────│   // Subtraction is a linear
                           │ ∂p_WSo     ∂p_WSo │   // operator.
                           └                   ┘

                           ┌                  ┐
                           │ ∂p_WQ     │1 0 0││
                   = ∇p_Wᵀ │──────── - │0 1 0││    // E affixed to Frame S.
                           │ ∂p_WSo    │0 0 1││
                           └                  ┘

   ∂p_WQ/∂p_WSo are the derivatives that were confirmed in the VertexPosition
   test, so we can use those to compute the expected pressure derivative. */
  const Vector3d grad_p_S = field_S_->EvaluateGradient(0);
  auto evaluate_pressure = [X_WR = convert_to_double(this->X_WR_), &grad_p_S](
                               const ContactSurface<AutoDiffXd>& surface,
                               const RigidTransform<AutoDiffXd>& X_WS,
                               TetPose) {
    constexpr double kEps = 8 * std::numeric_limits<double>::epsilon();
    const RigidTransform<double> X_WS_d = convert_to_double(X_WS);
    const Vector3d grad_p_W = X_WS_d.rotation() * grad_p_S;
    for (int v = 0; v < surface.tri_mesh_W().num_vertices(); ++v) {
      const Matrix3<double> dp_WQ_dp_WSo_W =
          math::ExtractGradient(surface.tri_mesh_W().vertex(v));
      const Vector3d dp_dp_WSo_W_expected =
          grad_p_W.transpose() * (dp_WQ_dp_WSo_W - Matrix3<double>::Identity());
      const AutoDiffXd& p = surface.tri_e_MN().EvaluateAtVertex(v);
      EXPECT_TRUE(CompareMatrices(p.derivatives(), dp_dp_WSo_W_expected, kEps))
          << v << " out of " << surface.tri_mesh_W().num_vertices();
    }
  };

  TestPositionDerivative(evaluate_pressure);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
