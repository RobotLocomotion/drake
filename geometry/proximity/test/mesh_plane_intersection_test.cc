#include "drake/geometry/proximity/mesh_plane_intersection.h"

#include <limits>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
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
  vector<VolumeVertex<T>> vertices;
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

/* Compute a unit-length normal to the plane spanned by the three vertices. The
 direction of the normal will be based on right-handed winding and it is assumed
 the vertex positions are _not_ colinear. */
Vector3d CalcPlaneNormal(
    const Vector3d& p_FV0, const Vector3d& p_FV1, const Vector3d& p_FV2) {
  const Vector3d phat_V0V1 = (p_FV1 - p_FV0).normalized();
  const Vector3d phat_V0V2 = (p_FV2 - p_FV0).normalized();
  return phat_V0V1.cross(phat_V0V2).normalized();
}

/* Infrastructure for testing SliceTetWithPlane(). These tests involve three
 frames: M, F, and W. The trivial mesh (see above) is defined in its frame M.
 However, we transform that trivial mesh into some arbitrary frame F (so that
 the mesh vertices don't have a bunch of trivial zeros and ones). The
 intersecting plane is likewise defined in that same frame F. Finally, the
 result of slicing the test with the plane produces results in the *world* frame
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

  /* Performs a test of SliceTetWithPlane() by constructing a trivial mesh in
   frame F and invoking SliceTetWithPlane() using the test class's accumulator
   structures to capture the result. The various accumulators are guaranteed
   to be cleared with each invocation of this method before calling
   SliceTetWithPlane(), so the resultant state is solely attributable to the
   last invocation.

   If `do_analysis` is `true`, the mesh will be evaluated for internal
   consistency, and the return value reflects that analysis. Otherwise, it
   trivially reports success.

   @param plane_F       The slicing plane (measured and expressed in Frame F).
   @param X_FM          The relative pose between the trivial mesh's canonical
                        frame M and the query frame F.
   @param do_analysis   If `true`, evaluate the resulting intersection mesh for
                        consistency and return the result.
   @returns AssertionSuccess if `do_analysis` is `false`, otherwise the
            AssertionResult produced by the consistency analysis.
  */
  ::testing::AssertionResult CallSliceTetWithPlane(
      const Plane<double>& plane_F, const RigidTransformd& X_FM,
      bool do_analysis) {
    VolumeMesh<double> mesh_F = TrivialVolumeMesh(X_FM);
    // Make an arbitrary mesh field with heterogeneous values.
    vector<double> values{0.25, 0.5, 0.75, 1, -1};
    VolumeMeshFieldLinear<double, double> field_F{"pressure", move(values),
                                                  &mesh_F};

    faces_.clear();
    vertices_W_.clear();
    surface_pressure_.clear();
    cut_edges_.clear();
    VolumeElementIndex tet_index{0};
    SliceTetWithPlane(tet_index, field_F, plane_F, X_WF_, &faces_, &vertices_W_,
                      &surface_pressure_, &cut_edges_);
    if (do_analysis) return SliceIsConsistent(tet_index, field_F);
    return ::testing::AssertionSuccess();
  }

  /* Reports `true` if the accumulators' states are consistent with the expected
   number of faces generated by a call to `SliceTestWithPlane()`. */
  ::testing::AssertionResult HasNFaces(size_t num_faces) const {
    bool error = false;
    ::testing::AssertionResult failure = ::testing::AssertionFailure();
    auto test_features = [&failure, &error](const auto& features, size_t count,
                                            const char* label) {
      if (features.size() != count) {
        error = true;
        failure << "Wrong number of " << label << ".\n Expected: " << count
                << "\n Found: " << features.size() << "\n";
      }
    };
    test_features(faces_, num_faces, "faces");
    // For no faces, there are no vertices. But for n faces, we expect n + 1
    // vertices.
    const size_t num_verts = num_faces == 0 ? 0 : num_faces + 1;
    test_features(vertices_W_, num_verts, "vertices");
    test_features(surface_pressure_, num_verts, "pressure values");
    test_features(cut_edges_, num_faces, "cut edges");

    return error ? failure : ::testing::AssertionSuccess();
  }

  /* Determine which vertex is the center of the triangle fan; it should be the
   single vertex referenced by every face (all other vertices should be
   referenced twice). */
  static pair<SurfaceVertexIndex, ::testing::AssertionResult> IdentifyFanVertex(
      const vector<SurfaceFace>& faces) {
    const int num_faces = static_cast<int>(faces.size());
    SurfaceVertexIndex centroid_index;
    std::unordered_map<SurfaceVertexIndex, int> vertex_references;
    for (const auto& face : faces) {
      for (int i = 0; i < 3; ++i) {
        SurfaceVertexIndex v = face.vertex(i);
        // This relies on unordered_map's value initialization of the value
        // type. In this case, int gets initialized to zero and then
        // incremented.
        ++vertex_references[v];
      }
    }
    int num_perim_vertex{0};
    for (const auto& pair : vertex_references) {
      if (pair.second == 2) {
        ++num_perim_vertex;
      } else if (pair.second == num_faces) {
        // While this test could blindly accept two different vertices to be
        // classified as the centroid, that means the subsequent test on the
        // expected number of perimeter tests will fail.
        centroid_index = pair.first;
      } else {
        return {{},
                ::testing::AssertionFailure()
                    << "The surface vertex " << pair.first
                    << " is neither boundary nor centroid"};
      }
    }
    // There is one and only one centroid candidate.
    if (num_perim_vertex != num_faces) {
      return {{},
              ::testing::AssertionFailure()
                  << "There should be " << num_faces
                  << " identified perimeter vertices; "
                  << "only " << num_perim_vertex << " were identified"};
    }
    return {centroid_index, ::testing::AssertionSuccess()};
  }

  /* Struct for associating a vertex produced from a slice with the mesh edge
   that got sliced. It includes the resultant _surface_ vertex index, the
   indices of the vertices in the volume_ mesh that define the edge, and a
   weight value such that the position of the slice vertex is:
   weight * P(edge.first) + (1 - weight) * P(edge.second), where P(v) is the
   position of the volume vertex with index v.  */
  struct EdgeVertex {
    SurfaceVertexIndex slice_vertex;
    SortedPair<VolumeVertexIndex> edge;
    double weight{};
  };

  /* For each vertex in the "slice" mesh (that isn't the fan vertex), determine
   which edge in the tet it lies on and its "weight" along that edge. If the
   classification fails for any vertex, report failure (without edge data). */
  pair<vector<EdgeVertex>, ::testing::AssertionResult>
  CharacterizeEdgeVertices(
      SurfaceVertexIndex fan_index, const vector<SurfaceFace>& slice,
      const vector<SurfaceVertex<double>>& slice_vertices_W,
      VolumeElementIndex tet_index, const VolumeMesh<double>& mesh_F) const {
    constexpr double kEps = std::numeric_limits<double>::epsilon();
    const VolumeElement& tet = mesh_F.element(tet_index);

    // Enumeration of all tetrahedron edges defined by vertex pairs referenced
    // by tet-local index values.
    const int tet_edges[6][2] = {{0, 1}, {0, 2}, {0, 3},
                                 {1, 2}, {1, 3}, {2, 3}};

    const RigidTransformd X_FW = X_WF_.inverse();

    vector<EdgeVertex> edge_vertices;
    // We include the fan index in the set of vertices already processed so that
    // we don't attempt to process it (it is the one vertex we presume should
    // _not_ lie on any edge).
    set<SurfaceVertexIndex> slice_vertex_indices{fan_index};
    for (const auto& face : slice) {
      for (int i = 0; i < 3; ++i) {
        const SurfaceVertexIndex v = face.vertex(i);
        if (slice_vertex_indices.count(v) > 0) continue;
        slice_vertex_indices.insert(v);
        // The slice polygon vertex S in the mesh frame F.
        const Vector3d p_FS = X_FW * slice_vertices_W[v].r_MV();
        for (int e = 0; e < 6; ++e) {
          const VolumeVertexIndex V0 = tet.vertex(tet_edges[e][0]);
          const VolumeVertexIndex V1 = tet.vertex(tet_edges[e][1]);
          const Vector3d p_FV0 = mesh_F.vertex(V0).r_MV();
          const Vector3d p_FV1 = mesh_F.vertex(V1).r_MV();
          const Vector3d p_V0V1_F = p_FV1 - p_FV0;
          const double d_V0V1 = p_V0V1_F.norm();
          const Vector3d p_V0S_F = p_FS - p_FV0;
          const double d_V0S = p_V0S_F.norm();
          // Test that S lies on the line segment spanned by V0V1.
          //  - It is colinear with the _line_ passing through V0V1 and
          //  - its "weight" (i.e., interpolation between V0 and V1) is in the
          //    interval [0, 1].

          // We can implicitly test colinearity (and part of the the weight
          // value) by comparing the vector V0S with V0V1. A negative dot
          // product indicates that S *cannot* lie between V0 and V1 (colinear
          // or not) and a positive value equal to |V0S| * |V0V1| indicates
          // colinearity.
          if (std::abs(p_V0V1_F.dot(p_V0S_F) - d_V0V1 * d_V0S) > kEps) continue;
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
    }
    // The number of edge vertices should be equal to the number of faces.
    if (edge_vertices.size() != slice.size()) {
      return {{},
              ::testing::AssertionFailure()
                  << "Only " << edge_vertices.size() << " slice vertices were "
                  << "mapped to tet edges; expected " << slice.size()};
    }
    // Every edge I've identified should be located in the clipping algorithm's
    // cache: cut_edges_. Make sure they line up.
    for (const auto& edge_vertex : edge_vertices) {
      const SortedPair<VolumeVertexIndex>& computed_edge = edge_vertex.edge;
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

  /* Given the edge vertices (assuming they are properly ordered to form a
   coherent polygon), computes the centroid. */
  Vector3d CentroidFromFan(const vector<EdgeVertex>& edge_vertices) const {
    vector<SurfaceVertexIndex> polygon;
    for (const auto& edge_vertex : edge_vertices) {
      polygon.push_back(edge_vertex.slice_vertex);
    }
    const Vector3d nhat_W = CalcPlaneNormal(vertices_W_[polygon[0]].r_MV(),
                                            vertices_W_[polygon[1]].r_MV(),
                                            vertices_W_[polygon[2]].r_MV());
    return CalcPolygonCentroid(polygon, nhat_W, vertices_W_);
  }

  /* Confirm that the centroid is properly the geometric centroid of the
   polygon formed by the perimeter vertices. This relies on the edge
   vertices to be ordered properly (i.e., around the perimeter and not
   jumbled up). */
  ::testing::AssertionResult FanVertexIsCentroid(
      const vector<EdgeVertex>& edge_vertices,
      SurfaceVertexIndex centroid_index) const {
    constexpr double kEps = 8 * std::numeric_limits<double>::epsilon();
    const Vector3d p_WC = CentroidFromFan(edge_vertices);
    return CompareMatrices(p_WC, vertices_W_[centroid_index].r_MV(), kEps);
  }

  /* Confirms that the pressures stored in surface_pressure_ can be reproduced
   by linearly interpolating pressure values on the `tet` based on the
   data in `edge_vertices`. */
  ::testing::AssertionResult PressuresMatchVertices(
      VolumeElementIndex tet_index,
      SurfaceVertexIndex centroid_index,
      const vector<EdgeVertex>& edge_vertices,
      const VolumeMeshFieldLinear<double, double>& field_F) const {
    constexpr double kEps = 32 * std::numeric_limits<double>::epsilon();
    for (const auto& edge_vertex : edge_vertices) {
      // Compute expected pressure.
      const double p0 =
          field_F.EvaluateAtVertex(edge_vertex.edge.first());
      const double p1 =
          field_F.EvaluateAtVertex(edge_vertex.edge.second());
      // Combine using the weight as documented in EdgeVertex.
      const double expected_pressure =
          p0 * edge_vertex.weight + (1 - edge_vertex.weight) * p1;
      const SurfaceVertexIndex test_vertex_index = edge_vertex.slice_vertex;
      const double pressure = surface_pressure_[test_vertex_index];
      if (std::abs(pressure - expected_pressure) > kEps) {
        return ::testing::AssertionFailure()
               << "\nSurface vertex " << test_vertex_index
               << " has wrong pressure\n"
               << " Found    " << pressure << "\n"
               << " Expected " << expected_pressure;
      }
    }

    // Pressure is likewise correct at the centroid.
    const Vector3d p_FC = X_WF_.inverse() * CentroidFromFan(edge_vertices);
    const double p_at_C_expected = field_F.EvaluateCartesian(tet_index, p_FC);
    const double p_at_C = surface_pressure_.at(centroid_index);
    if (std::abs(p_at_C - p_at_C_expected) > kEps) {
      return ::testing::AssertionFailure()
          << "\nPressure at centroid is wrong.\n"
          << " Found    " << p_at_C << "\n"
          << " Expected " << p_at_C_expected;
    }

    return ::testing::AssertionSuccess();
  }

  /* This tests the values in faces_, vertices_M, and surface_pressure_ against
   the given mesh and confirms they are consistent. It assumes that the
   result has no duplicate vertices. The following is tested:

      1. The vertices all lie within the tetrahedron (and contains no NaN
         values).
      2. The vertices define a fan (i.e., each triangle shares a common
         vertex, the fan origin). All other vertices are referenced twice,
         once by each face.
      3. Every non-centroid (perimeter) vertex lies on an edge of the
         indicated tet.
      4. The shared vertex is properly the centroid of the polygon formed
         by the the perimeter vertices.
      5. The reported pressure at a perimeter vertex is a linear combination
         of its edge vertex values just like the vertex position is -- using
         the same weight. */
  ::testing::AssertionResult SliceIsConsistent(
      VolumeElementIndex tet_index,
      const VolumeMeshFieldLinear<double, double>& field_F) const {
    // Confirms that the triangles lie within the tetrahedron; the barycentric
    // coordinates should all lie within [0, 1]. This will also catch NaN
    // values because the condition will be de facto false.
    constexpr double kEps = 32 * std::numeric_limits<double>::epsilon();
    const RigidTransformd X_FW = X_WF_.inverse();
    for (const auto& V_W : vertices_W_) {
      const Vector3d& p_WV = V_W.r_MV();
      const Vector3d& p_FV = X_FW * p_WV;
      const VolumeMesh<double>::Barycentric b_V =
          field_F.mesh().CalcBarycentric(p_FV, tet_index);
      for (int i = 0; i < 4; ++i) {
        if (!(b_V(i) >= -kEps && b_V(i) <= 1 + kEps)) {
          return ::testing::AssertionFailure()
                 << "Mesh vertex reported outside of the tet:\n"
                 << "    p_FV: " << p_FV.transpose() << "\n"
                 << "    b_V: " << b_V.transpose();
        }
      }
    }
    // The slicing produces a small triangle fan around a common vertex. That
    // vertex _should_ be the centroid. This identifies the fan vertex (but
    // confirmation that it is the centroid is deferred to below).
    const auto [fan_index, result1] = IdentifyFanVertex(faces_);
    if (!result1) return result1;

    // Confirms that each perimeter vertex lies on an edge of the tetrahedron.
    const auto [edge_vertices, result2] = CharacterizeEdgeVertices(
        fan_index, faces_, vertices_W_, tet_index, field_F.mesh());
    if (!result2) return result2;

    const auto result3 = FanVertexIsCentroid(edge_vertices, fan_index);
    if (!result3) return result3;

    return PressuresMatchVertices(tet_index, fan_index, edge_vertices, field_F);
  }

  /* Computes the centroid of the indicated tet in the given mesh. */
  static Vector3d ComputeTetCentroid(const VolumeMesh<double>& mesh_M,
                                     const VolumeElement& tet) {
    // Accumulator for the position of the centroid C in the mesh frame M.
    Vector3d p_MC = Vector3d::Zero();
    for (int i = 0; i < 4; ++i) {
      const Vector3d& p_MVi = mesh_M.vertex(tet.vertex(i)).r_MV();
      p_MC += p_MVi;
    }
    return p_MC / 4.0;
  }

  /* Confirms that the normal implied by the triangle indicated by `face_index`
   matches the given expected normal. */
  ::testing::AssertionResult FaceNormalMatches(
      int tri_index, const Vector3d& expected_n_W) const {
    const SurfaceFace& tri = faces_[tri_index];
    const Vector3d& r_WV0 = vertices_W_[tri.vertex(0)].r_MV();
    const Vector3d& r_WV1 = vertices_W_[tri.vertex(1)].r_MV();
    const Vector3d& r_WV2 = vertices_W_[tri.vertex(2)].r_MV();
    const Vector3d n_W = ((r_WV1 - r_WV0).cross(r_WV2 - r_WV0)).normalized();
    // The difference between computing the normal like this and the
    // transformation of nhat_F into nhat_W for the space of arbitrary R_WF used
    // in these tests seem to lead to a difference in about six bits -- so we
    // scale epsilon by 2^6.
    constexpr double kEps = 64 * std::numeric_limits<double>::epsilon();
    return CompareMatrices(expected_n_W, n_W, kEps);
  }

  /* The accumulators for the SliceTetWithPlane() method. */
  vector<SurfaceFace> faces_;
  vector<SurfaceVertex<double>> vertices_W_;
  vector<double> surface_pressure_;
  RigidTransformd X_WF_;
  unordered_map<SortedPair<VolumeVertexIndex>, SurfaceVertexIndex> cut_edges_;
};

/* This tests the *boundaries* of intersection. Confirms that a tet lying
 completely on either side of the plane produces no mesh artifacts. Then, with
 a small perturbation confirms that artifacts *are* created, properly bounding
 the sensitivity between intersecting and not intersecting. The correctness of
 the values of those artifacts can be found in subsequent tests. */
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

  // For these tests, we do not want to do analysis; just confirming the
  // existence/non-existence of intersection results.
  const bool no_analysis = false;

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
    CallSliceTetWithPlane(Plane<double>{Mz_F, p_FMax + offset_F}, X_FM,
                          no_analysis);
    EXPECT_TRUE(HasNFaces(0));

    // Case: Plane lies _almost_ completely above the tet (V3 penetrates the
    // plane).
    CallSliceTetWithPlane(Plane<double>{Mz_F, p_FMax - offset_F}, X_FM,
                          no_analysis);
    EXPECT_TRUE(HasNFaces(3));

    // Case: Plane lies completely below the tet (the bottom faces lies on the
    // z = 0 plane in Frame M).
    CallSliceTetWithPlane(Plane<double>{Mz_F, p_FMin - offset_F}, X_FM,
                          no_analysis);
    EXPECT_TRUE(HasNFaces(0));

    // Case: Plane lies _almost_ completely below the tet.
    CallSliceTetWithPlane(Plane<double>{Mz_F, p_FMin + offset_F}, X_FM,
                          no_analysis);
    EXPECT_TRUE(HasNFaces(3));

    // Tests in which the plane normal is _not_ the Mz axis. We'll align it so
    // that it is normal to the v1, v2, v3 plane. We happen to know that
    // that normal (expressed in M) points in the <1, 1, 1> direction.
    const Vector3d normal_F = X_FM.rotation() * Vector3d{1, 1, 1}.normalized();

    // We need to construct an instance of mesh_F so we can find out where
    // vertices v1, v2, v3 are so we know where to position the plane.
    VolumeMesh<double> mesh_F = TrivialVolumeMesh(X_FM);

    // Case: Plane is just beyond the v1, v2, v3, plane; it passes through a
    // point _near_ V3, but just offset in the normal direction.
    CallSliceTetWithPlane(Plane<double>{normal_F, p_FMax + kEps * normal_F},
                          X_FM, no_analysis);
    EXPECT_TRUE(HasNFaces(0));

    // Case: Plane intersects the tet near the v1, v2, v3, plane; it passes
    // through a point _near_ V3, but offset in the _negative_ normal direction.
    CallSliceTetWithPlane(
        Plane<double>{normal_F, p_FMax - 4 * kEps * normal_F}, X_FM,
        no_analysis);
    EXPECT_TRUE(HasNFaces(3));
  }
}

/* This tests the cases where the plane intersects with the tet, creating a
 triangle. It confirms the number of triangles produced, confirms face normal
 direction, and uses the test infrastructure to evaluate the coherency of the
 set of triangles. */
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

  const VolumeElementIndex tet_index{0};

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
      const Vector3d& p_FVi = mesh_F.vertex(tet.vertex(i)).r_MV();
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
            const Vector3d& p_FVj = mesh_F.vertex(tet.vertex(j)).r_MV();
            // All other vertices are on the opposite side of the plane.
            EXPECT_LT(plane_sign * plane_F.CalcHeight(p_FVj), 0);
          }
        }
        ASSERT_TRUE(
            CallSliceTetWithPlane(plane_F, X_FM, true /* do_analysis */));

        // Further consistency analysis.
        ASSERT_TRUE(HasNFaces(3));
        const Vector3d nhat_W = X_WF_.rotation() * plane_F.normal();
        for (int t = 0; t < 3; ++t) {
          EXPECT_TRUE(FaceNormalMatches(t, nhat_W));
        }
      }
    }
  }
}

/* Similar for the TriangleIntersections test, but for intersections that for
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

  const VolumeElementIndex tet_index{0};

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
      const Vector3d& p_FV0 = mesh_F.vertex(tet.vertex(v0)).r_MV();
      const Vector3d& p_FV1 = mesh_F.vertex(tet.vertex(v1)).r_MV();
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
          const Vector3d& p_FVi = mesh_F.vertex(tet.vertex(i)).r_MV();
          if (i == v0 || i == v1) {
            EXPECT_GT(plane_sign * plane_F.CalcHeight(p_FVi), 0.0);
          } else {
            EXPECT_LT(plane_sign * plane_F.CalcHeight(p_FVi), 0.0);
          }
        }

        ASSERT_TRUE(
            CallSliceTetWithPlane(plane_F, X_FM, true /* do_analysis */));

        // Further consistency analysis.
        ASSERT_TRUE(HasNFaces(4));
        const Vector3d nhat_W = X_WF_.rotation() * plane_F.normal();
        for (int t = 0; t < 4; ++t) {
          FaceNormalMatches(t, nhat_W);
        }
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
  // sub-triangles each). One resulting surface will have five vertices
  // and one will have eight. We confirm that the triangles  occupy the
  // same space -- i.e., the only difference is duplication of vertices.

  // We're going to do this with a single X_FM; we've already evaluated the
  // correctness of the per-tet calculation. This is about checking the
  // bookkeeping via feature counts.
  const RigidTransformd X_FM;  // Identity.

  // The infrastructure for the mesh with the minimum number of vertices.
  VolumeMesh<double> min_mesh_F =
      TrivialVolumeMesh(X_FM, true /* min_vertices */);
  VolumeMeshFieldLinear<double, double> min_field_F{
      "pressure", vector<double>{0, 0, 0, 1, -1}, &min_mesh_F};
  vector<SurfaceFace> min_faces;
  vector<SurfaceVertex<double>> min_vertices_F;
  vector<double> min_surface_pressure;
  unordered_map<SortedPair<VolumeVertexIndex>, SurfaceVertexIndex>
      min_cut_edges;

  // The infrastructure for the mesh with duplicate vertices.
  VolumeMesh<double> dupe_mesh_F =
      TrivialVolumeMesh(X_FM, false /* min_vertices */);
  VolumeMeshFieldLinear<double, double> dupe_field_F{
      "pressure", vector<double>{0, 0, 0, 1, 0, 0, 0, -1}, &dupe_mesh_F};
  vector<SurfaceFace> dupe_faces;
  vector<SurfaceVertex<double>> dupe_vertices_F;
  vector<double> dupe_surface_pressure;
  unordered_map<SortedPair<VolumeVertexIndex>, SurfaceVertexIndex>
      dupe_cut_edges;

  // The common slicing plane.
  Plane<double> plane_F{Vector3d::UnitX(), Vector3d{0.5, 0, 0}};

  for (VolumeElementIndex i{0}; i < 2; ++i) {
    SliceTetWithPlane(i, min_field_F, plane_F, X_WF_, &min_faces,
                      &min_vertices_F, &min_surface_pressure, &min_cut_edges);
    SliceTetWithPlane(i, dupe_field_F, plane_F, X_WF_, &dupe_faces,
                      &dupe_vertices_F, &dupe_surface_pressure,
                      &dupe_cut_edges);
  }

  // Both have the expected number (6) of faces.
  ASSERT_EQ(dupe_faces.size(), 6u);
  ASSERT_EQ(min_faces.size(), 6u);
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  // We exploit knowledge of the algorithm to assume that the faces in both
  // results are in the exact same order with the constituent vertices also in
  // same order.
  for (int f = 0; f < 6; ++f) {
    const SurfaceFace& min_face = min_faces[0];
    const SurfaceFace& dupe_face = dupe_faces[0];
    for (int v = 0; v < 3; ++v) {
      const Vector3d& p_FVm = min_vertices_F[min_face.vertex(v)].r_MV();
      const Vector3d& p_FVd = dupe_vertices_F[dupe_face.vertex(v)].r_MV();
      EXPECT_NEAR((p_FVm - p_FVd).norm(), 0, kEps);
    }
  }

  // Confirm that all of the vertices in the duplicate mesh are referenced.
  set<SurfaceVertexIndex> vertex_indices;
  for (const auto& face : dupe_faces) {
    for (int i = 0; i < 3; ++i) vertex_indices.insert(face.vertex(i));
  }
  ASSERT_EQ(vertex_indices.size(), dupe_vertices_F.size());
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
   give us that perfect precision for free. */

  const Plane<double> plane_M{Vector3d::UnitZ(), Vector3d::Zero()};
  const RigidTransformd I;
  VolumeMesh<double> mesh_M = TrivialVolumeMesh(I);
  // Make an arbitrary mesh field with heterogeneous values.
  vector<double> values{0.25, 0.5, 0.75, 1, -1};
  VolumeMeshFieldLinear<double, double> field_M{"pressure", move(values),
                                                &mesh_M};

  {
    // Slicing against tet 0 should intersect and produce the three faces.
    vector<SurfaceFace> faces;
    vector<SurfaceVertex<double>> vertices_W;
    vector<double> surface_pressure;
    unordered_map<SortedPair<VolumeVertexIndex>, SurfaceVertexIndex> cut_edges;
    VolumeElementIndex tet_index{0};
    SliceTetWithPlane(tet_index, field_M, plane_M, I, &faces, &vertices_W,
                      &surface_pressure, &cut_edges);
    EXPECT_EQ(faces.size(), 3u);
  }

  {
    // Slicing against tet 1 should produce no intersection.
    vector<SurfaceFace> faces;
    vector<SurfaceVertex<double>> vertices_W;
    vector<double> surface_pressure;
    unordered_map<SortedPair<VolumeVertexIndex>, SurfaceVertexIndex> cut_edges;
    VolumeElementIndex tet_index{1};
    SliceTetWithPlane(tet_index, field_M, plane_M, I, &faces, &vertices_W,
                      &surface_pressure, &cut_edges);
    EXPECT_EQ(faces.size(), 0);
  }
}

/* Test of ComputeContactSurface(). Most of the hard-core math is contained in
 SliceTetWithPlane (and is covered by its unit tests). This function has the
 following unique responsibilities:
  1. No intersection returns nullptr.
  2. The resulting contact surface is a function of the tets provided; they
     are all considered and only those that actually intersect contribute to
     to the final output.
  3. Duplicates handled correctly; if the input mesh has no duplicates, the
     result has none. Alternatively, if the input has duplicates, the output
     does as well.
  4. Confirm that the surface mesh field references the surface mesh in the
     resultant ContactSurface.
  5. Mesh normals point out of plane and into soft mesh.
 These tests are done in a single, non-trivial frame F. The robustness of the
 numerics is covered in the SliceTetWithPlaneTest and this just needs to
 account for data tracking. */
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
        "pressure", vector<double>{0, 0, 0, 1, -1}, mesh_F_.get());
    mesh_id_ = GeometryId::get_new_id();
    plane_id_ = GeometryId::get_new_id();
  }

  // The soft mesh and the pressure field on that mesh.
  unique_ptr<VolumeMesh<double>> mesh_F_;
  unique_ptr<VolumeMeshFieldLinear<double, double>> field_F_;
  GeometryId mesh_id_;
  GeometryId plane_id_;

  // Vectors of tet indices; to facilitate calls to ComputeContactSurface.
  using VIndex = VolumeElementIndex;
  const vector<VIndex> both_tets_{VIndex(0), VIndex(1)};
  const vector<VIndex> only_tet_0_{VIndex(0)};
  const vector<VIndex> only_tet_1_{VIndex(1)};

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
    EXPECT_EQ(ComputeContactSurface<double>(mesh_id_, *field_F_, plane_id_,
                                            plane_F, both_tets_, X_WF_),
              nullptr);
  }

  {
    // Case: plane slices through tet 0, but we omit tet 0 from the test case
    // therefore no intersection is detected and nullptr is returned.
    const Vector3d p_FB = X_FM_.translation() + 0.5 * Mz_F;
    const Plane<double> plane_F{Mz_F, p_FB};
    EXPECT_EQ(ComputeContactSurface<double>(mesh_id_, *field_F_, plane_id_,
                                            plane_F, only_tet_1_, X_WF_),
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
      ComputeContactSurface<double>(mesh_id_, *field_F_, plane_id_, plane_F,
                                    both_tets_, X_WF_);
  ASSERT_NE(contact_surface, nullptr);
  EXPECT_EQ(contact_surface->mesh_W().num_elements(), 6);
  EXPECT_EQ(contact_surface->mesh_W().num_vertices(), 6);

  // Passing just one or the other produces only one tet's worth of triangles.
  auto contact_surface_0 =
      ComputeContactSurface<double>(mesh_id_, *field_F_, plane_id_, plane_F,
                                    only_tet_0_, X_WF_);
  ASSERT_NE(contact_surface_0, nullptr);
  EXPECT_EQ(contact_surface_0->mesh_W().num_elements(), 3);
  EXPECT_EQ(contact_surface_0->mesh_W().num_vertices(), 4);
  const RigidTransformd X_MW = X_FM_.inverse() * X_WF_.inverse();
  const SurfaceVertexIndex last_vertex{3};
  {
    // For tet 0, the z-value of the last vertex should be > 0.
    const Vector3d& p_WV =
        contact_surface_0->mesh_W().vertex(last_vertex).r_MV();
    const Vector3d& p_MV = X_MW * p_WV;
    EXPECT_GT(p_MV(2), 0);
  }

  auto contact_surface_1 =
      ComputeContactSurface<double>(mesh_id_, *field_F_, plane_id_, plane_F,
                                    only_tet_1_, X_WF_);
  ASSERT_NE(contact_surface_1, nullptr);
  EXPECT_EQ(contact_surface_1->mesh_W().num_elements(), 3);
  EXPECT_EQ(contact_surface_1->mesh_W().num_vertices(), 4);
  {
    // For tet 1, the z-value of the last vertex should be < 0.
    const Vector3d& p_WV =
        contact_surface_1->mesh_W().vertex(last_vertex).r_MV();
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
        ComputeContactSurface<double>(mesh_id_, *field_F_, plane_id_, plane_F,
                                      both_tets_, X_WF_);
    ASSERT_NE(contact_surface, nullptr);
    const SurfaceMesh<double>& contact_mesh_W = contact_surface->mesh_W();
    EXPECT_EQ(contact_mesh_W.num_elements(), 6);
    EXPECT_EQ(contact_mesh_W.num_vertices(), 6);

    // O(N^2) test comparing all vertex distances; report the minimum distance.
    double min_distance = std::numeric_limits<double>::infinity();
    for (SurfaceVertexIndex i{0}; i < 5; ++i) {
      const Vector3d& p_WVi = contact_mesh_W.vertex(i).r_MV();
      for (SurfaceVertexIndex j{i + 1}; j < 6; ++j) {
        const Vector3d& p_WVj = contact_mesh_W.vertex(j).r_MV();
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
        "pressure", vector<double>{0, 0, 0, 1, 0, 0, 0, -1}, &dupe_mesh_F};

    // Passing in all tet indices produces the full intersection: 6 triangles.
    unique_ptr<ContactSurface<double>> contact_surface =
        ComputeContactSurface<double>(mesh_id_, dupe_field_F, plane_id_,
                                      plane_F, both_tets_, X_WF_);
    ASSERT_NE(contact_surface, nullptr);
    const SurfaceMesh<double>& contact_mesh_W = contact_surface->mesh_W();
    EXPECT_EQ(contact_mesh_W.num_elements(), 6);
    EXPECT_EQ(contact_mesh_W.num_vertices(), 8);

    // We'll count the number of duplicates (based on distance between
    // vertices). We expect two vertices to be duplicated.
    constexpr double kEps = std::numeric_limits<double>::epsilon();
    int duplicate_count = 0;
    for (SurfaceVertexIndex i{0}; i < 5; ++i) {
      const Vector3d& p_WVi = contact_mesh_W.vertex(i).r_MV();
      for (SurfaceVertexIndex j{i + 1}; j < 6; ++j) {
        const Vector3d& p_WVj = contact_mesh_W.vertex(j).r_MV();
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
  unique_ptr<ContactSurface<double>> contact = ComputeContactSurface<double>(
      mesh_id_, *field_F_, plane_id_, plane_F, both_tets_, X_WF_);
  const auto& mesh_W = contact->mesh_W();
  const auto& field_W = contact->e_MN();
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
    unique_ptr<ContactSurface<double>> contact = ComputeContactSurface<double>(
        id_A, *field_F_, id_B, plane_F, both_tets_, X_WF_);

    const double normal_sign = contact->id_N() == id_B ? 1.0 : -1.0;
    const Vector3d nhat_W = X_WF_.rotation() * (normal_sign * Mz_F);
    // NOTE: When we set the normals directly from the plane, this precision
    // will improve.
    constexpr double kEps = 64 * std::numeric_limits<double>::epsilon();
    const SurfaceMesh<double>& mesh_W = contact->mesh_W();
    for (SurfaceFaceIndex f{0}; f < mesh_W.num_faces(); ++f) {
      EXPECT_TRUE(CompareMatrices(mesh_W.face_normal(f), nhat_W, kEps));
    }
  }
}

/* Test of ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(). This function
 has the following unique responsibilities:

  1. Simply that all of the values contribute to the output (poses, ids, etc.)
  2. Autodiff-valued transforms throw. */
GTEST_TEST(MeshPlaneIntersectionTest, SoftVolumeRigidHalfSpace) {
  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  // Create mesh and volume mesh.
  const VolumeMesh<double> mesh_F = TrivialVolumeMesh(RigidTransformd{});
  const VolumeMeshFieldLinear<double, double> field_F{
      "pressure", vector<double>{0.25, 0.5, 0.75, 1, -1}, &mesh_F};
  const BoundingVolumeHierarchy<VolumeMesh<double>> bvh_F(mesh_F);

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
        id_A, field_F, bvh_F, X_WS, id_B, X_WR);
    ASSERT_NE(contact_surface, nullptr);
    // We exploit the knowledge that id_M < id_N and id_A < id_B.
    ASSERT_LT(id_A, id_B);
    EXPECT_EQ(contact_surface->id_M(), id_A);
    EXPECT_EQ(contact_surface->id_N(), id_B);
    EXPECT_EQ(contact_surface->mesh_W().num_elements(), 6);
    // Sample the face normals.
    const Vector3d& norm_W =
        contact_surface->mesh_W().face_normal(SurfaceFaceIndex{0});
    const Vector3d& Sx_W = X_WS.rotation().col(0);
    EXPECT_TRUE(CompareMatrices(norm_W, Sx_W, kEps));
    // Sample the vertex positions: in the S frame they should all have x = 0.5.
    const Vector3d& p_WV =
        contact_surface->mesh_W().vertex(SurfaceVertexIndex{0}).r_MV();
    const Vector3d p_SV = X_WS.inverse() * p_WV;
    EXPECT_NEAR(p_SV(0), 0.5, kEps);
  }

  // Subsequent tests will not test all the properties of the resulting contact
  // surface. We assume that the initial test indicates the values come through.

  {
    // Case 2: Move the mesh out of intersection.
    const RigidTransformd X_SM{Vector3d{-0.51, 0, 0}};
    auto contact_surface = ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(
        id_A, field_F, bvh_F, X_WS * X_SM, id_B, X_WR);
    ASSERT_EQ(contact_surface, nullptr);
  }

  {
    // Case 3: Move the plane out of intersection.
    const RigidTransformd X_SR2{X_SR.rotation(), Vector3d{1.01, 0, 0}};
    auto contact_surface = ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(
        id_A, field_F, bvh_F, X_WS, id_B, X_WS * X_SR2);
    ASSERT_EQ(contact_surface, nullptr);
  }

  {
    // Case 4: Reverse GeometryIds.
    auto contact_surface = ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(
        id_B, field_F, bvh_F, X_WS, id_A, X_WR);
    ASSERT_NE(contact_surface, nullptr);
    EXPECT_EQ(contact_surface->id_M(), id_A);
    EXPECT_EQ(contact_surface->id_N(), id_B);
    // The effect of reversing the labels reverses the normals, so repeat the
    // normal test, but in the opposite direction.
    const Vector3d& norm_W =
        contact_surface->mesh_W().face_normal(SurfaceFaceIndex{0});
    const Vector3d& Sx_W = X_WS.rotation().col(0);
    EXPECT_TRUE(CompareMatrices(norm_W, -Sx_W, kEps));
  }
}

GTEST_TEST(MeshPlaneIntersectionTest, AutoDiffThrows) {
  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  // Create mesh and volume mesh.
  const VolumeMesh<double> mesh_F = TrivialVolumeMesh(RigidTransformd{});
  const VolumeMeshFieldLinear<double, double> field_F{
      "pressure", vector<double>{0.25, 0.5, 0.75, 1, -1}, &mesh_F};
  const BoundingVolumeHierarchy<VolumeMesh<double>> bvh_F(mesh_F);

  const RigidTransform<AutoDiffXd> X_WS;
  const RigidTransform<AutoDiffXd> X_WR;

  DRAKE_EXPECT_THROWS_MESSAGE(
      ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(id_A, field_F, bvh_F,
                                                        X_WS, id_B, X_WR),
      std::logic_error,
      "AutoDiff-valued ContactSurface calculations are not currently "
      "supported");
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
