#include "drake/geometry/proximity/mesh_half_space_intersection.h"

#include <limits>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransform;
using math::RotationMatrix;
using std::move;

/// Creates a SurfaceMesh of a box. The box is defined in frame B, centered on
/// Bo with its dimensions aligned with Bx, By, and Bz. The resultant mesh
/// has the vertices expressed in Frame F, courtesy of the transform `X_FB`.
SurfaceMesh<double> CreateBoxMesh(const RigidTransform<double>& X_FB) {
  // Set the box vertices according to the following diagram. The box is
  // centered at the origin of Frame B. We transform it to some query frame F,
  // making it less likely that all of the underlying functionality succeeds
  // because of some fortuitous initial values. The half space's boundary plane
  // is z = 0 in Frame B. Set the box vertices according to the following
  // diagram.
  //               v₁      +Z         v₃
  //               ●───────┆─────────●
  //              /│       ┆        /│
  //             / │       ┆     v₇/ │
  //         v₅ ●─────────────────●  │
  //            │  │       ┆      │  │
  //            │  │       o┄┄┄┄┄┄│┄┄┄┄┄ +Y
  //            │  │      /       │  │
  //            │  ●─────/────────│──● v₂
  //            │ /v₀   /         │ /
  //            │/     /          │/
  //         v₄ ●─────────────────● v₆
  //                 /
  //                +X

  std::vector<SurfaceVertex<double>> vertices_F;
  vertices_F.emplace_back(X_FB * Vector3d(-1, -1, -1));
  vertices_F.emplace_back(X_FB * Vector3d(-1, -1, 1));
  vertices_F.emplace_back(X_FB * Vector3d(-1, 1, -1));
  vertices_F.emplace_back(X_FB * Vector3d(-1, 1, 1));
  vertices_F.emplace_back(X_FB * Vector3d(1, -1, -1));
  vertices_F.emplace_back(X_FB * Vector3d(1, -1, 1));
  vertices_F.emplace_back(X_FB * Vector3d(1, 1, -1));
  vertices_F.emplace_back(X_FB * Vector3d(1, 1, 1));

  // Set the twelve box faces using a counter-clockwise winding.
  typedef SurfaceVertexIndex VIndex;
  std::vector<SurfaceFace> faces;
  faces.emplace_back(VIndex(4), VIndex(6), VIndex(7));  // +X face
  faces.emplace_back(VIndex(7), VIndex(5), VIndex(4));  // +X face
  faces.emplace_back(VIndex(1), VIndex(3), VIndex(2));  // -X face
  faces.emplace_back(VIndex(2), VIndex(0), VIndex(1));  // -X face
  faces.emplace_back(VIndex(2), VIndex(3), VIndex(7));  // +Y face
  faces.emplace_back(VIndex(7), VIndex(6), VIndex(2));  // +Y face
  faces.emplace_back(VIndex(4), VIndex(5), VIndex(1));  // -Y face
  faces.emplace_back(VIndex(1), VIndex(0), VIndex(4));  // -Y face
  faces.emplace_back(VIndex(7), VIndex(3), VIndex(1));  // +Z face
  faces.emplace_back(VIndex(1), VIndex(5), VIndex(7));  // +Z face
  faces.emplace_back(VIndex(4), VIndex(0), VIndex(2));  // -Z face
  faces.emplace_back(VIndex(2), VIndex(6), VIndex(4));  // -Z face

  // Construct the mesh.
  return {move(faces), move(vertices_F)};
}

/// Creates a new SurfaceMesh mesh from the given input mesh, such that the
/// resulting mesh has the same domain, but with two differences:
///
///   1. The vertices in the resultant mesh are measured and expressed in
///      Frame W (as opposed to the input mesh's Frame F)
///   2. Each triangle in the input mesh has been turned into three triangles;
///      the original turned into a triangle fan around its centroid.
/// This method confirms that the face normal directions are preserved.
template <typename T>
static SurfaceMesh<double> CreateMeshWithCentroids(
    const SurfaceMesh<double>& mesh_F, const RigidTransform<T>& X_WF) {
  using VIndex = SurfaceVertexIndex;
  using FIndex = SurfaceFaceIndex;
  constexpr double kEps = 8 * std::numeric_limits<double>::epsilon();
  const RigidTransform<double>& X_WF_d = convert_to_double(X_WF);

  // All of the original vertices are part of the final mesh. And each face
  // is a polygon we want to add to the new mesh. So, we copy the vertices
  // and then simply add polygon after polygon.
  std::vector<SurfaceFace> new_faces;
  std::vector<SurfaceVertex<double>> new_vertices_W;
  std::transform(mesh_F.vertices().begin(), mesh_F.vertices().end(),
                 std::back_inserter(new_vertices_W),
                 [&X_WF_d](const SurfaceVertex<double>& v_F) {
                   return SurfaceVertex<double>{X_WF_d * v_F.r_MV()};
                 });

  const RotationMatrix<double>& R_WF = X_WF_d.rotation();
  for (FIndex f_index(0); f_index < mesh_F.num_faces(); ++f_index) {
    const SurfaceFace& f = mesh_F.element(f_index);
    // We want to add the triangle fan for the existing face, but also want to
    // confirm that the new faces have normals that match the input face.
    std::vector<VIndex> polygon{f.vertex(0), f.vertex(1), f.vertex(2)};
    const Vector3d& nhat_W = R_WF * mesh_F.face_normal(f_index);
    AddPolygonToMeshData(polygon, nhat_W, &new_faces, &new_vertices_W);

    // Confirm polygon winding matches between source triangle and triangles
    // in the new fan.
    // The triangle fan consists of the last three faces in `new_faces`.
    const int first_new_face_index = static_cast<int>(new_faces.size()) - 3;
    for (FIndex j(first_new_face_index); j < new_faces.size(); ++j) {
      const SurfaceFace& new_face = new_faces[j];
      const Vector3d& a = new_vertices_W[new_face.vertex(0)].r_MV();
      const Vector3d& b = new_vertices_W[new_face.vertex(1)].r_MV();
      const Vector3d& c = new_vertices_W[new_face.vertex(2)].r_MV();
      const Vector3d new_nhat_W = (b - a).cross((c - a)).normalized();
      if (!CompareMatrices(nhat_W, new_nhat_W, kEps)) {
        throw std::runtime_error("Derived mesh's normals are incorrect");
      }
    }
  }

  return SurfaceMesh<double>(move(new_faces), move(new_vertices_W));
}

/// Creates a SurfaceMesh with a single triangle using the given vertices.
SurfaceMesh<double> CreateOneTriangleMesh(const Vector3d& v0,
                                          const Vector3d& v1,
                                          const Vector3d& v2) {
  std::vector<SurfaceVertex<double>> vertices;
  vertices.emplace_back(v0);
  vertices.emplace_back(v1);
  vertices.emplace_back(v2);

  std::vector<SurfaceFace> faces = {SurfaceFace(
      SurfaceVertexIndex(0), SurfaceVertexIndex(1), SurfaceVertexIndex(2))};

  return SurfaceMesh<double>(move(faces), move(vertices));
}

template <typename T>
class MeshHalfspaceIntersectionTest : public ::testing::Test {
 public:
  // Accessors for data structures used repeatedly.
  std::vector<SurfaceVertex<T>>& new_vertices_W() { return new_vertices_W_; }
  std::vector<SurfaceFace>& new_faces() { return new_faces_; }
  std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>&
  vertices_to_newly_created_vertices() {
    return vertices_to_newly_created_vertices_;
  }
  std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>&
  edges_to_newly_created_vertices() {
    return edges_to_newly_created_vertices_;
  }

  // Checks whether two faces from two meshes are equivalent, which we define
  // to mean as some permutation of acceptable windings for the vertices of
  // `fb` yields vertices coincident with those of `fa`.
  bool AreFacesEquivalent(const SurfaceFace& fa,
                          const SurfaceMesh<double>& mesh_a,
                          const SurfaceFace& fb, const SurfaceMesh<T>& mesh_b) {
    constexpr double kEps = 10 * std::numeric_limits<double>::epsilon();
    // Get the three vertices from each.
    std::array<Vector3d, 3> vertices_a = {mesh_a.vertex(fa.vertex(0)).r_MV(),
                                          mesh_a.vertex(fa.vertex(1)).r_MV(),
                                          mesh_a.vertex(fa.vertex(2)).r_MV()};
    std::array<Vector3<T>, 3> vertices_b = {mesh_b.vertex(fb.vertex(0)).r_MV(),
                                            mesh_b.vertex(fb.vertex(1)).r_MV(),
                                            mesh_b.vertex(fb.vertex(2)).r_MV()};

    // Set an array of faces that will be used to determine how the two
    // triangles align. Each of these faces encodes an "acceptable" triangle
    // winding. They are "acceptable" because they represent triangles with the
    // same winding and, therefore, the same normal.
    typedef SurfaceVertexIndex Index;
    std::array<SurfaceFace, 3> permutations = {
        SurfaceFace(Index(0), Index(1), Index(2)),
        SurfaceFace(Index(1), Index(2), Index(0)),
        SurfaceFace(Index(2), Index(0), Index(1))};

    // Verify that at least one of the permutations gives essentially the
    // vertices from fb. We use epsilon because the centroid vertex may have
    // rounding error in it.
    using std::min;
    T closest_dist = std::numeric_limits<T>::max();
    for (const auto& face : permutations) {
      T dist = 0;
      for (int i = 0; i < 3; ++i)
        dist += (vertices_a[face.vertex(i)] - vertices_b[i]).norm();
      closest_dist = min(closest_dist, dist);
      if (closest_dist <= kEps) break;
    }

    return closest_dist <= kEps;
  }

  // Checks whether two meshes are equivalent, meaning that there is a bijective
  // mapping from every face in mesh a to every face in mesh b. A mapping
  // between faces can only exist if the face in a is considered equivalent to
  // the corresponding face in b -- see AreFacesEquivalent().
  void VerifyMeshesEquivalent(const SurfaceMesh<double>& mesh_a,
                              const SurfaceMesh<T>& mesh_b) {
    // Simple checks first.
    ASSERT_EQ(mesh_a.num_faces(), mesh_b.num_faces());
    ASSERT_EQ(mesh_a.num_vertices(), mesh_b.num_vertices());

    // Iterate through each face of the first mesh, looking for a face from the
    // second mesh that is within the given tolerance. This is a quadratic
    // time algorithm (in the number of faces of the meshes) but we expect the
    // number of faces that this algorithm is run on to be small.
    std::vector<SurfaceFace> faces_from_mesh_b = mesh_b.faces();
    for (const SurfaceFace& f1 : mesh_a.faces()) {
      bool found_match = false;
      for (int i = 0; i < static_cast<int>(faces_from_mesh_b.size()); ++i) {
        if (AreFacesEquivalent(f1, mesh_a, faces_from_mesh_b[i], mesh_b)) {
          found_match = true;
          faces_from_mesh_b[i] = faces_from_mesh_b.back();
          faces_from_mesh_b.pop_back();
          break;
        }
      }

      // If this test fails, it may fail catastrophically badly such that no two
      // faces match between the meshes. We don't want to spew *all* the
      // failures. The first is sufficient.
      ASSERT_TRUE(found_match)
          << "At least one face failed to match; there may be more.";
    }
  }

  // Clears all data structures used for constructing the intersection.
  void ClearConstructionDataStructures() {
    new_vertices_W().clear();
    new_faces().clear();
    vertices_to_newly_created_vertices().clear();
    edges_to_newly_created_vertices().clear();
  }

  // Calls the triangle-half space intersection routine using this object's
  // half space as well as the test harness's built-in mesh and construction
  // data structures. The vertices of the final answer will be expressed in the
  // world frame based on the given relative pose between mesh frame F and
  // world frame W.
  void CallConstructTriangleHalfspaceIntersectionPolygon(
      const SurfaceMesh<double>& mesh_F,
      const RigidTransform<T> X_WF) {
    ClearConstructionDataStructures();
    for (SurfaceFaceIndex f_index(0); f_index < mesh_F.num_elements();
         ++f_index) {
      ConstructTriangleHalfspaceIntersectionPolygon(
          mesh_F, f_index, *this->half_space_F_, X_WF, &this->new_vertices_W_,
          &this->new_faces_, &this->vertices_to_newly_created_vertices_,
          &this->edges_to_newly_created_vertices_);
    }
  }

 private:
  void SetUp() {
    // The tests all use half space normal [0 0 1], with point [0 0 2]
    // lying on the half space.
    Vector3<T> normal_H(0, 0, 1);
    const Vector3<T> point_H(0, 0, 2);
    half_space_F_ = std::make_unique<PosedHalfSpace<T>>(normal_H, point_H);
  }

  std::vector<SurfaceVertex<T>> new_vertices_W_;
  std::vector<SurfaceFace> new_faces_;
  std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>
      vertices_to_newly_created_vertices_;
  std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>
      edges_to_newly_created_vertices_;
  // In this test harness, the half space is always simply defined in the
  // mesh's frame F.
  std::unique_ptr<PosedHalfSpace<T>> half_space_F_;
};  // namespace
TYPED_TEST_SUITE_P(MeshHalfspaceIntersectionTest);

// Verifies that a triangle that lies fully outside of the half space yields an
// empty intersection. This covers Case 4 in the code.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, NoIntersection) {
  // Create the mesh, constructing the vertices of the triangle to lie outside
  // the half space.
  const SurfaceMesh<double> mesh_F = CreateOneTriangleMesh(
      Vector3d(0, 0, 3), Vector3d(1, 0, 3), Vector3d(0, 1, 3));

  // Verify no intersection.
  // X_WF = I -- because there is no intersection, the frame of the non-existent
  // intersection mesh is irrelevant.
  this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, {});
  EXPECT_TRUE(this->new_vertices_W().empty());
  EXPECT_TRUE(this->new_faces().empty());
  EXPECT_TRUE(this->vertices_to_newly_created_vertices().empty());
  EXPECT_TRUE(this->edges_to_newly_created_vertices().empty());
}

// Verifies that a triangle that lies inside or on the half space yields
// that same triangle. This covers Case 1 in the code.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, InsideOrOnIntersection) {
  using T = TypeParam;

  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<T> X_WF(
      RotationMatrix<T>(
          AngleAxis<T>{M_PI / 7, Vector3<T>{1, 2, 3}.normalized()}),
      Vector3<T>{-0.25, 0.5, 0.75});

  {
    // Case: The triangular mesh lies well inside the half space.
    const SurfaceMesh<double> mesh_F = CreateOneTriangleMesh(
        Vector3d(3, 5, 1), Vector3d(4, 5, 1), Vector3d(3, 6, 1));

    this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, X_WF);

    // Verify the intersection.
    SCOPED_TRACE("Single triangle inside half space");
    this->VerifyMeshesEquivalent(
        CreateMeshWithCentroids(mesh_F, X_WF),
        SurfaceMesh<T>{move(this->new_faces()), move(this->new_vertices_W())});
  }

  {
    // Case: triangle lies on the boundary plane.
    const SurfaceMesh<double> mesh_F = CreateOneTriangleMesh(
        Vector3d(3, 5, 2), Vector3d(4, 5, 2), Vector3d(3, 6, 2));

    // Verify the intersection.
    this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, X_WF);

    SCOPED_TRACE("Single triangle on half space boundary");
    this->VerifyMeshesEquivalent(
        CreateMeshWithCentroids(mesh_F, X_WF),
        SurfaceMesh<T>{move(this->new_faces()), move(this->new_vertices_W())});
  }

  {
    // Case: two triangles with a shared edge, completely enclosed in the half
    // space. Confirms that no extraneous vertices are introduced.
    std::vector<SurfaceVertex<double>> vertices = {
        SurfaceVertex<double>(Vector3d(4, 5, 2)),
        SurfaceVertex<double>(Vector3d(3, 5, 2)),
        SurfaceVertex<double>(Vector3d(3, 5, 1)),
        SurfaceVertex<double>(Vector3d(2, 5, 2))};
    typedef SurfaceVertexIndex Index;
    std::vector<SurfaceFace> faces = {
        SurfaceFace(Index(0), Index(1), Index(2)),
        SurfaceFace(Index(2), Index(1), Index(3))};
    const SurfaceMesh<double> mesh_F(move(faces), move(vertices));

    // Verify the intersection.
    this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, X_WF);

    SCOPED_TRACE("Two triangles with shared edge clipped");
    this->VerifyMeshesEquivalent(
        CreateMeshWithCentroids(mesh_F, X_WF),
        SurfaceMesh<T>{move(this->new_faces()), move(this->new_vertices_W())});
  }
}

// Verifies that a triangle that has exactly one vertex lying on the half space
// produces either (a) a degenerate intersection (if the other two vertices lie
// outside the half space) or (b) the original triangle (if the other two
// vertices lie within the half space). This covers Case 1.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, VertexOnHalfspaceIntersection) {
  using T = TypeParam;

  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<T> X_WF(
      RotationMatrix<T>(
          AngleAxis<T>{M_PI / 7, Vector3<T>{1, 2, 3}.normalized()}),
      Vector3<T>{-0.25, 0.5, 0.75});

  {
    // Case: one vertex on the boundary, two vertices outside.
    const SurfaceMesh<double> mesh_F = CreateOneTriangleMesh(
        Vector3d(3, 5, 2), Vector3d(4, 5, 3), Vector3d(3, 6, 3));

    // Verify the degenerate intersection.
    this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, X_WF);

    EXPECT_EQ(this->new_vertices_W().size(), 4);
    EXPECT_EQ(this->new_faces().size(), 3);
    // Evidence that we copied one vertex and split two edges.
    EXPECT_EQ(this->vertices_to_newly_created_vertices().size(), 1);
    EXPECT_EQ(this->edges_to_newly_created_vertices().size(), 2);

    // Check that the mesh that results is equivalent to the expected degenerate
    // mesh. The degenerate mesh is a triangle fan around a centroid. We model
    // the degeneracy as four identical vertices with the appropriate face
    // topologies.
    const RigidTransform<double>& X_WF_d = convert_to_double(X_WF);
    std::vector<SurfaceVertex<double>> expected_vertices_W = {
        SurfaceVertex<double>(X_WF_d * Vector3d(3, 5, 2)),
        SurfaceVertex<double>(X_WF_d * Vector3d(3, 5, 2)),
        SurfaceVertex<double>(X_WF_d * Vector3d(3, 5, 2)),
        SurfaceVertex<double>(X_WF_d * Vector3d(3, 5, 2))};
    typedef SurfaceVertexIndex Index;
    std::vector<SurfaceFace> expected_faces = {
        SurfaceFace(Index(0), Index(1), Index(3)),
        SurfaceFace(Index(1), Index(2), Index(3)),
        SurfaceFace(Index(2), Index(0), Index(3))};
    const SurfaceMesh<double> expected_mesh_W(move(expected_faces),
                                              move(expected_vertices_W));
    const SurfaceMesh<T> actual_mesh_W(move(this->new_faces()),
                                       move(this->new_vertices_W()));

    SCOPED_TRACE("Triangle outside half space has single vertex on the plane");
    this->VerifyMeshesEquivalent(expected_mesh_W, actual_mesh_W);
  }

  {
    // Case: one vertex on the boundary, two vertices inside.
    const SurfaceMesh<double> mesh_F = CreateOneTriangleMesh(
        Vector3d(3, 5, 2), Vector3d(4, 5, 1), Vector3d(3, 6, 1));

    // Verify the intersection.
    this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, X_WF);

    SCOPED_TRACE("Triangle inside half space has single vertex on the plane");
    this->VerifyMeshesEquivalent(
        CreateMeshWithCentroids(mesh_F, X_WF),
        SurfaceMesh<T>{move(this->new_faces()), move(this->new_vertices_W())});
  }
}

// Verifies that a triangle that has exactly two vertices lying on the half
// space produces either (a) a degenerate intersection (if the remaining vertex
// lies outside the half space) or (b) the original triangle (if the other
// vertex lies within the half space). This covers Case 1.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, EdgeOnHalfspaceIntersection) {
  using T = TypeParam;

  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<T> X_WF(
      RotationMatrix<T>(
          AngleAxis<T>{M_PI / 7, Vector3<T>{1, 2, 3}.normalized()}),
      Vector3<T>{-0.25, 0.5, 0.75});

  {
    // Case: two vertices on the boundary, one outside.
    const SurfaceMesh<double> mesh_F = CreateOneTriangleMesh(
        Vector3d(3, 5, 3), Vector3d(2, 5, 2), Vector3d(4, 5, 2));

    this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, X_WF);

    EXPECT_EQ(this->new_vertices_W().size(), 5);
    EXPECT_EQ(this->new_faces().size(), 4);
    EXPECT_EQ(this->vertices_to_newly_created_vertices().size(), 2);
    EXPECT_EQ(this->edges_to_newly_created_vertices().size(), 2);

    //                    a
    //                   ╱ ╲
    // ^ z              ╱   ╲
    // |         ______╱__.__╲_____
    // |         ▒▒▒b/e▒▒▒f▒▒▒c/d▒▒
    // |         ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
    // |         ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
    // --------> x
    //
    // The degenerate quad is formed by (b, c, d, e) which results in a triangle
    // fan around centroid f.

    // Check that the mesh that results is equivalent to the expected degenerate
    // mesh.
    const RigidTransform<double>& X_WF_d = convert_to_double(X_WF);
    std::vector<SurfaceVertex<double>> expected_vertices = {
        SurfaceVertex<double>(X_WF_d * Vector3d(2, 5, 2)),
        SurfaceVertex<double>(X_WF_d * Vector3d(4, 5, 2)),
        SurfaceVertex<double>(X_WF_d * Vector3d(4, 5, 2)),
        SurfaceVertex<double>(X_WF_d * Vector3d(2, 5, 2)),
        SurfaceVertex<double>(X_WF_d * Vector3d(3, 5, 2))};
    typedef SurfaceVertexIndex Index;
    const Index b(0), c(1), d(2), e(3), f(4);
    std::vector<SurfaceFace> expected_faces = {
        SurfaceFace(b, c, f), SurfaceFace(c, d, f), SurfaceFace(d, e, f),
        SurfaceFace(e, b, f)};

    SCOPED_TRACE("Triangle outside half space has single edge on the plane");
    this->VerifyMeshesEquivalent(
        SurfaceMesh<double>{move(expected_faces), move(expected_vertices)},
        SurfaceMesh<T>{move(this->new_faces()), move(this->new_vertices_W())});
  }

  {
    // Case: two vertices on the boundary, one vertex inside.
    const SurfaceMesh<double> mesh_F = CreateOneTriangleMesh(
        Vector3d(3, 5, 1), Vector3d(4, 5, 2), Vector3d(3, 6, 2));

    // Verify the intersection.
    this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, X_WF);

    SCOPED_TRACE("Triangle inside half space has single edge on the plane");
    this->VerifyMeshesEquivalent(
        CreateMeshWithCentroids(mesh_F, X_WF),
        SurfaceMesh<T>{move(this->new_faces()), move(this->new_vertices_W())});
  }
}

// Verifies that a triangle that has two vertices within the half space and
// another outside the half space produces a quadrilateral. This covers Case 2.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, QuadrilateralResults) {
  using T = TypeParam;

  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<T> X_WF(
      RotationMatrix<T>(
          AngleAxis<T>{M_PI / 7, Vector3<T>{1, 2, 3}.normalized()}),
      Vector3<T>{-0.25, 0.5, 0.75});

  // Construct one vertex of the triangle to lie outside the half space and the
  // other two to lie inside the half space.
  const SurfaceMesh<double> mesh_F = CreateOneTriangleMesh(
      Vector3d(3, 5, 3), Vector3d(2, 5, 1), Vector3d(4, 5, 1));

  const Vector3d nhat_F =
      (mesh_F.vertices()[1].r_MV() - mesh_F.vertices()[0].r_MV())
          .cross(mesh_F.vertices()[2].r_MV() - mesh_F.vertices()[0].r_MV())
          .normalized();

  //                   a
  //                   ╱╲
  // ^ z              ╱  ╲
  // |         _____e╱____╲d____
  // |         ▒▒▒▒▒╱▒▒▒▒▒▒╲▒▒▒▒
  // |         ▒▒▒▒╱________╲▒▒▒
  // |         ▒▒▒b▒▒▒▒▒▒▒▒▒▒c▒▒
  // |         ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
  // --------> x

  // Verify the intersection.
  this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, X_WF);

  ASSERT_EQ(this->new_faces().size(), 4);
  ASSERT_EQ(this->new_vertices_W().size(), 5);
  EXPECT_EQ(this->vertices_to_newly_created_vertices().size(), 2);
  ASSERT_EQ(this->edges_to_newly_created_vertices().size(), 2);

  // Check that the mesh that results is equivalent to the expected mesh.
  const RigidTransform<double>& X_WF_d = convert_to_double(X_WF);
  std::vector<SurfaceVertex<double>> expected_vertices_W = {
      SurfaceVertex<double>(X_WF_d * Vector3d(2, 5, 1)),     // b
      SurfaceVertex<double>(X_WF_d * Vector3d(4, 5, 1)),     // c
      SurfaceVertex<double>(X_WF_d * Vector3d(3.5, 5, 2)),   // d
      SurfaceVertex<double>(X_WF_d * Vector3d(2.5, 5, 2))};  // e
  std::vector<SurfaceFace> expected_faces;
  typedef SurfaceVertexIndex Index;
  const Index b(0), c(1), d(2), e(3);
  std::vector<Index> polygon{b, c, d, e};
  AddPolygonToMeshData(polygon, nhat_F, &expected_faces, &expected_vertices_W);

  SCOPED_TRACE("Triangle intersects; forms quad");
  this->VerifyMeshesEquivalent(
      SurfaceMesh<double>{move(expected_faces), move(expected_vertices_W)},
      SurfaceMesh<T>{move(this->new_faces()), move(this->new_vertices_W())});
}

// Verifies that a triangle that has one vertex outside the half space, one
// vertex inside the half space, and one vertex on the half space produces two
// triangles, one degenerate and one non-degenerate. This test covers Case 2.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, OutsideInsideOn) {
  using T = TypeParam;

  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<T> X_WF(
      RotationMatrix<T>(
          AngleAxis<T>{M_PI / 7, Vector3<T>{1, 2, 3}.normalized()}),
      Vector3<T>{-0.25, 0.5, 0.75});

  const SurfaceMesh<double> mesh_F = CreateOneTriangleMesh(
      Vector3d(3, 5, 3), Vector3d(2, 5, 1), Vector3d(3.5, 5, 2));

  const Vector3d nhat_F =
      (mesh_F.vertices()[1].r_MV() - mesh_F.vertices()[0].r_MV())
          .cross(mesh_F.vertices()[2].r_MV() - mesh_F.vertices()[0].r_MV())
          .normalized();

  // Verify the intersection.
  this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, X_WF);

  ASSERT_EQ(this->edges_to_newly_created_vertices().size(), 2);

  //                   a
  //                   ╱╲
  // ^ z              ╱  ╲
  // |         _____e╱____╲c/d___
  // |         ▒▒▒▒▒╱▒▒▒▒▒▒▒▒▒▒▒▒
  // |         ▒▒▒▒╱▒▒▒▒▒▒▒▒▒▒▒▒▒
  // |         ▒▒▒b▒▒▒▒▒▒▒▒▒▒▒▒▒▒
  // |         ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
  // --------> x
  //
  // Note: there is an unvisualized edge between vertices b and c -- ascii art
  // can't draw a line at that angle.

  // Check that the mesh that results is equivalent to the expected mesh.
  const RigidTransform<double>& X_WF_d = convert_to_double(X_WF);
  std::vector<SurfaceVertex<double>> expected_vertices_W = {
      SurfaceVertex<double>(X_WF_d * Vector3d(2, 5, 1)),      // b
      SurfaceVertex<double>(X_WF_d * Vector3d(3.5, 5, 2)),    // c
      SurfaceVertex<double>(X_WF_d * Vector3d(3.5, 5, 2)),    // d
      SurfaceVertex<double>(X_WF_d * Vector3d(2.5, 5, 2))};   // e
  std::vector<SurfaceFace> expected_faces;
  typedef SurfaceVertexIndex Index;
  const Index b(0), c(1), d(2), e(3);
  std::vector<Index> polygon{b, c, d, e};
  AddPolygonToMeshData(polygon, nhat_F, &expected_faces, &expected_vertices_W);

  SCOPED_TRACE("Triangle intersects; single vertex on boundary");
  this->VerifyMeshesEquivalent(
      SurfaceMesh<double>{move(expected_faces), move(expected_vertices_W)},
      SurfaceMesh<T>{move(this->new_faces()), move(this->new_vertices_W())});
}

// Verifies that a triangle with one vertex inside the half space and two
// vertices outside of the half space produces a triangle. This test covers
// Case 3.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, OneInsideTwoOutside) {
  using T = TypeParam;

  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<T> X_WF(
      RotationMatrix<T>(
          AngleAxis<T>{M_PI / 7, Vector3<T>{1, 2, 3}.normalized()}),
      Vector3<T>{-0.25, 0.5, 0.75});

  const SurfaceMesh<double> mesh_F = CreateOneTriangleMesh(
      Vector3d(3, 5, 3), Vector3d(4, 5, 3), Vector3d(3, 6, 1));

  const Vector3d nhat_F =
      (mesh_F.vertices()[1].r_MV() - mesh_F.vertices()[0].r_MV())
          .cross(mesh_F.vertices()[2].r_MV() - mesh_F.vertices()[0].r_MV())
          .normalized();

  // ^ z
  // |            b ________ c
  // |              ╲      ╱
  // |         _____e╲____╱d____
  // |         ▒▒▒▒▒▒▒╲▒▒╱▒▒▒▒▒▒
  // |         ▒▒▒▒▒▒▒▒╲╱▒▒▒▒▒▒▒
  // |         ▒▒▒▒▒▒▒▒a▒▒▒▒▒▒▒▒
  // --------> x

  // Verify the intersection.
  this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, X_WF);

  // Check that the mesh that results is equivalent to the expected mesh.
  const RigidTransform<double>& X_WF_d = convert_to_double(X_WF);
  std::vector<SurfaceVertex<double>> expected_vertices_W = {
      SurfaceVertex<double>(X_WF_d * Vector3d(3, 6, 1)),
      SurfaceVertex<double>(X_WF_d * Vector3d(3, 5.5, 2)),
      SurfaceVertex<double>(X_WF_d * Vector3d(3.5, 5.5, 2))};
  std::vector<SurfaceFace> expected_faces;
  typedef SurfaceVertexIndex Index;
  std::vector<Index> polygon{Index(0), Index(1), Index(2)};
  AddPolygonToMeshData(polygon, nhat_F, &expected_faces, &expected_vertices_W);

  SCOPED_TRACE("Triangle intersects; forms a smaller triangle");
  this->VerifyMeshesEquivalent(
      SurfaceMesh<double>{move(expected_faces), move(expected_vertices_W)},
      SurfaceMesh<T>{move(this->new_faces()), move(this->new_vertices_W())});
}

// Tests that a mesh of a box bisected by the half space produces the expected
// number of faces and vertices (other unit tests in this file assess the
// correctness of various kinds of triangle/half space intersections).
TYPED_TEST_P(MeshHalfspaceIntersectionTest, BoxMesh) {
  using T = TypeParam;

  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<T> X_WF(
      RotationMatrix<T>(
          AngleAxis<T>{M_PI / 7, Vector3<T>{1, 2, 3}.normalized()}),
      Vector3<T>{-0.25, 0.5, 0.75});

  // Set B to an arbitrarily chosen pose relative to F.
  RigidTransform<T> X_FB(math::RollPitchYaw<T>(M_PI_4, M_PI_4, M_PI_4),
                               Vector3<T>(1.0, 2.0, 3.0));
  RigidTransform<double> X_FB_d = convert_to_double(X_FB);

  const SurfaceMesh<double> mesh_F = CreateBoxMesh(X_FB_d);

  // Construct the half-space.
  const Vector3<T> Bz_F = X_FB.rotation().col(2);
  const PosedHalfSpace<T> half_space_F(Bz_F, X_FB.translation());
  typedef SurfaceFaceIndex FIndex;

  {
    // Case: Plane doesn't intersect. In fact, the plane _does_ intersect the
    // mesh, but we will simulate non-intersection by providing indices to
    // triangles that _don't_ intersect (the two triangles on the +z face).
    std::vector<FIndex> tri_indices{FIndex{8}, FIndex{9}};
    const std::unique_ptr<SurfaceMesh<T>> intersection_mesh_W =
        ConstructSurfaceMeshFromMeshHalfspaceIntersection(
            mesh_F, half_space_F, tri_indices, X_WF);
    EXPECT_EQ(intersection_mesh_W, nullptr);
  }

  {
    // Case: Plane intersect the mesh.

    // We pass in indices of *all* the triangles.
    std::vector<FIndex> tri_indices(mesh_F.num_elements());
    std::iota(tri_indices.begin(), tri_indices.end(), FIndex{0});

    const std::unique_ptr<SurfaceMesh<T>> intersection_mesh_W =
        ConstructSurfaceMeshFromMeshHalfspaceIntersection(
            mesh_F, half_space_F, tri_indices, X_WF);
    // Total number of vertices:            22
    //   -------------------------------------
    //   vertices lying in half space:       4
    //   vertices from split edges:          8
    //   centroid per intersection polygon: 10
    ASSERT_EQ(intersection_mesh_W->num_vertices(), 22);

    // Each of the +/- X, Y faces of the box have to triangles. They get clipped
    // into a triangle and quad. Each of those is transformed into a triangle
    // fan around its centroid, producing 3 and 4 triangles for a total of seven
    // per direction.
    // Total number of faces:                         34
    //   -----------------------------------------------
    //   -Z triangles, each fanned around centroid:    6
    //   4 X new triangles per box face:      7 * 4 = 28
    ASSERT_EQ(intersection_mesh_W->num_faces(), 34);
  }
}

REGISTER_TYPED_TEST_SUITE_P(MeshHalfspaceIntersectionTest, NoIntersection,
                            InsideOrOnIntersection,
                            VertexOnHalfspaceIntersection,
                            EdgeOnHalfspaceIntersection, QuadrilateralResults,
                            OutsideInsideOn, OneInsideTwoOutside, BoxMesh);

// The ComputeContactSurfaceFromSoftHalfSpaceRigidMesh() method has the
// following responsibilities:
//
//    - Dispatch BVH culling
//    - If no candidates, return nullptr.
//    - Pass candidates to ConstructSurfaceMeshFromMeshHalfspaceIntersection()
//    - If no faces, return nullptr.
//    - Compute pressure field on mesh.
//    - Return contact surface with mesh and pressure; field must have pointer
//      to the mesh.
//
// It is impossible to test correct BVH use (it's all fully contained within
// the implementation). So, we'll focus on intersecting and non-intersecting
// configurations and make sure the data reported is as expected.
GTEST_TEST(ComputeContactSurfaceFromSoftHalfSpaceRigidMeshTest, DoubleValued) {
  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<double> X_WF(
      RotationMatrix<double>(
          AngleAxis<double>{M_PI / 7, Vector3<double>{1, 2, 3}.normalized()}),
      Vector3<double>{-0.25, 0.5, 0.75});

  // Orient and position the mesh arbitrarily in Frame F.
  const RigidTransform<double> X_FB(
      math::RollPitchYaw<double>(M_PI_4, M_PI_4, M_PI_4),
      Vector3<double>(1.0, 2.0, 3.0));
  const SurfaceMesh<double> mesh_F = CreateBoxMesh(X_FB);
  const GeometryId mesh_id = GeometryId::get_new_id();
  const BoundingVolumeHierarchy<SurfaceMesh<double>> bvh_F(mesh_F);

  // Construct the half-space.
  const GeometryId hs_id = GeometryId::get_new_id();
  const double pressure_scale{1.5};

  {
    // Case: non-intersecting.

    // Put the half space well below the box.
    const RigidTransform<double> X_WH{Vector3<double>{0, 0, -5}};
    const std::unique_ptr<ContactSurface<double>> contact_surface =
        ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
            hs_id, X_WH, pressure_scale, mesh_id, mesh_F, bvh_F, X_WF);
    EXPECT_EQ(contact_surface, nullptr);
  }

  {
    // Case: intersecting.

    // The half space is defined in Frame H. The box is defined in Frame B.
    // Have the half space's boundary plane lie _near_ the box's origin and its
    // normal not quite aligned with Bz -- the goal is to get a contact surface
    // that has the same general _topology_ as the
    // (MeshHalfSpaceIntersectionTest, BoxMesh) test (see above), but not
    // actually have the half space perfectly aligned with the mesh.
    // We pick a rotation roughly 5 degrees away from Bz and a small offset
    // from the origin.
    const RigidTransform<double> X_BH{
        AngleAxis<double>{M_PI * 0.027, Vector3d{1, 1, 0}.normalized()},
        Vector3<double>{0.1, -0.05, 0.075}};
    const RigidTransform<double> X_WH = X_WF * X_FB * X_BH;
    const std::unique_ptr<ContactSurface<double>> contact_surface =
        ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
            hs_id, X_WH, pressure_scale, mesh_id, mesh_F, bvh_F, X_WF);
    ASSERT_NE(contact_surface, nullptr);

    // Quick reality check on the topology of the contact surface's mesh. See
    // BoxMesh test for the origin of the numbers 22 and 34.
    EXPECT_EQ(contact_surface->mesh_W().num_vertices(), 22);
    EXPECT_EQ(contact_surface->mesh_W().num_faces(), 34);

    // Confirm wiring in the contact surface.
    EXPECT_EQ(&contact_surface->mesh_W(), &contact_surface->e_MN().mesh());

    // Confirm pressure values.
    PosedHalfSpace<double> hs_W{X_WH.rotation().col(2), X_WH.translation()};
    // Simply test each vertex and confirm that the corresponding pressure
    // value is expected.
    const SurfaceMesh<double>& mesh_W = contact_surface->mesh_W();
    for (SurfaceVertexIndex v(0); v < mesh_W.num_vertices(); ++v) {
      const double pressure = contact_surface->e_MN().EvaluateAtVertex(v);
      const Vector3d& p_WV = mesh_W.vertex(v).r_MV();
      // Pressure is a function of _penetration depth_ = -signed distance.
      const double depth = -hs_W.CalcSignedDistance(p_WV);
      const double expected_pressure = depth * pressure_scale;
      EXPECT_NEAR(pressure, expected_pressure, 1e-15);
    }
  }
}


// Confirm that the rigid-soft intersection correctly culls backface geometry.
GTEST_TEST(CompupteContactSurfaceFromSoftHalfSpaceRigidMeshTest, BackfaceCull) {
  // For this test, we're just testing for culling. We presume that for a given
  // configuration of mesh and half space, the right calculations will be done
  // to compute the intersection mesh. So, to that end:
  //
  //   1. We'll leave the box centered on the world frame origin and aligned
  //      with the world frame basis.  This implies that B = F = W.
  //   2. However, we'll pose the half space in the world frame such that:
  //      a. its normal_W is *not* Wz (so we can confirm that the correct normal
  //         value is used for culling).
  //      b. it is positioned such that the *entire* box is contained in the
  //         half space's volume.

  const RigidTransform<double> X_WF, X_FB;  // Both the identity.

  const SurfaceMesh<double> mesh_F = CreateBoxMesh(X_FB);
  const GeometryId mesh_id = GeometryId::get_new_id();
  const BoundingVolumeHierarchy<SurfaceMesh<double>> bvh_F(mesh_F);

  // Construct the half-space.
  const GeometryId hs_id = GeometryId::get_new_id();
  // We align the half space normal along a diagonal of the box (to ease the
  // prediction of what the outcome should be -- see below).
  const Vector3<double> normal_W = Vector3<double>{1, -1, -1}.normalized();
  const RigidTransform<double> X_WH = HalfSpace::MakePose(
      normal_W, Vector3<double>{0, 0, -30});
  const double pressure_scale{1.5};

  const std::unique_ptr<ContactSurface<double>> contact_surface =
      ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
          hs_id, X_WH, pressure_scale, mesh_id, mesh_F, bvh_F, X_WF);
  // It definitely produces a contact surface.
  ASSERT_NE(contact_surface, nullptr);

  const SurfaceMesh<double>& contact_mesh_W = contact_surface->mesh_W();

  // For a wholly enclosed mesh _without_ backface culling, the contact surface
  // would be the full mesh. The input mesh has two triangles per face for a
  // total of 12 triangles and 8 vertices. As a contact surface, each of those
  // triangles would be split around its centroid. This means for each triangle
  // in the input mesh, we have three triangles in the contact surface mesh,
  // and one additional 1 vertex. Thus the unculled contact surface would have
  // 8 + 12 = 20 vertices and 12 * 3 = 36 triangles.
  //
  // In fact, because the face normal points along one of the box diagonals.
  // Only three of the box's faces will contribute to the contact surface.
  // So, we'll have 18 faces (6 triangles per face). For vertices, we'll include
  // all but one of the input mesh's vertices (7), and introduce one vertex per
  // input triangle (6) for a total of 13 vertices.
  ASSERT_EQ(contact_mesh_W.num_faces(), 18);
  ASSERT_EQ(contact_mesh_W.num_vertices(), 13);

  // Let's explicitly confirm that every face in the contact surface mesh passes
  // the culling test.
  // There are two tricky bits in setting up this test.
  //
  //   1. The gradient of the half space pressure is in the opposite direction
  //      as its normal. So, in normal circumstances: grad_p_W = -normal_W.
  //   2. We only know that the normals of the contact surface mesh point *out*
  //      of geometry N and *into* geometry M (in the contact surface). We don't
  //      know what the mapping between those ids and the mesh and half space
  //      ids are. If id N is the same as the mesh_id, then we know the contact
  //      surface normals point in the mesh normal direction. We're done.
  //      However, if mesh_id is the id for M, the contact surface mesh normals
  //      point in the opposite direction as the input mesh. So, to evaluate
  //      an _equivalent_ culling test, we should likewise reverse the gradient
  //      direction. That's what the next two lines of code do to define the
  //      gradient direction vector.
  const double grad_scale = contact_surface->id_N() == mesh_id ? 1.0 : -1.0;
  const Vector3<double> grad_p_W = grad_scale * -normal_W;
  for (SurfaceFaceIndex tri(0); tri < contact_mesh_W.num_faces(); ++tri) {
    // Everything is defined in the world frame; so the transform X_WM = I.
    ASSERT_TRUE(
        IsFaceNormalInNormalDirection(grad_p_W, contact_mesh_W, tri, {}))
        << "Face " << tri;
  }
  // Note: this confirms that the reported faces satisfy the non-culling
  // criteria, but doesn't (can't) confirm that the omitted faces don't. Rethink
  // this test if that proves to be a problem (highly unlikely).
}

// AutoDiff is not currently supported and should throw with an acceptable
// message.
GTEST_TEST(ComputeContactSurfaceFromSoftHalfSpaceRigidMeshTest,
           AutoDiffValued) {
  using T = AutoDiffXd;
  const RigidTransform<T> X_WF;

  const SurfaceMesh<double> mesh_F =
      CreateOneTriangleMesh({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
  const GeometryId mesh_id = GeometryId::get_new_id();
  const BoundingVolumeHierarchy<SurfaceMesh<double>> bvh_F(mesh_F);

  // Construct the half-space.
  const RigidTransform<T> X_WH;
  const GeometryId hs_id = GeometryId::get_new_id();
  const double pressure_scale{1.5};

  DRAKE_EXPECT_THROWS_MESSAGE(
      ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
          hs_id, X_WH, pressure_scale, mesh_id, mesh_F, bvh_F, X_WF),
      std::logic_error,
      "AutoDiff-valued ContactSurface calculations are not currently "
      "supported");
}

// TODO(SeanCurtis-TRI): All of the tests where we actually examine the mesh
//  use a mesh with a _single_ triangle. In this case, the face-local index
//  values are perfectly aligned with the mesh-local index values. This hides
//  potential errors where the two sets of indices do _not_ align so nicely.
//  Add a test where we intersect a face that doesn't use vertices 0, 1, and 2.

// TODO(SeanCurtis-TRI): The AutoDiffXd type here is *largely* a smoke test.
//  We're confirming that it builds and that it runs. We're also confirming that
//  the _value_ is correct -- we have _not_ validated that the derivative values
//  are correct. That still needs to be done when AutoDiffXd support for
//  hydroelastics is fully supported.
typedef ::testing::Types<double, AutoDiffXd> MyTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, MeshHalfspaceIntersectionTest, MyTypes);

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
