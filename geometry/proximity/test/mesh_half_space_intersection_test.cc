#include "drake/geometry/proximity/mesh_half_space_intersection.h"

#include <limits>
#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/utilities.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransform;
using math::RotationMatrix;
using std::make_unique;
using std::move;
using std::unique_ptr;

// Creates a SurfaceMesh of a box. The box is defined in frame B, centered on
// Bo with its dimensions aligned with Bx, By, and Bz. The resultant mesh
// has the vertices expressed in Frame F, courtesy of the transform `X_FB`.
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

// Creates a new SurfaceMesh mesh from the given input mesh, such that the
// resulting mesh has the same domain, but with two differences:
//
//   1. The vertices in the resultant mesh are measured and expressed in
//      Frame W (as opposed to the input mesh's Frame F)
//   2. Each triangle in the input mesh has been turned into three triangles;
//      the original turned into a triangle fan around its centroid.
// This method confirms that the face normal directions are preserved.
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

// Creates a SurfaceMesh with a single triangle using the given vertices.
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

/* This test evaluates the *values* of the mesh-half space intersection
 algorithm for all supported scalars (double and AutoDiffXd). The derivatives
 for AutoDiffXd are tested below. */
template <typename T>
class MeshHalfSpaceValueTest : public ::testing::Test {
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
TYPED_TEST_SUITE_P(MeshHalfSpaceValueTest);

// Verifies that a triangle that lies fully outside of the half space yields an
// empty intersection. This covers Case 4 in the code.
TYPED_TEST_P(MeshHalfSpaceValueTest, NoIntersection) {
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
TYPED_TEST_P(MeshHalfSpaceValueTest, InsideOrOnIntersection) {
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
TYPED_TEST_P(MeshHalfSpaceValueTest, VertexOnHalfspaceIntersection) {
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
TYPED_TEST_P(MeshHalfSpaceValueTest, EdgeOnHalfspaceIntersection) {
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
TYPED_TEST_P(MeshHalfSpaceValueTest, QuadrilateralResults) {
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
TYPED_TEST_P(MeshHalfSpaceValueTest, OutsideInsideOn) {
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
TYPED_TEST_P(MeshHalfSpaceValueTest, OneInsideTwoOutside) {
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
TYPED_TEST_P(MeshHalfSpaceValueTest, BoxMesh) {
  using T = TypeParam;

  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<T> X_WF(RotationMatrix<T>(AngleAxis<T>{
                                   M_PI / 7, Vector3<T>{1, 2, 3}.normalized()}),
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
        ConstructSurfaceMeshFromMeshHalfspaceIntersection(mesh_F, half_space_F,
                                                          tri_indices, X_WF);
    EXPECT_EQ(intersection_mesh_W, nullptr);
  }

  {
    // Case: Plane intersect the mesh.

    // We pass in indices of *all* the triangles.
    std::vector<FIndex> tri_indices(mesh_F.num_elements());
    std::iota(tri_indices.begin(), tri_indices.end(), FIndex{0});

    const std::unique_ptr<SurfaceMesh<T>> intersection_mesh_W =
        ConstructSurfaceMeshFromMeshHalfspaceIntersection(mesh_F, half_space_F,
                                                          tri_indices, X_WF);
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

REGISTER_TYPED_TEST_SUITE_P(MeshHalfSpaceValueTest, NoIntersection,
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
//    - Report the gradients of soft half space's pressure field across all
//      triangles.
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
  const Bvh<SurfaceMesh<double>> bvh_F(mesh_F);

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

  // Define an intersecting configuration.
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

  {
    // Case: intersecting configuration (ignoring the gradients of the
    // constituent pressure fields).
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

  {
    // Case:  A repeat of the previous test, this time, we simply test for the
    // existence (and correctness) of the pressure field gradient associated
    // with the half space; it should be the negative scaled half space normal.
    // We'll repeat the test twice, swapping ids to make sure the pressure field
    // moves from M to N.
    const Vector3d grad_eH_W_expected =
        -pressure_scale * X_WH.rotation().col(2);
    {
      const std::unique_ptr<ContactSurface<double>> contact_surface =
          ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
              hs_id, X_WH, pressure_scale, mesh_id, mesh_F, bvh_F, X_WF);
      ASSERT_NE(contact_surface, nullptr);
      ASSERT_LT(mesh_id, hs_id);
      EXPECT_FALSE(contact_surface->HasGradE_M());
      EXPECT_TRUE(contact_surface->HasGradE_N());

      EXPECT_GT(contact_surface->mesh_W().num_faces(), 0);
      for (SurfaceFaceIndex f(0); f < contact_surface->mesh_W().num_faces();
           ++f) {
        ASSERT_TRUE(CompareMatrices(contact_surface->EvaluateGradE_N_W(f),
                                    grad_eH_W_expected));
      }
    }

    {
      const std::unique_ptr<ContactSurface<double>> contact_surface =
          ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
              mesh_id, X_WH, pressure_scale, hs_id, mesh_F, bvh_F, X_WF);
      ASSERT_NE(contact_surface, nullptr);
      EXPECT_TRUE(contact_surface->HasGradE_M());
      EXPECT_FALSE(contact_surface->HasGradE_N());

      EXPECT_GT(contact_surface->mesh_W().num_faces(), 0);
      for (SurfaceFaceIndex f(0); f < contact_surface->mesh_W().num_faces();
           ++f) {
        ASSERT_TRUE(CompareMatrices(contact_surface->EvaluateGradE_M_W(f),
                                    grad_eH_W_expected));
      }
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
  const Bvh<SurfaceMesh<double>> bvh_F(mesh_F);

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

// TODO(SeanCurtis-TRI): All of the tests where we actually examine the mesh
//  use a mesh with a _single_ triangle. In this case, the face-local index
//  values are perfectly aligned with the mesh-local index values. This hides
//  potential errors where the two sets of indices do _not_ align so nicely.
//  Add a test where we intersect a face that doesn't use vertices 0, 1, and 2.

typedef ::testing::Types<double, AutoDiffXd> MyTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, MeshHalfSpaceValueTest, MyTypes);

/* This test fixture enables some limited testing of the autodiff-valued contact
 surface. It computes the intersection between a half space and simple triangle
 mesh. The basic configuration looks like this:

                                 b
                               ╱│                   ▒ - portion of triangle mesh
                              ╱ │                       inside the half space.
                             ╱  │                   n - outward-pointing
                ────────────╱   │────────────────       half space normal.
               ╱░░░░░░░░a░░╱   h│░░░░░░░░░░░░░░╱    a, b, c, d - mesh vertices.
              ╱░░░░░░░░░░░│\   ╱│░░░░░░░░░░░░░╱         In the rigid frame R,
             ╱░░░░░░░░░░░░│ \ ╱▒│░░░░░░░░░░░░╱          a and b are at z = 0 and
            ╱░░n░░░░░░░░░░│ g╱▒▒│░░░░░░░░░░░╱           c and d are at z = -1.
           ╱░░░^░░░░░░░░░░│ ╱▒\▒│░░░░░░░░░░╱        f, g, h - vertices on the
          ╱░░░░│░░░░░░░░░░│╱▒▒▒\│░░░░░░░░░╱             surface of the half
         ╱░░░░░│░░░░░░░░░f│▒▒j▒╱░d░░░░░░░╱              space, formed by
        ╱░░░░░░░░░░░░░░░░░│▒▒▒╱░░░░░░░░░╱               intersecting edges.
       ╱░░░░░░░░░░░░░░░░░░│▒▒╱░░░░░░░░░╱            j - the centroid of quad
      ╱░░░░░░░░░░░░░░░░░░░│▒╱░░░░░░░░░╱                 (cdgf). We don't show
     ╱░░░░░░░░░░░░░░░░░░░░│╱░░░░░░░░░╱                  the centroid of triangle
    ╱░░░░░░░░░░░░░░░░░░░░░c░░░░░░░░░╱                   (dgh).
    ───────────────────────────────

 In the image above, the rectangle mesh is perpendicular to the half space
 boundary. The function TestPositionDerivative() will compute multiple contact
 surfaces by varying the relative orientation of mesh and and half space. (See
 notes for that function.) It uses AutoDiffXd to automatically compute the
 derivatives of all calculations *with respect to the position of frame R's
 origin*. This is selected because it's a simple derivative to reason about.

 Each individual unit test will create a function that consumes a contact
 surface, the pose of the surface X_WR, and the angle between mesh and Wz
 (theta) and test some aspect of the mesh and its derivatives. The
 TestPositionDerivative() invokes that function for each unique theta. */
class MeshHalfSpaceDerivativesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    /* Set up the *soft* half space. */
    id_S_ = GeometryId::get_new_id();
    pressure_scale_ = 1.5e5;

    /* Set up the *rigid* mesh. */
    using Vertex = SurfaceVertex<double>;
    std::vector<Vertex> vertices{Vertex{Vector3d{0, -kWidth / 2, 0}},   // a
                                 Vertex{Vector3d{0, kWidth / 2, 0}},    // b
                                 Vertex{Vector3d{0, -kWidth / 2, -1}},  // c
                                 Vertex{Vector3d{0, kWidth / 2, -1}}};  // d
    using VIndex = SurfaceVertexIndex;
    using Face = SurfaceFace;
    std::vector<SurfaceFace> faces{{Face{VIndex(0), VIndex(3), VIndex(1)},
                                    Face{VIndex(0), VIndex(2), VIndex(3)}}};
    id_R_ = GeometryId::get_new_id();
    mesh_R_ = make_unique<SurfaceMesh<double>>(move(faces), move(vertices));
    bvh_R_ = make_unique<Bvh<SurfaceMesh<double>>>(*mesh_R_);
  }

  /* Test for a particular quantity of the contact surface with multiple
   rotations. For each rotation, the mesh is positioned so that a fixed portion
   (25%) of the mesh penetrates into the half space. For each configuration, the
   evaluate_quantity() is invoked.

   @param evaluate_quantity  A function that assess some aspect of the contact
                             surface and its derivatives. It must be written
                             to take a variety of relative poses between
                             half space and mesh. The provided function should
                             make use of googletest EXPECT_* macros to perform
                             the assessment.
   @param diff_wrt_position  If true, we differentiate w.r.t. Ro (origin of the
                             rigid mesh's frame). If false, we differentiate
                             w.r.t. theta -- R_WR is a rotation of theta radians
                             around Wy. */
  void TestPositionDerivative(
      const std::function<void(const ContactSurface<AutoDiffXd>&,
                               const RigidTransform<AutoDiffXd>&, double)>&
          evaluate_quantity) {
    /* Theta is the angle between the half space's normal (Wz) and the plane
     of the rectangular mesh. At 90 degrees, the mesh lies parallel with the
     half space surface -- we don't sample at 90 degrees.  */
    for (const double theta :
         {0.0, M_PI / 6, M_PI / 2 * 0.9, M_PI / 2 * 0.99}) {
      /* Set the mesh so it's 25% in and 75% out of the half space. The x- and
       y- components don't matter. We just want to make sure they aren't zero.
       */
      const double z = std::cos(theta) * 0.75;
      const Vector3<AutoDiffXd> p_WR =
          math::initializeAutoDiff(Vector3d{0.25, -0.7, z});
      RigidTransform<AutoDiffXd> X_WR(
          RotationMatrix<AutoDiffXd>::MakeYRotation(theta), p_WR);

      auto surface = ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
          id_S_, X_WS_, pressure_scale_, id_R_, *mesh_R_, *bvh_R_, X_WR);
      ASSERT_NE(surface, nullptr);

      /* Categorize the surface vertices so that quantities that depend on
       understanding the vertex categories can operate. */
      CategorizeVertices(*surface, X_WR, theta);

      /* Two triangles intersect; one gets cut into a quad, the other a
       triangle. Together, they contribute 4 + 3 triangles to the contact
       surface. */
      ASSERT_EQ(surface->mesh_W().num_faces(), 7);

      SCOPED_TRACE(fmt::format("theta = {:.5f} radians", theta));
      evaluate_quantity(*surface, X_WR, theta);
    }
  }

  /* Given the mesh corresponding to a contact surface, it categorizes each
   vertex as being "original", "surface", or "centroid" (see the definition
   of VertexCategory below).

   This function makes a number of assumptions about the nature of the contact
   surface. Specifically, it assumes that it is some variant that is illustrated
   in the class documentation.

     - The half space surface is the Wz = 0 plane.
     - Vertices a and b lie outside the half space
     - Vertices c and d lie inside the half space.
     - Vertices ca  at the *same* known depth (based on the dimensions of the
       rigid mesh, the rotation around Ry, and the position of Ro).

   Invalidating any of these assumptions may lead to googletest assertions
   failing or, worse yet, silent, erroneous categorizations. */
  void CategorizeVertices(const ContactSurface<AutoDiffXd>& surface,
                          const RigidTransform<AutoDiffXd>& X_WR,
                          double theta) {
    const SurfaceMesh<AutoDiffXd>& mesh_W = surface.mesh_W();
    constexpr double kEps = std::numeric_limits<double>::epsilon();
    vertex_data_.clear();
    using VIndex = SurfaceVertexIndex;
    /* Return the indices of the indicated vertex's neighbors. This assumes
     that we have a contact surface with good topological properties (i.e., no
     duplicated vertices such that two adjacent triangles reference the same
     vertex index). */
    auto find_neighbors = [&mesh_W](VIndex v) {
      std::set<VIndex> neighbors;
      for (SurfaceFaceIndex t(0); t < mesh_W.num_elements(); ++t) {
        const auto& tri = mesh_W.element(t);
        for (int i = 0; i < 3; ++i) {
          if (tri.vertex(i) == v) {
            neighbors.insert(tri.vertex((i + 1) % 3));
            neighbors.insert(tri.vertex((i + 2) % 3));
            break;
          }
        }
      }
      return neighbors;
    };

    const double Ro_z = X_WR.translation().z().value();
    const double lowest_z = Ro_z - std::cos(theta);
    for (VIndex i(0); i < mesh_W.num_vertices(); ++i) {
      const Vector3<AutoDiffXd>& p_WV = mesh_W.vertex(i).r_MV();
      if (p_WV.z().value() > -kEps) {
        /* We stash a zero vector for now and will update it after all vertices
         have been categorized. */
        vertex_data_.push_back({kSurface, Vector3<double>::Zero()});
      } else if (p_WV.z() < lowest_z + kEps) {
        vertex_data_.push_back({kOriginal, 0});
      } else {
        vertex_data_.push_back({kCentroid, find_neighbors(i)});
      }
    }

    /* Surface vertices will be adjacent to one or more surface vertices, one
     or more centroid vertices, but only one original vertex. We can infer the
     direction of the edge that intersected the half space boundary by looking
     at the direction from the original vertex to the surface vertex. */
    for (VIndex i(0); i < mesh_W.num_vertices(); ++i) {
      if (vertex_data_[i].category == kSurface) {
        std::set<VIndex> neighbors = find_neighbors(i);
        std::set<VIndex> original_neighbors;
        for (VIndex n : neighbors) {
          if (vertex_data_[n].category == kOriginal) {
            original_neighbors.insert(n);
          }
        }
        DRAKE_DEMAND(original_neighbors.size() == 1);
        const Vector3<AutoDiffXd>& v_WS = mesh_W.vertex(i).r_MV();
        const Vector3<AutoDiffXd>& v_WO =
            mesh_W.vertex(*original_neighbors.begin()).r_MV();
        vertex_data_[i].data =
            math::autoDiffToValueMatrix((v_WS - v_WO).normalized());
      }
    }
  }

  /* The soft half space. */
  const RigidTransform<AutoDiffXd> X_WS_;
  GeometryId id_S_;
  double pressure_scale_{};

  /* The rigid mesh. */
  GeometryId id_R_;
  unique_ptr<SurfaceMesh<double>> mesh_R_;
  unique_ptr<Bvh<SurfaceMesh<double>>> bvh_R_;
  static constexpr double kWidth = 1;

  /* Classification of contact surface vertices.

   The vertices in the contact surface can be classified into three
   categories.

     1. Original vertices from the triangular mesh.
       - These are the vertices on the contact surface that came from mesh
         vertices that were inside the half space.
     2. Vertices that lie on the half space surface.
       - In this test, these vertices are *created* by intersecting an edge
   with half space boundary. (Generally, it *could* include a rigid mesh
   vertex that already lies on the surface, but that is unlikely to happen
   even in the real world.) These vertices are, in some sense, *fixed* to the
         surface of the half space; they can't ever get deeper or shallower.
     3. Polygon centroids (e.g., vertex j in the diagram)
       - For continuous hydroelastic, every polygon created by mesh-half space
         intersection is tesselated into a triangle fan around its own
   centroid. In the case where the polygon is a triangle, the centroid is
   simply the average position of the triangle vertices. Where the polygon had
   > 3 vertices, it becomes a (non-trivial) weighted combination of the
         vertex positions. */
  enum Category { kOriginal, kSurface, kCentroid };
  /* For each vertex, we record its category and any data associated with that
   category. For kOriginal, there is no data and we use a dummy int value.
   For kSurface, we store the direction of the edge that created the surface
   vertex. For kCentroid, we store the indices of the neighboring vertices. */
  struct VertexData {
    Category category;
    std::variant<int, Vector3<double>, std::set<SurfaceVertexIndex>> data;
  };
  std::vector<VertexData> vertex_data_;
};

TEST_F(MeshHalfSpaceDerivativesTest, Area) {
  /* The area of the contact surface is simply the width of the rectangle times
   the length of the rectangle embedded in the half space. We'll possibly tilt
   the rectangle (around Ry) to keep things interesting. We can analyze the area
   and derivative of the area by looking at the 2D analogy.

   Consider a line segment of length 1. We need to determine what portion of
   the line segment that lies below the z = 0 line.

                  +Wz
           Ro ○ __^_______ __         ○ - Ro, the origin of the R frame.
               ╲  ┆      ┆            a - The measure of the z-component of Ro.
             ╱  ╲θ┆      ┆  a
   ──────────────╲┼──────┼────> +Wx
            1-L   ╲      ┆
               ╱  ┆╲     ┆
                  ┆ ╲    ┆  b
                 L┆  ╲   ┆
                  ┆   ╲  ┆
                  ┆    ╲θ┆
                  ┆     ╲┆ __

                       ╱

   The diagonal line shows the line segment. The frame R has been moved to the
   point Ro, and the frame has been tilted such that Rz is θ radians away Wz.
   The length we want is labeled L. If we examine the portion of the line
   segment that lies above Wz = 0, we can see that:

              Ro.z
     cos(θ) = ─────, so, 1 - L = Ro.z / cos(θ), which gives us
               1-L
     L = 1 - Ro.z / cos(θ)
     dL/dz = -1 / cos(θ)

   When we consider the problem in 3D, we can see that L does not depend on
   Ro.x or Ro.y (as any movement parallel with the half space doesn't change the
   *amount* of penetration). So,

     ∂L/∂Ro = [0, 0, -1 / cos(θ)]

   So, we can predict the area, and its gradient for a fixed mesh and
   orientation theta as:

         A = W * L(Ro)
           = W * (1 - Ro.z / cos(θ))
    ∂A/∂Ro = W * ∂L/∂Ro
           = W * [0, 0, -1 / cos(θ)]
           = [0, 0, -width / cos(θ)]
   */
  auto evaluate_area =
      [w = this->kWidth](const ContactSurface<AutoDiffXd>& surface,
                         const RigidTransform<AutoDiffXd>& X_WR, double theta) {
        constexpr double kEps = std::numeric_limits<double>::epsilon();
        const double p_WRz = X_WR.translation().z().value();
        const double expected_area = w * (1 - p_WRz / std::cos(theta));
        const AutoDiffXd total_area = surface.mesh_W().total_area();
        EXPECT_NEAR(total_area.value(), expected_area, kEps);
        EXPECT_TRUE(CompareMatrices(total_area.derivatives(),
                                    Vector3d{0, 0, -w / std::cos(theta)},
                                    10 * kEps));
      };
  TestPositionDerivative(evaluate_area);
}

TEST_F(MeshHalfSpaceDerivativesTest, VertexPosition) {
  /* In principle, we might expect that the derivatives of contact surface
   vertex position, v, with respect to Ro, ∂v/∂Ro, would simply be the Identity.
   After all, the mesh vertices are affixed to the mesh. This is not entirely
   true. The actual Jacobian of vertex position depends on the vertex category
   (see class documentation).

     1. Original vertices
        - These vertices *can* be considered to be rigidly affixed to the mesh,
         so ∂v/∂Ro = I is accurate.
     2. Surface vertices
        - We'll examine ∂v/∂Ro component wise.
          - ∂v/∂Ro.x = [1, 0, -p.x] (see Equation (1) below)
          - ∂v/∂Ro.y = [1, 0, -p.y] (see Equation (1) below)
          - ∂v/∂Ro.z = [0, 0, 0]. Surface points always lie on the surface. No
            perturbation of Ro will move surface points *off* the surface.

           - p = ê − (ê⋅n̂)n̂.                                         (1)

         Surface vertices come from edge-plane intersections. If the edge is not
         parallel to the plane normal, moving the plane along the normal will
         move the intersection point across the plane. The rate of movement
         along the plane, relative to motion perpendicular to the plane is
         captured by p, the *projection* of the edge direction onto the plane.
     3. Centroid vertices
        - For centroids of *triangles*:

                 C = (v0 + v1 + v2) / 3, therefore
            ∂C/∂Ro = (∂v0/∂Ro + ∂v1/∂Ro + ∂v2/∂Ro) / 3

          Therefore, we can confirm that ∂C/∂Ro is at least consistent with the
          derivatives of the original and surface vertices that have been
          independently confirmed.
        - For centroids of *N-gons*, the math is far more complex:
          - The polygon is decomposed into triangles, the area and centroid of
            each triangle are computed. The N-gon centroid is the weighted (by
            area) average of each of the triangle centroids. There is no simple
            way to validate this answer, so we'll skip it for now.

   We rely on the test harness to classify the vertices, and will test the
   derivatives accordingly.  */
  auto evaluate_positions =
      [w = this->kWidth, &vertex_data = this->vertex_data_](
          const ContactSurface<AutoDiffXd>& surface,
          const RigidTransform<AutoDiffXd>&, double theta) {
        constexpr double kEps = std::numeric_limits<double>::epsilon();
        // We know the normal of the half space boundary to be +Wz.
        const Vector3d n{0, 0, 1};
        for (SurfaceVertexIndex i(0); i < vertex_data.size(); ++i) {
          const auto p_WV = surface.mesh_W().vertex(i).r_MV();
          switch (vertex_data[i].category) {
            case kOriginal:
              EXPECT_TRUE(CompareMatrices(math::autoDiffToGradientMatrix(p_WV),
                                          Matrix3<double>::Identity(), kEps));
              break;
            case kSurface: {
              const Vector3d& e = std::get<Vector3d>(vertex_data[i].data);
              const double e_z = e.dot(n);
              DRAKE_DEMAND(e_z > 0);
              const Vector3d p = (e - (e.dot(n)) * n) / e_z;
              Matrix3<double> expected_J;
              expected_J << 1, 0, -p.x(), 0, 1, -p.y(), 0, 0, 0;
              const Matrix3<double> J = math::autoDiffToGradientMatrix(p_WV);
              /* For large theta, the edge directions become closer to
               perpendicular to the plane normal, so we need a looser tolerance.
               This simple switch accounts for the worst tolerance. */
              EXPECT_TRUE(CompareMatrices(J, expected_J,
                                          kEps * (theta > 1.3 ? 200 : 1.0)));
            } break;
            case kCentroid:
              if (std::get<2>(vertex_data[i].data).size() == 3) {
                /* Only attempt to validate for triangle centroids. */
                Matrix3<double> expected_J = Matrix3<double>::Zero();
                for (const auto nbr : std::get<2>(vertex_data[i].data)) {
                  expected_J += math::autoDiffToGradientMatrix(
                      surface.mesh_W().vertex(nbr).r_MV());
                }
                expected_J /= 3;
                const Matrix3<double> J = math::autoDiffToGradientMatrix(p_WV);
                EXPECT_TRUE(CompareMatrices(J, expected_J,
                                            kEps * (theta > 1.3 ? 200 : 1.0)));
              }
              break;
          }
        }
      };
  TestPositionDerivative(evaluate_positions);
}

TEST_F(MeshHalfSpaceDerivativesTest, FaceNormalsWrtPosition) {
  /* None of the face normals depend on Ro. They should all report the zero
   matrix. */
  auto evaluate_normals = [](const ContactSurface<AutoDiffXd>& surface,
                             const RigidTransform<AutoDiffXd>&, double theta) {
    constexpr double kEps = std::numeric_limits<double>::epsilon();
    const Matrix3<double> zero_matrix = Matrix3<double>::Zero();
    for (SurfaceFaceIndex t(0); t < surface.mesh_W().num_elements(); ++t) {
      const auto& n = surface.mesh_W().face_normal(t);
      /* For large theta, the mesh faces get closer to parallel with the half
       space surface, so we need a looser tolerance. This simple switch accounts
       for the observed behavior in this test. */
      EXPECT_TRUE(CompareMatrices(math::autoDiffToGradientMatrix(n),
                                  zero_matrix,
                                  kEps * (theta > 1.3 ? 1024 : 16)));
    }
  };
  TestPositionDerivative(evaluate_normals);
}

TEST_F(MeshHalfSpaceDerivativesTest, FaceNormalsWrtOrientation) {
  /* The normals n̂ *do* depend on θ. When θ = 0, the normals point in the +Rx
   direction. For this test, we rotate around the arbitrary axis v̂. The expected
   derivative is simply dn̂/dθ = v̂ × n̂  = v̂ × Rx.

   This test does *not* use the TestPositionDerivative() API because that
   differentiates with respect to Ro and makes assumptions about the resulting
   mesh. For this test, those assumptions would be invalid, so we'll simply
   duplicate that portion of TestPositionDerivative() that is relevant for this
   test. */

  /* Arbitrary rotation axis. It is important that the vector be defined such
   that rotations don't cause all intersecting triangles to be "backface
   culled".  */
  const Vector3<double> v_W = Vector3<double>{-1, 2, -3}.normalized();
  for (const double theta : {0.0, M_PI / 6, M_PI / 2 * 0.9, M_PI / 2 * 0.99}) {
    /* We'll push the origin down into the half space to guarantee
     intersection without doing otherwise unnecessary analysis). We're only
     testing normal direction; any intersection is sufficient. The x- and y-
     elements are arbitrary, non-zero values. */
    const Vector3<AutoDiffXd> p_WR{0.25, -0.7, -0.1};
    AutoDiffXd theta_ad = theta;
    theta_ad.derivatives().resize(1);
    theta_ad.derivatives() << 1;
    RigidTransform<AutoDiffXd> X_WR{
        RotationMatrix<AutoDiffXd>(AngleAxis<AutoDiffXd>(theta_ad, v_W)), p_WR};

    auto surface = ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
        id_S_, X_WS_, pressure_scale_, id_R_, *mesh_R_, *bvh_R_, X_WR);

    SCOPED_TRACE(fmt::format("theta = {:.5f} radians", theta));
    ASSERT_NE(surface, nullptr);
    /* Make sure the test doesn't pass simply because we have no triangles. */
    ASSERT_GT(surface->mesh_W().num_elements(), 0);

    /* Test dn̂/dθ = v̂ × n̂  = v̂ × Rx. */
    const Vector3<double> Rx_W =
        math::autoDiffToValueMatrix(X_WR.rotation().col(0));
    const Vector3<double> expected_deriv = v_W.cross(Rx_W);
    for (SurfaceFaceIndex t(0); t < surface->mesh_W().num_elements(); ++t) {
      const auto& n = surface->mesh_W().face_normal(t);
      /* Precision decreases as the mesh gets closer to lying parallel to the
       half space surface. This simple switch accounts for the observed
       behavior in this test. */
      EXPECT_TRUE(CompareMatrices(math::autoDiffToGradientMatrix(n),
                                  expected_deriv, theta > 1.3 ? 1e-13 : 5e-15));
    }
  }
}

TEST_F(MeshHalfSpaceDerivativesTest, Pressure) {
  /* The pressure at a point v embedded in a half space with normal n̂ and point
   on its surface o is:

     p = En̂⋅(o − v)
       = -En̂⋅v          // The origin lies on the surface; o = 0.
       = -E * v.z       // n̂ = [0, 0, 1]

     ∂p/∂Ro = -E * ∂v.z/∂Ro

   We've already tested the derivative of vertex position w.r.t. Ro, so we can
   make use of that to confirm that the pressure derivatives are consistent. */
  auto evaluate_pressures = [E = this->pressure_scale_](
                                const ContactSurface<AutoDiffXd>& surface,
                                const RigidTransform<AutoDiffXd>&,
                                double theta) {
    constexpr double kEps = std::numeric_limits<double>::epsilon();
    for (SurfaceVertexIndex v(0); v < surface.mesh_W().num_vertices(); ++v) {
      const auto& p_WV = surface.mesh_W().vertex(v).r_MV();
      const Matrix3<double> dV_dRo = math::autoDiffToGradientMatrix(p_WV);
      const Vector3d expected_dp_dRo = -E * dV_dRo.row(2);
      const AutoDiffXd& p = surface.EvaluateE_MN(v);
      EXPECT_TRUE(CompareMatrices(p.derivatives(), expected_dp_dRo, kEps));
    }
  };
  TestPositionDerivative(evaluate_pressures);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
