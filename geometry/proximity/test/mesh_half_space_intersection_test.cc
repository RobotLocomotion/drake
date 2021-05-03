#include "drake/geometry/proximity/mesh_half_space_intersection.h"

#include <limits>
#include <memory>
#include <utility>
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
using math::RigidTransformd;
using math::RotationMatrix;
using math::RotationMatrixd;
using std::make_unique;
using std::move;
using std::pair;
using std::unique_ptr;
using std::vector;

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
 surface. It computes the intersection between a half space and a simple
 triangle mesh (single tri).

 The rigid triangle, defined in the frame R, is simply:

             Rz               The triangle's normal points in the Ry direction.
             ┆   Ry           Vertex v1 is at <1, 0, 0>.
             ┆  ╱             Vertex v2 is at <0, 0, -1>.
             ┆ ╱
          v0 ┆╱    v1
      ┄┄┄┄┄┄┄●┄┄┄┄┄●┄┄┄ Rx
            ╱┆
           ╱ ┆
          ╱  ● v2

  The soft half space, defined in the frame S, has its planar boundary at Sz = 0
  with the normal in the +Sz direction.

  We will create a number of fixed poses of the triangle w.r.t. the half space:

    - horizontal slice: R_RS = I, v2 lies inside the half space and the edge
      connecting vertices 0 and 1 lies parallel with the half space boundary. It
      slices the full triangle into a smaller, similar triangle.
    - skew triangle: v2 lies inside, vertices 0 and 1 lie outside, but no edges
      are parallel with either planar boundary or plane normal. The intersecting
      polygon is a triangle.
    - skew quad: vertices v1 and v2 lie inside the half space, v0 lies outside.
      Again, no triangle edges are aligned with plane or normal. The
      intersecting polygon is a quad.

 The function TestPositionDerivative() will pose the triangle mesh and compute
 a contact surface. It invokes a provided functor to assess the reported
 derivatives of some arbitrary quantity of the contact surface with respect to
 the position of the origin of frame R. */
class MeshHalfSpaceDerivativesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    /* Set up the *soft* half space. */
    id_S_ = GeometryId::get_new_id();
    pressure_scale_ = 1.5e5;
    /* Orient and position the soft half space arbitrarily. */
    X_WS_ = HalfSpace::MakePose(Vector3d{1, 2, 3}.normalized(),
                                Vector3d{0.25, 0.1, -0.2})
                .cast<AutoDiffXd>();

    /* Set up the *rigid* mesh. */
    using Vertex = SurfaceVertex<double>;
    vector<Vertex> vertices{Vertex{Vector3d{0, 0, 0}},
                            Vertex{Vector3d{1, 0, 0}},
                            Vertex{Vector3d{0, 0, -1}}};
    using VIndex = SurfaceVertexIndex;
    using Face = SurfaceFace;
    vector<Face> faces{{Face{VIndex(0), VIndex(1), VIndex(2)}}};
    id_R_ = GeometryId::get_new_id();
    mesh_R_ = make_unique<SurfaceMesh<double>>(move(faces), move(vertices));
    bvh_R_ = make_unique<Bvh<SurfaceMesh<double>>>(*mesh_R_);
  }

  /* Indicator for the relative pose of the mesh (tri) relative to the half
   space. See class documentation for details. */
  enum TriPose { kHorizontalSlice, kSkewTriangle, kSkewQuad };

  /* Tests for an arbitrary quantity of the contact surface against multiple
   relative poses between mesh and half space. We evaluate three different
   configurations (as documented above). For each configuration, we invoke
   evaluate_quantity(). The contact surface is being differentiated with respect
   to the position of the rigid frame's origin in the world frame, p_WRo.

   @param evaluate_quantity  A function that assess some aspect of the contact
                             surface and its derivatives. It must be written
                             to account for the documented relative pose between
                             half space and mesh. The provided function should
                             make use of googletest EXPECT_* macros to perform
                             the assessment. */
  void TestPositionDerivative(
      const std::function<void(const ContactSurface<AutoDiffXd>&,
                               const RigidTransform<AutoDiffXd>&, TriPose)>&
          evaluate_quantity) {
    /* Definition of a test configuration: a name, a relative pose between soft
     and rigid frames, encoded as a double-valued (_d suffix) relative position
     between frame origins and relative orientation, the expected number of
     faces in the resultant contact surface, and the pose enumeration that the
     `evaluate_quantity` method will use to define the expected results. */
    struct Configuration {
      std::string name;
      Vector3d p_SR_d;
      RotationMatrixd R_SR_d;
      int expected_num_faces{};
      TriPose pose;
    };
    vector<Configuration> configurations;

    const RigidTransformd X_WS_d = convert_to_double(X_WS_);
    // We want to make sure that p_SoRo is completely non-zero. So, we pick
    // a point N on the boundary of the half space offset from its origin and
    // position the triangle with respect to that point.
    const Vector3d p_SN{0.25, -0.3, 0};
    {
      /* Leave the triangle unrotated, i.e. R_RS = I. The contact mesh will
       be a "horizontal" slice of the triangle: a right isosceles triangle. */
      const Vector3d n_S{0, 0, 1};
      const Vector3d p_SR_d{p_SN + (1 - kDepth) * n_S};
      configurations.push_back(
          {"Horizontal slice", p_SR_d, RotationMatrixd{}, 3, kHorizontalSlice});
    }

    {
      /* We'll pose the triangle in the soft half space's frame S. If we start
       with bases S and R aligned, we'll rotate the triangle around Sy so that
       edge 01 is no longer parallel with the Sz = 0 plane, and then rotate
       around Sx so that the edge 02 isn't parallel with Sz. Finally, we'll
       position vertex 2 displaced from N and inset into the half space kDepth
       distance.

       The rotation around Sx must be less than pi / 4 to ensure the
       intersection doesn't get culled. */
      const RotationMatrixd R_SR_d = RotationMatrixd::MakeXRotation(-M_PI / 7) *
                                     RotationMatrixd::MakeYRotation(-M_PI / 6);

      const Vector3d p_RoV2_S =
          R_SR_d * mesh_R_->vertex(SurfaceVertexIndex(2)).r_MV();
      const Vector3d p_SV2 = p_SN + Vector3d{0, 0, -kDepth};
      const Vector3d p_SR_d = p_SV2 - p_RoV2_S;
      configurations.push_back(
          {"Skewed triangle", p_SR_d, R_SR_d, 3, kSkewTriangle});
    }

    {
      /* We'll pose the triangle in the soft half space's frame S. If we start
       with bases S and R aligned, we'll rotate the triangle around Sy so that
       vertices 1 & 2 lie inside the half space. We'll rotate it between zero
       and 45 degrees for two reasons: keep V2 as the deepest vertex and prevent
       the edge connecting V1 and V2 from lying parallel with the plane.
       Finally, we'll tilt the triangle around the Sx axis to break alignment.
       */
      const RotationMatrixd R_SR_d = RotationMatrixd::MakeXRotation(-M_PI / 7) *
                                   RotationMatrixd::MakeYRotation(M_PI / 4.5);

      const Vector3d p_RoV2_S =
          R_SR_d * mesh_R_->vertex(SurfaceVertexIndex(2)).r_MV();
      const Vector3d p_SV2 = p_SN + Vector3d{0, 0, -kDepth};
      const Vector3d p_SR_d = p_SV2 - p_RoV2_S;
      configurations.push_back({"Skewed quad", p_SR_d, R_SR_d, 4, kSkewQuad});
    }

    for (const auto& config : configurations) {
      const RotationMatrixd R_WR_d = X_WS_d.rotation() * config.R_SR_d;
      const Vector3d p_WR_d = X_WS_d * config.p_SR_d;
      const Vector3<AutoDiffXd> p_WR = math::initializeAutoDiff(p_WR_d);
      const RigidTransform<AutoDiffXd> X_WR{R_WR_d.cast<AutoDiffXd>(), p_WR};

      auto surface = ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
          id_S_, X_WS_, pressure_scale_, id_R_, *mesh_R_, *bvh_R_, X_WR);

      SCOPED_TRACE(config.name);
      ASSERT_NE(surface, nullptr);
      ASSERT_EQ(surface->mesh_W().num_faces(), config.expected_num_faces);

      evaluate_quantity(*surface, X_WR, config.pose);
    }
  }

  /* Given the point E which purports to lie on an edge of the triangular mesh,
   finds the edge it lies on (spanning vertices A and B) and returns p_AB_R. */
  Vector3d GetEdgeDirInR(const Vector3d& p_RE) const {
    // We determine the edge that E lies on with this simple metric. If E lies
    // on edge AB, then AB⋅AE = |AB|⋅|AE|, or, to avoid square roots:
    // (AB⋅AE)² = |AB|²⋅|AE|²
    const vector<SurfaceVertex<double>>& verts_R = mesh_R_->vertices();
    const vector<pair<int, int>> edges{{0, 1}, {0, 2}, {1, 2}};
    for (const auto [a, b] : edges) {
      const Vector3d& p_AB_R = verts_R[b].r_MV() - verts_R[a].r_MV();
      const Vector3d p_AE_R = p_RE - verts_R[a].r_MV();
      const double lhs = std::pow(p_AB_R.dot(p_AE_R), 2);
      const double rhs = p_AB_R.squaredNorm() * p_AE_R.squaredNorm();
      if (std::abs(lhs - rhs) < 1e-15) {
        return p_AB_R;
      }
    }
    throw std::logic_error(
        "Querying for point I that isn't actually on a triangle edge");
  }

  /* The soft half space. */
  RigidTransform<AutoDiffXd> X_WS_;
  GeometryId id_S_;
  double pressure_scale_{};

  /* The rigid mesh. */
  GeometryId id_R_;
  unique_ptr<SurfaceMesh<double>> mesh_R_;
  unique_ptr<Bvh<SurfaceMesh<double>>> bvh_R_;

  /* The amount we penetrate the triangle mesh into the half space.  */
  static constexpr double kDepth = 0.25;
};

TEST_F(MeshHalfSpaceDerivativesTest, Area) {
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
      ∂Ro                ∂Ro

                   │                  ∂(B - A)                    ∂(C - A) │ᵀ
           = 0.5 * │ -skew_sym(C - A) ────────  + skew_sym(B - A) ──────── │ n̂
                   │                    ∂Ro                          ∂Ro   │

   We are *given* the quantities ∂A/∂Ro, ∂B/∂Ro, and ∂C/∂Ro which have been
   independently validated by the VertexPosition test. */
  auto evaluate_area = [X_WS = convert_to_double(this->X_WS_)](
                           const ContactSurface<AutoDiffXd>& surface,
                           const RigidTransform<AutoDiffXd>& X_WR_ad,
                           TriPose pose) {
    const auto& mesh_W = surface.mesh_W();

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
     of the behavior of the mesh-half space intersection algorithm. If it
     changes, these hard-coded indices could become invalid. A more robust
     solution would be to *infer* the boundary polygon edges from the contact
     surface, but that's a lot of effort for little present value. */
    using VIndex = SurfaceVertexIndex;
    vector<vector<VIndex>> triangles;
    switch (pose) {
      case TriPose::kHorizontalSlice:
      case TriPose::kSkewTriangle:
        triangles.emplace_back(vector<VIndex>{VIndex(0), VIndex(1), VIndex(2)});
        break;
      case TriPose::kSkewQuad:
        /* This is a bit brittle and is predicated on knowledge of how
         the intersection algorithm processes the particular geometry. If that
         proves to be too brittle, we'll need to reconstruct this by looking
         at the provided mesh. */
        triangles.emplace_back(vector<VIndex>{VIndex(2), VIndex(1), VIndex(0)});
        triangles.emplace_back(vector<VIndex>{VIndex(2), VIndex(3), VIndex(1)});
        break;
    }

    /* The normal for the contact surface is simply Ry (here, expressed in
     world). */
    const Vector3d n_W =
        convert_to_double(Vector3<AutoDiffXd>(X_WR_ad.rotation().col(1)));

    double area_expected = 0;
    Vector3d dArea_dRo_expected = Vector3d::Zero();
    for (const auto& tri : triangles) {
      const auto& p_WA_ad = mesh_W.vertex(tri[0]).r_MV();
      const auto& p_WB_ad = mesh_W.vertex(tri[1]).r_MV();
      const auto& p_WC_ad = mesh_W.vertex(tri[2]).r_MV();
      const Vector3d p_WA = convert_to_double(p_WA_ad);
      const Vector3d p_WB = convert_to_double(p_WB_ad);
      const Vector3d p_WC = convert_to_double(p_WC_ad);
      const Matrix3<double> dA_dRo = math::autoDiffToGradientMatrix(p_WA_ad);
      const Matrix3<double> dB_dRo = math::autoDiffToGradientMatrix(p_WB_ad);
      const Matrix3<double> dC_dRo = math::autoDiffToGradientMatrix(p_WC_ad);

      const Vector3d p_AB_W = p_WB - p_WA;
      const Vector3d p_AC_W = p_WC - p_WA;
      const double tri_area = 0.5 * p_AB_W.cross(p_AC_W).dot(n_W);
      area_expected += tri_area;

      const Matrix3<double> left = -skew_matrix(p_AC_W) * (dB_dRo - dA_dRo);
      const Matrix3<double> right = skew_matrix(p_AB_W) * (dC_dRo - dA_dRo);
      dArea_dRo_expected += 0.5 * ((left + right).transpose() * n_W);
    }

    constexpr double kEps = std::numeric_limits<double>::epsilon();
    const AutoDiffXd& total_area = mesh_W.total_area();
    EXPECT_NEAR(total_area.value(), area_expected, 4 * kEps);
    EXPECT_TRUE(CompareMatrices(total_area.derivatives(), dArea_dRo_expected,
                                4 * kEps));
  };

  TestPositionDerivative(evaluate_area);
}

TEST_F(MeshHalfSpaceDerivativesTest, VertexPosition) {
  /* In principle, we might expect that the derivatives of contact surface
   vertex position, v, with respect to Ro, ∂v/∂Ro, would simply be the identity.
   After all, the mesh vertices are affixed to the mesh. This is not entirely
   true. In reality, it depends on where the vertex in the contact surface mesh
   comes from:

   - Vertices from mesh_R_ inside the half space
     - These vertices *can* be considered to be rigidly affixed to the mesh,
      so ∂v/∂Ro = I is correct.

   - For vertices at the intersection of plane and tri edge, the derivative of
     the vertex position w.r.t. the position of the mesh origin (Ro) can be
     derived as follows:

     The point V lies on an edge of a triangle. We'll characterize the edge as
     a one of the two end points A, and the *direction* of the edge from A to
     the other vertex, ê. Therefore (with everything measured and expressed
     in the S frame),

         V = A + tê

     And we know that V.z = 0 because it lies on the half space surface. Hence,
     the scalar t (which is equal to the length |AV|) equals -A.z/ê.z and we can
     write V as a function of A:

             | A.x - ê.x * A.z / ê.z |
         V = | A.y - ê.y * A.z / ê.z |
             |            0          |

     So, since A is fixed to the triangle frame R (i.e., ∂A / ∂Ro = I), we see
     that:

                   | 1  0  -ê.x / ê.z |
          ∂V/∂Ro = | 0  1  -ê.y / ê.z |
                   | 0  0       0     |

                   | 1  0  -e.x / e.z |   // We don't actually require ê, e
          ∂V/∂Ro = | 0  1  -e.y / e.z |   // will suffice.
                   | 0  0       0     |

     However, the derivatives reported in the test are ∂p_WV/∂p_WRo, so we have
     to transform the expected result from the soft frame S to world.

          ∂p_WV    ∂(R_WS⋅p_SV + p_WSo)
         ------- = --------------------          // Expand p_WV = X_WS * p_SV.
          ∂p_WRo         ∂p_WRo

                   ∂(R_WS⋅p_SV)
                 = -------------                 // p_WSo doesn't depend on Ro.
                      ∂p_WRo

                   ∂(R_WS⋅p_SV)     ∂p_SRo
                 = ------------- * --------      // Chain rule.
                      ∂p_SRo        ∂p_WRo

                   ∂(R_WS⋅p_SV)                  // Change of position in S is
                 = ------------- * R_SW          // related to change in W by
                      ∂p_SRo                     // R_SW.

                           ∂p_SV
                 = R_WS * -------- * R_SW        // R_WS doesn't depend on Ro.
                           ∂p_SRo

                          | 1 0 -e.x / e.z |
                 = R_WS * | 0 1 -e.y / e.z | * R_SW  // Definition of ∂V/∂Ro.
                          | 0 0      0     |

   - Vertices at the centroid of intersection polygons. Every polygon formed by
     clipping a mesh triangle is tessellated around its centroid. For centroids
     of triangles (where the intersecting polygon between tri and half space is
     a triangle), the centroid position and its derivative are simply:

                 C = (v0 + v1 + v2) / 3, and
            ∂C/∂Ro = (∂v0/∂Ro + ∂v1/∂Ro + ∂v2/∂Ro) / 3

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
                                 const RigidTransform<AutoDiffXd>& X_WR_ad,
                                 TriPose pose) {
    constexpr double kEps = 5 * std::numeric_limits<double>::epsilon();
    using VIndex = SurfaceVertexIndex;

    /* The test is set up so there is only ever a single intersecting polygon.
     So, there is *one* centroid (the last vertex). All other vertices come
     from intersecting a tri edge with the plane or are mesh vertices embedded
     in the half space. We'll evaluate all of those and then handle the centroid
     specially. */
    const Vector3d n_S{0, 0, 1};
    const SurfaceMesh<AutoDiffXd>& mesh_W = surface.mesh_W();
    const RigidTransformd X_WS = convert_to_double(this->X_WS_);
    const RotationMatrixd& R_WS = X_WS.rotation();
    const RotationMatrixd R_SW = R_WS.inverse();
    const RigidTransformd X_WR = convert_to_double(X_WR_ad);
    const RotationMatrixd& R_WR = X_WR.rotation();

    for (VIndex v(0); v < mesh_W.num_vertices() - 1; ++v) {
      const Vector3<AutoDiffXd>& p_WV_ad = mesh_W.vertex(v).r_MV();
      const Vector3d p_WV = convert_to_double(p_WV_ad);
      const Vector3d p_SV = X_WS.inverse() * p_WV;
      if (p_SV.z() < -1e-10) {
        /* Because this point lies *below* the half space surface (and we know
         it is not the centroid), this must be one of mesh_R's vertices inside
         the half space. It is rigidly affixed to R so the derivatives are just
         the identity. */
        const Matrix3<double> J_W = math::autoDiffToGradientMatrix(p_WV_ad);
        ASSERT_TRUE(CompareMatrices(J_W, Matrix3<double>::Identity(), kEps));
        continue;
      }
      const Vector3d e_S = R_SW * R_WR * GetEdgeDirInR(X_WR.inverse() * p_WV);
      Matrix3<double> expected_J_S;
      // clang-format off
      expected_J_S << 1,  0, -e_S.x() / e_S.z(),
                      0,  1, -e_S.y() / e_S.z(),
                      0,  0,  0;
      // clang-format on
      const Matrix3<double> expected_J_W =
          R_WS * (expected_J_S * R_SW.matrix());
      const Matrix3<double> J_W = math::autoDiffToGradientMatrix(p_WV_ad);
      ASSERT_TRUE(CompareMatrices(J_W, expected_J_W, kEps));
    }

    /* Now handle the centroid. */
    switch (pose) {
      case kHorizontalSlice:
      case kSkewTriangle: {
        /* The derivative of the centroid should simply be the mean of the first
         three vertices' derivatives. */
        Matrix3<double> expected_J_W = Matrix3<double>::Zero();
        for (VIndex v(0); v < 3; ++v) {
          expected_J_W +=
              math::autoDiffToGradientMatrix(mesh_W.vertex(v).r_MV());
        }
        expected_J_W /= 3;
        const Vector3<AutoDiffXd>& p_WC = mesh_W.vertex(VIndex(3)).r_MV();
        const Matrix3<double> J_W = math::autoDiffToGradientMatrix(p_WC);
        EXPECT_TRUE(CompareMatrices(J_W, expected_J_W, kEps));
        break;
      }
      case kSkewQuad:
        /* We skip the centroid for the quad case. */
        break;
    }
  };

  TestPositionDerivative(evalute_position);
}

TEST_F(MeshHalfSpaceDerivativesTest, FaceNormalsWrtPosition) {
  /* None of the face normals depend on Ro. They should all report the zero
   matrix. */
  auto evaluate_normals = [](const ContactSurface<AutoDiffXd>& surface,
                             const RigidTransform<AutoDiffXd>&, TriPose) {
    constexpr double kEps = std::numeric_limits<double>::epsilon();
    const Matrix3<double> zero_matrix = Matrix3<double>::Zero();
    for (SurfaceFaceIndex t(0); t < surface.mesh_W().num_elements(); ++t) {
      const Vector3<AutoDiffXd> n_W = surface.mesh_W().face_normal(t);
      EXPECT_TRUE(CompareMatrices(math::autoDiffToGradientMatrix(n_W),
                                  zero_matrix, 16 * kEps));
    }
  };

  TestPositionDerivative(evaluate_normals);
}

TEST_F(MeshHalfSpaceDerivativesTest, FaceNormalsWrtOrientation) {
  /* The normals of the contact surface are always parallel with the normals on
   the rigid mesh. Therefore, as the mesh normals change, so should the contact
   surface normals.

   So, we'll define the rotation matrix R_WR as an angle of θ radians around an
   arbitrary unit vector v̂ and differentiate with respect to θ. The mesh normals
   n̂ point in the Ry direction. We can show that the expected derivative is
   dn̂/dθ = v̂ × n̂ = v̂ × Ry as follows:

       R_WR = exp(skew_sym(v̂)*θ)
        n̂_W = R_WR * Ry_R

    dn̂_W/dθ = d(R_WR * Ry_R)/dθ
            = dR_WR/dθ * Ry_R
            = skew_sym(v̂) * exp(skew_sym(v̂)*θ) * Ry_R
            = v̂ × n̂

   This test does *not* use the TestPositionDerivative() API because that
   differentiates with respect to Ro and makes assumptions about the resulting
   mesh. For this test, those assumptions would be invalid, so we'll simply
   duplicate that portion of TestPositionDerivative() that is relevant for this
   test. */

  /* Arbitrary rotation axis. It is important that the vector be defined such
   that rotations don't cause all intersecting triangles to be "backface
   culled".  */
  const Vector3d v_S = Vector3d{-1, 2, -3}.normalized();
  for (const double theta : {0.0, M_PI / 6, M_PI / 2 * 0.9, M_PI / 2 * 0.99}) {
    /* We'll push the origin down into the half space to guarantee
     intersection without doing otherwise unnecessary analysis). We're only
     testing normal direction; any intersection is sufficient. The x- and y-
     elements are arbitrary, non-zero values. */
    const Vector3<AutoDiffXd> p_WR =
        this->X_WS_ * Vector3<AutoDiffXd>{0.25, -0.7, 0.1};
    AutoDiffXd theta_ad = theta;
    theta_ad.derivatives().resize(1);
    theta_ad.derivatives() << 1;
    RotationMatrix<AutoDiffXd> R_SR(AngleAxis<AutoDiffXd>(theta_ad, v_S));
    RigidTransform<AutoDiffXd> X_WR{this->X_WS_.rotation() * R_SR, p_WR};
    const Vector3d v_W = convert_to_double(this->X_WS_).rotation() * v_S;

    auto surface = ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
        id_S_, X_WS_, pressure_scale_, id_R_, *mesh_R_, *bvh_R_, X_WR);

    SCOPED_TRACE(fmt::format("theta = {:.5f} radians", theta));
    /* surface != nullptr --> num_elements() > 0 as documented in the API but
     we'll explicitly confirm to make sure the test doesn't pass by accident. */
    ASSERT_NE(surface, nullptr);
    ASSERT_GT(surface->mesh_W().num_elements(), 0);

    /* Test dn̂/dθ = v̂ × n̂  = v̂ × Ry. */
    const Vector3d Ry_W = math::autoDiffToValueMatrix(X_WR.rotation().col(1));
    const Vector3d expected_deriv = v_W.cross(Ry_W);
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

          p = En̂ᵀ(o − v)

     ∂p/∂Ro = ∂[En̂ᵀ(o − v)]/∂Ro
            = En̂ᵀ∂[o − v]/∂Ro
            = En̂ᵀ[∂o/∂Ro − ∂v/∂Ro]
            = -En̂ᵀ∂v/∂Ro

   We've already tested the derivative of vertex position w.r.t. Ro, so we can
   make use of that to confirm that the pressure derivatives are consistent. */
  auto evaluate_pressures = [E = this->pressure_scale_,
                             X_WS = convert_to_double(this->X_WS_)](
                                const ContactSurface<AutoDiffXd>& surface,
                                const RigidTransform<AutoDiffXd>&,
                                double theta) {
    constexpr double kEps = std::numeric_limits<double>::epsilon();
    const Vector3d& n_W = X_WS.rotation().col(2);
    for (SurfaceVertexIndex v(0); v < surface.mesh_W().num_vertices(); ++v) {
      const auto& p_WV = surface.mesh_W().vertex(v).r_MV();
      const Matrix3<double> dV_dRo = math::autoDiffToGradientMatrix(p_WV);
      const Vector3d expected_dp_dRo = -E * n_W.transpose() * dV_dRo;
      const AutoDiffXd& p = surface.e_MN().EvaluateAtVertex(v);
      EXPECT_TRUE(
          CompareMatrices(p.derivatives(), expected_dp_dRo, 3 * E * kEps));
    }
  };

  TestPositionDerivative(evaluate_pressures);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
