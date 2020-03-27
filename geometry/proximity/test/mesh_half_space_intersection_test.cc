#include "drake/geometry/proximity/mesh_half_space_intersection.h"

#include <limits>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
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

  /// Creates a SurfaceMesh with a single triangle using the given vertices.
  SurfaceMesh<double> CreateOneTriangleMesh(const Vector3d& v0,
                                            const Vector3d& v1,
                                            const Vector3d& v2) const {
    std::vector<SurfaceVertex<double>> vertices;
    vertices.emplace_back(v0);
    vertices.emplace_back(v1);
    vertices.emplace_back(v2);

    std::vector<SurfaceFace> faces = {SurfaceFace(
        SurfaceVertexIndex(0), SurfaceVertexIndex(1), SurfaceVertexIndex(2))};

    return SurfaceMesh<double>(move(faces), move(vertices));
  }

  /// Creates a new SurfaceMesh mesh from the given input mesh, such that the
  /// resulting mesh has the same domain, but with two differences:
  ///
  ///   1. The vertices in the resultant mesh are measured and expressed in
  ///      Frame W (as opposed to the input mesh's Frame F)
  ///   2. Each triangle in the input mesh has been turned into three triangles;
  ///      the original turned into a triangle fan around its centroid.
  /// This method confirms that the face normal directions are preserved.
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
  const SurfaceMesh<double> mesh_F = this->CreateOneTriangleMesh(
      Vector3d(0, 0, 3), Vector3d(1, 0, 3), Vector3d(0, 1, 3));

  // Verify no intersection.
  // X_WF = I -- because there is no intersection, the frame of the non-existant
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
    const SurfaceMesh<double> mesh_F = this->CreateOneTriangleMesh(
        Vector3d(3, 5, 1), Vector3d(4, 5, 1), Vector3d(3, 6, 1));

    this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, X_WF);

    // Verify the intersection.
    SCOPED_TRACE("Single triangle inside half space");
    this->VerifyMeshesEquivalent(
        this->CreateMeshWithCentroids(mesh_F, X_WF),
        SurfaceMesh<T>{move(this->new_faces()), move(this->new_vertices_W())});
  }

  {
    // Case: triangle lies on the boundary plane.
    const SurfaceMesh<double> mesh_F = this->CreateOneTriangleMesh(
        Vector3d(3, 5, 2), Vector3d(4, 5, 2), Vector3d(3, 6, 2));

    // Verify the intersection.
    this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, X_WF);

    SCOPED_TRACE("Single triangle on half space boundary");
    this->VerifyMeshesEquivalent(
        this->CreateMeshWithCentroids(mesh_F, X_WF),
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
        this->CreateMeshWithCentroids(mesh_F, X_WF),
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
    const SurfaceMesh<double> mesh_F = this->CreateOneTriangleMesh(
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
    const SurfaceMesh<double> mesh_F = this->CreateOneTriangleMesh(
        Vector3d(3, 5, 2), Vector3d(4, 5, 1), Vector3d(3, 6, 1));

    // Verify the intersection.
    this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, X_WF);

    SCOPED_TRACE("Triangle inside half space has single vertex on the plane");
    this->VerifyMeshesEquivalent(
        this->CreateMeshWithCentroids(mesh_F, X_WF),
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
    const SurfaceMesh<double> mesh_F = this->CreateOneTriangleMesh(
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
    const SurfaceMesh<double> mesh_F = this->CreateOneTriangleMesh(
        Vector3d(3, 5, 1), Vector3d(4, 5, 2), Vector3d(3, 6, 2));

    // Verify the intersection.
    this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, X_WF);

    SCOPED_TRACE("Triangle inside half space has single edge on the plane");
    this->VerifyMeshesEquivalent(
        this->CreateMeshWithCentroids(mesh_F, X_WF),
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
  const SurfaceMesh<double> mesh_F = this->CreateOneTriangleMesh(
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

  const SurfaceMesh<double> mesh_F = this->CreateOneTriangleMesh(
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

  const SurfaceMesh<double> mesh_F = this->CreateOneTriangleMesh(
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


  // Set the box vertices according to the following diagram. The box is
  // centered at the origin of Frame B. We transform it to some query frame F,
  // making it less  likely that all of the underlying functionality succeeds
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

  // Set B to an arbitrarily chosen pose relative to F.
  RigidTransform<T> X_FB(math::RollPitchYaw<T>(M_PI_4, M_PI_4, M_PI_4),
                               Vector3<T>(1.0, 2.0, 3.0));
  RigidTransform<double> X_FB_d = convert_to_double(X_FB);

  std::vector<SurfaceVertex<double>> vertices_F;
  vertices_F.emplace_back(X_FB_d * Vector3d(-1, -1, -1));
  vertices_F.emplace_back(X_FB_d * Vector3d(-1, -1, 1));
  vertices_F.emplace_back(X_FB_d * Vector3d(-1, 1, -1));
  vertices_F.emplace_back(X_FB_d * Vector3d(-1, 1, 1));
  vertices_F.emplace_back(X_FB_d * Vector3d(1, -1, -1));
  vertices_F.emplace_back(X_FB_d * Vector3d(1, -1, 1));
  vertices_F.emplace_back(X_FB_d * Vector3d(1, 1, -1));
  vertices_F.emplace_back(X_FB_d * Vector3d(1, 1, 1));

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
  const SurfaceMesh<double> mesh_F(move(faces), move(vertices_F));

  // Construct the half-space.
  const Vector3<T> Bz_F = X_FB.rotation().col(2);
  const PosedHalfSpace<T> half_space_F(Bz_F, X_FB.translation());

  // Compute the intersection.
  typedef SurfaceFaceIndex FIndex;
  std::vector<FIndex> tri_indices(mesh_F.num_elements());
  std::iota(tri_indices.begin(), tri_indices.end(), FIndex{0});

  const SurfaceMesh<T> intersection_mesh_W =
      ConstructSurfaceMeshFromMeshHalfspaceIntersection(
          mesh_F, half_space_F, tri_indices, X_WF);
  // Total number of vertices:            22
  //   -------------------------------------
  //   vertices lying in half space:       4
  //   vertices from split edges:          8
  //   centroid per intersection polygon: 10
  ASSERT_EQ(intersection_mesh_W.num_vertices(), 22);

  // Each of the +/- X, Y faces of the box have two triangles. They get clipped
  // into a triangle and quad. Each of those is transformed into a triangle
  // fan around its centroid, producing 3 and 4 triangles for a total of seven
  // per direction.
  // Total number of faces:                         34
  //   -----------------------------------------------
  //   -Z triangles, each fanned around centroid:    6
  //   4 X new triangles per box face:      7 * 4 = 28
  ASSERT_EQ(intersection_mesh_W.num_faces(), 34);
}

REGISTER_TYPED_TEST_SUITE_P(MeshHalfspaceIntersectionTest, NoIntersection,
                           InsideOrOnIntersection,
                           VertexOnHalfspaceIntersection,
                           EdgeOnHalfspaceIntersection, QuadrilateralResults,
                           OutsideInsideOn, OneInsideTwoOutside, BoxMesh);

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
