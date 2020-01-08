#include "drake/geometry/proximity/mesh_half_space_intersection.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace {

using internal::Plane;

template <typename T>
class MeshHalfspaceIntersectionTest : public ::testing::Test {
 public:
  /// Returns the half space with normal expressed in Frame H.
  const Plane<T> half_space_H() const { return *half_space_H_; }

  // Accessors for data structures used repeatedly.
  std::vector<SurfaceVertex<T>>& new_vertices() { return new_vertices_; }
  std::vector<SurfaceFace>& new_faces() { return new_faces_; }
  std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>&
  vertices_to_newly_created_vertices() {
    return vertices_to_newly_created_vertices_;
  }
  std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>&
  edges_to_newly_created_vertices() {
    return edges_to_newly_created_vertices_;
  }

  /// Creates a SurfaceMesh with a triangle using the given vertices.
  SurfaceMesh<T> CreateSurfaceMesh(const Vector3<T>& v0, const Vector3<T>& v1,
                                   const Vector3<T>& v2) const {
    std::vector<SurfaceVertex<T>> vertices;
    vertices.emplace_back(v0);
    vertices.emplace_back(v1);
    vertices.emplace_back(v2);

    std::vector<SurfaceFace> faces = {SurfaceFace(
        SurfaceVertexIndex(0), SurfaceVertexIndex(1), SurfaceVertexIndex(2))};

    return SurfaceMesh<T>(std::move(faces), std::move(vertices));
  }

  // Checks whether two faces from two meshes are equivalent, which we define
  // to mean as some permutation of acceptable windings for the vertices of
  // `fb` yields vertices coincident with those of `fa`.
  bool AreFacesEquivalent(const SurfaceFace& fa, const SurfaceMesh<T>& mesh_a,
                          const SurfaceFace& fb, const SurfaceMesh<T>& mesh_b) {
    // Get the three vertices from each.
    std::array<Vector3<T>, 3> vertices_a = {mesh_a.vertex(fa.vertex(0)).r_MV(),
                                            mesh_a.vertex(fa.vertex(1)).r_MV(),
                                            mesh_a.vertex(fa.vertex(2)).r_MV()};
    std::array<Vector3<T>, 3> vertices_b = {mesh_b.vertex(fb.vertex(0)).r_MV(),
                                            mesh_b.vertex(fb.vertex(1)).r_MV(),
                                            mesh_b.vertex(fb.vertex(2)).r_MV()};

    // Set an array of faces that will be used to determine how the two
    // triangles align. Each of these faces encodes an acceptable triangle
    // winding.
    typedef SurfaceVertexIndex Index;
    std::array<SurfaceFace, 3> permutations = {
        SurfaceFace(Index(0), Index(1), Index(2)),
        SurfaceFace(Index(1), Index(2), Index(0)),
        SurfaceFace(Index(2), Index(0), Index(1))};

    // Verify that at least one of the permutations gives exactly the vertices
    // from fb. The unit tests permit this since all of the numbers used and
    // computed have exact floating point representations.
    using std::min;
    T closest_dist = std::numeric_limits<T>::max();
    for (const auto& face : permutations) {
      T dist = 0;
      for (int i = 0; i < 3; ++i)
        dist += (vertices_a[face.vertex(i)] - vertices_b[i]).norm();
      closest_dist = min(closest_dist, dist);
      if (closest_dist == 0.0) break;
    }

    return (closest_dist == 0.0);
  }

  // Checks whether two meshes are equivalent, meaning that there is a bijective
  // mapping from every face in mesh a to every face in mesh b. A mapping
  // between faces can only exist if the face in a is considered equivalent to
  // the corresponding face in b -- see AreFacesEquivalent().
  void VerifyMeshesEquivalent(const SurfaceMesh<T>& mesh1,
                              const SurfaceMesh<T>& mesh2) {
    // Simple checks first.
    ASSERT_EQ(mesh1.num_faces(), mesh2.num_faces());
    ASSERT_EQ(mesh1.num_vertices(), mesh2.num_vertices());

    // Iterate through each face of the first mesh, looking for a face from the
    // second mesh that is within the given tolerance. This is a quadratic
    // time algorithm (in the number of faces of the meshes) but we expect the
    // number of faces that this algorithm is run on to be small.
    std::vector<SurfaceFace> faces_from_mesh2 = mesh2.faces();
    for (const SurfaceFace& f1 : mesh1.faces()) {
      bool found_match = false;
      for (int i = 0; i < static_cast<int>(faces_from_mesh2.size()); ++i) {
        if (AreFacesEquivalent(f1, mesh1, faces_from_mesh2[i], mesh2)) {
          found_match = true;
          faces_from_mesh2[i] = faces_from_mesh2.back();
          faces_from_mesh2.pop_back();
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
    new_vertices().clear();
    new_faces().clear();
    vertices_to_newly_created_vertices().clear();
    edges_to_newly_created_vertices().clear();
  }

  // Calls the triangle-half space intersection routine using this object's
  // half space as well as the given mesh and construction data structures.
  void ConstructTriangleHalfspaceIntersectionPolygon(
      const SurfaceMesh<T>& mesh,
      std::vector<SurfaceVertex<T>>* new_vertices_F_in,
      std::vector<SurfaceFace>* new_faces_in,
      std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>*
          vertices_to_newly_created_vertices_in,
      std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>*
          edges_to_newly_created_vertices_in) {
    for (const SurfaceFace& face : mesh.faces()) {
      internal::ConstructTriangleHalfspaceIntersectionPolygon(
          mesh.vertices(), face, this->half_space_H(),
          new_vertices_F_in, new_faces_in,
          vertices_to_newly_created_vertices_in,
          edges_to_newly_created_vertices_in);
    }
  }

  // Convenience function for verifying that the original mesh was output.
  void VerifyOriginalMeshOutput(
      const SurfaceMesh<T>& mesh,
      const std::vector<SurfaceVertex<T>>& new_vertices_in,
      const std::vector<SurfaceFace>& new_faces_in) {
    ASSERT_EQ(mesh.num_faces(), new_faces_in.size());
    ASSERT_EQ(new_vertices_in.size(), mesh.num_vertices());
    for (SurfaceVertexIndex i(0); i < new_vertices_in.size(); ++i)
      EXPECT_EQ(new_vertices_in[i].r_MV(), mesh.vertex(i).r_MV());
    for (SurfaceFaceIndex j(0); j < mesh.num_faces(); ++j) {
      const SurfaceFace& original_face = mesh.element(j);
      const SurfaceFace& new_face = new_faces_in[j];
      for (int i = 0; i < 3; ++i)
        EXPECT_EQ(new_face.vertex(i), original_face.vertex(i));
    }
  }

 private:
  void SetUp() {
    // The tests all use half space normal [0 0 1], with point [0 0 2]
    // lying on the half space.
    Vector3<T> normal_H(0, 0, 1);
    const Vector3<T> point_H(0, 0, 2);
    const T displacement = normal_H.dot(point_H);
    half_space_H_ = std::make_unique<Plane<T>>(normal_H, displacement);
  }

  std::vector<SurfaceVertex<T>> new_vertices_;
  std::vector<SurfaceFace> new_faces_;
  std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>
      vertices_to_newly_created_vertices_;
  std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>
      edges_to_newly_created_vertices_;
  std::unique_ptr<Plane<T>> half_space_H_;
};  // namespace
TYPED_TEST_SUITE_P(MeshHalfspaceIntersectionTest);

// Verifies that a triangle that lies fully outside of the half space yields an
// empty intersection. This covers Case 4 in the code.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, NoIntersection) {
  using T = TypeParam;

  // Create the mesh, constructing the vertices of the triangle to lie outside
  // the half space.
  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
      Vector3<T>(0, 0, 3), Vector3<T>(1, 0, 3), Vector3<T>(0, 1, 3));

  // Verify no intersection.
  this->ConstructTriangleHalfspaceIntersectionPolygon(
      mesh, &this->new_vertices(), &this->new_faces(),
      &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  EXPECT_TRUE(this->new_vertices().empty());
  EXPECT_TRUE(this->new_faces().empty());
  EXPECT_TRUE(this->vertices_to_newly_created_vertices().empty());
  EXPECT_TRUE(this->edges_to_newly_created_vertices().empty());
}

// Verifies that a triangle that lies inside or on the half space yields
// that same triangle. This covers Case 1 in the code.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, InsideOrOnIntersection) {
  using T = TypeParam;

  // Create the mesh, constructing the vertices of the triangle to lie well
  // inside the half space.
  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
      Vector3<T>(3, 5, 1), Vector3<T>(4, 5, 1), Vector3<T>(3, 6, 1));

  // Verify the intersection.
  this->ConstructTriangleHalfspaceIntersectionPolygon(
      mesh, &this->new_vertices(), &this->new_faces(),
      &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  this->VerifyOriginalMeshOutput(mesh,
                                 this->new_vertices(), this->new_faces());

  // Construct the vertices of the triangle to lie exactly on the half space.
  // Note that this test works because the half space is defined using a
  // "standard basis" normal vector and simple constant as well a mesh defined
  // using integral vertex values. If any of these conditions were unmet, this
  // test might fail.
  const SurfaceMesh<T> second_mesh = this->CreateSurfaceMesh(
      Vector3<T>(3, 5, 2), Vector3<T>(4, 5, 2), Vector3<T>(3, 6, 2));

  // Verify the intersection.
  this->ClearConstructionDataStructures();
  this->ConstructTriangleHalfspaceIntersectionPolygon(
      second_mesh, &this->new_vertices(), &this->new_faces(),
      &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  this->VerifyOriginalMeshOutput(second_mesh,
                                 this->new_vertices(), this->new_faces());

  // Construct two triangles using a shared edge to verify no extraneous
  // vertices are constructed.
  std::vector<SurfaceVertex<T>> vertices = {
      SurfaceVertex<T>(Vector3<T>(4, 5, 2)),
      SurfaceVertex<T>(Vector3<T>(3, 5, 2)),
      SurfaceVertex<T>(Vector3<T>(3, 5, 1)),
      SurfaceVertex<T>(Vector3<T>(2, 5, 2))};
  typedef SurfaceVertexIndex Index;
  std::vector<SurfaceFace> faces = {SurfaceFace(Index(0), Index(1), Index(2)),
                                    SurfaceFace(Index(2), Index(1), Index(3))};
  const SurfaceMesh<T> third_mesh(std::move(faces), std::move(vertices));

  // Verify the intersection.
  this->ClearConstructionDataStructures();
  this->ConstructTriangleHalfspaceIntersectionPolygon(
      third_mesh, &this->new_vertices(), &this->new_faces(),
      &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  this->VerifyOriginalMeshOutput(third_mesh,
                                 this->new_vertices(), this->new_faces());
}

// Verifies that a triangle that has exactly one vertex lying on the half space
// produces either (a) a degenerate intersection (if the other two vertices lie
// outside the half space) or (b) the original triangle (if the other two
// vertices lie within the half space). This covers Case 1.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, VertexOnHalfspaceIntersection) {
  using T = TypeParam;

  // Construct two vertices of the triangle to lie outside the half space and
  // the other to lie on the half space.
  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
      Vector3<T>(3, 5, 2), Vector3<T>(4, 5, 3), Vector3<T>(3, 6, 3));

  // Verify the degenerate intersection.
  this->ConstructTriangleHalfspaceIntersectionPolygon(
      mesh, &this->new_vertices(), &this->new_faces(),
      &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  EXPECT_EQ(this->new_vertices().size(), 3);
  EXPECT_EQ(this->new_faces().size(), 1);
  EXPECT_EQ(this->vertices_to_newly_created_vertices().size(), 1);
  EXPECT_EQ(this->edges_to_newly_created_vertices().size(), 2);

  // Check that the mesh that results is equivalent to the expected degenerate
  // mesh.
  std::vector<SurfaceVertex<T>> expected_vertices = {
      SurfaceVertex<T>(Vector3<T>(3, 5, 2)),
      SurfaceVertex<T>(Vector3<T>(3, 5, 2)),
      SurfaceVertex<T>(Vector3<T>(3, 5, 2))};
  typedef SurfaceVertexIndex Index;
  std::vector<SurfaceFace> expected_faces = {
      SurfaceFace(Index(0), Index(1), Index(2))};
  const SurfaceMesh<T> expected_mesh(std::move(expected_faces),
                                     std::move(expected_vertices));
  const SurfaceMesh<T> actual_mesh(std::move(this->new_faces()),
                                   std::move(this->new_vertices()));
  this->VerifyMeshesEquivalent(expected_mesh, actual_mesh);

  // Construct two vertices of the triangle to lie inside the half space and the
  // other to lie on the half space.
  const SurfaceMesh<T> second_mesh = this->CreateSurfaceMesh(
      Vector3<T>(3, 5, 2), Vector3<T>(4, 5, 1), Vector3<T>(3, 6, 1));

  // Verify the intersection.
  this->ClearConstructionDataStructures();
  this->ConstructTriangleHalfspaceIntersectionPolygon(
      second_mesh, &this->new_vertices(), &this->new_faces(),
      &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  this->VerifyOriginalMeshOutput(second_mesh,
                                 this->new_vertices(), this->new_faces());
}

// Verifies that a triangle that has exactly two vertices lying on the half
// space produces either (a) a degenerate intersection (if the remaining vertex
// lies outside the half space) or (b) the original triangle (if the other
// vertex lies within the half space). This covers Case 1.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, EdgeOnHalfspaceIntersection) {
  using T = TypeParam;

  // Construct one vertex of the triangle to lie outside the half space and the
  // other two to lie on the half space.
  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
      Vector3<T>(3, 5, 3), Vector3<T>(2, 5, 2), Vector3<T>(4, 5, 2));

  // Verify that there is no intersection.
  this->ConstructTriangleHalfspaceIntersectionPolygon(
      mesh, &this->new_vertices(), &this->new_faces(),
      &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  EXPECT_EQ(this->new_vertices().size(), 4);
  EXPECT_EQ(this->new_faces().size(), 2);
  EXPECT_EQ(this->vertices_to_newly_created_vertices().size(), 2);
  EXPECT_EQ(this->edges_to_newly_created_vertices().size(), 2);

  //                   a
  //                   ╱╲
  // ^ z              ╱  ╲
  // |         ______╱____╲_____
  // |            b/e      c/d
  // |
  // |
  // --------> x

  // Check that the mesh that results is equivalent to the expected degenerate
  // mesh.
  std::vector<SurfaceVertex<T>> expected_vertices = {
      SurfaceVertex<T>(Vector3<T>(2, 5, 2)),
      SurfaceVertex<T>(Vector3<T>(4, 5, 2)),
      SurfaceVertex<T>(Vector3<T>(4, 5, 2)),
      SurfaceVertex<T>(Vector3<T>(2, 5, 2))};
  typedef SurfaceVertexIndex Index;
  const Index b(0), c(1), d(2), e(3);
  std::vector<SurfaceFace> expected_faces = {SurfaceFace(b, c, e),
                                             SurfaceFace(c, d, e)};
  const SurfaceMesh<T> expected_mesh(std::move(expected_faces),
                                     std::move(expected_vertices));
  const SurfaceMesh<T> actual_mesh(std::move(this->new_faces()),
                                   std::move(this->new_vertices()));
  this->VerifyMeshesEquivalent(expected_mesh, actual_mesh);

  // Construct one vertex of the triangle to lie inside the half space and the
  // other two to lie on the half space.
  const SurfaceMesh<T> second_mesh = this->CreateSurfaceMesh(
      Vector3<T>(3, 5, 1), Vector3<T>(4, 5, 2), Vector3<T>(3, 6, 2));

  // Verify the intersection.
  this->ClearConstructionDataStructures();
  this->ConstructTriangleHalfspaceIntersectionPolygon(
      second_mesh, &this->new_vertices(), &this->new_faces(),
      &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  this->VerifyOriginalMeshOutput(second_mesh,
                                 this->new_vertices(), this->new_faces());
}

// Verifies that a triangle that has two vertices within the half space and
// another outside the half space produces a quadrilateral. This covers Case 2.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, QuadrilateralResults) {
  using T = TypeParam;

  // Construct one vertex of the triangle to lie outside the half space and the
  // other two to lie inside the half space.
  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
      Vector3<T>(3, 5, 3), Vector3<T>(2, 5, 1), Vector3<T>(4, 5, 1));

  //                   a
  //                   ╱╲
  // ^ z              ╱  ╲
  // |         _____e╱____╲d___
  // |              ╱      ╲
  // |             ╱________╲
  // |            b          c
  // --------> x

  // Verify the intersection.
  this->ConstructTriangleHalfspaceIntersectionPolygon(
      mesh, &this->new_vertices(), &this->new_faces(),
      &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  ASSERT_EQ(this->new_faces().size(), 2);
  ASSERT_EQ(this->new_vertices().size(), 4);
  ASSERT_EQ(this->edges_to_newly_created_vertices().size(), 2);

  // Check that the mesh that results is equivalent to the expected mesh.
  std::vector<SurfaceVertex<T>> expected_vertices = {
      SurfaceVertex<T>(Vector3<T>(2, 5, 1)),    // b
      SurfaceVertex<T>(Vector3<T>(4, 5, 1)),     // c
      SurfaceVertex<T>(Vector3<T>(3.5, 5, 2)),    // d
      SurfaceVertex<T>(Vector3<T>(2.5, 5, 2))};  // e
  typedef SurfaceVertexIndex Index;
  const Index b(0), c(1), d(2), e(3);
  std::vector<SurfaceFace> expected_faces = {SurfaceFace(b, c, e),
                                             SurfaceFace(c, d, e)};
  const SurfaceMesh<T> expected_mesh(std::move(expected_faces),
                                     std::move(expected_vertices));
  const SurfaceMesh<T> actual_mesh(std::move(this->new_faces()),
                                   std::move(this->new_vertices()));
  this->VerifyMeshesEquivalent(expected_mesh, actual_mesh);
}

// Verifies that a triangle that has one vertex outside the half space, one
// vertex inside the half space, and one vertex on the half space produces two
// triangles, one degenerate and one non-degenerate. This test covers Case 2.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, OutsideInsideOn) {
  using T = TypeParam;

  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
      Vector3<T>(3, 5, 3), Vector3<T>(2, 5, 1), Vector3<T>(3.5, 5, 2));

  // Verify the intersection.
  this->ConstructTriangleHalfspaceIntersectionPolygon(
      mesh, &this->new_vertices(), &this->new_faces(),
      &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  ASSERT_EQ(this->edges_to_newly_created_vertices().size(), 2);

  //                   a
  //                   ╱╲
  // ^ z              ╱  ╲
  // |         _____e╱____╲c/d___
  // |              ╱
  // |             ╱
  // |            b
  // --------> x

  // Check that the mesh that results is equivalent to the expected mesh.
  std::vector<SurfaceVertex<T>> expected_vertices = {
      SurfaceVertex<T>(Vector3<T>(2, 5, 1)),    // b
      SurfaceVertex<T>(Vector3<T>(3.5, 5, 2)),    // c
      SurfaceVertex<T>(Vector3<T>(3.5, 5, 2)),    // d
      SurfaceVertex<T>(Vector3<T>(2.5, 5, 2))};  // e
  typedef SurfaceVertexIndex Index;
  const Index b(0), c(1), d(2), e(3);
  std::vector<SurfaceFace> expected_faces = {SurfaceFace(b, c, e),
                                             SurfaceFace(c, d, e)};
  const SurfaceMesh<T> expected_mesh(std::move(expected_faces),
                                     std::move(expected_vertices));
  const SurfaceMesh<T> actual_mesh(std::move(this->new_faces()),
                                   std::move(this->new_vertices()));
  this->VerifyMeshesEquivalent(expected_mesh, actual_mesh);
}

// Verifies that a triangle with one vertex inside the half space and two
// vertices outside of the half space produces a triangle. This test covers
// Case 3.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, OneInsideTwoOutside) {
  using T = TypeParam;

  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
      Vector3<T>(3, 5, 3), Vector3<T>(4, 5, 3), Vector3<T>(3, 6, 1));

  // Verify the intersection.
  this->ConstructTriangleHalfspaceIntersectionPolygon(
      mesh, &this->new_vertices(), &this->new_faces(),
      &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());

  // Check that the mesh that results is equivalent to the expected mesh.
  std::vector<SurfaceVertex<T>> expected_vertices = {
      SurfaceVertex<T>(Vector3<T>(3, 6, 1)),
      SurfaceVertex<T>(Vector3<T>(3, 5.5, 2)),
      SurfaceVertex<T>(Vector3<T>(3.5, 5.5, 2))};
  typedef SurfaceVertexIndex Index;
  std::vector<SurfaceFace> expected_faces = {
      SurfaceFace(Index(0), Index(1), Index(2))};
  const SurfaceMesh<T> expected_mesh(std::move(expected_faces),
                                     std::move(expected_vertices));
  const SurfaceMesh<T> actual_mesh(std::move(this->new_faces()),
                                   std::move(this->new_vertices()));
  this->VerifyMeshesEquivalent(expected_mesh, actual_mesh);
}

// Tests that a mesh of a box bisected by the half space produces the expected
// number of faces and vertices (other unit tests in this file assess the
// correctness of various kinds of triangle/half space intersections).
TYPED_TEST_P(MeshHalfspaceIntersectionTest, BoxMesh) {
  using T = TypeParam;

  // Set the box vertices according to the following diagram. The box is
  // centered at the origin of Frame F. Introducing this frame makes it less
  // likely that all of the underlying tests might succeed because of some
  // fortuitous initial values. The halfspace normal points along F's +z axis.
  // Set the box vertices according to the following diagram. The box is
  // centered at the origin.
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

  // Set F to an arbitrarily chosen pose relative to the world frame.
  math::RigidTransform<T> X_WF(math::RollPitchYaw<T>(M_PI_4, M_PI_4, M_PI_4),
                               Vector3<T>(1.0, 2.0, 3.0));

  std::vector<SurfaceVertex<T>> vertices_W;
  vertices_W.emplace_back(X_WF * Vector3<T>(-1, -1, -1));
  vertices_W.emplace_back(X_WF * Vector3<T>(-1, -1, 1));
  vertices_W.emplace_back(X_WF * Vector3<T>(-1, 1, -1));
  vertices_W.emplace_back(X_WF * Vector3<T>(-1, 1, 1));
  vertices_W.emplace_back(X_WF * Vector3<T>(1, -1, -1));
  vertices_W.emplace_back(X_WF * Vector3<T>(1, -1, 1));
  vertices_W.emplace_back(X_WF * Vector3<T>(1, 1, -1));
  vertices_W.emplace_back(X_WF * Vector3<T>(1, 1, 1));

  // Set the twelve box faces using a counter-clockwise winding.
  typedef SurfaceVertexIndex Index;
  std::vector<SurfaceFace> faces;
  faces.emplace_back(Index(4), Index(6), Index(7));  // +X face
  faces.emplace_back(Index(7), Index(5), Index(4));  // +X face
  faces.emplace_back(Index(1), Index(3), Index(2));  // -X face
  faces.emplace_back(Index(2), Index(0), Index(1));  // -X face
  faces.emplace_back(Index(2), Index(3), Index(7));  // +Y face
  faces.emplace_back(Index(7), Index(6), Index(2));  // +Y face
  faces.emplace_back(Index(4), Index(5), Index(1));  // -Y face
  faces.emplace_back(Index(1), Index(0), Index(4));  // -Y face
  faces.emplace_back(Index(7), Index(3), Index(1));  // +Z face
  faces.emplace_back(Index(1), Index(5), Index(7));  // +Z face
  faces.emplace_back(Index(4), Index(0), Index(2));  // -Z face
  faces.emplace_back(Index(2), Index(6), Index(4));  // -Z face

  // Construct the mesh.
  const SurfaceMesh<T> mesh(std::move(faces), std::move(vertices_W));

  // Construct the half-space.
  const Vector3<T> half_space_normal_W = X_WF.rotation() * Vector3<T>(0, 0, 1);
  const T half_space_displacement = half_space_normal_W.dot(X_WF.translation());
  const Plane<T> half_space(half_space_normal_W, half_space_displacement);

  // Compute the intersection.
  const SurfaceMesh<T> intersection_mesh =
      ConstructSurfaceMeshFromMeshHalfspaceIntersection(
          mesh, half_space);
  ASSERT_EQ(intersection_mesh.num_vertices(), 8);

  // Note: The sides of the box facing the +/- X and +/- Y directions are each
  // split into three triangles. The side of the box facing -Z is fully enclosed
  // (two more triangles) yielding 4 x 3 + 2 = 14 triangles. (The side facing +Z
  // is outside the half space.)
  ASSERT_EQ(intersection_mesh.num_faces(), 14);
}

REGISTER_TYPED_TEST_SUITE_P(MeshHalfspaceIntersectionTest, NoIntersection,
                           InsideOrOnIntersection,
                           VertexOnHalfspaceIntersection,
                           EdgeOnHalfspaceIntersection, QuadrilateralResults,
                           OutsideInsideOn, OneInsideTwoOutside, BoxMesh);

typedef ::testing::Types<double, AutoDiffXd> MyTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, MeshHalfspaceIntersectionTest, MyTypes);

}  // namespace
}  // namespace geometry
}  // namespace drake
