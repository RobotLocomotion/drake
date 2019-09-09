#include "drake/geometry/proximity/mesh_half_space_intersection.h"

#include <unordered_set>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace {

template <typename T>
class MeshHalfspaceIntersectionTest : public ::testing::Test {
 public:
  /// Returns the normal to the half space.
  const Vector3<T>& normal_H() const { return normal_H_; }

  /// Returns the constant d to the half space such that normal_Hᵀx = d,
  /// where x is an arbitrary point on the surface of the half space.
  const T& half_space_constant() const { return d_; }

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
    // from fb. The tests permit this since
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

  // Checks whether two meshes are equivalent, meaning that each face in `mesh1`
  // is equivalent (see definition in `AreFacesEquivalent()`) to *at least one*
  // face in `mesh2`.
  void VerifyMeshesEquivalent(const SurfaceMesh<T>& mesh1,
                              const SurfaceMesh<T>& mesh2) {
    // Simple checks first.
    ASSERT_EQ(mesh1.num_faces(), mesh2.num_faces());
    ASSERT_EQ(mesh1.num_vertices(), mesh2.num_vertices());

    // Iterate through each face of the first mesh, looking for a face from the
    // second mesh that is within the given tolerance. This is a quadratic
    // time algorithm (in the number of faces of the meshes) but we expect the
    // number of faces that this algorithm is run on to be small.
    std::unordered_set<const SurfaceFace*> used_faces_from_mesh2;
    for (const SurfaceFace& f1 : mesh1.faces()) {
      bool found_match = false;
      for (const SurfaceFace& f2 : mesh2.faces()) {
        if (used_faces_from_mesh2.find(&f2) == used_faces_from_mesh2.end()) {
          if (AreFacesEquivalent(f1, mesh1, f2, mesh2)) {
            found_match = true;
            used_faces_from_mesh2.insert(&f2);
            break;
          }
        }
      }

      EXPECT_TRUE(found_match);
    }
  }

  // Convenience function for verifying that the original mesh was output.
  // Note that this method goes beyond checking whether every pair of two
  // triangles is equivalent but for some permutation of the vertex ordering;
  // the method expects exactly the same vertices in exactly the same order.
  void VerifyOriginalMeshOutput(
      const SurfaceMesh<T>& mesh,
      const std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>&
          vertices_to_newly_created_vertices,
      const std::unordered_map<SortedPair<SurfaceVertexIndex>,
                               SurfaceVertexIndex>&
          edges_to_newly_created_vertices,
      const std::vector<SurfaceVertex<T>>& new_vertices,
      const std::vector<SurfaceFace>& new_faces) {
    ASSERT_EQ(mesh.num_faces(), new_faces.size());
    EXPECT_TRUE(edges_to_newly_created_vertices.empty());
    ASSERT_EQ(new_vertices.size(), mesh.num_vertices());
    for (int i = 0; i < static_cast<int>(new_vertices.size()); ++i) {
      EXPECT_EQ(new_vertices[i].r_MV(),
                mesh.vertex(SurfaceVertexIndex(i)).r_MV());
    }
    ASSERT_EQ(vertices_to_newly_created_vertices.size(), mesh.num_vertices());
    for (int j = 0; j < mesh.num_faces(); ++j) {
      const SurfaceFace& original_face = mesh.element(SurfaceFaceIndex(j));
      const SurfaceFace& new_face = new_faces[j];
      for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(new_face.vertex(i), original_face.vertex(i));
        EXPECT_EQ(
            vertices_to_newly_created_vertices.at(original_face.vertex(i)),
            new_face.vertex(i));
      }
    }
  }

 private:
  void SetUp() {
    // The tests all use half space normal [0 0 1], with point [0 0 0]
    // lying on the half space.
    normal_H_ = Vector3<T>(0, 0, 1);
    const Vector3<T> point_H(0, 0, 0);
    d_ = normal_H_.dot(point_H);
  }

  std::vector<SurfaceVertex<T>> new_vertices_;
  std::vector<SurfaceFace> new_faces_;
  std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>
      vertices_to_newly_created_vertices_;
  std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>
      edges_to_newly_created_vertices_;
  Vector3<T> normal_H_;
  T d_;
};  // namespace
TYPED_TEST_CASE_P(MeshHalfspaceIntersectionTest);

// Verifies that a triangle that lies fully outside of the half space yields an
// empty intersection. This covers Case 4 in the code.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, NoIntersection) {
  using T = TypeParam;

  // Create the mesh, constructing the vertices of the triangle to lie outside
  // the half space.
  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
      Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 1), Vector3<T>(0, 1, 1));

  // Verify no intersection.
  internal::ConstructTriangleHalfspaceIntersectionPolygon(
      mesh.vertices(), mesh.element(SurfaceFaceIndex(0)), this->normal_H(),
      this->half_space_constant(), &this->new_vertices(), &this->new_faces(),
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
      Vector3<T>(0, 0, -1), Vector3<T>(1, 0, -1), Vector3<T>(0, 1, -1));

  // Verify the intersection.
  internal::ConstructTriangleHalfspaceIntersectionPolygon(
      mesh.vertices(), mesh.element(SurfaceFaceIndex(0)), this->normal_H(),
      this->half_space_constant(), &this->new_vertices(), &this->new_faces(),
      &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  this->VerifyOriginalMeshOutput(mesh,
                                 this->vertices_to_newly_created_vertices(),
                                 this->edges_to_newly_created_vertices(),
                                 this->new_vertices(), this->new_faces());

  // Construct the vertices of the triangle to lie exactly on the half space.
  // Note that this test works because the half space is defined using a
  // "standard basis" normal vector and simple constant as well a mesh defined
  // using integral vertex values. If any of these conditions were unmet, this
  // test might fail.
  const SurfaceMesh<T> second_mesh = this->CreateSurfaceMesh(
      Vector3<T>(0, 0, 0), Vector3<T>(1, 0, 0), Vector3<T>(0, 1, 0));

  // Verify the intersection.
  this->new_vertices().clear();
  this->new_faces().clear();
  this->vertices_to_newly_created_vertices().clear();
  this->edges_to_newly_created_vertices().clear();
  internal::ConstructTriangleHalfspaceIntersectionPolygon(
      second_mesh.vertices(), second_mesh.element(SurfaceFaceIndex(0)),
      this->normal_H(), this->half_space_constant(), &this->new_vertices(),
      &this->new_faces(), &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  this->VerifyOriginalMeshOutput(second_mesh,
                                 this->vertices_to_newly_created_vertices(),
                                 this->edges_to_newly_created_vertices(),
                                 this->new_vertices(), this->new_faces());

  // Construct two triangles using a shared edge to verify no extraneous
  // vertices are constructed.
  std::vector<SurfaceVertex<T>> vertices = {
      SurfaceVertex<T>(Vector3<T>(1, 0, 0)),
      SurfaceVertex<T>(Vector3<T>(0, 0, 0)),
      SurfaceVertex<T>(Vector3<T>(0, 0, -1)),
      SurfaceVertex<T>(Vector3<T>(-1, 0, 0))};
  typedef SurfaceVertexIndex Index;
  std::vector<SurfaceFace> faces = {SurfaceFace(Index(0), Index(1), Index(2)),
                                    SurfaceFace(Index(2), Index(1), Index(3))};
  const SurfaceMesh<T> third_mesh(std::move(faces), std::move(vertices));

  // Verify the intersection.
  this->new_vertices().clear();
  this->new_faces().clear();
  this->vertices_to_newly_created_vertices().clear();
  this->edges_to_newly_created_vertices().clear();
  internal::ConstructTriangleHalfspaceIntersectionPolygon(
      third_mesh.vertices(), third_mesh.element(SurfaceFaceIndex(0)),
      this->normal_H(), this->half_space_constant(), &this->new_vertices(),
      &this->new_faces(), &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  internal::ConstructTriangleHalfspaceIntersectionPolygon(
      third_mesh.vertices(), third_mesh.element(SurfaceFaceIndex(1)),
      this->normal_H(), this->half_space_constant(), &this->new_vertices(),
      &this->new_faces(), &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  this->VerifyOriginalMeshOutput(third_mesh,
                                 this->vertices_to_newly_created_vertices(),
                                 this->edges_to_newly_created_vertices(),
                                 this->new_vertices(), this->new_faces());
}

// Verifies that a triangle that has exactly one vertex lying on the half space
// produces either (a) a degenerate intersection (if the other two vertices lie
// outside the half space) or (b) the original triangle (if the other two
// vertices lie within the half space). This covers Cases 3 and 1,
// respectively.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, VertexOnHalfspaceIntersection) {
  using T = TypeParam;

  // Construct two vertices of the triangle to lie outside the half space and
  // the other to lie on the half space.
  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
      Vector3<T>(0, 0, 0), Vector3<T>(1, 0, 1), Vector3<T>(0, 1, 1));

  // Verify the degenerate intersection.
  internal::ConstructTriangleHalfspaceIntersectionPolygon(
      mesh.vertices(), mesh.element(SurfaceFaceIndex(0)), this->normal_H(),
      this->half_space_constant(), &this->new_vertices(), &this->new_faces(),
      &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  EXPECT_EQ(this->new_vertices().size(), 3);
  EXPECT_EQ(this->new_faces().size(), 1);
  EXPECT_EQ(this->vertices_to_newly_created_vertices().size(), 1);
  EXPECT_EQ(this->edges_to_newly_created_vertices().size(), 2);

  // Check that the mesh that results is equivalent to the expected degenerate
  // mesh.
  std::vector<SurfaceVertex<T>> expected_vertices = {
      SurfaceVertex<T>(Vector3<T>(0, 0, 0)),
      SurfaceVertex<T>(Vector3<T>(0, 0, 0)),
      SurfaceVertex<T>(Vector3<T>(0, 0, 0))};
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
      Vector3<T>(0, 0, 0), Vector3<T>(1, 0, -1), Vector3<T>(0, 1, -1));

  // Verify the intersection.
  this->new_vertices().clear();
  this->new_faces().clear();
  this->vertices_to_newly_created_vertices().clear();
  this->edges_to_newly_created_vertices().clear();
  internal::ConstructTriangleHalfspaceIntersectionPolygon(
      second_mesh.vertices(), second_mesh.element(SurfaceFaceIndex(0)),
      this->normal_H(), this->half_space_constant(), &this->new_vertices(),
      &this->new_faces(), &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  this->VerifyOriginalMeshOutput(second_mesh,
                                 this->vertices_to_newly_created_vertices(),
                                 this->edges_to_newly_created_vertices(),
                                 this->new_vertices(), this->new_faces());
}

// Verifies that a triangle that has exactly two vertices lying on the half
// space produces either (a) a degenerate intersection (if the remaining vertex
// lies outside the half space) or (b) the original triangle (if the other
// vertex lies within the half space). This covers Cases 2 and 1, respectively.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, EdgeOnHalfspaceIntersection) {
  using T = TypeParam;

  // Construct one vertex of the triangle to lie outside the half space and the
  // other two to lie on the half space.
  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
      Vector3<T>(0, 0, 1), Vector3<T>(-1, 0, 0), Vector3<T>(1, 0, 0));

  // Verify that there is no intersection.
  internal::ConstructTriangleHalfspaceIntersectionPolygon(
      mesh.vertices(), mesh.element(SurfaceFaceIndex(0)), this->normal_H(),
      this->half_space_constant(), &this->new_vertices(), &this->new_faces(),
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
      SurfaceVertex<T>(Vector3<T>(-1, 0, 0)),
      SurfaceVertex<T>(Vector3<T>(1, 0, 0)),
      SurfaceVertex<T>(Vector3<T>(1, 0, 0)),
      SurfaceVertex<T>(Vector3<T>(-1, 0, 0))};
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
      Vector3<T>(0, 0, -1), Vector3<T>(1, 0, 0), Vector3<T>(0, 1, 0));

  // Verify the intersection.
  this->new_vertices().clear();
  this->new_faces().clear();
  this->vertices_to_newly_created_vertices().clear();
  this->edges_to_newly_created_vertices().clear();
  internal::ConstructTriangleHalfspaceIntersectionPolygon(
      second_mesh.vertices(), second_mesh.element(SurfaceFaceIndex(0)),
      this->normal_H(), this->half_space_constant(), &this->new_vertices(),
      &this->new_faces(), &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  this->VerifyOriginalMeshOutput(second_mesh,
                                 this->vertices_to_newly_created_vertices(),
                                 this->edges_to_newly_created_vertices(),
                                 this->new_vertices(), this->new_faces());
}

// Verifies that a triangle that has two vertices within the half space and
// another outside the half space produces a quadrilateral. This covers Case 2.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, QuadrilateralResults) {
  using T = TypeParam;

  // Construct one vertex of the triangle to lie outside the half space and the
  // other two to lie inside the half space.
  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
      Vector3<T>(0, 0, 1), Vector3<T>(-1, 0, -1), Vector3<T>(1, 0, -1));

  //                   a
  //                   ╱╲
  // ^ z              ╱  ╲
  // |         _____e╱____╲d___
  // |              ╱      ╲
  // |             ╱________╲
  // |            b          c
  // --------> x

  // Verify the intersection.
  internal::ConstructTriangleHalfspaceIntersectionPolygon(
      mesh.vertices(), mesh.element(SurfaceFaceIndex(0)), this->normal_H(),
      this->half_space_constant(), &this->new_vertices(), &this->new_faces(),
      &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());
  ASSERT_EQ(this->new_faces().size(), 2);
  ASSERT_EQ(this->new_vertices().size(), 4);
  ASSERT_EQ(this->edges_to_newly_created_vertices().size(), 2);

  // Check that the mesh that results is equivalent to the expected mesh.
  std::vector<SurfaceVertex<T>> expected_vertices = {
      SurfaceVertex<T>(Vector3<T>(-1, 0, -1)),    // b
      SurfaceVertex<T>(Vector3<T>(1, 0, -1)),     // c
      SurfaceVertex<T>(Vector3<T>(0.5, 0, 0)),    // d
      SurfaceVertex<T>(Vector3<T>(-0.5, 0, 0))};  // e
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
      Vector3<T>(0, 0, 1), Vector3<T>(-1, 0, -1), Vector3<T>(0.5, 0, 0));

  // Verify the intersection.
  internal::ConstructTriangleHalfspaceIntersectionPolygon(
      mesh.vertices(), mesh.element(SurfaceFaceIndex(0)), this->normal_H(),
      this->half_space_constant(), &this->new_vertices(), &this->new_faces(),
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
      SurfaceVertex<T>(Vector3<T>(-1, 0, -1)),    // b
      SurfaceVertex<T>(Vector3<T>(0.5, 0, 0)),    // c
      SurfaceVertex<T>(Vector3<T>(0.5, 0, 0)),    // d
      SurfaceVertex<T>(Vector3<T>(-0.5, 0, 0))};  // e
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
      Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 1), Vector3<T>(0, 1, -1));

  // Verify the intersection.
  internal::ConstructTriangleHalfspaceIntersectionPolygon(
      mesh.vertices(), mesh.element(SurfaceFaceIndex(0)), this->normal_H(),
      this->half_space_constant(), &this->new_vertices(), &this->new_faces(),
      &this->vertices_to_newly_created_vertices(),
      &this->edges_to_newly_created_vertices());

  // Check that the mesh that results is equivalent to the expected mesh.
  std::vector<SurfaceVertex<T>> expected_vertices = {
      SurfaceVertex<T>(Vector3<T>(0, 1, -1)),
      SurfaceVertex<T>(Vector3<T>(0, .5, 0)),
      SurfaceVertex<T>(Vector3<T>(.5, .5, 0))};
  typedef SurfaceVertexIndex Index;
  std::vector<SurfaceFace> expected_faces = {
      SurfaceFace(Index(0), Index(1), Index(2))};
  const SurfaceMesh<T> expected_mesh(std::move(expected_faces),
                                     std::move(expected_vertices));
  const SurfaceMesh<T> actual_mesh(std::move(this->new_faces()),
                                   std::move(this->new_vertices()));
  this->VerifyMeshesEquivalent(expected_mesh, actual_mesh);
}

REGISTER_TYPED_TEST_CASE_P(MeshHalfspaceIntersectionTest, NoIntersection,
                           InsideOrOnIntersection,
                           VertexOnHalfspaceIntersection,
                           EdgeOnHalfspaceIntersection, QuadrilateralResults,
                           OutsideInsideOn, OneInsideTwoOutside);

typedef ::testing::Types<double, AutoDiffXd> MyTypes;
INSTANTIATE_TYPED_TEST_CASE_P(My, MeshHalfspaceIntersectionTest, MyTypes);

}  // namespace
}  // namespace geometry
}  // namespace drake
