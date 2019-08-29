#include "drake/geometry/proximity/mesh_halfspace_intersection.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace {

typedef double T;

template <typename T>
class MeshHalfspaceIntersectionTest : public ::testing::Test {
 public:
  void SetUp() {
    // The tests all use halfspace normal [0 0 1], with point [0 0 0]
    // lying on the halfspace.
    normal_H_ = Vector3<T>(0, 0, 1);
    const Vector3<T> point_H(0, 0, 0);
    d_ = normal_H_.dot(point_H);
  }

  /// Returns the normal to the halfspace.
  const Vector3<T>& normal_H() const { return normal_H_; }

  /// Returns the constant d to the halfspace such that normal_H' * x = d,
  /// where x is an arbitrary point on the surface of the halfspace.
  const T& halfspace_constant() const { return d_; }

  /// Creates the SurfaceMesh using the given vertices.
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

  // Checks whether two faces from two meshes are equivalent.
  bool AreFacesClose(const SurfaceFace& fa, const SurfaceMesh<T>& mesh_a,
                     const SurfaceFace& fb, const SurfaceMesh<T>& mesh_b,
                     double tol) {
    // Get the three vertices from each.
    std::array<Vector3<T>, 3> vertices_a = {
      mesh_a.vertex(fa.vertex(0)).r_MV(), mesh_a.vertex(fa.vertex(1)).r_MV(),
      mesh_a.vertex(fa.vertex(2)).r_MV() };
    std::array<Vector3<T>, 3> vertices_b = {
      mesh_b.vertex(fb.vertex(0)).r_MV(), mesh_b.vertex(fb.vertex(1)).r_MV(),
      mesh_b.vertex(fb.vertex(2)).r_MV() };

    // Set an array of indices that will be used to determine how the two
    // triangles align.
    std::array<int, 3> indices = {0, 1, 2};

    // Verify that at least one of the permutations is close to within the
    // requested tolerancce.
    double closest_dist = std::numeric_limits<double>::max();
    do {
      double dist = 0;
      for (int i = 0; i < 3; ++i)
        dist += (vertices_a[indices[i]] - vertices_b[i]).norm();
      closest_dist = std::min(closest_dist, dist);
      if (closest_dist < tol)
        break;
    } while (std::next_permutation(indices.begin(), indices.end()));
    if (closest_dist > tol)
      return false;

    // Finally, ensure that the normals are pointing the same direction.
    const Vector3<T> na =
        (vertices_a[1] - vertices_a[0]).cross(vertices_a[2] - vertices_a[1]);
    const Vector3<T> nb =
        (vertices_b[1] - vertices_b[0]).cross(vertices_b[2] - vertices_b[1]);
    return (na.dot(nb) > 0);
  }

  // Checks whether two meshes are equivalent, regardless of how
  // vertices and faces are ordered.
  void VerifyMeshesEquivalent(
      const SurfaceMesh<T>& mesh1, const SurfaceMesh<T>& mesh2, double tol) {
    // Simple checks first.
    ASSERT_EQ(mesh1.num_faces(), mesh2.num_faces());
    ASSERT_EQ(mesh1.num_vertices(), mesh2.num_vertices());

    // Iterate through each face of the first mesh, looking for a face from the
    // second mesh that is within the given tolerance. This is a quadratic
    // time algorithm (in the number of faces of the meshes) but we expect the
    // number of faces that this algorithm is run on to be small.
    for (const SurfaceFace& f1 : mesh1.faces()) {
      bool found_match = false;
      for (const SurfaceFace& f2 : mesh2.faces()) {
        if (AreFacesClose(f1, mesh1, f2, mesh2, tol)) {
          found_match = true;
          break;
        }
      }

      EXPECT_TRUE(found_match);
    }
  }

  // Convenience function for verifying that the original triangle was output.
  void VerifyOriginalTriangleOutput(
      const SurfaceMesh<T>& mesh,
      const std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>&
          vertices_to_newly_created_vertices,
      const std::unordered_map<SortedPair<SurfaceVertexIndex>,
                               SurfaceVertexIndex>&
          edges_to_newly_created_vertices,
      const std::vector<SurfaceVertex<T>>& new_vertices,
      const std::vector<SurfaceFace>& new_faces) {
    EXPECT_TRUE(edges_to_newly_created_vertices.empty());
    ASSERT_EQ(new_vertices.size(), 3);
    for (int i = 0; i < static_cast<int>(new_vertices.size()); ++i) {
      EXPECT_EQ(new_vertices[i].r_MV(),
                mesh.vertex(SurfaceVertexIndex(i)).r_MV());
    }
    ASSERT_EQ(new_faces.size(), 1);
    ASSERT_EQ(vertices_to_newly_created_vertices.size(), 3);
    for (int i = 0; i < 3; ++i) {
      EXPECT_EQ(new_faces[0].vertex(i), SurfaceVertexIndex(i));
      EXPECT_EQ(vertices_to_newly_created_vertices.at(SurfaceVertexIndex(i)),
                SurfaceVertexIndex(i));
    }
  }



 private:
  Vector3<T> normal_H_;
  T d_;
};
TYPED_TEST_CASE_P(MeshHalfspaceIntersectionTest);

// Verifies that a triangle that lies fully outside of the halfspace yields an
// empty intersection. This covers Case (10) in the code.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, NoIntersection) {
  // Create the mesh, constructing the vertices of the triangle to lie outside
  // the halfspace.
  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
      Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 1), Vector3<T>(0, 1, 1));

  // Verify no intersection.
  std::vector<SurfaceVertex<T>> new_vertices;
  std::vector<SurfaceFace> new_faces;
  std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>
      vertices_to_newly_created_vertices;
  std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>
      edges_to_newly_created_vertices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      mesh.vertices(), mesh.element(SurfaceFaceIndex(0)), this->normal_H(),
      this->halfspace_constant(), &new_vertices, &new_faces,
      &vertices_to_newly_created_vertices, &edges_to_newly_created_vertices);
  EXPECT_TRUE(new_vertices.empty());
  EXPECT_TRUE(new_faces.empty());
  EXPECT_TRUE(vertices_to_newly_created_vertices.empty());
  EXPECT_TRUE(edges_to_newly_created_vertices.empty());
}

// Verifies that a triangle that lies inside or on the halfspace yields
// that same triangle. This covers Cases (1) and (4) in the code.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, InsideOrOnIntersection) {
  // Create the mesh, constructing the vertices of the triangle to lie outside
  // the halfspace.
  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
    Vector3<T>(0, 0, -1), Vector3<T>(1, 0, -1), Vector3<T>(0, 1, -1));

  // Verify the intersection.
  std::vector<SurfaceVertex<T>> new_vertices;
  std::vector<SurfaceFace> new_faces;
  std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>
      vertices_to_newly_created_vertices;
  std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>
      edges_to_newly_created_vertices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      mesh.vertices(), mesh.element(SurfaceFaceIndex(0)), this->normal_H(),
      this->halfspace_constant(), &new_vertices, &new_faces,
      &vertices_to_newly_created_vertices, &edges_to_newly_created_vertices);
  this->VerifyOriginalTriangleOutput(mesh, vertices_to_newly_created_vertices,
                               edges_to_newly_created_vertices, new_vertices,
                               new_faces);

  // Construct the vertices of the triangle to lie on the halfspace, using a
  // distance very close to zero.
  const T near_zero = std::numeric_limits<double>::epsilon();
  const SurfaceMesh<T> second_mesh = this->CreateSurfaceMesh(
    Vector3<T>(0, 0, near_zero), Vector3<T>(1, 0, near_zero),
    Vector3<T>(0, 1, near_zero));

  // Verify the intersection, loosening the tolerance only slightly so the
  // test will pass.
  new_vertices.clear();
  new_faces.clear();
  vertices_to_newly_created_vertices.clear();
  edges_to_newly_created_vertices.clear();
  ConstructTriangleHalfspaceIntersectionPolygon(
      second_mesh.vertices(), second_mesh.element(SurfaceFaceIndex(0)),
      this->normal_H(), this->halfspace_constant(), &new_vertices, &new_faces,
      &vertices_to_newly_created_vertices, &edges_to_newly_created_vertices,
      near_zero * 10);
  EXPECT_TRUE(edges_to_newly_created_vertices.empty());
  ASSERT_EQ(new_vertices.size(), 3);
  ASSERT_EQ(new_faces.size(), 1);
  ASSERT_EQ(vertices_to_newly_created_vertices.size(), 3);
}

// Verifies that a triangle that has exactly one vertex lying on the halfspace
// produces either (a) no intersection (if the other two vertices lie outside the
// halfspace) or (b) the original triangle (if the other two vertices lie
// within the halfspace). This covers Cases (9) and (2), respectively.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, VertexOnHalfspaceIntersection) {
  // Construct two vertices of the triangle to lie outside the halfspace and the
  // other to lie on the halfspace.
  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
    Vector3<T>(0, 0, 0), Vector3<T>(1, 0, 1), Vector3<T>(0, 1, 1));

  // Verify no intersection.
  std::vector<SurfaceVertex<T>> new_vertices;
  std::vector<SurfaceFace> new_faces;
  std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>
      vertices_to_newly_created_vertices;
  std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>
      edges_to_newly_created_vertices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      mesh.vertices(), mesh.element(SurfaceFaceIndex(0)), this->normal_H(),
      this->halfspace_constant(), &new_vertices, &new_faces,
      &vertices_to_newly_created_vertices, &edges_to_newly_created_vertices);
  EXPECT_TRUE(new_vertices.empty());
  EXPECT_TRUE(new_faces.empty());
  EXPECT_TRUE(vertices_to_newly_created_vertices.empty());
  EXPECT_TRUE(edges_to_newly_created_vertices.empty());

  // Construct two vertices of the triangle to lie inside the halfspace and the
  // other to lie on the halfspace.
  const SurfaceMesh<T> second_mesh = this->CreateSurfaceMesh(
    Vector3<T>(0, 0, 0), Vector3<T>(1, 0, -1), Vector3<T>(0, 1, -1));

  // Verify the intersection.
  new_vertices.clear();
  new_faces.clear();
  vertices_to_newly_created_vertices.clear();
  edges_to_newly_created_vertices.clear();
  ConstructTriangleHalfspaceIntersectionPolygon(
      second_mesh.vertices(), second_mesh.element(SurfaceFaceIndex(0)),
      this->normal_H(), this->halfspace_constant(), &new_vertices, &new_faces,
      &vertices_to_newly_created_vertices, &edges_to_newly_created_vertices);
  this->VerifyOriginalTriangleOutput(
      second_mesh, vertices_to_newly_created_vertices,
      edges_to_newly_created_vertices, new_vertices, new_faces);
}

// Verifies that a triangle that has exactly two vertices lying on the halfspace
// produces either (a) no intersection (if the other vertex lies outside the
// halfspace) or (b) the original triangle (if the other vertex lies
// within the halfspace). This covers Cases (7) and (3), respectively.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, EdgeOnHalfspaceIntersection) {
  // Construct one vertex of the triangle to lie outside the halfspace and the
  // other two to lie on the halfspace.
 const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
     Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 0), Vector3<T>(0, 1, 0));

  // Verify that there is no intersection.
  std::vector<SurfaceVertex<T>> new_vertices;
  std::vector<SurfaceFace> new_faces;
  std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>
      vertices_to_newly_created_vertices;
  std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>
      edges_to_newly_created_vertices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      mesh.vertices(), mesh.element(SurfaceFaceIndex(0)), this->normal_H(),
      this->halfspace_constant(), &new_vertices, &new_faces,
      &vertices_to_newly_created_vertices, &edges_to_newly_created_vertices);
  EXPECT_TRUE(new_vertices.empty());
  EXPECT_TRUE(new_faces.empty());
  EXPECT_TRUE(vertices_to_newly_created_vertices.empty());
  EXPECT_TRUE(edges_to_newly_created_vertices.empty());

  // Construct one vertex of the triangle to lie inside the halfspace and the
  // other two to lie on the halfspace.
  const SurfaceMesh<T> second_mesh = this->CreateSurfaceMesh(
    Vector3<T>(0, 0, -1), Vector3<T>(1, 0, 0), Vector3<T>(0, 1, 0));

  // Verify the intersection.
  new_vertices.clear();
  new_faces.clear();
  vertices_to_newly_created_vertices.clear();
  edges_to_newly_created_vertices.clear();
  ConstructTriangleHalfspaceIntersectionPolygon(
      second_mesh.vertices(), second_mesh.element(SurfaceFaceIndex(0)),
      this->normal_H(), this->halfspace_constant(), &new_vertices, &new_faces,
      &vertices_to_newly_created_vertices, &edges_to_newly_created_vertices);
  this->VerifyOriginalTriangleOutput(
      second_mesh, vertices_to_newly_created_vertices,
      edges_to_newly_created_vertices, new_vertices, new_faces);
}

// Verifies that a triangle that has two vertices within the halfspace and
// another outside the halfspace produces a quadrilateral. This covers Case 5.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, QuadrilateralResults) {
  // Construct one vertex of the triangle to lie outside the halfspace and the
  // other two to lie inside the halfspace.
  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, -1), Vector3<T>(0, 1, -1));

  // Verify the intersection. Note that the following test is overly specific
  // to the logic of the mesh-halfspace intersection routine: it is not actually
  // required that the vertices be output in the order inside.
  std::vector<SurfaceVertex<T>> new_vertices;
  std::vector<SurfaceFace> new_faces;
  std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>
      vertices_to_newly_created_vertices;
  std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>
      edges_to_newly_created_vertices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      mesh.vertices(), mesh.element(SurfaceFaceIndex(0)), this->normal_H(),
      this->halfspace_constant(), &new_vertices, &new_faces,
      &vertices_to_newly_created_vertices, &edges_to_newly_created_vertices);
  ASSERT_EQ(new_faces.size(), 2);
  ASSERT_EQ(new_vertices.size(), 4);
  ASSERT_EQ(edges_to_newly_created_vertices.size(), 2);

  // Check that the mesh that results is equivalent to the expected mesh.
  std::vector<SurfaceVertex<T>> expected_vertices = {
      SurfaceVertex<T>(Vector3<T>(1, 0, -1)),
      SurfaceVertex<T>(Vector3<T>(0, 1, -1)),
      SurfaceVertex<T>(Vector3<T>(0.5, 0, 0)),
      SurfaceVertex<T>(Vector3<T>(0, 0.5, 0)) };
  typedef SurfaceVertexIndex Index;
  std::vector<SurfaceFace> expected_faces = {
      SurfaceFace(Index(0), Index(1), Index(2)),
      SurfaceFace(Index(2), Index(3), Index(0)) };
  const SurfaceMesh<T> expected_mesh(std::move(expected_faces),
                                     std::move(expected_vertices));
  const SurfaceMesh<T> actual_mesh(std::move(new_faces),
                                   std::move(new_vertices));
  this->VerifyMeshesEquivalent(
      expected_mesh, actual_mesh, 10 * std::numeric_limits<double>::epsilon());
}

// Verifies that a triangle that has one vertex outside the halfspace, one
// vertex inside the halfspace, and one vertex on the halfspace produces a
// triangle. This test covers Case 6.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, OutsideInsideOn) {
  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 0), Vector3<T>(0, 1, -1));

  // Verify the intersection. Note that the following test is overly specific
  // to the logic of the mesh-halfspace intersection routine: it is not actually
  // required that the vertices be output in the order inside.
  std::vector<SurfaceVertex<T>> new_vertices;
  std::vector<SurfaceFace> new_faces;
  std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>
      vertices_to_newly_created_vertices;
  std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>
      edges_to_newly_created_vertices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      mesh.vertices(), mesh.element(SurfaceFaceIndex(0)), this->normal_H(),
      this->halfspace_constant(), &new_vertices, &new_faces,
      &vertices_to_newly_created_vertices, &edges_to_newly_created_vertices);
  ASSERT_EQ(edges_to_newly_created_vertices.size(), 1);

  // Check that the mesh that results is equivalent to the expected mesh.
  std::vector<SurfaceVertex<T>> expected_vertices = {
      SurfaceVertex<T>(Vector3<T>(1, 0, 0)),
      SurfaceVertex<T>(Vector3<T>(0, 1, -1)),
      SurfaceVertex<T>(Vector3<T>(0, .5, 0)) };
  typedef SurfaceVertexIndex Index;
  std::vector<SurfaceFace> expected_faces = {
      SurfaceFace(Index(0), Index(1), Index(2)) };
  const SurfaceMesh<T> expected_mesh(std::move(expected_faces),
                                     std::move(expected_vertices));
  const SurfaceMesh<T> actual_mesh(std::move(new_faces),
                                   std::move(new_vertices));
  this->VerifyMeshesEquivalent(
      expected_mesh, actual_mesh, 10 * std::numeric_limits<double>::epsilon());
}

// Verifies that a triangle with one vertex inside the halfspace and two
// vertices outside of the halfspace produces a triangle. This test covers
// Case 8.
TYPED_TEST_P(MeshHalfspaceIntersectionTest, OneInsideTwoOutside) {
  const SurfaceMesh<T> mesh = this->CreateSurfaceMesh(
    Vector3<T>(0, 0, 1), Vector3<T>(1, 0, 1), Vector3<T>(0, 1, -1));

  // Verify the intersection. Note that the following test is overly specific
  // to the logic of the mesh-halfspace intersection routine: it is not actually
  // required that the vertices be output in the order inside.
  std::vector<SurfaceVertex<T>> new_vertices;
  std::vector<SurfaceFace> new_faces;
  std::unordered_map<SurfaceVertexIndex, SurfaceVertexIndex>
      vertices_to_newly_created_vertices;
  std::unordered_map<SortedPair<SurfaceVertexIndex>, SurfaceVertexIndex>
      edges_to_newly_created_vertices;
  ConstructTriangleHalfspaceIntersectionPolygon(
      mesh.vertices(), mesh.element(SurfaceFaceIndex(0)), this->normal_H(),
      this->halfspace_constant(), &new_vertices, &new_faces,
      &vertices_to_newly_created_vertices, &edges_to_newly_created_vertices);

  // Check that the mesh that results is equivalent to the expected mesh.
  std::vector<SurfaceVertex<T>> expected_vertices = {
      SurfaceVertex<T>(Vector3<T>(0, 1, -1)),
      SurfaceVertex<T>(Vector3<T>(0, .5, 0)),
      SurfaceVertex<T>(Vector3<T>(.5, .5, 0)) };
  typedef SurfaceVertexIndex Index;
  std::vector<SurfaceFace> expected_faces = {
      SurfaceFace(Index(0), Index(1), Index(2)) };
  const SurfaceMesh<T> expected_mesh(std::move(expected_faces),
                                     std::move(expected_vertices));
  const SurfaceMesh<T> actual_mesh(std::move(new_faces),
                                   std::move(new_vertices));
  this->VerifyMeshesEquivalent(
      expected_mesh, actual_mesh, 10 * std::numeric_limits<double>::epsilon());
}

REGISTER_TYPED_TEST_CASE_P(MeshHalfspaceIntersectionTest, NoIntersection,
                           InsideOrOnIntersection,
                           VertexOnHalfspaceIntersection,
                           EdgeOnHalfspaceIntersection, QuadrilateralResults,
                           OutsideInsideOn, OneInsideTwoOutside);

typedef ::testing::Types<double> MyTypes;
INSTANTIATE_TYPED_TEST_CASE_P(My, MeshHalfspaceIntersectionTest, MyTypes);

}  // namespace
}  // namespace geometry
}  // namespace drake
