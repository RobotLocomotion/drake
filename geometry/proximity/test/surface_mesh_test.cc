#include "drake/geometry/proximity/surface_mesh.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace {

// Used for testing instantiation of SurfaceMesh and inspecting its components.
template <typename T>
std::unique_ptr<SurfaceMesh<T>> GenerateTwoTriangleMesh() {
  // The surface mesh will consist of four vertices and two faces and will
  // be constructed such that area and geometric centroid are straightforward
  // to check.

  // Create the vertices.
  std::vector<SurfaceVertex<T>> vertices;
  vertices.emplace_back(Vector3<T>(0.5, 0.5, -0.5));
  vertices.emplace_back(Vector3<T>(-0.5, 0.5, -0.5));
  vertices.emplace_back(Vector3<T>(-0.5, -0.5, -0.5));
  vertices.emplace_back(Vector3<T>(1.0, -1.0, -0.5));

  // Create the two triangles. Note that SurfaceMesh does not specify (or use) a
  // particular winding.
  std::vector<SurfaceFace> faces;
  faces.emplace_back(
      SurfaceVertexIndex(0), SurfaceVertexIndex(1), SurfaceVertexIndex(2));
  faces.emplace_back(
      SurfaceVertexIndex(2), SurfaceVertexIndex(3), SurfaceVertexIndex(0));

  return std::make_unique<SurfaceMesh<T>>(
      std::move(faces), std::move(vertices));
}

// Generates an empty mesh.
std::unique_ptr<SurfaceMesh<double>> GenerateEmptyMesh() {
  std::vector<SurfaceVertex<double>> vertices;
  std::vector<SurfaceFace> faces;
  return std::make_unique<SurfaceMesh<double>>(
      std::move(faces), std::move(vertices));
}

// Generates a zero-area mesh.
std::unique_ptr<SurfaceMesh<double>> GenerateZeroAreaMesh() {
  // The surface mesh will consist of four vertices and two faces.

  // Create the vertices.
  std::vector<SurfaceVertex<double>> vertices;
  for (int i = 0; i < 4; ++i)
    vertices.emplace_back(Vector3<double>::Zero());

  // Create the two triangles.
  std::vector<SurfaceFace> faces;
  faces.emplace_back(
      SurfaceVertexIndex(0), SurfaceVertexIndex(1), SurfaceVertexIndex(2));
  faces.emplace_back(
      SurfaceVertexIndex(2), SurfaceVertexIndex(3), SurfaceVertexIndex(0));

  return std::make_unique<SurfaceMesh<double>>(
      std::move(faces), std::move(vertices));
}

// Test instantiation of SurfaceMesh using `double` as the underlying scalar
// type.
GTEST_TEST(SurfaceMeshTest, GenerateTwoTriangleMeshDouble) {
  auto surface_mesh = GenerateTwoTriangleMesh<double>();
  EXPECT_EQ(surface_mesh->num_faces(), 2);
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There is no test of
// differentiation.
GTEST_TEST(SurfaceMeshTest, GenerateTwoTriangleMeshAutoDiffXd) {
  auto surface_mesh = GenerateTwoTriangleMesh<AutoDiffXd>();
  EXPECT_EQ(surface_mesh->num_faces(), 2);
}

// Checks the area calculations.
GTEST_TEST(SurfaceMeshTest, TestArea) {
  const double tol = 10 * std::numeric_limits<double>::epsilon();
  auto surface_mesh = GenerateTwoTriangleMesh<double>();
  EXPECT_NEAR(surface_mesh->area(SurfaceFaceIndex(0)), 0.5, tol);
  EXPECT_NEAR(surface_mesh->area(SurfaceFaceIndex(1)), 1.0, tol);
  EXPECT_NEAR(surface_mesh->total_area(), 1.5, tol);

  // Verify that the empty mesh and the zero area mesh both give zero area.
  EXPECT_NEAR(GenerateEmptyMesh()->total_area(), 0.0, tol);
  EXPECT_NEAR(GenerateZeroAreaMesh()->total_area(), 0.0, tol);
}

// Checks the centroid calculations.
GTEST_TEST(SurfaceMeshTest, TestCentroid) {
  const double tol = 10 * std::numeric_limits<double>::epsilon();
  auto surface_mesh = GenerateTwoTriangleMesh<double>();
  const Vector3<double> centroid = surface_mesh->centroid();
  EXPECT_NEAR(centroid[0], 1.0/6, tol);
  EXPECT_NEAR(centroid[1], -1.0/6, tol);
  EXPECT_NEAR(centroid[2], -0.5, tol);

  // The documentation for the centroid method specifies particular behavior
  // when the total area is zero. Test that.
  EXPECT_NEAR(GenerateEmptyMesh()->centroid().norm(), 0.0, tol);
  EXPECT_NEAR(GenerateZeroAreaMesh()->centroid().norm(), 0.0, tol);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
