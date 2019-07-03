#include "drake/geometry/proximity/surface_mesh.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;

// TODO(SeanCurtis-TRI): Due to parallel PRs, there are now *two* two-triangle
//  toy meshes for examining mesh functions. This should be distilled back down
//  to just one.

// Used for testing instantiation of SurfaceMesh and inspecting its components.
template <typename T>
std::unique_ptr<SurfaceMesh<T>> GenerateTwoTriangleMesh() {
  // The surface mesh will consist of four vertices and two co-planar faces and
  // will be constructed such that area and geometric centroid are
  // straightforward to check.

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

// Test instantiation of SurfaceMesh of a surface M and inspecting its
// components. By default, the vertex positions are expressed in M's frame.
// The optional parameter X_WM will change the vertex positions to W's frame.
template<typename T>
std::unique_ptr<SurfaceMesh<T>> TestSurfaceMesh(
    const math::RigidTransform<T> X_WM = math::RigidTransform<T>::Identity()) {
  // A simple surface mesh comprises of two co-planar triangles with vertices on
  // the coordinate axes and the origin like this:
  //   y
  //   |
  //   |
  //   |
  //   v3(0,15,0)  v2(15,15,0)
  //   +-----------+
  //   |         . |
  //   |  f1  . .  |
  //   |    . .    |
  //   | . .   f0  |
  //   |.          |
  //   +-----------+---------- x
  //   v0(0,0,0)  v1(15,0,0)
  //
  // In the picture above, the positions are expressed in a reference M's
  // frame. The optional parameter X_WM will change the vertex positions to W's
  // frame.
  //
  const int face_data[2][3] = {{0, 1, 2}, {2, 3, 0}};
  std::vector<SurfaceFace> faces;
  for (int f = 0; f < 2; ++f) faces.emplace_back(face_data[f]);
  const Vector3<T> vertex_data_M[4] = {
      {0., 0., 0.}, {15., 0., 0.}, {15., 15., 0.}, {0., 15., 0.}};
  std::vector<SurfaceVertex<T>> vertices_W;
  for (int v = 0; v < 4; ++v) vertices_W.emplace_back(X_WM * vertex_data_M[v]);
  auto surface_mesh_W =
      std::make_unique<SurfaceMesh<T>>(move(faces), std::move(vertices_W));

  EXPECT_EQ(2, surface_mesh_W->num_faces());
  EXPECT_EQ(4, surface_mesh_W->num_vertices());
  for (int v = 0; v < 4; ++v)
    EXPECT_EQ(X_WM * vertex_data_M[v],
              surface_mesh_W->vertex(SurfaceVertexIndex(v)).r_MV());
  for (int f = 0; f < 2; ++f)
    for (int v = 0; v < 3; ++v)
      EXPECT_EQ(face_data[f][v],
                surface_mesh_W->element(SurfaceFaceIndex(f)).vertex(v));
  return surface_mesh_W;
}

// Test instantiation of SurfaceMesh using `double` as the underlying scalar
// type.
GTEST_TEST(SurfaceMeshTest, GenerateTwoTriangleMeshDouble) {
  auto surface_mesh = GenerateTwoTriangleMesh<double>();
  EXPECT_EQ(surface_mesh->num_faces(), 2);
}

GTEST_TEST(SurfaceMeshTest, TestSurfaceMeshDouble) {
  auto surface_mesh = TestSurfaceMesh<double>();
  EXPECT_EQ(surface_mesh->num_faces(), 2);
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There is no test of
// differentiation.
GTEST_TEST(SurfaceMeshTest, GenerateTwoTriangleMeshAutoDiffXd) {
  auto surface_mesh = GenerateTwoTriangleMesh<AutoDiffXd>();
  EXPECT_EQ(surface_mesh->num_faces(), 2);
}

// Tests that referring_triangles() produces the correct sets of referring
// triangles.
GTEST_TEST(SurfaceMeshTest, ReferringTriangles) {
  auto surface_mesh = GenerateTwoTriangleMesh<double>();
  ASSERT_EQ(surface_mesh->num_vertices(), 4);

  // Get the four sets.
  const std::set<SurfaceFaceIndex>& tris_referring_to_0 =
      surface_mesh->referring_triangles(SurfaceVertexIndex(0));
  const std::set<SurfaceFaceIndex>& tris_referring_to_1 =
      surface_mesh->referring_triangles(SurfaceVertexIndex(1));
  const std::set<SurfaceFaceIndex>& tris_referring_to_2 =
      surface_mesh->referring_triangles(SurfaceVertexIndex(2));
  const std::set<SurfaceFaceIndex>& tris_referring_to_3 =
      surface_mesh->referring_triangles(SurfaceVertexIndex(3));

  // Construct the expected sets.
  std::set<SurfaceFaceIndex> expected_tris_referring_to_0{
      SurfaceFaceIndex(0), SurfaceFaceIndex(1) };
  std::set<SurfaceFaceIndex> expected_tris_referring_to_1{SurfaceFaceIndex(0)};
  std::set<SurfaceFaceIndex> expected_tris_referring_to_2{
      SurfaceFaceIndex(0), SurfaceFaceIndex(1) };
  std::set<SurfaceFaceIndex> expected_tris_referring_to_3{SurfaceFaceIndex(1)};

  // Check the results.
  EXPECT_EQ(expected_tris_referring_to_0, tris_referring_to_0);
  EXPECT_EQ(expected_tris_referring_to_1, tris_referring_to_1);
  EXPECT_EQ(expected_tris_referring_to_2, tris_referring_to_2);
  EXPECT_EQ(expected_tris_referring_to_3, tris_referring_to_3);
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

GTEST_TEST(SurfaceMeshTest, TestSurfaceMeshAutoDiffXd) {
  auto surface_mesh = TestSurfaceMesh<AutoDiffXd>();
}

template<typename T>
void TestCalcBarycentric() {
  const math::RigidTransform<T> X_WM(
      math::RollPitchYaw<T>(M_PI / 6.0, 2.0 * M_PI / 3.0, 7.0 * M_PI / 4.0),
      Vector3<T>(1.0, 2.0, 3.0));
  auto surface_mesh_W = TestSurfaceMesh<T>(X_WM);
  // Empirically the std::numeric_limits<double>::epsilon() is too small to
  // account for the pose.
  const T kTolerance(1e-14);
  const SurfaceFaceIndex f0(0);

  // At v1.
  {
    const Vector3<T> p_M(15., 0., 0.);
    auto barycentric = surface_mesh_W->CalcBarycentric(X_WM * p_M, f0);
    typename SurfaceMesh<T>::Barycentric expect_barycentric(0., 1., 0.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Twice closer to v0 than v1.
  {
    const Vector3<T> p_M(5., 0., 0);
    auto barycentric = surface_mesh_W->CalcBarycentric(X_WM * p_M, f0);
    typename SurfaceMesh<T>::Barycentric expect_barycentric(2. / 3., 1. / 3.,
                                                            0.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Generic position in the triangle.
  {
    const Vector3<T> p_M(10., 3., 0);
    auto barycentric = surface_mesh_W->CalcBarycentric(X_WM * p_M, f0);
    typename SurfaceMesh<T>::Barycentric expect_barycentric(1. / 3, 7. / 15.,
                                                            1. / 5.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Outside but still on the plane of the triangle.
  {
    const Vector3<T> p_M(30., 7.5, 0.);
    auto barycentric = surface_mesh_W->CalcBarycentric(X_WM * p_M, f0);
    typename SurfaceMesh<T>::Barycentric expect_barycentric(-1., 1.5, 0.5);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Out of the plane of the triangle but still projected into the triangle.
  {
    const Vector3<T> above_midpoint(10., 3., 27.);
    auto barycentric =
        surface_mesh_W->CalcBarycentric(X_WM * above_midpoint, f0);
    typename SurfaceMesh<T>::Barycentric expect_barycentric(1. / 3, 7. / 15.,
                                                            1. / 5.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Out of the plane of the triangle and projected outside the triangle.
  {
    const Vector3<T> p_M(30., 7.5, 27.);
    auto barycentric = surface_mesh_W->CalcBarycentric(X_WM * p_M, f0);
    typename SurfaceMesh<T>::Barycentric expect_barycentric(-1., 1.5, 0.5);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
}

GTEST_TEST(SurfaceMeshTest, TestCalcBarycentricDouble) {
  TestCalcBarycentric<double>();
}

GTEST_TEST(SurfaceMeshTest, TestCalcBarycentricAutoDiffXd) {
  TestCalcBarycentric<AutoDiffXd>();
}

GTEST_TEST(SurfaceMeshTest, ReverseFaceWinding) {
  auto ref_mesh = TestSurfaceMesh<double>();
  auto test_mesh = std::make_unique<SurfaceMesh<double>>(*ref_mesh);

  // Simply confirms the two faces have the same indices in the same order.
  auto faces_match = [](const SurfaceFace& ref_face,
                            const SurfaceFace& test_face) {
    for (int i = 0; i < 3; ++i) {
      if (ref_face.vertex(i) != test_face.vertex(i)) return false;
    }
    return true;
  };

  for (int value : {0, 1}) {
    SurfaceFaceIndex i(value);
    EXPECT_TRUE(faces_match(ref_mesh->element(i), test_mesh->element(i)));
  }

  test_mesh->ReverseFaceWinding();

  // Confirms that the two faces have the same indices but in reverse order.
  auto winding_reversed = [](const SurfaceFace& ref_face,
                             const SurfaceFace& test_face) {
    int test_first = test_face.vertex(0);
    int ref_first = -1;
    for (int i = 0; i < 3; ++i) {
      if (test_first == ref_face.vertex(i)) {
        ref_first = i;
        break;
      }
    }
    if (ref_first == -1)  return false;

    // We now have a common vertex: v at indices r and t, for the ref face and
    // test face, respectively. Confirm that ref[r + 1] == test[t - 1]
    // and ref[r + 2] == test[t - 2]. We know that t = 0 by construction so
    // we know at compile time that t - 1 = 2 and t - 2 = 1 (via circular
    // indexing). So, we only need to find r as (r + i) % 3.
    bool winding_is_valid = true;
    winding_is_valid &= ref_face.vertex((ref_first + 1) % 3) ==
        test_face.vertex(2);
    winding_is_valid &= ref_face.vertex((ref_first + 2) % 3) ==
        test_face.vertex(1);
    return winding_is_valid;
  };

  for (int value : {0, 1}) {
    SurfaceFaceIndex i(value);
    EXPECT_TRUE(winding_reversed(ref_mesh->element(i), test_mesh->element(i)));
  }
}

GTEST_TEST(SurfaceMeshTest, TransformVertices) {
  auto ref_mesh = TestSurfaceMesh<double>();
  auto test_mesh = std::make_unique<SurfaceMesh<double>>(*ref_mesh);

  // Assume that the copy constructor works properly.

  RigidTransformd X_FM{AngleAxisd{M_PI / 4, Vector3d(1, 2, 3).normalized()},
                       Vector3d{1, 2, 3}};
  test_mesh->TransformVertices(X_FM);

  for (SurfaceVertexIndex v(0); v < test_mesh->num_vertices(); ++v) {
    const Vector3d& p_FV_test = test_mesh->vertex(v).r_MV();
    const Vector3d& p_MV_ref = ref_mesh->vertex(v).r_MV();
    const Vector3d p_FV_ref = X_FM * p_MV_ref;
    EXPECT_TRUE(CompareMatrices(p_FV_test, p_FV_ref));
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
