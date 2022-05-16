#include "drake/geometry/proximity/triangle_surface_mesh.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

// TODO(DamrongGuoy): Remove this helper class if we change
//  CalcGradBarycentric() from private to public.
template <typename T>
class TriangleSurfaceMeshTester {
 public:
  explicit TriangleSurfaceMeshTester(const TriangleSurfaceMesh<T>& mesh)
      : mesh_(mesh) {}
  Vector3<T> CalcGradBarycentric(int f, int i) const {
    return mesh_.CalcGradBarycentric(f, i);
  }
 private:
  const TriangleSurfaceMesh<T>& mesh_;
};

namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;

// TODO(SeanCurtis-TRI): Due to parallel PRs, there are now *two* two-triangle
//  toy meshes for examining mesh functions. This should be distilled back down
//  to just one.

// Used for testing instantiation of TriangleSurfaceMesh and inspecting its
// components.
template <typename T>
std::unique_ptr<TriangleSurfaceMesh<T>> GenerateTwoTriangleMesh() {
  // The surface mesh will consist of four vertices and two co-planar faces and
  // will be constructed such that area and geometric centroid are
  // straightforward to check.

  // Create the vertices.
  std::vector<Vector3<T>> vertices;
  vertices.emplace_back(Vector3<T>(0.5, 0.5, -0.5));
  vertices.emplace_back(Vector3<T>(-0.5, 0.5, -0.5));
  vertices.emplace_back(Vector3<T>(-0.5, -0.5, -0.5));
  vertices.emplace_back(Vector3<T>(1.0, -1.0, -0.5));

  // Create the two triangles. Note that TriangleSurfaceMesh does not specify
  // (or use) a particular winding.
  std::vector<SurfaceTriangle> faces;
  faces.emplace_back(0, 1, 2);
  faces.emplace_back(2, 3, 0);

  return std::make_unique<TriangleSurfaceMesh<T>>(
      std::move(faces), std::move(vertices));
}

// Generates an empty mesh.
std::unique_ptr<TriangleSurfaceMesh<double>> GenerateEmptyMesh() {
  std::vector<Vector3d> vertices;
  std::vector<SurfaceTriangle> faces;
  return std::make_unique<TriangleSurfaceMesh<double>>(
      std::move(faces), std::move(vertices));
}

// Generates a zero-area mesh.
std::unique_ptr<TriangleSurfaceMesh<double>> GenerateZeroAreaMesh() {
  // The surface mesh will consist of four vertices and two faces.

  // Create the vertices.
  std::vector<Vector3d> vertices;
  for (int i = 0; i < 4; ++i)
    vertices.emplace_back(Vector3<double>::Zero());

  // Create the two triangles.
  std::vector<SurfaceTriangle> faces;
  faces.emplace_back(0, 1, 2);
  faces.emplace_back(2, 3, 0);

  return std::make_unique<TriangleSurfaceMesh<double>>(
      std::move(faces), std::move(vertices));
}

// Test instantiation of TriangleSurfaceMesh of a surface M and inspecting its
// components. By default, the vertex positions are expressed in M's frame.
// The optional parameter X_WM will change the vertex positions to W's frame.
template<typename T>
std::unique_ptr<TriangleSurfaceMesh<T>> TestSurfaceMesh(
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
  std::vector<SurfaceTriangle> faces;
  for (int f = 0; f < 2; ++f) faces.emplace_back(face_data[f]);
  const Vector3<T> vertex_data_M[4] = {
      {0., 0., 0.}, {15., 0., 0.}, {15., 15., 0.}, {0., 15., 0.}};
  std::vector<Vector3<T>> vertices_W;
  for (int v = 0; v < 4; ++v) vertices_W.emplace_back(X_WM * vertex_data_M[v]);
  auto surface_mesh_W = std::make_unique<TriangleSurfaceMesh<T>>(
      move(faces), std::move(vertices_W));

  EXPECT_EQ(2, surface_mesh_W->num_triangles());
  EXPECT_EQ(4, surface_mesh_W->num_vertices());
  for (int v = 0; v < 4; ++v)
    EXPECT_EQ(X_WM * vertex_data_M[v], surface_mesh_W->vertex(v));
  for (int f = 0; f < 2; ++f) {
    EXPECT_EQ(surface_mesh_W->element(f).num_vertices(), 3);
    for (int v = 0; v < 3; ++v)
      EXPECT_EQ(face_data[f][v], surface_mesh_W->element(f).vertex(v));
  }
  return surface_mesh_W;
}

// Test instantiation of TriangleSurfaceMesh using `double` as the underlying
// scalar type.
GTEST_TEST(SurfaceMeshTest, GenerateTwoTriangleMeshDouble) {
  auto surface_mesh = GenerateTwoTriangleMesh<double>();
  EXPECT_EQ(surface_mesh->num_triangles(), 2);
}

GTEST_TEST(SurfaceMeshTest, TestSurfaceMeshDouble) {
  auto surface_mesh = TestSurfaceMesh<double>();
  EXPECT_EQ(surface_mesh->num_triangles(), 2);
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There is no test of
// differentiation.
GTEST_TEST(SurfaceMeshTest, GenerateTwoTriangleMeshAutoDiffXd) {
  auto surface_mesh = GenerateTwoTriangleMesh<AutoDiffXd>();
  EXPECT_EQ(surface_mesh->num_triangles(), 2);
}

// Checks the area calculations.
GTEST_TEST(SurfaceMeshTest, TestArea) {
  const double tol = 10 * std::numeric_limits<double>::epsilon();
  auto surface_mesh = GenerateTwoTriangleMesh<double>();
  EXPECT_NEAR(surface_mesh->area(0), 0.5, tol);
  EXPECT_NEAR(surface_mesh->area(1), 1.0, tol);
  EXPECT_NEAR(surface_mesh->total_area(), 1.5, tol);

  // Verify that the zero area mesh gives zero area.
  EXPECT_NEAR(GenerateZeroAreaMesh()->total_area(), 0.0, tol);
}

// Checks the face normal calculations.
GTEST_TEST(SurfaceMeshTest, TestFaceNormal) {
  const math::RigidTransform<double> X_WM(
      math::RollPitchYaw<double>(M_PI / 6.0, M_PI / 3.0, M_PI / 4.0),
      Vector3<double>(1.0, 2.0, 3.0));
  // We estimate the rounding errors from rotation (multiply a vector by a
  // 3x3 matrix) about 3 machine epsilons.
  const double tol = 3.0 * std::numeric_limits<double>::epsilon();
  const auto surface_mesh = TestSurfaceMesh<double>(X_WM);
  const Vector3<double> expect_normal =
      X_WM.rotation() * Vector3<double>::UnitZ();
  EXPECT_TRUE(
      CompareMatrices(expect_normal, surface_mesh->face_normal(0), tol));
  EXPECT_TRUE(
      CompareMatrices(expect_normal, surface_mesh->face_normal(1), tol));

  // Verify that the zero-area mesh has zero-vector face normal.
  const auto zero_mesh = GenerateZeroAreaMesh();
  const Vector3<double> zero_normal = Vector3<double>::Zero();
  EXPECT_EQ(zero_normal, zero_mesh->face_normal(0));
  EXPECT_EQ(zero_normal, zero_mesh->face_normal(1));
}

// Checks the centroid calculations.
GTEST_TEST(SurfaceMeshTest, TestCentroid) {
  const double tol = 10 * std::numeric_limits<double>::epsilon();
  auto surface_mesh = GenerateTwoTriangleMesh<double>();
  const Vector3<double> centroid = surface_mesh->centroid();
  EXPECT_TRUE(
      CompareMatrices(centroid, Vector3d(1.0 / 6, -1.0 / 6, -0.5), tol));
  EXPECT_TRUE(CompareMatrices(surface_mesh->element_centroid(0),
                              Vector3d(-0.5 / 3, 0.5 / 3, -0.5), tol));
  EXPECT_TRUE(CompareMatrices(surface_mesh->element_centroid(1),
                              Vector3d(1.0 / 3, -1.0 / 3, -0.5), tol));

  // The documentation for the centroid method specifies particular behavior
  // when the total area is zero. Test that.
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
  const int f0 = 0;
  using Barycentric = typename TriangleSurfaceMesh<T>::template Barycentric<T>;

  // At v1.
  {
    const Vector3<T> p_M(15., 0., 0.);
    auto barycentric = surface_mesh_W->CalcBarycentric(X_WM * p_M, f0);
    const Barycentric expect_barycentric(0., 1., 0.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Twice closer to v0 than v1.
  {
    const Vector3<T> p_M(5., 0., 0);
    auto barycentric = surface_mesh_W->CalcBarycentric(X_WM * p_M, f0);
    const Barycentric expect_barycentric(2. / 3., 1. / 3., 0.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Generic position in the triangle.
  {
    const Vector3<T> p_M(10., 3., 0);
    auto barycentric = surface_mesh_W->CalcBarycentric(X_WM * p_M, f0);
    const Barycentric expect_barycentric(1. / 3, 7. / 15., 1. / 5.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Outside but still on the plane of the triangle.
  {
    const Vector3<T> p_M(30., 7.5, 0.);
    auto barycentric = surface_mesh_W->CalcBarycentric(X_WM * p_M, f0);
    const Barycentric expect_barycentric(-1., 1.5, 0.5);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Out of the plane of the triangle but still projected into the triangle.
  {
    const Vector3<T> above_midpoint(10., 3., 27.);
    auto barycentric =
        surface_mesh_W->CalcBarycentric(X_WM * above_midpoint, f0);
    const Barycentric expect_barycentric(1. / 3, 7. / 15., 1. / 5.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // Out of the plane of the triangle and projected outside the triangle.
  {
    const Vector3<T> p_M(30., 7.5, 27.);
    auto barycentric = surface_mesh_W->CalcBarycentric(X_WM * p_M, f0);
    const Barycentric expect_barycentric(-1., 1.5, 0.5);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
}

GTEST_TEST(SurfaceMeshTest, TestCalcBarycentricDouble) {
  TestCalcBarycentric<double>();
}

GTEST_TEST(SurfaceMeshTest, TestCalcBarycentricAutoDiffXd) {
  TestCalcBarycentric<AutoDiffXd>();
}

template <typename T>
void TestCalcGradBarycentric() {
  // The mesh M consists of one triangle whose vertices are at the origin and
  // on the X's axis and Y's axis of M's frame. The chosen vertex coordinates
  // avoid symmetry but are easy to calculate ∇bᵢ manually.
  //
  //                                 Z
  //                                 |
  //     v0_M = (0,0,0)              |
  //     v1_M = (1,0,0)              |   M's frame
  //     v2_M = (0,2,0)              |
  //                               v0+----+----v2----Y
  //                                /
  //                              v1
  //                              /
  //                             /
  //                            X
  //
  const int triangle[3] = {0, 1, 2};
  const Vector3<T> v0_M(0., 0., 0.);
  const Vector3<T> v1_M(1., 0., 0.);
  const Vector3<T> v2_M(0., 2., 0.);

  // We use an arbitrary pose of the surface mesh M in World frame W to make the
  // test more realistic.
  const math::RigidTransform<T> X_WM(
      math::RollPitchYaw<T>(M_PI / 6.0, 2.0 * M_PI / 3.0, 5.0 * M_PI / 4.0),
      Vector3<T>(1.0, 2.0, 3.0));

  // Create the mesh with vertex coordinates expressed in World frame to make
  // the test more realistic.
  const TriangleSurfaceMesh<T> mesh_W(
      {SurfaceTriangle(triangle)},
      {Vector3<T>(X_WM * v0_M), Vector3<T>(X_WM * v1_M),
       Vector3<T>(X_WM * v2_M)});

  const TriangleSurfaceMeshTester<T> tester(mesh_W);
  const auto gradb0_W = tester.CalcGradBarycentric(0, 0);
  const auto gradb1_W = tester.CalcGradBarycentric(0, 1);
  const auto gradb2_W = tester.CalcGradBarycentric(0, 2);

  // In this example, we have these equations, expressed in M's frame:
  //      b₀(x,y,z) = -x - y/2 + 1,
  //      b₁(x,y,z) = x,
  //      b₂(x,y,z) = y/2.
  // We can manually verify the equations by checking that bᵢ(vi) = 1 for all i,
  // bⱼ(vj) = 0 for all i ≠ j, and ∑bⱼ = 1.
  //     We can read off ∇bᵢ from the coefficients of x,y,z in the above
  // equations.
  const Vector3<T> expect_gradb0_M(-1., -1. / 2., 0.);
  const Vector3<T> expect_gradb1_M = Vector3<T>::UnitX();
  const Vector3<T> expect_gradb2_M = Vector3<T>::UnitY() / 2.;

  const auto& R_WM = X_WM.rotation();
  EXPECT_TRUE(CompareMatrices(R_WM * expect_gradb0_M, gradb0_W, 1e-14));
  EXPECT_TRUE(CompareMatrices(R_WM * expect_gradb1_M, gradb1_W, 1e-14));
  EXPECT_TRUE(CompareMatrices(R_WM * expect_gradb2_M, gradb2_W, 1e-14));

  // Since ∑bᵢ = 1, the gradients have zero sum: ∑(∇bᵢ) = 0.
  EXPECT_TRUE(CompareMatrices(gradb0_W + gradb1_W + gradb2_W,
                              Vector3<T>::Zero(), 1e-14));
}

GTEST_TEST(SurfaceMeshTest, TestCalcGradBarycentricDouble) {
  TestCalcGradBarycentric<double>();
}

GTEST_TEST(SurfaceMeshTest, TestCalcGradBarycentricAutoDiffXd) {
  TestCalcGradBarycentric<AutoDiffXd>();
}

GTEST_TEST(SurfaceMeshTest, TestCalcGradBarycentricZeroAreaTriangle) {
  std::unique_ptr<TriangleSurfaceMesh<double>> mesh = GenerateZeroAreaMesh();
  const TriangleSurfaceMeshTester<double> tester(*mesh);
  EXPECT_THROW(tester.CalcGradBarycentric(0, 0),
               std::runtime_error);
}

template <typename T>
void TestCalcGradientVectorOfLinearField() {
  // CalcGradientVectorOfLinearField() is a weighted sum of
  // CalcGradBarycentric() which is already tested with non-symmetric vertex
  // coordinates and an arbitrary pose. Therefore, it suffices to test
  // CalcGradientVectorOfLinearField() in the mesh's frame M with symmetric
  // vertex coordinates but non-symmetric field values.
  //
  // The three vertices v0,v1,v2 of the triangle have coordinates expressed
  // in M's frame as:
  //
  //                                +Z
  //                                 |
  //     v0_M = (1,0,0), f₀ = 2      v2
  //     v1_M = (0,1,0), f₁ = 3      |   M's frame
  //     v2_M = (0,0,1), f₂ = 4      |
  //                                 +------v1---+Y
  //                                /
  //                               /
  //                             v0
  //                             /
  //                           +X
  //
  const int triangle[3] = {0, 1, 2};
  const Vector3<T> v0_M(1., 0., 0.);
  const Vector3<T> v1_M(0., 1., 0.);
  const Vector3<T> v2_M(0., 0., 1.);
  const TriangleSurfaceMesh<T> mesh_M(
      {SurfaceTriangle(triangle)},
      {Vector3<T>(v0_M), Vector3<T>(v1_M), Vector3<T>(v2_M)});
  const std::array<T, 3> f{2., 3., 4.};

  const Vector3<T> gradf_M =
      mesh_M.CalcGradientVectorOfLinearField(f, 0);

  // This function
  //       f(x,y,z) = -x + z + 3
  // satisfies f(vi_M) = fᵢ, i.e.,
  //       f(1,0,0) = 2
  //       f(0,1,0) = 3
  //       f(0,0,1) = 4
  // and its gradient ∇f is orthogonal to the triangle normal (1,1,1)/√3, i.e.,
  // ∇f is along the triangle (belong to the tangent plane of the triangle).
  // There are many linear functions that satisfy f(vi_M) = fᵢ since there
  // are only three constraints for i=0,1,2, but a linear function has four
  // parameters A*x + B*y + C*z + D.  We want the one with its gradient along
  // the triangle.
  //     Therefore, we can read off ∇f from the coefficients of x,y,z in the
  // equation.
  const Vector3<T> expect_gradf_M{-1., 0., 1};

  EXPECT_TRUE(CompareMatrices(expect_gradf_M, gradf_M, 1e-14));
}

GTEST_TEST(SurfaceMeshTest, TestCalcGradientVectorOfLinearFieldDouble) {
  TestCalcGradientVectorOfLinearField<double>();
}

GTEST_TEST(SurfaceMeshTest, TestCalcGradientVectorOfLinearFieldAutoDiffXd) {
  TestCalcGradientVectorOfLinearField<AutoDiffXd>();
}

GTEST_TEST(SurfaceMeshTest, ReverseFaceWinding) {
  auto ref_mesh = TestSurfaceMesh<double>();
  auto test_mesh = std::make_unique<TriangleSurfaceMesh<double>>(*ref_mesh);

  // Simply confirms the two faces have the same indices in the same order.
  auto faces_match = [](const SurfaceTriangle& ref_face,
                        const SurfaceTriangle& test_face) {
    for (int i = 0; i < 3; ++i) {
      if (ref_face.vertex(i) != test_face.vertex(i)) return false;
    }
    return true;
  };

  for (int i : {0, 1}) {
    EXPECT_TRUE(faces_match(ref_mesh->element(i), test_mesh->element(i)));
  }

  test_mesh->ReverseFaceWinding();

  // Confirms that the two faces have the same indices but in reverse order.
  auto winding_reversed = [](const SurfaceTriangle& ref_face,
                             const SurfaceTriangle& test_face) {
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

  for (int i : {0, 1}) {
    EXPECT_TRUE(winding_reversed(ref_mesh->element(i), test_mesh->element(i)));
  }

  for (int i : {0, 1}) {
    EXPECT_EQ(ref_mesh->face_normal(i), - test_mesh->face_normal(i));
  }
}

GTEST_TEST(SurfaceMeshTest, TransformVertices) {
  auto ref_mesh = TestSurfaceMesh<double>();
  auto test_mesh = std::make_unique<TriangleSurfaceMesh<double>>(*ref_mesh);

  // Assume that the copy constructor works properly.

  RigidTransformd X_FM{AngleAxisd{M_PI / 4, Vector3d(1, 2, 3).normalized()},
                       Vector3d{1, 2, 3}};
  test_mesh->TransformVertices(X_FM);

  for (int v = 0; v < test_mesh->num_vertices(); ++v) {
    const Vector3d& p_FV_test = test_mesh->vertex(v);
    const Vector3d& p_MV_ref = ref_mesh->vertex(v);
    const Vector3d p_FV_ref = X_FM * p_MV_ref;
    EXPECT_TRUE(CompareMatrices(p_FV_test, p_FV_ref));
  }

  for (int f = 0; f < test_mesh->num_triangles(); ++f) {
    const Vector3d& nhat_F_test = test_mesh->face_normal(f);
    const Vector3d& nhat_M_ref = ref_mesh->face_normal(f);
    const Vector3d nhat_F_ref = X_FM.rotation() * nhat_M_ref;
    EXPECT_TRUE(CompareMatrices(nhat_F_test, nhat_F_ref));
  }

  const Vector3d& p_FSc_test = test_mesh->centroid();
  const Vector3d& p_MSc_ref = ref_mesh->centroid();
  const Vector3d p_FSc_ref = X_FM * p_MSc_ref;
  EXPECT_TRUE(CompareMatrices(p_FSc_test, p_FSc_ref));
}

// Checks the equality calculations.
GTEST_TEST(SurfaceMeshTest, TestEqual) {
  const auto zero_area_mesh = GenerateZeroAreaMesh();
  const auto triangle_mesh = GenerateTwoTriangleMesh<double>();
  TriangleSurfaceMesh<double> triangle_mesh_copy = *triangle_mesh;
  EXPECT_TRUE(triangle_mesh->Equal(*triangle_mesh));
  EXPECT_TRUE(triangle_mesh->Equal(triangle_mesh_copy));
  EXPECT_FALSE(zero_area_mesh->Equal(*triangle_mesh));
}

// Checks that constructing an empty mesh throws.
GTEST_TEST(SurfaceMeshTest, TestEmptyMeshElements) {
  EXPECT_THROW(GenerateEmptyMesh(), std::logic_error);
}

GTEST_TEST(SurfaceMeshTest, CalcBoundingBox) {
  auto mesh = TestSurfaceMesh<double>();
  const auto [center, size] = mesh->CalcBoundingBox();
  EXPECT_EQ(center, Vector3d(7.5, 7.5, 0));
  EXPECT_EQ(size, Vector3d(15, 15, 0));
}

/* A double-valued mesh can produce AutoDiffXd-valued results for
 CalcBarycentric() and CalcCartesianFromBarycentric() based on the scalar type
 of the query point. This confirms that mixing behavior. The tests work by
 instantiating meshes on both scalar types and various query parameters on
 both types. Then we mix and match mesh and parameter scalars and verify the
 outputs. By "verify", we don't worry too much about the correctness of the
 values and derivatives; we're just checking that, mechanically speaking,
 derivatives are propagating as expected. We make sure that AutoDiffXd-valued
 parameters have at least *one* value with non-zero derivatives. */
class ScalarMixingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    mesh_d_ = GenerateTwoTriangleMesh<double>();

    // We construct an AutoDiffXd-valued mesh from the double-valued mesh. We
    // only set the derivatives for vertex 1. That means, operations on
    // triangle 0 *must* have derivatives, but triangle 1 may not have them.
    std::vector<Vector3<AutoDiffXd>> vertices;
    vertices.emplace_back(mesh_d_->vertex(0));
    vertices.emplace_back(math::InitializeAutoDiff(mesh_d_->vertex(1)));
    vertices.emplace_back(mesh_d_->vertex(2));
    vertices.emplace_back(mesh_d_->vertex(3));
    std::vector<SurfaceTriangle> faces(mesh_d_->triangles());

    mesh_ad_ = std::make_unique<TriangleSurfaceMesh<AutoDiffXd>>(
        std::move(faces), std::move(vertices));

    p_WQ_d_ = Vector3d::Zero();
    for (int v = 0; v < 3; ++v) {
      p_WQ_d_ += mesh_d_->vertex(v);
    }
    p_WQ_d_ /= 3;
    p_WQ_ad_ = math::InitializeAutoDiff(p_WQ_d_);

    b_expected_d_ = Vector3d(1, 1, 1) / 3.0;
    b_expected_ad_ = math::InitializeAutoDiff(b_expected_d_);
  }

  std::unique_ptr<TriangleSurfaceMesh<double>> mesh_d_;
  std::unique_ptr<TriangleSurfaceMesh<AutoDiffXd>> mesh_ad_;

  int e0_{0};
  int e1_{1};
  // The centroid of triangle 0.
  Vector3<double> p_WQ_d_;
  Vector3<AutoDiffXd> p_WQ_ad_;
  // The centroid of a triangle.
  Vector3<double> b_expected_d_;
  Vector3<AutoDiffXd> b_expected_ad_;
};

TEST_F(ScalarMixingTest, CalcBarycentric) {
  constexpr double kEps = std::numeric_limits<double>::epsilon();

  {
    // Double-valued mesh and double-valued point: double-valued result.
    const Vector3d b = mesh_d_->CalcBarycentric(p_WQ_d_, e0_);
    EXPECT_TRUE(CompareMatrices(b, b_expected_d_, kEps));
  }

  {
    // Double-valued mesh with AutoDiffXd-valued point: AutodDiffXd-valued
    // result.
    const Vector3<AutoDiffXd>& b = mesh_d_->CalcBarycentric(p_WQ_ad_, e0_);
    EXPECT_EQ(b(0).derivatives().size(), 3);
    EXPECT_EQ(b(1).derivatives().size(), 3);
    EXPECT_EQ(b(2).derivatives().size(), 3);
    EXPECT_TRUE(CompareMatrices(math::ExtractValue(b), b_expected_d_, kEps));
  }

  {
    // AutoDiffXd-valued mesh with double-valued point on triangle *with*
    // derivatives: AutodDiffXd-valued result *with* derivatives.
    const Vector3<AutoDiffXd>& b1 = mesh_ad_->CalcBarycentric(p_WQ_d_, e0_);
    EXPECT_EQ(b1(0).derivatives().size(), 3);
    EXPECT_EQ(b1(1).derivatives().size(), 3);
    EXPECT_EQ(b1(2).derivatives().size(), 3);
    EXPECT_TRUE(CompareMatrices(math::ExtractValue(b1), b_expected_d_, kEps));

    // AutoDiffXd-valued mesh with double-valued point on triangle *without*
    // derivatives: AutodDiffXd-valued result *without* derivatives.
    const Vector3<AutoDiffXd>& b2 = mesh_ad_->CalcBarycentric(p_WQ_d_, e1_);
    EXPECT_EQ(b2(0).derivatives().size(), 0);
    EXPECT_EQ(b2(1).derivatives().size(), 0);
    EXPECT_EQ(b2(2).derivatives().size(), 0);
    // We'll assume the *value* on this one is correct.
  }

  {
    // AutoDiffXd-valued mesh with AutoDiffXd-valued point on triangle:
    // AutodDiffXd-valued.
    const Vector3<AutoDiffXd>& b = mesh_ad_->CalcBarycentric(p_WQ_ad_, e0_);
    EXPECT_EQ(b(0).derivatives().size(), 3);
    EXPECT_EQ(b(1).derivatives().size(), 3);
    EXPECT_EQ(b(2).derivatives().size(), 3);
    EXPECT_TRUE(CompareMatrices(math::ExtractValue(b), b_expected_d_, kEps));
  }
}

TEST_F(ScalarMixingTest, CalcCartesianFromBarycentric) {
  constexpr double kEps = std::numeric_limits<double>::epsilon();

  {
    // Double-valued mesh and double-valued point: double-valued result.
    const Vector3d p_WC =
        mesh_d_->CalcCartesianFromBarycentric(e0_, b_expected_d_);
    EXPECT_TRUE(CompareMatrices(p_WC, p_WQ_d_, kEps));
  }

  {
    // Double-valued mesh with AutoDiffXd-valued point: AutodDiffXd-valued
    // result.
    const Vector3<AutoDiffXd>& p_WC =
        mesh_d_->CalcCartesianFromBarycentric(e0_, b_expected_ad_);
    EXPECT_EQ(p_WC(0).derivatives().size(), 3);
    EXPECT_EQ(p_WC(1).derivatives().size(), 3);
    EXPECT_EQ(p_WC(2).derivatives().size(), 3);
    EXPECT_TRUE(CompareMatrices(math::ExtractValue(p_WC), p_WQ_d_, kEps));
  }

  {
    // AutoDiffXd-valued mesh with double-valued point on triangle *with*
    // derivatives: AutodDiffXd-valued result *with* derivatives.
    const Vector3<AutoDiffXd>& p_WC1 =
        mesh_ad_->CalcCartesianFromBarycentric(e0_, b_expected_d_);
    EXPECT_EQ(p_WC1(0).derivatives().size(), 3);
    EXPECT_EQ(p_WC1(1).derivatives().size(), 3);
    EXPECT_EQ(p_WC1(2).derivatives().size(), 3);
    EXPECT_TRUE(CompareMatrices(math::ExtractValue(p_WC1), p_WQ_d_, kEps));

    // AutoDiffXd-valued mesh with double-valued point on triangle *without*
    // derivatives: AutodDiffXd-valued result *without* derivatives.
    const Vector3<AutoDiffXd>& p_WC2 =
        mesh_ad_->CalcCartesianFromBarycentric(e1_, b_expected_d_);
    EXPECT_EQ(p_WC2(0).derivatives().size(), 0);
    EXPECT_EQ(p_WC2(1).derivatives().size(), 0);
    EXPECT_EQ(p_WC2(2).derivatives().size(), 0);
    // We'll assume the *value* on this one is correct.
  }

  {
    // AutoDiffXd-valued mesh with AutoDiffXd-valued point on triangle:
    // AutodDiffXd-valued.
    const Vector3<AutoDiffXd>& p_WC =
        mesh_ad_->CalcCartesianFromBarycentric(e0_, b_expected_ad_);
    EXPECT_EQ(p_WC(0).derivatives().size(), 3);
    EXPECT_EQ(p_WC(1).derivatives().size(), 3);
    EXPECT_EQ(p_WC(2).derivatives().size(), 3);
    EXPECT_TRUE(CompareMatrices(math::ExtractValue(p_WC), p_WQ_d_, kEps));
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
