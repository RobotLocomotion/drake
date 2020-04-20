#include "drake/geometry/proximity/volume_mesh.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

// TODO(DamrongGuoy): Remove this helper class if we change
//  CalcGradBarycentric() from private to public.
template<typename T>
class VolumeMeshTester {
 public:
  explicit VolumeMeshTester(const VolumeMesh<T>& mesh) : mesh_(mesh) {}
  Vector3<T> CalcGradBarycentric(VolumeElementIndex e, int i) const {
    return mesh_.CalcGradBarycentric(e, i);
  }
 private:
  const VolumeMesh<T>& mesh_;
};

namespace {

using math::RigidTransform;
using math::RollPitchYaw;

// Test instantiation of VolumeMesh of a geometry M and inspecting its
// components. By default, the vertex positions are expressed in M's frame.
// The optional parameter X_WM will change the vertex positions to W's frame.
template<typename T>
std::unique_ptr<VolumeMesh<T>> TestVolumeMesh(
    const RigidTransform<T> X_WM = RigidTransform<T>::Identity()) {
  // A  trivial volume mesh comprises of two tetrahedral elements with
  // vertices on the coordinate axes and the origin like this:
  //
  //      +Z
  //       |
  //       v3
  //       |
  //       |
  //     v0+------v2---+Y
  //      /|
  //     / |
  //   v1  v4
  //   /   |
  // +X    |
  //      -Z
  //
  // In the picture above, the positions are expressed in M's frame. The
  // optional parameter X_WM will change the vertex positions to W's frame.
  //
  const int element_data[2][4] = {{0, 1, 2, 3}, {0, 2, 1, 4}};
  std::vector<VolumeElement> elements;
  for (int e = 0; e < 2; ++e) elements.emplace_back(element_data[e]);
  const Vector3<T> vertex_data[5] = {Vector3<T>::Zero(), Vector3<T>::UnitX(),
                                     Vector3<T>::UnitY(), Vector3<T>::UnitZ(),
                                     -Vector3<T>::UnitZ()};
  std::vector<VolumeVertex<T>> vertices_W;
  for (int v = 0; v < 5; ++v) vertices_W.emplace_back(X_WM * vertex_data[v]);
  auto volume_mesh_W = std::make_unique<VolumeMesh<T>>(std::move(elements),
                                                       std::move(vertices_W));
  EXPECT_EQ(2, volume_mesh_W->num_elements());
  EXPECT_EQ(5, volume_mesh_W->num_vertices());
  for (int v = 0; v < 5; ++v)
    EXPECT_EQ(X_WM * vertex_data[v],
              volume_mesh_W->vertex(VolumeVertexIndex(v)).r_MV());
  for (int e = 0; e < 2; ++e)
    for (int v = 0; v < 4; ++v)
      EXPECT_EQ(element_data[e][v],
                volume_mesh_W->element(VolumeElementIndex(e)).vertex(v));
  return volume_mesh_W;
}

// Test instantiation of VolumeMesh using `double` as the underlying scalar
// type.
GTEST_TEST(VolumeMeshTest, TestVolumeMeshDouble) {
  auto volume_mesh = TestVolumeMesh<double>();
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There is no test of
// differentiation.
GTEST_TEST(VolumeMeshTest, TestVolumeMeshAutoDiffXd) {
  auto volume_mesh = TestVolumeMesh<AutoDiffXd>();
}

template<typename T>
void TestVolumeMeshEqual() {
  const auto mesh = TestVolumeMesh<T>();

  // Mesh equals itself.
  EXPECT_TRUE(mesh->Equal(*mesh));

  // Mesh equals its copy.
  {
    const VolumeMesh<T> mesh_copy = *mesh;
    EXPECT_TRUE(mesh->Equal(mesh_copy));
  }

  // Different positions of vertices.
  {
    const auto mesh_translate =
        TestVolumeMesh<T>(RigidTransform<T>(Vector3<T>(1., 1., 1.)));
    EXPECT_FALSE(mesh->Equal(*mesh_translate));
  }

  // Different tetrahedral connectivity.
  {
    std::vector<VolumeVertex<T>> vertices_copy = mesh->vertices();
    std::vector<VolumeElement> tetrahedra = mesh->tetrahedra();
    // Re-order vertices of the first tetrahedron.
    tetrahedra[0] = VolumeElement(tetrahedra[0].vertex(1),
                                  tetrahedra[0].vertex(2),
                                  tetrahedra[0].vertex(0),
                                  tetrahedra[0].vertex(3));
    VolumeMesh<T> mesh_different_tetrahedra(std::move(tetrahedra),
                                             std::move(vertices_copy));
    EXPECT_FALSE(mesh->Equal(mesh_different_tetrahedra));
  }

  // Different number of vertices.
  {
    std::vector<VolumeVertex<T>> vertices = mesh->vertices();
    vertices.emplace_back(1.2, 3.7, 0.15);
    std::vector<VolumeElement> tetrahedra = mesh->tetrahedra();
    VolumeMesh<T> another_mesh(std::move(tetrahedra), std::move(vertices));
    EXPECT_FALSE(mesh->Equal(another_mesh));
  }

  // Different number of tetrahedra.
  {
    std::vector<VolumeVertex<T>> vertices = mesh->vertices();
    std::vector<VolumeElement> tetrahedra = mesh->tetrahedra();
    tetrahedra.pop_back();
    VolumeMesh<T> another_mesh(std::move(tetrahedra), std::move(vertices));
    EXPECT_FALSE(mesh->Equal(another_mesh));
  }
}

GTEST_TEST(VolumeMeshTest, TestEqualDouble) {
  TestVolumeMeshEqual<double>();
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles.
GTEST_TEST(VolumeMeshTest, TestEqualAutoDiffXd) {
  TestVolumeMeshEqual<AutoDiffXd>();
}

template<typename T>
void TestCalcTetrahedronVolume() {
  const RigidTransform<T> X_WM(
      RollPitchYaw<T>(M_PI / 6.0, 2.0 * M_PI / 3.0, 7.0 * M_PI / 4.0),
      Vector3<T>(1.0, 2.0, 3.0));
  auto volume_mesh = TestVolumeMesh<T>(X_WM);
  // Estimate 4 multiply+add, each introduces 2 epsilons.
  const double kTolerance(8.0 * std::numeric_limits<double>::epsilon());

  const double expect_tetrahedron_volume(1. / 6.);
  for (int e = 0; e < 2; ++e) {
    const double tetrahedron_volume = ExtractDoubleOrThrow(
        volume_mesh->CalcTetrahedronVolume(VolumeElementIndex(e)));
    EXPECT_NEAR(expect_tetrahedron_volume, tetrahedron_volume, kTolerance);
  }
}

GTEST_TEST(VolumeMeshTest, TestCalcTetrahedronVolumeDouble) {
  TestCalcTetrahedronVolume<double>();
}

GTEST_TEST(VolumeMeshTest, TestCalcTetrahedronVolumeAutoDiffXd) {
  TestCalcTetrahedronVolume<AutoDiffXd>();
}

template<typename T>
void TestCalcBarycentric() {
  const RigidTransform<T> X_WM(
      RollPitchYaw<T>(M_PI / 6.0, 2.0 * M_PI / 3.0, 7.0 * M_PI / 4.0),
      Vector3<T>(1.0, 2.0, 3.0));
  auto volume_mesh = TestVolumeMesh<T>(X_WM);
  // Empirically the std::numeric_limits<double>::epsilon() 2.2e-16 is too
  // small to account for the pose.
  const T kTolerance(1e-14);
  const VolumeElementIndex element(0);

  // At the centroid of the tetrahedral element v0v1v2v3.
  {
    Vector3<T> p_M(0.25, 0.25, 0.25);
    typename VolumeMesh<T>::Barycentric barycentric =
        volume_mesh->CalcBarycentric(X_WM * p_M, element);
    typename VolumeMesh<T>::Barycentric expect_barycentric(0.25, 0.25, 0.25,
                                                           0.25);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // At the centroid of the face v0v1v2.
  {
    Vector3<T> p_M(1. / 3., 1. / 3., 0.);
    typename VolumeMesh<T>::Barycentric barycentric =
        volume_mesh->CalcBarycentric(X_WM * p_M, element);
    typename VolumeMesh<T>::Barycentric expect_barycentric(1. / 3., 1. / 3.,
                                                           1. / 3., 0.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // At the middle of the edge v0v1.
  {
    Vector3<T> p_M(0.5, 0., 0.);
    typename VolumeMesh<T>::Barycentric barycentric =
        volume_mesh->CalcBarycentric(X_WM * p_M, element);
    typename VolumeMesh<T>::Barycentric expect_barycentric(0.5, 0.5, 0., 0.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // At the vertex v3.
  {
    Vector3<T> p_M(0., 0., 1.);
    typename VolumeMesh<T>::Barycentric barycentric =
        volume_mesh->CalcBarycentric(X_WM * p_M, element);
    typename VolumeMesh<T>::Barycentric expect_barycentric(0., 0., 0., 1.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
}

GTEST_TEST(VolumeMeshTest, TestCalcBarycentricDouble) {
  TestCalcBarycentric<double>();
}

GTEST_TEST(VolumeMeshTest, TestCalcBarycentricAutoDiffXd) {
  TestCalcBarycentric<AutoDiffXd>();
}

template<typename T>
void TestCalcGradBarycentric() {
  // The mesh M consists of one tetrahedral element whose vertices are at the
  // origin and on the coordinate axes of M's frame. The chosen vertex
  // coordinates avoid symmetry but are easy to calculate ∇bᵢ manually.
  //
  //                                 Z
  //                                 |
  //                                 v3
  //                                 |
  //                                 |
  //     v0_M = (0,0,0)              |
  //     v1_M = (1,0,0)              |   M's frame
  //     v2_M = (0,2,0)              |
  //     v3_M = (0,0,3)            v0+---+---v2-----Y
  //                                /
  //                              v1
  //                              /
  //                             /
  //                            X
  //
  const int tetrahedron[4] = {0, 1, 2, 3};
  const Vector3<T> v0_M(0., 0., 0.);
  const Vector3<T> v1_M(1., 0., 0.);
  const Vector3<T> v2_M(0., 2., 0.);
  const Vector3<T> v3_M(0., 0., 3.);

  // We use an arbitrary pose of the volume mesh M in World frame W to make the
  // test more realistic.
  const RigidTransform<T> X_WM(
      RollPitchYaw<T>(M_PI / 6.0, 2.0 * M_PI / 3.0, 5.0 * M_PI / 4.0),
      Vector3<T>(1.0, 2.0, 3.0));

  // Create the mesh with vertex coordinates expressed in World frame to make
  // the test more realistic.
  const VolumeMesh<T> mesh_W(
      {VolumeElement(tetrahedron)},
      {VolumeVertex<T>(X_WM * v0_M), VolumeVertex<T>(X_WM * v1_M),
       VolumeVertex<T>(X_WM * v2_M), VolumeVertex<T>(X_WM * v3_M)});

  const VolumeMeshTester<T> tester(mesh_W);
  const auto gradb0_W = tester.CalcGradBarycentric(VolumeElementIndex(0), 0);
  const auto gradb1_W = tester.CalcGradBarycentric(VolumeElementIndex(0), 1);
  const auto gradb2_W = tester.CalcGradBarycentric(VolumeElementIndex(0), 2);
  const auto gradb3_W = tester.CalcGradBarycentric(VolumeElementIndex(0), 3);

  // In this example, we have these equations, expressed in M's frame:
  //      b₀(x,y,z) = -x - y/2 - z/3 + 1,
  //      b₁(x,y,z) = x,
  //      b₂(x,y,z) = y/2,
  //      b₃(x,y,z) = z/3.
  // We can manually verify the equations by checking that bᵢ(vi) = 1 for all i,
  // bⱼ(vj) = 0 for all i ≠ j, and ∑bⱼ = 1.
  //     We can read off ∇bᵢ from the coefficients of x,y,z in the above
  // equations.
  const Vector3<T> expect_gradb0_M(-1., -1. / 2., -1. / 3.);
  const Vector3<T> expect_gradb1_M = Vector3<T>::UnitX();
  const Vector3<T> expect_gradb2_M = Vector3<T>::UnitY() / 2.;
  const Vector3<T> expect_gradb3_M = Vector3<T>::UnitZ() / 3.;

  const auto R_WM = X_WM.rotation();
  EXPECT_TRUE(CompareMatrices(R_WM * expect_gradb0_M, gradb0_W, 1e-14));
  EXPECT_TRUE(CompareMatrices(R_WM * expect_gradb1_M, gradb1_W, 1e-14));
  EXPECT_TRUE(CompareMatrices(R_WM * expect_gradb2_M, gradb2_W, 1e-14));
  EXPECT_TRUE(CompareMatrices(R_WM * expect_gradb3_M, gradb3_W, 1e-14));

  // Since ∑bᵢ = 1, the gradients have zero sum: ∑(∇bᵢ) = 0.
  EXPECT_TRUE(CompareMatrices(gradb0_W + gradb1_W + gradb2_W + gradb3_W,
                              Vector3<T>::Zero(), 1e-14));
}

GTEST_TEST(VolumeMeshTest, TestCalcGradBarycentricDouble) {
  TestCalcGradBarycentric<double>();
}

GTEST_TEST(VolumeMeshTest, TestCalcGradBarycentricAutoDiffXd) {
  TestCalcGradBarycentric<AutoDiffXd>();
}

template<typename T>
void TestCalcGradientVectorOfLinearField() {
  // CalcGradientVectorOfLinearField() is a weighted sum of
  // CalcGradBarycentric() which is already tested with non-symmetric vertex
  // coordinates and an arbitrary pose. Therefore, it suffices to test
  // CalcGradientVectorOfLinearField() in the mesh's frame M with symmetric
  // vertex coordinates but non-symmetric field values.
  //
  // The four vertices v0,v1,v2,v3 of the tetrahedral element e0 have
  // coordinates expressed in M's frame as:
  //
  //                                +Z
  //                                 |
  //     v0_M = (0,0,0), f₀ = 2      v3
  //     v1_M = (1,0,0), f₁ = 3      |   M's frame
  //     v2_M = (0,1,0), f₂ = 4      |
  //     v3_M = (0,0,1), f₃ = 5    v0+------v2---+Y
  //                                /
  //                               /
  //                             v1
  //                             /
  //                           +X
  //
  auto mesh_M = TestVolumeMesh<T>();
  const std::array<T, 4> f{2., 3., 4., 5.};

  const Vector3<T> gradf_M =
      mesh_M->CalcGradientVectorOfLinearField(f, VolumeElementIndex(0));

  // The field f on the tetrahedral element e0 satisfies this equation with
  // coordinates expressed in M's frame:
  //       f(x,y,z) = x + 2y + 3z + 2.
  // We can verify the equation manually by checking that f(vi_M) = fᵢ like
  // these:
  //       f(0,0,0) = 2
  //       f(1,0,0) = 3
  //       f(0,1,0) = 4
  //       f(0,0,1) = 5
  // Therefore, we can read off ∇f, expressed in M's frame, from the
  // coefficients of x,y,z in the equation.
  const Vector3<T> expect_gradf_M{1., 2., 3.};

  EXPECT_TRUE(CompareMatrices(expect_gradf_M, gradf_M, 1e-14));
}

GTEST_TEST(VolumeMeshTest, TestCalcGradientVectorOfLinearFieldDouble) {
  TestCalcGradientVectorOfLinearField<double>();
}

GTEST_TEST(VolumeMeshTest, TestCalcGradientVectorOfLinearFieldAutoDiffXd) {
  TestCalcGradientVectorOfLinearField<AutoDiffXd>();
}

template<typename T>
std::unique_ptr<VolumeMeshFieldLinear<T, T>> TestVolumeMeshFieldLinear() {
  auto volume_mesh = TestVolumeMesh<T>();

  // We give names to the values at vertices for testing later.
  const T f0{1.};
  const T f1{2.};
  const T f2{3.};
  const T f3{4.};
  const T f4{5.};
  std::vector<T> f_values = {f0, f1, f2, f3, f4};

  auto volume_mesh_field = std::make_unique<VolumeMeshFieldLinear<T, T>>(
      "pressure", std::move(f_values), volume_mesh.get());

  // Tests evaluation of the field on the element e0 {v0, v1, v2, v3}.
  const VolumeElementIndex e0(0);
  const typename VolumeMesh<T>::Barycentric b{0.4, 0.3, 0.2, 0.1};
  const T expect_p = b(0) * f0 + b(1) * f1 + b(2) * f2 + b(3) * f3;
  EXPECT_EQ(expect_p, volume_mesh_field->Evaluate(e0, b));

  // Tests the gradient vector ∇p on the element e0 {v0, v1, v2, v3}.
  // VolumeMeshFieldLinear stores ∇p for each element. We only check that
  // ∇p stored in VolumeMeshFieldLinear is the same as the gradient
  // calculated by VolumeMesh. The vectors are expressed in frame M of the mesh.
  Vector3<T> gradp_M = volume_mesh_field->EvaluateGradient(e0);
  Vector3<T> expect_gradp_M = volume_mesh->CalcGradientVectorOfLinearField(
      std::array<T, 4>{f0, f1, f2, f3}, e0);
  EXPECT_TRUE(CompareMatrices(expect_gradp_M, gradp_M, 1e-14));

  return volume_mesh_field;
}

// Test instantiation of VolumeMeshField using `double` as the underlying
// scalar type.
GTEST_TEST(VolumeMeshFieldTest, TestVolumeMeshFieldLinearDouble) {
  auto volume_mesh_field = TestVolumeMeshFieldLinear<double>();
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There are no tests of
// differentiation.
GTEST_TEST(VolumeMeshFieldTest, TestVolumeMeshFieldLinearAutoDiffXd) {
  auto volume_mesh_field = TestVolumeMeshFieldLinear<AutoDiffXd>();
}

}  // namespace
}  // namespace geometry
}  // namespace drake
