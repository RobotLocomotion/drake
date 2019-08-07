#include "drake/geometry/proximity/volume_mesh.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"
#include "drake/geometry/proximity/mesh_field_linear.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

// Test instantiation of VolumeMesh of a geometry M and inspecting its
// components. By default, the vertex positions are expressed in M's frame.
// The optional parameter X_WM will change the vertex positions to W's frame.
template <typename T>
std::unique_ptr<VolumeMesh<T>> TestVolumeMesh(
    const math::RigidTransform<T> X_WM = math::RigidTransform<T>::Identity()) {
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

template <typename T>
void TestCalcTetrahedronVolume() {
  const math::RigidTransform<T> X_WM(
      math::RollPitchYaw<T>(M_PI / 6.0, 2.0 * M_PI / 3.0, 7.0 * M_PI / 4.0),
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

template <typename T>
void TestCalcBarycentric() {
  const math::RigidTransform<T> X_WM(
      math::RollPitchYaw<T>(M_PI / 6.0, 2.0 * M_PI / 3.0, 7.0 * M_PI / 4.0),
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
        volume_mesh->CalcBarycentric(X_WM *p_M, element);
    typename VolumeMesh<T>::Barycentric expect_barycentric(0.25, 0.25, 0.25,
                                                           0.25);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // At the centroid of the face v0v1v2.
  {
    Vector3<T> p_M(1./3., 1./3., 0.);
    typename VolumeMesh<T>::Barycentric barycentric =
        volume_mesh->CalcBarycentric(X_WM *p_M, element);
    typename VolumeMesh<T>::Barycentric expect_barycentric(1. / 3., 1. / 3.,
                                                           1. / 3., 0.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // At the middle of the edge v0v1.
  {
    Vector3<T> p_M(0.5, 0., 0.);
    typename VolumeMesh<T>::Barycentric barycentric =
        volume_mesh->CalcBarycentric(X_WM *p_M, element);
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

GTEST_TEST(VolumeMeshTest, TestCalcBarycentricAutoffXd) {
  TestCalcBarycentric<AutoDiffXd>();
}

// TODO(SeanCurtis-TRI): Remove the forward declaration in place of simply
//  defining the method here.
// Tests instantiation of VolumeMeshField and evaluating a scalar field value.
template <typename T>
std::unique_ptr<VolumeMeshFieldLinear<T, T>> TestVolumeMeshField();

// Test instantiation of VolumeMeshField using `double` as the underlying
// scalar type.
GTEST_TEST(VolumeMeshFieldTest, TestVolumeMeshFieldDouble) {
  auto volume_mesh_field = TestVolumeMeshField<double>();
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There are no tests of
// differentiation.
GTEST_TEST(VolumeMeshFieldTest, TestVolumeMeshFieldAutoDiffXd) {
  auto volume_mesh_field = TestVolumeMeshField<AutoDiffXd>();
}

template <typename T>
std::unique_ptr<VolumeMeshFieldLinear<T, T>> TestVolumeMeshField() {
  auto volume_mesh = TestVolumeMesh<T>();

  // We give names to the values at vertices for testing later.
  const T p0{1.};
  const T p1{2.};
  const T p2{3.};
  const T p3{4.};
  const T p4{5.};
  std::vector<T> p_values = {p0, p1, p2, p3, p4};

  auto volume_mesh_field = std::make_unique<VolumeMeshFieldLinear<T, T>>(
      "pressure", std::move(p_values), volume_mesh.get());

  // Tests evaluation of the field on the element e0 {v0, v1, v2, v3}.
  const VolumeElementIndex e0(0);
  const typename VolumeMesh<T>::Barycentric b{0.4, 0.3, 0.2, 0.1};
  const T expect_p = b(0) * p0 + b(1) * p1 + b(2) * p2 + b(3) * p3;
  EXPECT_EQ(expect_p, volume_mesh_field->Evaluate(e0, b));

  return volume_mesh_field;
}

}  // namespace geometry
}  // namespace drake
