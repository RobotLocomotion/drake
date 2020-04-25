#include "drake/geometry/proximity/mesh_field_linear.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace geometry {
namespace {

using math::RigidTransformd;
using math::RollPitchYawd;
using Eigen::Vector3d;

template <typename T>
std::unique_ptr<SurfaceMesh<T>> GenerateMesh() {
// A simple surface mesh consists of two right triangles that make a square.
//
//   y
//   |
//   |
//   |
//   v3(0,1,0)  v2(1,1,0)
//   +-----------+
//   |         . |
//   |  f1  . .  |
//   |    . .    |
//   | . .   f0  |
//   |.          |
//   +-----------+---------- x
//   v0(0,0,0)  v1(1,0,0)
//
  const int face_data[2][3] = {{0, 1, 2}, {2, 3, 0}};
  std::vector<SurfaceFace> faces;
  for (int f = 0; f < 2; ++f) faces.emplace_back(face_data[f]);
  const Vector3<T> vertex_data[4] = {
      {0., 0., 0.}, {1., 0., 0.}, {1., 1., 0.}, {0., 1., 0.}};
  std::vector<SurfaceVertex<T>> vertices;
  for (int v = 0; v < 4; ++v) vertices.emplace_back(vertex_data[v]);
  auto surface_mesh =
      std::make_unique<SurfaceMesh<T>>(move(faces), std::move(vertices));
  return surface_mesh;
}

// Tests Evaluate(VertexIndex).
GTEST_TEST(MeshFieldLinearTest, EvaluateAtVertex) {
  auto mesh = GenerateMesh<double>();
  std::vector<double> e_values = {0., 1., 2., 3.};
  auto mesh_field =
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e", std::move(e_values), mesh.get());
  EXPECT_EQ(mesh_field->EvaluateAtVertex(SurfaceVertexIndex(0)), 0);
  EXPECT_EQ(mesh_field->EvaluateAtVertex(SurfaceVertexIndex(1)), 1);
  EXPECT_EQ(mesh_field->EvaluateAtVertex(SurfaceVertexIndex(2)), 2);
  EXPECT_EQ(mesh_field->EvaluateAtVertex(SurfaceVertexIndex(3)), 3);
}

// Tests CloneAndSetMesh(). We use `double` and SurfaceMesh<double> as
// representative arguments for type parameters.
GTEST_TEST(MeshFieldLinearTest, TestDoCloneWithMesh) {
  using FieldValue = double;
  using MeshType = SurfaceMesh<double>;
  using MeshFieldLineard = MeshFieldLinear<FieldValue, MeshType>;

  auto mesh1 = GenerateMesh<double>();
  std::vector<FieldValue> e_values = {0., 1., 2., 3.};
  auto original =
      std::make_unique<MeshFieldLineard>("e", std::move(e_values), mesh1.get());

  MeshType mesh2 = *mesh1;
  auto clone_base = original->CloneAndSetMesh(&mesh2);
  auto clone = dynamic_cast<MeshFieldLineard*>(clone_base.get());

  // Check uniqueness.
  EXPECT_NE(original.get(), clone);

  // Check equivalence.
  EXPECT_EQ(original->name(), clone->name());
  EXPECT_EQ(original->values(), clone->values());
}

// Tests Equal.
GTEST_TEST(MeshFieldLinearTest, TestEqual) {
  auto mesh = GenerateMesh<double>();
  std::vector<double> e_values = {0., 1., 2., 3.};
  auto mesh_field =
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e", std::move(e_values), mesh.get());

  // Same field.
  auto field0 = mesh_field->CloneAndSetMesh(mesh.get());
  EXPECT_TRUE(mesh_field->Equal(*field0));

  // Different mesh.
  SurfaceMesh<double> alt_mesh = *mesh;
  alt_mesh.ReverseFaceWinding();
  auto field1 = mesh_field->CloneAndSetMesh(&alt_mesh);
  EXPECT_FALSE(mesh_field->Equal(*field1));

  // Different e values.
  std::vector<double> alt_e_values = {3., 2., 1., 0.};
  auto field2 = MeshFieldLinear<double, SurfaceMesh<double>>(
      "e", std::move(alt_e_values), mesh.get());
  EXPECT_FALSE(mesh_field->Equal(field2));
}

// EvaluateGradient() only looks up the std::vector<Vector3<>> gradients_;.
// The actual gradient calculation is in VolumeMesh and SurfaceMesh, and
// we test the calculation there.
GTEST_TEST(MeshFieldLinearTest, TestEvaluateGradient) {
  auto mesh = GenerateMesh<double>();
  std::vector<double> e_values = {0., 1., 2., 3.};
  auto mesh_field =
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e", std::move(e_values), mesh.get());

  Vector3d gradient = mesh_field->EvaluateGradient(SurfaceFaceIndex(0));

  Vector3d expect_gradient(1., 1., 0.);
  EXPECT_TRUE(CompareMatrices(expect_gradient, gradient, 1e-14));
}

// Tests that EvaluateGradient() `throw` if the gradient vector was not
// calculated.
GTEST_TEST(MeshFieldLinearTest, TestEvaluateGradientThrow) {
  auto mesh = GenerateMesh<double>();
  std::vector<double> e_values = {0., 1., 2., 3.};
  const bool calculate_gradient = false;
  auto mesh_field =
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e", std::move(e_values), mesh.get(), calculate_gradient);

  EXPECT_THROW(mesh_field->EvaluateGradient(SurfaceFaceIndex(0)),
               std::runtime_error);
}

GTEST_TEST(MeshFieldLinearTest, TestTransformGradients) {
  // Create mesh and field. Both mesh vertices and field gradient are expressed
  // in frame M.
  auto mesh = GenerateMesh<double>();
  std::vector<double> e_values = {0., 1., 2., 3.};
  auto mesh_field =
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e", std::move(e_values), mesh.get());

  // We will not check all gradient vectors. Instead we will use the gradient
  // vector of the first triangle as a representative.
  Vector3d gradient_M = mesh_field->EvaluateGradient(SurfaceFaceIndex(0));

  // Confirm the gradient vector before rigid transform.
  Vector3d expect_gradient_M(1., 1., 0.);
  ASSERT_TRUE(CompareMatrices(expect_gradient_M, gradient_M, 1e-14));

  RigidTransformd X_MN(RollPitchYawd(M_PI_2, M_PI_4, M_PI / 6.),
                       Vector3d(1.2, 1.3, -4.3));
  mesh->TransformVertices(X_MN);
  mesh_field->TransformGradients(X_MN);

  Vector3d gradient_N = mesh_field->EvaluateGradient(SurfaceFaceIndex(0));
  Vector3d expect_gradient_N = X_MN.rotation() * expect_gradient_M;

  EXPECT_TRUE(CompareMatrices(expect_gradient_N, gradient_N, 1e-14));
}

// Tests EvaluateCartesian() of MeshFieldLinear that has calculated gradients
// and that has not. Evaluates the field values at various points inside and
// outside tetrahedral elements including the origin and a point outside the
// mesh. We pick the piecewise-linear field abs(x) as the ground truth, so that
// some tetrahedra have f(x,y,z)=x, and the others have f(x,y,z)=-x.
GTEST_TEST(MeshFieldLinearTest, EvaluateCartesianWithAndWithoutGradient) {
  // This mesh is symmetric with respect to the x=0 plane in frame M, so we
  // can use it to define the field abs(x) accurately.
  const VolumeMesh<double> mesh_M =
      internal::MakeBoxVolumeMesh<double>(Box(0.5, 1.5, 2), 0.125);

  // Verify the symmetry and classify tetrahedra. Each tetrahedron is either
  // in the x ≥ 0 half-space or the x ≤ 0 half-space. Here we call the former
  // "positive tetrahedron" and the latter "negative tetrahedron".
  // No tetrahedron crosses the x = 0 plane in frame M. Obviously we assume
  // the tetrahedra fill the volume of the box.
  std::set<VolumeElementIndex> positive_set;  // set of positive tetrahedra.
  for (VolumeElementIndex e(0); e < mesh_M.num_elements(); ++e) {
    int num_positive_or_zero = 0;
    int num_negative_or_zero = 0;
    for (int i = 0; i < 4; ++i) {
      const Vector3d p_MV = mesh_M.vertex(mesh_M.element(e).vertex(i)).r_MV();
      if (p_MV.x() >= 0) ++num_positive_or_zero;
      if (p_MV.x() <= 0) ++num_negative_or_zero;
    }
    // If in the future we change the mesh, this assertion might fail. You
    // might want to use a different mesh that has x=0 symmetry.
    ASSERT_TRUE(num_positive_or_zero == 4 || num_negative_or_zero == 4);
    // Assume no tetrahedron has both num_positive_or_zero == 4 and
    // num_negative_or_zero == 4. It would be a zero-volume tetrahedron with
    // all four vertices lying on the x=0 plane.
    ASSERT_FALSE(num_positive_or_zero == 4 && num_negative_or_zero == 4);
    if (num_positive_or_zero == 4) positive_set.insert(e);
  }

  // Sampling abs(x) function. For tetrahedral elements in the x ≥ 0 half-space,
  // they will represent f(x) = x. For tetrahedral elements in the x ≤ 0
  // half-space, they will represent f(x) = -x.
  using std::abs;
  std::vector<double> values;
  for (const VolumeVertex<double> v : mesh_M.vertices()) {
    values.push_back(abs(v.r_MV().x()));
  }
  std::vector<double> values_too = values;

  const MeshFieldLinear<double, VolumeMesh<double>> field_without_gradient(
      "abs(x)", std::move(values_too), &mesh_M, false);
  const MeshFieldLinear<double, VolumeMesh<double>> field_with_gradient(
      "abs(x)", std::move(values), &mesh_M, true);

  ASSERT_THROW(field_without_gradient.EvaluateGradient(VolumeElementIndex(0)),
               std::runtime_error);
  ASSERT_NO_THROW(field_with_gradient.EvaluateGradient(VolumeElementIndex(0)));

  // Empirically the remaining tests suggest that field_with_gradient is more
  // accurate probably because the gradient vectors (1,0,0) for f(x,y,z)=x and
  // (-1,0,0) for f(x,y,z)=-x are represented accurately. The
  // field_without_gradient has to solve a linear system to convert Cartesian
  // coordinates to barycentric coordinates.

  // Evaluate field value at a point inside each tetrahedron.
  // Pick the centroid of each tetrahedral element.
  for (VolumeElementIndex e(0); e < mesh_M.num_elements(); ++e) {
    const Vector3d p_MV0 = mesh_M.vertex(mesh_M.element(e).vertex(0)).r_MV();
    const Vector3d p_MV1 = mesh_M.vertex(mesh_M.element(e).vertex(1)).r_MV();
    const Vector3d p_MV2 = mesh_M.vertex(mesh_M.element(e).vertex(2)).r_MV();
    const Vector3d p_MV3 = mesh_M.vertex(mesh_M.element(e).vertex(3)).r_MV();
    const Vector3d p_MC = (p_MV0 + p_MV1 + p_MV2 + p_MV3) / 4.0;
    EXPECT_NEAR(abs(p_MC.x()),
                field_without_gradient.EvaluateCartesian(e, p_MC), 0.0);
    EXPECT_NEAR(abs(p_MC.x()), field_with_gradient.EvaluateCartesian(e, p_MC),
                0.0);
  }

  // Evaluate field value at origin, which is inside some tetrahedra and
  // outside others. The tolerance is zero in this case.
  {
    const Vector3d p_Mo(0, 0, 0);
    for (VolumeElementIndex e(0); e < mesh_M.num_elements(); ++e) {
      EXPECT_NEAR(0.0, field_without_gradient.EvaluateCartesian(e, p_Mo), 0.0);
      EXPECT_NEAR(0.0, field_with_gradient.EvaluateCartesian(e, p_Mo), 0.0);
    }
  }

  // Evaluate field value at a generic point in the box. It is inside some
  // tetrahedra and outside others. Empirically the tolerance for
  // field_without_gradient is 1e-15, and the tolerance for
  // field_with_gradient is 0.0.
  {
    const Vector3d p_MQ(0.1, 0.2, 0.3);
    for (VolumeElementIndex e(0); e < mesh_M.num_elements(); ++e) {
      if (positive_set.count(e) == 1) {
        // f(x,y,z) = x
        EXPECT_NEAR(0.1, field_without_gradient.EvaluateCartesian(e, p_MQ),
                    1e-15);
        EXPECT_NEAR(0.1, field_with_gradient.EvaluateCartesian(e, p_MQ), 0.0);
      } else {
        // f(x,y,z) = -x
        EXPECT_NEAR(-0.1, field_without_gradient.EvaluateCartesian(e, p_MQ),
                    1e-15);
        EXPECT_NEAR(-0.1, field_with_gradient.EvaluateCartesian(e, p_MQ), 0.0);
      }
    }
  }

  // Evaluate field value at a point outside the box.
  // The tolerance 1e-13 for field_without_gradient is empirically determined.
  // It is larger than the previous block probably because larger coordinates
  // introduce larger errors. The tolerance 0.0 for field_with_gradient is
  // empirically determined.
  {
    const Vector3d p_MQ(-10.23, 27, 77);
    for (VolumeElementIndex e(0); e < mesh_M.num_elements(); ++e) {
      if (positive_set.count(e) == 1) {
        // f(x,y,z) = x, f(-10.23, 27, 77) = -10.23.
        EXPECT_NEAR(-10.23, field_without_gradient.EvaluateCartesian(e, p_MQ),
                    1e-13);
        EXPECT_NEAR(-10.23, field_with_gradient.EvaluateCartesian(e, p_MQ),
                    0.0);
      } else {
        // f(x,y,z) = -x, f(-10.23, 27, 77) = 10.23.
        EXPECT_NEAR(10.23, field_without_gradient.EvaluateCartesian(e, p_MQ),
                    1e-13);
        EXPECT_NEAR(10.23, field_with_gradient.EvaluateCartesian(e, p_MQ), 0.0);
      }
    }
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
