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

template <class T, class MeshType>
class MeshFieldLinearTester {
 public:
  explicit MeshFieldLinearTester(MeshFieldLinear<T, MeshType>* field)
      : field_(*field) {}
  void CalcGradientField() {
    field_.CalcGradientField();
  }
  void CalcValueAtMeshOriginForAllElements() {
    field_.CalcValueAtMeshOriginForAllElements();
  }
  T CalcValueAtMeshOrigin(typename MeshType::ElementIndex e) const {
    return field_.CalcValueAtMeshOrigin(e);
  }

 private:
  MeshFieldLinear<T, MeshType>& field_;
};

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

GTEST_TEST(MeshFieldLinearTest, TestCalcValueAtMeshOrigin) {
  const VolumeMesh<double> mesh_M =
      internal::MakeBoxVolumeMesh<double>(Box(0.5, 1.5, 2), 0.25);

  // Use one linear function f for the entire mesh for simplicity.
  auto f = [](const Vector3d& p_MQ) -> double {
    return 3.5 * p_MQ.x() - 2.7 * p_MQ.y() + 0.7 * p_MQ.z() + 1.23;
  };
  std::vector<double> values;
  for (const VolumeVertex<double> v : mesh_M.vertices()) {
    values.push_back(f(v.r_MV()));
  }
  MeshFieldLinear<double, VolumeMesh<double>> field("f", std::move(values),
                                                    &mesh_M);
  MeshFieldLinearTester<double, VolumeMesh<double>> tester(&field);

  // Testing one representative tetrahedral element would have been adequate,
  // but we test all elements for completion.
  const double f_at_Mo = f(Vector3d::Zero());
  for (VolumeElementIndex e(0); e < mesh_M.num_elements(); ++e) {
    // The tolerance 1e-14 is empirically determined. It is related to
    // gradient calculation and conversion between Cartesian coordinates and
    // barycentric coordinates. We might need a larger tolerance if we use a
    // larger geometry.
    EXPECT_NEAR(f_at_Mo, tester.CalcValueAtMeshOrigin(e), 1e-14);
  }
}

GTEST_TEST(MeshFieldLinearTest, TestCalcValueAtMeshOriginForAllElements) {
  const VolumeMesh<double> mesh_M =
      internal::MakeBoxVolumeMesh<double>(Box(0.5, 1.5, 2), 0.25);

  // Use one linear function f for the entire mesh for simplicity.
  auto f = [](const Vector3d& p_MQ) -> double {
    return 3.5 * p_MQ.x() - 2.7 * p_MQ.y() + 0.7 * p_MQ.z() + 1.23;
  };
  std::vector<double> values;
  for (const VolumeVertex<double> v : mesh_M.vertices()) {
    values.push_back(f(v.r_MV()));
  }
  // First we construct the field without gradient.
  MeshFieldLinear<double, VolumeMesh<double>> field(
      "f", std::move(values), &mesh_M, false /* no calcultion of gradient */);
  MeshFieldLinearTester<double, VolumeMesh<double>> tester(&field);

  tester.CalcGradientField();
  tester.CalcValueAtMeshOriginForAllElements();

  Vector3d p_MQ(1.2, 2.3, 3.4);
  double f_at_Q = f(p_MQ);
  for (VolumeElementIndex e(0); e < mesh_M.num_elements(); ++e) {
    // The tolerance 1e-13 is empirically determined. It is related to
    // gradient calculation and conversion between Cartesian coordinates and
    // barycentric coordinates. We might need a larger tolerance if we use a
    // larger geometry.
    //     We use EvaluateCartesian() as an indicator that
    // CalcValueAtMeshOriginForAllElements() did the right job.
    //     Notice that p_MQ is outside the box geometry, so it is outside every
    // tetrahedral element. There are applications that need to evaluate the
    // linear function of a tetrahedral element at a point outside the
    // tetrahedron.
    EXPECT_NEAR(f_at_Q, field.EvaluateCartesian(e, p_MQ), 1e-13);
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
