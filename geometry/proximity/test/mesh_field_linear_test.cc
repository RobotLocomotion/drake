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

// Confirms that invoking EvaluateCartesian() produces equivalent expected
// values whether gradients have been pre-computed or not. Also characterizes
// the differences in accuracy between the two modes.
GTEST_TEST(MeshFieldLinearTest, EvaluateCartesianWithAndWithoutGradient) {
  // This mesh is symmetric with respect to the x=0 plane in frame M, so we
  // can use it to define the field abs(x) accurately. No edges of
  // tetrahedra cross the x=0 plane.
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
    // If, in the future, the implementation of MakeBoxVolumeMesh changes so
    // that the mesh generated here no longer has strictly positive/negative
    // tetrahedra, the mesh used for this test will need to be changed to one
    // that does satisfy that requirement.
    ASSERT_TRUE(num_positive_or_zero == 4 || num_negative_or_zero == 4)
        << "Mesh no longer satisfies the requirements for this test";
    // Assume no tetrahedron has both num_positive_or_zero == 4 and
    // num_negative_or_zero == 4. It would be a zero-volume tetrahedron with
    // all four vertices lying on the x=0 plane.
    ASSERT_FALSE(num_positive_or_zero == 4 && num_negative_or_zero == 4)
        << "Mesh no longer satisfies the requirements for this test";
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
  std::vector<double> values_copy = values;

  const MeshFieldLinear<double, VolumeMesh<double>> field_without_gradient(
      "abs(x)", std::move(values_copy), &mesh_M, false);
  const MeshFieldLinear<double, VolumeMesh<double>> field_with_gradient(
      "abs(x)", std::move(values), &mesh_M, true);

  ASSERT_THROW(field_without_gradient.EvaluateGradient(VolumeElementIndex(0)),
               std::runtime_error);
  ASSERT_NO_THROW(field_with_gradient.EvaluateGradient(VolumeElementIndex(0)));

  // Generally, evaluating the field without gradients produces more error than
  // with gradients. This is largely attributable to the transformations to
  // barycentric coordinates required in the absence of the gradient. The error
  // tends to scale with the distance between the query popint Q and the tet.
  // For this field and this mesh, the gradients seems to perfectly reproduce
  // the field. This may not generally be the case.

  {
    // Evaluating the field for points within tetrahedra.
    using Barycentric = VolumeMesh<double>::Barycentric;
    constexpr double kEps = std::numeric_limits<double>::epsilon();
    for (const Barycentric& b_Q :
        {Barycentric{0.25, 0.25, 0.25, 0.25} /* centroid */,
         Barycentric{0.5, 0.5, 0, 0} /* on edge */,
         Barycentric{0.49999, 0.49999, 1e-5, 1e-5} /* near edge */}) {
      // TODO(SeanCurtis-TRI): it's ridiculous that we don't have a mesh method
      //  that turns (element index, barycentric) --> cartesian.
      for (VolumeElementIndex e(0); e < mesh_M.num_elements(); ++e) {
        Vector3d p_MQ{0, 0, 0};
        for (int i = 0; i < 4; ++i) {
          p_MQ += mesh_M.vertex(mesh_M.element(e).vertex(i)).r_MV() * b_Q(i);
        }
        EXPECT_NEAR(abs(p_MQ.x()),
                    field_without_gradient.EvaluateCartesian(e, p_MQ), kEps);
        EXPECT_NEAR(abs(p_MQ.x()),
                    field_with_gradient.EvaluateCartesian(e, p_MQ), 0);
      }
    }
  }

  // Evaluating the field for points that don't necessarily lie within the tet.
  for (const Vector3d& p_MQ :
      {Vector3d{1e-15, 1e-15, 1e-15}, Vector3d{0.1, 0.2, 0.3},
       Vector3d{-10.23, 27, 77}, Vector3d{321.3, -843.2, 202.02}}) {
    const double kEps = 4 * std::numeric_limits<double>::epsilon() *
        std::max(1.0, std::abs(p_MQ.cwiseAbs().maxCoeff()));
    for (VolumeElementIndex e(0); e < mesh_M.num_elements(); ++e) {
      if (positive_set.count(e) == 1) {
        // f(x,y,z) = x
        EXPECT_NEAR(p_MQ.x(), field_without_gradient.EvaluateCartesian(e, p_MQ),
                    kEps);
        EXPECT_EQ(p_MQ.x(), field_with_gradient.EvaluateCartesian(e, p_MQ));
      } else {
        // f(x,y,z) = -x
        EXPECT_NEAR(-p_MQ.x(),
                    field_without_gradient.EvaluateCartesian(e, p_MQ), kEps);
        EXPECT_EQ(-p_MQ.x(), field_with_gradient.EvaluateCartesian(e, p_MQ));
      }
    }
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
