#include "drake/geometry/proximity/mesh_field_linear.h"

#include <cmath>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/math/autodiff.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace geometry {
namespace {

using math::RigidTransformd;
using math::RollPitchYawd;
using Eigen::Vector3d;

template <typename T>
std::unique_ptr<TriangleSurfaceMesh<T>> GenerateMesh() {
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
  std::vector<SurfaceTriangle> faces;
  for (int f = 0; f < 2; ++f) faces.emplace_back(face_data[f]);
  const Vector3<T> vertex_data[4] = {
      {0., 0., 0.}, {1., 0., 0.}, {1., 1., 0.}, {0., 1., 0.}};
  std::vector<Vector3<T>> vertices;
  for (int v = 0; v < 4; ++v) vertices.emplace_back(vertex_data[v]);
  auto surface_mesh = std::make_unique<TriangleSurfaceMesh<T>>(
      move(faces), std::move(vertices));
  return surface_mesh;
}

GTEST_TEST(MeshFieldLinearTest, EvaluateAtVertex) {
  auto mesh = GenerateMesh<double>();
  std::vector<double> e_values = {0., 1., 2., 3.};
  auto mesh_field =
      std::make_unique<MeshFieldLinear<double, TriangleSurfaceMesh<double>>>(
          std::move(e_values), mesh.get());
  EXPECT_EQ(mesh_field->EvaluateAtVertex(0), 0);
  EXPECT_EQ(mesh_field->EvaluateAtVertex(1), 1);
  EXPECT_EQ(mesh_field->EvaluateAtVertex(2), 2);
  EXPECT_EQ(mesh_field->EvaluateAtVertex(3), 3);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
GTEST_TEST(MeshFieldLinearTest, DeprecatedEvaluateAtVertex) {
  auto mesh = GenerateMesh<double>();
  std::vector<double> e_values = {0., 1., 2., 3.};
  auto mesh_field =
      std::make_unique<MeshFieldLinear<double, TriangleSurfaceMesh<double>>>(
          "my_name", std::move(e_values), mesh.get());
  EXPECT_EQ(mesh_field->name(), "my_name");
}
#pragma GCC diagnostic pop

// Tests CloneAndSetMesh(). We use `double` and TriangleSurfaceMesh<double> as
// representative arguments for type parameters.
GTEST_TEST(MeshFieldLinearTest, TestDoCloneWithMesh) {
  using FieldValue = double;
  using MeshType = TriangleSurfaceMesh<double>;
  using MeshFieldLineard = MeshFieldLinear<FieldValue, MeshType>;

  auto mesh1 = GenerateMesh<double>();
  std::vector<FieldValue> e_values = {0., 1., 2., 3.};
  MeshFieldLineard original(std::move(e_values), mesh1.get());

  MeshType mesh2 = *mesh1;
  std::unique_ptr<MeshFieldLineard> clone = original.CloneAndSetMesh(&mesh2);

  // Check uniqueness.
  EXPECT_NE(&original, clone.get());
  EXPECT_EQ(&mesh2, &clone->mesh());
  EXPECT_NE(&original.mesh(), &clone->mesh());

  // Check equivalence.
  EXPECT_EQ(original.values(), clone->values());
  // TODO(SeanCurtis-TRI) We haven't tested gradients_ or values_at_Mo_.
}

// Tests Equal.
GTEST_TEST(MeshFieldLinearTest, TestEqual) {
  auto mesh = GenerateMesh<double>();
  std::vector<double> e_values = {0., 1., 2., 3.};
  auto mesh_field =
      std::make_unique<MeshFieldLinear<double, TriangleSurfaceMesh<double>>>(
          std::move(e_values), mesh.get());

  // Same field.
  auto field0 = mesh_field->CloneAndSetMesh(mesh.get());
  EXPECT_TRUE(mesh_field->Equal(*field0));

  // Different mesh.
  TriangleSurfaceMesh<double> alt_mesh = *mesh;
  alt_mesh.ReverseFaceWinding();
  auto field1 = mesh_field->CloneAndSetMesh(&alt_mesh);
  EXPECT_FALSE(mesh_field->Equal(*field1));

  // Different e values.
  std::vector<double> alt_e_values = {3., 2., 1., 0.};
  auto field2 = MeshFieldLinear<double, TriangleSurfaceMesh<double>>(
      std::move(alt_e_values), mesh.get());
  EXPECT_FALSE(mesh_field->Equal(field2));
}

// EvaluateGradient() only looks up the std::vector<Vector3<>> gradients_;.
// The actual gradient calculation is in VolumeMesh and TriangleSurfaceMesh, and
// we test the calculation there.
GTEST_TEST(MeshFieldLinearTest, TestEvaluateGradient) {
  auto mesh = GenerateMesh<double>();
  std::vector<double> e_values = {0., 1., 2., 3.};
  auto mesh_field =
      std::make_unique<MeshFieldLinear<double, TriangleSurfaceMesh<double>>>(
          std::move(e_values), mesh.get());

  Vector3d gradient = mesh_field->EvaluateGradient(0);

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
      std::make_unique<MeshFieldLinear<double, TriangleSurfaceMesh<double>>>(
          std::move(e_values), mesh.get(), calculate_gradient);

  EXPECT_THROW(mesh_field->EvaluateGradient(0),
               std::runtime_error);
}

// Tests the transformation of the field when calling Transform(). As
// documented, there are *two* APIs that depend on the field's frame:
// EvaluateGradient() and EvaluateCartesian(). We test both to confirm that
// the transformation is complete.
GTEST_TEST(MeshFieldLinearTest, TestTransform) {
  // Create mesh and field. Both mesh vertices and field gradient are expressed
  // in frame M.
  auto mesh_M = GenerateMesh<double>();
  std::vector<double> e_values = {0., 1., 2., 3.};
  MeshFieldLinear<double, TriangleSurfaceMesh<double>> mesh_field_M(
      std::move(e_values), mesh_M.get(), true /* calc_gradients */);

  RigidTransformd X_NM(RollPitchYawd(M_PI_2, M_PI_4, M_PI / 6.),
                       Vector3d(1.2, 1.3, -4.3));
  MeshFieldLinear<double, TriangleSurfaceMesh<double>> mesh_field_N(
      mesh_field_M);
  // NOTE: re-expressing the field like this (and subsequent calls to
  // EvaluateCartesian()) don't actually require the field's captured mesh to
  // have properly transformed vertex positions. That is not generally true and
  // this should be considered an anti-pattern. Generally, the mesh and field
  // should be transformed in tandem. But it is expedient for this test to
  // ignore that.
  mesh_field_N.Transform(X_NM);

  for (int f = 0; f < mesh_M->num_triangles(); ++f) {
    // Successful invocation of EvaluateGradient() is proof that the gradients
    // have been computed.
    const Vector3d& gradient_M = mesh_field_M.EvaluateGradient(f);
    const Vector3d& gradient_N = mesh_field_N.EvaluateGradient(f);
    // The gradients are different, but related by rotation.
    ASSERT_FALSE(CompareMatrices(gradient_M, gradient_N, 1e-2));
    ASSERT_TRUE(
        CompareMatrices(X_NM.rotation() * gradient_M, gradient_N, 1e-15));
    // Pick an arbitrary point within the triangle.
    const Vector3d p_MQ =
        mesh_M->CalcCartesianFromBarycentric(f, Vector3d{0.1, 0.3, 0.6});
    const Vector3d p_NQ = X_NM * p_MQ;
    ASSERT_NEAR(mesh_field_M.EvaluateCartesian(f, p_MQ),
                mesh_field_N.EvaluateCartesian(f, p_NQ), 2e-15);
  }
}

// Confirms that invoking EvaluateCartesian() produces equivalent expected
// values whether gradients have been pre-computed or not.
GTEST_TEST(MeshFieldLinearTest, EvaluateCartesianWithAndWithoutGradient) {
  // This mesh is symmetric with respect to the x=0 plane in frame M, so we
  // can use it to define the testing field (defined below) accurately. No
  // edges of tetrahedra cross the x=0 plane.
  const VolumeMesh<double> mesh_M =
      internal::MakeBoxVolumeMesh<double>(Box(0.5, 1.5, 2), 0.125);

  // Verify the symmetry and classify tetrahedra. Each tetrahedron is either
  // in the x ≥ 0 half-space or the x ≤ 0 half-space. Here we call the former
  // "positive tetrahedron" and the latter "negative tetrahedron".
  // No tetrahedron crosses the x=0 plane in frame M. Obviously we assume
  // the tetrahedra fill the volume of the box.
  std::set<int> positive_set;  // set of positive tetrahedra.
  for (int e = 0; e < mesh_M.num_elements(); ++e) {
    int num_positive_or_zero = 0;
    int num_negative_or_zero = 0;
    for (int i = 0; i < 4; ++i) {
      const Vector3d p_MV = mesh_M.vertex(mesh_M.element(e).vertex(i));
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

  // We will test with this piecweise-linear function:
  //          f(x,y,z) = 3.5|x| - 2.7y + 0.7z + 1.23.
  // For tetrahedral elements in the x ≥ 0 half-space, they will represent
  //          f⁺(x,y,z) =  3.5x - 2.7y + 0.7z + 1.23.
  // For tetrahedral elements in the x ≤ 0 half-space, they will represent
  //          f⁻(x,y,z) = -3.5x - 2.7y + 0.7z + 1.23.
  // On the x=0 plane, the three functions agree:
  //          f(0,y,z) = f⁺(0,y,z) = f⁻(0,y,z),
  // and that is why we need the mesh with the above property.
  using std::abs;
  auto f = [](const Vector3d& p_MQ) -> double {
    return 3.5 * abs(p_MQ.x()) - 2.7 * p_MQ.y() + 0.7 * p_MQ.z() + 1.23;
  };
  auto f_p = [](const Vector3d& p_MQ) -> double {
    return 3.5 * p_MQ.x() - 2.7 * p_MQ.y() + 0.7 * p_MQ.z() + 1.23;
  };
  auto f_n = [](const Vector3d& p_MQ) -> double {
    return -3.5 * p_MQ.x() - 2.7 * p_MQ.y() + 0.7 * p_MQ.z() + 1.23;
  };

  std::vector<double> values;
  for (const Vector3d& p_MV : mesh_M.vertices()) {
    values.push_back(f(p_MV));
  }
  std::vector<double> values_copy = values;

  const MeshFieldLinear<double, VolumeMesh<double>> field_without_gradient(
      std::move(values_copy), &mesh_M, false);
  const MeshFieldLinear<double, VolumeMesh<double>> field_with_gradient(
      std::move(values), &mesh_M, true);

  ASSERT_THROW(field_without_gradient.EvaluateGradient(0), std::runtime_error);
  ASSERT_NO_THROW(field_with_gradient.EvaluateGradient(0));

  {
    // Evaluating the field for points within tetrahedra.  The tolerance
    // 2e-15 is empirically determined.
    using Barycentric = VolumeMesh<double>::Barycentric<double>;
    for (const Barycentric& b_Q :
         {Barycentric{0.25, 0.25, 0.25, 0.25} /* centroid */,
          Barycentric{0.5, 0.5, 0, 0} /* on edge */,
          Barycentric{0.49999, 0.49999, 1e-5, 1e-5} /* near edge */}) {
      // TODO(SeanCurtis-TRI): it's ridiculous that we don't have a mesh method
      //  that turns (element index, barycentric) --> cartesian.
      for (int e = 0; e < mesh_M.num_elements(); ++e) {
        Vector3d p_MQ{0, 0, 0};
        for (int i = 0; i < 4; ++i) {
          p_MQ += mesh_M.vertex(mesh_M.element(e).vertex(i)) * b_Q(i);
        }
        const double expect = f(p_MQ);
        constexpr double tolerance = 2e-15;
        EXPECT_NEAR(expect, field_without_gradient.EvaluateCartesian(e, p_MQ),
                    tolerance);
        EXPECT_NEAR(expect, field_with_gradient.EvaluateCartesian(e, p_MQ),
                    tolerance);
      }
    }
  }

  // Evaluating the field for points that don't necessarily lie within
  // tetrahedra. The relative tolerance 1e-13 is empirically determined.
  for (const Vector3d& p_MQ :
       {Vector3d{1e-15, 1e-15, 1e-15}, Vector3d{0.1, 0.2, 0.3},
        Vector3d{-10.23, 27, 77}, Vector3d{321.3, -843.2, 202.02}}) {
    for (int e = 0; e < mesh_M.num_elements(); ++e) {
      if (positive_set.count(e) == 1) {
        // f⁺(x,y,z) =  3.5x - 2.7y + 0.7z + 1.23
        const double expect = f_p(p_MQ);
        const double tolerance = 1e-13 * abs(expect);
        EXPECT_NEAR(expect, field_without_gradient.EvaluateCartesian(e, p_MQ),
                    tolerance);
        EXPECT_NEAR(expect, field_with_gradient.EvaluateCartesian(e, p_MQ),
                    tolerance);
      } else {
        // f⁻(x,y,z) = -3.5x - 2.7y + 0.7z + 1.23
        const double expect = f_n(p_MQ);
        const double tolerance = 1e-13 * abs(expect);
        EXPECT_NEAR(expect, field_without_gradient.EvaluateCartesian(e, p_MQ),
                    tolerance);
        EXPECT_NEAR(expect, field_with_gradient.EvaluateCartesian(e, p_MQ),
                    tolerance);
      }
    }
  }
}

/* A double-valued field can produce AutoDiffXd-valued results for
 Evaluate() and EvaluateCartesian() based on the scalar type of the query point.
 This confirms that mixing behavior. The tests work by instantiating fields and
 various query parameters on *both* types. Then we mix and match field and
 parameter scalars and verify the outputs. By "verify", we don't worry too much
 about the correctness of the values and derivatives (we assume that has been
 tested elsewhere); we're just checking that, mechanically speaking, derivatives
 are propagating as expected. To that end, we make sure that AutoDiffXd-valued
 parameters have at least *one* component with non-zero derivatives.

 Because the scalar type of the mesh is independent of the scalar type of the
 field, we'll simply use a double-valued mesh. */
class ScalarMixingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    mesh_d_ = GenerateMesh<double>();
    std::vector<double> e_d = {0., 1., 2., 3.};
    // We apply non-zero derivatives to the field at vertex 1, it is only
    // referenced by triangle 0, so, it will be the only triangle that *forces*
    // derivatives into the output.
    std::vector<AutoDiffXd> e_ad{0, 1, 2, 3};
    e_ad[1].derivatives().resize(3);
    e_ad[1].derivatives() << 1, 2, 3;  // Arbitrary values.

    field_d_ =
        std::make_unique<MeshFieldLinear<double, TriangleSurfaceMesh<double>>>(
            std::move(e_d), mesh_d_.get());
    field_ad_ = std::make_unique<
        MeshFieldLinear<AutoDiffXd, TriangleSurfaceMesh<double>>>(
        std::move(e_ad), mesh_d_.get());

    p_WQ_d_ = Vector3d::Zero();
    for (int v = 0; v < 3; ++v) {
      p_WQ_d_ += mesh_d_->vertex(v);
    }
    p_WQ_d_ /= 3;
    p_WQ_ad_ = math::InitializeAutoDiff(p_WQ_d_);

    b_d_ = Vector3d(1, 1, 1) / 3.0;
    b_ad_ = math::InitializeAutoDiff(b_d_);

    centroid_value_ = 0;
    for (int v = 0; v < 3; ++v) {
      centroid_value_ += field_d_->EvaluateAtVertex(v);
    }
    centroid_value_ /= 3;
  }

  std::unique_ptr<TriangleSurfaceMesh<double>> mesh_d_;
  std::unique_ptr<MeshFieldLinear<double, TriangleSurfaceMesh<double>>>
      field_d_;
  std::unique_ptr<MeshFieldLinear<AutoDiffXd, TriangleSurfaceMesh<double>>>
      field_ad_;

  int e0_{0};
  int e1_{1};
  // The centroid of triangle 0.
  Vector3<double> p_WQ_d_;
  Vector3<AutoDiffXd> p_WQ_ad_;
  // The centroid of a triangle.
  Vector3<double> b_d_;
  Vector3<AutoDiffXd> b_ad_;

  double centroid_value_{};
};

TEST_F(ScalarMixingTest, Evaluate) {
  constexpr double kEps = std::numeric_limits<double>::epsilon();

  {
    // Double-valued field and double-valued point: double-valued result.
    const double v = field_d_->Evaluate(e0_, b_d_);
    EXPECT_NEAR(v, centroid_value_, kEps);
  }

  {
    // Double-valued field with AutoDiffXd-valued point: AutodDiffXd-valued
    // result.
    const AutoDiffXd v = field_d_->Evaluate(e0_, b_ad_);
    EXPECT_EQ(v.derivatives().size(), 3);
    EXPECT_NEAR(v.value(), centroid_value_, kEps);
  }

  {
    // AutoDiffXd-valued field with double-valued point on triangle *with*
    // derivatives: AutodDiffXd-valued result *with* derivatives.
    const AutoDiffXd v0 = field_ad_->Evaluate(e0_, b_d_);
    EXPECT_EQ(v0.derivatives().size(), 3);
    EXPECT_NEAR(v0.value(), centroid_value_, kEps);

    // AutoDiffXd-valued field with double-valued point on triangle *without*
    // derivatives: AutodDiffXd-valued result *without* derivatives.
    const AutoDiffXd v1 = field_ad_->Evaluate(e1_, b_d_);
    EXPECT_EQ(v1.derivatives().size(), 0);
    // We'll assume the *value* on this one is correct.
  }

  {
    // AutoDiffXd-valued field with AutoDiffXd-valued point on triangle:
    // AutodDiffXd-valued.
    const AutoDiffXd v = field_ad_->Evaluate(e0_, b_ad_);
    EXPECT_EQ(v.derivatives().size(), 3);
    EXPECT_NEAR(v.value(), centroid_value_, kEps);
  }
}

TEST_F(ScalarMixingTest, EvaluateCartesian) {
  constexpr double kEps = std::numeric_limits<double>::epsilon();

  {
    // Double-valued field and double-valued point: double-valued result.
    const double v = field_d_->EvaluateCartesian(e0_, p_WQ_d_);
    EXPECT_NEAR(v, centroid_value_, kEps);
  }

  {
    // Double-valued field with AutoDiffXd-valued point: AutodDiffXd-valued
    // result.
    const AutoDiffXd v = field_d_->EvaluateCartesian(e0_, p_WQ_ad_);
    EXPECT_EQ(v.derivatives().size(), 3);
    EXPECT_NEAR(v.value(), centroid_value_, kEps);
  }

  {
    // AutoDiffXd-valued field with double-valued point on triangle *with*
    // derivatives: AutodDiffXd-valued result *with* derivatives.
    const AutoDiffXd v0 = field_ad_->EvaluateCartesian(e0_, p_WQ_d_);
    EXPECT_EQ(v0.derivatives().size(), 3);
    EXPECT_NEAR(v0.value(), centroid_value_, kEps);

    // AutoDiffXd-valued field with double-valued point on triangle *without*
    // derivatives: AutodDiffXd-valued result *without* derivatives.
    const AutoDiffXd v1 = field_ad_->EvaluateCartesian(e1_, p_WQ_d_);
    EXPECT_EQ(v1.derivatives().size(), 0);
    // We'll assume the *value* on this one is correct.
  }

  {
    // AutoDiffXd-valued field with AutoDiffXd-valued point on triangle:
    // AutodDiffXd-valued.
    const AutoDiffXd v = field_ad_->EvaluateCartesian(e0_, p_WQ_ad_);
    EXPECT_EQ(v.derivatives().size(), 3);
    EXPECT_NEAR(v.value(), centroid_value_, kEps);
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
