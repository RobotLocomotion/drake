#include "drake/geometry/proximity/field_intersection.h"

#include <memory>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/make_box_field.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_sphere_field.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh_field.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;

// This fixture is for intersection of two tetrahedra with linear functions.
// We set up the tetrahedra and linear functions in such a way that
// the two functions have the equilibrium plane that intersects the two
// tetrahedra into an octagon. See the picture:
// geometry/proximity/images/two_linear_tetrahedra_intersect_into_octagon.png
class FieldIntersectionLowLevelTest : public ::testing::Test {
 public:
  FieldIntersectionLowLevelTest()
      : mesh0_M_(std::vector<VolumeElement>{{0, 1, 2, 3}},
                 std::vector<Vector3d>{{2.0, 0.0, 2.0},
                                       {-2.0, 0.0, 2.0},
                                       {0.0, 2.0, -2.0},
                                       {0.0, -2.0, -2.0}}),
        field0_M_({8e4, 4e4, 4e4, 0}, &mesh0_M_),  // (4 + x + y + z)*10⁴
        mesh1_N_(std::vector<VolumeElement>{{0, 1, 2, 3}},
                 std::vector<Vector3d>{{1.5, 1.5, 2.5},
                                       {-1.5, -1.5, 2.5},
                                       {-1.5, 1.5, -2.5},
                                       {1.5, -1.5, -2.5}}),
        field1_N_({7e4, 1e4, 4e4, 4e4}, &mesh1_N_)  // (4 + x + y)*10⁴
  {}

 protected:
  // Returns the mesh expressed in frame F given X_FG and the mesh expressed
  // in frame G.
  static VolumeMesh<double> TransformVolumeMesh(
      const RigidTransformd& X_FG, const VolumeMesh<double>& mesh_G) {
    std::vector<Vector3d> p_FVs;
    for (const Vector3d& p_GV : mesh_G.vertices()) {
      p_FVs.emplace_back(X_FG * p_GV);
    }
    return {std::vector<VolumeElement>(mesh_G.tetrahedra()), std::move(p_FVs)};
  }

  const VolumeMesh<double> mesh0_M_;
  const VolumeMeshFieldLinear<double, double> field0_M_;
  const VolumeMesh<double> mesh1_N_;
  const VolumeMeshFieldLinear<double, double> field1_N_;

  double tolerance(double scale = 1) {
    const double kEps = 1e-14;
    return (scale > 1) ? kEps * scale : kEps;
  }
};

// For simplicity, this test identifies the two frames of the two meshes to
// be the same, so we can easily calculate the expected equilibrium plane for
// verification. The next test will set up a complex rigid transform between
// two frames.
TEST_F(FieldIntersectionLowLevelTest, CalcEquilibriumPlaneIdenticalFrames) {
  const auto X_MN = RigidTransformd::Identity();
  //     f0(x,y,z) = (4 + x + y + z)*10⁴   (1)
  //     f1(x,y,z) = (4 + x + y)*10⁴       (2)
  // The equilibrium plane satisfies:
  //       f0 - f1 = z*10⁴ = 0      (1)-(2) = 0.
  // Therefore, z = 0 is the equilibrium plane. We pick the plane normal
  // +UnitZ() instead of -UnitZ(), so that it points out of f1 and into f0.
  const Plane<double> expected_plane_M{Vector3d::UnitZ(), Vector3d::Zero()};

  // Initialize the plane to be different from the expected plane.
  Plane<double> plane_M{Vector3d::UnitX(), Vector3d(1, 2, 3)};
  const bool success =
      CalcEquilibriumPlane(0, field0_M_, 0, field1_N_, X_MN, &plane_M);

  ASSERT_TRUE(success);
  EXPECT_TRUE(CompareMatrices(plane_M.normal(), expected_plane_M.normal(),
                              tolerance()));
  // The choice of this query point is arbitrary.
  const Vector3d p_MQ(0.1, 0.2, 0.3);
  const double expected_height = expected_plane_M.CalcHeight<double>(p_MQ);
  EXPECT_NEAR(plane_M.CalcHeight<double>(p_MQ), expected_height,
              tolerance(expected_height));
}

// Expresses the two meshes in two different frames F and G for a
// stronger test. Correctness of this test also depends on the previous test.
//
//     mesh0_F <--> mesh0_M  <--M=N--> mesh1_N <--> mesh1_G
//
TEST_F(FieldIntersectionLowLevelTest, CalcEquilibriumPlaneComplexTransform) {
  const auto X_FG = RigidTransformd(RollPitchYawd(M_PI_4, M_PI/3, M_PI_2),
                                    Vector3d(-1., -2., -3.));
  const auto X_FM = RigidTransformd(RollPitchYawd(M_PI, M_PI_2, M_PI/3),
                                    Vector3d(2, -6, 4));
  // From the previous test, we know that identifying frame M with frame N
  // will give a simple expression of the expected equilibrium plane in frame M.
  const RigidTransformd X_MN = RigidTransformd::Identity();
  const Vector3d expected_normal_M = Vector3d::UnitZ();
  const Vector3d expected_p_M = Vector3d::Zero();
  Plane<double> expected_plane_F(X_FM.rotation() * expected_normal_M,
                                 X_FM * expected_p_M);
  const RigidTransformd X_GN = X_FG.InvertAndCompose(X_FM) * X_MN;

  const VolumeMesh<double> mesh0_F = TransformVolumeMesh(X_FM, mesh0_M_);
  const VolumeMeshFieldLinear<double, double> field0_F(
      std::vector<double>(field0_M_.values()), &mesh0_F);

  const VolumeMesh<double> mesh1_G = TransformVolumeMesh(X_GN, mesh1_N_);
  const VolumeMeshFieldLinear<double, double> field1_G(
      std::vector<double>(field1_N_.values()), &mesh1_G);

  // Initialize the plane arbitrarily.
  Plane<double> plane_F{Vector3d(-1, -2, -3), Vector3d(-4, -5, -6)};
  bool success = CalcEquilibriumPlane(0, field0_F, 0, field1_G, X_FG, &plane_F);

  ASSERT_TRUE(success);
  EXPECT_TRUE(CompareMatrices(plane_F.normal(), expected_plane_F.normal(),
                              tolerance()));
  // The choice of this query point is arbitrary.
  const Vector3d p_FQ(0.1, 0.2, 0.3);
  const double expected_height = expected_plane_F.CalcHeight<double>(p_FQ);
  EXPECT_NEAR(plane_F.CalcHeight<double>(p_FQ), expected_height,
              tolerance(expected_height));
}

// Tests special cases when the two linear functions have no equilibrium
// plane. It can happen when their gradients are deemed identical, which means
// the two functions are equal everywhere, or they are nowhere equal.
TEST_F(FieldIntersectionLowLevelTest, CalcEquilibriumPlaneNone) {
  // Same function in the same frame are equal everywhere, so there is no
  // equilibrium plane. (Mathematically speaking, every plane is an
  // equilibrium plane.)
  {
    // An arbitrary initial value of the plane.
    const Plane<double> init_plane_M{Vector3d::UnitX(), Vector3d(1, 2, 3)};

    Plane<double> plane_M(init_plane_M);
    const bool success = CalcEquilibriumPlane(
        0, field0_M_, 0, field0_M_, RigidTransformd::Identity(), &plane_M);

    ASSERT_FALSE(success);
    // Verify that the plane has not changed from its initial value.
    EXPECT_EQ(plane_M.normal(), init_plane_M.normal());
    // The choice of this query point is arbitrary.
    const Vector3d p_FQ(0.1, 0.2, 0.3);
    const double expected_height = init_plane_M.CalcHeight<double>(p_FQ);
    EXPECT_EQ(plane_M.CalcHeight<double>(p_FQ), expected_height);
  }
  // Use a translational frame L with respect to frame M to make two functions
  // that are nowhere equal.
  {
    const RigidTransformd X_ML(Vector3d(0.4, 0.3, 0.5));
    std::unique_ptr<VolumeMeshFieldLinear<double, double>> field2_L_ptr =
        field0_M_.CloneAndSetMesh(&field0_M_.mesh());
    const VolumeMeshFieldLinear<double, double>& field2_L = *field2_L_ptr;

    // An arbitrary initial value of the plane.
    const Plane<double> init_plane_M{Vector3d::UnitX(), Vector3d(1, 2, 3)};

    Plane<double> plane_M(init_plane_M);
    const bool success =
        CalcEquilibriumPlane(0, field0_M_, 0, field2_L, X_ML, &plane_M);

    ASSERT_FALSE(success);
    // Verify that the plane has not changed from its initial value.
    EXPECT_EQ(plane_M.normal(), init_plane_M.normal());
    // The choice of this query point is arbitrary.
    const Vector3d p_FQ(0.1, 0.2, 0.3);
    const double expected_height = init_plane_M.CalcHeight<double>(p_FQ);
    EXPECT_EQ(plane_M.CalcHeight<double>(p_FQ), expected_height);
  }
}

// Tests two tetrahedra intersecting their pressure-equilibrium plane into an
// octahedron as shown in this picture:
// geometry/proximity/images/two_linear_tetrahedra_intersect_into_octagon.png
TEST_F(FieldIntersectionLowLevelTest, IntersectTetrahedra) {
  const int first_element_in_field0{0};
  const int first_element_in_field1{0};
  const RigidTransformd identity_X_MN = RigidTransformd::Identity();
  Plane<double> plane_M{Vector3d::UnitZ(), Vector3d::Zero()};
  bool success = CalcEquilibriumPlane(first_element_in_field0, field0_M_,
                                      first_element_in_field1, field1_N_,
                                      identity_X_MN, &plane_M);
  ASSERT_TRUE(success);

  const std::vector<Vector3d> polygon_M = IntersectTetrahedra(
      first_element_in_field0, field0_M_.mesh(), first_element_in_field1,
      field1_N_.mesh(), identity_X_MN, plane_M);

  ASSERT_EQ(polygon_M.size(), 8);

  // Use empirical tolerance 1e-14 meters.
  const double kEps = 1e-14;
  // This check relies on the order of tetrahedral elements in the mesh. It
  // might need to change if the mesh changes. See the picture
  // two_linear_tetrahedra_intersect_into_octagon.png
  // to infer coordinates of the polygon's vertices together with this diagram:
  //
  //                  +Y
  //                   ^
  //                   |              Pi = position of i-th vertex
  //                   |
  //            P7     1      P6
  //                   |
  //                   |
  //     P0           0.5            P5
  //                   |
  //                   |
  //     +------+------0------+------+---------> +X
  //    -1    -0.5     |     0.5     1
  //                   |
  //     P1          -0.5            P4
  //                   |
  //                   |
  //            P2    -1      P3
  //
  EXPECT_TRUE(CompareMatrices(polygon_M.at(0), Vector3d(-1, 0.5, 0), kEps));
  EXPECT_TRUE(CompareMatrices(polygon_M.at(1), Vector3d(-1, -0.5, 0), kEps));
  EXPECT_TRUE(CompareMatrices(polygon_M.at(2), Vector3d(-0.5, -1, 0), kEps));
  EXPECT_TRUE(CompareMatrices(polygon_M.at(3), Vector3d(0.5, -1, 0), kEps));
  EXPECT_TRUE(CompareMatrices(polygon_M.at(4), Vector3d(1, -0.5, 0), kEps));
  EXPECT_TRUE(CompareMatrices(polygon_M.at(5), Vector3d(1, 0.5, 0), kEps));
  EXPECT_TRUE(CompareMatrices(polygon_M.at(6), Vector3d(0.5, 1, 0), kEps));
  EXPECT_TRUE(CompareMatrices(polygon_M.at(7), Vector3d(-0.5, 1, 0), kEps));
}

// Move the equilibrium plane so far away that the above IntersectTetrahedra
// test becomes no intersection.
TEST_F(FieldIntersectionLowLevelTest, IntersectTetrahedra_NoIntersection) {
  const int first_element_in_mesh0{0};
  const int first_element_in_mesh1{0};
  // This plane is far away from the two tetrahedra.
  const Plane<double> plane_M{Vector3d::UnitZ(), 5.0 * Vector3d::UnitZ()};

  const std::vector<Vector3d> polygon_M = IntersectTetrahedra(
      first_element_in_mesh0, field0_M_.mesh(), first_element_in_mesh1,
      field1_N_.mesh(), RigidTransformd::Identity(), plane_M);

  EXPECT_EQ(polygon_M.size(), 0);
}

TEST_F(FieldIntersectionLowLevelTest, IsPlaneNormalAlongPressureGradient) {
  const int first_tetrahedron_in_field0{0};

  // The field0_M_ is (4 + x + y + z)*10⁴, so its gradient vector is
  // (10⁴, 10⁴, 10⁴). Therefore, this unit vector is along that gradient
  // vector.
  const Vector3d nhat_M_along_gradient = Vector3d(1, 1, 1).normalized();
  EXPECT_TRUE(IsPlaneNormalAlongPressureGradient(
      nhat_M_along_gradient, first_tetrahedron_in_field0, field0_M_));

  const Vector3d nhat_M_against_gradient = -nhat_M_along_gradient;
  EXPECT_FALSE(IsPlaneNormalAlongPressureGradient(
      nhat_M_against_gradient, first_tetrahedron_in_field0, field0_M_));
}

class FieldIntersectionHighLevelTest : public ::testing::Test {
 public:
  FieldIntersectionHighLevelTest()
      : box_mesh0_M_(MakeBoxVolumeMeshWithMa<double>(box_)),
        box_field0_M_(MakeBoxPressureField<double>(box_, &box_mesh0_M_,
                                                   kBoxElasitcModulus_)),
        box_bvh0_M_(box_mesh0_M_),
        // Get a mesh of an octahedron from a sphere specification by
        // specifying very coarse resolution hint.
        octahedron_mesh1_N_(MakeSphereVolumeMesh<double>(
            sphere_, 10 * sphere_.radius(),
            TessellationStrategy::kSingleInteriorVertex)),
        octahedron_field1_N_(MakeSpherePressureField<double>(
            sphere_, &octahedron_mesh1_N_, kOctahedronElasticModulus_)),
        octahedron_bvh1_N_(octahedron_mesh1_N_) {}

 protected:
  void SetUp() override {
    DRAKE_DEMAND(octahedron_mesh1_N_.num_elements() == 8);
  }

  // Geometry 0 and its field.
  const Box box_{0.06, 0.10, 0.14};  // 6cm-thick compliant pad.
  const double kBoxElasitcModulus_{1.0e5};
  const VolumeMesh<double> box_mesh0_M_;
  const VolumeMeshFieldLinear<double, double> box_field0_M_;
  const Bvh<Obb, VolumeMesh<double>> box_bvh0_M_;

  // Geometry 1 and its field.
  const Sphere sphere_{0.03};  // 3cm-radius (6cm-diameter) finger tip.
  const double kOctahedronElasticModulus_{1.0e5};
  const VolumeMesh<double> octahedron_mesh1_N_;
  const VolumeMeshFieldLinear<double, double> octahedron_field1_N_;
  const Bvh<Obb, VolumeMesh<double>> octahedron_bvh1_N_;
};

TEST_F(FieldIntersectionHighLevelTest, IntersectFields) {
  const RigidTransformd X_MN = RigidTransformd(0.03 * Vector3d::UnitX());
  std::vector<Vector3d> grad_e0_Ms;
  std::vector<Vector3d> grad_e1_Ms;
  {
    SCOPED_TRACE("Use TriMeshBuilder.");
    std::unique_ptr<TriangleSurfaceMesh<double>> surface_01_M;
    std::unique_ptr<TriangleSurfaceMeshFieldLinear<double, double>> e_MN_M;
    IntersectFields<TriangleSurfaceMesh<double>, TriMeshBuilder<double>>(
        box_field0_M_, box_bvh0_M_, octahedron_field1_N_, octahedron_bvh1_N_,
        X_MN, &surface_01_M, &e_MN_M, &grad_e0_Ms, &grad_e1_Ms);

    EXPECT_NE(surface_01_M.get(), nullptr);
  }
  {
    SCOPED_TRACE("Use PolyMeshBuilder.");
    std::unique_ptr<PolygonSurfaceMesh<double>> surface_01_M;
    std::unique_ptr<PolygonSurfaceMeshFieldLinear<double, double>> e_MN_M;
    IntersectFields<PolygonSurfaceMesh<double>, PolyMeshBuilder<double>>(
        box_field0_M_, box_bvh0_M_, octahedron_field1_N_, octahedron_bvh1_N_,
        X_MN, &surface_01_M, &e_MN_M, &grad_e0_Ms, &grad_e1_Ms);

    EXPECT_NE(surface_01_M.get(), nullptr);

    // Both types of MeshBuilder got the same pressure and gradient
    // calculations, so we check only the PolyMeshBuilder.

    // TODO(DamrongGuoy) Add more rigorous check and documentation. Right now
    //  we do it empirically. If the order of tetrahedra in the input meshes
    //  change, the first vertex and the first polygon may not be the same as
    //  the ones we check below. Therefore, the expected pressure value and
    //  pressure gradients will change.
    const double kRelativeTolerance = 1e-13;
    // The hydroelastic modulus is in the order of 1e5 Pa. Empirically
    // we determined that the first vertex of the contact surface has half of
    // maximum pressure, i.e., 1e5 / 2 = 50 kPa = 5e4 Pa.
    const double kExpectPressure = 5.0e4;
    const double kPressureTolerance = kRelativeTolerance * kExpectPressure;
    EXPECT_NEAR(e_MN_M->EvaluateAtVertex(0), kExpectPressure,
                kPressureTolerance);

    // The hydroelastic modulus is in the order of 100 kPa, and the size of
    // the geometries are in the order of centimeters. Therefore, the
    // pressure gradient is in the order of 100 kPa / 1 cm = 10 MPa/meter
    // = 1e7 Pa/m.
    const double kExpectGradientOrderOfMagnitude = 1e7;
    const double kGradientTolerance =
        kRelativeTolerance * kExpectGradientOrderOfMagnitude;
    const Vector3d expect_grad_e0_M =
        kExpectGradientOrderOfMagnitude * Vector3d(-1 / 3.0, 0, 0);
    EXPECT_TRUE(CompareMatrices(grad_e0_Ms.at(0), expect_grad_e0_M,
                                kGradientTolerance));
    const Vector3d expect_grad_e1_M =
        kExpectGradientOrderOfMagnitude * Vector3d(1 / 3.0, -1 / 3.0, -1 / 3.0);
    EXPECT_TRUE(CompareMatrices(grad_e1_Ms.at(0), expect_grad_e1_M,
                                kGradientTolerance));
  }
}

// Smoke tests that AutoDiffXd can build. No checking on the values of
// derivatives.
TEST_F(FieldIntersectionHighLevelTest, FieldIntersectionAutoDiffXd) {
  const math::RigidTransform<AutoDiffXd> X_MN(0.03 *
                                              Vector3<AutoDiffXd>::UnitX());
  std::vector<Vector3<AutoDiffXd>> grad_e0_Ms;
  std::vector<Vector3<AutoDiffXd>> grad_e1_Ms;
  {
    SCOPED_TRACE("Use TriMeshBuilder.");
    std::unique_ptr<TriangleSurfaceMesh<AutoDiffXd>> surface_01_M;
    std::unique_ptr<TriangleSurfaceMeshFieldLinear<AutoDiffXd, AutoDiffXd>>
        e_MN_M;
    IntersectFields<TriangleSurfaceMesh<AutoDiffXd>,
                    TriMeshBuilder<AutoDiffXd>>(
        box_field0_M_, box_bvh0_M_, octahedron_field1_N_, octahedron_bvh1_N_,
        X_MN, &surface_01_M, &e_MN_M, &grad_e0_Ms, &grad_e1_Ms);

    EXPECT_NE(surface_01_M.get(), nullptr);
  }
  {
    SCOPED_TRACE("Use PolyMeshBuilder.");
    std::unique_ptr<PolygonSurfaceMesh<AutoDiffXd>> surface_01_M;
    std::unique_ptr<PolygonSurfaceMeshFieldLinear<AutoDiffXd, AutoDiffXd>>
        e_MN_M;
    IntersectFields<PolygonSurfaceMesh<AutoDiffXd>,
                    PolyMeshBuilder<AutoDiffXd>>(
        box_field0_M_, box_bvh0_M_, octahedron_field1_N_, octahedron_bvh1_N_,
        X_MN, &surface_01_M, &e_MN_M, &grad_e0_Ms, &grad_e1_Ms);

    EXPECT_NE(surface_01_M.get(), nullptr);
  }
}

// Special case of no intersection. Request PolygonSurfaceMesh<double> as the
// representative template argument.
TEST_F(FieldIntersectionHighLevelTest, FieldIntersectionNoIntersection) {
  // 1 meter apart is well separated.
  const RigidTransformd X_MN(Vector3d::UnitX());
  std::vector<Vector3d> grad_e0_Ms;
  std::vector<Vector3d> grad_e1_Ms;
  std::unique_ptr<PolygonSurfaceMesh<double>> surface_01_M;
  std::unique_ptr<PolygonSurfaceMeshFieldLinear<double, double>> e_MN_M;
  IntersectFields<PolygonSurfaceMesh<double>, PolyMeshBuilder<double>>(
      box_field0_M_, box_bvh0_M_, octahedron_field1_N_, octahedron_bvh1_N_,
      X_MN, &surface_01_M, &e_MN_M, &grad_e0_Ms, &grad_e1_Ms);

  EXPECT_EQ(surface_01_M.get(), nullptr);
  EXPECT_EQ(e_MN_M.get(), nullptr);
  EXPECT_EQ(grad_e0_Ms.size(), 0);
  EXPECT_EQ(grad_e1_Ms.size(), 0);
}

TEST_F(FieldIntersectionHighLevelTest, IntersectCompliantVolumes) {
  GeometryId first_id = GeometryId::get_new_id();
  GeometryId second_id = GeometryId::get_new_id();
  const RigidTransformd X_WM = RigidTransformd::Identity();
  const RigidTransformd X_WN(0.03 * Vector3d::UnitX());
  {
    SCOPED_TRACE("Triangle contact surface.");
    std::unique_ptr<ContactSurface<double>> contact_patch_W =
        IntersectCompliantVolumes<TriangleSurfaceMesh<double>,
                                  TriMeshBuilder<double>>(
            first_id, box_field0_M_, box_bvh0_M_, X_WM, second_id,
            octahedron_field1_N_, octahedron_bvh1_N_, X_WN);
    ASSERT_NE(contact_patch_W.get(), nullptr);
    EXPECT_EQ(contact_patch_W->representation(),
              HydroelasticContactRepresentation::kTriangle);
  }
  {
    SCOPED_TRACE("Polygon contact surface.");
    std::unique_ptr<ContactSurface<double>> contact_patch_W =
        IntersectCompliantVolumes<PolygonSurfaceMesh<double>,
                                  PolyMeshBuilder<double>>(
            first_id, box_field0_M_, box_bvh0_M_, X_WM, second_id,
            octahedron_field1_N_, octahedron_bvh1_N_, X_WN);
    ASSERT_NE(contact_patch_W.get(), nullptr);
    EXPECT_EQ(contact_patch_W->representation(),
              HydroelasticContactRepresentation::kPolygon);
  }
}

// Smoke tests that AutoDiffXd can build. No checking on the values of
// derivatives.
TEST_F(FieldIntersectionHighLevelTest, IntersectCompliantVolumesAutoDiffXd) {
  GeometryId first_id = GeometryId::get_new_id();
  GeometryId second_id = GeometryId::get_new_id();
  const math::RigidTransform<AutoDiffXd> X_WM =
      math::RigidTransform<AutoDiffXd>::Identity();
  const math::RigidTransform<AutoDiffXd> X_WN(0.03 *
                                              Vector3<AutoDiffXd>::UnitX());
  {
    SCOPED_TRACE("Triangle contact surface.");
    std::unique_ptr<ContactSurface<AutoDiffXd>> contact_patch_W =
        IntersectCompliantVolumes<TriangleSurfaceMesh<AutoDiffXd>,
                                  TriMeshBuilder<AutoDiffXd>>(
            first_id, box_field0_M_, box_bvh0_M_, X_WM, second_id,
            octahedron_field1_N_, octahedron_bvh1_N_, X_WN);
    ASSERT_NE(contact_patch_W.get(), nullptr);
    EXPECT_EQ(contact_patch_W->representation(),
              HydroelasticContactRepresentation::kTriangle);
  }
  {
    SCOPED_TRACE("Polygon contact surface.");
    std::unique_ptr<ContactSurface<AutoDiffXd>> contact_patch_W =
        IntersectCompliantVolumes<PolygonSurfaceMesh<AutoDiffXd>,
                                  PolyMeshBuilder<AutoDiffXd>>(
            first_id, box_field0_M_, box_bvh0_M_, X_WM, second_id,
            octahedron_field1_N_, octahedron_bvh1_N_, X_WN);
    ASSERT_NE(contact_patch_W.get(), nullptr);
    EXPECT_EQ(contact_patch_W->representation(),
              HydroelasticContactRepresentation::kPolygon);
  }
}

// Special case: no intersection. Request PolygonSurfaceMesh<double> as a
// representative.
TEST_F(FieldIntersectionHighLevelTest,
       IntersectCompliantVolumesNoIntersection) {
  GeometryId first_id = GeometryId::get_new_id();
  GeometryId second_id = GeometryId::get_new_id();
  const RigidTransformd X_WM = RigidTransformd::Identity();
  // 1 meter makes them well separated.
  const RigidTransformd X_WN(Vector3d::UnitX());

  std::unique_ptr<ContactSurface<double>> contact_patch_W =
      IntersectCompliantVolumes<PolygonSurfaceMesh<double>,
                                PolyMeshBuilder<double>>(
          first_id, box_field0_M_, box_bvh0_M_, X_WM, second_id,
          octahedron_field1_N_, octahedron_bvh1_N_, X_WN);
  EXPECT_EQ(contact_patch_W.get(), nullptr);
}

TEST_F(FieldIntersectionHighLevelTest,
       ComputeContactSurfaceFromCompliantVolumes) {
  GeometryId first_id = GeometryId::get_new_id();
  GeometryId second_id = GeometryId::get_new_id();
  const RigidTransformd X_WM = RigidTransformd::Identity();
  const RigidTransformd X_WN(0.03 * Vector3d::UnitX());
  {
    SCOPED_TRACE("Request triangles.");
    std::unique_ptr<ContactSurface<double>> contact_patch_W =
        ComputeContactSurfaceFromCompliantVolumes(
            first_id, box_field0_M_, box_bvh0_M_, X_WM,
            second_id, octahedron_field1_N_, octahedron_bvh1_N_, X_WN,
            HydroelasticContactRepresentation::kTriangle);
    ASSERT_NE(contact_patch_W.get(), nullptr);
    EXPECT_EQ(contact_patch_W->representation(),
              HydroelasticContactRepresentation::kTriangle);
  }
  {
    SCOPED_TRACE("Request polygons.");
    std::unique_ptr<ContactSurface<double>> contact_patch_W =
        ComputeContactSurfaceFromCompliantVolumes(
            first_id, box_field0_M_, box_bvh0_M_, X_WM,
            second_id, octahedron_field1_N_, octahedron_bvh1_N_, X_WN,
            HydroelasticContactRepresentation::kPolygon);
    ASSERT_NE(contact_patch_W.get(), nullptr);
    EXPECT_EQ(contact_patch_W->representation(),
              HydroelasticContactRepresentation::kPolygon);
  }
}

// Smoke tests that AutoDiffXd can build. No checking on the values of
// derivatives.
TEST_F(FieldIntersectionHighLevelTest,
       ComputeContactSurfaceFromCompliantVolumesAutoDiffXd) {
  GeometryId first_id = GeometryId::get_new_id();
  GeometryId second_id = GeometryId::get_new_id();
  const math::RigidTransform<AutoDiffXd> X_WM =
      math::RigidTransform<AutoDiffXd>::Identity();
  const math::RigidTransform<AutoDiffXd> X_WN(0.03 *
                                              Vector3<AutoDiffXd>::UnitX());
  {
    SCOPED_TRACE("Request triangles.");
    std::unique_ptr<ContactSurface<AutoDiffXd>> contact_patch_W =
        ComputeContactSurfaceFromCompliantVolumes(
            first_id, box_field0_M_, box_bvh0_M_, X_WM,
            second_id, octahedron_field1_N_, octahedron_bvh1_N_, X_WN,
            HydroelasticContactRepresentation::kTriangle);
  }
  {
    SCOPED_TRACE("Request polygons.");
    std::unique_ptr<ContactSurface<AutoDiffXd>> contact_patch_W =
        ComputeContactSurfaceFromCompliantVolumes(
            first_id, box_field0_M_, box_bvh0_M_, X_WM,
            second_id, octahedron_field1_N_, octahedron_bvh1_N_, X_WN,
            HydroelasticContactRepresentation::kPolygon);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
