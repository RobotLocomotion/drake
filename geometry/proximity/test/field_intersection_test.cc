#include "drake/geometry/proximity/field_intersection.h"

#include <memory>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/make_box_field.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_cylinder_field.h"
#include "drake/geometry/proximity/make_cylinder_mesh.h"
#include "drake/geometry/proximity/make_sphere_field.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/mesh_intersection.h"
#include "drake/geometry/proximity/mesh_to_vtk.h"
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
// TODO(DamrongGuoy): Complete the code that can generate the above picture.
//  We will need a few more PRs.
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
    std::vector<Vector3<double>> p_FVs;
    for (const Vector3<double>& p_GV : mesh_G.vertices()) {
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
    VolumeMeshFieldLinear<double, double> field2_L(field0_M_);

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

TEST_F(FieldIntersectionLowLevelTest, IntersectPlaneTetrahedron) {
  const int first_element_in_field0{0};
  // Define the X-Y plane in frame M.
  const Plane<double> plane_M(Vector3d::UnitZ(), Vector3d::Zero());

  std::vector<Vector3d> polygon_M;
  IntersectPlaneTetrahedron(first_element_in_field0, field0_M_, plane_M,
                            &polygon_M);

  EXPECT_EQ(polygon_M.size(), 4);
}

TEST_F(FieldIntersectionLowLevelTest, IntersectTetrahedra) {
  const int first_element_in_field0{0};
  const int first_element_in_field1{0};
  const RigidTransformd identity_X_MN = RigidTransformd::Identity();
  Plane<double> plane_M{Vector3d::UnitZ(), Vector3d::Zero()};
  bool success = CalcEquilibriumPlane(first_element_in_field0, field0_M_,
                                      first_element_in_field1, field1_N_,
                                      identity_X_MN, &plane_M);
  ASSERT_TRUE(success);

  Vector3d normal_M;
  const std::vector<Vector3d> polygon_M = IntersectTetrahedra(
      first_element_in_field0, field0_M_, first_element_in_field1, field1_N_,
      identity_X_MN, plane_M);

  EXPECT_EQ(polygon_M.size(), 8);
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
//    WriteVolumeMeshFieldLinearToVtk(
//        "box_field0_M.vtk", box_field0_M_,
//        "Pressure distribution on compliant tetrahedron in frame M");
//    WriteVolumeMeshFieldLinearToVtk(
//        "octahedron_field1_N.vtk", octahedron_field1_N_,
//        "Pressure distribution on compliant tetrahedron in frame N");
    ASSERT_EQ(octahedron_mesh1_N_.num_elements(), 8);
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

TEST_F(FieldIntersectionHighLevelTest, FieldIntersection) {
  const RigidTransformd X_MN = RigidTransformd::Identity();
  std::unique_ptr<TriangleSurfaceMesh<double>> surface_MN_M;
  std::unique_ptr<TriangleSurfaceMeshFieldLinear<double, double>> e_MN_M;
  std::vector<Vector3<double>> grad_eM_Ms;
  std::vector<Vector3<double>> grad_eN_Ms;

  FieldIntersection(box_field0_M_, box_bvh0_M_,
                    octahedron_field1_N_, octahedron_bvh1_N_, X_MN,
                    &surface_MN_M, &e_MN_M, &grad_eM_Ms, &grad_eN_Ms);

  EXPECT_TRUE(surface_MN_M);

  // Visualization.
//  if (surface_MN_M) {
//    WriteTriangleSurfaceMeshFieldLinearToVtk(
//        "box_octahedron_field_intersection.vtk", *e_MN_M,
//        "Pressure distribution on compliant-compliant contact patch.");
//  }
}

TEST_F(FieldIntersectionHighLevelTest,
       ComputeContactSurfaceFromCompliantVolumes) {
  GeometryId first_id = GeometryId::get_new_id();
  GeometryId second_id = GeometryId::get_new_id();
  const RigidTransformd X_WM = RigidTransformd::Identity();
  const RigidTransformd X_WN = RigidTransformd::Identity();

  std::unique_ptr<ContactSurface<double>> contact_patch_W =
  ComputeContactSurfaceFromCompliantVolumes(
      first_id, box_field0_M_, box_bvh0_M_, X_WM,
      second_id, octahedron_field1_N_, octahedron_bvh1_N_, X_WN);
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//  Tested by human via visualization.  This section will not go into master.
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

class FieldIntersectionVizTest : public ::testing::Test {
 public:
  FieldIntersectionVizTest()
      : box_mesh0_M_(MakeBoxVolumeMeshWithMa<double>(box_)),
        box_field0_M_(MakeBoxPressureField<double>(box_, &box_mesh0_M_,
                                                   kBoxElasticModulus_)),
        box_bvh0_M_(box_mesh0_M_),

        ball_mesh1_N_(MakeSphereVolumeMesh<double>(
            ball_, ball_.radius() / 4.0,
            TessellationStrategy::kSingleInteriorVertex)),
        ball_field1_N_(MakeSpherePressureField<double>(ball_, &ball_mesh1_N_,
                                                       kBallElasticModulus_)),
        ball_bvh1_N_(ball_mesh1_N_),

        box2_mesh2_L_(MakeBoxVolumeMeshWithMa<double>(box2_)),
        box2_field2_L_(MakeBoxPressureField<double>(box2_, &box2_mesh2_L_,
                                                    kBoxElasticModulus_)),
        box2_bvh2_L_(box2_mesh2_L_),

        cylinder_mesh3_C_(MakeCylinderVolumeMeshWithMa<double>(
            cylinder_, cylinder_.radius() / 4.0)),
        cylinder_field3_C_(MakeCylinderPressureField<double>(
            cylinder_, &cylinder_mesh3_C_, kCylinderElasticModulus_)),
        cylinder_bvh3_C_(cylinder_mesh3_C_),

        box_thin_mesh4_T_(MakeBoxVolumeMeshWithMa<double>(box_thin_)),
        box_thin_field4_T_(MakeBoxPressureField<double>(
            box_thin_, &box_thin_mesh4_T_, kBoxElasticModulus_)),
        box_thin_bvh4_T_(box_thin_mesh4_T_) {}

 protected:
  void SetUp() override {
//    WriteVolumeMeshFieldLinearToVtk(
//        "box_field0_M.vtk", box_field0_M_,
//        "Pressure field in a compliant box in frame M");
//    WriteVolumeMeshFieldLinearToVtk(
//        "ball_field1_N.vtk", ball_field1_N_,
//        "Pressure field in a compliant ball in frame N");
  }

  static VolumeMesh<double> TransformVolumeMesh(const RigidTransformd& X_MN,
                                         const VolumeMesh<double>& mesh_N) {
    std::vector<Vector3<double>> vertices_M;
    for (const Vector3<double>& p_NV : mesh_N.vertices()) {
      vertices_M.emplace_back(X_MN * p_NV);
    }
    std::vector<VolumeElement> tetrahedra = mesh_N.tetrahedra();
    VolumeMesh<double> mesh_M(std::move(tetrahedra), std::move(vertices_M));
    return mesh_M;
  }

  // The `transformed_mesh` will own the copied mesh, on which the scalar field
  // relies on. You should keep `transformed_mesh` alive as long as the
  // return value is alive.
  VolumeMeshFieldLinear<double, double> TransformVolumeMeshFieldLinear(
      const RigidTransformd& X_MN,
      const VolumeMeshFieldLinear<double, double>& field_N,
      std::unique_ptr<VolumeMesh<double>>* transformed_mesh) {
    *transformed_mesh = std::make_unique<VolumeMesh<double>>(
        TransformVolumeMesh(X_MN, field_N.mesh()));
    // Make another copy of the field values.
    std::vector<double> field_values = field_N.values();
    VolumeMeshFieldLinear<double, double> copy_field_M(std::move(field_values),
                                                       transformed_mesh->get());
    return copy_field_M;
  }

  void WriteTwoFieldsAndTheirContact(
      const VolumeMeshFieldLinear<double, double>& field0_W,
      const std::string& file_name0,
      const VolumeMeshFieldLinear<double, double>& field1_W,
      const std::string& file_name1, const ContactSurface<double>& contact_W,
      const std::string& contact_file_name) {
    WriteVolumeMeshFieldLinearToVtk(file_name0, "Pressure[Pa]", field0_W,
                                    "Pressure field in " + file_name0);
    WriteVolumeMeshFieldLinearToVtk(file_name1, "Pressure[Pa]", field1_W,
                                    "Pressure field in " + file_name1);
    WriteTriangleSurfaceMeshFieldLinearToVtk(
        contact_file_name, "Pressure[Pa]", contact_W.e_MN(),
        "Contact pressure distribution in " + contact_file_name);
  }

  // Geometry 0 and its field.
  const Box box_{0.06, 0.10, 0.14};  // 6cm-thick compliant pad.
  const double kBoxElasticModulus_{1.0e5};
  const VolumeMesh<double> box_mesh0_M_;
  const VolumeMeshFieldLinear<double, double> box_field0_M_;
  const Bvh<Obb, VolumeMesh<double>> box_bvh0_M_;

  // Geometry 1 and its field.
  const Sphere ball_{0.03};  // 3cm-radius (6cm-diameter) finger tip.
  const double kBallElasticModulus_{1.0e5};
  const VolumeMesh<double> ball_mesh1_N_;
  const VolumeMeshFieldLinear<double, double> ball_field1_N_;
  const Bvh<Obb, VolumeMesh<double>> ball_bvh1_N_;

  // Gemetry 2 and its field.
  const Box box2_{0.03, 0.05, 0.07};  // Half of the first box_.
  const VolumeMesh<double> box2_mesh2_L_;
  const VolumeMeshFieldLinear<double, double> box2_field2_L_;
  const Bvh<Obb, VolumeMesh<double>> box2_bvh2_L_;

  // Geometry 3 and its field.
  const Cylinder cylinder_{0.025, 0.20};
  const double kCylinderElasticModulus_{5.0e4};
  const VolumeMesh<double> cylinder_mesh3_C_;
  const VolumeMeshFieldLinear<double, double> cylinder_field3_C_;
  const Bvh<Obb, VolumeMesh<double>> cylinder_bvh3_C_;

  const RigidTransformd X_MN_{Vector3d(0.04, 0, 0)};

  // Geometry 4 and its field. It's a thin box.
  const Box box_thin_{0.03, 0.05, 0.006};
  const VolumeMesh<double> box_thin_mesh4_T_;
  const VolumeMeshFieldLinear<double, double> box_thin_field4_T_;
  const Bvh<Obb, VolumeMesh<double>> box_thin_bvh4_T_;
};

TEST_F(FieldIntersectionVizTest, BoxBall) {
  const VolumeMesh<double> ball_mesh1_M =
      TransformVolumeMesh(X_MN_, ball_mesh1_N_);
  std::vector<double> field_values = ball_field1_N_.values();
  const VolumeMeshFieldLinear<double, double> ball_field1_M(
      std::move(field_values), &ball_mesh1_M);

  // WriteVolumeMeshFieldLinearToVtk(
  //     "ball_field1_M.vtk", ball_field1_M,
  //     "Pressure field in a compliant ball in frame M");

  const GeometryId first_id = GeometryId::get_new_id();
  const GeometryId second_id = GeometryId::get_new_id();
  const RigidTransformd X_WM = RigidTransformd::Identity();
  const RigidTransformd X_WN = X_WM * X_MN_;

  std::unique_ptr<ContactSurface<double>> contact_patch_W =
      ComputeContactSurfaceFromCompliantVolumes(
          first_id, box_field0_M_, box_bvh0_M_, X_WM,
          second_id, ball_field1_N_, ball_bvh1_N_, X_WN);

  ASSERT_TRUE(contact_patch_W != nullptr);

  // Visualization.
  // if (contact_patch_W != nullptr) {
  //   WriteTriangleSurfaceMeshFieldLinearToVtk(
  //       "box_ball_field_intersection.vtk", contact_patch_W->e_MN(),
  //       "Pressure distribution on compliant-compliant contact patch.");
  // }
}

TEST_F(FieldIntersectionVizTest, BoxBall_TimeSeries) {
  const Vector3d start_p_MN =
      box_.size() / 2 + 0.9 * ball_.radius() * box_.size().normalized();
  const Vector3d end_p_MN =
      box_.size() / 2 + 0.1 * ball_.radius() * box_.size().normalized();
  const double start_t = 0;  // seconds
  const double end_t = 1.0;  // seconds
  DRAKE_DEMAND(start_t < end_t);
  const Vector3d velocity_MN = (end_p_MN - start_p_MN) / (end_t - start_t);

  const int num_snapshots = 7;  // snap: 00 01 02 03 04 05 06
  // Demand at least two snapshots for the start and the end.
  DRAKE_DEMAND(num_snapshots >= 2);
  const double dt = (end_t - start_t) / (num_snapshots - 1);

  // Box's frame M is fixed over time and identified with World frame.
  const RigidTransformd X_WM = RigidTransformd::Identity();
  std::unique_ptr<VolumeMesh<double>> box_mesh_W;
  const VolumeMeshFieldLinear<double, double> box_field_W =
      TransformVolumeMeshFieldLinear(X_WM, box_field0_M_, &box_mesh_W);
  const Bvh<Obb, VolumeMesh<double>> box_bvh_W(*box_mesh_W);

  const GeometryId first_id = GeometryId::get_new_id();
  const GeometryId second_id = GeometryId::get_new_id();

  for (int snap = 0; snap < num_snapshots; ++snap) {
    const double t = start_t + snap * dt;
    const Vector3d p_MN = start_p_MN + (t - start_t) * velocity_MN;
    // N is the frame of the moving ball.
    // M is the frame of the stationary box.
    const RigidTransformd X_MN = RigidTransformd(p_MN);
    const RigidTransformd X_WN = X_WM * X_MN;

    std::unique_ptr<VolumeMesh<double>> ball_mesh_W;
    const VolumeMeshFieldLinear<double, double> ball_field_W =
        TransformVolumeMeshFieldLinear(X_WN, ball_field1_N_, &ball_mesh_W);
    const Bvh<Obb, VolumeMesh<double>> ball_bvh_W(*ball_mesh_W);

    std::unique_ptr<ContactSurface<double>> contact_patch_W =
        ComputeContactSurfaceFromCompliantVolumes(
            first_id, box_field_W, box_bvh_W, RigidTransformd::Identity(),
            second_id, ball_field_W, ball_bvh_W, RigidTransformd::Identity());

    ASSERT_TRUE(contact_patch_W != nullptr);

    const std::string box_file_name =
        fmt::format("box_field_W_{:02d}.vtk", snap);
    const std::string ball_file_name =
        fmt::format("ball_field_W_{:02d}.vtk", snap);
    const std::string contact_file_name =
        fmt::format("contact_W_{:02d}.vtk", snap);

    WriteTwoFieldsAndTheirContact(box_field_W, box_file_name, ball_field_W,
                                  ball_file_name, *contact_patch_W,
                                  contact_file_name);
  }
}

TEST_F(FieldIntersectionVizTest, BigBoxSmallBox) {
  // // Face contact.
  // const RigidTransformd X_ML((box_.height() / 2) * Vector3d::UnitZ());
  // // Edge contact.
  // const RigidTransformd X_ML {
  //     Vector3d(box_.width() / 2, box_.depth() / 2, 0)};
  // Corner contact.
  const RigidTransformd X_ML{
      Vector3d(box_.width() / 2, box_.depth() / 2, box_.height() / 2)};
  const VolumeMesh<double> box2_mesh2_M =
      TransformVolumeMesh(X_ML, box2_mesh2_L_);
  std::vector<double> field_values = box2_field2_L_.values();
  const VolumeMeshFieldLinear<double, double> box2_field2_M(
      std::move(field_values), &box2_mesh2_M);
  const Bvh<Obb, VolumeMesh<double>> box2_bvh2_M(box2_mesh2_M);
  // WriteVolumeMeshFieldLinearToVtk(
  //    "box2_field2_M.vtk", box2_field2_M,
  //    "Pressure field in the second compliant box in frame M");

  const GeometryId first_id = GeometryId::get_new_id();
  const GeometryId second_id = GeometryId::get_new_id();
  const RigidTransformd X_WM = RigidTransformd::Identity();

  std::unique_ptr<ContactSurface<double>> contact_patch_W =
      ComputeContactSurfaceFromCompliantVolumes(
          first_id, box_field0_M_, box_bvh0_M_, X_WM,
          second_id, box2_field2_M, box2_bvh2_M, X_WM);

  ASSERT_TRUE(contact_patch_W != nullptr);

  // Visualization.
  // if (contact_patch_W != nullptr) {
  //   WriteTriangleSurfaceMeshFieldLinearToVtk(
  //       "box_box_field_intersection.vtk", contact_patch_W->e_MN(),
  //       "Pressure distribution on compliant-compliant contact patch.");
  // }
}

// Clone the box_ into clone_box. Conceptually the clone's mesh is posed
// in frame L. We set X_ML to be the translation by half the box's height in
// +Mz direction.
TEST_F(FieldIntersectionVizTest, BoxCloneBox) {
  // Contact on the top face.
  const RigidTransformd X_ML((0.9 * box_.height()) * Vector3d::UnitZ());
  const VolumeMesh<double> clone_box_mesh_M =
      TransformVolumeMesh(X_ML, box_mesh0_M_);
  std::vector<double> field_values = box_field0_M_.values();
  const VolumeMeshFieldLinear<double, double> clone_box_field_M(
      std::move(field_values), &clone_box_mesh_M);
  const Bvh<Obb, VolumeMesh<double>> clone_box_bvh_M(clone_box_mesh_M);
  // WriteVolumeMeshFieldLinearToVtk(
  //    "clone_box_field_M.vtk", clone_box_field_M,
  //    "Pressure field in the cloned compliant box in frame M");

  const GeometryId first_id = GeometryId::get_new_id();
  const GeometryId second_id = GeometryId::get_new_id();
  const RigidTransformd X_WM = RigidTransformd::Identity();

  std::unique_ptr<ContactSurface<double>> contact_patch_W =
      ComputeContactSurfaceFromCompliantVolumes(
          first_id, box_field0_M_, box_bvh0_M_, X_WM,
          second_id, clone_box_field_M, clone_box_bvh_M, X_WM);

  ASSERT_TRUE(contact_patch_W != nullptr);

  // Visualization.
  // if (contact_patch_W != nullptr) {
  //   WriteTriangleSurfaceMeshFieldLinearToVtk(
  //       "box_clone_box_field_intersection.vtk", contact_patch_W->e_MN(),
  //       "Pressure distribution on compliant-compliant contact patch.");
  // }
}

TEST_F(FieldIntersectionVizTest, BoxCylinder) {
  const RigidTransformd X_MC(-cylinder_.radius() * Vector3d::UnitX());
  const VolumeMesh<double> cylinder_mesh3_M =
      TransformVolumeMesh(X_MC, cylinder_mesh3_C_);
  std::vector<double> field_values = cylinder_field3_C_.values();
  const VolumeMeshFieldLinear<double, double> cylinder_field3_M(
      std::move(field_values), &cylinder_mesh3_M);
  // WriteVolumeMeshFieldLinearToVtk(
  //     "cylinder_field3_M.vtk", cylinder_field3_M,
  //     "Pressure field in the compliant cylinder in frame M");

  const GeometryId first_id = GeometryId::get_new_id();
  const GeometryId second_id = GeometryId::get_new_id();
  const RigidTransformd X_WM = RigidTransformd::Identity();
  const RigidTransformd X_WC = X_WM * X_MC;

  std::unique_ptr<ContactSurface<double>> contact_patch_W =
      ComputeContactSurfaceFromCompliantVolumes(
          first_id, box_field0_M_, box_bvh0_M_, X_WM,
          second_id, cylinder_field3_C_, cylinder_bvh3_C_, X_WC);

  ASSERT_TRUE(contact_patch_W != nullptr);

  // Visualization.
  // if (contact_patch_W != nullptr) {
  //   WriteTriangleSurfaceMeshFieldLinearToVtk(
  //       "box_cylinder_field_intersection.vtk", contact_patch_W->e_MN(),
  //       "Pressure distribution on compliant-compliant contact patch.");
  // }
}

TEST_F(FieldIntersectionVizTest, BoxBoxThin) {
  const RigidTransformd X_MT{
      RotationMatrixd::MakeYRotation(M_PI_4),
      Vector3d(box_.width() / 2, 0, -0.8 * box_.height() / 2)};
  const VolumeMesh<double> box_thin_mesh4_M =
      TransformVolumeMesh(X_MT, box_thin_mesh4_T_);
  std::vector<double> field_values = box_thin_field4_T_.values();
  const VolumeMeshFieldLinear<double, double> box_thin_field4_M(
      std::move(field_values), &box_thin_mesh4_M);
  const Bvh<Obb, VolumeMesh<double>> box_thin_bvh4_M(box_thin_mesh4_M);
  // WriteVolumeMeshFieldLinearToVtk("box_thin_field4_M.vtk", box_thin_field4_M,
  //                                 "Pressure field in the thin box in frame "
  //                                 "M");

  const GeometryId first_id = GeometryId::get_new_id();
  const GeometryId second_id = GeometryId::get_new_id();
  const RigidTransformd X_WM = RigidTransformd::Identity();

  std::unique_ptr<ContactSurface<double>> contact_patch_W =
      ComputeContactSurfaceFromCompliantVolumes(
          first_id, box_field0_M_, box_bvh0_M_, X_WM,
          second_id, box_thin_field4_M, box_thin_bvh4_M, X_WM);

  ASSERT_TRUE(contact_patch_W != nullptr);

  // Visualization.
  // if (contact_patch_W != nullptr) {
  //   WriteTriangleSurfaceMeshFieldLinearToVtk(
  //       "box_box_thin_field_intersection.vtk", contact_patch_W->e_MN(),
  //       "Pressure distribution on compliant-compliant contact patch.");
  // }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
