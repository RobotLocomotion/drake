// This file is not intended to merge into master. It is here temporarily
// for human to run the tests and verify result in VTK files visually in
// Paraview. Its result is suitable for communication and presentation as
// projects develop.

#include <memory>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/field_intersection.h"
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
        contact_file_name, "Pressure[Pa]", contact_W.tri_e_MN(),
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
          second_id, ball_field1_N_, ball_bvh1_N_, X_WN,
          HydroelasticContactRepresentation::kTriangle);

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
            second_id, ball_field_W, ball_bvh_W, RigidTransformd::Identity(),
            HydroelasticContactRepresentation::kTriangle);

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
          second_id, box2_field2_M, box2_bvh2_M, X_WM,
          HydroelasticContactRepresentation::kTriangle);

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
          second_id, clone_box_field_M, clone_box_bvh_M, X_WM,
          HydroelasticContactRepresentation::kTriangle);

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
          second_id, cylinder_field3_C_, cylinder_bvh3_C_, X_WC,
          HydroelasticContactRepresentation::kTriangle);

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
          second_id, box_thin_field4_M, box_thin_bvh4_M, X_WM,
          HydroelasticContactRepresentation::kTriangle);

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
