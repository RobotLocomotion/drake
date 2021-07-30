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
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/surface_mesh_field.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using std::make_unique;
using std::move;
using std::unique_ptr;
using std::vector;
using math::RigidTransformd;
using math::RotationMatrixd;

#if 0
void WritePolygonToVtkAsSurfaceField(
    const vector<Vector3d>& polygon_M, const Vector3d& normal,
    VolumeElementIndex tetrahedron,
    const VolumeMeshFieldLinear<double, double>& field_M,
    const std::string& vtk_file_name, const std::string& description) {
  vector<SurfaceVertexIndex> polygon;
  vector<SurfaceVertex<double>> vertices_M;
  for (const Vector3d& p_MV : polygon_M) {
    polygon.emplace_back(vertices_M.size());
    vertices_M.emplace_back(p_MV);
  }
  vector<SurfaceFace> faces;
  AddPolygonToMeshData(polygon, normal, &faces, &vertices_M);
  vector<double> pressure_values;
  for (const SurfaceVertex<double>& v_M : vertices_M) {
    pressure_values.push_back(
        field_M.EvaluateCartesian(tetrahedron, v_M.r_MV()));
  }
  SurfaceMesh<double> polygon_mesh_M(move(faces), move(vertices_M));
  SurfaceMeshFieldLinear<double, double> field_on_polygon_M(
      "pressure(Pa)", move(pressure_values), &polygon_mesh_M, false);
  // WriteSurfaceMeshFieldLinearToVtk(vtk_file_name, field_on_polygon_M,
  //                                 description);
}
#endif

class FieldIntersectionLowLevelTest : public ::testing::Test {
 public:
  FieldIntersectionLowLevelTest()
      : mesh0_M_(std::vector<VolumeElement>{VolumeElement(
                     VolumeVertexIndex(0), VolumeVertexIndex(1),
                     VolumeVertexIndex(2), VolumeVertexIndex(3))},
                 std::vector<VolumeVertex<double>>{
                     VolumeVertex<double>(2, 0, 2),
                     VolumeVertex<double>(-2, 0, 2),
                     VolumeVertex<double>(0, 2, -2),
                     VolumeVertex<double>(0, -2, -2)}),
        field0_M_("Pressure(Pa)",
                  // (4 + x + y + z)e4
                  vector<double>({8e4, 4e4, 4e4, 0}), &mesh0_M_),
        mesh1_N_(std::vector<VolumeElement>{VolumeElement(
                     VolumeVertexIndex(0), VolumeVertexIndex(1),
                     VolumeVertexIndex(2), VolumeVertexIndex(3))},
                 std::vector<VolumeVertex<double>>{
                     VolumeVertex<double>(1.5, 1.5, 2.5),
                     VolumeVertex<double>(-1.5, -1.5, 2.5),
                     VolumeVertex<double>(-1.5, 1.5, -2.5),
                     VolumeVertex<double>(1.5, -1.5, -2.5)}),
        field1_N_("Pressure(Pa)",
                  // (4 + x + y)e4
                  vector<double>({7e4, 1e4, 4e4, 4e4}), &mesh1_N_) {}

 protected:
  void SetUp() override {
//    WriteVolumeMeshFieldLinearToVtk(
//        "field0_M.vtk", field0_M_,
//        "Pressure distribution on compliant tetrahedron in frame M");
//    WriteVolumeMeshFieldLinearToVtk(
//        "field1_N.vtk", field1_N_,
//        "Pressure distribution on compliant tetrahedron in frame N");
    ASSERT_TRUE(
        CompareMatrices(field0_M_.EvaluateGradient(VolumeElementIndex{0}),
                        Vector3d(1, 1, 1) * 1e4, 1e-10));
    ASSERT_TRUE(
        CompareMatrices(field1_N_.EvaluateGradient(VolumeElementIndex{0}),
                        Vector3d(1, 1, 0) * 1e4, 1e-10));
  }

  const VolumeMesh<double> mesh0_M_;
  const VolumeMeshFieldLinear<double, double> field0_M_;
  const VolumeMesh<double> mesh1_N_;
  const VolumeMeshFieldLinear<double, double> field1_N_;
};

TEST_F(FieldIntersectionLowLevelTest, CalcEquilibriumPlane) {
  const VolumeElementIndex first_element_in_field0{0};
  const VolumeElementIndex first_element_in_field1{0};
  const RigidTransformd identity_X_MN = RigidTransformd::Identity();
  Plane<double> plane_M{Vector3d::UnitZ(), Vector3d::Zero()};
  bool success = CalcEquilibriumPlane(first_element_in_field0, field0_M_,
                                      first_element_in_field1, field1_N_,
                                      identity_X_MN, &plane_M);
  ASSERT_TRUE(success);

  // Verify that the equilibrium plane is the X-Y plane in frame M.
  // This is the because the two fields are:
  //     f0(x,y,z) = (4 + x + y + z)e4
  //     f1(x,y,z) = (4 + x + y)e4,
  // so the plane of f0 = f1 is:
  //     (4 + x + y + z)e4 - (4 + x + y)e4 = 0 ==> The plane z = 0.
  EXPECT_TRUE(CompareMatrices(plane_M.normal(), Vector3d::UnitZ(), 1e-14));
  EXPECT_NEAR(plane_M.CalcHeight<double>(Vector3d::Zero()), 0.0, 1e-14);
}

TEST_F(FieldIntersectionLowLevelTest, IntersectPlaneTetrahedron) {
  const VolumeElementIndex first_element_in_field0{0};
  // Define the X-Y plane in frame M.
  const Plane<double> plane_M(Vector3d::UnitZ(), Vector3d::Zero());

  std::vector<Vector3d> polygon_M;
  IntersectPlaneTetrahedron(first_element_in_field0, field0_M_, plane_M,
                            &polygon_M);

  EXPECT_EQ(polygon_M.size(), 4);

//  // Visualization.
//  if (polygon_M.size() != 0) {
//    WritePolygonToVtkAsSurfaceField(
//        polygon_M, plane_M.normal(), first_element_in_field0, field0_M_,
//        "polygon_of_plane_tetrahedron.vtk",
//        "Pressure on the polygon of plane-tetrahedron intersection");
//  }
}

TEST_F(FieldIntersectionLowLevelTest, IntersectTetrahedra) {
  const VolumeElementIndex first_element_in_field0{0};
  const VolumeElementIndex first_element_in_field1{0};
  const RigidTransformd identity_X_MN = RigidTransformd::Identity();
  Plane<double> plane_M{Vector3d::UnitZ(), Vector3d::Zero()};
  bool success = CalcEquilibriumPlane(first_element_in_field0, field0_M_,
                                      first_element_in_field1, field1_N_,
                                      identity_X_MN, &plane_M);
  ASSERT_TRUE(success);

  Vector3d normal_M;
  const vector<Vector3d> polygon_M = IntersectTetrahedra(
      first_element_in_field0, field0_M_, first_element_in_field1, field1_N_,
      identity_X_MN, plane_M);

  EXPECT_EQ(polygon_M.size(), 8);

//  // Visualization.
//  if (polygon_M.size() != 0) {
//    WritePolygonToVtkAsSurfaceField(
//        polygon_M, normal_M, first_element_in_field0, field0_M_,
//        "field_intersection_polygon.vtk",
//        "Pressure distribution on compliant-compliant contact polygon");
//  }
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
  std::unique_ptr<SurfaceMesh<double>> surface_MN_M;
  std::unique_ptr<SurfaceMeshFieldLinear<double, double>> e_MN_M;
  std::vector<Vector3<double>> grad_eM_Ms;
  std::vector<Vector3<double>> grad_eN_Ms;

  FieldIntersection(box_field0_M_, box_bvh0_M_,
                    octahedron_field1_N_, octahedron_bvh1_N_, X_MN,
                    ContactPolygonRepresentation::kCentroidSubdivision,
                    &surface_MN_M, &e_MN_M, &grad_eM_Ms, &grad_eN_Ms);

  EXPECT_TRUE(surface_MN_M);

  // Visualization.
//  if (surface_MN_M) {
//    WriteSurfaceMeshFieldLinearToVtk(
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
      second_id, octahedron_field1_N_, octahedron_bvh1_N_, X_WN,
      ContactPolygonRepresentation::kCentroidSubdivision);
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
    std::vector<VolumeVertex<double>> vertices_M;
    for (const VolumeVertex<double>& p_NV : mesh_N.vertices()) {
      vertices_M.emplace_back(X_MN * p_NV.r_MV());
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
    VolumeMeshFieldLinear<double, double> copy_field_M(ball_field1_N_.name(),
                                                       std::move(field_values),
                                                       transformed_mesh->get());
    return copy_field_M;
  }

  void WriteTwoFieldsAndTheirContact(
      const VolumeMeshFieldLinear<double, double>& field0_W,
      const std::string& file_name0,
      const VolumeMeshFieldLinear<double, double>& field1_W,
      const std::string& file_name1, const ContactSurface<double>& contact_W,
      const std::string& contact_file_name) {
    WriteVolumeMeshFieldLinearToVtk(file_name0, field0_W,
                                    "Pressure field in " + file_name0);
    WriteVolumeMeshFieldLinearToVtk(file_name1, field1_W,
                                    "Pressure field in " + file_name1);
    WriteSurfaceMeshFieldLinearToVtk(
        contact_file_name, contact_W.e_MN(),
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
      ball_field1_N_.name(), std::move(field_values), &ball_mesh1_M);

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
          ContactPolygonRepresentation::kCentroidSubdivision);

  ASSERT_TRUE(contact_patch_W != nullptr);

  // Visualization.
  // if (contact_patch_W != nullptr) {
  //   WriteSurfaceMeshFieldLinearToVtk(
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
            ContactPolygonRepresentation::kCentroidSubdivision);

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
      box2_field2_L_.name(), std::move(field_values), &box2_mesh2_M);
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
          ContactPolygonRepresentation::kCentroidSubdivision);

  ASSERT_TRUE(contact_patch_W != nullptr);

  // Visualization.
  // if (contact_patch_W != nullptr) {
  //   WriteSurfaceMeshFieldLinearToVtk(
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
      box_field0_M_.name(), std::move(field_values), &clone_box_mesh_M);
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
          ContactPolygonRepresentation::kCentroidSubdivision);

  ASSERT_TRUE(contact_patch_W != nullptr);

  // Visualization.
  // if (contact_patch_W != nullptr) {
  //   WriteSurfaceMeshFieldLinearToVtk(
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
      cylinder_field3_C_.name(), std::move(field_values), &cylinder_mesh3_M);
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
          ContactPolygonRepresentation::kCentroidSubdivision);

  ASSERT_TRUE(contact_patch_W != nullptr);

  // Visualization.
  // if (contact_patch_W != nullptr) {
  //   WriteSurfaceMeshFieldLinearToVtk(
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
      box_thin_field4_T_.name(), std::move(field_values), &box_thin_mesh4_M);
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
          ContactPolygonRepresentation::kCentroidSubdivision);

  ASSERT_TRUE(contact_patch_W != nullptr);

  // Visualization.
  // if (contact_patch_W != nullptr) {
  //   WriteSurfaceMeshFieldLinearToVtk(
  //       "box_box_thin_field_intersection.vtk", contact_patch_W->e_MN(),
  //       "Pressure distribution on compliant-compliant contact patch.");
  // }
}

TEST_F(FieldIntersectionVizTest, HackCompliantBoxRigidBall) {
  // TODO(DamrongGuoy): Remove #include volume_to_surface_mesh.h when we
  //  don't need this.
  const SurfaceMesh<double> ball_surface_mesh1_M =
      ConvertVolumeToSurfaceMesh(TransformVolumeMesh(X_MN_, ball_mesh1_N_));
  const Bvh<Obb, SurfaceMesh<double>> ball_surface_bvh1_M(ball_surface_mesh1_M);

//  WriteSurfaceMeshToVtk("rigid_ball_surface_mesh1_M.vtk",
//                        ball_surface_mesh1_M,
//                        "Surface mesh of a rigid ball in frame M");

  const GeometryId first_id = GeometryId::get_new_id();
  const GeometryId second_id = GeometryId::get_new_id();

  // TODO(DamrongGuoy): Remove #include mesh_intersection.h when we don't
  //  need this.
  std::unique_ptr<ContactSurface<double>> contact_M =
      ComputeContactSurfaceFromSoftVolumeRigidSurface(
          first_id, box_field0_M_, box_bvh0_M_, RigidTransformd::Identity(),
          second_id, ball_surface_mesh1_M, ball_surface_bvh1_M,
          RigidTransformd::Identity(),
          ContactPolygonRepresentation::kCentroidSubdivision);

  EXPECT_TRUE(contact_M);

//  // Visualization.
//  if (contact_M) {
//    WriteSurfaceMeshFieldLinearToVtk(
//        "compliant_box_rigid_ball_contact.vtk", contact_M->e_MN(),
//        "Pressure distribution on contact patch between a compliant "
//        "box and a rigid ball");
//  }
}

TEST_F(FieldIntersectionVizTest, HackRigidBoxCompliantBall) {
  // TODO(DamrongGuoy): Remove #include volume_to_surface_mesh.h when we
  //  don't need this.
  const SurfaceMesh<double> box_surface_mesh0_M =
      ConvertVolumeToSurfaceMesh(box_mesh0_M_);
  Bvh<Obb, SurfaceMesh<double>> box_surface_bvh0_M(box_surface_mesh0_M);
//  WriteSurfaceMeshToVtk("rigid_box_surface_mesh0_M.vtk", box_surface_mesh0_M,
//                        "Surface mesh of a rigid box in frame M");

  VolumeMesh<double> ball_mesh1_M = TransformVolumeMesh(X_MN_, ball_mesh1_N_);
  std::vector<double> field_values = ball_field1_N_.values();
  const VolumeMeshFieldLinear<double, double> ball_field1_M(
      ball_field1_N_.name(), std::move(field_values), &ball_mesh1_M);
  Bvh<Obb, VolumeMesh<double>> ball_bvh1_M(ball_mesh1_M);

  const GeometryId first_id = GeometryId::get_new_id();
  const GeometryId second_id = GeometryId::get_new_id();

  // TODO(DamrongGuoy): Remove #include mesh_intersection.h when we don't
  //  need this.
  std::unique_ptr<ContactSurface<double>> contact_M =
      ComputeContactSurfaceFromSoftVolumeRigidSurface(
          first_id, ball_field1_M, ball_bvh1_M, RigidTransformd::Identity(),
          second_id, box_surface_mesh0_M, box_surface_bvh0_M,
          RigidTransformd::Identity(),
          ContactPolygonRepresentation::kCentroidSubdivision);

  EXPECT_TRUE(contact_M);

//  // Visualization.
//  if (contact_M) {
//    WriteSurfaceMeshFieldLinearToVtk(
//        "rigid_box_compliant_ball_contact.vtk", contact_M->e_MN(),
//        "Pressure distribution on contact patch between a rigid "
//        "box and a compliant ball");
//  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
