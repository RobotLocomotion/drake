#include "drake/geometry/proximity/field_intersection.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_box_field.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/make_sphere_field.h"
#include "drake/geometry/proximity/mesh_deformer.h"
#include "drake/geometry/proximity/mesh_to_vtk.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/surface_mesh_field.h"

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
  WriteSurfaceMeshFieldLinearToVtk(vtk_file_name, field_on_polygon_M,
                                   description);
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
  Plane<double> plane_M =
      CalcEquilibriumPlane(first_element_in_field0, field0_M_,
                           first_element_in_field1, field1_N_, identity_X_MN);

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
  Vector3d normal_M;
  const vector<Vector3d> polygon_M = IntersectTetrahedra(
      first_element_in_field0, field0_M_, first_element_in_field1, field1_N_,
      identity_X_MN, &normal_M);

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
        // Get a mesh of an octahedron from a sphere specification by
        // specifying very coarse resolution hint.
        octahedron_mesh1_N_(MakeSphereVolumeMesh<double>(
            sphere_, 10 * sphere_.radius(),
            TessellationStrategy::kSingleInteriorVertex)),
        octahedron_field1_N_(MakeSpherePressureField<double>(
            sphere_, &octahedron_mesh1_N_, kOctahedronElasticModulus_)) {}

 protected:
  void SetUp() override {
    WriteVolumeMeshFieldLinearToVtk(
        "box_field0_M.vtk", box_field0_M_,
        "Pressure distribution on compliant tetrahedron in frame M");
    WriteVolumeMeshFieldLinearToVtk(
        "octahedron_field1_N.vtk", octahedron_field1_N_,
        "Pressure distribution on compliant tetrahedron in frame N");
    ASSERT_EQ(octahedron_mesh1_N_.num_elements(), 8);
  }

  // Geometry 0 and its field.
  const Box box_{0.06, 0.10, 0.14};  // 6cm-thick compliant pad.
  const double kBoxElasitcModulus_{1.0e5};
  const VolumeMesh<double> box_mesh0_M_;
  const VolumeMeshFieldLinear<double, double> box_field0_M_;

  // Geometry 1 and its field.
  const Sphere sphere_{0.03};  // 3cm-radius (6cm-diameter) finger tip.
  const double kOctahedronElasticModulus_{1.0e5};
  const VolumeMesh<double> octahedron_mesh1_N_;
  const VolumeMeshFieldLinear<double, double> octahedron_field1_N_;
};

TEST_F(FieldIntersectionHighLevelTest, FieldIntersection) {
  const math::RigidTransform<double> X_MN =
      math::RigidTransform<double>::Identity();
  std::unique_ptr<SurfaceMesh<double>> surface_MN_M;
  std::unique_ptr<SurfaceMeshFieldLinear<double, double>> e_MN_M;
  std::vector<Vector3<double>> grad_eM_Ms;
  std::vector<Vector3<double>> grad_eN_Ms;

  FieldIntersection(box_field0_M_, octahedron_field1_N_, X_MN, &surface_MN_M,
                    &e_MN_M, &grad_eM_Ms, &grad_eN_Ms);

  EXPECT_TRUE(surface_MN_M);

  // Visualization.
//  if (surface_MN_M) {
//    WriteSurfaceMeshFieldLinearToVtk(
//        "box_octahedron_field_intersection.vtk", *e_MN_M,
//        "Pressure distribution on compliant-compliant contact patch.");
//  }
}

TEST_F(FieldIntersectionHighLevelTest, FieldIntersection_BoxBall) {
  const double resolution = sphere_.radius() / 4.0;
  const VolumeMesh<double> ball_mesh1_N = MakeSphereVolumeMesh<double>(
      sphere_, resolution, TessellationStrategy::kSingleInteriorVertex);
  const double kBallElasticModulus = 1.0e5;
  const VolumeMeshFieldLinear<double, double> ball_field1_N =
      MakeSpherePressureField<double>(sphere_, &ball_mesh1_N,
                                      kBallElasticModulus);

  const math::RigidTransform<double> X_MN(Vector3d(0.03, 0, 0));

  std::vector<VolumeVertex<double>> vertices_M;
  for (const VolumeVertex<double>& p_NV : ball_mesh1_N.vertices()) {
    vertices_M.emplace_back(X_MN * p_NV.r_MV());
  }
  std::vector<VolumeElement> tetrahedra = ball_mesh1_N.tetrahedra();
  const VolumeMesh<double> ball_mesh1_M(std::move(tetrahedra),
                                        std::move(vertices_M));
  std::vector<double> field_values = ball_field1_N.values();
  const VolumeMeshFieldLinear<double, double> ball_field1_M(
      ball_field1_N.name(), std::move(field_values), &ball_mesh1_M);

  WriteVolumeMeshFieldLinearToVtk("ball_field1_M.vtk", ball_field1_M,
                                  "Pressure field of a ball");

  std::unique_ptr<SurfaceMesh<double>> surface_MN_M;
  std::unique_ptr<SurfaceMeshFieldLinear<double, double>> e_MN_M;
  std::vector<Vector3<double>> grad_eM_Ms;
  std::vector<Vector3<double>> grad_eN_Ms;

  FieldIntersection(box_field0_M_, ball_field1_N, X_MN, &surface_MN_M, &e_MN_M,
                    &grad_eM_Ms, &grad_eN_Ms);

  EXPECT_TRUE(surface_MN_M);

  // Visualization.
  if (surface_MN_M) {
    WriteSurfaceMeshFieldLinearToVtk(
        "box_ball_field_intersection.vtk", *e_MN_M,
        "Pressure distribution on compliant-compliant contact patch.");
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
