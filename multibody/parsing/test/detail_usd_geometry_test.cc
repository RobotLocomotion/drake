// TODO(#20898) Remove this #if wrapper when USD is a first-class dependency.
#if WITH_USD

#include "drake/multibody/parsing/detail_usd_geometry.h"

#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/capsule.h"
#include "pxr/usd/usdGeom/cube.h"
#include "pxr/usd/usdGeom/cylinder.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/sphere.h"
#include "pxr/usd/usdGeom/xform.h"
#include "pxr/usd/usdPhysics/massAPI.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/is_approx_equal_abstol.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/multibody/parsing/detail_usd_parser.h"

namespace drake {
namespace multibody {
// We don't open the `internal` namespace in this file so that function calls
// to the USD parser has to spell the `internal::` prefix. We do this in order
// to make it easier to identify function calls that are being tested.
namespace {

class UsdGeometryTest : public test::DiagnosticPolicyTestBase {
 public:
  UsdGeometryTest() {
    internal::UsdParserWrapper::InitializeOpenUsdLibrary();

    stage_ = pxr::UsdStage::CreateInMemory();
    meters_per_unit_ = 0.01;
    stage_up_axis_ = pxr::TfToken("Z");
  }

 protected:
  pxr::UsdStageRefPtr stage_;
  pxr::TfToken stage_up_axis_;
  double meters_per_unit_;
};

TEST_F(UsdGeometryTest, BoxParsingTest) {
  // Case: All inputs are valid.
  pxr::UsdGeomCube box = pxr::UsdGeomCube::Define(stage_, pxr::SdfPath("/Box"));

  const double size = 49.0;
  pxr::UsdAttribute size_attribute = box.CreateSizeAttr();
  size_attribute.Set(size);

  const pxr::GfVec3d scale_factor = pxr::GfVec3d(0.4, 0.5, 0.6);
  pxr::UsdGeomXformOp scale_op =
      box.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble);
  scale_op.Set(scale_factor);

  const Eigen::Vector3d correct_dimension =
      internal::UsdVec3dToEigen(scale_factor) * size * meters_per_unit_;
  std::optional<Eigen::Vector3d> parsed_dimension = internal::GetBoxDimension(
      box.GetPrim(), meters_per_unit_, diagnostic_policy_);
  EXPECT_EQ(parsed_dimension.value(), correct_dimension);

  std::unique_ptr<geometry::Shape> shape = internal::CreateGeometryBox(
      box.GetPrim(), meters_per_unit_, diagnostic_policy_);
  ASSERT_TRUE(shape != nullptr);
  geometry::Box& shape_box = dynamic_cast<geometry::Box&>(*shape);
  EXPECT_EQ(shape_box.size(), correct_dimension);

  const float mass = 2.71;
  pxr::UsdPhysicsMassAPI mass_api =
      pxr::UsdPhysicsMassAPI::Apply(box.GetPrim());
  pxr::UsdAttribute mass_attribute = mass_api.CreateMassAttr();
  mass_attribute.Set(mass);
  std::optional<SpatialInertia<double>> inertia =
      internal::CreateSpatialInertiaForBox(box.GetPrim(), meters_per_unit_,
                                           diagnostic_policy_);
  EXPECT_EQ(mass, static_cast<float>(inertia.value().get_mass()));

  // Case: Input Prim is not an UsdGeomCube.
  pxr::UsdPrim empty_prim =
      stage_->DefinePrim(pxr::SdfPath("/InvalidType"), pxr::TfToken(""));
  parsed_dimension = internal::GetBoxDimension(empty_prim, meters_per_unit_,
                                               diagnostic_policy_);
  EXPECT_FALSE(parsed_dimension.has_value());
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*Failed to cast the Prim at .* into an UsdGeomCube.*"));

  // Case: Input Prim is missing the size attribute.
  box.GetPrim().RemoveProperty(size_attribute.GetName());
  parsed_dimension = internal::GetBoxDimension(box.GetPrim(), meters_per_unit_,
                                               diagnostic_policy_);
  EXPECT_FALSE(parsed_dimension.has_value());
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*Failed to read the 'size' attribute of the Prim at.*"));
}

TEST_F(UsdGeometryTest, EllipsoidParsingTest) {
  // Case: All inputs are valid.
  pxr::UsdGeomSphere ellipsoid =
      pxr::UsdGeomSphere::Define(stage_, pxr::SdfPath("/Ellipsoid"));

  const double radius = 17.0;
  pxr::UsdAttribute radius_attribute = ellipsoid.CreateRadiusAttr();
  radius_attribute.Set(radius);

  const pxr::GfVec3d scale_factor = pxr::GfVec3d(0.6, 1.1, 2.9);
  pxr::UsdGeomXformOp scale_op =
      ellipsoid.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble);
  scale_op.Set(scale_factor);

  const Eigen::Vector3d correct_dimension =
      internal::UsdVec3dToEigen(scale_factor) * radius * meters_per_unit_;
  std::optional<Eigen::Vector3d> parsed_dimension =
      internal::GetEllipsoidDimension(ellipsoid.GetPrim(), meters_per_unit_,
                                      diagnostic_policy_);
  EXPECT_EQ(parsed_dimension.value(), correct_dimension);

  std::unique_ptr<geometry::Shape> shape = internal::CreateGeometryEllipsoid(
      ellipsoid.GetPrim(), meters_per_unit_, diagnostic_policy_);
  ASSERT_TRUE(shape != nullptr);
  geometry::Ellipsoid& shape_ellipsoid =
      dynamic_cast<geometry::Ellipsoid&>(*shape);
  const Eigen::Vector3d shape_dimension = Eigen::Vector3d(
      shape_ellipsoid.a(), shape_ellipsoid.b(), shape_ellipsoid.c());
  EXPECT_EQ(shape_dimension, correct_dimension);

  const float mass = 77.241;
  pxr::UsdPhysicsMassAPI mass_api =
      pxr::UsdPhysicsMassAPI::Apply(ellipsoid.GetPrim());
  pxr::UsdAttribute mass_attribute = mass_api.CreateMassAttr();
  mass_attribute.Set(mass);
  std::optional<SpatialInertia<double>> inertia =
      internal::CreateSpatialInertiaForEllipsoid(
          ellipsoid.GetPrim(), meters_per_unit_, diagnostic_policy_);
  EXPECT_EQ(mass, static_cast<float>(inertia.value().get_mass()));

  // Case: Input Prim is not an UsdGeomSphere.
  pxr::UsdPrim empty_prim =
      stage_->DefinePrim(pxr::SdfPath("/InvalidType"), pxr::TfToken(""));
  parsed_dimension = internal::GetEllipsoidDimension(
      empty_prim, meters_per_unit_, diagnostic_policy_);
  EXPECT_FALSE(parsed_dimension.has_value());
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*Failed to cast the Prim at .* into an UsdGeomSphere.*"));

  // Case: Input Prim is missing the radius attribute.
  ellipsoid.GetPrim().RemoveProperty(radius_attribute.GetName());
  parsed_dimension = internal::GetEllipsoidDimension(
      ellipsoid.GetPrim(), meters_per_unit_, diagnostic_policy_);
  EXPECT_FALSE(parsed_dimension.has_value());
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*Failed to read the 'radius' attribute of the Prim at.*"));
}

TEST_F(UsdGeometryTest, CylinderParsingTest) {
  // Case: All inputs are valid.
  pxr::UsdGeomCylinder cylinder =
      pxr::UsdGeomCylinder::Define(stage_, pxr::SdfPath("/Cylinder"));

  const double radius = 62.0;
  const double height = 199.0;
  const pxr::TfToken axis = pxr::TfToken("Z");
  pxr::UsdAttribute radius_attribute = cylinder.CreateRadiusAttr();
  pxr::UsdAttribute height_attribute = cylinder.CreateHeightAttr();
  pxr::UsdAttribute axis_attribute = cylinder.CreateAxisAttr();
  radius_attribute.Set(radius);
  height_attribute.Set(height);
  axis_attribute.Set(axis);

  const pxr::GfVec3d scale_factor = pxr::GfVec3d(0.7, 0.7, 0.9);
  pxr::UsdGeomXformOp scale_op =
      cylinder.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble);
  scale_op.Set(scale_factor);

  const double correct_radius = scale_factor[0] * radius * meters_per_unit_;
  const double correct_height = scale_factor[2] * height * meters_per_unit_;
  const Eigen::Vector2d correct_dimension =
      Eigen::Vector2d(correct_radius, correct_height);
  std::optional<Eigen::Vector2d> parsed_dimension =
      internal::GetCylinderDimension(cylinder.GetPrim(), meters_per_unit_,
                                     stage_up_axis_, diagnostic_policy_);
  EXPECT_EQ(parsed_dimension.value(), correct_dimension);

  std::unique_ptr<geometry::Shape> shape = internal::CreateGeometryCylinder(
      cylinder.GetPrim(), meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  ASSERT_TRUE(shape != nullptr);
  geometry::Cylinder& shape_cylinder =
      dynamic_cast<geometry::Cylinder&>(*shape);
  Eigen::Vector2d shape_dimension =
      Eigen::Vector2d(shape_cylinder.radius(), shape_cylinder.length());
  EXPECT_EQ(shape_dimension, correct_dimension);

  const float mass = 152.0;
  pxr::UsdPhysicsMassAPI mass_api =
      pxr::UsdPhysicsMassAPI::Apply(cylinder.GetPrim());
  pxr::UsdAttribute mass_attribute = mass_api.CreateMassAttr();
  mass_attribute.Set(mass);
  std::optional<SpatialInertia<double>> inertia =
      internal::CreateSpatialInertiaForCylinder(
          cylinder.GetPrim(), meters_per_unit_, stage_up_axis_,
          diagnostic_policy_);
  EXPECT_EQ(mass, static_cast<float>(inertia.value().get_mass()));

  // Case: Input Prim is not an UsdGeomCylinder.
  pxr::UsdPrim empty_prim =
      stage_->DefinePrim(pxr::SdfPath("/InvalidType"), pxr::TfToken(""));
  parsed_dimension = internal::GetCylinderDimension(
      empty_prim, meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(parsed_dimension.has_value());
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*Failed to cast the Prim at .* into an UsdGeomCylinder.*"));

  // Case: Input Prim is missing the height attribute.
  cylinder.GetPrim().RemoveProperty(height_attribute.GetName());
  parsed_dimension = internal::GetCylinderDimension(
      cylinder.GetPrim(), meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(parsed_dimension.has_value());
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*Failed to read the 'height' attribute of the Prim at.*"));

  // Case: Input Prim is missing the radius attribute.
  cylinder.GetPrim().RemoveProperty(radius_attribute.GetName());
  parsed_dimension = internal::GetCylinderDimension(
      cylinder.GetPrim(), meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(parsed_dimension.has_value());
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*Failed to read the 'radius' attribute of the Prim at.*"));

  // Case: Cylinder has different scaling in X and Y axis.
  scale_op.Set(pxr::GfVec3d(0.8, 0.7, 0.9));
  parsed_dimension = internal::GetCylinderDimension(
      cylinder.GetPrim(), meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(parsed_dimension.has_value());
  EXPECT_THAT(
      TakeError(),
      ::testing::MatchesRegex(
          ".*The cylinder at .* has different scaling in X and Y axis.*"));

  // Case: The axis attribute of the cylinder is invalid.
  axis_attribute.Set(pxr::TfToken("A"));
  inertia = internal::CreateSpatialInertiaForCylinder(
      cylinder.GetPrim(), meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(inertia.has_value());
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(".*The cylinder at .* is not upright.*"));
  EXPECT_THAT(
      TakeError(),
      ::testing::MatchesRegex(".*The axis of the geometry at .* is invalid.*"));

  // Case: The axis attribute of the cylinder does not exist.
  cylinder.GetPrim().RemoveProperty(axis_attribute.GetName());
  parsed_dimension = internal::GetCylinderDimension(
      cylinder.GetPrim(), meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(parsed_dimension.has_value());
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*Failed to read the 'axis' attribute of the Prim at.*"));
}

TEST_F(UsdGeometryTest, CapsuleParsingTest) {
  // Case: All inputs are valid.
  pxr::UsdGeomCapsule capsule =
      pxr::UsdGeomCapsule::Define(stage_, pxr::SdfPath("/Capsule"));

  const double radius = 101;
  const double height = 45;
  const pxr::TfToken axis = pxr::TfToken("Z");
  pxr::UsdAttribute radius_attribute = capsule.CreateRadiusAttr();
  pxr::UsdAttribute height_attribute = capsule.CreateHeightAttr();
  pxr::UsdAttribute axis_attribute = capsule.CreateAxisAttr();
  radius_attribute.Set(radius);
  height_attribute.Set(height);
  axis_attribute.Set(axis);

  const pxr::GfVec3d scale_factor = pxr::GfVec3d(0.7, 0.7, 0.9);
  pxr::UsdGeomXformOp scale_op =
      capsule.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble);
  scale_op.Set(scale_factor);

  const double correct_radius = scale_factor[0] * radius * meters_per_unit_;
  const double correct_height = scale_factor[2] * height * meters_per_unit_;
  const Eigen::Vector2d correct_dimension =
      Eigen::Vector2d(correct_radius, correct_height);
  std::optional<Eigen::Vector2d> parsed_dimension =
      internal::GetCapsuleDimension(capsule.GetPrim(), meters_per_unit_,
                                    stage_up_axis_, diagnostic_policy_);
  EXPECT_EQ(parsed_dimension.value(), correct_dimension);

  std::unique_ptr<geometry::Shape> shape = internal::CreateGeometryCapsule(
      capsule.GetPrim(), meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  ASSERT_TRUE(shape != nullptr);
  geometry::Capsule& shape_capsule = dynamic_cast<geometry::Capsule&>(*shape);
  const Eigen::Vector2d shape_dimension =
      Eigen::Vector2d(shape_capsule.radius(), shape_capsule.length());
  EXPECT_EQ(shape_dimension, correct_dimension);

  const float mass = 152.0;
  pxr::UsdPhysicsMassAPI mass_api =
      pxr::UsdPhysicsMassAPI::Apply(capsule.GetPrim());
  pxr::UsdAttribute mass_attribute = mass_api.CreateMassAttr();
  mass_attribute.Set(mass);
  std::optional<SpatialInertia<double>> inertia =
      internal::CreateSpatialInertiaForCapsule(capsule.GetPrim(),
                                               meters_per_unit_, stage_up_axis_,
                                               diagnostic_policy_);
  EXPECT_EQ(mass, static_cast<float>(inertia.value().get_mass()));

  // Case: Input Prim is not an UsdGeomCapsule.
  pxr::UsdPrim empty_prim =
      stage_->DefinePrim(pxr::SdfPath("/InvalidType"), pxr::TfToken(""));
  parsed_dimension = internal::GetCapsuleDimension(
      empty_prim, meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(parsed_dimension.has_value());
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*Failed to cast the Prim at .* into an UsdGeomCapsule.*"));

  // Case: Input Prim is missing the height attribute.
  capsule.GetPrim().RemoveProperty(height_attribute.GetName());
  parsed_dimension = internal::GetCapsuleDimension(
      capsule.GetPrim(), meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(parsed_dimension.has_value());
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*Failed to read the 'height' attribute of the Prim at.*"));

  // Case: Input Prim is missing the radius attribute.
  capsule.GetPrim().RemoveProperty(radius_attribute.GetName());
  parsed_dimension = internal::GetCapsuleDimension(
      capsule.GetPrim(), meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(parsed_dimension.has_value());
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*Failed to read the 'radius' attribute of the Prim at.*"));

  // Case: Capsule has different scaling in X and Y axis.
  scale_op.Set(pxr::GfVec3d(0.8, 0.7, 0.9));
  parsed_dimension = internal::GetCapsuleDimension(
      capsule.GetPrim(), meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(parsed_dimension.has_value());
  EXPECT_THAT(
      TakeError(),
      ::testing::MatchesRegex(
          ".*The capsule at .* has different scaling in X and Y axis.*"));

  // Case: The axis of the capsule is not the same as the up-axis of the stage.
  axis_attribute.Set(pxr::TfToken("Y"));
  parsed_dimension = internal::GetCapsuleDimension(
      capsule.GetPrim(), meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(parsed_dimension.has_value());
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(".*The capsule at .* is not upright.*"));
}

TEST_F(UsdGeometryTest, MeshParsingTest) {
  // Case: All inputs are valid.
  pxr::UsdGeomMesh mesh =
      pxr::UsdGeomMesh::Define(stage_, pxr::SdfPath("/Mesh"));

  // The following specifies an octahedron mesh.
  pxr::VtArray<pxr::GfVec3f> vertices = pxr::VtArray<pxr::GfVec3f>{
      pxr::GfVec3f(1, 0, 0), pxr::GfVec3f(0, -1, 0), pxr::GfVec3f(-1, 0, 0),
      pxr::GfVec3f(0, 1, 0), pxr::GfVec3f(0, 0, 1),  pxr::GfVec3f(0, 0, -1)};
  pxr::VtArray<int> face_vertex_counts =
      pxr::VtArray<int>{3, 3, 3, 3, 3, 3, 3, 3};
  pxr::VtArray<int> face_vertex_indices = pxr::VtArray<int>{
      1, 0, 4, 2, 1, 4, 3, 2, 4, 0, 3, 4, 0, 1, 5, 1, 2, 5, 2, 3, 5, 3, 0, 5};
  const double scale_factor = 129.2;

  pxr::UsdAttribute points_attribute = mesh.CreatePointsAttr();
  pxr::UsdAttribute face_counts_attribute = mesh.CreateFaceVertexCountsAttr();
  pxr::UsdAttribute indices_attribute = mesh.CreateFaceVertexIndicesAttr();
  points_attribute.Set(vertices);
  face_counts_attribute.Set(face_vertex_counts);
  indices_attribute.Set(face_vertex_indices);
  pxr::UsdGeomXformOp scale_op =
      mesh.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble);

  scale_op.Set(pxr::GfVec3d(scale_factor, scale_factor, scale_factor));
  std::unique_ptr<geometry::Shape> shape = internal::CreateGeometryMesh(
      "octahedron.obj", mesh.GetPrim(), meters_per_unit_, diagnostic_policy_);
  ASSERT_TRUE(shape != nullptr);
  geometry::Mesh& shape_mesh = dynamic_cast<geometry::Mesh&>(*shape);
  EXPECT_EQ(shape_mesh.scale(), scale_factor);

  // Check whether Drake can sucessfully parse that file by computing the
  // convex hull of the octahedron mesh.
  const geometry::PolygonSurfaceMesh<double>& convex_hull =
      shape_mesh.GetConvexHull();
  EXPECT_EQ(convex_hull.num_faces(), 8);

  // Case: Input Prim is not an UsdGeomMesh.
  pxr::UsdPrim empty_prim =
      stage_->DefinePrim(pxr::SdfPath("/InvalidType"), pxr::TfToken(""));
  shape = internal::CreateGeometryMesh("invalid_prim.obj", empty_prim,
                                       meters_per_unit_, diagnostic_policy_);
  EXPECT_TRUE(shape == nullptr);
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*Failed to cast the Prim at .* into an UsdGeomMesh.*"));

  // Case: The UsdGeomMesh Prim does not have indices attribute.
  mesh.GetPrim().RemoveProperty(indices_attribute.GetName());
  shape = internal::CreateGeometryMesh("no_indices.obj", mesh.GetPrim(),
                                       meters_per_unit_, diagnostic_policy_);
  EXPECT_TRUE(shape == nullptr);
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
                               ".*Failed to read the 'faceVertexIndices' "
                               "attribute of the Prim at.*"));

  // Case: The UsdGeomMesh Prim does not have points attribute.
  mesh.GetPrim().RemoveProperty(points_attribute.GetName());
  shape = internal::CreateGeometryMesh("no_points.obj", mesh.GetPrim(),
                                       meters_per_unit_, diagnostic_policy_);
  EXPECT_TRUE(shape == nullptr);
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*Failed to read the 'points' attribute of the Prim at.*"));

  // Case: The face count attribute of the Prim contains elements other than 3.
  face_vertex_counts[0] = 4;
  face_counts_attribute.Set(face_vertex_counts);
  shape = internal::CreateGeometryMesh("quad_mesh.obj", mesh.GetPrim(),
                                       meters_per_unit_, diagnostic_policy_);
  EXPECT_TRUE(shape == nullptr);
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
                               ".*The mesh at .* is not a triangle mesh..*"));

  // Case: The UsdGeomMesh Prim does not have face counts attribute.
  mesh.GetPrim().RemoveProperty(face_counts_attribute.GetName());
  shape = internal::CreateGeometryMesh("no_face_counts.obj", mesh.GetPrim(),
                                       meters_per_unit_, diagnostic_policy_);
  EXPECT_TRUE(shape == nullptr);
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(".*Failed to read the 'faceVertexCounts' "
                                      "attribute of the Prim at.*"));

  // Case: The UsdGeomMesh Prim has invalid (non-isotropic) scaling.
  scale_op.Set(pxr::GfVec3d(1.0, 2.0, 1.0));
  shape = internal::CreateGeometryMesh("invalid_scaling.obj", mesh.GetPrim(),
                                       meters_per_unit_, diagnostic_policy_);
  EXPECT_TRUE(shape == nullptr);
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*The scaling of the mesh at .* is not isotropic.*"));
}

TEST_F(UsdGeometryTest, GetRigidTransformTest) {
  // Case: All inputs are valid.
  pxr::UsdGeomXform xform =
      pxr::UsdGeomXform::Define(stage_, pxr::SdfPath("/Xform"));

  const pxr::GfVec3d translation = pxr::GfVec3d(196, 51, 133.1);
  const pxr::GfVec3d rotation_xyz = pxr::GfVec3d(21.59, -9.56, 155);
  pxr::UsdGeomXformOp translate_op =
      xform.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionDouble);
  pxr::UsdGeomXformOp rotate_op =
      xform.AddRotateXYZOp(pxr::UsdGeomXformOp::PrecisionDouble);
  translate_op.Set(translation);
  rotate_op.Set(rotation_xyz);

  std::optional<math::RigidTransform<double>> transform =
      internal::GetPrimRigidTransform(xform.GetPrim(), meters_per_unit_,
                                      diagnostic_policy_);
  const Eigen::Vector3d parsed_translation = transform.value().translation();
  const Eigen::Vector3d parsed_rotation_xyz =
      transform.value().rotation().ToRollPitchYaw().vector();

  const Eigen::Vector3d intended_translation =
      internal::UsdVec3dToEigen(translation * meters_per_unit_);
  const Eigen::Vector3d intended_rotation_xyz =
      internal::UsdVec3dToEigen(rotation_xyz * (M_PI / 180.0));
  EXPECT_TRUE(
      is_approx_equal_abstol(parsed_translation, intended_translation, 1e-10));
  EXPECT_TRUE(is_approx_equal_abstol(parsed_rotation_xyz, intended_rotation_xyz,
                                     1e-10));

  // Case: Input Prim is not an UsdGeomXformable type.
  pxr::UsdPrim empty_prim =
      stage_->DefinePrim(pxr::SdfPath("/InvalidType"), pxr::TfToken(""));
  std::optional<math::RigidTransform<double>> empty_prim_transform =
      internal::GetPrimRigidTransform(empty_prim, meters_per_unit_,
                                      diagnostic_policy_);
  EXPECT_FALSE(empty_prim_transform.has_value());
  EXPECT_THAT(
      TakeError(),
      ::testing::MatchesRegex(
          ".*Failed to cast the Prim at .* into an UsdGeomXformable.*"));
}

TEST_F(UsdGeometryTest, InvalidPrimScaleTest) {
  // Case: Input Prim is not an UsdGeomXformable type.
  pxr::UsdPrim empty_prim =
      stage_->DefinePrim(pxr::SdfPath("/InvalidType"), pxr::TfToken(""));
  std::optional<Eigen::Vector3d> scale =
      internal::GetPrimScale(empty_prim, diagnostic_policy_);
  EXPECT_FALSE(scale.has_value());
  EXPECT_THAT(
      TakeError(),
      ::testing::MatchesRegex(
          ".*Failed to cast the Prim at .* into an UsdGeomXformable.*"));
}

TEST_F(UsdGeometryTest, GetPrimColorTest) {
  // Case: Invalid Prim type.
  pxr::UsdPrim empty_prim =
      stage_->DefinePrim(pxr::SdfPath("/InvalidType"), pxr::TfToken(""));
  std::optional<Eigen::Vector4d> color =
      internal::GetGeomPrimColor(empty_prim, diagnostic_policy_);
  EXPECT_FALSE(color.has_value());

  // Case: Prim does not contain color attribute.
  pxr::UsdGeomCube box = pxr::UsdGeomCube::Define(stage_, pxr::SdfPath("/Box"));
  color = internal::GetGeomPrimColor(box.GetPrim(), diagnostic_policy_);
  EXPECT_FALSE(color.has_value());

  // Case: All inputs are valid.
  pxr::UsdGeomGprim box_gprim = pxr::UsdGeomGprim(box);
  const pxr::VtArray<pxr::GfVec3f> input_color = {pxr::GfVec3f(0.1, 0.2, 0.0)};
  box_gprim.CreateDisplayColorAttr().Set(input_color);
  color = internal::GetGeomPrimColor(box.GetPrim(), diagnostic_policy_);
  ASSERT_TRUE(color.has_value());
  const pxr::GfVec3f output_color =
      pxr::GfVec3f((*color)[0], (*color)[1], (*color)[2]);
  EXPECT_EQ(input_color[0], output_color);
}

TEST_F(UsdGeometryTest, GetPrimFrictionTest) {
  // TODO(hong-nvidia): Implement this test case when GetPrimFriction()
  // is implemented.
  pxr::UsdPrim empty_prim =
      stage_->DefinePrim(pxr::SdfPath("/InvalidType"), pxr::TfToken(""));
  CoulombFriction<double> friction = internal::GetPrimFriction(empty_prim);
}

TEST_F(UsdGeometryTest, InvalidMassTest) {
  // Case: Prim contains mass attribute of the wrong type (double instead of
  // float).
  std::string file = R"""(#usda 1.0
    def Cube "Box" (
      prepend apiSchemas = ["PhysicsMassAPI"]
    )
    {
      double physics:mass = 1
      double size = 1
    })""";

  ASSERT_TRUE(stage_->GetRootLayer()->ImportFromString(file));
  const double mass = internal::GetPrimMass(
      stage_->GetPrimAtPath(pxr::SdfPath("/Box")), diagnostic_policy_);
  EXPECT_EQ(1.0, mass);
  EXPECT_THAT(
      TakeError(),
      ::testing::MatchesRegex(
          ".*Double precision float is not supported by UsdPhysicsMassAPI.*"));
  EXPECT_THAT(TakeWarning(), ::testing::MatchesRegex(
                                 ".*Failed to read the mass of the Prim at.*"));
}

}  // namespace
}  // namespace multibody
}  // namespace drake

#endif  // WITH_USD
