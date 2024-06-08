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
namespace internal {
namespace {

class UsdGeometryTest : public test::DiagnosticPolicyTestBase {
 public:
  UsdGeometryTest() {
    // Construct a parser object so that it initializes the USD library.
    UsdParserWrapper parser;

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
  // Case: all inputs are valid.
  pxr::UsdGeomCube box = pxr::UsdGeomCube::Define(
    stage_, pxr::SdfPath("/Box"));

  double size = 49.0;
  pxr::UsdAttribute size_attribute = box.CreateSizeAttr();
  EXPECT_TRUE(size_attribute.Set(size));

  pxr::GfVec3d scale_factor = pxr::GfVec3d(0.4, 0.5, 0.6);
  auto scale_op = box.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble);
  EXPECT_TRUE(scale_op.Set(scale_factor));

  std::optional<Eigen::Vector3d> dimension = GetBoxDimension(
    box.GetPrim(), meters_per_unit_, diagnostic_policy_);
  EXPECT_TRUE(dimension.has_value());

  Eigen::Vector3d correct_dimension =
    UsdVec3dToEigen(scale_factor) * size * meters_per_unit_;
  EXPECT_EQ(dimension.value(), correct_dimension);

  auto shape = CreateGeometryBox(box.GetPrim(), meters_per_unit_,
    diagnostic_policy_);
  EXPECT_TRUE(shape != nullptr);
  geometry::Box* drake_box = dynamic_cast<geometry::Box*>(shape.get());
  EXPECT_EQ(drake_box->size(), correct_dimension);

  float mass = 2.71;
  auto mass_api = pxr::UsdPhysicsMassAPI::Apply(box.GetPrim());
  auto mass_attribute = mass_api.CreateMassAttr();
  EXPECT_TRUE(mass_attribute.Set(mass));
  auto inertia = CreateSpatialInertiaForBox(box.GetPrim(), meters_per_unit_,
    diagnostic_policy_);
  EXPECT_TRUE(inertia.has_value());
  EXPECT_EQ(mass, static_cast<float>(inertia.value().get_mass()));

  // Case: the input Prim is not an UsdGeomCube.
  auto empty_prim = stage_->DefinePrim(pxr::SdfPath("/InvalidType"),
    pxr::TfToken(""));
  dimension = GetBoxDimension(empty_prim.GetPrim(), meters_per_unit_,
    diagnostic_policy_);
  EXPECT_FALSE(dimension.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*Failed to cast the Prim at .* into an UsdGeomCube.*"));

  // Case: the input Prim is missing the size attribute
  EXPECT_TRUE(box.GetPrim().RemoveProperty(size_attribute.GetName()));
  dimension = GetBoxDimension(box.GetPrim(), meters_per_unit_,
    diagnostic_policy_);
  EXPECT_FALSE(dimension.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*Failed to read the 'size' attribute of the Prim at .*"));
}

TEST_F(UsdGeometryTest, EllipsoidParsingTest) {
  // Case: all inputs are valid.
  pxr::UsdGeomSphere ellipsoid = pxr::UsdGeomSphere::Define(
    stage_, pxr::SdfPath("/Ellipsoid"));

  double radius = 17.0;
  pxr::UsdAttribute radius_attribute = ellipsoid.CreateRadiusAttr();
  EXPECT_TRUE(radius_attribute.Set(radius));

  pxr::GfVec3d scale_factor = pxr::GfVec3d(0.6, 1.1, 2.9);
  auto scale_op = ellipsoid.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble);
  EXPECT_TRUE(scale_op.Set(scale_factor));

  std::optional<Eigen::Vector3d> dimension = GetEllipsoidDimension(
    ellipsoid.GetPrim(), meters_per_unit_, diagnostic_policy_);
  EXPECT_TRUE(dimension.has_value());

  Eigen::Vector3d correct_dimension =
    UsdVec3dToEigen(scale_factor) * radius * meters_per_unit_;
  EXPECT_EQ(dimension.value(), correct_dimension);

  auto shape = CreateGeometryEllipsoid(ellipsoid.GetPrim(), meters_per_unit_,
    diagnostic_policy_);
  EXPECT_TRUE(shape != nullptr);
  geometry::Ellipsoid* drake_ellipsoid =
    dynamic_cast<geometry::Ellipsoid*>(shape.get());
  auto actual_dimension = Eigen::Vector3d(
    drake_ellipsoid->a(), drake_ellipsoid->b(), drake_ellipsoid->c());
  EXPECT_EQ(actual_dimension, correct_dimension);

  float mass = 77.241;
  auto mass_api = pxr::UsdPhysicsMassAPI::Apply(ellipsoid.GetPrim());
  auto mass_attribute = mass_api.CreateMassAttr();
  EXPECT_TRUE(mass_attribute.Set(mass));
  auto inertia = CreateSpatialInertiaForEllipsoid(ellipsoid.GetPrim(),
    meters_per_unit_, diagnostic_policy_);
  EXPECT_TRUE(inertia.has_value());
  EXPECT_EQ(mass, static_cast<float>(inertia.value().get_mass()));

  // Case: the input Prim is not an UsdGeomSphere.
  auto empty_prim = stage_->DefinePrim(pxr::SdfPath("/InvalidType"),
    pxr::TfToken(""));
  auto invalid_dimension = GetEllipsoidDimension(empty_prim.GetPrim(),
    meters_per_unit_, diagnostic_policy_);
  EXPECT_FALSE(invalid_dimension.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*Failed to cast the Prim at .* into an UsdGeomSphere.*"));

  // Case: the input Prim is missing the radius attribute
  EXPECT_TRUE(ellipsoid.GetPrim().RemoveProperty(radius_attribute.GetName()));
  dimension = GetEllipsoidDimension(ellipsoid.GetPrim(), meters_per_unit_,
    diagnostic_policy_);
  EXPECT_FALSE(dimension.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*Failed to read the 'radius' attribute of the Prim at .*"));
}

TEST_F(UsdGeometryTest, CylinderParsingTest) {
  // Case: all inputs are valid.
  pxr::UsdGeomCylinder cylinder = pxr::UsdGeomCylinder::Define(
    stage_, pxr::SdfPath("/Cylinder"));

  double radius = 62.0;
  double height = 199.0;
  pxr::TfToken axis = pxr::TfToken("Z");
  pxr::UsdAttribute radius_attribute = cylinder.CreateRadiusAttr();
  pxr::UsdAttribute height_attribute = cylinder.CreateHeightAttr();
  pxr::UsdAttribute axis_attribute = cylinder.CreateAxisAttr();
  EXPECT_TRUE(radius_attribute.Set(radius));
  EXPECT_TRUE(height_attribute.Set(height));
  EXPECT_TRUE(axis_attribute.Set(axis));

  pxr::GfVec3d scale_factor = pxr::GfVec3d(0.7, 0.7, 0.9);
  auto scale_op = cylinder.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble);
  EXPECT_TRUE(scale_op.Set(scale_factor));

  std::optional<Eigen::Vector2d> dimension = GetCylinderDimension(
    cylinder.GetPrim(), meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_TRUE(dimension.has_value());

  double correct_radius = scale_factor[0] * radius * meters_per_unit_;
  double correct_height = scale_factor[2] * height * meters_per_unit_;
  auto correct_dimension = Eigen::Vector2d(correct_radius, correct_height);
  EXPECT_EQ(dimension.value(), correct_dimension);

  auto shape = CreateGeometryCylinder(cylinder.GetPrim(), meters_per_unit_,
    stage_up_axis_, diagnostic_policy_);
  EXPECT_TRUE(shape != nullptr);
  geometry::Cylinder* drake_cylinder =
    dynamic_cast<geometry::Cylinder*>(shape.get());
  auto actual_dimension = Eigen::Vector2d(
    drake_cylinder->radius(), drake_cylinder->length());
  EXPECT_EQ(actual_dimension, correct_dimension);

  float mass = 152.0;
  auto mass_api = pxr::UsdPhysicsMassAPI::Apply(cylinder.GetPrim());
  auto mass_attribute = mass_api.CreateMassAttr();
  EXPECT_TRUE(mass_attribute.Set(mass));
  auto inertia = CreateSpatialInertiaForCylinder(cylinder.GetPrim(),
    meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_TRUE(inertia.has_value());
  EXPECT_EQ(mass, static_cast<float>(inertia.value().get_mass()));

  // Case: the input Prim is not an UsdGeomCylinder.
  auto empty_prim = stage_->DefinePrim(pxr::SdfPath("/InvalidType"),
    pxr::TfToken(""));
  auto invalid_dimension = GetCylinderDimension(empty_prim.GetPrim(),
    meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(invalid_dimension.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*Failed to cast the Prim at .* into an UsdGeomCylinder.*"));

  // Case: the input Prim is missing the height attribute
  EXPECT_TRUE(cylinder.GetPrim().RemoveProperty(height_attribute.GetName()));
  dimension = GetCylinderDimension(cylinder.GetPrim(), meters_per_unit_,
    stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(dimension.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*Failed to read the 'height' attribute of the Prim at .*"));

  // Case: the input Prim is missing the radius attribute
  EXPECT_TRUE(cylinder.GetPrim().RemoveProperty(radius_attribute.GetName()));
  dimension = GetCylinderDimension(cylinder.GetPrim(), meters_per_unit_,
    stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(dimension.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*Failed to read the 'radius' attribute of the Prim at .*"));

  // Case: the cylinder has different scaling in X and Y axis
  scale_op.Set(pxr::GfVec3d(0.8, 0.7, 0.9));
  dimension = GetCylinderDimension(cylinder.GetPrim(), meters_per_unit_,
    stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(dimension.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*The cylinder at .* has different scaling in X and Y axis.*"));

  // Case: the axis of the cylinder is not the same as the up-axis of the stage
  axis_attribute.Set(pxr::TfToken("Y"));
  dimension = GetCylinderDimension(cylinder.GetPrim(), meters_per_unit_,
    stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(dimension.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*The cylinder at .* is not upright.*"));
}

TEST_F(UsdGeometryTest, CapsuleParsingTest) {
  // Case: all inputs are valid.
  pxr::UsdGeomCapsule capsule = pxr::UsdGeomCapsule::Define(
    stage_, pxr::SdfPath("/Capsule"));

  double radius = 101;
  double height = 45;
  pxr::TfToken axis = pxr::TfToken("Z");
  pxr::UsdAttribute radius_attribute = capsule.CreateRadiusAttr();
  pxr::UsdAttribute height_attribute = capsule.CreateHeightAttr();
  pxr::UsdAttribute axis_attribute = capsule.CreateAxisAttr();
  EXPECT_TRUE(radius_attribute.Set(radius));
  EXPECT_TRUE(height_attribute.Set(height));
  EXPECT_TRUE(axis_attribute.Set(axis));

  pxr::GfVec3d scale_factor = pxr::GfVec3d(0.7, 0.7, 0.9);
  auto scale_op = capsule.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble);
  EXPECT_TRUE(scale_op.Set(scale_factor));

  std::optional<Eigen::Vector2d> dimension = GetCapsuleDimension(
    capsule.GetPrim(), meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_TRUE(dimension.has_value());

  double correct_radius = scale_factor[0] * radius * meters_per_unit_;
  double correct_height = scale_factor[2] * height * meters_per_unit_;
  auto correct_dimension = Eigen::Vector2d(correct_radius, correct_height);
  EXPECT_EQ(dimension.value(), correct_dimension);

  auto shape = CreateGeometryCapsule(capsule.GetPrim(), meters_per_unit_,
    stage_up_axis_, diagnostic_policy_);
  EXPECT_TRUE(shape != nullptr);
  geometry::Capsule* drake_capsule =
    dynamic_cast<geometry::Capsule*>(shape.get());
  auto actual_dimension = Eigen::Vector2d(
    drake_capsule->radius(), drake_capsule->length());
  EXPECT_EQ(actual_dimension, correct_dimension);

  float mass = 152.0;
  auto mass_api = pxr::UsdPhysicsMassAPI::Apply(capsule.GetPrim());
  auto mass_attribute = mass_api.CreateMassAttr();
  EXPECT_TRUE(mass_attribute.Set(mass));
  auto inertia = CreateSpatialInertiaForCapsule(capsule.GetPrim(),
    meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_TRUE(inertia.has_value());
  EXPECT_EQ(mass, static_cast<float>(inertia.value().get_mass()));

  // Case: the input Prim is not an UsdGeomCapsule.
  auto empty_prim = stage_->DefinePrim(pxr::SdfPath("/InvalidType"),
    pxr::TfToken(""));
  auto invalid_dimension = GetCapsuleDimension(empty_prim.GetPrim(),
    meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(invalid_dimension.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*Failed to cast the Prim at .* into an UsdGeomCapsule.*"));

  // Case: the input Prim is missing the height attribute
  EXPECT_TRUE(capsule.GetPrim().RemoveProperty(height_attribute.GetName()));
  dimension = GetCapsuleDimension(capsule.GetPrim(), meters_per_unit_,
    stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(dimension.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*Failed to read the 'height' attribute of the Prim at .*"));

  // Case: the input Prim is missing the radius attribute
  EXPECT_TRUE(capsule.GetPrim().RemoveProperty(radius_attribute.GetName()));
  dimension = GetCapsuleDimension(capsule.GetPrim(), meters_per_unit_,
    stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(dimension.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*Failed to read the 'radius' attribute of the Prim at .*"));

  // Case: the capsule has different scaling in X and Y axis
  scale_op.Set(pxr::GfVec3d(0.8, 0.7, 0.9));
  dimension = GetCapsuleDimension(capsule.GetPrim(), meters_per_unit_,
    stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(dimension.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*The capsule at .* has different scaling in X and Y axis.*"));

  // Case: the axis of the capsule is not the same as the up-axis of the stage
  axis_attribute.Set(pxr::TfToken("Y"));
  dimension = GetCapsuleDimension(capsule.GetPrim(), meters_per_unit_,
    stage_up_axis_, diagnostic_policy_);
  EXPECT_FALSE(dimension.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*The capsule at .* is not upright.*"));
}

TEST_F(UsdGeometryTest, MeshParsingTest) {
  // Case: all inputs are valid.
  pxr::UsdGeomMesh mesh = pxr::UsdGeomMesh::Define(
    stage_, pxr::SdfPath("/Mesh"));

  // The following specifies an octahedron mesh.
  auto vertices = pxr::VtArray<pxr::GfVec3f>{
    pxr::GfVec3f(1, 0, 0), pxr::GfVec3f(0, -1, 0),
    pxr::GfVec3f(-1, 0, 0), pxr::GfVec3f(0, 1, 0),
    pxr::GfVec3f(0, 0, 1), pxr::GfVec3f(0, 0, -1)};
  auto face_vertex_counts = pxr::VtArray<int>{3, 3, 3, 3, 3, 3, 3, 3};
  auto face_vertex_indices = pxr::VtArray<int>{
    1, 0, 4, 2, 1, 4, 3, 2, 4, 0, 3, 4, 0, 1, 5, 1, 2, 5, 2, 3, 5, 3, 0, 5};
  double scale_factor = 129.2;

  EXPECT_TRUE(mesh.CreatePointsAttr().Set(vertices));
  EXPECT_TRUE(mesh.CreateFaceVertexCountsAttr().Set(face_vertex_counts));
  EXPECT_TRUE(mesh.CreateFaceVertexIndicesAttr().Set(face_vertex_indices));
  auto scale_op = mesh.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble);

  EXPECT_TRUE(scale_op.Set(
    pxr::GfVec3d(scale_factor, scale_factor, scale_factor)));
  auto shape = CreateGeometryMesh("octahedron.obj", mesh.GetPrim(),
    meters_per_unit_, diagnostic_policy_);
  EXPECT_TRUE(shape != nullptr);
  geometry::Mesh* drake_mesh = dynamic_cast<geometry::Mesh*>(shape.get());
  EXPECT_EQ(drake_mesh->scale(), scale_factor);

  // Check whether Drake can sucessfully parse that file by computing the
  // convex hull of the octahedron mesh.
  auto convex_hull = drake_mesh->GetConvexHull();
  EXPECT_EQ(convex_hull.num_faces(), 8);

  // Case: the UsdGeomMesh Prim has invalid (non-isotropic) scaling.
  EXPECT_TRUE(scale_op.Set(pxr::GfVec3d(1.0, 2.0, 1.0)));
  shape = CreateGeometryMesh("invalid_scaling.obj", mesh.GetPrim(),
    meters_per_unit_, diagnostic_policy_);
  EXPECT_TRUE(shape == nullptr);
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*The scaling of the mesh at .* is not isotropic.*"));
}

TEST_F(UsdGeometryTest, GetRigidTransformTest) {
  // Case: all inputs are valid.
  pxr::UsdGeomXform xform = pxr::UsdGeomXform::Define(
    stage_, pxr::SdfPath("/Xform"));

  pxr::GfVec3d translation = pxr::GfVec3d(196, 51, 133.1);
  pxr::GfVec3d rotation_xyz = pxr::GfVec3d(21.59, -9.56, 155);
  auto translate_op = xform.AddTranslateOp(
    pxr::UsdGeomXformOp::PrecisionDouble);
  auto rotate_op = xform.AddRotateXYZOp(
    pxr::UsdGeomXformOp::PrecisionDouble);
  EXPECT_TRUE(translate_op.Set(translation));
  EXPECT_TRUE(rotate_op.Set(rotation_xyz));

  auto transform = GetPrimRigidTransform(
    xform.GetPrim(), meters_per_unit_, diagnostic_policy_);
  EXPECT_TRUE(transform.has_value());
  auto actual_translation = transform.value().translation();
  auto actual_rotation_xyz =
    transform.value().rotation().ToRollPitchYaw().vector();

  auto intended_translation = UsdVec3dToEigen(translation * meters_per_unit_);
  auto intended_rotation_xyz = UsdVec3dToEigen(rotation_xyz * (M_PI / 180.0));
  EXPECT_TRUE(is_approx_equal_abstol(
    actual_translation, intended_translation, 1e-10));
  EXPECT_TRUE(is_approx_equal_abstol(
    actual_rotation_xyz, intended_rotation_xyz, 1e-10));

  // Case: the input Prim is not an UsdGeomXformable type
  auto empty_prim = stage_->DefinePrim(pxr::SdfPath("/InvalidType"),
    pxr::TfToken(""));
  auto empty_prim_transform = GetPrimRigidTransform(empty_prim.GetPrim(),
    meters_per_unit_, diagnostic_policy_);
  EXPECT_FALSE(empty_prim_transform.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*Failed to cast the Prim at .* into an UsdGeomXformable.*"));
}

TEST_F(UsdGeometryTest, InvalidPrimScaleTest) {
  // Case: input Prim is not an UsdGeomXformable type.
  auto empty_prim = stage_->DefinePrim(pxr::SdfPath("/InvalidType"),
    pxr::TfToken(""));
  auto scale = GetPrimScale(empty_prim.GetPrim(),
    diagnostic_policy_);
  EXPECT_FALSE(scale.has_value());
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*Failed to cast the Prim at .* into an UsdGeomXformable.*"));
}

TEST_F(UsdGeometryTest, GetPrimColorTest) {
  // Case: invalid Prim type.
  auto empty_prim = stage_->DefinePrim(pxr::SdfPath("/InvalidType"),
    pxr::TfToken(""));
  auto color = GetGeomPrimColor(empty_prim, diagnostic_policy_);
  EXPECT_THAT(TakeWarning(), ::testing::MatchesRegex(
    ".*Failed to cast the Prim at .* into an UsdGeomGprim.*"));

  // Case: Prim does not contain color attribute.
  pxr::UsdGeomCube box = pxr::UsdGeomCube::Define(stage_,
    pxr::SdfPath("/Box"));
  color = GetGeomPrimColor(box.GetPrim(), diagnostic_policy_);
  EXPECT_THAT(TakeWarning(), ::testing::MatchesRegex(
    ".*Failed to read the DisplayColor of the Prim at.*"));

  // Case: all inputs are valid.
  pxr::UsdGeomGprim box_gprim = pxr::UsdGeomGprim(box);
  pxr::VtArray<pxr::GfVec3f> input_color = { pxr::GfVec3f(0.1, 0.2, 0.0) };
  EXPECT_TRUE(box_gprim.CreateDisplayColorAttr().Set(input_color));
  color = GetGeomPrimColor(box.GetPrim(), diagnostic_policy_);
  auto output_color = pxr::GfVec3f(color[0], color[1], color[2]);
  EXPECT_EQ(input_color[0], output_color);
}

TEST_F(UsdGeometryTest, GetPrimFrictionTest) {
  // TODO(hong-nvidia): Implement this test case when GetPrimFriction()
  // is implemented.
  auto empty_prim = stage_->DefinePrim(pxr::SdfPath("/InvalidType"),
    pxr::TfToken(""));
  auto friction = GetPrimFriction(empty_prim);
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

  EXPECT_TRUE(stage_->GetRootLayer()->ImportFromString(file));
  double mass = GetPrimMass(stage_->GetPrimAtPath(pxr::SdfPath("/Box")),
    diagnostic_policy_);
  EXPECT_EQ(1.0, mass);
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*Double precision float is not supported by UsdPhysicsMassAPI.*"));
  EXPECT_THAT(TakeWarning(), ::testing::MatchesRegex(
    ".*Failed to read the mass of the Prim at.*"));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake

#endif  // WITH_USD
