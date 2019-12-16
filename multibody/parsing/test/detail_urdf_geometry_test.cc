#include "drake/multibody/parsing/detail_urdf_geometry.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/package_map.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using std::make_unique;
using std::unique_ptr;

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

using math::RigidTransformd;
using geometry::GeometryInstance;
using geometry::ProximityProperties;

// Creates a special XML DOM consisting of *only* a collision object. XML text
// can be provided as an input and it will be injected as a child of the
// <collision> tag.
unique_ptr<XMLDocument> MakeCollisionDocFromString(
    const std::string& collision_spec) {
  const std::string urdf_harness = R"_(
<?xml version="1.0"?>
  <collision>
    <geometry>
      <box size=".1 .2 .3"/>
    </geometry>{}
  </collision>)_";
  const std::string urdf = fmt::format(urdf_harness, collision_spec);
  auto doc = make_unique<XMLDocument>();
  doc->Parse(urdf.c_str());
  return doc;
}

class UrdfGeometryTests : public testing::Test {
 public:
  // Loads a URDF file and parses the minimal amount of it which
  // urdf_geometry.cc handles.
  void ParseUrdfGeometry(const std::string& file_name) {
    const std::string full_path = GetFullPath(file_name);

    xml_doc_.LoadFile(full_path.c_str());
    ASSERT_FALSE(xml_doc_.ErrorID()) << xml_doc_.ErrorName();

    size_t found = full_path.find_last_of("/\\");
    if (found != std::string::npos) {
      root_dir_ = full_path.substr(0, found);
    }

    // TODO(sam.creasey) Add support for using an existing package map.
    package_map_.PopulateUpstreamToDrake(full_path);

    const XMLElement* node = xml_doc_.FirstChildElement("robot");
    ASSERT_TRUE(node);

    for (const XMLElement* material_node = node->FirstChildElement("material");
         material_node;
         material_node = material_node->NextSiblingElement("material")) {
      ParseMaterial(material_node, &materials_);
    }

    // Parses geometry out of the model's link elements.
    for (const XMLElement* link_node = node->FirstChildElement("link");
         link_node;
         link_node = link_node->NextSiblingElement("link")) {
      const char* attr = node->Attribute("name");
      ASSERT_TRUE(attr);
      const std::string body_name = attr;

      for (const XMLElement* visual_node =
               link_node->FirstChildElement("visual");
           visual_node;
           visual_node = visual_node->NextSiblingElement("visual")) {
        geometry::GeometryInstance geometry_instance =
            internal::ParseVisual(body_name, package_map_, root_dir_,
                                  visual_node, &materials_);
        visual_instances_.push_back(geometry_instance);
      }

      for (const XMLElement* collision_node =
               link_node->FirstChildElement("collision");
           collision_node;
           collision_node = collision_node->NextSiblingElement("collision")) {
        geometry::GeometryInstance geometry_instance =
            internal::ParseCollision(body_name, package_map_, root_dir_,
                                     collision_node);
        collision_instances_.push_back(geometry_instance);
      }
    }
  }

 protected:
  XMLDocument xml_doc_;
  std::string root_dir_{"."};
  multibody::PackageMap package_map_;
  MaterialMap materials_;

  std::vector<GeometryInstance> visual_instances_;
  std::vector<GeometryInstance> collision_instances_;
};

// This test dives into more detail for some things than other tests.  We
// assume if parsing certain things works here that it will keep working.
TEST_F(UrdfGeometryTests, TestParseMaterial1) {
  using Eigen::Vector4d;
  const std::string resource_dir{
    "drake/multibody/parsing/test/urdf_parser_test/"};
  const std::string file_no_conflict_1 = FindResourceOrThrow(
      resource_dir + "non_conflicting_materials_1.urdf");

  DRAKE_EXPECT_NO_THROW(ParseUrdfGeometry(file_no_conflict_1));

  ASSERT_EQ(materials_.size(), 3);

  Vector4d brown(0.93333333333, 0.79607843137, 0.67843137254, 1);
  EXPECT_TRUE(CompareMatrices(materials_.at("brown"), brown, 1e-10));

  Vector4d red(0.93333333333, 0.2, 0.2, 1);
  EXPECT_TRUE(CompareMatrices(materials_.at("red"), red, 1e-10));

  Vector4d green(0, 1, 0, 1);
  EXPECT_TRUE(CompareMatrices(materials_.at("green"), green, 1e-10));

  ASSERT_EQ(visual_instances_.size(), 2);
  const auto& visual = visual_instances_.front();
  const std::string name_base = "non_conflicting_materials_1";
  EXPECT_EQ(visual.name().substr(0, name_base.size()), name_base);

  EXPECT_TRUE(CompareMatrices(
      visual.pose().GetAsMatrix34(), RigidTransformd().GetAsMatrix34()));

  const geometry::Box* box =
      dynamic_cast<const geometry::Box*>(&visual.shape());
  ASSERT_TRUE(box);
  EXPECT_TRUE(CompareMatrices(box->size(), Eigen::Vector3d(0.2, 0.2, 0.2)));

  const auto& capsule_visual = visual_instances_.back();
  const geometry::Capsule* capsule =
      dynamic_cast<const geometry::Capsule*>(&capsule_visual.shape());
  ASSERT_TRUE(capsule);
  EXPECT_EQ(capsule->radius(), 0.2);
  EXPECT_EQ(capsule->length(), 0.5);

  const geometry::IllustrationProperties* properties =
      visual.illustration_properties();
  ASSERT_NE(properties, nullptr);
  EXPECT_TRUE(properties->HasProperty("phong", "diffuse"));
  EXPECT_TRUE(
      CompareMatrices(properties->GetProperty<Vector4d>("phong", "diffuse"),
          materials_.at("green")));
}

TEST_F(UrdfGeometryTests, TestParseMaterial2) {
  const std::string resource_dir{
    "drake/multibody/parsing/test/urdf_parser_test/"};
  const std::string file_no_conflict_2 = FindResourceOrThrow(
      resource_dir + "non_conflicting_materials_2.urdf");

  DRAKE_EXPECT_NO_THROW(ParseUrdfGeometry(file_no_conflict_2));
  EXPECT_EQ(materials_.size(), 1);

  ASSERT_EQ(visual_instances_.size(), 2);
  const auto& visual = visual_instances_.front();

  const RigidTransformd expected_pose(Eigen::Vector3d(0, 0, 0.3));
  EXPECT_TRUE(CompareMatrices(
      visual.pose().GetAsMatrix34(), expected_pose.GetAsMatrix34()));

  const geometry::Cylinder* cylinder =
      dynamic_cast<const geometry::Cylinder*>(&visual.shape());
  ASSERT_TRUE(cylinder);
  EXPECT_EQ(cylinder->radius(), 0.1);
  EXPECT_EQ(cylinder->length(), 0.6);

  const auto& mesh_visual = visual_instances_.back();
  const geometry::Mesh* mesh =
      dynamic_cast<const geometry::Mesh*>(&mesh_visual.shape());
  ASSERT_TRUE(mesh);

  const std::string& mesh_filename = mesh->filename();
  std::string obj_name = "tri_cube.obj";
  EXPECT_EQ(mesh_filename.rfind(obj_name),
            mesh_filename.size() - obj_name.size());

  ASSERT_EQ(collision_instances_.size(), 1);
  const auto& collision = collision_instances_.front();

  const geometry::Sphere* sphere =
      dynamic_cast<const geometry::Sphere*>(&collision.shape());
  ASSERT_TRUE(sphere);
  EXPECT_EQ(sphere->radius(), 0.2);
}

TEST_F(UrdfGeometryTests, TestParseMaterial3) {
  const std::string resource_dir{
    "drake/multibody/parsing/test/urdf_parser_test/"};
  const std::string file_no_conflict_3 = FindResourceOrThrow(
      resource_dir + "non_conflicting_materials_3.urdf");

  DRAKE_EXPECT_NO_THROW(ParseUrdfGeometry(file_no_conflict_3));
}

TEST_F(UrdfGeometryTests, TestParseMaterialDuplicateButSame) {
  const std::string resource_dir{
    "drake/multibody/parsing/test/urdf_parser_test/"};
  // This URDF defines the same color multiple times in different links.
  const std::string file_same_color_diff_links = FindResourceOrThrow(
      resource_dir + "duplicate_but_same_materials.urdf");
  DRAKE_EXPECT_NO_THROW(ParseUrdfGeometry(file_same_color_diff_links));

  ASSERT_GE(visual_instances_.size(), 1);

  math::RollPitchYaw<double> expected_rpy(0, 1.57, 0);
  Eigen::Vector3d expected_xyz(0.01429, 0, 0);
  math::RigidTransform<double> expected_pose(
      math::RotationMatrix<double>(expected_rpy), expected_xyz);

  const auto& visual = visual_instances_.front();
  math::RigidTransform<double> actual_pose(visual.pose());
  EXPECT_TRUE(actual_pose.IsNearlyEqualTo(expected_pose, 1e-10));
}

TEST_F(UrdfGeometryTests, TestDuplicateMaterials) {
  const std::string resource_dir{
    "drake/multibody/parsing/test/urdf_parser_test/"};
  const std::string file_duplicate = FindResourceOrThrow(
      resource_dir + "duplicate_materials.urdf");

  EXPECT_THROW(ParseUrdfGeometry(file_duplicate), std::runtime_error);
}

TEST_F(UrdfGeometryTests, TestConflictingMaterials) {
  const std::string resource_dir{
    "drake/multibody/parsing/test/urdf_parser_test/"};
  const std::string file_conflict = FindResourceOrThrow(
      resource_dir + "conflicting_materials.urdf");

  EXPECT_THROW(ParseUrdfGeometry(file_conflict), std::runtime_error);
}

TEST_F(UrdfGeometryTests, TestWrongElementType) {
  const std::string resource_dir{
    "drake/multibody/parsing/test/urdf_parser_test/"};
  const std::string file_no_conflict_1 = FindResourceOrThrow(
      resource_dir + "non_conflicting_materials_1.urdf");

  DRAKE_EXPECT_NO_THROW(ParseUrdfGeometry(file_no_conflict_1));

  const XMLElement* node = xml_doc_.FirstChildElement("robot");
  ASSERT_TRUE(node);

  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::ParseMaterial(node, &materials_), std::runtime_error,
      "Expected material element, got robot");

  const XMLElement* material_node = node->FirstChildElement("material");
  ASSERT_TRUE(material_node);

  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::ParseVisual("fake_name", package_map_, root_dir_, material_node,
                            &materials_), std::runtime_error,
      "In link fake_name expected visual element, got material");

  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::ParseCollision("fake_name", package_map_, root_dir_,
                               material_node), std::runtime_error,
      "In link 'fake_name' expected collision element, got material");
}

TEST_F(UrdfGeometryTests, TestParseConvexMesh) {
  const std::string resource_dir{
      "drake/multibody/parsing/test/urdf_parser_test/"};
  const std::string convex_and_nonconvex_test =
      FindResourceOrThrow(resource_dir + "convex_and_nonconvex_test.urdf");

  DRAKE_EXPECT_NO_THROW(ParseUrdfGeometry(convex_and_nonconvex_test));

  ASSERT_EQ(collision_instances_.size(), 2);

  {
    const auto& instance = collision_instances_[0];
    const geometry::Convex* convex =
        dynamic_cast<const geometry::Convex*>(&instance.shape());
    ASSERT_TRUE(convex);
  }

  {
    const auto& instance = collision_instances_[1];
    const geometry::Mesh* mesh =
        dynamic_cast<const geometry::Mesh*>(&instance.shape());
    ASSERT_TRUE(mesh);
  }
}

// Verify we can parse drake collision properties from a <collision> element.
TEST_F(UrdfGeometryTests, CollisionProperties) {
  // Verifies that the property exists with the given double-typed value.
  auto verify_single_property = [](const ProximityProperties& properties,
                                   const char* group, const char* property,
                                   double value) {
    ASSERT_TRUE(properties.HasProperty(group, property))
        << fmt::format("  for property: ('{}', '{}')", group, property);
    EXPECT_EQ(properties.GetProperty<double>(group, property), value);
  };

  // Verifies that the properties has friction and it matches the given values.
  auto verify_friction = [](const ProximityProperties& properties,
                            const CoulombFriction<double>& expected_friction) {
    ASSERT_TRUE(properties.HasProperty("material", "coulomb_friction"));
    const auto& friction = properties.GetProperty<CoulombFriction<double>>(
        "material", "coulomb_friction");
    EXPECT_EQ(friction.static_friction(), expected_friction.static_friction());
    EXPECT_EQ(friction.dynamic_friction(),
              expected_friction.dynamic_friction());
  };

  const PackageMap package_map;     // An empty package map.
  const std::string root_dir(".");  // Arbitrary, un-used root directory.

  // This parser uses the ParseProximityProperties found in detail_common
  // (which already has exhaustive tests). So, we'll put in a smoke test to
  // confirm that all of the basic tags get parsed and focus on the logic that
  // is unique to `ParseCollision()`.
  {
    unique_ptr<XMLDocument> doc = MakeCollisionDocFromString(R"_(
  <drake:proximity_properties>
    <drake:mesh_resolution_hint value="2.5"/>
    <drake:elastic_modulus value="3.5" />
    <drake:hunt_crossley_dissipation value="3.5" />
    <drake:mu_dynamic value="3.25" />
    <drake:mu_static value="3.5" />
  </drake:proximity_properties>)_");
    const XMLElement* collision_node = doc->FirstChildElement("collision");
    ASSERT_NE(collision_node, nullptr);
    GeometryInstance instance =
        ParseCollision("link_name", package_map, root_dir, collision_node);
    ASSERT_NE(instance.proximity_properties(), nullptr);
    const ProximityProperties& properties = *instance.proximity_properties();
    verify_single_property(properties, geometry::internal::kHydroGroup,
                           geometry::internal::kRezHint, 2.5);
    verify_single_property(properties, geometry::internal::kMaterialGroup,
                           geometry::internal::kElastic, 3.5);
    verify_single_property(properties, geometry::internal::kMaterialGroup,
                           geometry::internal::kHcDissipation, 3.5);
    verify_friction(properties, {3.5, 3.25});
  }

  // Case: specifies rigid hydroelastic.
  {
    unique_ptr<XMLDocument> doc = MakeCollisionDocFromString(R"_(
  <drake:proximity_properties>
    <drake:rigid_hydroelastic/>
  </drake:proximity_properties>)_");
    const XMLElement* collision_node = doc->FirstChildElement("collision");
    ASSERT_NE(collision_node, nullptr);
    GeometryInstance instance =
        ParseCollision("link_name", package_map, root_dir, collision_node);
    ASSERT_NE(instance.proximity_properties(), nullptr);
    const ProximityProperties& properties = *instance.proximity_properties();
    ASSERT_TRUE(properties.HasProperty(geometry::internal::kHydroGroup,
                                       geometry::internal::kComplianceType));
    EXPECT_EQ(properties.GetProperty<geometry::internal::HydroelasticType>(
        geometry::internal::kHydroGroup, geometry::internal::kComplianceType),
            geometry::internal::HydroelasticType::kRigid);
  }

  // Case: specifies soft hydroelastic.
  {
    unique_ptr<XMLDocument> doc = MakeCollisionDocFromString(R"_(
  <drake:proximity_properties>
    <drake:soft_hydroelastic/>
  </drake:proximity_properties>)_");
    const XMLElement* collision_node = doc->FirstChildElement("collision");
    ASSERT_NE(collision_node, nullptr);
    GeometryInstance instance =
        ParseCollision("link_name", package_map, root_dir, collision_node);
    ASSERT_NE(instance.proximity_properties(), nullptr);
    const ProximityProperties& properties = *instance.proximity_properties();
    ASSERT_TRUE(properties.HasProperty(geometry::internal::kHydroGroup,
                                       geometry::internal::kComplianceType));
    EXPECT_EQ(properties.GetProperty<geometry::internal::HydroelasticType>(
        geometry::internal::kHydroGroup, geometry::internal::kComplianceType),
              geometry::internal::HydroelasticType::kSoft);
  }

  // Case: specifies both -- should be an error.
  {
    unique_ptr<XMLDocument> doc = MakeCollisionDocFromString(R"_(
  <drake:proximity_properties>
    <drake:soft_hydroelastic/>
    <drake:rigid_hydroelastic/>
  </drake:proximity_properties>)_");
    const XMLElement* collision_node = doc->FirstChildElement("collision");
    ASSERT_NE(collision_node, nullptr);
    DRAKE_EXPECT_THROWS_MESSAGE(
        ParseCollision("link_name", package_map, root_dir, collision_node),
        std::runtime_error,
        "Collision geometry has defined mutually-exclusive tags .*rigid.* and "
        ".*soft.*");
  }

  // TODO(SeanCurtis-TRI): This is the *old* interface; the new
  //  drake::proximity_properties should supplant it. Deprecate and remove this.
  // Case: has no drake:proximity_properties coefficients, only drake_compliance
  // coeffs.
  {
    unique_ptr<XMLDocument> doc = MakeCollisionDocFromString(R"_(
  <drake_compliance>
    <static_friction>3.5</static_friction>
    <dynamic_friction>2.5</dynamic_friction>
  </drake_compliance>)_");
    const XMLElement* collision_node = doc->FirstChildElement("collision");
    ASSERT_NE(collision_node, nullptr);
    GeometryInstance instance =
        ParseCollision("link_name", package_map, root_dir, collision_node);
    ASSERT_NE(instance.proximity_properties(), nullptr);
    const ProximityProperties& properties = *instance.proximity_properties();
    verify_friction(properties, {3.5, 2.5});
  }

  // Case: has both drake_compliance and drake:proximity_properties;
  // drake:proximity_properties wins.
  unique_ptr<XMLDocument> doc = MakeCollisionDocFromString(R"_(
  <drake_compliance>
    <static_friction>3.5</static_friction>
    <dynamic_friction>2.5</dynamic_friction>
  </drake_compliance>
  <drake:proximity_properties>
    <drake:mu_dynamic value="4.5" />
  </drake:proximity_properties>)_");
  const XMLElement* collision_node = doc->FirstChildElement("collision");
  ASSERT_NE(collision_node, nullptr);
  GeometryInstance instance =
      ParseCollision("link_name", package_map, root_dir, collision_node);
  ASSERT_NE(instance.proximity_properties(), nullptr);
  const ProximityProperties& properties = *instance.proximity_properties();
  verify_friction(properties, {4.5, 4.5});
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
