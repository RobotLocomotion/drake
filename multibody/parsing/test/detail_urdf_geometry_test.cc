#include "drake/multibody/parsing/detail_urdf_geometry.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/package_map.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

using math::RigidTransformd;
using geometry::GeometryInstance;

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
        CoulombFriction<double> friction;
        geometry::GeometryInstance geometry_instance =
            internal::ParseCollision(body_name, package_map_, root_dir_,
                                     collision_node, &friction);
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

  EXPECT_NO_THROW(ParseUrdfGeometry(file_no_conflict_1));

  ASSERT_EQ(materials_.size(), 3);

  Vector4d brown(0.93333333333, 0.79607843137, 0.67843137254, 1);
  EXPECT_TRUE(CompareMatrices(materials_.at("brown"), brown, 1e-10));

  Vector4d red(0.93333333333, 0.2, 0.2, 1);
  EXPECT_TRUE(CompareMatrices(materials_.at("red"), red, 1e-10));

  Vector4d green(0, 1, 0, 1);
  EXPECT_TRUE(CompareMatrices(materials_.at("green"), green, 1e-10));

  ASSERT_EQ(visual_instances_.size(), 1);
  const auto& visual = visual_instances_.front();
  const std::string name_base = "non_conflicting_materials_1";
  EXPECT_EQ(visual.name().substr(0, name_base.size()), name_base);

  EXPECT_TRUE(CompareMatrices(
      visual.pose().GetAsMatrix34(), RigidTransformd().GetAsMatrix34()));

  const geometry::Box* box =
      dynamic_cast<const geometry::Box*>(&visual.shape());
  ASSERT_TRUE(box);
  EXPECT_TRUE(CompareMatrices(box->size(), Eigen::Vector3d(0.2, 0.2, 0.2)));

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

  EXPECT_NO_THROW(ParseUrdfGeometry(file_no_conflict_2));
  EXPECT_EQ(materials_.size(), 1);

  ASSERT_EQ(visual_instances_.size(), 2);
  const auto& visual = visual_instances_.front();

  const RigidTransformd expected_pose(Eigen::Vector3d(0, 0, 0.3));
  EXPECT_TRUE(CompareMatrices(
      visual.pose().GetAsMatrix34(), expected_pose.GetAsMatrix34()));

  const geometry::Cylinder* cylinder =
      dynamic_cast<const geometry::Cylinder*>(&visual.shape());
  ASSERT_TRUE(cylinder);
  EXPECT_EQ(cylinder->get_radius(), 0.1);
  EXPECT_EQ(cylinder->get_length(), 0.6);

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
  EXPECT_EQ(sphere->get_radius(), 0.2);
}

TEST_F(UrdfGeometryTests, TestParseMaterial3) {
  const std::string resource_dir{
    "drake/multibody/parsing/test/urdf_parser_test/"};
  const std::string file_no_conflict_3 = FindResourceOrThrow(
      resource_dir + "non_conflicting_materials_3.urdf");

  EXPECT_NO_THROW(ParseUrdfGeometry(file_no_conflict_3));
}

TEST_F(UrdfGeometryTests, TestParseMaterialDuplicateButSame) {
  const std::string resource_dir{
    "drake/multibody/parsing/test/urdf_parser_test/"};
  // This URDF defines the same color multiple times in different links.
  const std::string file_same_color_diff_links = FindResourceOrThrow(
      resource_dir + "duplicate_but_same_materials.urdf");
  EXPECT_NO_THROW(ParseUrdfGeometry(file_same_color_diff_links));

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

  EXPECT_NO_THROW(ParseUrdfGeometry(file_no_conflict_1));

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

  CoulombFriction<double> friction;
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::ParseCollision("fake_name", package_map_, root_dir_,
                               material_node, &friction), std::runtime_error,
      "In link fake_name expected collision element, got material");
}

TEST_F(UrdfGeometryTests, TestParseConvexMesh) {
  const std::string resource_dir{
      "drake/multibody/parsing/test/urdf_parser_test/"};
  const std::string convex_and_nonconvex_test =
      FindResourceOrThrow(resource_dir + "convex_and_nonconvex_test.urdf");

  EXPECT_NO_THROW(ParseUrdfGeometry(convex_and_nonconvex_test));

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

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
