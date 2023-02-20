#include "drake/multibody/parsing/detail_urdf_geometry.h"

#include <memory>
#include <vector>

#include <fmt/ostream.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/parsing/test/diagnostic_policy_test_base.h"

namespace drake {
namespace multibody {
namespace internal {

std::ostream& operator<<(std::ostream& out, const UrdfMaterial& m) {
  if (m.rgba.has_value()) {
    fmt::print(out, "RGBA: {}", fmt_eigen(m.rgba->transpose()));
  } else {
    out << "RGBA: None";
  }
  out << ", ";
  if (m.diffuse_map.has_value()) {
    fmt::print(out, "Diffuse map: {}", *m.diffuse_map);
  } else {
    out << "Diffuse map: None";
  }
  return out;
}

bool operator==(const UrdfMaterial& m1, const UrdfMaterial& m2) {
  return m1.rgba == m2.rgba && m1.diffuse_map == m2.diffuse_map;
}

namespace {

using std::make_unique;
using std::unique_ptr;

using Eigen::Vector4d;
using testing::MatchesRegex;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;
using math::RigidTransformd;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::ProximityProperties;

class UrdfMaterialTest : public testing::Test {
 public:
  UrdfMaterialTest() {
    policy_.SetActionForErrors(
        [this](const DiagnosticDetail& detail) {
          EXPECT_TRUE(error_.empty());
          error_ = detail.message;
        });
  }

  // Call the function under test, taking care of repetitive details.
  UrdfMaterial AddMaterial(const std::string& material_name,
                           UrdfMaterial material,
                           bool error_if_name_clash) {
    return AddMaterialToMaterialMap(
        policy_, material_name, material, error_if_name_clash, &materials_);
  }

  // Call the function under test, expect no error.
  UrdfMaterial AddMaterialGood(const std::string& material_name,
                               UrdfMaterial material,
                               bool error_if_name_clash,
                               UrdfMaterial expected) {
    auto result = AddMaterial(material_name, material, error_if_name_clash);
    EXPECT_TRUE(error_.empty());
    EXPECT_EQ(result, expected);
    return result;
  }

  // Call the function under test, expect an error matching a pattern.
  UrdfMaterial AddMaterialBad(const std::string& material_name,
                              UrdfMaterial material,
                              bool error_if_name_clash,
                              const std::string& error_pattern) {
    auto result = AddMaterial(material_name, material, error_if_name_clash);
    EXPECT_THAT(error_, MatchesRegex(error_pattern));
    return result;
  }

 protected:
  MaterialMap materials_;
  DiagnosticPolicy policy_;
  std::string error_;

  const bool error_if_name_clash_{true};

  const Vector4d black_color_{0, 0, 0, 0};
  const Vector4d full_color_{0.3, 0.6, 0.9, 0.99};
  const Vector4d rgba_color_{0.25, 0.5, 0.75, 1.0};
  const UrdfMaterial default_mat_{black_color_, {}};
  const UrdfMaterial empty_mat_{{}, {}};
  const UrdfMaterial rgba_mat_{rgba_color_, {}};
  const UrdfMaterial full_mat_{full_color_, "full"};

  const std::string empty_mat_name_{"empty"};
  const std::string rgba_mat_name_{"just_rgba"};
  const std::string full_mat_name_{"full"};
};

// ----   Simple addition of new materials and redundant addition of identical
//        material.

TEST_F(UrdfMaterialTest, UniqueNoData) {
  // Case: Adding a unique material with no data - gets default rgba.
  AddMaterialGood(empty_mat_name_, empty_mat_, error_if_name_clash_,
                  default_mat_);
  AddMaterialGood(empty_mat_name_, empty_mat_, !error_if_name_clash_,
                  default_mat_);
}

TEST_F(UrdfMaterialTest, UniqueOnlyRgba) {
  // Case: Adding a unique material with only rgba.
  AddMaterialGood(rgba_mat_name_, rgba_mat_, error_if_name_clash_, rgba_mat_);
  AddMaterialGood(rgba_mat_name_, rgba_mat_, !error_if_name_clash_, rgba_mat_);
}

TEST_F(UrdfMaterialTest, RedundantDifferentColor) {
  // Case: Adding a redundant material whose rgba values are different, but
  // within tolerance.
  AddMaterialGood(rgba_mat_name_, rgba_mat_, error_if_name_clash_, rgba_mat_);
  const Vector4d rgba_color2 = rgba_color_ + Vector4d::Constant(1e-11);
  AddMaterialGood(rgba_mat_name_, UrdfMaterial{rgba_color2, {}},
                  !error_if_name_clash_, rgba_mat_);

  // Deviation between colors too big.
  const Vector4d rgba_color3 = rgba_color_ + Vector4d::Constant(5e-11);
  AddMaterialBad(rgba_mat_name_, UrdfMaterial{rgba_color3, {}},
                 !error_if_name_clash_,
                 "Material '.+' was previously defined.+");
}

TEST_F(UrdfMaterialTest, UniqueDiffuse) {
  // Case: Adding a unique material with only diffuse map - get default rgba.
  const UrdfMaterial diffuse_mat{{}, "arbitrary_diffuse"};
  const std::string diffuse_mat_name{"just_diffuse"};
  const UrdfMaterial expected{black_color_, diffuse_mat.diffuse_map};
  AddMaterialGood(diffuse_mat_name, diffuse_mat, error_if_name_clash_,
                  expected);
  AddMaterialGood(diffuse_mat_name, diffuse_mat, !error_if_name_clash_,
                  expected);
}

TEST_F(UrdfMaterialTest, UniqueBoth) {
  // Case: adding a unique material with both diffuse map and rgba.
  AddMaterialGood(full_mat_name_, full_mat_, error_if_name_clash_, full_mat_);
  AddMaterialGood(full_mat_name_, full_mat_, !error_if_name_clash_, full_mat_);
}

// ----    Name matches to cached value, but new material is nullopt.

// Note: There is no case where cached has *only* diffuse map because any
// material missing rgba is assigned default black before being stored in the
// cache.

TEST_F(UrdfMaterialTest, CacheRgbaInputEmpty) {
  // Case: Cache only has rgba defined; input material is empty.
  AddMaterialGood(rgba_mat_name_, rgba_mat_, error_if_name_clash_, rgba_mat_);
  AddMaterialGood(rgba_mat_name_, empty_mat_, !error_if_name_clash_, rgba_mat_);
}

TEST_F(UrdfMaterialTest, CacheBothInputEmpty) {
  // Case: Cache only has diffuse map and rgba defined; input material is empty.
  AddMaterialGood(full_mat_name_, full_mat_, error_if_name_clash_, full_mat_);
  AddMaterialGood(full_mat_name_, empty_mat_, !error_if_name_clash_, full_mat_);
}

// ----    Name clashes.

TEST_F(UrdfMaterialTest, NameClashParameter) {
  // Case: Adding an identical material is/isn't an error based on name clash
  // parameter.
  AddMaterialGood(empty_mat_name_, empty_mat_, error_if_name_clash_,
                  default_mat_);
  ASSERT_NE(materials_.find(empty_mat_name_), materials_.end());
  // Redundant adding is not an error.
  AddMaterialGood(empty_mat_name_, empty_mat_, !error_if_name_clash_,
                  default_mat_);

  // Redundant adding with name clash *is* an error.
  AddMaterialBad(empty_mat_name_, empty_mat_, error_if_name_clash_,
                 "Material '.+' was previously defined.+");
}

// ----    Failure modes.

TEST_F(UrdfMaterialTest, RgbaMismatch) {
  // Case: rgba doesn't match.
  AddMaterialGood(rgba_mat_name_, rgba_mat_, error_if_name_clash_, rgba_mat_);
  AddMaterialBad(rgba_mat_name_, UrdfMaterial{Vector4d{0.1, 0.1, 0.1, 0.1}, {}},
                 !error_if_name_clash_,
                 "Material '.+' was previously defined.+");
}

TEST_F(UrdfMaterialTest, DiffuseMismatch) {
  // Case: Cached diffuse_map is nullopt, input is not.
  AddMaterialGood(rgba_mat_name_, rgba_mat_, error_if_name_clash_, rgba_mat_);
  AddMaterialBad(rgba_mat_name_, UrdfMaterial{{}, "bad_name"},
                 !error_if_name_clash_,
                 "Material '.+' was previously defined.+");
}

// The failure mode where cached rgba is nullopt and new material is *not* is
// not possible because any material missing rgba is assigned default black
// before being stored in the cache.

TEST_F(UrdfMaterialTest, ValueMismatch) {
  // Case: Cached and input have non-matching values.
  AddMaterialGood(full_mat_name_, full_mat_, error_if_name_clash_, full_mat_);
  AddMaterialBad(full_mat_name_, UrdfMaterial{full_color_, "bad_name"},
                 !error_if_name_clash_,
                 "Material '.+' was previously defined.+");
}

class UrdfGeometryTest : public test::DiagnosticPolicyTestBase {
 public:
  // Creates a special XML DOM consisting of *only* a collision object. XML text
  // can be provided as an input and it will be injected as a child of the
  // <collision> tag.
  void MakeCollisionDocFromString(
      const std::string& collision_spec) {
    constexpr const char* urdf_harness = R"""(
<?xml version="1.0"?>
  <collision>
    <geometry>
      <box size=".1 .2 .3"/>
    </geometry>{}
  </collision>)""";

    contents_ = fmt::format(urdf_harness, collision_spec);
    UpdateSource(DataSource::kContents, &contents_);
    xml_doc_.Parse(contents_.c_str());
  }

  // Parses a "collision doc", maybe returning the geometry instance.
  std::optional<GeometryInstance> ParseCollisionDoc(
      const std::string& collision_spec) {
    MakeCollisionDocFromString(collision_spec);
    const XMLElement* collision_node = xml_doc_.FirstChildElement("collision");
    EXPECT_NE(collision_node, nullptr) << collision_spec;
    DRAKE_DEMAND(collision_node != nullptr);
    return ParseCollision(*diag_, "link_name", package_map_, root_dir_,
                          collision_node, &geometry_names_,
                          numeric_name_suffix_limit_);
  }


  // Processes a "collision doc" all the way to proximity properties in the
  // expect-success case. As a side effect, the collision_instances_ member is
  // updated with the constructed instance.
  const ProximityProperties& ParseCollisionDocGood(
      const std::string& collision_spec) {
    std::optional<GeometryInstance> maybe_instance =
        ParseCollisionDoc(collision_spec);
    EXPECT_TRUE(maybe_instance.has_value()) << collision_spec;
    DRAKE_DEMAND(maybe_instance.has_value());
    collision_instances_.push_back(std::move(*maybe_instance));
    const GeometryInstance& instance = collision_instances_.back();
    EXPECT_NE(instance.proximity_properties(), nullptr) << collision_spec;
    DRAKE_DEMAND(instance.proximity_properties() != nullptr);
    return *instance.proximity_properties();
  }

  // Parses the minimal amount of an URDF-format string which urdf_geometry.cc
  // handles.
  void ParseUrdfGeometryString(const std::string& contents) {
    contents_ = contents;
    UpdateSource(DataSource::kContents, &contents_);
    DoParseUrdfGeometry();
  }

  // Loads a URDF file and parses the minimal amount of it which
  // urdf_geometry.cc handles.
  void ParseUrdfGeometryFile(const std::string& file_name) {
    file_name_ = file_name;
    UpdateSource(DataSource::kFilename, &file_name_);
    DoParseUrdfGeometry();
  }

  // Verifies that the property exists with the given value.
  template <typename T>
  void VerifySingleProperty(const ProximityProperties& properties,
                            const char* group, const char* property,
                            const T& value) {
    bool has_property = properties.HasProperty(group, property);
    EXPECT_TRUE(has_property)
        << fmt::format("  for property: ('{}', '{}')", group, property);
    if (!has_property) { return; }
    EXPECT_EQ(properties.GetProperty<T>(group, property), value);
  }

  // Verifies that the properties has friction and it matches the given values.
  void VerifyFriction(const ProximityProperties& properties,
                      const CoulombFriction<double>& expected_friction) {
    VerifySingleProperty(properties, "material", "coulomb_friction",
                         expected_friction);
  }

  // Finds a file resource within 'urdf_parser_test'.
  std::string FindUrdfTestResourceOrThrow(const std::string& filename) {
      const std::string resource_dir{
        "drake/multibody/parsing/test/urdf_parser_test/"};
      return FindResourceOrThrow(resource_dir + filename);
  }

 protected:
  void UpdateSource(DataSource::DataSourceType type, std::string* data) {
    source_ = std::make_unique<DataSource>(type, data);
    root_dir_ = source_->GetRootDir();
    diag_ = std::make_unique<TinyXml2Diagnostic>(&diagnostic_policy_,
                                                 &*source_);
  }

  // Parses the currently set source_.
  void DoParseUrdfGeometry() {
    DRAKE_DEMAND(source_ != nullptr);
    if (source_->IsFilename()) {
      xml_doc_.LoadFile(source_->filename().c_str());
    } else {
      DRAKE_DEMAND(source_->IsContents());
      xml_doc_.Parse(source_->contents().c_str());
    }
    ASSERT_FALSE(xml_doc_.ErrorID()) << xml_doc_.ErrorName();

    const XMLElement* node = xml_doc_.FirstChildElement("robot");
    ASSERT_TRUE(node);

    for (const XMLElement* material_node = node->FirstChildElement("material");
         material_node;
         material_node = material_node->NextSiblingElement("material")) {
      ParseMaterial(*diag_, material_node, true, package_map_, root_dir_,
                    &materials_);
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
        std::optional<geometry::GeometryInstance> geometry_instance =
            ParseVisual(*diag_, body_name, package_map_, root_dir_,
                        visual_node, &materials_, &geometry_names_,
                        numeric_name_suffix_limit_);
        if (geometry_instance.has_value()) {
          visual_instances_.push_back(*geometry_instance);
        }
      }

      for (const XMLElement* collision_node =
               link_node->FirstChildElement("collision");
           collision_node;
           collision_node = collision_node->NextSiblingElement("collision")) {
        std::optional<geometry::GeometryInstance> geometry_instance =
            ParseCollision(*diag_, body_name, package_map_, root_dir_,
                           collision_node, &geometry_names_,
                           numeric_name_suffix_limit_);
        if (geometry_instance.has_value()) {
          collision_instances_.push_back(*geometry_instance);
        }
      }
    }
  }

  XMLDocument xml_doc_;
  multibody::PackageMap package_map_;
  MaterialMap materials_;
  std::string file_name_;
  std::string contents_;
  std::unique_ptr<DataSource> source_;
  std::string root_dir_;
  std::unique_ptr<TinyXml2Diagnostic> diag_;

  std::unordered_set<std::string> geometry_names_;
  int numeric_name_suffix_limit_{kDefaultNumericSuffixLimit};

  std::vector<GeometryInstance> visual_instances_;
  std::vector<GeometryInstance> collision_instances_;
};

// Some tests contain deliberate typos to provoke parser errors or warnings. In
// those cases, the sequence `QQQ` will be inserted to stand in for some more
// naturalistic typo.

// This test dives into more detail for some things than other tests.  We
// assume if parsing certain things works here that it will keep working.
TEST_F(UrdfGeometryTest, TestParseMaterial1) {
  const std::string file_no_conflict_1 =
      FindUrdfTestResourceOrThrow("non_conflicting_materials_1.urdf");

  DRAKE_EXPECT_NO_THROW(ParseUrdfGeometryFile(file_no_conflict_1));

  ASSERT_EQ(materials_.size(), 5);

  const Vector4d brown(0.93333333333, 0.79607843137, 0.67843137254, 1);
  EXPECT_TRUE(materials_.at("brown").rgba.has_value());
  EXPECT_FALSE(materials_.at("brown").diffuse_map.has_value());
  EXPECT_TRUE(CompareMatrices(*(materials_.at("brown").rgba), brown, 1e-10));

  const Vector4d red(0.93333333333, 0.2, 0.2, 1);
  EXPECT_TRUE(materials_.at("red").rgba.has_value());
  EXPECT_FALSE(materials_.at("red").diffuse_map.has_value());
  EXPECT_TRUE(CompareMatrices(*(materials_.at("red").rgba), red, 1e-10));

  const Vector4d green(0, 1, 0, 1);
  EXPECT_TRUE(materials_.at("green").rgba.has_value());
  EXPECT_FALSE(materials_.at("green").diffuse_map.has_value());
  EXPECT_TRUE(CompareMatrices(*(materials_.at("green").rgba), green, 1e-10));

  const Vector4d black(0, 0, 0, 0);
  EXPECT_TRUE(materials_.at("textured").rgba.has_value());
  EXPECT_TRUE(materials_.at("textured").diffuse_map.has_value());
  const std::string& file_name1 = *(materials_.at("textured").diffuse_map);
  EXPECT_EQ("empty.png", file_name1.substr(file_name1.size() - 9));
  EXPECT_TRUE(CompareMatrices(*(materials_.at("textured").rgba), black, 1e-10));

  EXPECT_TRUE(materials_.at("texture_and_color").rgba.has_value());
  EXPECT_TRUE(materials_.at("texture_and_color").diffuse_map.has_value());
  const std::string& file_name2 =
      *(materials_.at("texture_and_color").diffuse_map);
  EXPECT_EQ("empty.png", file_name2.substr(file_name2.size() - 9));
  EXPECT_TRUE(
      CompareMatrices(*(materials_.at("texture_and_color").rgba), green));

  ASSERT_EQ(visual_instances_.size(), 4);
  const auto& visual = visual_instances_[0];

  // The <visual/> has no name, so type of Shape is used instead.
  EXPECT_EQ(visual.name(), "Box");

  EXPECT_TRUE(CompareMatrices(
      visual.pose().GetAsMatrix34(), RigidTransformd().GetAsMatrix34()));

  const geometry::IllustrationProperties* properties =
      visual.illustration_properties();
  ASSERT_NE(properties, nullptr);
  EXPECT_TRUE(properties->HasProperty("phong", "diffuse"));
  EXPECT_TRUE(
      CompareMatrices(properties->GetProperty<Vector4d>("phong", "diffuse"),
                      *(materials_.at("green").rgba)));

  const geometry::Box* box =
      dynamic_cast<const geometry::Box*>(&visual.shape());
  ASSERT_TRUE(box);
  EXPECT_TRUE(CompareMatrices(box->size(), Eigen::Vector3d(0.2, 0.2, 0.2)));

  const auto& capsule_visual = visual_instances_[1];
  const geometry::Capsule* capsule =
      dynamic_cast<const geometry::Capsule*>(&capsule_visual.shape());
  ASSERT_TRUE(capsule);
  EXPECT_EQ(capsule->radius(), 0.2);
  EXPECT_EQ(capsule->length(), 0.5);

  const auto& sphere_visual = visual_instances_[2];
  const geometry::Sphere* sphere =
      dynamic_cast<const geometry::Sphere*>(&sphere_visual.shape());
  ASSERT_TRUE(sphere);
  EXPECT_EQ(sphere->radius(), 0.25);
}

TEST_F(UrdfGeometryTest, TestParseMaterial2) {
  const std::string file_no_conflict_2 =
      FindUrdfTestResourceOrThrow("non_conflicting_materials_2.urdf");

  DRAKE_EXPECT_NO_THROW(ParseUrdfGeometryFile(file_no_conflict_2));
  EXPECT_EQ(materials_.size(), 1);

  const Vector4d brown{0.93333333333, 0.79607843137, 0.67843137254, 1};
  EXPECT_TRUE(materials_.at("brown").rgba.has_value());
  EXPECT_TRUE(CompareMatrices(*(materials_.at("brown").rgba), brown, 1e-14));

  ASSERT_EQ(visual_instances_.size(), 3);

  const auto& visual = visual_instances_[0];
  EXPECT_TRUE(
      visual.illustration_properties()->HasProperty("phong", "diffuse"));
  EXPECT_TRUE(
      CompareMatrices(visual.illustration_properties()->GetProperty<Vector4d>(
                          "phong", "diffuse"),
                      brown, 1e-14));
  const RigidTransformd expected_pose(Eigen::Vector3d(0, 0, 0.3));
  EXPECT_TRUE(CompareMatrices(
      visual.pose().GetAsMatrix34(), expected_pose.GetAsMatrix34()));

  const geometry::Cylinder* cylinder =
      dynamic_cast<const geometry::Cylinder*>(&visual.shape());
  ASSERT_TRUE(cylinder);
  EXPECT_EQ(cylinder->radius(), 0.1);
  EXPECT_EQ(cylinder->length(), 0.6);

  const auto& mesh_visual = visual_instances_[1];
  // Mesh has a "default" material.
  const geometry::Mesh* mesh =
      dynamic_cast<const geometry::Mesh*>(&mesh_visual.shape());
  ASSERT_TRUE(mesh);

  const std::string& mesh_filename = mesh->filename();
  std::string obj_name = "tri_cube.obj";
  EXPECT_EQ(mesh_filename.rfind(obj_name),
            mesh_filename.size() - obj_name.size());

  const auto& sphere_visual = visual_instances_[2];
  EXPECT_TRUE(
      sphere_visual.illustration_properties()->HasProperty("phong", "diffuse"));
  EXPECT_TRUE(CompareMatrices(
      sphere_visual.illustration_properties()->GetProperty<Vector4d>("phong",
                                                                     "diffuse"),
      brown, 1e-14));

  ASSERT_EQ(collision_instances_.size(), 1);
  const auto& collision = collision_instances_.front();

  const geometry::Sphere* sphere =
      dynamic_cast<const geometry::Sphere*>(&collision.shape());
  ASSERT_TRUE(sphere);
  EXPECT_EQ(sphere->radius(), 0.2);
}

TEST_F(UrdfGeometryTest, TestParseMaterial3) {
  const std::string file_no_conflict_3 =
      FindUrdfTestResourceOrThrow("non_conflicting_materials_3.urdf");

  DRAKE_EXPECT_NO_THROW(ParseUrdfGeometryFile(file_no_conflict_3));
}

TEST_F(UrdfGeometryTest, TestParseMaterialNoName) {
  ParseUrdfGeometryString("<robot name='a'><material/></robot>");
  EXPECT_THAT(TakeError(), MatchesRegex(".*Material.*missing.*name.*"));
}

TEST_F(UrdfGeometryTest, TestParseMaterialBadTextureName) {
  // This family of errors is more fully tested in test for ResolveUri(). Here
  // we just provoke a ResolveUri() failure to exercise the URDF geometry error
  // path.
  ParseUrdfGeometryString(R"""(
    <robot name='a'>
     <material name='b'><texture filename='/QQQ'/></material>
    </robot>)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*/QQQ.*invalid.*"));
}

TEST_F(UrdfGeometryTest, TestParseMaterialBadRgba) {
  ParseUrdfGeometryString(R"""(
    <robot name='a'>
     <material name='b'><color rgQQQba=''/></material>
    </robot>)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*Failed to parse 'rgba'.*"));
}

TEST_F(UrdfGeometryTest, TestParseMaterialEmpty) {
  ParseUrdfGeometryString("<robot name='a'><material name='b'/></robot>");
  EXPECT_THAT(TakeError(), MatchesRegex(
                  ".*Material 'b'.*no color or texture.*"));
}

TEST_F(UrdfGeometryTest, TestParseMaterialDuplicateButSame) {
  // This URDF defines the same color multiple times in different links.
  const std::string file_same_color_diff_links = FindUrdfTestResourceOrThrow(
      "duplicate_but_same_materials_with_texture.urdf");
  DRAKE_EXPECT_NO_THROW(ParseUrdfGeometryFile(file_same_color_diff_links));

  ASSERT_GE(visual_instances_.size(), 1);

  math::RollPitchYaw<double> expected_rpy(0, 1.57, 0);
  Eigen::Vector3d expected_xyz(0.01429, 0, 0);
  math::RigidTransform<double> expected_pose(
      math::RotationMatrix<double>(expected_rpy), expected_xyz);

  const auto& visual = visual_instances_.front();
  math::RigidTransform<double> actual_pose(visual.pose());
  EXPECT_TRUE(actual_pose.IsNearlyEqualTo(expected_pose, 1e-10));
}

TEST_F(UrdfGeometryTest, TestDuplicateMaterials) {
  const std::string file_duplicate =
      FindUrdfTestResourceOrThrow("duplicate_materials.urdf");

  ParseUrdfGeometryFile(file_duplicate);
  EXPECT_THAT(TakeError(), MatchesRegex(".*brown.*was previously defined.*"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*red.*was previously defined.*"));
}

TEST_F(UrdfGeometryTest, TestConflictingMaterials) {
  const std::string file_conflict =
      FindUrdfTestResourceOrThrow("conflicting_materials.urdf");

  ParseUrdfGeometryFile(file_conflict);
  EXPECT_THAT(TakeError(), MatchesRegex(".*brown.*was previously defined.*"));
}

TEST_F(UrdfGeometryTest, TestWrongElementType) {
  const std::string file_no_conflict_1 =
      FindUrdfTestResourceOrThrow("non_conflicting_materials_1.urdf");

  DRAKE_EXPECT_NO_THROW(ParseUrdfGeometryFile(file_no_conflict_1));

  const XMLElement* node = xml_doc_.FirstChildElement("robot");
  ASSERT_TRUE(node);

  ParseMaterial(*diag_, node, false, package_map_, root_dir_, &materials_);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Expected material element, got <robot>"));

  const XMLElement* material_node = node->FirstChildElement("material");
  ASSERT_TRUE(material_node);

  ParseVisual(*diag_, "fake_name", package_map_, root_dir_,
              material_node, &materials_, &geometry_names_);
  EXPECT_THAT(TakeError(), MatchesRegex(".*In link fake_name expected visual"
                                        " element, got material"));

  ParseCollision(*diag_, "fake_name", package_map_, root_dir_,
               material_node, &geometry_names_);
  EXPECT_THAT(TakeError(), MatchesRegex(".*In link 'fake_name' expected"
                                        " collision element, got material"));
}

TEST_F(UrdfGeometryTest, TestParseConvexMesh) {
  const std::string convex_and_nonconvex_test = FindUrdfTestResourceOrThrow(
      "convex_and_nonconvex_test.urdf");

  DRAKE_EXPECT_NO_THROW(ParseUrdfGeometryFile(convex_and_nonconvex_test));

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
TEST_F(UrdfGeometryTest, CollisionSmokeTest) {
  // This parser uses the ParseProximityProperties found in detail_common
  // (which already has exhaustive tests). So, we'll put in a smoke test to
  // confirm that all of the basic tags get parsed and focus on the logic that
  // is unique to `ParseCollision()`.
  const ProximityProperties& properties = ParseCollisionDocGood(R"""(
  <drake:proximity_properties>
    <drake:mesh_resolution_hint value="2.5"/>
    <drake:hydroelastic_modulus value="3.5" />
    <drake:hunt_crossley_dissipation value="3.5" />
    <drake:relaxation_time value="3.1" />
    <drake:mu_dynamic value="3.25" />
    <drake:mu_static value="3.5" />
  </drake:proximity_properties>)""");
  VerifySingleProperty(properties, geometry::internal::kHydroGroup,
                       geometry::internal::kRezHint, 2.5);
  VerifySingleProperty(properties, geometry::internal::kHydroGroup,
                         geometry::internal::kElastic, 3.5);
  VerifySingleProperty(properties, geometry::internal::kMaterialGroup,
                       geometry::internal::kHcDissipation, 3.5);
  VerifySingleProperty(properties, geometry::internal::kMaterialGroup,
                       geometry::internal::kRelaxationTime, 3.1);
  VerifyFriction(properties, {3.5, 3.25});
}

TEST_F(UrdfGeometryTest, TestCollisionNameExhaustion) {
  // Test that name exhaustion doesn't cause ParseCollsion() to throw from
  // geometry back end code, and that it emits an appropriate error.
  numeric_name_suffix_limit_ = 0;
  geometry_names_.insert("Box");
  DRAKE_EXPECT_NO_THROW(ParseCollisionDoc(""));
  EXPECT_THAT(TakeError(), MatchesRegex(".*Too many identical.*"));
}

TEST_F(UrdfGeometryTest, TestVisualNameExhaustion) {
  // Test that name exhaustion doesn't cause ParseVisual() to throw from
  // geometry back end code, and that it emits an appropriate error.
  numeric_name_suffix_limit_ = 0;
  geometry_names_.insert("Box");
  std::string robot = R"""(
    <robot name='a'>
      <link name='b'>
        <visual>
          <geometry><box size='1 2 3'/></geometry>
        </visual>
      </link>
    </robot>)""";
  DRAKE_EXPECT_NO_THROW(ParseUrdfGeometryString(robot));
  EXPECT_THAT(TakeError(), MatchesRegex(".*Too many identical.*"));
}

TEST_F(UrdfGeometryTest, TestBadCollision) {
  ParseUrdfGeometryString(R"""(
    <robot name='a'>
      <link name='b'><collision/></link>
    </robot>)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*collision.*without geometry.*"));
}

TEST_F(UrdfGeometryTest, TestCollisionGroup) {
  ParseUrdfGeometryString(R"""(
    <robot name='a'>
      <link name='b'>
        <collision group='c'>
          <geometry><box size='1 2 3'/></geometry>
        </collision>
      </link>
    </robot>)""");
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*'group' attribute.*ignored.*"));
}

TEST_F(UrdfGeometryTest, TestBadBox) {
  ParseUrdfGeometryString(R"""(
    <robot name='a'>
      <link name='b'>
        <collision>
          <geometry><box/></geometry>
        </collision>
      </link>
    </robot>)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*Missing box attribute.*"));
}

TEST_F(UrdfGeometryTest, TestBadSphere) {
  constexpr const char* base = R"""(
    <robot name='a'>
      <link name='b'>
        <collision>
          <geometry><sphere {}/></geometry>
        </collision>
      </link>
    </robot>)""";

  ParseUrdfGeometryString(fmt::format(base, ""));
  EXPECT_THAT(TakeError(), MatchesRegex(".*Missing sphere attribute.*"));

  ParseUrdfGeometryString(fmt::format(base, "radius='1 2 3'"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*"));
}

TEST_F(UrdfGeometryTest, TestBadCylinder) {
  constexpr const char* base = R"""(
    <robot name='a'>
      <link name='b'>
        <collision>
          <geometry><cylinder {}/></geometry>
        </collision>
      </link>
    </robot>)""";

  ParseUrdfGeometryString(fmt::format(base, ""));
  EXPECT_THAT(TakeError(), MatchesRegex(
                  ".*Missing cylinder attribute.*radius.*"));

  ParseUrdfGeometryString(fmt::format(base, "radius='1'"));
  EXPECT_THAT(TakeError(), MatchesRegex(
                  ".*Missing cylinder attribute.*length.*"));

  ParseUrdfGeometryString(fmt::format(base, "radius='1 2 3' length='1'"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*radius.*"));

  ParseUrdfGeometryString(fmt::format(base, "radius='1' length='1 2 3'"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*length.*"));
}

TEST_F(UrdfGeometryTest, TestBadCapsule) {
  constexpr const char* base = R"""(
    <robot name='a'>
      <link name='b'>
        <collision>
          <geometry>
            <{}capsule {}/>
          </geometry>
        </collision>
      </link>
    </robot>)""";

  // Loop over capsule, drake:capsule.
  for (const auto& prefix : std::array<std::string, 2>{"", "drake:"}) {
    SCOPED_TRACE(prefix + "capsule");
    ParseUrdfGeometryString(fmt::format(base, prefix, ""));
    EXPECT_THAT(TakeError(), MatchesRegex(
                    ".*Missing capsule attribute.*radius.*"));
    ParseUrdfGeometryString(fmt::format(base, prefix, "radius='1'"));
    EXPECT_THAT(TakeError(), MatchesRegex(
                    ".*Missing capsule attribute.*length.*"));
    ParseUrdfGeometryString(fmt::format(base, prefix,
                                        "radius='1 2 3' length='1'"));
    EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*radius.*"));
    ParseUrdfGeometryString(fmt::format(base, prefix,
                                        "radius='1' length='1 2 3'"));
    EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*length.*"));
  }
}

TEST_F(UrdfGeometryTest, TestBadEllipsoid) {
  constexpr const char* base = R"""(
    <robot name='a'>
      <link name='b'>
        <collision>
          <geometry>
            <drake:ellipsoid {}/>
          </geometry>
        </collision>
      </link>
    </robot>)""";
  ParseUrdfGeometryString(fmt::format(base, ""));
  EXPECT_THAT(TakeError(), MatchesRegex(
                  ".*Missing ellipsoid attribute.*'a'.*"));

  ParseUrdfGeometryString(fmt::format(base, "a='1'"));
  EXPECT_THAT(TakeError(), MatchesRegex(
                  ".*Missing ellipsoid attribute.*'b'.*"));

  ParseUrdfGeometryString(fmt::format(base, "a='1' b='2'"));
  EXPECT_THAT(TakeError(), MatchesRegex(
                  ".*Missing ellipsoid attribute.*'c'.*"));

  ParseUrdfGeometryString(fmt::format(base, "a='1 1 1' b='2' c='3'"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*'a'.*"));

  ParseUrdfGeometryString(fmt::format(base, "a='1' b='2 1 1' c='3'"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*'b'.*"));

  ParseUrdfGeometryString(fmt::format(base, "a='1' b='2' c='3 1 1'"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*'c'.*"));
}

TEST_F(UrdfGeometryTest, TestBadMesh) {
  constexpr const char* base = R"""(
    <robot name='a'>
      <link name='b'>
        <collision>
          <geometry>
            <mesh {}/>
          </geometry>
        </collision>
      </link>
    </robot>)""";
  ParseUrdfGeometryString(fmt::format(base, ""));
  EXPECT_THAT(TakeError(), MatchesRegex(".*Mesh element.*no filename.*"));

  ParseUrdfGeometryString(fmt::format(base, "filename='/QQQ'"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*/QQQ.*invalid.*"));

  ParseUrdfGeometryString(fmt::format(base, R"""(
   filename='package://drake/multibody/parsing/test/tri_cube.obj' scale='1 2 3'
   )"""));
  EXPECT_THAT(TakeError(), MatchesRegex(".*only.*isotropic scaling.*"));
}

TEST_F(UrdfGeometryTest, TestBadShapeCollision) {
  ParseUrdfGeometryString(R"""(
    <robot name='a'>
      <link name='b'>
        <collision>
          <geometry><QQQ/></geometry>
        </collision>
      </link>
    </robot>)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*not.*recognizable shape type.*"));
}

TEST_F(UrdfGeometryTest, TestLegacyDrakeCompliance) {
  constexpr const char* base = R"""(
    <robot name='a'>
      <link name='b'>
        <collision>
          <geometry><box size='1 2 3'/></geometry>
          <drake_compliance>{}</drake_compliance>
        </collision>
      </link>
    </robot>)""";

  ParseUrdfGeometryString(fmt::format(base, "<youngs_modulus/>"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*<youngs_modulus>.*ignored.*"));

  ParseUrdfGeometryString(fmt::format(base, "<dissipation/>"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*<dissipation>.*ignored.*"));

  ParseUrdfGeometryString(fmt::format(base, "<dynamic_friction/>"));
  EXPECT_THAT(TakeError(), MatchesRegex(
                  ".*Unable to parse dynamic_friction.*"));

  ParseUrdfGeometryString(fmt::format(base, "<static_friction/>"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*Unable to parse static_friction.*"));

  ParseUrdfGeometryString(
      fmt::format(base, "<static_friction>10</static_friction>"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*both.*must be defined.*"));
}

TEST_F(UrdfGeometryTest, TestBadVisual) {
  ParseUrdfGeometryString(R"""(
    <robot name='a'>
      <link name='b'><visual/></link>
    </robot>)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*visual.*without geometry.*"));
}

TEST_F(UrdfGeometryTest, TestBadShapeVisual) {
  ParseUrdfGeometryString(R"""(
    <robot name='a'>
      <link name='b'>
        <visual>
          <geometry><QQQ/></geometry>
        </visual>
      </link>
    </robot>)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*not.*recognizable shape type.*"));
}

TEST_F(UrdfGeometryTest, TestBadProperty) {
  constexpr const char* base = R"""(
  <drake:proximity_properties>
    <drake:mu_dynamic {}/>
  </drake:proximity_properties>)""";
  ParseCollisionDoc(fmt::format(base, ""));
  EXPECT_THAT(TakeError(), MatchesRegex(
                  ".*Unable to read.*attribute.*mu_dynamic.*"));
  ParseCollisionDoc(fmt::format(base, "value='1 2 3'"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*Expected single value.*"));
}

TEST_F(UrdfGeometryTest, RigidHydroelastic) {
  // Case: specifies rigid hydroelastic.
  const ProximityProperties& properties = ParseCollisionDocGood(R"""(
  <drake:proximity_properties>
    <drake:rigid_hydroelastic/>
  </drake:proximity_properties>)""");
  ASSERT_TRUE(properties.HasProperty(geometry::internal::kHydroGroup,
                                     geometry::internal::kComplianceType));
  EXPECT_EQ(properties.GetProperty<geometry::internal::HydroelasticType>(
                geometry::internal::kHydroGroup,
                geometry::internal::kComplianceType),
            geometry::internal::HydroelasticType::kRigid);
}

TEST_F(UrdfGeometryTest, CompliantHydroelastic) {
  // Case: specifies compliant hydroelastic.
  const ProximityProperties& properties = ParseCollisionDocGood(R"""(
  <drake:proximity_properties>
    <drake:compliant_hydroelastic/>
  </drake:proximity_properties>)""");
  ASSERT_TRUE(properties.HasProperty(geometry::internal::kHydroGroup,
                                     geometry::internal::kComplianceType));
  EXPECT_EQ(properties.GetProperty<geometry::internal::HydroelasticType>(
                geometry::internal::kHydroGroup,
                geometry::internal::kComplianceType),
            geometry::internal::HydroelasticType::kSoft);
}

TEST_F(UrdfGeometryTest, LegacySoftHydroelastic) {
  // TODO(16229): Remove this ad-hoc input sanitization when we resolve
  //  issue 16229 "Diagnostics for unsupported SDFormat and URDF stanzas."
  // Case: specifies unsupported drake:soft_hydroelastic -- should be an error.
  ParseCollisionDoc(R"""(
  <drake:proximity_properties>
    <drake:soft_hydroelastic/>
  </drake:proximity_properties>)""");
  EXPECT_THAT(TakeError(), MatchesRegex(
                  ".*Collision geometry uses the tag"
                  " <drake:soft_hydroelastic>.* which is no longer supported."
                  " Please change it to <drake:compliant_hydroelastic>."));
}

TEST_F(UrdfGeometryTest, BothHydroelastic) {
  // Case: specifies both -- should be an error.
  ParseCollisionDoc(R"""(
  <drake:proximity_properties>
    <drake:compliant_hydroelastic/>
    <drake:rigid_hydroelastic/>
  </drake:proximity_properties>)""");
  EXPECT_THAT(TakeError(), MatchesRegex(
                  ".*Collision geometry has defined mutually-exclusive tags"
                  " .*rigid.* and .*compliant.*"));
}


TEST_F(UrdfGeometryTest, LegacyDrakeCompliance) {
  // TODO(SeanCurtis-TRI): This is the *old* interface; the new
  //  drake::proximity_properties should supplant it. Deprecate and remove this.
  // Case: has no drake:proximity_properties coefficients, only drake_compliance
  // coeffs.
  const ProximityProperties& properties = ParseCollisionDocGood(R"""(
  <drake_compliance>
    <static_friction>3.5</static_friction>
    <dynamic_friction>2.5</dynamic_friction>
  </drake_compliance>)""");
  VerifyFriction(properties, {3.5, 2.5});
}

TEST_F(UrdfGeometryTest, LegacyDrakeComplianceVsProxProperties) {
  // Case: has both drake_compliance and drake:proximity_properties;
  // drake:proximity_properties wins.
  const ProximityProperties& properties = ParseCollisionDocGood(R"""(
  <drake_compliance>
    <static_friction>3.5</static_friction>
    <dynamic_friction>2.5</dynamic_friction>
  </drake_compliance>
  <drake:proximity_properties>
    <drake:mu_dynamic value="4.5" />
  </drake:proximity_properties>)""");
  EXPECT_THAT(TakeWarning(), MatchesRegex(
                  ".*both <drake:proximity_properties> and"
                  " <drake_compliance> tags.*"));
  VerifyFriction(properties, {4.5, 4.5});
}

TEST_F(UrdfGeometryTest, UnsupportedRosUrfdomStuff) {
  // Not observed in the wild, but seen in the ROS urdfdom XSD Schema.  If
  // anyone expects this to do something, the warning should make clear that
  // Drake ignores it.
  // See https://github.com/ros/urdfdom/blob/dbecca0/xsd/urdf.xsd
  ParseCollisionDocGood("<verbose value='unknown'/>");
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*verbose.*ignored.*"));
}

// Confirms that the <drake:accepting_renderer> tag gets properly parsed.
TEST_F(UrdfGeometryTest, AcceptingRenderers) {
  const std::string file_no_conflict_1 =
      FindUrdfTestResourceOrThrow("accepting_renderer.urdf");

  // TODO(SeanCurtis-TRI): Test for the <drake:accepting_renderer> tag without
  //  name attribute when we can test using an in-memory XML.

  DRAKE_EXPECT_NO_THROW(ParseUrdfGeometryFile(file_no_conflict_1));

  ASSERT_EQ(visual_instances_.size(), 3);

  const std::string group = "renderer";
  const std::string property = "accepting";

  for (const auto& instance : visual_instances_) {
    // TODO(SeanCurtis-TRI): When perception properties are uniquely parsed
    //  from the file, change this to PerceptionProperties.
    EXPECT_NE(instance.illustration_properties(), nullptr);
    const IllustrationProperties& props = *instance.illustration_properties();
    if (instance.name() == "all_renderers") {
      EXPECT_FALSE(props.HasProperty(group, property));
    } else if (instance.name() == "single_renderer") {
      EXPECT_TRUE(props.HasProperty(group, property));
      const auto& names =
          props.GetProperty<std::set<std::string>>(group, property);
      EXPECT_EQ(names.size(), 1);
      EXPECT_EQ(names.count("renderer1"), 1);
    } else if (instance.name() == "multi_renderer") {
      EXPECT_TRUE(props.HasProperty(group, property));
      const auto& names =
          props.GetProperty<std::set<std::string>>(group, property);
      EXPECT_EQ(names.size(), 2);
      EXPECT_EQ(names.count("renderer1"), 1);
      EXPECT_EQ(names.count("renderer2"), 1);
    } else {
      GTEST_FAIL() << "Encountered visual geometry not expected: "
                     << instance.name();
    }
  }
}

TEST_F(UrdfGeometryTest, BadAcceptingRenderer) {
  ParseUrdfGeometryString(R"""(
    <robot name='a'>
      <link name='b'>
        <visual>
          <geometry><box size='1 2 3'/></geometry>
          <drake:accepting_renderer/>
        </visual>
      </link>
    </robot>)""");
  EXPECT_THAT(TakeError(), MatchesRegex(
                  ".*<drake:accepting_renderer>.*without.* name.*"));
}

// Check that unnamed geometry is given a name based on its shape, and that
// more than one instance of a shape has an additional numeric suffix added.
TEST_F(UrdfGeometryTest, TwoUnnamedBoxes) {
  const std::string two_unnamed_boxes =
      FindUrdfTestResourceOrThrow("two_unnamed_boxes.urdf");
  ParseUrdfGeometryFile(two_unnamed_boxes);

  ASSERT_EQ(collision_instances_.size(), 2);
  EXPECT_EQ(collision_instances_[0].name(), "Box");
  EXPECT_EQ(collision_instances_[1].name(), "Box1");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
