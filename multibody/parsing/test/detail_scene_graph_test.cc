#include "drake/multibody/parsing/detail_scene_graph.h"

#include <limits>
#include <memory>
#include <sstream>

#include <gtest/gtest.h>
#include "fmt/ostream.h"

#include "drake/common/drake_optional.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace multibody {
namespace detail {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using geometry::Box;
using geometry::Cylinder;
using geometry::GeometryInstance;
using geometry::HalfSpace;
using geometry::IllustrationProperties;
using geometry::Mesh;
using geometry::SceneGraph;
using geometry::Shape;
using geometry::Sphere;
using math::RigidTransformd;
using math::RollPitchYaw;
using math::RotationMatrix;
using multibody::detail::MakeCoulombFrictionFromSdfCollisionOde;
using multibody::detail::MakeGeometryInstanceFromSdfVisual;
using multibody::detail::MakeGeometryPoseFromSdfCollision;
using multibody::detail::MakeShapeFromSdfGeometry;
using multibody::detail::MakeVisualPropertiesFromSdfVisual;
using std::make_unique;
using std::unique_ptr;
using systems::Context;
using systems::LeafSystem;

// Helper to create an sdf::Geometry object from its SDF specification given
// as a string. Example of what the string should contain:
//   <cylinder>
//     <radius>0.5</radius>
//     <length>1.2</length>
//   </cylinder>
// and similarly for other SDF geometries.
unique_ptr<sdf::Geometry> MakeSdfGeometryFromString(
    const std::string& geometry_spec) {
  const std::string sdf_str =
      "<?xml version='1.0'?>"
      "<sdf version='1.6'>"
      "  <model name='my_model'>"
      "    <link name='link'>"
      "      <visual name='link_visual'>"
      "        <geometry>"
          + geometry_spec +
      "        </geometry>"
      "      </visual>"
      "    </link>"
      "  </model>"
      "</sdf>";
  sdf::SDFPtr sdf_parsed(new sdf::SDF());
  sdf::init(sdf_parsed);
  sdf::readString(sdf_str, sdf_parsed);
  sdf::ElementPtr geometry_element =
      sdf_parsed->Root()->GetElement("model")->
          GetElement("link")->GetElement("visual")->GetElement("geometry");
  auto sdf_geometry = make_unique<sdf::Geometry>();
  sdf_geometry->Load(geometry_element);
  return sdf_geometry;
}

// Helper to create an sdf::Visual object from its SDF specification given
// as a string. Example of what the string should contain:
//       <visual name = 'some_link_visual'>
//         <pose>1.0 2.0 3.0 3.14 6.28 1.57</pose>
//         <geometry>
//           <cylinder>
//             <radius>0.5</radius>
//             <length>1.2</length>
//           </cylinder>
//         </geometry>
//       </visual>
unique_ptr<sdf::Visual> MakeSdfVisualFromString(
    const std::string& visual_spec) {
  const std::string sdf_str =
      "<?xml version='1.0'?>"
      "<sdf version='1.6'>"
      "  <model name='my_model'>"
      "    <link name='link'>"
      + visual_spec +
      "    </link>"
      "  </model>"
      "</sdf>";
  sdf::SDFPtr sdf_parsed(new sdf::SDF());
  sdf::init(sdf_parsed);
  sdf::readString(sdf_str, sdf_parsed);
  sdf::ElementPtr visual_element =
      sdf_parsed->Root()->GetElement("model")->
          GetElement("link")->GetElement("visual");
  auto sdf_visual = make_unique<sdf::Visual>();
  sdf_visual->Load(visual_element);
  return sdf_visual;
}

// Helper to create an sdf::Collision object from its SDF specification given
// as a string. Example of what the string should contain:
//       <collision name = 'some_link_collision'>
//         <pose>1.0 2.0 3.0 3.14 6.28 1.57</pose>
//         <geometry>
//           <cylinder>
//             <radius>0.5</radius>
//             <length>1.2</length>
//           </cylinder>
//         </geometry>
//         <drake_friction>
//           <static_friction>0.8</static_friction>
//           <dynamic_friction>0.3</dynamic_friction>
//         </drake_friction>
//       </collision>
unique_ptr<sdf::Collision> MakeSdfCollisionFromString(
    const std::string& collision_spec) {
  const std::string sdf_str =
      "<?xml version='1.0'?>"
          "<sdf version='1.6'>"
          "  <model name='my_model'>"
          "    <link name='link'>"
          + collision_spec +
          "    </link>"
          "  </model>"
          "</sdf>";
  sdf::SDFPtr sdf_parsed(new sdf::SDF());
  sdf::init(sdf_parsed);
  sdf::readString(sdf_str, sdf_parsed);
  sdf::ElementPtr collision_element =
      sdf_parsed->Root()->GetElement("model")->
          GetElement("link")->GetElement("collision");
  auto sdf_collision = make_unique<sdf::Collision>();
  sdf_collision->Load(collision_element);
  return sdf_collision;
}

// Verify MakeShapeFromSdfGeometry returns nullptr when we specify an <empty>
// sdf::Geometry.
GTEST_TEST(SceneGraphParserDetail, MakeEmptyFromSdfGeometry) {
  unique_ptr<sdf::Geometry> sdf_geometry =
      MakeSdfGeometryFromString("<empty/>");
  unique_ptr<Shape> shape = MakeShapeFromSdfGeometry(*sdf_geometry);
  EXPECT_EQ(shape, nullptr);
}

// Verify MakeShapeFromSdfGeometry can make a box from an sdf::Geometry.
GTEST_TEST(SceneGraphParserDetail, MakeBoxFromSdfGeometry) {
  unique_ptr<sdf::Geometry> sdf_geometry = MakeSdfGeometryFromString(
      "<box>"
      "  <size>1.0 2.0 3.0</size>"
      "</box>");
  unique_ptr<Shape> shape = MakeShapeFromSdfGeometry(*sdf_geometry);
  const Box* box = dynamic_cast<const Box*>(shape.get());
  ASSERT_NE(box, nullptr);
  EXPECT_EQ(box->size(), Vector3d(1.0, 2.0, 3.0));
}

// Verify MakeShapeFromSdfGeometry can make a cylinder from an sdf::Geometry.
GTEST_TEST(SceneGraphParserDetail, MakeCylinderFromSdfGeometry) {
  unique_ptr<sdf::Geometry> sdf_geometry = MakeSdfGeometryFromString(
      "<cylinder>"
      "  <radius>0.5</radius>"
      "  <length>1.2</length>"
      "</cylinder>");
  unique_ptr<Shape> shape = MakeShapeFromSdfGeometry(*sdf_geometry);
  const Cylinder* cylinder = dynamic_cast<const Cylinder*>(shape.get());
  ASSERT_NE(cylinder, nullptr);
  EXPECT_EQ(cylinder->get_radius(), 0.5);
  EXPECT_EQ(cylinder->get_length(), 1.2);
}

// Verify MakeShapeFromSdfGeometry can make a sphere from an sdf::Geometry.
GTEST_TEST(SceneGraphParserDetail, MakeSphereFromSdfGeometry) {
  unique_ptr<sdf::Geometry> sdf_geometry = MakeSdfGeometryFromString(
      "<sphere>"
      "  <radius>0.5</radius>"
      "</sphere>");
  unique_ptr<Shape> shape = MakeShapeFromSdfGeometry(*sdf_geometry);
  const Sphere* sphere = dynamic_cast<const Sphere*>(shape.get());
  ASSERT_NE(sphere, nullptr);
  EXPECT_EQ(sphere->get_radius(), 0.5);
}

// Verify MakeShapeFromSdfGeometry can make a half space from an sdf::Geometry.
GTEST_TEST(SceneGraphParserDetail, MakeHalfSpaceFromSdfGeometry) {
  unique_ptr<sdf::Geometry> sdf_geometry = MakeSdfGeometryFromString(
      "<plane>"
      "  <normal>1.0 0.0 0.0</normal>"
      "  <size>1.0 1.0 1.0</size>"
      "</plane>");
  // MakeShapeFromSdfGeometry() ignores <normal> and <size> to create the
  // HalfSpace. Therefore we only verify it created the right object.
  unique_ptr<Shape> shape = MakeShapeFromSdfGeometry(*sdf_geometry);
  EXPECT_TRUE(dynamic_cast<const HalfSpace*>(shape.get()) != nullptr);
}

// Verify MakeShapeFromSdfGeometry can make a mesh from an sdf::Geometry.
GTEST_TEST(SceneGraphParserDetail, MakeMeshFromSdfGeometry) {
  // TODO(amcastro-tri): Be warned, the result of this test might (should)
  // change as we add support allowing to specify paths relative to the SDF file
  // location.
  const std::string absolute_file_path = "path/to/some/mesh.obj";
  unique_ptr<sdf::Geometry> sdf_geometry = MakeSdfGeometryFromString(
      "<mesh>"
      "  <uri>" + absolute_file_path + "</uri>"
      "  <scale> 3 3 3 </scale>"
      "</mesh>");
  unique_ptr<Shape> shape = MakeShapeFromSdfGeometry(*sdf_geometry);
  const Mesh* mesh = dynamic_cast<const Mesh*>(shape.get());
  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->filename(), absolute_file_path);
  EXPECT_EQ(mesh->scale(), 3);
}

// Verify MakeGeometryInstanceFromSdfVisual can make a GeometryInstance from an
// sdf::Visual.
// Since we test MakeShapeFromSdfGeometry separately, there is no need to unit
// test every combination of a <visual> with a different <geometry>.
GTEST_TEST(SceneGraphParserDetail, MakeGeometryInstanceFromSdfVisual) {
  unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
      "<visual name = 'some_link_visual'>"
      "  <pose>1.0 2.0 3.0 3.14 6.28 1.57</pose>"
      "  <geometry>"
      "    <cylinder>"
      "      <radius>0.5</radius>"
      "      <length>1.2</length>"
      "    </cylinder>"
      "  </geometry>"
      "</visual>");
  unique_ptr<GeometryInstance> geometry_instance =
      MakeGeometryInstanceFromSdfVisual(*sdf_visual);

  const RigidTransformd X_LC(geometry_instance->pose());

  // These are the expected values as specified by the string above.
  const Vector3d expected_rpy(3.14, 6.28, 1.57);
  const Matrix3d R_LC_expected =
      RotationMatrix<double>(RollPitchYaw<double>(expected_rpy)).matrix();
  const Vector3d p_LCo_expected(1.0, 2.0, 3.0);

  // Verify results to precision given by kTolerance.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(X_LC.linear(), R_LC_expected,
                              kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(X_LC.translation(), p_LCo_expected,
                              kTolerance, MatrixCompareType::relative));
}

// Confirms the failure conditions for SDFormat. SceneGraph requirements on
// geometry names are supposed to mirror the SDFormat behavior. If these tests
// no longer fail, the requirements in SceneGraph should become more relaxed.
// Alternatively, if more failure modes are learned, they should be encoded
// here and in the SceneGraph logic (start at the documentation of
// GeometryInstace).
// Note: This is only tested for visual geometries, but the same requirements
// are assumed for collision geometries.
GTEST_TEST(SceneGraphParserDetail, VisualGeometryNameRequirements) {
  // It is necessary to do a full, deep parse from the root to reveal *all*
  // of the failure modes.

  // A fmt::format-compatible string for testing various permutations of visual
  // names.
  const std::string visual_tag =
      "<visual name='{}'>"
      "  <pose>1.0 2.0 3.0 3.14 6.28 1.57</pose>"
      "  <geometry>"
      "    <cylinder>"
      "      <radius>0.5</radius>"
      "      <length>1.2</length>"
      "    </cylinder>"
      "  </geometry>"
      "</visual>";

  auto valid_parse = [](const std::string& visual_str) -> bool {
    const std::string sdf_str = fmt::format(
        "<?xml version='1.0'?>"
        "<sdf version='1.6'>"
        "  <model name='my_model'>"
        "    <link name='link'>{}"
        "    </link>"
        "  </model>"
        "</sdf>",
        visual_str);
    sdf::Root root;
    auto errors = root.LoadSdfString(sdf_str);
    return errors.empty();
  };

  // Allowable naming.
  // Case: control group - a simple valid name.
  EXPECT_TRUE(valid_parse(fmt::format(visual_tag, "visual")));

  // Case: Valid name with leading whitespace.
  EXPECT_TRUE(valid_parse(fmt::format(visual_tag, "  visual")));

  // Case: Valid name with trailing whitespace.
  EXPECT_TRUE(valid_parse(fmt::format(visual_tag, "visual   ")));

  // These whitespace characters are *not* considered to be whitespace by SDF.
  std::vector<std::pair<char, std::string>> ignored_whitespace{
      {'\n', "\\n"}, {'\v', "\\v"}, {'\r', "\\r"}, {'\f', "\\f"}};
  for (const auto& pair : ignored_whitespace) {
    // Case: Whitespace-only name.
    EXPECT_TRUE(valid_parse(fmt::format(visual_tag, pair.first)))
        << "Failed on " << pair.second;
  }

  {
    // Case: Same name for two different geometry types (collision vs visual).
    const std::string collision_tag =
        "<collision name='thing'>"
        "  <pose>1.0 2.0 3.0 3.14 6.28 1.57</pose>"
        "  <geometry>"
        "    <sphere/>"
        "  </geometry>"
        "</collision>";
    EXPECT_TRUE(valid_parse(fmt::format(visual_tag, "thing") + collision_tag));
  }

  // Invalid naming
  {
    // Case: Missing name element.
    const std::string missing_name_parameter =
        "<visual>"
        "  <pose>1.0 2.0 3.0 3.14 6.28 1.57</pose>"
        "  <geometry>"
        "    <cylinder>"
        "      <radius>0.5</radius>"
        "      <length>1.2</length>"
        "    </cylinder>"
        "  </geometry>"
        "</visual>";
    EXPECT_FALSE(valid_parse(missing_name_parameter));
  }

  // Case: Empty name element.
  EXPECT_FALSE(valid_parse(fmt::format(visual_tag, "")));

  std::vector<std::pair<char, std::string>> invalid_whitespace{{' ', "space"},
                                                               {'\t', "\\t"}};
  for (const auto& pair : invalid_whitespace) {
    // Case: Whitespace-only name.
    EXPECT_FALSE(valid_parse(fmt::format(visual_tag, pair.first)))
        << "Failed on " << pair.second;
  }

  // Case: Duplicate names.
  EXPECT_FALSE(valid_parse(fmt::format(visual_tag, "visual") +
                           fmt::format(visual_tag, "visual")));

  // Case: Duplicate names which arise from trimming whitespace.
  EXPECT_FALSE(valid_parse(fmt::format(visual_tag, "visual  ") +
                           fmt::format(visual_tag, "  visual")));
}

// Verify MakeGeometryInstanceFromSdfVisual can make a GeometryInstance from an
// sdf::Visual with a <plane> geometry.
// We test this case separately since, while geometry::HalfSpace is defined in a
// canonical frame C whose pose needs to be specified at a GeometryInstance
// level, the SDF specification does not define this pose at the <geometry>
// level but at the <visual> (or <collision>) level.
GTEST_TEST(SceneGraphParserDetail, MakeHalfSpaceGeometryInstanceFromSdfVisual) {
  unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
      "<visual name = 'some_link_visual'>"
      "  <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>"
      "  <geometry>"
      "    <plane>"
      "      <normal>1.0 2.0 3.0</normal>"
      "    </plane>"
      "  </geometry>"
      "</visual>");
  unique_ptr<GeometryInstance> geometry_instance =
      MakeGeometryInstanceFromSdfVisual(*sdf_visual);

  // Verify we do have a plane geometry.
  const HalfSpace* shape =
      dynamic_cast<const HalfSpace*>(&geometry_instance->shape());
  ASSERT_TRUE(shape != nullptr);

  // The expected coordinates of the normal vector in the link frame L.
  const Vector3d normal_L_expected = Vector3d(1.0, 2.0, 3.0).normalized();

  // The expected orientation of the canonical frame C (in which the plane's
  // normal aligns with Cz) in the link frame L.
  const Matrix3d R_LC_expected =
      HalfSpace::MakePose(normal_L_expected, Vector3d::Zero()).linear();

  // Retrieve the GeometryInstance pose as parsed from the sdf::Visual.
  const RigidTransformd X_LC(geometry_instance->pose());
  const Matrix3d& R_LC = X_LC.linear();
  const Vector3d normal_L = R_LC.col(2);

  // Verify results to precision given by kTolerance.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(X_LC.linear(), R_LC_expected,
                              kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(normal_L, normal_L_expected,
                              kTolerance, MatrixCompareType::relative));
}

// Verify MakeSdfVisualFromString() returns nullptr when the visual specifies
// an <empty/> geometry.
GTEST_TEST(SceneGraphParserDetail, MakeEmptyGeometryInstanceFromSdfVisual) {
  unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
      "<visual name = 'some_link_visual'>"
      "  <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>"
      "  <geometry>"
      "    <empty/>"
      "  </geometry>"
      "</visual>");
  unique_ptr<GeometryInstance> geometry_instance =
      MakeGeometryInstanceFromSdfVisual(*sdf_visual);
  EXPECT_EQ(geometry_instance, nullptr);
}

// Verify visual material parsing: default for unspecified, and diffuse color
// given where specified in the SDF.
GTEST_TEST(SceneGraphParserDetail, ParseVisualMaterial) {
  using Eigen::Vector4d;

  // Searches the illustration properties for an optional phong material
  // specification with optional color values.
  auto expect_phong = [](
      const IllustrationProperties& dut, bool has_group,
      const optional<Vector4d> diffuse, const optional<Vector4d> specular,
      const optional<Vector4d> ambient,
      const optional<Vector4d> emissive) -> ::testing::AssertionResult {
    ::testing::AssertionResult failure = ::testing::AssertionFailure();
    bool success = true;
    if (has_group) {
      if (dut.HasGroup("phong")) {
        auto test_color = [&failure, &success, &dut](
            const char* name, const optional<Vector4d> ref_color) {
          const bool has_property = dut.HasProperty("phong", name);
          if (ref_color) {
            if (has_property) {
              const Vector4d& color = dut.GetProperty<Vector4d>("phong", name);
              auto result = CompareMatrices(*ref_color, color);
              if (result == ::testing::AssertionFailure()) {
                success = false;
                failure << "Incorrect values for '" << name << "':"
                        << "\n expected: " << (*ref_color)
                        << "\n found:    " << color;
              }
            } else {
              success = false;
              failure << ", missing expected property '" << name << "'";
            }
          } else {
            if (has_property) {
              success = false;
              failure << ", found unexpected property '" << name << "'";
            }
          }
        };
        test_color("diffuse", diffuse);
        test_color("specular", specular);
        test_color("ambient", ambient);
        test_color("emissive", emissive);
      } else {
        failure << ", missing the expected 'phong' group";
        success = false;
      }
    } else {
      if (dut.HasGroup("phong")) {
        failure << ", found unexpected 'phong' group";
        success = false;
      }
    }
    if (success) {
      return ::testing::AssertionSuccess();
    } else {
      return failure;
    }
  };

  // Builds a visual XML tag with an optional <material> tag and optional
  // color values.
  auto make_xml = [](bool has_material, Vector4d* diffuse, Vector4d* specular,
                     Vector4d* ambient, Vector4d* emissive) {
    std::stringstream ss;
    ss << "<visual name='some_link_visual'>"
       << "  <pose>0 0 0 0 0 0</pose>"
       << "  <geometry>"
       << "    <sphere>"
       << "      <radius>1</radius>"
       << "    </sphere>"
       << "  </geometry>";
    if (has_material) {
      auto write_color = [&ss](const char* name, const Vector4d* color) {
        if (color) {
          const Vector4d& c = *color;
          ss << fmt::format("    <{0}>{1} {2} {3} {4}</{0}>", name,
                            c(0), c(1), c(2), c(3));
        }
      };
      ss << "  <material>";
      write_color("diffuse", diffuse);
      write_color("specular", specular);
      write_color("ambient", ambient);
      write_color("emissive", emissive);
      ss << "  </material>";
    }
    ss << "</visual>";

    return ss.str();
  };

  // Case: No material defined -- empty illustration properties.
  {
    unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
        make_xml(false, nullptr, nullptr, nullptr, nullptr));
    IllustrationProperties material =
        MakeVisualPropertiesFromSdfVisual(*sdf_visual);
    EXPECT_TRUE(
        expect_phong(material, false, {}, {}, {}, {}));
  }

  // Case: Material tag defined, but no material properties -- empty
  // illustration properties.
  {
    unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
        make_xml(true, nullptr, nullptr, nullptr, nullptr));
    IllustrationProperties material =
        MakeVisualPropertiesFromSdfVisual(*sdf_visual);
    EXPECT_TRUE(
        expect_phong(material, false, {}, {}, {}, {}));
  }

  Vector4<double> diffuse{0.25, 0.5, 0.75, 1.0};
  Vector4<double> specular{0.5, 0.75, 1.0, 0.25};
  Vector4<double> ambient{0.75, 1.0, 0.25, 0.5};
  Vector4<double> emissive{1.0, 0.25, 0.5, 0.75};

  // Case: Only valid diffuse material.
  {
    unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
        make_xml(true, &diffuse, nullptr, nullptr, nullptr));
    IllustrationProperties material =
        MakeVisualPropertiesFromSdfVisual(*sdf_visual);
    EXPECT_TRUE(
        expect_phong(material, true, diffuse, {}, {}, {}));
  }

  // Case: Only valid specular defined.
  {
    unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
        make_xml(true, nullptr, &specular, nullptr, nullptr));
    IllustrationProperties material =
        MakeVisualPropertiesFromSdfVisual(*sdf_visual);
    EXPECT_TRUE(
        expect_phong(material, true, {}, specular, {}, {}));
  }

  // Case: Only valid ambient defined.
  {
    unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
        make_xml(true, nullptr, nullptr, &ambient, nullptr));
    IllustrationProperties material =
        MakeVisualPropertiesFromSdfVisual(*sdf_visual);
    EXPECT_TRUE(
        expect_phong(material, true, {}, {}, ambient, {}));
  }

  // Case: Only valid emissive defined.
  {
    unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
        make_xml(true, nullptr, nullptr, nullptr, &emissive));
    IllustrationProperties material =
        MakeVisualPropertiesFromSdfVisual(*sdf_visual);
    EXPECT_TRUE(
        expect_phong(material, true, {}, {}, {}, emissive));
  }

  // Case: All four
  {
    unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
        make_xml(true, &diffuse, &specular, &ambient, &emissive));
    IllustrationProperties material =
        MakeVisualPropertiesFromSdfVisual(*sdf_visual);
    EXPECT_TRUE(
        expect_phong(material, true, diffuse, specular, ambient, emissive));
  }

  // TODO(SeanCurtis-TRI): The following tests capture current behavior for
  // sdformat. The behavior isn't necessarily desirable and an issue has been
  // filed.
  // https://bitbucket.org/osrf/sdformat/issues/193/using-element-get-has-surprising-defaults
  // When this issue gets resolved, modify these tests accordingly.

  // sdformat maps the diffuse values into a `Color` using the following rules:
  //   1. Truncate to no more than four values (more than 4 values are simply
  //      ignored).
  //   2. If fewer than four, use 1 for a default alpha value and zero for
  //      default r, g, b values.

  // Case: Too many channel values -- truncate.
  {
    unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
        "<visual name='some_link_visual'>"
        "  <pose>0 0 0 0 0 0</pose>"
        "  <geometry>"
        "    <sphere>"
        "      <radius>1</radius>"
        "    </sphere>"
        "  </geometry>"
        "  <material>"
        "    <diffuse>0.25 1 0.5 0.25 2</diffuse>"
        "  </material>"
        "</visual>");
    IllustrationProperties material =
        MakeVisualPropertiesFromSdfVisual(*sdf_visual);
    Vector4<double> expected_diffuse{0.25, 1, 0.5, 0.25};
    EXPECT_TRUE(expect_phong(material, true, expected_diffuse, {}, {}, {}));
  }

  // Case: Too few channel values -- fill in with 0 for b and 1 for alpha.
  {
    unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
        "<visual name='some_link_visual'>"
        "  <pose>0 0 0 0 0 0</pose>"
        "  <geometry>"
        "    <sphere>"
        "      <radius>1</radius>"
        "    </sphere>"
        "  </geometry>"
        "  <material>"
        "    <diffuse>0 1</diffuse>"
        "  </material>"
        "</visual>");
    IllustrationProperties material =
        MakeVisualPropertiesFromSdfVisual(*sdf_visual);
    Vector4<double> expected_diffuse{0, 1, 0, 1};
    EXPECT_TRUE(expect_phong(material, true, expected_diffuse, {}, {}, {}));
  }

  // Case: Values out of range:
  //  Alpha simply gets clamped to the range [0, 1]
  //  Negative R, G, B get set to zero.
  //  R, G, B > 1 get divided by 255.
  // These rules don't guarantee valid values.
  {
    unique_ptr<sdf::Visual> sdf_visual = MakeSdfVisualFromString(
        "<visual name='some_link_visual'>"
        "  <pose>0 0 0 0 0 0</pose>"
        "  <geometry>"
        "    <sphere>"
        "      <radius>1</radius>"
        "    </sphere>"
        "  </geometry>"
        "  <material>"
        "    <diffuse>-0.1 255 65025 2</diffuse>"
        "  </material>"
        "</visual>");
    IllustrationProperties material =
        MakeVisualPropertiesFromSdfVisual(*sdf_visual);
    Vector4<double> expected_diffuse{0, 1, 255, 1};
    EXPECT_TRUE(expect_phong(material, true, expected_diffuse, {}, {}, {}));
  }
}

// Verify MakeGeometryPoseFromSdfCollision() makes the pose X_LG of geometry
// frame G in the link frame L.
// Since we test MakeShapeFromSdfGeometry separately, there is no need to unit
// test every combination of a <collision> with a different <geometry>.
GTEST_TEST(SceneGraphParserDetail, MakeGeometryPoseFromSdfCollision) {
  unique_ptr<sdf::Collision> sdf_collision = MakeSdfCollisionFromString(
      "<collision name = 'some_link_collision'>"
      "  <pose>1.0 2.0 3.0 3.14 6.28 1.57</pose>"
      "  <geometry>"
      "    <sphere/>"
      "  </geometry>"
      "</collision>");
  const RigidTransformd X_LG = MakeGeometryPoseFromSdfCollision(*sdf_collision);

  // These are the expected values as specified by the string above.
  const Vector3d expected_rpy(3.14, 6.28, 1.57);
  const Matrix3d R_LG_expected =
      RotationMatrix<double>(RollPitchYaw<double>(expected_rpy)).matrix();
  const Vector3d p_LGo_expected(1.0, 2.0, 3.0);

  // Verify results to precision given by kTolerance.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(X_LG.linear(), R_LG_expected,
                              kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(X_LG.translation(), p_LGo_expected,
                              kTolerance, MatrixCompareType::relative));
}

// Verify MakeGeometryPoseFromSdfCollision can make the pose X_LG of the
// geometry frame G in the link frame L when the specified shape is a plane.
// We test this case separately since, while geometry::HalfSpace is defined in a
// canonical frame C whose pose needs to be specified at a GeometryInstance
// level, the SDF specification does not define this pose at the <geometry>
// level but at the <collision> level.
GTEST_TEST(SceneGraphParserDetail,
           MakeGeometryPoseFromSdfCollisionForHalfSpace) {
  unique_ptr<sdf::Collision> sdf_collision = MakeSdfCollisionFromString(
      "<collision name = 'some_link_collision'>"
      "  <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>"
      "  <geometry>"
      "    <plane>"
      "      <normal>1.0 2.0 3.0</normal>"
      "    </plane>"
      "  </geometry>"
      "</collision>");
  const RigidTransformd X_LG = MakeGeometryPoseFromSdfCollision(*sdf_collision);

  // The expected coordinates of the normal vector in the link frame L.
  const Vector3d normal_L_expected = Vector3d(1.0, 2.0, 3.0).normalized();

  // The expected orientation of the canonical frame C (in which the plane's
  // normal aligns with Cz) in the link frame L.
  const Matrix3d R_LG_expected =
      HalfSpace::MakePose(normal_L_expected, Vector3d::Zero()).linear();

  // Verify results to precision given by kTolerance.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(X_LG.linear(), R_LG_expected,
                              kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(X_LG.translation(), Vector3d::Zero(),
                              kTolerance, MatrixCompareType::relative));
}

// Verify we can parse friction coefficients from an <ode> element in
// <collision><surface><friction>. Drake understands <mu> to be the static
// coefficient and <mu2> the dynamic coefficient of friction.
GTEST_TEST(SceneGraphParserDetail, MakeCoulombFrictionFromSdfCollisionOde) {
  unique_ptr<sdf::Collision> sdf_collision = MakeSdfCollisionFromString(
      "<collision name = 'some_link_collision'>"
      "  <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>"
      "  <geometry>"
      "    <plane>"
      "      <normal>1.0 2.0 3.0</normal>"
      "    </plane>"
      "  </geometry>"
      "  <surface>"
      "    <friction>"
      "      <ode>"
      "        <mu>0.8</mu>"
      "        <mu2>0.3</mu2>"
      "      </ode>"
      "    </friction>"
      "  </surface>"
      "</collision>");
  const CoulombFriction<double> friction =
      MakeCoulombFrictionFromSdfCollisionOde(*sdf_collision);
  EXPECT_EQ(friction.static_friction(), 0.8);
  EXPECT_EQ(friction.dynamic_friction(), 0.3);
}

// Verify MakeCoulombFrictionFromSdfCollisionOde() throws an exception if
// provided a dynamic friction coefficient larger than the static friction
// coefficient.
// We do not need testing for each possible case of an invalid input such as:
// - negative coefficients.
// - dynamic > static.
// - only one coefficient is negative.
// Since class CoulombFriction performs these tests at construction and its unit
// tests provide coverage for these cases. In that regard, this following
// test is not needed but we provide it just to show how the exception message
// thrown from CoulombFriction gets concatenated and re-thrown by
// MakeCoulombFrictionFromSdfCollisionOde().
GTEST_TEST(SceneGraphParserDetail,
           MakeCoulombFrictionFromSdfCollisionOde_DynamicLargerThanStatic) {
  unique_ptr<sdf::Collision> sdf_collision = MakeSdfCollisionFromString(
      "<collision name = 'some_link_collision'>"
      "  <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>"
      "  <geometry>"
      "    <plane>"
      "      <normal>1.0 2.0 3.0</normal>"
      "    </plane>"
      "  </geometry>"
      "  <surface>"
      "    <friction>"
      "      <ode>"
      "        <mu>0.3</mu>"
      "        <mu2>0.8</mu2>"
      "      </ode>"
      "    </friction>"
      "  </surface>"
      "</collision>");
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeCoulombFrictionFromSdfCollisionOde(*sdf_collision),
      std::logic_error,
      "The given dynamic friction \\(.*\\) is greater than the given static "
      "friction \\(.*\\); dynamic friction must be less than or equal to "
      "static friction.");
}

GTEST_TEST(SceneGraphParserDetail,
           MakeCoulombFrictionFromSdfCollisionOde_MuMissing) {
  unique_ptr<sdf::Collision> sdf_collision = MakeSdfCollisionFromString(
      "<collision name = 'some_link_collision'>"
      "  <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>"
      "  <geometry>"
      "    <plane>"
      "      <normal>1.0 2.0 3.0</normal>"
      "    </plane>"
      "  </geometry>"
      "  <surface>"
      "    <friction>"
      "      <ode>"
      "        <mu2>0.8</mu2>"
      "      </ode>"
      "    </friction>"
      "  </surface>"
      "</collision>");
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeCoulombFrictionFromSdfCollisionOde(*sdf_collision),
      std::runtime_error,
      "Element <mu> is required within element <ode>.");
}

GTEST_TEST(SceneGraphParserDetail,
           MakeCoulombFrictionFromSdfCollisionOde_Mu2Missing) {
  unique_ptr<sdf::Collision> sdf_collision = MakeSdfCollisionFromString(
      "<collision name = 'some_link_collision'>"
      "  <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>"
      "  <geometry>"
      "    <plane>"
      "      <normal>1.0 2.0 3.0</normal>"
      "    </plane>"
      "  </geometry>"
      "  <surface>"
      "    <friction>"
      "      <ode>"
      "        <mu>0.8</mu>"
      "      </ode>"
      "    </friction>"
      "  </surface>"
      "</collision>");
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeCoulombFrictionFromSdfCollisionOde(*sdf_collision),
      std::runtime_error,
      "Element <mu2> is required within element <ode>.");
}

GTEST_TEST(SceneGraphParserDetail,
           MakeCoulombFrictionFromSdfCollisionOde_FrictionMissing) {
  unique_ptr<sdf::Collision> sdf_collision = MakeSdfCollisionFromString(
      "<collision name = 'some_link_collision'>"
      "  <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>"
      "  <geometry>"
      "    <plane>"
      "      <normal>1.0 2.0 3.0</normal>"
      "    </plane>"
      "  </geometry>"
      "  <surface>"
      "    <ode>"
      "      <mu>0.3</mu>"
      "      <mu2>0.8</mu2>"
      "    </ode>"
      "  </surface>"
      "</collision>");
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeCoulombFrictionFromSdfCollisionOde(*sdf_collision),
      std::runtime_error,
      "Element <friction> not found nested within element <surface>.");
}

GTEST_TEST(SceneGraphParserDetail,
           MakeCoulombFrictionFromSdfCollisionOde_OdeMissing) {
  unique_ptr<sdf::Collision> sdf_collision = MakeSdfCollisionFromString(
      "<collision name = 'some_link_collision'>"
          "  <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>"
          "  <geometry>"
          "    <plane>"
          "      <normal>1.0 2.0 3.0</normal>"
          "    </plane>"
          "  </geometry>"
          "  <surface>"
          "    <friction>"
          "      <mu>0.3</mu>"
          "      <mu2>0.8</mu2>"
          "    </friction>"
          "  </surface>"
          "</collision>");
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeCoulombFrictionFromSdfCollisionOde(*sdf_collision),
      std::runtime_error,
      "Element <ode> not found nested within element <friction>.");
}

}  // namespace
}  // namespace detail
}  // namespace multibody
}  // namespace drake

