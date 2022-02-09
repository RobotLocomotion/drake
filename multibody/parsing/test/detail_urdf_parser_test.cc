#include "drake/multibody/parsing/detail_urdf_parser.h"

#include <fstream>
#include <limits>
#include <stdexcept>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/universal_joint.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
using drake::internal::DiagnosticPolicy;
using geometry::GeometryId;
using geometry::SceneGraph;

class UrdfParserTest : public ::testing::Test {
 public:
  UrdfParserTest() {
    plant_.RegisterAsSourceForSceneGraph(&scene_graph_);
  }

  ModelInstanceIndex AddModelFromUrdfFile(
      const std::string& file_name,
      const std::string& model_name) {
    return AddModelFromUrdf({ .file_name = &file_name }, model_name, {}, w_);
  }

 protected:
  PackageMap package_map_;
  DiagnosticPolicy diagnostic_;
  MultibodyPlant<double> plant_{0.0};
  SceneGraph<double> scene_graph_;
  ParsingWorkspace w_{package_map_, diagnostic_, &plant_};
};

// Verifies that the URDF loader can leverage a specified package map.
TEST_F(UrdfParserTest, PackageMapSpecified) {
  // We start with the world and default model instances (model_instance.h
  // explains why there are two).
  ASSERT_EQ(plant_.num_model_instances(), 2);

  const std::string full_urdf_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/urdfs/box.urdf");
  filesystem::path package_path = full_urdf_filename;
  package_path = package_path.parent_path();
  package_path = package_path.parent_path();

  // Construct the PackageMap.
  package_map_.PopulateFromFolder(package_path.string());

  // Read in the URDF file.
  AddModelFromUrdfFile(full_urdf_filename, "");
  plant_.Finalize();

  // Verify the number of model instances.
  EXPECT_EQ(plant_.num_model_instances(), 3);
}

TEST_F(UrdfParserTest, DoublePendulum) {
  std::string full_name = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/double_pendulum.urdf");
  AddModelFromUrdfFile(full_name, "");
  plant_.Finalize();

  EXPECT_EQ(plant_.num_bodies(), 4);
  EXPECT_EQ(plant_.num_frames(), 10);

  ASSERT_TRUE(plant_.HasFrameNamed("frame_on_link1"));
  ASSERT_TRUE(plant_.HasFrameNamed("frame_on_link2"));
  ASSERT_TRUE(plant_.HasFrameNamed("link1_com"));
  ASSERT_TRUE(plant_.HasFrameNamed("link2_com"));

  // Sample a couple of frames.
  const Frame<double>& frame_on_link1 = plant_.GetFrameByName("frame_on_link1");
  EXPECT_EQ(frame_on_link1.body().index(),
            plant_.GetBodyByName("link1").index());

  math::RollPitchYaw<double> rpy_expected(-1, 0.1, 0.2);
  Vector3d xyz_expected(0.8, -0.2, 0.3);
  math::RigidTransform<double> X_BF_expected(rpy_expected.ToRotationMatrix(),
                                             xyz_expected);

  EXPECT_TRUE(CompareMatrices(
      frame_on_link1.GetFixedPoseInBodyFrame().GetAsMatrix4(),
      X_BF_expected.GetAsMatrix4(), 1e-10));

  const Frame<double>& link2_com = plant_.GetFrameByName("link2_com");
  EXPECT_EQ(link2_com.body().index(), plant_.GetBodyByName("link2").index());
}

// This test verifies that we're able to successfully look up meshes using the
// "package://" syntax internally to the URDF (at least for packages which are
// successfully found in the same directory at the URDF.
TEST_F(UrdfParserTest, TestAtlasMinimalContact) {
  std::string full_name = FindResourceOrThrow(
      "drake/examples/atlas/urdf/atlas_minimal_contact.urdf");
  AddModelFromUrdfFile(full_name, "");
  plant_.Finalize();

  EXPECT_EQ(plant_.num_positions(), 37);
  EXPECT_EQ(plant_.num_velocities(), 36);

  // Verify that joint actuator limits are set correctly.
  ASSERT_TRUE(plant_.HasJointActuatorNamed("back_bkz_motor"));
  const JointActuator<double>& actuator =
      plant_.GetJointActuatorByName("back_bkz_motor");
  EXPECT_EQ(actuator.effort_limit(), 106);
}

TEST_F(UrdfParserTest, TestAddWithQuaternionFloatingDof) {
  const std::string resource_dir{
      "drake/multibody/parsing/test/urdf_parser_test/"};
  const std::string model_file =
      FindResourceOrThrow(resource_dir + "zero_dof_robot.urdf");
  AddModelFromUrdfFile(model_file, "");
  plant_.Finalize();

  EXPECT_EQ(plant_.num_positions(), 7);
  EXPECT_EQ(plant_.num_velocities(), 6);
}

TEST_F(UrdfParserTest, TestRegisteredSceneGraph) {
  const std::string full_name = FindResourceOrThrow(
      "drake/examples/atlas/urdf/atlas_minimal_contact.urdf");
  // Test that registration with scene graph results in visual geometries.
  AddModelFromUrdfFile(full_name, "");
  plant_.Finalize();
  EXPECT_NE(plant_.num_visual_geometries(), 0);
}

TEST_F(UrdfParserTest, JointParsingTest) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "joint_parsing_test.urdf");
  AddModelFromUrdfFile(full_name, "");
  plant_.Finalize();

  // Revolute joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<RevoluteJoint>("revolute_joint"));
  const RevoluteJoint<double>& revolute_joint =
      plant_.GetJointByName<RevoluteJoint>("revolute_joint");
  EXPECT_EQ(revolute_joint.name(), "revolute_joint");
  EXPECT_EQ(revolute_joint.parent_body().name(), "link1");
  EXPECT_EQ(revolute_joint.child_body().name(), "link2");
  EXPECT_EQ(revolute_joint.revolute_axis(), Vector3d::UnitZ());
  EXPECT_EQ(revolute_joint.damping(), 0.1);
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.position_lower_limits(), Vector1d(-1)));
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.position_upper_limits(), Vector1d(2)));
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.velocity_lower_limits(), Vector1d(-100)));
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.velocity_upper_limits(), Vector1d(100)));
  EXPECT_TRUE(CompareMatrices(
      revolute_joint.acceleration_lower_limits(), Vector1d(-200)));
  EXPECT_TRUE(CompareMatrices(
      revolute_joint.acceleration_upper_limits(), Vector1d(200)));

  // Revolute actuator
  const JointActuator<double>& revolute_actuator =
      plant_.GetJointActuatorByName("revolute_actuator");
  EXPECT_EQ(revolute_actuator.effort_limit(), 100);

  // Prismatic joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<PrismaticJoint>("prismatic_joint"));
  const PrismaticJoint<double>& prismatic_joint =
      plant_.GetJointByName<PrismaticJoint>("prismatic_joint");
  EXPECT_EQ(prismatic_joint.name(), "prismatic_joint");
  EXPECT_EQ(prismatic_joint.parent_body().name(), "link2");
  EXPECT_EQ(prismatic_joint.child_body().name(), "link3");
  EXPECT_EQ(prismatic_joint.translation_axis(), Vector3d::UnitZ());
  EXPECT_EQ(prismatic_joint.damping(), 0.1);
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.position_lower_limits(), Vector1d(-2)));
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.position_upper_limits(), Vector1d(1)));
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.velocity_lower_limits(), Vector1d(-5)));
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.velocity_upper_limits(), Vector1d(5)));
  EXPECT_TRUE(CompareMatrices(
      prismatic_joint.acceleration_lower_limits(), Vector1d(-10)));
  EXPECT_TRUE(CompareMatrices(
      prismatic_joint.acceleration_upper_limits(), Vector1d(10)));
  EXPECT_FALSE(plant_.HasJointActuatorNamed("prismatic_actuator"));

  // Ball joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<BallRpyJoint>("ball_joint"));
  const BallRpyJoint<double>& ball_joint =
      plant_.GetJointByName<BallRpyJoint>("ball_joint");
  EXPECT_EQ(ball_joint.name(), "ball_joint");
  EXPECT_EQ(ball_joint.parent_body().name(), "link3");
  EXPECT_EQ(ball_joint.child_body().name(), "link4");
  EXPECT_EQ(ball_joint.damping(), 0.1);
  const Vector3d inf3(std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity());
  const Vector3d neg_inf3(-std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity());
  EXPECT_TRUE(CompareMatrices(ball_joint.position_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(ball_joint.position_upper_limits(), inf3));
  EXPECT_TRUE(CompareMatrices(ball_joint.velocity_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(ball_joint.velocity_upper_limits(), inf3));
  EXPECT_GT(ball_joint.index(), prismatic_joint.index());

  // Limitless revolute joint
  const Joint<double>& no_limit_joint =
      plant_.GetJointByName("revolute_joint_no_limits");
  const Vector1d inf(std::numeric_limits<double>::infinity());
  const Vector1d neg_inf(-std::numeric_limits<double>::infinity());

  EXPECT_TRUE(CompareMatrices(no_limit_joint.position_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.position_upper_limits(), inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.velocity_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.velocity_upper_limits(), inf));
  EXPECT_TRUE(CompareMatrices(
      no_limit_joint.acceleration_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.acceleration_upper_limits(), inf));
  EXPECT_GT(no_limit_joint.index(), ball_joint.index());

  // Limitless revolute actuator
  const JointActuator<double>& revolute_actuator_no_limits =
      plant_.GetJointActuatorByName("revolute_actuator_no_limits");
  EXPECT_EQ(revolute_actuator_no_limits.effort_limit(), inf(0));

  // Universal joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<UniversalJoint>("universal_joint"));
  const UniversalJoint<double>& universal_joint =
      plant_.GetJointByName<UniversalJoint>("universal_joint");
  EXPECT_EQ(universal_joint.name(), "universal_joint");
  EXPECT_EQ(universal_joint.parent_body().name(), "link5");
  EXPECT_EQ(universal_joint.child_body().name(), "link6");
  EXPECT_EQ(universal_joint.damping(), 0.1);
  const Vector2d inf2(std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity());
  const Vector2d neg_inf2(-std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity());
  EXPECT_TRUE(CompareMatrices(universal_joint.position_lower_limits(),
                              neg_inf2));
  EXPECT_TRUE(CompareMatrices(universal_joint.position_upper_limits(), inf2));
  EXPECT_TRUE(CompareMatrices(universal_joint.velocity_lower_limits(),
                              neg_inf2));
  EXPECT_TRUE(CompareMatrices(universal_joint.velocity_upper_limits(), inf2));

  // Planar joint
  DRAKE_EXPECT_NO_THROW(plant_.GetJointByName<PlanarJoint>("planar_joint"));
  const PlanarJoint<double>& planar_joint =
      plant_.GetJointByName<PlanarJoint>("planar_joint");
  EXPECT_EQ(planar_joint.name(), "planar_joint");
  EXPECT_EQ(planar_joint.parent_body().name(), "link6");
  EXPECT_EQ(planar_joint.child_body().name(), "link7");
  EXPECT_TRUE(CompareMatrices(planar_joint.damping(), Vector3d::Constant(0.1)));
  EXPECT_TRUE(CompareMatrices(planar_joint.position_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint.position_upper_limits(), inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint.velocity_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint.velocity_upper_limits(), inf3));
}

TEST_F(UrdfParserTest, JointParsingTagMismatchTest) {
  // Improperly declared joints.
  const std::string full_name_mismatch_1 = FindResourceOrThrow(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "joint_parsing_test_tag_mismatch_1.urdf");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromUrdfFile(full_name_mismatch_1, ""),
      ".*Joint fixed_joint of type fixed is a standard joint type, "
      "and should be a <joint>");

  const std::string full_name_mismatch_2 = FindResourceOrThrow(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "joint_parsing_test_tag_mismatch_2.urdf");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromUrdfFile(full_name_mismatch_2, ""),
      ".*Joint ball_joint of type ball is a custom joint type, "
      "and should be a <drake:joint>");
}

// We allow users to declare the "world" link for the purpose of declaring
// "anchored" geometry (visual and collision). Specifying inertial properties
// is not *strictly* an error -- a warning will be written to the console.
// We can't test the warning, but we'll confirm there's no error.
//
// As for the geometry, we'll simply confirm that the expected numbers of
// geometries get instantiated (with expected roles). We'll assume that because
// the geometry parsing got triggered, it is correct and ignore the other
// details.
TEST_F(UrdfParserTest, AddingGeometriesToWorldLink) {
  const std::string test_urdf = R"""(
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="joint_parsing_test">
  <link name="world">
    <!-- Declaring mass properties on the "world" link is bad. But it won't
     cause the parser to throw. -->
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.1 0.2 0.3"/>
        <material>
          <color rgba="0.8 0.7 0.6 0.5"/>
        </material>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.25"/>
      </geometry>
    </collision>
  </link>
</robot>
)""";
  DataSource source;
  source.file_contents = &test_urdf;

  AddModelFromUrdf(source, "urdf", {}, w_);

  const auto& inspector = scene_graph_.model_inspector();
  EXPECT_EQ(inspector.num_geometries(), 2);
  EXPECT_EQ(inspector.NumGeometriesForFrame(scene_graph_.world_frame_id()), 2);
  EXPECT_EQ(inspector.NumGeometriesForFrameWithRole(
                scene_graph_.world_frame_id(), geometry::Role::kProximity),
            1);
  // This does not total three geometries; the sphere has two roles,
  EXPECT_EQ(inspector.NumGeometriesForFrameWithRole(
                scene_graph_.world_frame_id(), geometry::Role::kIllustration),
            1);
  EXPECT_EQ(inspector.NumGeometriesForFrameWithRole(
                scene_graph_.world_frame_id(), geometry::Role::kPerception),
            1);
}

// Reports if the frame with the given id has a geometry with the given role
// whose name is the same as what ShapeName(ShapeType{}) would produce.
template <typename ShapeType>
::testing::AssertionResult FrameHasShape(geometry::FrameId frame_id,
                                         geometry::Role role,
                                         const SceneGraph<double>& scene_graph,
                                         const ShapeType& shape) {
  const auto& inspector = scene_graph.model_inspector();
  const std::string name = geometry::ShapeName(shape).name();
  try {
    // Note: MBP prepends the model index to the geometry name; in this case
    // that model instance name is "test_robot".
    const geometry::GeometryId geometry_id =
        inspector.GetGeometryIdByName(frame_id, role, "test_robot::" + name);
    const std::string shape_type =
        geometry::ShapeName(inspector.GetShape(geometry_id)).name();
    if (shape_type != name) {
      return ::testing::AssertionFailure()
          << "Geometry with role " << role << " has wrong shape type."
          << "\nExpected: " << name
          << "\nFound: " << shape_type;
    }
  } catch (const std::exception& e) {
    return ::testing::AssertionFailure()
        << "Frame " << frame_id << " does not have a geometry with role "
        << role << " and name " << name
        << ".\n  Exception message: " << e.what();
  }
  return ::testing::AssertionSuccess();
}

class UrdfParsedGeometryTest : public UrdfParserTest {
 public:
  // Confirms that all supported geometries in an URDF file are registered. The
  // *details* of the geometries are ignored -- we assume that that
  // functionality is tested in detail_urdf_geometry_test.cc. This merely makes
  // sure that *that* functionality is exercised appropriately.
  void TestForParsedGeometry(const char* sdf_name, geometry::Role role) {
    const std::string full_name = FindResourceOrThrow(sdf_name);
    AddModelFromUrdfFile(full_name, "");
    plant_.Finalize();

    const auto frame_id =
        plant_.GetBodyFrameIdOrThrow(plant_.GetBodyByName("link1").index());

    const std::string mesh_uri = "drake/multibody/parsing/test/tri_cube.obj";

    // Note: the parameters for the various example shapes do not matter to this
    // test.
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Box{0.1, 0.1, 0.1}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Capsule{0.1, 0.1}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Convex{mesh_uri, 1.0}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Cylinder{0.1, 0.1}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Ellipsoid{0.1, 0.1, 0.1}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Mesh{mesh_uri, 1.0}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Sphere{0.1}));
  }
};

TEST_F(UrdfParsedGeometryTest, CollisionGeometryParsing) {
  TestForParsedGeometry(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "all_geometries_as_collision.urdf",
      geometry::Role::kProximity);
}

TEST_F(UrdfParsedGeometryTest, VisualGeometryParsing) {
  TestForParsedGeometry(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "all_geometries_as_visual.urdf",
      geometry::Role::kPerception);
}

struct PlantAndSceneGraph {
  std::unique_ptr<MultibodyPlant<double>> plant;
  std::unique_ptr<SceneGraph<double>> scene_graph;
};

void ParseTestString(const std::string& inner,
                     const std::string& model_name,
                     MultibodyPlant<double>* plant) {
  std::string contents("<?xml version='1.0' ?>\n" + inner + "\n\n");
  PackageMap package_map;
  DiagnosticPolicy diagnostic;
  ParsingWorkspace workspace{package_map, diagnostic, plant};
  drake::log()->debug("inner: {}", inner);
  AddModelFromUrdf({ .file_name = nullptr, .file_contents = &contents },
                   model_name, {}, workspace);
}

PlantAndSceneGraph ParseTestString(const std::string& inner) {
  PlantAndSceneGraph pair;
  pair.plant = std::make_unique<MultibodyPlant<double>>(0.0);
  pair.scene_graph = std::make_unique<SceneGraph<double>>();
  pair.plant->RegisterAsSourceForSceneGraph(pair.scene_graph.get());
  ParseTestString(inner, "", pair.plant.get());
  return pair;
}

GTEST_TEST(MultibodyPlantUrdfParserTest, EntireInertialTagOmitted) {
  // Test that parsing a link with no inertial tag yields the expected result
  // (mass = 0, ixx = ixy = ixz = iyy = iyz = izz = 0).
  PlantAndSceneGraph pair = ParseTestString(R"""(
<robot name='entire_inertial_tag_omitted'>
  <link name='entire_inertial_tag_omitted'/>
</robot>)""");
  const RigidBody<double>* body = dynamic_cast<const RigidBody<double>*>(
    &pair.plant->GetBodyByName("entire_inertial_tag_omitted"));
  EXPECT_EQ(body->get_default_mass(), 0.);
  EXPECT_TRUE(body->default_rotational_inertia().get_moments().isZero());
  EXPECT_TRUE(body->default_rotational_inertia().get_products().isZero());
}

GTEST_TEST(MultibodyPlantUrdfParserTest, InertiaTagOmitted) {
  // Test that parsing a link with no inertia tag yields the expected result
  // (mass as specified, ixx = ixy = ixz = iyy = iyz = izz = 0).
  PlantAndSceneGraph pair = ParseTestString(R"""(
<robot name='inertia_tag_omitted'>
  <link name='inertia_tag_omitted'>
    <inertial>
      <mass value="2"/>
    </inertial>
  </link>
</robot>)""");
  const RigidBody<double>* body = dynamic_cast<const RigidBody<double>*>(
    &pair.plant->GetBodyByName("inertia_tag_omitted"));
  EXPECT_EQ(body->get_default_mass(), 2.);
  EXPECT_TRUE(body->default_rotational_inertia().get_moments().isZero());
  EXPECT_TRUE(body->default_rotational_inertia().get_products().isZero());
}

GTEST_TEST(MultibodyPlantUrdfParserTest, MassTagOmitted) {
  // Test that parsing a link with no mass tag yields the expected result
  // (mass 0, inertia as specified). Note that, because the default mass is 0,
  // we specify zero inertia here - otherwise the parsing would fail (See
  // ZeroMassNonZeroInertia below).
  PlantAndSceneGraph pair = ParseTestString(R"""(
<robot name='mass_tag_omitted'>
  <link name='mass_tag_omitted'>
    <inertial>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>
</robot>)""");
  const RigidBody<double>* body = dynamic_cast<const RigidBody<double>*>(
    &pair.plant->GetBodyByName("mass_tag_omitted"));
  EXPECT_EQ(body->get_default_mass(), 0.);
  EXPECT_TRUE(body->default_rotational_inertia().get_moments().isZero());
  EXPECT_TRUE(body->default_rotational_inertia().get_products().isZero());
}

GTEST_TEST(MultibodyPlantUrdfParserTest, MasslessBody) {
  // Test that massless bodies can be parsed.
  PlantAndSceneGraph pair = ParseTestString(R"""(
<robot name='has_massless_link'>
  <link name='massless_link'>
    <inertial>
      <mass value="0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>
</robot>)""");
  const RigidBody<double>* body = dynamic_cast<const RigidBody<double>*>(
    &pair.plant->GetBodyByName("massless_link"));
  EXPECT_EQ(body->get_default_mass(), 0.);
  EXPECT_TRUE(body->default_rotational_inertia().get_moments().isZero());
  EXPECT_TRUE(body->default_rotational_inertia().get_products().isZero());
}

GTEST_TEST(MultibodyPlantUrdfParserTest, PointMass) {
  // Test that point masses don't get sent through the massless body branch.
  PlantAndSceneGraph pair = ParseTestString(R"""(
<robot name='point_mass'>
  <link name='point_mass'>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>
</robot>)""");
  const RigidBody<double>* body = dynamic_cast<const RigidBody<double>*>(
    &pair.plant->GetBodyByName("point_mass"));
  EXPECT_EQ(body->get_default_mass(), 1.);
  EXPECT_TRUE(body->default_rotational_inertia().get_moments().isZero());
  EXPECT_TRUE(body->default_rotational_inertia().get_products().isZero());
}

namespace {
  void ParseZeroMassNonZeroInertia() {
    ParseTestString(R"""(
<robot name='bad'>
  <link name='bad'>
    <inertial>
      <mass value="0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>
</robot>)""");
  }
}  // namespace

GTEST_TEST(MultibodyPlantUrdfParserTest, ZeroMassNonZeroInertia) {
  // Test that attempt to parse links with zero mass and non-zero inertia fails.
  if (!::drake::kDrakeAssertIsArmed) {
    EXPECT_THROW(ParseZeroMassNonZeroInertia(), std::runtime_error);
  }
}

GTEST_TEST(MultibodyPlantUrdfParserDeathTest, ZeroMassNonZeroInertia) {
  // Test that attempt to parse links with zero mass and non-zero inertia fails.
  const std::string expected_message = ".*condition 'mass > 0' failed.";
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseZeroMassNonZeroInertia(), expected_message);
}

GTEST_TEST(MultibodyPlantUrdfParserTest, BushingParsing) {
  // Test successful parsing.
  const std::string good_bushing_model = R"(
    <robot name="bushing_test">
        <link name='A'/>
        <link name='C'/>
        <frame name="frameA" link="A" rpy="0 0 0" xyz="0 0 0"/>
        <frame name="frameC" link="C" rpy="0 0 0" xyz="0 0 0"/>
        <drake:linear_bushing_rpy>
            <drake:bushing_frameA name="frameA"/>
            <drake:bushing_frameC name="frameC"/>
            <drake:bushing_torque_stiffness value="1 2 3"/>
            <drake:bushing_torque_damping   value="4 5 6"/>
            <drake:bushing_force_stiffness  value="7 8 9"/>
            <drake:bushing_force_damping    value="10 11 12"/>
        </drake:linear_bushing_rpy>
    </robot>)";

  auto [plant, scene_graph] = ParseTestString(good_bushing_model);
  ParseTestString(good_bushing_model, "bushing2", plant.get());

  // MBP will always create a UniformGravityField, so the only other
  // ForceElement should be the LinearBushingRollPitchYaw element parsed.
  EXPECT_EQ(plant->num_force_elements(), 3);

  const LinearBushingRollPitchYaw<double>& bushing =
      plant->GetForceElement<LinearBushingRollPitchYaw>(ForceElementIndex(1));

  EXPECT_STREQ(bushing.frameA().name().c_str(), "frameA");
  EXPECT_STREQ(bushing.frameC().name().c_str(), "frameC");
  EXPECT_EQ(bushing.torque_stiffness_constants(), Eigen::Vector3d(1, 2, 3));
  EXPECT_EQ(bushing.torque_damping_constants(), Eigen::Vector3d(4, 5, 6));
  EXPECT_EQ(bushing.force_stiffness_constants(), Eigen::Vector3d(7, 8, 9));
  EXPECT_EQ(bushing.force_damping_constants(), Eigen::Vector3d(10, 11, 12));

  const LinearBushingRollPitchYaw<double>& bushing2 =
      plant->GetForceElement<LinearBushingRollPitchYaw>(ForceElementIndex(2));

  EXPECT_STREQ(bushing2.frameA().name().c_str(), "frameA");
  EXPECT_STREQ(bushing2.frameC().name().c_str(), "frameC");
  EXPECT_EQ(bushing2.frameA().model_instance(), bushing2.model_instance());
  EXPECT_NE(bushing.model_instance(), bushing2.model_instance());

  // Test missing frame tag
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(R"(
    <robot name="bushing_test">
        <link name='A'/>
        <link name='C'/>
        <frame name="frameA" link="A" rpy="0 0 0" xyz="0 0 0"/>
        <frame name="frameC" link="C" rpy="0 0 0" xyz="0 0 0"/>
        <drake:linear_bushing_rpy>
            <drake:bushing_frameA name="frameA"/>
            <!-- missing the drake:bushing_frameC tag -->
            <drake:bushing_torque_stiffness value="1 2 3"/>
            <drake:bushing_torque_damping   value="4 5 6"/>
            <drake:bushing_force_stiffness  value="7 8 9"/>
            <drake:bushing_force_damping    value="10 11 12"/>
        </drake:linear_bushing_rpy>
    </robot>)"),
      ".*Unable to find the <drake:bushing_frameC> tag on line [0-9]+");

  // Test non-existent frame
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(R"(
    <robot name="bushing_test">
        <link name='A'/>
        <link name='C'/>
        <frame name="frameA" link="A" rpy="0 0 0" xyz="0 0 0"/>
        <frame name="frameC" link="C" rpy="0 0 0" xyz="0 0 0"/>
        <drake:linear_bushing_rpy>
            <drake:bushing_frameA name="frameA"/>
            <drake:bushing_frameC name="frameZ"/>
            <!-- frameZ does not exist in the model -->
            <drake:bushing_torque_stiffness value="1 2 3"/>
            <drake:bushing_torque_damping   value="4 5 6"/>
            <drake:bushing_force_stiffness  value="7 8 9"/>
            <drake:bushing_force_damping    value="10 11 12"/>
        </drake:linear_bushing_rpy>
    </robot>)"),
      ".*Frame: frameZ specified for <drake:bushing_frameC> does not exist in "
      "the model.");

  // Test missing constants tag
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(R"(
    <robot name="bushing_test">
        <link name='A'/>
        <link name='C'/>
        <frame name="frameA" link="A" rpy="0 0 0" xyz="0 0 0"/>
        <frame name="frameC" link="C" rpy="0 0 0" xyz="0 0 0"/>
        <drake:linear_bushing_rpy>
            <drake:bushing_frameA name="frameA"/>
            <drake:bushing_frameC name="frameC"/>
            <drake:bushing_torque_stiffness value="1 2 3"/>
            <!-- missing the drake:bushing_torque_damping tag -->
            <drake:bushing_force_stiffness  value="7 8 9"/>
            <drake:bushing_force_damping    value="10 11 12"/>
        </drake:linear_bushing_rpy>
    </robot>)"),
      ".*Unable to find the <drake:bushing_torque_damping> tag on line [0-9]+");

  // Test missing `value` attribute
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(R"(
    <robot name="bushing_test">
        <link name='A'/>
        <link name='C'/>
        <frame name="frameA" link="A" rpy="0 0 0" xyz="0 0 0"/>
        <frame name="frameC" link="C" rpy="0 0 0" xyz="0 0 0"/>
        <drake:linear_bushing_rpy>
            <drake:bushing_frameA name="frameA"/>
            <drake:bushing_frameC name="frameC"/>
            <!-- missing `value` attribute -->
            <drake:bushing_torque_stiffness />
            <drake:bushing_torque_damping   value="4 5 6"/>
            <drake:bushing_force_stiffness  value="7 8 9"/>
            <drake:bushing_force_damping    value="10 11 12"/>
        </drake:linear_bushing_rpy>
    </robot>)"),
      ".*Unable to read the 'value' attribute for the"
      " <drake:bushing_torque_stiffness> tag on line [0-9]+");
}

GTEST_TEST(MultibodyPlantUrdfParserTest, ReflectedInertiaParametersParsing) {
  // Common URDF string with format options for the two custom tags.
  const std::string test_string = R"""(
    <robot name='reflected_inertia_test'>
      <link name='A'/>
      <link name='B'/>
      <joint name='revolute_AB' type='revolute'>
        <axis xyz='0 0 1'/>
        <parent link='A'/>
        <child link='B'/>
        <origin rpy='0 0 0' xyz='0 0 0'/>
        <limit effort='100' lower='-1' upper='2' velocity='100'/>
        <dynamics damping='0.1'/>
      </joint>
      <transmission>
        <type>transmission_interface/SimpleTransmission</type>
        <joint name='revolute_AB'>
          <hardwareInterface>PositionJointInterface</hardwareInterface>
        </joint>
        <actuator name='revolute_AB'>
          {0}
          {1}
        </actuator>
      </transmission>
    </robot>)""";

  // Test successful parsing of both parameters.
  {
    auto [plant, scene_graph] = ParseTestString(
        fmt::format(test_string, "<drake:rotor_inertia value='1.5' />",
                    "<drake:gear_ratio value='300.0' />"));

    const JointActuator<double>& actuator =
        plant->GetJointActuatorByName("revolute_AB");

    EXPECT_EQ(actuator.default_rotor_inertia(), 1.5);
    EXPECT_EQ(actuator.default_gear_ratio(), 300.0);
  }

  // Test successful parsing of rotor_inertia and default value for
  // gear_ratio.
  {
    auto [plant, scene_graph] = ParseTestString(fmt::format(test_string,
        "<drake:rotor_inertia value='1.5' />",
        ""));

    const JointActuator<double>& actuator =
        plant->GetJointActuatorByName("revolute_AB");

    EXPECT_EQ(actuator.default_rotor_inertia(), 1.5);
    EXPECT_EQ(actuator.default_gear_ratio(), 1.0);
  }

  // Test successful parsing of gear_ratio and default value for
  // rotor_inertia.
  {
    auto [plant, scene_graph] = ParseTestString(
        fmt::format(test_string, "", "<drake:gear_ratio value='300.0' />"));

    const JointActuator<double>& actuator =
        plant->GetJointActuatorByName("revolute_AB");

    EXPECT_EQ(actuator.default_rotor_inertia(), 0.0);
    EXPECT_EQ(actuator.default_gear_ratio(), 300.0);
  }
}

// TODO(SeanCurtis-TRI) The logic testing for collision filter group parsing
// belongs in detail_common_test.cc. Urdf and Sdf parsing just need enough
// testing to indicate that the method is being invoked correctly.
TEST_F(UrdfParserTest, CollisionFilterGroupParsingTest) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "collision_filter_group_parsing_test.urdf");
  AddModelFromUrdfFile(full_name, "");

  // Get geometry ids for all the bodies.
  const geometry::SceneGraphInspector<double>& inspector =
      scene_graph_.model_inspector();
  static constexpr int kNumLinks = 6;
  std::vector<GeometryId> ids(1 + kNumLinks);  // allow 1-based indices.
  for (int k = 1; k <= 6; ++k) {
    const auto geometry_id = inspector.GetGeometryIdByName(
        plant_.GetBodyFrameIdOrThrow(
            plant_.GetBodyByName(fmt::format("link{}", k)).index()),
        geometry::Role::kProximity,
        fmt::format("collision_filter_group_parsing_test::link{}_sphere", k));
    ids[k] = geometry_id;
  }

  // Make sure the plant is not finalized such that the adjacent joint filter
  // has not taken into effect yet. This guarantees that the collision filtering
  // is applied due to the collision filter group parsing.
  ASSERT_FALSE(plant_.is_finalized());

  // We have six geometries and 15 possible pairs, each with a particular
  // disposition.
  // (1, 2) - unfiltered
  EXPECT_FALSE(inspector.CollisionFiltered(ids[1], ids[2]));
  // (1, 3) - filtered by group_link3 ignores group_link14
  EXPECT_TRUE(inspector.CollisionFiltered(ids[1], ids[3]));
  // (1, 4) - filtered by group_link14 ignores itself
  EXPECT_TRUE(inspector.CollisionFiltered(ids[1], ids[4]));
  // (1, 5) - unfiltered
  EXPECT_FALSE(inspector.CollisionFiltered(ids[1], ids[5]));
  // (1, 6) - unfiltered
  EXPECT_FALSE(inspector.CollisionFiltered(ids[1], ids[6]));
  // (2, 3) - filtered by group_link2 ignores group_link3
  EXPECT_TRUE(inspector.CollisionFiltered(ids[2], ids[3]));
  // (2, 4) - unfiltered (although declared in an *ignored* self-filtering
  // group_link24).
  EXPECT_FALSE(inspector.CollisionFiltered(ids[2], ids[4]));
  // (2, 5) - filtered by group_link56 ignored group_link2
  EXPECT_TRUE(inspector.CollisionFiltered(ids[2], ids[5]));
  // (2, 6) - filtered by group_link56 ignored group_link2
  EXPECT_TRUE(inspector.CollisionFiltered(ids[2], ids[6]));
  // (3, 4) - filtered by group_link3 ignores group_link14
  EXPECT_TRUE(inspector.CollisionFiltered(ids[3], ids[4]));
  // (3, 5) - filtered by group_link56 ignored group_link3
  EXPECT_TRUE(inspector.CollisionFiltered(ids[3], ids[5]));
  // (3, 6) - filtered by group_link56 ignored group_link3
  EXPECT_TRUE(inspector.CollisionFiltered(ids[3], ids[6]));
  // (4, 5) - unfiltered
  EXPECT_FALSE(inspector.CollisionFiltered(ids[4], ids[5]));
  // (4, 6) - unfiltered
  EXPECT_FALSE(inspector.CollisionFiltered(ids[4], ids[6]));
  // (5, 6) - filtered by group_link56 ignores itself
  EXPECT_TRUE(inspector.CollisionFiltered(ids[5], ids[6]));

  // Make sure we can add the model a second time.
  AddModelFromUrdfFile(full_name, "model2");
}

// TODO(marcoag) We might want to add some form of feedback for:
// - ignore_collision_filter_groups with non-existing group names.
// - Empty collision_filter_groups.
GTEST_TEST(MultibodyPlantUrdfParserTest,
           CollisionFilterGroupParsingErrorsTest) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(R"""(
<robot name='robot'>
  <link name='a'/>
  <drake:collision_filter_group>
  </drake:collision_filter_group>
</robot>)"""),
      ".*The tag <drake:collision_filter_group> does not specify the required "
      "attribute \"name\".*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(R"""(
<robot name='robot'>
  <link name='a'/>
  <drake:collision_filter_group name="group_a">
    <drake:member/>
  </drake:collision_filter_group>
</robot>)"""),
      ".*The tag <drake:member> does not specify the required "
      "attribute \"link\".*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(R"""(
<robot name='robot'>
  <link name='a'/>
  <drake:collision_filter_group name="group_a">
    <drake:ignored_collision_filter_group/>
  </drake:collision_filter_group>
</robot>)"""),
      ".*The tag <drake:ignored_collision_filter_group> does not specify the "
      "required attribute \"name\".*");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
