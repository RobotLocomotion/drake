#include "drake/multibody/parsing/detail_urdf_parser.h"

#include <fstream>
#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Vector3d;
using geometry::GeometryId;
using geometry::SceneGraph;

// Verifies that the URDF loader can leverage a specified package map.
GTEST_TEST(MultibodyPlantUrdfParserTest, PackageMapSpecified) {
  // We start with the world and default model instances (model_instance.h
  // explains why there are two).
  MultibodyPlant<double> plant(0.0);
  geometry::SceneGraph<double> scene_graph;
  ASSERT_EQ(plant.num_model_instances(), 2);

  const std::string full_urdf_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/urdfs/box.urdf");
  filesystem::path package_path = full_urdf_filename;
  package_path = package_path.parent_path();
  package_path = package_path.parent_path();

  // Construct the PackageMap.
  PackageMap package_map;
  package_map.PopulateFromFolder(package_path.string());

  // Read in the URDF file.
  AddModelFromUrdfFile(full_urdf_filename, "", package_map, &plant,
                       &scene_graph);
  plant.Finalize();

  // Verify the number of model instances.
  EXPECT_EQ(plant.num_model_instances(), 3);
}

GTEST_TEST(MultibodyPlantUrdfParserTest, DoublePendulum) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  std::string full_name = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/double_pendulum.urdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);
  AddModelFromUrdfFile(full_name, "", package_map, &plant, &scene_graph);
  plant.Finalize();

  EXPECT_EQ(plant.num_bodies(), 4);
  EXPECT_EQ(plant.num_frames(), 10);

  ASSERT_TRUE(plant.HasFrameNamed("frame_on_link1"));
  ASSERT_TRUE(plant.HasFrameNamed("frame_on_link2"));
  ASSERT_TRUE(plant.HasFrameNamed("link1_com"));
  ASSERT_TRUE(plant.HasFrameNamed("link2_com"));

  // Sample a couple of frames.
  const Frame<double>& frame_on_link1 = plant.GetFrameByName("frame_on_link1");
  EXPECT_EQ(frame_on_link1.body().index(),
            plant.GetBodyByName("link1").index());

  math::RollPitchYaw<double> rpy_expected(-1, 0.1, 0.2);
  Vector3d xyz_expected(0.8, -0.2, 0.3);
  math::RigidTransform<double> X_BF_expected(rpy_expected.ToRotationMatrix(),
                                             xyz_expected);

  EXPECT_TRUE(CompareMatrices(frame_on_link1.GetFixedPoseInBodyFrame().matrix(),
                              X_BF_expected.GetAsMatrix4(), 1e-10));

  const Frame<double>& link2_com = plant.GetFrameByName("link2_com");
  EXPECT_EQ(link2_com.body().index(), plant.GetBodyByName("link2").index());
}

// This test verifies that we're able to successfully look up meshes using the
// "package://" syntax internally to the URDF (at least for packages which are
// successfully found in the same directory at the URDF.
GTEST_TEST(MultibodyPlantUrdfParserTest, TestAtlasMinimalContact) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  std::string full_name = FindResourceOrThrow(
      "drake/examples/atlas/urdf/atlas_minimal_contact.urdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);

  AddModelFromUrdfFile(full_name, "", package_map, &plant, &scene_graph);
  plant.Finalize();

  EXPECT_EQ(plant.num_positions(), 37);
  EXPECT_EQ(plant.num_velocities(), 36);

  // Verify that joint actuator limits are set correctly.
  ASSERT_TRUE(plant.HasJointActuatorNamed("back_bkz_motor"));
  const JointActuator<double>& actuator =
      plant.GetJointActuatorByName("back_bkz_motor");
  EXPECT_EQ(actuator.effort_limit(), 106);
}

GTEST_TEST(MultibodyPlantUrdfParserTest, TestAddWithQuaternionFloatingDof) {
  const std::string resource_dir{
      "drake/multibody/parsing/test/urdf_parser_test/"};
  const std::string model_file =
      FindResourceOrThrow(resource_dir + "zero_dof_robot.urdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(model_file);

  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  AddModelFromUrdfFile(model_file, "", package_map, &plant, &scene_graph);
  plant.Finalize();

  EXPECT_EQ(plant.num_positions(), 7);
  EXPECT_EQ(plant.num_velocities(), 6);
}

GTEST_TEST(MultibodyPlantUrdfParserTest, TestOptionalSceneGraph) {
  const std::string full_name = FindResourceOrThrow(
      "drake/examples/atlas/urdf/atlas_minimal_contact.urdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);
  int num_visuals_explicit{};
  {
    // Test explicitly specifying `scene_graph`.
    MultibodyPlant<double> plant(0.0);
    SceneGraph<double> scene_graph;
    AddModelFromUrdfFile(full_name, "", package_map, &plant, &scene_graph);
    plant.Finalize();
    num_visuals_explicit = plant.num_visual_geometries();
  }
  EXPECT_NE(num_visuals_explicit, 0);
  {
    // Test implicitly specifying.
    MultibodyPlant<double> plant(0.0);
    SceneGraph<double> scene_graph;
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
    AddModelFromUrdfFile(full_name, "", package_map, &plant);
    plant.Finalize();
    EXPECT_EQ(plant.num_visual_geometries(), num_visuals_explicit);
  }
}

GTEST_TEST(MultibodyPlantUrdfParserTest, JointParsingTest) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "joint_parsing_test.urdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);

  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  AddModelFromUrdfFile(full_name, "", package_map, &plant, &scene_graph);
  plant.Finalize();

  const Joint<double>& revolute_joint = plant.GetJointByName("revolute_joint");
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.position_lower_limits(), Vector1d(-1)));
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.position_upper_limits(), Vector1d(2)));
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.velocity_lower_limits(), Vector1d(-100)));
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.velocity_upper_limits(), Vector1d(100)));

  const JointActuator<double>& revolute_actuator =
      plant.GetJointActuatorByName("revolute_actuator");
  EXPECT_EQ(revolute_actuator.effort_limit(), 100);

  const Joint<double>& prismatic_joint =
      plant.GetJointByName("prismatic_joint");
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.position_lower_limits(), Vector1d(-2)));
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.position_upper_limits(), Vector1d(1)));
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.velocity_lower_limits(), Vector1d(-5)));
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.velocity_upper_limits(), Vector1d(5)));
  EXPECT_FALSE(plant.HasJointActuatorNamed("prismatic_actuator"));

  const Joint<double>& no_limit_joint =
      plant.GetJointByName("revolute_joint_no_limits");
  const Vector1d inf(std::numeric_limits<double>::infinity());
  const Vector1d neg_inf(-std::numeric_limits<double>::infinity());

  EXPECT_TRUE(CompareMatrices(no_limit_joint.position_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.position_upper_limits(), inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.velocity_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.velocity_upper_limits(), inf));

  const JointActuator<double>& revolute_actuator_no_limits =
      plant.GetJointActuatorByName("revolute_actuator_no_limits");
  EXPECT_EQ(revolute_actuator_no_limits.effort_limit(), inf(0));
}

GTEST_TEST(MultibodyPlantUrdfParserTest, CollisionFilterGroupParsingTest) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "collision_filter_group_parsing_test.urdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);

  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  AddModelFromUrdfFile(full_name, "", package_map, &plant, &scene_graph);

  // Get geometry ids for all the bodies.
  const geometry::SceneGraphInspector<double>& inspector =
      scene_graph.model_inspector();
  const auto geometry_id_link1 = inspector.GetGeometryIdByName(
      plant.GetBodyFrameIdOrThrow(plant.GetBodyByName("link1").index()),
      geometry::Role::kProximity,
      "collision_filter_group_parsing_test::link1_sphere");
  const auto geometry_id_link2 = inspector.GetGeometryIdByName(
      plant.GetBodyFrameIdOrThrow(plant.GetBodyByName("link2").index()),
      geometry::Role::kProximity,
      "collision_filter_group_parsing_test::link2_sphere");
  const auto geometry_id_link3 = inspector.GetGeometryIdByName(
      plant.GetBodyFrameIdOrThrow(plant.GetBodyByName("link3").index()),
      geometry::Role::kProximity,
      "collision_filter_group_parsing_test::link3_sphere");
  const auto geometry_id_link4 = inspector.GetGeometryIdByName(
      plant.GetBodyFrameIdOrThrow(plant.GetBodyByName("link4").index()),
      geometry::Role::kProximity,
      "collision_filter_group_parsing_test::link4_sphere");

  // Make sure the plant is not finalized such that the adjacent joint filter
  // has not taken into effect yet. This guarantees that the collision filtering
  // is applied due to the collision filter group parsing.
  ASSERT_FALSE(plant.is_finalized());

  // We have four geometries and six possible pairs, each with a particular
  // disposition.
  // (1, 2) - unfiltered
  // (1, 3) - filtered by group_link_3 ignores group_link_14
  // (1, 4) - filtered by group_link_14 ignores itself
  // (2, 3) - filtered by group_link_2 ignores group_link_3
  // (2, 4) - unfiltered (although declared in an *ignored* self-filtering
  // group_link_24).
  // (3, 4) - filtered by group_link_3 ignores group_link_14
  EXPECT_FALSE(
      inspector.CollisionFiltered(geometry_id_link1, geometry_id_link2));
  EXPECT_TRUE(
      inspector.CollisionFiltered(geometry_id_link1, geometry_id_link3));
  EXPECT_TRUE(
      inspector.CollisionFiltered(geometry_id_link1, geometry_id_link4));
  EXPECT_TRUE(
      inspector.CollisionFiltered(geometry_id_link2, geometry_id_link3));
  EXPECT_FALSE(
      inspector.CollisionFiltered(geometry_id_link2, geometry_id_link4));
  EXPECT_TRUE(
      inspector.CollisionFiltered(geometry_id_link3, geometry_id_link4));

  // Make sure we can add the model a second time.
  AddModelFromUrdfFile(full_name, "model2", package_map, &plant, &scene_graph);
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

// Confirms that all supported geometries in an URDF file are registered. The
// *details* of the geometries are ignored -- we assume that that functionality
// is tested in detail_urdf_geometry_test.cc. This merely makes sure that *that*
// functionality is exercised appropriately.
void TestForParsedGeometry(const char* sdf_name, geometry::Role role) {
  const std::string full_name = FindResourceOrThrow(sdf_name);
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  AddModelFromUrdfFile(full_name, "", package_map, &plant, &scene_graph);
  plant.Finalize();

  const auto frame_id =
      plant.GetBodyFrameIdOrThrow(plant.GetBodyByName("link1").index());

  const std::string mesh_uri = "drake/multibody/parsing/test/tri_cube.obj";

  // Note: the parameters for the various example shapes do not matter to this
  // test.
  EXPECT_TRUE(
      FrameHasShape(frame_id, role, scene_graph, geometry::Box{0.1, 0.1, 0.1}));
  EXPECT_TRUE(
      FrameHasShape(frame_id, role, scene_graph, geometry::Capsule{0.1, 0.1}));
  EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph,
                            geometry::Convex{mesh_uri, 1.0}));
  EXPECT_TRUE(
      FrameHasShape(frame_id, role, scene_graph, geometry::Cylinder{0.1, 0.1}));
  EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph,
                            geometry::Ellipsoid{0.1, 0.1, 0.1}));
  EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph,
                            geometry::Mesh{mesh_uri, 1.0}));
  EXPECT_TRUE(
      FrameHasShape(frame_id, role, scene_graph, geometry::Sphere{0.1}));
}

GTEST_TEST(MultibodyPlantUrdfParserTest, CollisionGeometryParsing) {
  TestForParsedGeometry(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "all_geometries_as_collision.urdf",
      geometry::Role::kProximity);
}

GTEST_TEST(MultibodyPlantUrdfParserTest, VisualGeometryParsing) {
  TestForParsedGeometry(
      "drake/multibody/parsing/test/urdf_parser_test/"
      "all_geometries_as_visual.urdf",
      geometry::Role::kPerception);
}

struct PlantAndSceneGraph {
  std::unique_ptr<MultibodyPlant<double>> plant;
  std::unique_ptr<SceneGraph<double>> scene_graph;
};

PlantAndSceneGraph ParseTestString(const std::string& inner) {
  const std::string filename = temp_directory() + "/test_string.urdf";
  std::ofstream file(filename);
  file << "<?xml version='1.0' ?>\n" << inner << "\n\n";
  file.close();
  PlantAndSceneGraph pair;
  pair.plant = std::make_unique<MultibodyPlant<double>>(0.0);
  pair.scene_graph = std::make_unique<SceneGraph<double>>();
  PackageMap package_map;
  pair.plant->RegisterAsSourceForSceneGraph(pair.scene_graph.get());
  drake::log()->debug("inner: {}", inner);
  AddModelFromUrdfFile(filename, {}, package_map, pair.plant.get());
  return pair;
}

GTEST_TEST(MultibodyPlantUrdfParserTest, BushingParsing) {
  // Test successful parsing
  auto [plant, scene_graph] = ParseTestString(R"(
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
    </robot>)");

  // MBP will always create a UniformGravityField, so the only other
  // ForceElement should be the LinearBushingRollPitchYaw element parsed.
  EXPECT_EQ(plant->num_force_elements(), 2);

  const LinearBushingRollPitchYaw<double>& bushing =
      plant->GetForceElement<LinearBushingRollPitchYaw>(ForceElementIndex(1));

  EXPECT_STREQ(bushing.frameA().name().c_str(), "frameA");
  EXPECT_STREQ(bushing.frameC().name().c_str(), "frameC");
  EXPECT_EQ(bushing.torque_stiffness_constants(), Eigen::Vector3d(1, 2, 3));
  EXPECT_EQ(bushing.torque_damping_constants(), Eigen::Vector3d(4, 5, 6));
  EXPECT_EQ(bushing.force_stiffness_constants(), Eigen::Vector3d(7, 8, 9));
  EXPECT_EQ(bushing.force_damping_constants(), Eigen::Vector3d(10, 11, 12));

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
      std::runtime_error,
      "Unable to find the <drake:bushing_frameC> tag on line [0-9]+");

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
      std::runtime_error,
      "Frame: frameZ specified for <drake:bushing_frameC> does not exist in "
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
      std::runtime_error,
      "Unable to find the <drake:bushing_torque_damping> tag on line [0-9]+");

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
      std::runtime_error,
      "Unable to read the 'value' attribute for the"
      " <drake:bushing_torque_stiffness> tag on line [0-9]+");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
