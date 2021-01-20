#include "drake/multibody/parsing/detail_sdf_parser.h"

#include <fstream>
#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>
#include <sdf/sdf.hh>

#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/universal_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::SceneGraph;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYaw;
using math::RollPitchYawd;
using systems::Context;

const double kEps = std::numeric_limits<double>::epsilon();

// TODO(jwnimmer-tri) This unit test has a lot of copy-pasta, including these
// helper functions as well as all their call sites below.  We should refactor
// the plant, scene_graph, etc. into a test fixture for brevity.
ModelInstanceIndex AddModelFromSdfFile(
    const std::string& file_name,
    const std::string& model_name,
    const PackageMap& package_map,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr) {
  return AddModelFromSdf(
      { .file_name = &file_name },
      model_name, package_map, plant, scene_graph);
}
std::vector<ModelInstanceIndex> AddModelsFromSdfFile(
    const std::string& file_name,
    const PackageMap& package_map,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr) {
  return AddModelsFromSdf(
      { .file_name = &file_name },
      package_map, plant, scene_graph);
}

// Verifies that the SDF loader can leverage a specified package map.
GTEST_TEST(MultibodyPlantSdfParserTest, PackageMapSpecified) {
  // We start with the world and default model instances (model_instance.h
  // explains why there are two).
  MultibodyPlant<double> plant(0.0);
  geometry::SceneGraph<double> scene_graph;
  ASSERT_EQ(plant.num_model_instances(), 2);

  const std::string full_sdf_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/sdfs/box.sdf");
  filesystem::path package_path = full_sdf_filename;
  package_path = package_path.parent_path();
  package_path = package_path.parent_path();

  // Construct the PackageMap.
  PackageMap package_map;
  package_map.PopulateFromFolder(package_path.string());

  // Read in the SDF file.
  AddModelFromSdfFile(full_sdf_filename, "", package_map, &plant, &scene_graph);
  plant.Finalize();

  // Verify the number of model instances.
  EXPECT_EQ(plant.num_model_instances(), 3);
}

// Acceptance test that libsdformat can upgrade very old files.  This ensures
// the upgrade machinery keeps working (in particular our re-implementation of
// the embedSdf.rb tool within tools/workspace/sdformat).
GTEST_TEST(MultibodyPlantSdfParserTest, VeryOldVersion) {
  MultibodyPlant<double> plant(0.0);
  PackageMap package_map;
  const std::string full_sdf_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/very_old_version.sdf");

  EXPECT_EQ(plant.num_model_instances(), 2);
  AddModelFromSdfFile(full_sdf_filename, "", package_map, &plant, nullptr);
  plant.Finalize();
  EXPECT_EQ(plant.num_model_instances(), 3);
}

// Verifies model instances are correctly created in the plant.
GTEST_TEST(MultibodyPlantSdfParserTest, ModelInstanceTest) {
  // We start with the world and default model instances (model_instance.h
  // explains why there are two).
  MultibodyPlant<double> plant(0.0);
  ASSERT_EQ(plant.num_model_instances(), 2);

  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
      "links_with_visuals_and_collisions.sdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);

  ModelInstanceIndex instance1 =
      AddModelFromSdfFile(full_name, "instance1", package_map, &plant);

  // Check that a duplicate model names are not allowed.
  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromSdfFile(full_name, "instance1", package_map, &plant),
      std::logic_error,
      "This model already contains a model instance named 'instance1'. "
      "Model instance names must be unique within a given model.");

  // Load two acrobots to check per-model-instance items.
  const std::string acrobot_sdf_name = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/acrobot.sdf");
  ModelInstanceIndex acrobot1 =
      AddModelFromSdfFile(acrobot_sdf_name, "", package_map, &plant);

  // Loading the model again without specifying a different model name should
  // throw.
  EXPECT_THROW(AddModelFromSdfFile(acrobot_sdf_name, "", package_map, &plant),
               std::logic_error);

  ModelInstanceIndex acrobot2 =
      AddModelFromSdfFile(acrobot_sdf_name, "acrobot2", package_map, &plant);

  // We are done adding models.
  plant.Finalize();

  ASSERT_EQ(plant.num_model_instances(), 5);
  EXPECT_EQ(plant.GetModelInstanceByName("instance1"), instance1);
  EXPECT_EQ(plant.GetModelInstanceByName("acrobot"), acrobot1);
  EXPECT_EQ(plant.GetModelInstanceByName("acrobot2"), acrobot2);

  // Check a couple links from the first model without specifying the model
  // instance.
  EXPECT_TRUE(plant.HasBodyNamed("link3"));
  EXPECT_FALSE(plant.HasBodyNamed("link_which_doesnt_exist"));

  // Links which appear in multiple model instances throw if the instance
  // isn't specified.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.HasBodyNamed("Link1"), std::logic_error,
      "Body Link1 appears in multiple model instances.");

  EXPECT_FALSE(plant.HasBodyNamed("Link1", instance1));
  EXPECT_TRUE(plant.HasBodyNamed("Link1", acrobot1));
  EXPECT_TRUE(plant.HasBodyNamed("Link1", acrobot2));

  const Body<double>& acrobot1_link1 =
      plant.GetBodyByName("Link1", acrobot1);
  const Body<double>& acrobot2_link1 =
      plant.GetBodyByName("Link1", acrobot2);
  EXPECT_NE(acrobot1_link1.index(), acrobot2_link1.index());
  EXPECT_EQ(acrobot1_link1.model_instance(), acrobot1);
  EXPECT_EQ(acrobot2_link1.model_instance(), acrobot2);

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetBodyByName("Link1"), std::logic_error,
      "Body Link1 appears in multiple model instances.");


  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.HasJointNamed("ShoulderJoint"), std::logic_error,
      "Joint ShoulderJoint appears in multiple model instances.");
  EXPECT_FALSE(plant.HasJointNamed("ShoulderJoint", instance1));
  EXPECT_TRUE(plant.HasJointNamed("ShoulderJoint", acrobot1));
  EXPECT_TRUE(plant.HasJointNamed("ShoulderJoint", acrobot2));

  const Joint<double>& acrobot1_joint =
      plant.GetJointByName("ShoulderJoint", acrobot1);
  const Joint<double>& acrobot2_joint =
      plant.GetJointByName("ShoulderJoint", acrobot2);
  EXPECT_NE(acrobot1_joint.index(), acrobot2_joint.index());
  EXPECT_EQ(acrobot1_joint.model_instance(), acrobot1);
  EXPECT_EQ(acrobot2_joint.model_instance(), acrobot2);

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetJointByName("ShoulderJoint"), std::logic_error,
      "Joint ShoulderJoint appears in multiple model instances.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.HasJointActuatorNamed("ElbowJoint"), std::logic_error,
      "Joint actuator ElbowJoint appears in multiple model instances.");

  const JointActuator<double>& acrobot1_actuator =
      plant.GetJointActuatorByName("ElbowJoint", acrobot1);
  const JointActuator<double>& acrobot2_actuator =
      plant.GetJointActuatorByName("ElbowJoint", acrobot2);
  EXPECT_NE(acrobot1_actuator.index(), acrobot2_actuator.index());

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetJointActuatorByName("ElbowJoint"), std::logic_error,
      "Joint actuator ElbowJoint appears in multiple model instances.");

  const Frame<double>& acrobot1_link1_frame =
      plant.GetFrameByName("Link1", acrobot1);
  const Frame<double>& acrobot2_link1_frame =
      plant.GetFrameByName("Link1", acrobot2);
  EXPECT_NE(acrobot1_link1_frame.index(), acrobot2_link1_frame.index());

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetFrameByName("Link1"), std::logic_error,
      "Frame Link1 appears in multiple model instances.");

  // Check model scope frames.
  auto context = plant.CreateDefaultContext();
  auto check_frame = [&plant, instance1, &context](
      std::string parent_name, std::string name,
      const RigidTransformd& X_PF_expected) {
    const Frame<double>& frame = plant.GetFrameByName(name, instance1);
    const Frame<double>& parent_frame =
        plant.GetFrameByName(parent_name, instance1);
    const RigidTransformd X_PF = plant.CalcRelativeTransform(
        *context, parent_frame, frame);
    EXPECT_TRUE(CompareMatrices(
        X_PF_expected.GetAsMatrix4(), X_PF.GetAsMatrix4(), kEps))
        << name;
  };

  const RigidTransformd X_L1F1(
      RollPitchYawd(0.4, 0.5, 0.6), Vector3d(0.1, 0.2, 0.3));
  check_frame("link1", "model_scope_link1_frame", X_L1F1);
  const RigidTransformd X_F1F2(Vector3d(0.1, 0.0, 0.0));
  check_frame(
      "model_scope_link1_frame", "model_scope_link1_frame_child", X_F1F2);
  const RigidTransformd X_MF3(Vector3d(0.7, 0.8, 0.9));
  check_frame(
      "__model__", "model_scope_model_frame_implicit", X_MF3);
}

struct PlantAndSceneGraph {
  std::unique_ptr<MultibodyPlant<double>> plant;
  std::unique_ptr<SceneGraph<double>> scene_graph;
};

PlantAndSceneGraph ParseTestString(const std::string& inner) {
  const std::string filename = temp_directory() + "/test_string.sdf";
  std::ofstream file(filename);
  file << "<sdf version='1.6'>" << inner << "\n</sdf>\n";
  file.close();
  PlantAndSceneGraph pair;
  pair.plant = std::make_unique<MultibodyPlant<double>>(0.0);
  pair.scene_graph = std::make_unique<SceneGraph<double>>();
  PackageMap package_map;
  pair.plant->RegisterAsSourceForSceneGraph(pair.scene_graph.get());
  drake::log()->debug("inner: {}", inner);
  AddModelsFromSdfFile(filename, package_map, pair.plant.get());
  return pair;
}

GTEST_TEST(SdfParser, EntireInertialTagOmitted) {
  // Test that parsing a link with no inertial tag yields the expected result
  // (mass = 1, ixx = iyy = izz = 1, ixy = ixz = iyz = 0).
  // TODO(avalenzu): Re-visit this if the SDF spec changes to allow for more
  // parsimonious specification of massless links. See #13903 for more details.
  PlantAndSceneGraph pair = ParseTestString(R"""(
<model name='entire_inertial_tag_omitted'>
  <link name='entire_inertial_tag_omitted'/>
</model>)""");
  const RigidBody<double>* body = dynamic_cast<const RigidBody<double>*>(
    &pair.plant->GetBodyByName("entire_inertial_tag_omitted"));
  EXPECT_EQ(body->get_default_mass(), 1.);
  EXPECT_TRUE(body->default_rotational_inertia().get_moments().isOnes());
  EXPECT_TRUE(body->default_rotational_inertia().get_products().isZero());
}

GTEST_TEST(SdfParser, InertiaTagOmitted) {
  // Test that parsing a link with no inertia tag yields the expected result
  // (mass as specified, ixx = iyy = izz = 1, ixy = ixz = iyz = 0).
  // TODO(avalenzu): Re-visit this if the SDF spec changes to allow for more
  // parsimonious specification of massless links. See #13903 for more details.
  PlantAndSceneGraph pair = ParseTestString(R"""(
<model name='inertia_tag_omitted'>
  <link name='inertia_tag_omitted'>
    <inertial>
      <mass>2</mass>
    </inertial>
  </link>
</model>)""");
  const RigidBody<double>* body = dynamic_cast<const RigidBody<double>*>(
    &pair.plant->GetBodyByName("inertia_tag_omitted"));
  EXPECT_EQ(body->get_default_mass(), 2.);
  EXPECT_TRUE(body->default_rotational_inertia().get_moments().isOnes());
  EXPECT_TRUE(body->default_rotational_inertia().get_products().isZero());
}

GTEST_TEST(SdfParser, MassTagOmitted) {
  // Test that parsing a link with no mass tag yields the expected result
  // (mass = 1, inertia as specified).
  // TODO(avalenzu): Re-visit this if the SDF spec changes to allow for more
  // parsimonious specification of massless links. See #13903 for more details.
  PlantAndSceneGraph pair = ParseTestString(R"""(
<model name='mass_tag_omitted'>
  <link name='mass_tag_omitted'>
    <inertial>
      <inertia>
        <ixx>1</ixx>
        <ixy>0.1</ixy>
        <ixz>0.1</ixz>
        <iyy>1</iyy>
        <iyz>0.1</iyz>
        <izz>1</izz>
      </inertia>
    </inertial>
  </link>
</model>)""");
  const RigidBody<double>* body = dynamic_cast<const RigidBody<double>*>(
    &pair.plant->GetBodyByName("mass_tag_omitted"));
  EXPECT_EQ(body->get_default_mass(), 1.);
  EXPECT_TRUE(body->default_rotational_inertia().get_moments().isOnes());
  EXPECT_EQ(body->default_rotational_inertia().get_products(),
            Vector3d::Constant(0.1));
}

GTEST_TEST(SdfParser, MasslessBody) {
  // Test that massless bodies can be parsed.
  PlantAndSceneGraph pair = ParseTestString(R"""(
<model name='has_massless_link'>
  <link name='massless_link'>
    <inertial>
      <mass>0</mass>
      <inertia>
        <ixx>0</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0</iyy>
        <iyz>0</iyz>
        <izz>0</izz>
      </inertia>
    </inertial>
  </link>
</model>)""");
  const RigidBody<double>* body = dynamic_cast<const RigidBody<double>*>(
    &pair.plant->GetBodyByName("massless_link"));
  EXPECT_EQ(body->get_default_mass(), 0.);
  EXPECT_TRUE(body->default_rotational_inertia().get_moments().isZero());
  EXPECT_TRUE(body->default_rotational_inertia().get_products().isZero());}

GTEST_TEST(SdfParser, PointMass) {
  // Test that point masses don't get sent through the massless body branch.
  PlantAndSceneGraph pair = ParseTestString(R"""(
<model name='point_mass'>
  <link name='point_mass'>
    <inertial>
      <mass>1</mass>
      <inertia>
        <ixx>0</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0</iyy>
        <iyz>0</iyz>
        <izz>0</izz>
      </inertia>
    </inertial>
  </link>
</model>)""");
  const RigidBody<double>* body = dynamic_cast<const RigidBody<double>*>(
    &pair.plant->GetBodyByName("point_mass"));
  EXPECT_EQ(body->get_default_mass(), 1.);
  EXPECT_TRUE(body->default_rotational_inertia().get_moments().isZero());
  EXPECT_TRUE(body->default_rotational_inertia().get_products().isZero());
}

namespace {
  void ParseZeroMassNonZeroInertia() {
    ParseTestString(R"""(
<model name='bad'>
  <link name='bad'>
    <inertial>
      <mass>0</mass>
      <inertia>
        <ixx>1</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>1</iyy>
        <iyz>0</iyz>
        <izz>1</izz>
      </inertia>
    </inertial>
  </link>
</model>)""");
  }
}  // namespace

GTEST_TEST(SdfParser, ZeroMassNonZeroInertia) {
  // Test that attempting to parse links with zero mass and non-zero inertia
  // fails.
  if (!::drake::kDrakeAssertIsArmed) {
    EXPECT_THROW(ParseZeroMassNonZeroInertia(), std::runtime_error);
  }
}

GTEST_TEST(SdfParserDeathTest, ZeroMassNonZeroInertia) {
  // Test that attempting to parse links with zero mass and non-zero inertia
  // fails.
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  if (::drake::kDrakeAssertIsArmed) {
    EXPECT_DEATH(ParseZeroMassNonZeroInertia(),
                 ".*condition 'mass > 0' failed");
  }
}

GTEST_TEST(SdfParser, FloatingBodyPose) {
  // Test that floating bodies (links) still have their poses preserved.
  PlantAndSceneGraph pair = ParseTestString(R"""(
<model name='good'>
  <link name='a'>
    <pose>1 2 3  0.1 0.2 0.3</pose>
  </link>
  <link name='b'>
    <pose>4 5 6  0.4 0.5 0.6</pose>
  </link>
</model>)""");
  pair.plant->Finalize();
  EXPECT_GT(pair.plant->num_positions(), 0);
  auto context = pair.plant->CreateDefaultContext();
  const RigidTransformd X_WA_expected(
      RollPitchYawd(0.1, 0.2, 0.3), Vector3d(1, 2, 3));
  const RigidTransformd X_WA =
      pair.plant->GetFrameByName("a").CalcPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(
      X_WA_expected.GetAsMatrix4(), X_WA.GetAsMatrix4(), kEps));
  const RigidTransformd X_WB_expected(
      RollPitchYawd(0.4, 0.5, 0.6), Vector3d(4, 5, 6));
  const RigidTransformd X_WB =
      pair.plant->GetFrameByName("b").CalcPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(
      X_WB_expected.GetAsMatrix4(), X_WB.GetAsMatrix4(), kEps));
}

GTEST_TEST(SdfParser, StaticModelSupported) {
  // Test that static models are partially supported.
  PlantAndSceneGraph pair = ParseTestString(R"""(
<model name='good'>
  <static>true</static>
  <link name='a'>
    <pose>1 2 3  0.1 0.2 0.3</pose>
  </link>
  <link name='b'>
    <pose>4 5 6  0.4 0.5 0.6</pose>
  </link>
</model>)""");
  pair.plant->Finalize();
  EXPECT_EQ(pair.plant->num_positions(), 0);
  auto context = pair.plant->CreateDefaultContext();
  const RigidTransformd X_WA_expected(
      RollPitchYawd(0.1, 0.2, 0.3), Vector3d(1, 2, 3));
  const RigidTransformd X_WA =
      pair.plant->GetFrameByName("a").CalcPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(
      X_WA_expected.GetAsMatrix4(), X_WA.GetAsMatrix4(), kEps));
  const RigidTransformd X_WB_expected(
      RollPitchYawd(0.4, 0.5, 0.6), Vector3d(4, 5, 6));
  const RigidTransformd X_WB =
      pair.plant->GetFrameByName("b").CalcPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(
      X_WB_expected.GetAsMatrix4(), X_WB.GetAsMatrix4(), kEps));
}

GTEST_TEST(SdfParser, StaticModelWithJoints) {
  // Specifying redundant welds in the model yields no errors, either to the
  // world or between links.
  DRAKE_EXPECT_NO_THROW(ParseTestString(R"""(
<model name='good'>
  <static>true</static>
  <link name='a'/>
  <joint name='a_weld' type='fixed'>
    <parent>world</parent>
    <child>a</child>
  </joint>
  <link name='b'/>
  <joint name='b_weld' type='fixed'>
    <parent>a</parent>
    <child>b</child>
  </joint>
</model>)"""));

  // Attempting to weld should fail fast, either during the welding or when
  // finalizing (due to loop).
  PlantAndSceneGraph pair = ParseTestString(R"""(
<model name='bad'>
  <static>true</static>
  <link name='a'/>
</model>
)""");
  auto weld_and_finalize = [&pair]() {
    pair.plant->WeldFrames(
        pair.plant->world_frame(), pair.plant->GetFrameByName("a"));
    pair.plant->Finalize();
  };
  // The message contains the elaborate joint name inserted by the parser.
  DRAKE_EXPECT_THROWS_MESSAGE(
      weld_and_finalize(), std::runtime_error,
      ".*sdformat_model_static.*");

  // Drake does not support "frozen" joints (#12227).
  DRAKE_EXPECT_THROWS_MESSAGE(
    ParseTestString(R"""(
<model name='good'>
  <static>true</static>
  <link name='a'/>
  <link name='b'/>
  <joint name='my_hinge' type='revolute'>
    <parent>a</parent>
    <child>b</child>
    <axis>
      <xyz>0 0 1</xyz>
    </axis>
  </joint>
</model>)"""),
    std::runtime_error,
    "Only fixed joints are permitted in static models.");
}

// Verify that our SDF parser throws an exception when a user specifies a joint
// with negative damping.
GTEST_TEST(SdfParserThrowsWhen, JointDampingIsNegative) {
  const std::string sdf_file_path = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "negative_damping_joint.sdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(sdf_file_path);
  MultibodyPlant<double> plant(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromSdfFile(sdf_file_path, "", package_map, &plant),
      std::runtime_error,
      /* Verify this method is throwing for the right reasons. */
      "Joint damping is negative for joint '.*'. "
          "Joint damping must be a non-negative number.");
}

GTEST_TEST(SdfParser, IncludeTags) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "include_models.sdf");
  sdf::addURIPath("model://", filesystem::path(full_name).parent_path());
  MultibodyPlant<double> plant(0.0);

  // We start with the world and default model instances.
  ASSERT_EQ(plant.num_model_instances(), 2);
  ASSERT_EQ(plant.num_bodies(), 1);
  ASSERT_EQ(plant.num_joints(), 0);

  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);
  AddModelsFromSdfFile(full_name, package_map, &plant);
  plant.Finalize();

  // We should have loaded 3 more models.
  EXPECT_EQ(plant.num_model_instances(), 5);
  // The models should have added 8 more bodies.
  EXPECT_EQ(plant.num_bodies(), 9);
  // The models should have added 5 more joints.
  EXPECT_EQ(plant.num_joints(), 5);

  // There should be a model instance with the name "robot1".
  EXPECT_TRUE(plant.HasModelInstanceNamed("robot1"));
  ModelInstanceIndex robot1_model = plant.GetModelInstanceByName("robot1");
  // There should be a body with the name "base_link".
  EXPECT_TRUE(plant.HasBodyNamed("base_link", robot1_model));
  // There should be another body with the name "moving_link".
  EXPECT_TRUE(plant.HasBodyNamed("moving_link", robot1_model));
  // There should be joint with the name "slider".
  EXPECT_TRUE(plant.HasJointNamed("slider", robot1_model));

  // There should be a model instance with the name "robot2".
  EXPECT_TRUE(plant.HasModelInstanceNamed("robot2"));
  ModelInstanceIndex robot2_model = plant.GetModelInstanceByName("robot2");

  // There should be a body with the name "base_link".
  EXPECT_TRUE(plant.HasBodyNamed("base_link", robot2_model));
  // There should be another body with the name "moving_link".
  EXPECT_TRUE(plant.HasBodyNamed("moving_link", robot2_model));
  // There should be joint with the name "slider".
  EXPECT_TRUE(plant.HasJointNamed("slider", robot2_model));

  // There should be a model instance with the name "weld_robots".
  EXPECT_TRUE(plant.HasModelInstanceNamed("weld_models"));
  ModelInstanceIndex weld_model = plant.GetModelInstanceByName("weld_models");

  // There should be all the bodies and joints contained in "simple_robot1"
  // prefixed with the model's name of "robot1".
  EXPECT_TRUE(plant.HasBodyNamed("robot1::base_link", weld_model));
  EXPECT_TRUE(plant.HasBodyNamed("robot1::moving_link", weld_model));
  EXPECT_TRUE(plant.HasJointNamed("robot1::slider", weld_model));
  // There should be all the bodies and joints contained in "simple_robot2"
  // prefixed with the model's name of "robot2".
  EXPECT_TRUE(plant.HasBodyNamed("robot2::base_link", weld_model));
  EXPECT_TRUE(plant.HasBodyNamed("robot2::moving_link", weld_model));
  EXPECT_TRUE(plant.HasJointNamed("robot2::slider", weld_model));
  // There should be a joint named "weld_robots"
  EXPECT_TRUE(plant.HasJointNamed("weld_robots", weld_model));
}

GTEST_TEST(SdfParser, TestOptionalSceneGraph) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
      "links_with_visuals_and_collisions.sdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);

  int num_visuals_explicit{};
  {
    // Test explicitly specifying `scene_graph`.
    MultibodyPlant<double> plant(0.0);
    SceneGraph<double> scene_graph;
    AddModelsFromSdfFile(full_name, package_map, &plant, &scene_graph);
    plant.Finalize();
    num_visuals_explicit = plant.num_visual_geometries();
  }
  EXPECT_NE(num_visuals_explicit, 0);
  {
    // Test implicitly specifying.
    MultibodyPlant<double> plant(0.0);
    SceneGraph<double> scene_graph;
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
    AddModelsFromSdfFile(full_name, package_map, &plant);
    plant.Finalize();
    EXPECT_EQ(plant.num_visual_geometries(), num_visuals_explicit);
  }
}

// Verifies that the SDF loader can leverage a specified package map.
GTEST_TEST(MultibodyPlantSdfParserTest, JointParsingTest) {
  MultibodyPlant<double> plant(0.0);
  geometry::SceneGraph<double> scene_graph;

  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "joint_parsing_test.sdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);

  // Read in the SDF file.
  AddModelFromSdfFile(full_name, "", package_map, &plant, &scene_graph);
  plant.Finalize();

  // Revolute joint
  DRAKE_EXPECT_NO_THROW(
      plant.GetJointByName<RevoluteJoint>("revolute_joint"));
  const RevoluteJoint<double>& revolute_joint =
      plant.GetJointByName<RevoluteJoint>("revolute_joint");
  EXPECT_EQ(revolute_joint.name(), "revolute_joint");
  EXPECT_EQ(revolute_joint.parent_body().name(), "link1");
  EXPECT_EQ(revolute_joint.child_body().name(), "link2");
  EXPECT_EQ(revolute_joint.revolute_axis(), Vector3d::UnitZ());
  EXPECT_EQ(revolute_joint.damping(), 0.2);
  EXPECT_TRUE(CompareMatrices(
      revolute_joint.position_lower_limits(), Vector1d(-1)));
  EXPECT_TRUE(CompareMatrices(
      revolute_joint.position_upper_limits(), Vector1d(2)));
  EXPECT_TRUE(CompareMatrices(
      revolute_joint.velocity_lower_limits(), Vector1d(-100)));
  EXPECT_TRUE(CompareMatrices(
      revolute_joint.velocity_upper_limits(), Vector1d(100)));

  // Prismatic joint
  DRAKE_EXPECT_NO_THROW(
      plant.GetJointByName<PrismaticJoint>("prismatic_joint"));
  const PrismaticJoint<double>& prismatic_joint =
      plant.GetJointByName<PrismaticJoint>("prismatic_joint");
  EXPECT_EQ(prismatic_joint.name(), "prismatic_joint");
  EXPECT_EQ(prismatic_joint.parent_body().name(), "link2");
  EXPECT_EQ(prismatic_joint.child_body().name(), "link3");
  EXPECT_EQ(prismatic_joint.translation_axis(), Vector3d::UnitZ());
  EXPECT_EQ(prismatic_joint.damping(), 0.3);
  EXPECT_TRUE(CompareMatrices(
      prismatic_joint.position_lower_limits(), Vector1d(-2)));
  EXPECT_TRUE(CompareMatrices(
      prismatic_joint.position_upper_limits(), Vector1d(1)));
  EXPECT_TRUE(CompareMatrices(
      prismatic_joint.velocity_lower_limits(), Vector1d(-5)));
  EXPECT_TRUE(CompareMatrices(
      prismatic_joint.velocity_upper_limits(), Vector1d(5)));

  // Limitless revolute joint
  DRAKE_EXPECT_NO_THROW(
      plant.GetJointByName<RevoluteJoint>("revolute_joint_no_limits"));
  const RevoluteJoint<double>& no_limit_joint =
      plant.GetJointByName<RevoluteJoint>("revolute_joint_no_limits");
  EXPECT_EQ(no_limit_joint.name(), "revolute_joint_no_limits");
  EXPECT_EQ(no_limit_joint.parent_body().name(), "link3");
  EXPECT_EQ(no_limit_joint.child_body().name(), "link4");
  EXPECT_EQ(no_limit_joint.revolute_axis(), Vector3d::UnitZ());
  const Vector1d inf(std::numeric_limits<double>::infinity());
  const Vector1d neg_inf(-std::numeric_limits<double>::infinity());
  EXPECT_TRUE(CompareMatrices(no_limit_joint.position_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.position_upper_limits(), inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.velocity_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.velocity_upper_limits(), inf));

  // Ball joint
  DRAKE_EXPECT_NO_THROW(plant.GetJointByName<BallRpyJoint>("ball_joint"));
  const BallRpyJoint<double>& ball_joint =
      plant.GetJointByName<BallRpyJoint>("ball_joint");
  EXPECT_EQ(ball_joint.name(), "ball_joint");
  EXPECT_EQ(ball_joint.parent_body().name(), "link4");
  EXPECT_EQ(ball_joint.child_body().name(), "link5");
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

  // Universal joint
  DRAKE_EXPECT_NO_THROW(
      plant.GetJointByName<UniversalJoint>("universal_joint"));
  const UniversalJoint<double>& universal_joint =
      plant.GetJointByName<UniversalJoint>("universal_joint");
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
  DRAKE_EXPECT_NO_THROW(plant.GetJointByName<PlanarJoint>("planar_joint"));
  const PlanarJoint<double>& planar_joint =
      plant.GetJointByName<PlanarJoint>("planar_joint");
  EXPECT_EQ(planar_joint.name(), "planar_joint");
  EXPECT_EQ(planar_joint.parent_body().name(), "link6");
  EXPECT_EQ(planar_joint.child_body().name(), "link7");
  EXPECT_TRUE(CompareMatrices(planar_joint.damping(), Vector3d::Constant(0.1)));
  EXPECT_TRUE(CompareMatrices(planar_joint.position_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint.position_upper_limits(), inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint.velocity_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint.velocity_upper_limits(), inf3));
}

// Verifies that the SDF parser parses the joint actuator limit correctly.
GTEST_TEST(MultibodyPlantSdfParserTest, JointActuatorParsingTest) {
  MultibodyPlant<double> plant(0.0);

  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "joint_actuator_parsing_test.sdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);

  // Read in the SDF file.
  AddModelFromSdfFile(full_name, "", package_map, &plant, nullptr);
  plant.Finalize();

  // In SDF, effort limits are specified in <joint><axis><limit><effort>,
  // which is the reason we read the joint actuator using the joint name.
  // Test the joint actuator with a positive effort limit.
  const auto& limited_joint_actuator =
      plant.GetJointActuatorByName("revolute_joint_positive_limit");
  EXPECT_EQ(limited_joint_actuator.effort_limit(), 100);

  // Test the joint actuator with the effort limit set to negative value,
  // which will be treated as no limit per the SDF standard.
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const auto& no_limit_joint_actuator =
      plant.GetJointActuatorByName("revolute_joint_no_limit");
  EXPECT_TRUE(no_limit_joint_actuator.effort_limit() == kInf);

  // Test the joint actuator with the effort limit set to 0, which means no
  // actuation.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetJointActuatorByName("prismatic_joint_zero_limit"),
      std::logic_error, "There is no joint actuator named '.*' in the model.");
}

// Verifies that the SDF parser parses the revolute spring parameters correctly.
GTEST_TEST(MultibodyPlantSdfParserTest, RevoluteSpringParsingTest) {
  MultibodyPlant<double> plant(0.0);

  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "revolute_spring_parsing_test.sdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);

  // Reads in the SDF file.
  AddModelFromSdfFile(full_name, "", package_map, &plant, nullptr);
  plant.Finalize();

  // Plant should have a UniformGravityFieldElement by default.
  // Our test contains two joints that have nonzero stiffness
  // and two joints that have zero stiffness. We only add a
  // spring for nonzero stiffness, so only two spring forces
  // should have been added.
  constexpr int kNumSpringForces = 2;
  DRAKE_DEMAND(plant.num_force_elements() == kNumSpringForces + 1);

  // In these two tests, we verify that the generalized forces are
  // correct for both springs. The first spring has a nonzero reference
  // of 1.0 radians so should have nonzero torque. The second spring
  // has a zero reference, so it should have no applied torque.
  MultibodyForces<double> forces(plant);
  auto context = plant.CreateDefaultContext();
  constexpr int kGeneralizedForcesSize = 10;
  Matrix2X<double> expected_generalized_forces(kNumSpringForces,
                                               kGeneralizedForcesSize);
  expected_generalized_forces << 0, 0, 0, 0, 0, 0, 5, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  for (int i = 0; i < kNumSpringForces; ++i) {
    // The ForceElement at index zero is gravity, so we skip that index.
    const ForceElementIndex force_index(i + 1);
    const auto& nonzero_reference = plant.GetForceElement(force_index);
    forces.SetZero();
    nonzero_reference.CalcAndAddForceContribution(
        *context, plant.EvalPositionKinematics(*context),
        plant.EvalVelocityKinematics(*context), &forces);

    const VectorX<double>& generalized_forces = forces.generalized_forces();
    EXPECT_TRUE(CompareMatrices(generalized_forces,
                                expected_generalized_forces.row(i).transpose(),
                                kEps, MatrixCompareType::relative));
  }
}

GTEST_TEST(SdfParser, TestSupportedFrames) {
  // Test `//link/pose[@relative_to]`.
  ParseTestString(R"(
<model name='good'>
  <frame name='my_frame'/>
  <link name='my_link'>
    <pose relative_to='my_frame'/>
  </link>
</model>
)");
  // Test `//link/visual/pose[@relative_to]`.
  ParseTestString(R"(
<model name='good'>
  <frame name='my_frame'/>
  <link name='my_link'>
    <visual name='my_visual'>
      <pose relative_to='my_frame'/>
    </visual>
  </link>
</model>
)");
  // Test `//link/collision/pose[@relative_to]`.
  ParseTestString(R"(
<model name='good'>
  <frame name='my_frame'/>
  <link name='my_link'>
    <collision name='my_collision'>
      <pose relative_to='my_frame'/>
    </collision>
  </link>
</model>
)");
  // Test `//joint/pose[@relative_to]`.
  ParseTestString(R"(
<model name='good'>
  <link name='a'/>
  <frame name='my_frame'/>
  <joint name='b' type='fixed'>"
    <pose relative_to='my_frame'/>"
    <parent>world</parent>
    <child>a</child>"
  </joint>
</model>)");
}

void FailWithUnsupportedRelativeTo(const std::string& inner) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(inner),
      std::runtime_error,
      R"(<pose relative_to='\{non-empty\}'/> is presently not supported )"
      R"(in <inertial/> or <model/> tags.)");
}

void FailWithInvalidWorld(const std::string& inner) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(inner),
      std::runtime_error,
      R"([\s\S]*(attached_to|relative_to) name\[world\] specified by frame )"
      R"(with name\[.*\] does not match a nested model, link, joint, or )"
      R"(frame name in model with name\[bad\][\s\S]*)");
}

void FailWithReservedName(const std::string& inner) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(inner),
      std::runtime_error,
      R"([\s\S]*The supplied frame name \[.*\] is reserved.[\s\S]*)");
}

GTEST_TEST(SdfParser, TestUnsupportedFrames) {
  // Model frames cannot attach to / nor be relative to the world frame.
  FailWithInvalidWorld(R"(
<model name='bad'>
  <link name='dont_crash_plz'/>  <!-- Need at least one link -->
  <frame name='model_scope_world_frame' attached_to='world'>
    <pose>0 0 0 0 0 0</pose>
  </frame>
</model>
)");
  FailWithInvalidWorld(R"(
<model name='bad'>
  <link name='dont_crash_plz'/>  <!-- Need at least one link -->
  <frame name='model_scope_world_relative_frame'>
    <pose relative_to='world'>0 0 0 0 0 0</pose>
  </frame>
</model>
)");
  for (std::string bad_name : {"world", "__model__", "__anything__"}) {
    FailWithReservedName(fmt::format(R"(
<model name='bad'>
  <link name='dont_crash_plz'/>  <!-- Need at least one link -->
  <frame name='{}'/>  <!-- Invalid name -->
</model>
)", bad_name));
  }

  FailWithUnsupportedRelativeTo(R"(
<model name='bad'>
  <pose relative_to='invalid_usage'/>
  <link name='dont_crash_plz'/>  <!-- Need at least one frame -->
</model>)");
  FailWithUnsupportedRelativeTo(R"(
<model name='bad'>
  <frame name='my_frame'/>
  <link name='a'>
    <inertial><pose relative_to='my_frame'/></inertial>
  </link>
</model>)");
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
    // that model instance  name is "test_robot".
    const geometry::GeometryId geometry_id =
        inspector.GetGeometryIdByName(frame_id, role, "test_robot::" + name);
    const std::string shape_type =
        geometry::ShapeName(inspector.GetShape(geometry_id)).name();
    if (shape_type != name) {
      return ::testing::AssertionFailure()
        << "Geometry with role " << role << " has wrong shape type."
        << "\n  Expected: " << name
        << "\n  Found: " << shape_type;
    }
  } catch (const std::exception& e) {
    return ::testing::AssertionFailure()
           << "Frame " << frame_id << " does not have a geometry with role "
           << role << " and name " << name
           << ". Exception message: " << e.what();
  }
  return ::testing::AssertionSuccess();
}

// Confirms that all supported geometries in an SDF file are registered. The
// *details* of the geometries are ignored -- we assume that that functionality
// is tested in detail_scene_graph_test.cc. This merely makes sure that *that*
// functionality is exercised appropriately.
void TestForParsedGeometry(const char* sdf_name, geometry::Role role) {
  const std::string full_name = FindResourceOrThrow(sdf_name);
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  AddModelsFromSdfFile(full_name, package_map, &plant);
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
  EXPECT_TRUE(
      FrameHasShape(frame_id, role, scene_graph, geometry::HalfSpace{}));
  EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph,
                            geometry::Mesh{mesh_uri, 1.0}));
  EXPECT_TRUE(
      FrameHasShape(frame_id, role, scene_graph, geometry::Sphere{0.1}));
}

GTEST_TEST(SdfParser, CollisionGeometryParsing) {
  TestForParsedGeometry(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "all_geometries_as_collision.sdf",
      geometry::Role::kProximity);
}

GTEST_TEST(SdfParser, VisualGeometryParsing) {
  TestForParsedGeometry(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "all_geometries_as_visual.sdf",
      geometry::Role::kPerception);
}

GTEST_TEST(SdfParser, BushingParsing) {
  // Test successful parsing
  auto [plant, scene_graph] = ParseTestString(R"(
    <model name='BushingModel'>
      <link name='A'/>
      <link name='C'/>
      <frame name='frameA' attached_to='A'/>
      <frame name='frameC' attached_to='C'/>
      <drake:linear_bushing_rpy>
        <drake:bushing_frameA>frameA</drake:bushing_frameA>
        <drake:bushing_frameC>frameC</drake:bushing_frameC>
        <drake:bushing_torque_stiffness>1 2 3</drake:bushing_torque_stiffness>
        <drake:bushing_torque_damping>4 5 6</drake:bushing_torque_damping>
        <drake:bushing_force_stiffness>7 8 9</drake:bushing_force_stiffness>
        <drake:bushing_force_damping>10 11 12</drake:bushing_force_damping>
      </drake:linear_bushing_rpy>
    </model>)");

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
  DRAKE_EXPECT_THROWS_MESSAGE(ParseTestString(R"(
    <model name='BushingModel'>
      <link name='A'/>
      <link name='C'/>
      <frame name='frameA' attached_to='A'/>
      <frame name='frameC' attached_to='C'/>
      <drake:linear_bushing_rpy>
        <drake:bushing_frameA>frameA</drake:bushing_frameA>
        <!-- missing the drake:bushing_frameC tag -->
        <drake:bushing_torque_stiffness>1 2 3</drake:bushing_torque_stiffness>
        <drake:bushing_torque_damping>4 5 6</drake:bushing_torque_damping>
        <drake:bushing_force_stiffness>7 8 9</drake:bushing_force_stiffness>
        <drake:bushing_force_damping>10 11 12</drake:bushing_force_damping>
      </drake:linear_bushing_rpy>
    </model>)"),
                              std::runtime_error,
                              "<drake:linear_bushing_rpy>: Unable to find the "
                              "<drake:bushing_frameC> child tag.");

  // Test non-existent frame
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(R"(
    <model name='BushingModel'>
      <link name='A'/>
      <link name='C'/>
      <frame name='frameA' attached_to='A'/>
      <frame name='frameC' attached_to='C'/>
      <drake:linear_bushing_rpy>
        <drake:bushing_frameA>frameA</drake:bushing_frameA>
        <drake:bushing_frameC>frameZ</drake:bushing_frameC>
        <!-- frameZ does not exist in the model -->
        <drake:bushing_torque_stiffness>1 2 3</drake:bushing_torque_stiffness>
        <drake:bushing_torque_damping>4 5 6</drake:bushing_torque_damping>
        <drake:bushing_force_stiffness>7 8 9</drake:bushing_force_stiffness>
        <drake:bushing_force_damping>10 11 12</drake:bushing_force_damping>
      </drake:linear_bushing_rpy>
    </model>)"),
      std::runtime_error,
      "<drake:linear_bushing_rpy>: Frame 'frameZ' specified for "
      "<drake:bushing_frameC> does not exist in "
      "the model.");

  // Test missing constants tag
  DRAKE_EXPECT_THROWS_MESSAGE(ParseTestString(R"(
    <model name='BushingModel'>
      <link name='A'/>
      <link name='C'/>
      <frame name='frameA' attached_to='A'/>
      <frame name='frameC' attached_to='C'/>
      <drake:linear_bushing_rpy>
        <drake:bushing_frameA>frameA</drake:bushing_frameA>
        <drake:bushing_frameC>frameC</drake:bushing_frameC>
        <drake:bushing_torque_stiffness>1 2 3</drake:bushing_torque_stiffness>
        <!-- missing the drake:bushing_torque_damping tag -->
        <drake:bushing_force_stiffness>7 8 9</drake:bushing_force_stiffness>
        <drake:bushing_force_damping>10 11 12</drake:bushing_force_damping>
      </drake:linear_bushing_rpy>
    </model>)"),
                              std::runtime_error,
                              "<drake:linear_bushing_rpy>: Unable to find the "
                              "<drake:bushing_torque_damping> child tag.");
}

GTEST_TEST(SdfParser, ReflectedInertiaParametersParsing) {
  // Common SDF string with format options for the two custom tags.
  const std::string test_string = R"""(
    <model name='ReflectedInertiaModel'>
      <link name='A'/>
      <link name='B'/>
      <joint name='revolute_AB' type='revolute'>
        <child>A</child>
        <parent>B</parent>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <effort>-1</effort>
          </limit>
        </axis>
        {0}
        {1}
      </joint>
    </model>)""";

  // Test successful parsing of both parameters.
  {
    auto [plant, scene_graph] = ParseTestString(fmt::format(test_string,
        "<drake:rotor_inertia>1.5</drake:rotor_inertia>",
        "<drake:gear_ratio>300.0</drake:gear_ratio>"));

    const JointActuator<double>& actuator =
        plant->GetJointActuatorByName("revolute_AB");

    EXPECT_EQ(actuator.default_rotor_inertia(), 1.5);
    EXPECT_EQ(actuator.default_gear_ratio(), 300.0);
  }

  // Test successful parsing of rotor_inertia and default value for
  // gear_ratio.
  {
    auto [plant, scene_graph] = ParseTestString(fmt::format(
        test_string, "<drake:rotor_inertia>1.5</drake:rotor_inertia>", ""));

    const JointActuator<double>& actuator =
        plant->GetJointActuatorByName("revolute_AB");

    EXPECT_EQ(actuator.default_rotor_inertia(), 1.5);
    EXPECT_EQ(actuator.default_gear_ratio(), 1.0);
  }

  // Test successful parsing of gear_ratio and default value for
  // rotor_inertia.
  {
    auto [plant, scene_graph] = ParseTestString(fmt::format(
        test_string, "", "<drake:gear_ratio>300.0</drake:gear_ratio>"));

    const JointActuator<double>& actuator =
        plant->GetJointActuatorByName("revolute_AB");

    EXPECT_EQ(actuator.default_rotor_inertia(), 0.0);
    EXPECT_EQ(actuator.default_gear_ratio(), 300.0);
  }
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
