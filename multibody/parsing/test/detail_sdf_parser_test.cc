#include "drake/multibody/parsing/detail_sdf_parser.h"

#include <memory>
#include <regex>
#include <stdexcept>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sdf/sdf.hh>

#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/scope_exit.h"
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
    geometry::SceneGraph<double>* scene_graph = nullptr,
    bool test_sdf_forced_nesting = false) {
  const DataSource data_source{&file_name, {}};
  return AddModelFromSdf(data_source, model_name, package_map, plant,
                         scene_graph, test_sdf_forced_nesting);
}
std::vector<ModelInstanceIndex> AddModelsFromSdfFile(
    const std::string& file_name,
    const PackageMap& package_map,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr,
    bool test_sdf_forced_nesting = false) {
  const DataSource data_source{&file_name, {}};
  return AddModelsFromSdf(
      data_source, package_map, plant, scene_graph, test_sdf_forced_nesting);
}
std::vector<ModelInstanceIndex> AddModelsFromSdfString(
    const std::string& file_contents,
    const PackageMap& package_map,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr,
    bool test_sdf_forced_nesting = false) {
  const DataSource data_source{{}, &file_contents};
  return AddModelsFromSdf(
      data_source, package_map, plant, scene_graph, test_sdf_forced_nesting);
}

const Frame<double>& GetModelFrameByName(const MultibodyPlant<double>& plant,
                                         const std::string& name) {
  const auto model_instance = plant.GetModelInstanceByName(name);
  return plant.GetFrameByName("__model__", model_instance);
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
      ".*Body.*Link1.*multiple model instances.*");

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
      ".*Body.*Link1.*multiple model instances.*");


  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.HasJointNamed("ShoulderJoint"), std::logic_error,
      ".*Joint.*ShoulderJoint.*multiple model instances.*");
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
      ".*Joint.*ShoulderJoint.*multiple model instances.*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.HasJointActuatorNamed("ElbowJoint"), std::logic_error,
      ".*JointActuator.*ElbowJoint.*multiple model instances.*");

  const JointActuator<double>& acrobot1_actuator =
      plant.GetJointActuatorByName("ElbowJoint", acrobot1);
  const JointActuator<double>& acrobot2_actuator =
      plant.GetJointActuatorByName("ElbowJoint", acrobot2);
  EXPECT_NE(acrobot1_actuator.index(), acrobot2_actuator.index());

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetJointActuatorByName("ElbowJoint"), std::logic_error,
      ".*JointActuator.*ElbowJoint.*multiple model instances.*");

  const Frame<double>& acrobot1_link1_frame =
      plant.GetFrameByName("Link1", acrobot1);
  const Frame<double>& acrobot2_link1_frame =
      plant.GetFrameByName("Link1", acrobot2);
  EXPECT_NE(acrobot1_link1_frame.index(), acrobot2_link1_frame.index());

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetFrameByName("Link1"), std::logic_error,
      ".*Frame.*Link1.*multiple model instances.*");

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

PlantAndSceneGraph ParseTestString(const std::string& inner,
                                   const std::string& sdf_version = "1.6") {
  const std::string file_contents =
      "<sdf version='" + sdf_version + "'>" + inner + "\n</sdf>\n";
  PlantAndSceneGraph pair;
  pair.plant = std::make_unique<MultibodyPlant<double>>(0.0);
  pair.scene_graph = std::make_unique<SceneGraph<double>>();
  PackageMap package_map;
  pair.plant->RegisterAsSourceForSceneGraph(pair.scene_graph.get());
  drake::log()->debug("inner: {}", inner);
  AddModelsFromSdfString(file_contents, package_map, pair.plant.get());
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
  // Test that attempt to parse links with zero mass and non-zero inertia fails.
  if (!::drake::kDrakeAssertIsArmed) {
    EXPECT_THROW(ParseZeroMassNonZeroInertia(), std::exception);
  }
}

GTEST_TEST(SdfParserDeathTest, ZeroMassNonZeroInertia) {
  // Test that attempt to parse links with zero mass and non-zero inertia fails.
  const std::string expected_message =
      "RotationalInertia::SetFromRotationalInertia\\(\\):"
      " Division by zero mass or negative mass.";
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseZeroMassNonZeroInertia(), expected_message);
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
  {
    SCOPED_TRACE("Test that static models are partially supported");
    auto [plant, scene_graph] = ParseTestString(R"""(
  <model name='good'>
    <static>true</static>
    <link name='a'>
      <pose>1 2 3  0.1 0.2 0.3</pose>
    </link>
    <link name='b'>
      <pose>4 5 6  0.4 0.5 0.6</pose>
    </link>
  </model>)""");
    plant->Finalize();
    EXPECT_EQ(plant->num_positions(), 0);
    auto context = plant->CreateDefaultContext();
    const RigidTransformd X_WA_expected(
        RollPitchYawd(0.1, 0.2, 0.3), Vector3d(1, 2, 3));
    const RigidTransformd X_WA =
        plant->GetFrameByName("a").CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(
          X_WA_expected.GetAsMatrix4(), X_WA.GetAsMatrix4(), kEps));
    const RigidTransformd X_WB_expected(
        RollPitchYawd(0.4, 0.5, 0.6), Vector3d(4, 5, 6));
    const RigidTransformd X_WB =
        plant->GetFrameByName("b").CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(
          X_WB_expected.GetAsMatrix4(), X_WB.GetAsMatrix4(), kEps));
  }

  {
    SCOPED_TRACE(
        "Verify that static models don't need to have a canonical link");
    auto [plant, scene_graph]= ParseTestString(R"""(
  <model name='a'>
    <pose>1 2 3  0.1 0.2 0.3</pose>
    <static>true</static>
  </model>)""", "1.8");
    plant->Finalize();
    auto context = plant->CreateDefaultContext();
    const RigidTransformd X_WA_expected(
        RollPitchYawd(0.1, 0.2, 0.3), Vector3d(1, 2, 3));

    const auto& frame_A = GetModelFrameByName(*plant, "a");
    const RigidTransformd X_WA = frame_A.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(
          X_WA_expected.GetAsMatrix4(), X_WA.GetAsMatrix4(), kEps));
    EXPECT_EQ(frame_A.body().index(), plant->world_body().index());
  }

  {
    // Verify that models that contain static models don't need a link
    SCOPED_TRACE(
        "Verify that models that contain static models don't need a link");
    auto [plant, scene_graph] = ParseTestString(R"""(
  <model name='a'>
    <pose>1 2 3  0.0 0.0 0.3</pose>
    <model name='b'>
      <pose>0 0 0  0.1 0.2 0.0</pose>
      <static>true</static>
    </model>
  </model>)""", "1.8");
    plant->Finalize();
    auto context = plant->CreateDefaultContext();
    const RigidTransformd X_WA_expected(
        RollPitchYawd(0.0, 0.0, 0.3), Vector3d(1, 2, 3));

    const auto& frame_A = GetModelFrameByName(*plant, "a");
    const RigidTransformd X_WA = frame_A.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(
          X_WA_expected.GetAsMatrix4(), X_WA.GetAsMatrix4(), kEps));
    EXPECT_EQ(frame_A.body().index(), plant->world_body().index());

    const RigidTransformd X_WB_expected(
        RollPitchYawd(0.1, 0.2, 0.3), Vector3d(1, 2, 3));

    const auto &frame_B = GetModelFrameByName(*plant, "a::b");
    const RigidTransformd X_WB = frame_B.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(
          X_WB_expected.GetAsMatrix4(), X_WB.GetAsMatrix4(), kEps));
    EXPECT_EQ(frame_B.body().index(), plant->world_body().index());
  }
}

GTEST_TEST(SdfParser, StaticFrameOnlyModelsSupported) {
  // Verify that static models can contain just frames
  auto [plant, scene_graph] = ParseTestString(R"""(
  <model name='a'>
    <static>true</static>
    <pose>1 0 0  0 0 0</pose>
    <frame name='b'>
      <pose>0 2 0 0 0 0</pose>
    </frame>
    <frame name='c' attached_to='b'>
      <pose>0 0 3 0 0 0</pose>
    </frame>
    <frame name='d'>
      <pose relative_to='c'>0 0 0 0 0 0.3</pose>
    </frame>
  </model>)""", "1.8");
  plant->Finalize();
  auto context = plant->CreateDefaultContext();

  auto test_frame = [&, &plant = plant](const std::string& frame_name,
                                        const RigidTransformd& X_WF_expected) {
    const auto& frame = plant->GetFrameByName(frame_name);
    const RigidTransformd X_WF = frame.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WF_expected.GetAsMatrix4(),
                                X_WF.GetAsMatrix4(), kEps));
    EXPECT_EQ(frame.body().index(), plant->world_body().index());
  };

  test_frame("__model__", {RollPitchYawd(0.0, 0.0, 0.0), Vector3d(1, 0, 0)});
  test_frame("b", {RollPitchYawd(0.0, 0.0, 0.0), Vector3d(1, 2, 0)});
  test_frame("c", {RollPitchYawd(0.0, 0.0, 0.0), Vector3d(1, 2, 3)});
  test_frame("d", {RollPitchYawd(0.0, 0.0, 0.3), Vector3d(1, 2, 3)});
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

  MultibodyPlant<double> plant(0.0);

  // We start with the world and default model instances.
  ASSERT_EQ(plant.num_model_instances(), 2);
  ASSERT_EQ(plant.num_bodies(), 1);
  ASSERT_EQ(plant.num_joints(), 0);

  PackageMap package_map;
  package_map.PopulateFromFolder(filesystem::path(full_name).parent_path());
  AddModelsFromSdfFile(full_name, package_map, &plant);
  plant.Finalize();

  // We should have loaded 5 more models.
  EXPECT_EQ(plant.num_model_instances(), 7);
  // The models should have added 8 more bodies.
  EXPECT_EQ(plant.num_bodies(), 9);
  // The models should have added 5 more joints.
  EXPECT_EQ(plant.num_joints(), 5);

  // There should be a model instance with the name "robot1".
  ASSERT_TRUE(plant.HasModelInstanceNamed("robot1"));
  ModelInstanceIndex robot1_model = plant.GetModelInstanceByName("robot1");
  // There should be a body with the name "base_link".
  EXPECT_TRUE(plant.HasBodyNamed("base_link", robot1_model));
  // There should be another body with the name "moving_link".
  EXPECT_TRUE(plant.HasBodyNamed("moving_link", robot1_model));
  // There should be joint with the name "slider".
  EXPECT_TRUE(plant.HasJointNamed("slider", robot1_model));

  // There should be a model instance with the name "robot2".
  ASSERT_TRUE(plant.HasModelInstanceNamed("robot2"));
  ModelInstanceIndex robot2_model = plant.GetModelInstanceByName("robot2");

  // There should be a body with the name "base_link".
  EXPECT_TRUE(plant.HasBodyNamed("base_link", robot2_model));
  // There should be another body with the name "moving_link".
  EXPECT_TRUE(plant.HasBodyNamed("moving_link", robot2_model));
  // There should be joint with the name "slider".
  EXPECT_TRUE(plant.HasJointNamed("slider", robot2_model));

  // There should be a model instance with the name "weld_robots".
  EXPECT_TRUE(plant.HasModelInstanceNamed("weld_models"));

  ASSERT_TRUE(plant.HasModelInstanceNamed("weld_models::robot1"));
  ModelInstanceIndex weld_model_robot1_model =
      plant.GetModelInstanceByName("weld_models::robot1");

  ASSERT_TRUE(plant.HasModelInstanceNamed("weld_models::robot2"));
  ModelInstanceIndex weld_model_robot2_model =
      plant.GetModelInstanceByName("weld_models::robot2");

  // There should be all the bodies and joints contained in "simple_robot1"
  // which is inside "weld_models"
  EXPECT_TRUE(plant.HasBodyNamed("base_link", weld_model_robot1_model));
  EXPECT_TRUE(plant.HasBodyNamed("moving_link", weld_model_robot1_model));
  EXPECT_TRUE(plant.HasJointNamed("slider", weld_model_robot1_model));
  // There should be all the bodies and joints contained in "simple_robot2"
  // which is inside "weld_models"
  EXPECT_TRUE(plant.HasBodyNamed("base_link", weld_model_robot2_model));
  EXPECT_TRUE(plant.HasBodyNamed("moving_link", weld_model_robot2_model));
  EXPECT_TRUE(plant.HasJointNamed("slider", weld_model_robot2_model));
  // There should be a joint named "weld_robots". By convention, the joint
  // will have the same model instance as the child frame.
  EXPECT_TRUE(plant.HasJointNamed("weld_robots", weld_model_robot2_model));
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
  const std::vector<ModelInstanceIndex> instances =
      AddModelsFromSdfFile(full_name, package_map, &plant, &scene_graph);
  const ModelInstanceIndex instance1 = instances.front();
  plant.Finalize();

  // Revolute joint
  DRAKE_EXPECT_NO_THROW(
      plant.GetJointByName<RevoluteJoint>("revolute_joint", instance1));
  const RevoluteJoint<double>& revolute_joint =
      plant.GetJointByName<RevoluteJoint>("revolute_joint", instance1);
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
  EXPECT_TRUE(CompareMatrices(
      revolute_joint.acceleration_lower_limits(), Vector1d(-200)));
  EXPECT_TRUE(CompareMatrices(
      revolute_joint.acceleration_upper_limits(), Vector1d(200)));

  // Prismatic joint
  DRAKE_EXPECT_NO_THROW(
      plant.GetJointByName<PrismaticJoint>("prismatic_joint", instance1));
  const PrismaticJoint<double>& prismatic_joint =
      plant.GetJointByName<PrismaticJoint>("prismatic_joint", instance1);
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
  EXPECT_TRUE(CompareMatrices(
      prismatic_joint.acceleration_lower_limits(), Vector1d(-10)));
  EXPECT_TRUE(CompareMatrices(
      prismatic_joint.acceleration_upper_limits(), Vector1d(10)));

  // Limitless revolute joint
  DRAKE_EXPECT_NO_THROW(
      plant.GetJointByName<RevoluteJoint>("revolute_joint_no_limits",
                                          instance1));
  const RevoluteJoint<double>& no_limit_joint =
      plant.GetJointByName<RevoluteJoint>("revolute_joint_no_limits",
                                          instance1);
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
  EXPECT_TRUE(CompareMatrices(
      no_limit_joint.acceleration_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.acceleration_upper_limits(), inf));

  // Ball joint
  DRAKE_EXPECT_NO_THROW(plant.GetJointByName<BallRpyJoint>("ball_joint",
                                                           instance1));
  const BallRpyJoint<double>& ball_joint =
      plant.GetJointByName<BallRpyJoint>("ball_joint", instance1);
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
      plant.GetJointByName<UniversalJoint>("universal_joint", instance1));
  const UniversalJoint<double>& universal_joint =
      plant.GetJointByName<UniversalJoint>("universal_joint", instance1);
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
  DRAKE_EXPECT_NO_THROW(plant.GetJointByName<PlanarJoint>("planar_joint",
                                                          instance1));
  const PlanarJoint<double>& planar_joint =
      plant.GetJointByName<PlanarJoint>("planar_joint", instance1);
  EXPECT_EQ(planar_joint.name(), "planar_joint");
  EXPECT_EQ(planar_joint.parent_body().name(), "link6");
  EXPECT_EQ(planar_joint.parent_body().model_instance(), instance1);
  EXPECT_EQ(planar_joint.child_body().name(), "link7");
  EXPECT_EQ(planar_joint.child_body().model_instance(), instance1);
  EXPECT_TRUE(CompareMatrices(planar_joint.damping(), Vector3d::Constant(0.1)));
  EXPECT_TRUE(CompareMatrices(planar_joint.position_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint.position_upper_limits(), inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint.velocity_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint.velocity_upper_limits(), inf3));

  const ModelInstanceIndex instance2 = instances.back();
  DRAKE_EXPECT_NO_THROW(plant.GetJointByName<PlanarJoint>("planar_joint",
                                                          instance2));
  const PlanarJoint<double>& planar_joint2 =
      plant.GetJointByName<PlanarJoint>("planar_joint", instance2);
  EXPECT_EQ(planar_joint2.name(), "planar_joint");
  EXPECT_EQ(planar_joint2.parent_body().name(), "link6");
  EXPECT_EQ(planar_joint2.parent_body().model_instance(), instance2);
  EXPECT_EQ(planar_joint2.child_body().name(), "link7");
  EXPECT_EQ(planar_joint2.child_body().model_instance(), instance2);
  EXPECT_TRUE(CompareMatrices(planar_joint2.damping(),
                              Vector3d::Constant(0.2)));
  EXPECT_TRUE(CompareMatrices(planar_joint2.position_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint2.position_upper_limits(), inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint2.velocity_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint2.velocity_upper_limits(), inf3));
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
      std::logic_error, ".*There is no JointActuator named.*");
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

void FailWithRelativeToNotDefined(const std::string& inner) {
  SCOPED_TRACE(inner);
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(inner),
      std::runtime_error,
      R"([\s\S]*XML Attribute\[relative_to\] in element\[pose\] not )"
      R"(defined in SDF.\n)");
}

void FailWithInvalidWorld(const std::string& inner) {
  SCOPED_TRACE(inner);
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(inner),
      std::runtime_error,
      R"([\s\S]*(attached_to|relative_to) name\[world\] specified by frame )"
      R"(with name\[.*\] does not match a nested model, link, joint, or )"
      R"(frame name in model with name\[bad\][\s\S]*)");
}

void FailWithReservedName(const std::string& inner) {
  SCOPED_TRACE(inner);
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

  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(R"(
<model name='bad'>
  <pose relative_to='invalid_usage'/>
  <link name='dont_crash_plz'/>  <!-- Need at least one frame -->
</model>)"),
      R"([\s\S]*Error: Attribute //pose\[@relative_to\] of top level model )"
      R"(must be left empty[\s\S]*)");

  FailWithRelativeToNotDefined(R"(
<model name='bad'>
  <frame name='my_frame'/>
  <link name='a'>
    <inertial><pose relative_to='my_frame'/></inertial>
  </link>
</model>)");
}

// Tests Drake's usage of sdf::EnforcementPolicy.
GTEST_TEST(SdfParser, TestSdformatParserPolicies) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(R"""(
<model name='model_with_bad_attribute' bad_attribute="junk">
  <link name='a'/>
</model>
)"""),
      std::runtime_error,
      R"([\s\S]*XML Attribute\[bad_attribute\] in element\[model\] not )"
      R"(defined in SDF.[\s\S]*)");

  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(R"""(
<model name='model_with_too_many_top_level_elements'>
  <link name='a'/>
</model>
<model name='two_models_too_many'>
  <link name='b'/>
</model>
)"""),
    R"([\s\S]*Root object can only contain one model.*)");

  std::stringstream buffer;
  sdf::Console::ConsoleStream old_stream =
    sdf::Console::Instance()->GetMsgStream();
  ScopeExit revert_stream(
    [&old_stream]()
    { sdf::Console::Instance()->GetMsgStream() = old_stream; });
  sdf::Console::Instance()->GetMsgStream() =
    sdf::Console::ConsoleStream(&buffer);

  // TODO(#15018): This throws a warning, make this an error.
  ParseTestString(R"""(
<model name='model_with_bad_element'>
  <link name='a'/>
  <bad_element/>
</model>
)""");

  EXPECT_THAT(buffer.str(), testing::MatchesRegex(
      ".*Warning.*XML Element\\[bad_element\\], child of"
      " element\\[model\\], not defined in SDF.*"));

  ParseTestString(R"""(
<model name='a'>
  <link name='l1'/>
  <link name='l2'/>
  <joint name='b' type="revolute">
    <child>l1</child>
    <parent>l2</parent>
    <axis>
      <initial_position>0</initial_position>
    </axis>
  </joint>
</model>)""", "1.9");

  EXPECT_THAT(buffer.str(), testing::MatchesRegex(
      ".*Warning.*XML Element\\[initial_position\\], child of element"
      "\\[axis\\], not defined in SDF.*"));
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
  // Test successful parsing.  Add two copies of the model to make sure the
  // bushings are associated with the proper model instance.
  auto [plant, scene_graph] = ParseTestString(R"(
    <world name='BushingWorld'>
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
      </model>
      <model name='BushingModel2'>
        <link name='A'/>
        <link name='C'/>
        <frame name='frameA' attached_to='A'/>
        <frame name='frameC' attached_to='C'/>
        <drake:linear_bushing_rpy>
          <drake:bushing_frameA>frameA</drake:bushing_frameA>
          <drake:bushing_frameC>frameC</drake:bushing_frameC>
          <drake:bushing_torque_stiffness>11 12 13</drake:bushing_torque_stiffness>
          <drake:bushing_torque_damping>14 15 16</drake:bushing_torque_damping>
          <drake:bushing_force_stiffness>17 18 19</drake:bushing_force_stiffness>
          <drake:bushing_force_damping>20 21 22</drake:bushing_force_damping>
        </drake:linear_bushing_rpy>
      </model>
    </world>)");

  // MBP will always create a UniformGravityField, so the only other
  // ForceElements should be the LinearBushingRollPitchYaw elements parsed.
  EXPECT_EQ(plant->num_force_elements(), 3);

  const LinearBushingRollPitchYaw<double>& bushing =
      plant->GetForceElement<LinearBushingRollPitchYaw>(ForceElementIndex(1));

  EXPECT_STREQ(bushing.frameA().name().c_str(), "frameA");
  EXPECT_STREQ(bushing.frameC().name().c_str(), "frameC");
  EXPECT_EQ(bushing.frameA().model_instance(), bushing.model_instance());
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
  EXPECT_EQ(bushing2.torque_stiffness_constants(), Eigen::Vector3d(11, 12, 13));
  EXPECT_EQ(bushing2.torque_damping_constants(), Eigen::Vector3d(14, 15, 16));
  EXPECT_EQ(bushing2.force_stiffness_constants(), Eigen::Vector3d(17, 18, 19));
  EXPECT_EQ(bushing2.force_damping_constants(), Eigen::Vector3d(20, 21, 22));

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

// Verifies that the SDFormat loader can add directly nested models to a
// multibody plant. For reference, the files test/integration/model_dom.cc and
// test/integration/nested_model.cc in the libsdformat source code (tag
// sdformat11_11.0.0) contain tests that show more detailed behavior of
// SDFormat's nested model support.
GTEST_TEST(SdfParser, LoadDirectlyNestedModelsInWorld) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "world_with_directly_nested_models.sdf");
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
  // The models should have added 4 more bodies.
  EXPECT_EQ(plant.num_bodies(), 5);
  // The models should have added 3 more joints.
  EXPECT_EQ(plant.num_joints(), 3);

  // There should be a model instance with the name "parent_model".
  ASSERT_TRUE(plant.HasModelInstanceNamed("parent_model"));

  // There should be a model instance with the name "parent_model::robot1".
  // This is the model "robot1" nested inside "parent_model"
  ASSERT_TRUE(plant.HasModelInstanceNamed("parent_model::robot1"));
  ModelInstanceIndex robot1_model =
    plant.GetModelInstanceByName("parent_model::robot1");

  // There should be a body with the name "base_link".
  EXPECT_TRUE(plant.HasBodyNamed("base_link", robot1_model));
  // There should be another body with the name "moving_link".
  EXPECT_TRUE(plant.HasBodyNamed("moving_link", robot1_model));
  // There should be joint with the name "slider".
  EXPECT_TRUE(plant.HasJointNamed("slider", robot1_model));

  // There should be a model instance with the name "parent_model::robot2".
  // This is the model "robot2" nested inside "parent_model"
  ASSERT_TRUE(plant.HasModelInstanceNamed("parent_model::robot2"));
  ModelInstanceIndex robot2_model =
    plant.GetModelInstanceByName("parent_model::robot2");

  // There should be a body with the name "base_link".
  EXPECT_TRUE(plant.HasBodyNamed("base_link", robot2_model));
  // There should be another body with the name "moving_link".
  EXPECT_TRUE(plant.HasBodyNamed("moving_link", robot2_model));
  // There should be joint with the name "slider".
  EXPECT_TRUE(plant.HasJointNamed("slider", robot2_model));

  // There should be a joint named "weld_robots". By convention, the joint
  // will have the same model instance as the child frame.
  EXPECT_TRUE(plant.HasJointNamed("weld_robots", robot2_model));
}

// Same test as LoadDirectlyNestedModelsInWorld, but where a model file contains
// direclty nested models.
GTEST_TEST(SdfParser, LoadDirectlyNestedModelsInModel) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "model_with_directly_nested_models.sdf");
  MultibodyPlant<double> plant(0.0);

  // We start with the world and default model instances.
  ASSERT_EQ(plant.num_model_instances(), 2);
  ASSERT_EQ(plant.num_bodies(), 1);
  ASSERT_EQ(plant.num_joints(), 0);

  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);
  AddModelsFromSdfFile(full_name, package_map, &plant);
  plant.Finalize();

  // We should have loaded 4 more models.
  EXPECT_EQ(plant.num_model_instances(), 6);
  // The models should have added 4 more bodies.
  EXPECT_EQ(plant.num_bodies(), 5);
  // The models should have added 3 more joints.
  EXPECT_EQ(plant.num_joints(), 3);

  // There should be a model instance with the name "grand_parent_model" (top
  // level model).
  ASSERT_TRUE(plant.HasModelInstanceNamed("grand_parent_model"));

  // There should be a model instance with the name
  // "grand_parent_model::parent_model". This is the model "parent_model"
  // nested inside "grand_parent_model"
  ASSERT_TRUE(
      plant.HasModelInstanceNamed("grand_parent_model::parent_model"));

  // There should be a model instance with the name
  // "grand_parent_model::parent_model::robot1". This is the model "robot1"
  // nested inside "parent_model" which itself is nested inside
  // grand_parent_model
  ASSERT_TRUE(plant.HasModelInstanceNamed(
        "grand_parent_model::parent_model::robot1"));
  ModelInstanceIndex robot1_model = plant.GetModelInstanceByName(
      "grand_parent_model::parent_model::robot1");

  // There should be a body with the name "base_link".
  EXPECT_TRUE(plant.HasBodyNamed("base_link", robot1_model));
  // There should be another body with the name "moving_link".
  EXPECT_TRUE(plant.HasBodyNamed("moving_link", robot1_model));
  // There should be joint with the name "slider".
  EXPECT_TRUE(plant.HasJointNamed("slider", robot1_model));

  // There should be a model instance with the name
  // "grand_parent_model::parent_model::robot2". This is the model "robot2"
  // nested inside "parent_model" which itself is nested inside
  // grand_parent_model
  ASSERT_TRUE(plant.HasModelInstanceNamed(
        "grand_parent_model::parent_model::robot2"));
  ModelInstanceIndex robot2_model = plant.GetModelInstanceByName(
      "grand_parent_model::parent_model::robot2");

  // There should be a body with the name "base_link".
  EXPECT_TRUE(plant.HasBodyNamed("base_link", robot2_model));
  // There should be another body with the name "moving_link".
  EXPECT_TRUE(plant.HasBodyNamed("moving_link", robot2_model));
  // There should be joint with the name "slider".
  EXPECT_TRUE(plant.HasJointNamed("slider", robot2_model));

  // There should be a joint named "weld_robots". By convention, the joint
  // will have the same model instance as the child frame.
  EXPECT_TRUE(plant.HasJointNamed("weld_robots", robot2_model));
}

// Example model taken from
// http://sdformat.org/tutorials?tut=composition_proposal&cat=pose_semantics_docs&#1-4-4-placement-frame-model-placement_frame-and-include-placement_frame
GTEST_TEST(SdfParser, ModelPlacementFrame) {
  const std::string model_string = R"""(
<model name='table'> <!-- T -->
  <pose>0 10 0  0 0 0</pose>
  <link name='table_top'> <!-- S -->
    <pose>0 0 1  0 0 0</pose>
  </link>

  <model name='mug' placement_frame='base'> <!-- M -->
    <pose relative_to='table_top'/>
    <link name='handle'> <!-- H -->
      <pose>0.1 0 0  0 0 0</pose>
    </link>
    <link name='base'> <!-- B -->
      <pose>0 0 -0.1  0 0 0</pose>
    </link>
  </model>

</model>)""";
  auto [plant, scene_graph] = ParseTestString(model_string, "1.8");
  ASSERT_NE(nullptr, plant);
  plant->Finalize();
  EXPECT_GT(plant->num_positions(), 0);
  auto context = plant->CreateDefaultContext();

  ASSERT_TRUE(plant->HasModelInstanceNamed("table::mug"));
  ModelInstanceIndex model_m = plant->GetModelInstanceByName("table::mug");

  ASSERT_TRUE(plant->HasFrameNamed("__model__", model_m));
  const Frame<double>& frame_M = plant->GetFrameByName("__model__", model_m);

  ASSERT_TRUE(plant->HasFrameNamed("table_top"));
  const Frame<double>& frame_S = plant->GetFrameByName("table_top");

  ASSERT_TRUE(plant->HasFrameNamed("base", model_m));
  const Frame<double>& frame_B = plant->GetFrameByName("base", model_m);

  ASSERT_TRUE(plant->HasFrameNamed("handle", model_m));
  const Frame<double>& frame_H = plant->GetFrameByName("handle", model_m);

  // X_SM = X_SB * X_MB^-1.
  const RigidTransformd X_SM_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                      Vector3d(0.0, 0.0, 0.1));

  const RigidTransformd X_SB_expected = RigidTransformd::Identity();
  // X_SH = X_SB * X_HB^-1.
  //      = X_SB * (X_MH^-1 * X_MB)^-1.
  const RigidTransformd X_SH_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                      Vector3d(0.1, 0.0, 0.1));

  const RigidTransformd X_SM = frame_M.CalcPose(*context, frame_S);
  const RigidTransformd X_SH = frame_H.CalcPose(*context, frame_S);
  const RigidTransformd X_SB = frame_B.CalcPose(*context, frame_S);

  EXPECT_TRUE(CompareMatrices(
      X_SM_expected.GetAsMatrix4(), X_SM.GetAsMatrix4(), kEps));
  EXPECT_TRUE(CompareMatrices(
      X_SB_expected.GetAsMatrix4(), X_SB.GetAsMatrix4(), kEps));
  EXPECT_TRUE(CompareMatrices(
      X_SH_expected.GetAsMatrix4(), X_SH.GetAsMatrix4(), kEps));

  // X_WM = X_WT * X_TM
  // X_TM = X_TS * X_MS^-1
  // X_MS = X_MB * X_SB^-1
  // The model frame M is 0.1m in the +z axis from frame B, but we know from the
  // use of placement_frame that frame B and frame S are coincident. So X_WM is
  // 0.1m in the +z axis from frame S.
  const RigidTransformd X_WM_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                      Vector3d(0.0, 10.0, 1.1));

  const RigidTransformd X_WB_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                      Vector3d(0.0, 10.0, 1.0));

  const RigidTransformd X_WH_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                      Vector3d(0.1, 10.0, 1.1));

  const RigidTransformd X_WM = frame_M.CalcPoseInWorld(*context);
  const RigidTransformd X_WH = frame_H.CalcPoseInWorld(*context);
  const RigidTransformd X_WB = frame_B.CalcPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(
      X_WM_expected.GetAsMatrix4(), X_WM.GetAsMatrix4(), kEps));
  EXPECT_TRUE(CompareMatrices(
      X_WB_expected.GetAsMatrix4(), X_WB.GetAsMatrix4(), kEps));
  EXPECT_TRUE(CompareMatrices(
      X_WH_expected.GetAsMatrix4(), X_WH.GetAsMatrix4(), kEps));
}

// Verify that poses can be given relative to deeply nested frames.
GTEST_TEST(SdfParser, PoseRelativeToMultiLevelNestedFrame) {
  const std::string model_string = R"""(
<model name='a'>
  <pose>0.1 0 0  0 0 0</pose>
  <model name='b'>
    <pose>0 0.2 0.0  0 0 0</pose>
    <model name='c'>
      <pose>0 0.0 0.3  0 0 0</pose>
      <link name='d'>
        <pose>0 0.0 0.0  0 0.5 0.6</pose>
      </link>
    </model>
  </model>
  <link name='e'>
    <pose relative_to="b::c::d">0 0 0  0.4 0 0.0</pose>
  </link>
</model>)""";
  auto [plant, scene_graph] = ParseTestString(model_string, "1.8");
  ASSERT_NE(nullptr, plant);
  plant->Finalize();
  EXPECT_GT(plant->num_positions(), 0);
  auto context = plant->CreateDefaultContext();

  const RigidTransformd X_WE_expected(RollPitchYawd(0.4, 0.5, 0.6),
                                      Vector3d(0.1, 0.2, 0.3));

  const RigidTransformd X_WE =
      plant->GetFrameByName("e").CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WE_expected.GetAsMatrix4(), X_WE.GetAsMatrix4(), kEps));
}

// Verify that joint axis can be expressed in deeply nested frames.
GTEST_TEST(SdfParser, AxisXyzExperssedInMultiLevelNestedFrame) {
  const std::string model_string = fmt::format(R"""(
<model name='a'>
  <pose>0.1 0 0  0 0 0</pose>
  <model name='b'>
    <pose>0 0.2 0.0  0 0 0</pose>
    <model name='c'>
      <pose>0 0.0 0.3  0 0 0</pose>
      <link name='d'>
        <pose>0 0.0 0.0  0 {} {}</pose>
      </link>
    </model>
  </model>
  <link name='e'/>
  <link name='f'/>
  <joint name="j" type="revolute">
    <parent>e</parent>
    <child>f</child>
    <axis>
      <xyz expressed_in="b::c::d">1 0 0</xyz>
    </axis>
  </joint>
</model>)""", M_PI_2, M_PI_2);
  auto [plant, scene_graph] = ParseTestString(model_string, "1.8");
  ASSERT_NE(nullptr, plant);
  plant->Finalize();
  EXPECT_GT(plant->num_positions(), 0);
  auto context = plant->CreateDefaultContext();

  const RollPitchYawd R_WD(0.0, M_PI_2, M_PI_2);

  const Vector3d xyz_D(1, 0, 0);

  const Vector3d xyz_W_expected = R_WD.ToRotationMatrix() * xyz_D;

  DRAKE_EXPECT_NO_THROW(plant->GetJointByName<RevoluteJoint>("j"));
  const RevoluteJoint<double>& joint_j =
      plant->GetJointByName<RevoluteJoint>("j");
  EXPECT_TRUE(CompareMatrices(xyz_W_expected, joint_j.revolute_axis(), kEps));
}

// Verify frames can be attached to nested links or models
GTEST_TEST(SdfParser, FrameAttachedToMultiLevelNestedFrame) {
  const std::string model_string = R"""(
<model name='a'>
  <pose>0.1 0 0  0 0 0</pose>
  <model name='b'>
    <pose>0 0.2 0.0  0 0 0</pose>
    <model name='c'>
      <pose>0 0.0 0.3  0 0 0</pose>
      <link name='d'>
        <pose>0 0.0 0.0  0 0.5 0.6</pose>
      </link>
    </model>
  </model>
  <frame name='e' attached_to='b::c::d'> <!-- Frame attached to a link -->
    <pose>0 0 0  0.4 0 0.0</pose>
  </frame>
  <frame name='f' attached_to='b::c'> <!-- Frame attached to a model -->
    <pose>0 0 0  0.4 0.5 0.6</pose>
  </frame>
</model>)""";
  auto [plant, scene_graph] = ParseTestString(model_string, "1.8");
  ASSERT_NE(nullptr, plant);
  plant->Finalize();
  EXPECT_GT(plant->num_positions(), 0);
  auto context = plant->CreateDefaultContext();

  const RigidTransformd X_WE_expected(RollPitchYawd(0.4, 0.5, 0.6),
                                      Vector3d(0.1, 0.2, 0.3));
  const RigidTransformd X_WF_expected(RollPitchYawd(0.4, 0.5, 0.6),
                                      Vector3d(0.1, 0.2, 0.3));

  const auto& frame_E = plant->GetFrameByName("e");
  const RigidTransformd X_WE = frame_E.CalcPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(
      X_WE_expected.GetAsMatrix4(), X_WE.GetAsMatrix4(), kEps));

  const auto& frame_F = plant->GetFrameByName("f");
  const RigidTransformd X_WF = frame_F.CalcPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(
      X_WF_expected.GetAsMatrix4(), X_WF.GetAsMatrix4(), kEps));

  // Also check that the frame is attached to the right body
  ModelInstanceIndex model_c_instance =
      plant->GetModelInstanceByName("a::b::c");
  EXPECT_EQ(frame_E.body().index(),
            plant->GetBodyByName("d", model_c_instance).index());

  EXPECT_EQ(frame_F.body().index(),
            plant->GetBodyByName("d", model_c_instance).index());
}

// Verify frames and links can have the same local name without violating name
// uniqueness requirements
GTEST_TEST(SdfParser, RepeatedLinkName) {
  const std::string model_string = R"""(
<world name='a'>
  <model name='b1'>
    <link name='c'/>
    <frame name='d'/>
  </model>
  <model name='b2'>
    <link name='c'/>
    <frame name='d'/>
  </model>
</world>)""";
  PlantAndSceneGraph pair;
  DRAKE_ASSERT_NO_THROW(pair = ParseTestString(model_string, "1.8"));
}

// Verify frames can be attached to models in a SDFormat world
GTEST_TEST(SdfParser, FrameAttachedToModelFrameInWorld) {
  const std::string model_string = R"""(
<world name='a'>
  <model name='b'>
    <pose>0.1 0.2 0.0  0 0 0</pose>
    <model name='c'>
      <pose>0 0.0 0.3  0 0 0</pose>
      <link name='d'/>
    </model>
  </model>
  <frame name='e' attached_to='b'>
    <pose>0 0 0.3  0.0 0.0 0.0</pose>
  </frame>
  <frame name='f' attached_to='b::c'>
    <pose>0 0 0  0.0 0.0 0.6</pose>
  </frame>
</world>)""";
  auto [plant, scene_graph] = ParseTestString(model_string, "1.8");

  ASSERT_NE(nullptr, plant);
  plant->Finalize();
  EXPECT_GT(plant->num_positions(), 0);
  auto context = plant->CreateDefaultContext();

  const RigidTransformd X_WE_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                      Vector3d(0.1, 0.2, 0.3));
  const RigidTransformd X_WF_expected(RollPitchYawd(0.0, 0.0, 0.6),
                                      Vector3d(0.1, 0.2, 0.3));

  const auto& frame_E = plant->GetFrameByName("e");
  const RigidTransformd X_WE = frame_E.CalcPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(
      X_WE_expected.GetAsMatrix4(), X_WE.GetAsMatrix4(), kEps));

  const auto& frame_F = plant->GetFrameByName("f");
  const RigidTransformd X_WF = frame_F.CalcPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(
      X_WF_expected.GetAsMatrix4(), X_WF.GetAsMatrix4(), kEps));

  // Also check that the frame is attached to the right body
  EXPECT_EQ(frame_E.body().index(),
            plant->GetBodyByName("d").index());

  EXPECT_EQ(frame_F.body().index(),
            plant->GetBodyByName("d").index());
}

// Verify frames can be attached to joint frames
GTEST_TEST(SdfParser, FrameAttachedToJointFrame) {
  const std::string model_string = R"""(
<world name='default'>
  <model name='parent_model'>
    <pose>0.1 0.2 0.0  0 0 0</pose>
    <link name='L1'/>
    <link name='L2'/>
    <model name='M1'>
      <pose>0 0.0 0.3  0 0 0</pose>
      <link name='L3'/>
    </model>
    <!-- SDFormat has implicit frames for joints J1 and J2, but Drake does not
    -->
    <joint name='J1' type='fixed'>
      <parent>L1</parent>
      <child>L2</child>
    </joint>
    <joint name='J2' type='fixed'>
      <parent>L1</parent>
      <child>M1::L3</child>
    </joint>
    <frame name='F1' attached_to='J1'/>
    <frame name='F2' attached_to='J2'/>
  </model>
</world>)""";
  auto [plant, scene_graph] = ParseTestString(model_string, "1.8");

  ASSERT_NE(nullptr, plant);
  plant->Finalize();
  auto context = plant->CreateDefaultContext();

  const RigidTransformd X_WF1_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                       Vector3d(0.1, 0.2, 0.0));

  const RigidTransformd X_WF2_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                       Vector3d(0.1, 0.2, 0.3));

  const auto& frame_F1 = plant->GetFrameByName("F1");
  const auto& frame_F2 = plant->GetFrameByName("F2");
  const RigidTransformd X_WF1 = frame_F1.CalcPoseInWorld(*context);
  const RigidTransformd X_WF2 = frame_F2.CalcPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(
      X_WF1_expected.GetAsMatrix4(), X_WF1.GetAsMatrix4(), kEps));
  EXPECT_TRUE(CompareMatrices(
      X_WF2_expected.GetAsMatrix4(), X_WF2.GetAsMatrix4(), kEps));

  // Also check that the frame is attached to the right body
  EXPECT_EQ(frame_F1.body().index(),
            plant->GetBodyByName("L2").index());
  EXPECT_EQ(frame_F2.body().index(),
            plant->GetBodyByName("L3").index());
}

GTEST_TEST(SdfParser, SupportNonDefaultCanonicalLink) {
  // Verify that non-default canonical links are handled properly. Here we have
  // three different types of references used for the canonical link:
  // * c::e - Nested link
  // * f - Link that is not the first link in the model
  const std::string model_string = R"""(
  <model name='a' canonical_link='c::e'>
    <link name='b'/>
    <model name='c' canonical_link='f'>
      <link name='d'/>
      <link name='e'/>
      <link name='f'/>
    </model>
  </model>)""";
  auto [plant, scene_graph] = ParseTestString(model_string, "1.8");

  ASSERT_NE(nullptr, plant);
  plant->Finalize();

  EXPECT_EQ(GetModelFrameByName(*plant, "a").body().index(),
            plant->GetBodyByName("e").index());

  EXPECT_EQ(GetModelFrameByName(*plant, "a::c").body().index(),
            plant->GetBodyByName("f").index());
}

// Verify that frames can be used for //joint/parent and //joint/child
GTEST_TEST(SdfParser, FramesAsJointParentOrChild) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "frames_as_joint_parent_or_child.sdf");
  MultibodyPlant<double> plant(0.0);

  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_name);
  AddModelsFromSdfFile(full_name, package_map, &plant);
  ASSERT_TRUE(plant.HasModelInstanceNamed("parent_model"));

  plant.Finalize();
  auto context = plant.CreateDefaultContext();

  const RigidTransformd X_CJc_expected = RigidTransformd::Identity();
  const RigidTransformd X_PJp_expected(RollPitchYawd(0, 0, 0),
                                       Vector3d(3, 3, 3));

  // Frames attached to links in the same model
  {
    const auto& joint = plant.GetJointByName("J1");
    const auto& frame_P = plant.GetFrameByName("L1_offset");
    const auto& frame_C = plant.GetFrameByName("L2_offset");

    const RigidTransformd X_PJp =
        joint.frame_on_parent().CalcPose(*context, frame_P);
    EXPECT_TRUE(CompareMatrices(X_PJp_expected.GetAsMatrix4(),
                                X_PJp.GetAsMatrix4(), kEps));

    const RigidTransformd X_CJc =
        joint.frame_on_child().CalcPose(*context, frame_C);
    EXPECT_TRUE(CompareMatrices(X_CJc_expected.GetAsMatrix4(),
                                X_CJc.GetAsMatrix4(), kEps));
  }
  // Frames attached to links in the other (nested) models
  {
    const auto& joint = plant.GetJointByName("J2");

    const auto& frame_P = plant.GetFrameByName("M1_base_link_offset");
    const auto& frame_C = plant.GetFrameByName("M2_base_link_offset");

    const RigidTransformd X_PJp =
        joint.frame_on_parent().CalcPose(*context, frame_P);
    EXPECT_TRUE(CompareMatrices(X_PJp_expected.GetAsMatrix4(),
                                X_PJp.GetAsMatrix4(), kEps));

    const RigidTransformd X_CJc =
        joint.frame_on_child().CalcPose(*context, frame_C);
    EXPECT_TRUE(CompareMatrices(X_CJc_expected.GetAsMatrix4(),
                                X_CJc.GetAsMatrix4(), kEps));
  }
}

// Verifies that URDF files can be loaded into Drake via libsdformat's Interface
// API which bypasses the URDF->SDFormat conversion. This also verifies that
// SDFormat files can be forced to be loaded via the Interface API by changing
// their file extension and registering the appropriate custom parser.
GTEST_TEST(SdfParser, InterfaceAPI) {
  const std::string sdf_file_path = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/interface_api_test/"
      "top.sdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(sdf_file_path);
  MultibodyPlant<double> plant(0.0);

  DRAKE_ASSERT_NO_THROW(AddModelFromSdfFile(sdf_file_path, "", package_map,
                                            &plant, nullptr, true));

  plant.Finalize();
  auto context = plant.CreateDefaultContext();

  {
    // Frame A represents the model frame of model top::arm
    const RigidTransformd X_WA_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                        Vector3d(1, 0, 0));
    const auto arm_model_instance = plant.GetModelInstanceByName("top::arm");
    const auto& arm_model_frame =
        plant.GetFrameByName("__model__", arm_model_instance);
    const RigidTransformd X_WA = arm_model_frame.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WA_expected.GetAsMatrix4(),
                                X_WA.GetAsMatrix4(), kEps));
    const auto& arm_L1 = plant.GetFrameByName("L1", arm_model_instance);
    const RigidTransformd X_WL1 = arm_L1.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WA_expected.GetAsMatrix4(),
                                X_WL1.GetAsMatrix4(), kEps));
  }

  {
    // Frame E represents the model frame of model top::extra_arm
    const RigidTransformd X_WE_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                        Vector3d(1, 2, 0));
    const auto extra_arm_model_instance =
        plant.GetModelInstanceByName("top::extra_arm");
    const auto& extra_arm_model_frame =
        plant.GetFrameByName("__model__", extra_arm_model_instance);
    const RigidTransformd X_WE =
        extra_arm_model_frame.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WE_expected.GetAsMatrix4(),
                                X_WE.GetAsMatrix4(), kEps));

    const RigidTransformd X_WL2_expected(RollPitchYawd(0.1, 0.2, 0.3),
                                        Vector3d(2, 4, 3));
    const auto& arm_L2 = plant.GetFrameByName("L2", extra_arm_model_instance);
    const RigidTransformd X_WL2 = arm_L2.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WL2_expected.GetAsMatrix4(),
                                X_WL2.GetAsMatrix4(), kEps));
  }
  {
    // Frame F represents the model frame of model top::arm::flange
    const RigidTransformd X_WF_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                        Vector3d(1, 2, 1));
    const auto flange_model_instance =
        plant.GetModelInstanceByName("top::arm::flange");
    const auto& flange_model_frame =
        plant.GetFrameByName("__model__", flange_model_instance);
    const RigidTransformd X_WF =
        flange_model_frame.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WF_expected.GetAsMatrix4(),
                                X_WF.GetAsMatrix4(), kEps));

    // Frame M represents the frame of model top::arm::flange::gripper_mount
    const RigidTransformd X_WM_expected(RollPitchYawd(0.1, 0.2, 0.3),
                                        Vector3d(1, 2, 3));
    const auto& gripper_mount_frame =
        plant.GetFrameByName("gripper_mount", flange_model_instance);
    const RigidTransformd X_WM = gripper_mount_frame .CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WM_expected.GetAsMatrix4(),
                                X_WM.GetAsMatrix4(), kEps));

    // Frame G represents the frame of model top::arm::flange::gripper
    const RigidTransformd X_WG_expected(RollPitchYawd(0.1, 0.2, 0.3),
                                        Vector3d(1, 2, 3));
    const auto gripper_model_instance =
        plant.GetModelInstanceByName("top::arm::gripper");
    const auto& gripper_model_frame =
        plant.GetFrameByName("__model__", gripper_model_instance);
    const RigidTransformd X_WG = gripper_model_frame.CalcPoseInWorld(*context);
    // TODO(azeey) There is a precision loss that occurs in libsdformat when
    // resolving poses. Use just kEps when the following ign-math issue is
    // resolved: https://github.com/ignitionrobotics/ign-math/issues/212.
    EXPECT_TRUE(CompareMatrices(X_WG_expected.GetAsMatrix4(),
                                X_WG.GetAsMatrix4(), 10 * kEps));
  }
  // Test placement_frame using a table and a mug flipped upside down
  {
    // Frame T represents the frame top::table_and_mug::mug::top
    const RigidTransformd X_WT_expected(RollPitchYawd(M_PI_2, 0.0, 0.0),
                                        Vector3d(3, 0, 0.5));
    const auto mug_model_instance =
        plant.GetModelInstanceByName("top::table_and_mug::mug");
    const auto& mug_top_frame =
        plant.GetFrameByName("top", mug_model_instance);
    const RigidTransformd X_WT =
        mug_top_frame.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WT_expected.GetAsMatrix4(),
                                X_WT.GetAsMatrix4(), kEps));
  }
}

// TODO(SeanCurtis-TRI) The logic testing for collision filter group parsing
// belongs in detail_common_test.cc. Urdf and Sdf parsing just need enough
// testing to indicate that the method is being invoked correctly.
GTEST_TEST(SdfParser, CollisionFilterGroupParsingTest) {
  const std::string full_sdf_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
      "sdf_parser_test/collision_filter_group_parsing_test.sdf");
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(full_sdf_filename);

  // Read in the SDF file.
  AddModelFromSdfFile(full_sdf_filename, "", package_map, &plant, &scene_graph);

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
  AddModelFromSdfFile(
      full_sdf_filename, "model2", package_map, &plant, &scene_graph);
}

// TODO(marcoag) We might want to add some form of feedback for:
// - ignore_collision_filter_groups with non-existing group names.
// - Empty collision_filter_groups.
GTEST_TEST(SdfParser, CollisionFilterGroupParsingErrorsTest) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(R"""(
<model name='error'>
  <link name='a'/>
  <drake:collision_filter_group/>
</model>)"""),
      std::runtime_error,
      ".*The tag <drake:collision_filter_group> is "
      "missing the required attribute "
      "\"name\".*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(R"""(
<model name='error'>
  <link name='a'/>
  <drake:collision_filter_group name="group_a">
    <drake:member></drake:member>
  </drake:collision_filter_group>
</model>)"""),
      std::runtime_error,
      ".*The tag <drake:member> is missing a required string value.*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(R"""(
<model name='error'>
  <link name='a'/>
  <drake:collision_filter_group name="group_a">
    <drake:ignored_collision_filter_group>
    </drake:ignored_collision_filter_group>
  </drake:collision_filter_group>
</model>)"""),
      std::runtime_error,
      ".*The tag <drake:ignored_collision_filter_group> is missing a "
      "required string value.*");
}

GTEST_TEST(SdfParser, PoseWithRotationInDegreesOrQuaternions) {
  const std::string model_string = R"""(
  <model name='test_model'>
    <link name='E1'>
      <pose degrees='true'>1 2 3   15 30 45</pose>
    </link>
    <link name='E2'>
      <pose rotation_format="euler_rpy" degrees='true'>1 2 3   15 30 45</pose>
    </link>
    <link name='Q'>
      <pose rotation_format='quat_xyzw'>
        1 2 3
        0.3826834323650898 0 0 0.9238795325112867
      </pose>
    </link>
  </model>)""";
  auto [plant, scene_graph] = ParseTestString(model_string, "1.9");
  plant->Finalize();
  auto context = plant->CreateDefaultContext();

  constexpr double kDegToRad = M_PI / 180.0;

  const RigidTransformd X_WE_expected(
      RollPitchYawd(kDegToRad * 15, kDegToRad * 30, kDegToRad * 45),
      Vector3d(1, 2, 3));

  const RigidTransformd X_WE1 =
      plant->GetFrameByName("E1").CalcPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(X_WE_expected.GetAsMatrix4(),
                              X_WE1.GetAsMatrix4(), kEps));

  const RigidTransformd X_WE2 =
      plant->GetFrameByName("E2").CalcPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(X_WE_expected.GetAsMatrix4(),
                              X_WE2.GetAsMatrix4(), kEps));

  const RigidTransformd X_WQ_expected(RollPitchYawd(M_PI_4, 0, 0),
                                      Vector3d(1, 2, 3));

  const RigidTransformd X_WQ =
      plant->GetFrameByName("Q").CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WQ_expected.GetAsMatrix4(), X_WQ.GetAsMatrix4(), kEps));
}

GTEST_TEST(SdfParser, MergeInclude) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "merge_include_models.sdf");

  MultibodyPlant<double> plant(0.0);

  // We start with the world and default model instances.
  ASSERT_EQ(plant.num_model_instances(), 2);
  ASSERT_EQ(plant.num_bodies(), 1);
  ASSERT_EQ(plant.num_joints(), 0);

  PackageMap package_map;
  package_map.PopulateFromFolder(filesystem::path(full_name).parent_path());
  AddModelsFromSdfFile(full_name, package_map, &plant);
  plant.Finalize();

  // We should have loaded *only* 1 more model.
  EXPECT_EQ(plant.num_model_instances(), 3);
  EXPECT_EQ(plant.num_bodies(), 4);
  EXPECT_EQ(plant.num_joints(), 2);

  ASSERT_TRUE(plant.HasModelInstanceNamed("robot1_with_tool"));
  ModelInstanceIndex robot1_model =
      plant.GetModelInstanceByName("robot1_with_tool");

  // The bodies and joints from "simple_robot1" should be merged into
  // "robot1_with_tool" making them direct children of the "robot1_with_tool"
  // model instance.
  EXPECT_TRUE(plant.HasBodyNamed("base_link", robot1_model));
  EXPECT_TRUE(plant.HasBodyNamed("moving_link", robot1_model));
  EXPECT_TRUE(plant.HasJointNamed("slider", robot1_model));

  // The bodies and joints directly specified in "robot1_with_tool" should be at
  // the same level of hierarchy as those merged from "simple_robot1"
  EXPECT_TRUE(plant.HasBodyNamed("tool", robot1_model));
  EXPECT_TRUE(plant.HasJointNamed("tool_joint", robot1_model));
}
}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
