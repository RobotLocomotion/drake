#include "drake/multibody/parsing/detail_sdf_parser.h"

#include <memory>

#include <gtest/gtest.h>
#include <sdf/sdf.hh>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace detail {
namespace {

using Eigen::Isometry3d;
using Eigen::Vector3d;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::SceneGraph;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYaw;
using math::RollPitchYawd;
using systems::Context;

// Verifies that the SDF loader can leverage a specified package map.
GTEST_TEST(MultibodyPlantSdfParserTest, PackageMapSpecified) {
  // We start with the world and default model instances (model_instance.h
  // explains why there are two).
  MultibodyPlant<double> plant;
  geometry::SceneGraph<double> scene_graph;
  ASSERT_EQ(plant.num_model_instances(), 2);

  const std::string full_sdf_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/sdfs/box.sdf");
  const std::string package_path = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package");

  // Construct the PackageMap.
  PackageMap package_map;
  package_map.PopulateFromFolder(package_path);

  // Read in the SDF file.
  AddModelFromSdfFile(full_sdf_filename, "", package_map, &plant, &scene_graph);
  plant.Finalize();

  // Verify the number of model instances.
  EXPECT_EQ(plant.num_model_instances(), 3);
}

// Verifies model instances are correctly created in the plant.
GTEST_TEST(MultibodyPlantSdfParserTest, ModelInstanceTest) {
  // We start with the world and default model instances (model_instance.h
  // explains why there are two).
  MultibodyPlant<double> plant;
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
  const double eps = std::numeric_limits<double>::epsilon();
  auto check_frame = [&plant, instance1, &context, eps](
      std::string parent_name, std::string name,
      const Isometry3d& X_PF_expected) {
    const Frame<double>& frame = plant.GetFrameByName(name, instance1);
    const Frame<double>& parent_frame =
        plant.GetFrameByName(parent_name, instance1);
    const Isometry3d X_PF = plant.CalcRelativeTransform(
        *context, parent_frame, frame);
    EXPECT_TRUE(CompareMatrices(X_PF_expected.matrix(), X_PF.matrix(), eps))
        << name;
  };

  const Isometry3d X_L1F1 = RigidTransformd(
      RollPitchYawd(0.4, 0.5, 0.6), Vector3d(0.1, 0.2, 0.3)).GetAsIsometry3();
  check_frame("link1", "model_scope_link1_frame", X_L1F1);
  const Isometry3d X_F1F2 = RigidTransformd(
      Vector3d(0.1, 0.0, 0.0)).GetAsIsometry3();
  check_frame(
      "model_scope_link1_frame", "model_scope_link1_frame_child", X_F1F2);
  const Isometry3d X_MF3 = RigidTransformd(
      Vector3d(0.7, 0.8, 0.9)).GetAsIsometry3();
  check_frame("instance1", "model_scope_model_frame_implicit", X_MF3);
}

// Verify that our SDF parser throws an exception when a user specifies a joint
// with negative damping.
GTEST_TEST(SdfParserThrowsWhen, JointDampingIsNegative) {
  const std::string sdf_file_path = FindResourceOrThrow(
      "drake/multibody/parsing/test/negative_damping_joint.sdf");
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(sdf_file_path);
  MultibodyPlant<double> plant;
  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromSdfFile(sdf_file_path, "", package_map, &plant),
      std::runtime_error,
      /* Verify this method is throwing for the right reasons. */
      "Joint damping is negative for joint '.*'. "
          "Joint damping must be a non-negative number.");
}

GTEST_TEST(SdfParser, IncludeTags) {
  const std::string sdf_file_path =
      "drake/multibody/parsing/test";
  sdf::addURIPath("model://", FindResourceOrThrow(sdf_file_path));
  MultibodyPlant<double> plant;

  // We start with the world and default model instances.
  ASSERT_EQ(plant.num_model_instances(), 2);
  ASSERT_EQ(plant.num_bodies(), 1);
  ASSERT_EQ(plant.num_joints(), 0);

  PackageMap package_map;
  const std::string full_name = FindResourceOrThrow(
      sdf_file_path + "/include_models.sdf");
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
    MultibodyPlant<double> plant;
    SceneGraph<double> scene_graph;
    AddModelsFromSdfFile(full_name, package_map, &plant, &scene_graph);
    plant.Finalize();
    num_visuals_explicit = plant.num_visual_geometries();
  }
  EXPECT_NE(num_visuals_explicit, 0);
  {
    // Test implicitly specifying.
    MultibodyPlant<double> plant;
    SceneGraph<double> scene_graph;
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
    AddModelsFromSdfFile(full_name, package_map, &plant);
    plant.Finalize();
    EXPECT_EQ(plant.num_visual_geometries(), num_visuals_explicit);
  }
}

}  // namespace
}  // namespace detail
}  // namespace multibody
}  // namespace drake
