#include "drake/multibody/parsing/process_model_directives.h"

#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/scoped_names.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace {

using std::optional;
using Eigen::Vector3d;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;

const char* const kTestDir =
    "drake/multibody/parsing/test/process_model_directives_test";

// Our unit test's package is not normally loaded; construct a parser that
// has it and can resolve package://process_model_directives_test urls.
std::unique_ptr<Parser> make_parser(MultibodyPlant<double>* plant) {
  auto parser = std::make_unique<Parser>(plant);
  const drake::filesystem::path abspath_xml = FindResourceOrThrow(
      std::string(kTestDir) + "/package.xml");
  parser->package_map().AddPackageXml(abspath_xml.string());
  return parser;
}

// Simple smoke test of the most basic model directives.
GTEST_TEST(ProcessModelDirectivesTest, BasicSmokeTest) {
  ModelDirectives station_directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) + "/add_scoped_sub.yaml"));
  const MultibodyPlant<double> empty_plant(0.0);

  MultibodyPlant<double> plant(0.0);
  ProcessModelDirectives(station_directives, &plant,
                         nullptr, make_parser(&plant).get());
  plant.Finalize();

  // Expect the two model instances added by the directives.
  EXPECT_EQ(plant.num_model_instances() - empty_plant.num_model_instances(), 2);

  // Expect the two bodies added by the directives.
  EXPECT_EQ(plant.num_bodies() - empty_plant.num_bodies(), 2);

  // A great many frames are added in model directives processing, but we
  // should at least expect that our named ones are present.
  EXPECT_TRUE(plant.HasFrameNamed("sub_added_frame"));
  EXPECT_TRUE(plant.HasFrameNamed("sub_added_frame_explicit"));
}

// Acceptance tests for the ModelDirectives name scoping, including acceptance
// testing its interaction with SceneGraph.
GTEST_TEST(ProcessModelDirectivesTest, AddScopedSmokeTest) {
  ModelDirectives directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) + "/add_scoped_top.yaml"));

  // Ensure that we have a SceneGraph present so that we test relevant visual
  // pieces.
  DiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = AddMultibodyPlantSceneGraph(&builder, 0.);
  ProcessModelDirectives(directives, &plant,
                         nullptr, make_parser(&plant).get());
  plant.Finalize();
  auto diagram = builder.Build();

  // Helper lambda for checking existence of frame in model scope.
  auto check_frame = [&plant](
      const std::string instance, const std::string frame) {
    const std::string scoped_frame = instance + "::" + frame;
    drake::log()->debug("Check: {}", scoped_frame);
    EXPECT_TRUE(
        plant.HasFrameNamed(frame, plant.GetModelInstanceByName(instance)));
  };
  // Query information and ensure we have expected results.
  for (const std::string prefix : {"", "left::", "right::", "mid::nested::"}) {
    const std::string simple_model = prefix + "simple_model";
    check_frame(simple_model, "base");
    check_frame(simple_model, "frame");
    check_frame(simple_model, "sub_added_frame");
    check_frame(simple_model, "top_added_frame");
    const std::string extra_model = prefix + "extra_model";
    check_frame(extra_model, "base");
    check_frame(extra_model, "frame");
  }
  // - Checking simple_model_test_frame frames that have model namespaces.
  for (const std::string model_namespace : {"left", "right", "mid::nested"}) {
    check_frame(model_namespace, "simple_model_test_frame");
  }
  // - Checking for simple_model_test_frame that was added without a model
  // namespace. This frame was added without a model namespace, but ties itself
  // to the model namespace of its base frame instead of the world. See next
  // test, AddFrameWithoutScope, for a more concrete example.
  check_frame("simple_model", "simple_model_test_frame");
}

// Tests for frames added without a model name, but different base_frame.
GTEST_TEST(ProcessModelDirectivesTest, AddFrameWithoutScope) {
  ModelDirectives directives = LoadModelDirectives(
      FindResourceOrThrow(
          std::string(kTestDir) + "/add_frame_without_model_namespace.yaml"));

  // Ensure that we have a SceneGraph present so that we test relevant visual
  // pieces.
  DiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = AddMultibodyPlantSceneGraph(&builder, 0.);
  ProcessModelDirectives(directives, &plant,
                         nullptr, make_parser(&plant).get());
  plant.Finalize();
  auto diagram = builder.Build();

  // When a frame is added without a namespace, it will scope itself under the
  // base_frame's model instance.

  // Frame added with world as base frame.
  EXPECT_TRUE(
      plant.HasFrameNamed("world_as_base_frame", world_model_instance()));

  // Frame added with included model as base frame.
  auto simple_model_instance = plant.GetModelInstanceByName("simple_model");
  EXPECT_TRUE(
      plant.HasFrameNamed("included_as_base_frame", simple_model_instance));
}

// Test backreference behavior in ModelDirectives.
GTEST_TEST(ProcessModelDirectivesTest, TestBackreferences) {
  ModelDirectives directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) + "/test_backreferences.yaml"));

  // Ensure that we have a SceneGraph present so that we test relevant visual
  // pieces.
  DiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = AddMultibodyPlantSceneGraph(&builder, 0.);
  ProcessModelDirectives(directives, &plant,
                         nullptr, make_parser(&plant).get());
  plant.Finalize();
  auto diagram = builder.Build();

  // Weld joint for the model without a namespace is placed under simple_model
  // instead of world.
  EXPECT_TRUE(plant.HasJointNamed(
      "simple_model_origin_welds_to_base",
      plant.GetModelInstanceByName("simple_model")));

  // Weld joint for the nested model.
  EXPECT_TRUE(plant.HasJointNamed(
      "simple_model_origin_welds_to_base",
      plant.GetModelInstanceByName("nested::simple_model")));
}

// Test frame injection in ModelDirectives.
GTEST_TEST(ProcessModelDirectivesTest, InjectFrames) {
  ModelDirectives directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) + "/inject_frames.yaml"));

  // Ensure that we have a SceneGraph present so that we test relevant visual
  // pieces.
  DiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = AddMultibodyPlantSceneGraph(&builder, 0.);
  ProcessModelDirectives(directives, &plant,
                         nullptr, make_parser(&plant).get());
  plant.Finalize();
  auto diagram = builder.Build();
  auto context = plant.CreateDefaultContext();

  // Check that injected frames exist.
  EXPECT_TRUE(plant.HasFrameNamed(
      "top_injected_frame", plant.GetModelInstanceByName("top_level_model")));
  EXPECT_TRUE(plant.HasFrameNamed(
      "base", plant.GetModelInstanceByName("mid_level_model")));

  // Check for pose of welded models' base.
  EXPECT_TRUE(plant
      .GetFrameByName("base", plant.GetModelInstanceByName("mid_level_model"))
      .CalcPoseInWorld(*context)
      .translation()
      .isApprox(Vector3d(1, 2, 3)));
  EXPECT_TRUE(plant
      .GetFrameByName("base",
                      plant.GetModelInstanceByName("bottom_level_model"))
      .CalcPoseInWorld(*context)
      .translation()
      .isApprox(Vector3d(2, 4, 6)));
}

// Make sure we have good error messages.
GTEST_TEST(ProcessModelDirectivesTest, ErrorMessages) {
  // When the user gives a bogus filename, at minimum we must echo it back to
  // them so they know what failed.
  DRAKE_EXPECT_THROWS_MESSAGE(
      LoadModelDirectives("no-such-file.yaml"),
      ".*no-such-file.yaml.*");
}

}  // namespace
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
