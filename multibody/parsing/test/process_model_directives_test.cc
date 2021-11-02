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

  // Query information and ensure we have expected results.
  // - Manually spell out one example.
  ASSERT_EQ(
      &GetScopedFrameByName(plant, "left::simple_model::frame"),
      &plant.GetFrameByName(
          "frame", plant.GetModelInstanceByName("left::simple_model")));
  // - Automate other stuff.
  auto check_frame = [&plant](
      const std::string instance, const std::string frame) {
    const std::string scoped_frame = instance + "::" + frame;
    drake::log()->debug("Check: {}", scoped_frame);
    ASSERT_EQ(
        &GetScopedFrameByName(plant, scoped_frame),
        &plant.GetFrameByName(frame, plant.GetModelInstanceByName(instance)));
  };
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
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Test the model error mechanism.
GTEST_TEST(ProcessModelDirectivesTest, SmokeTestInjectWeldError) {
  const RigidTransformd error_transform({0.1, 0., 0.1}, {2, 3, 4});
  ModelDirectives directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) + "/add_scoped_sub.yaml"));

  // This error function should add model error to exactly one weld, the
  // attachment of the `first_instance` sdf model to the `smoke_test_origin`
  // frame.
  MultibodyPlant<double> plant(0.0);

  auto error = [&](const std::string& parent, const std::string& child) {
    const std::string error_parent = "simple_model::frame";
    const std::string error_child = "extra_model::base";
    optional<RigidTransformd> out;
    if (parent == error_parent && child == error_child)
        out = error_transform;
    return out;
  };

  ProcessModelDirectives(directives, &plant,
                         nullptr, make_parser(&plant).get(), error);
  plant.Finalize();

  // This should have created an error frame for the relevant weld.
  const std::string expected_error_frame_name = "frame_weld_error_to_base";
  EXPECT_TRUE(plant.HasFrameNamed(expected_error_frame_name));
  const auto& frame = plant.GetFrameByName(expected_error_frame_name);
  EXPECT_TRUE(
      dynamic_cast<const drake::multibody::FixedOffsetFrame<double>*>(&frame));
  const RigidTransformd expected_error =
      (plant
       .GetFrameByName("frame", plant.GetModelInstanceByName("simple_model"))
       .GetFixedPoseInBodyFrame())
      * error_transform;

  EXPECT_TRUE(
      frame.GetFixedPoseInBodyFrame().IsExactlyEqualTo(expected_error));

  // This should not have created an error frame for other welds.
  for (drake::multibody::FrameIndex frame_id(0);
       frame_id < plant.num_frames();
       frame_id++) {
    const std::string frame_name = plant.get_frame(frame_id).name();
    if (frame_name != expected_error_frame_name) {
      EXPECT_TRUE(frame_name.find("error") == std::string::npos);
    }
  }
}
#pragma GCC diagnostic pop

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
