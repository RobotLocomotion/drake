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
