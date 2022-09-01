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
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace {

using std::optional;
using Eigen::Vector3d;
using drake::geometry::GeometryId;
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

using CollisionPair = SortedPair<std::string>;
void VerifyCollisionFilters(
    const geometry::SceneGraph<double>& scene_graph,
    const std::set<CollisionPair>& expected_filters) {
  const auto& inspector = scene_graph.model_inspector();
  // Get the collision geometry ids.
  const std::vector<GeometryId> all_ids = inspector.GetAllGeometryIds();
  geometry::GeometrySet id_set(all_ids);
  auto collision_id_set = inspector.GetGeometryIds(
      id_set, geometry::Role::kProximity);
  std::vector<GeometryId> ids(collision_id_set.begin(),
                              collision_id_set.end());
  const int num_links = ids.size();
  for (int m = 0; m < num_links; ++m) {
    const std::string& m_name = inspector.GetName(ids[m]);
    for (int n = m + 1; n < num_links; ++n) {
      const std::string& n_name = inspector.GetName(ids[n]);
      CollisionPair names{m_name, n_name};
      SCOPED_TRACE(fmt::format("{} vs {}", names.first(), names.second()));
      auto contains =
          [&expected_filters](const CollisionPair& key) {
            return expected_filters.count(key) > 0;
          };
      EXPECT_EQ(inspector.CollisionFiltered(ids[m], ids[n]),
                contains(names));
    }
  }
}

// Simple smoke test of the most basic model directives.
GTEST_TEST(ProcessModelDirectivesTest, BasicSmokeTest) {
  ModelDirectives station_directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) + "/add_scoped_sub.dmd.yaml"));
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

// Simple smoke test of the simpler function signature.
GTEST_TEST(ProcessModelDirectivesTest, SugarSmokeTest) {
  const ModelDirectives station_directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) + "/add_scoped_sub.dmd.yaml"));

  MultibodyPlant<double> plant(0.0);
  std::vector<ModelInstanceInfo> added_models =
      ProcessModelDirectives(station_directives, make_parser(&plant).get());
  plant.Finalize();

  // Check that the directives were loaded.
  EXPECT_TRUE(plant.HasFrameNamed("sub_added_frame"));

  // Check that the added models were returned.
  ASSERT_EQ(added_models.size(), 2);
  EXPECT_EQ(added_models[0].model_name, "simple_model");
  EXPECT_EQ(added_models[1].model_name, "extra_model");
}

// Acceptance tests for the ModelDirectives name scoping, including acceptance
// testing its interaction with SceneGraph.
GTEST_TEST(ProcessModelDirectivesTest, AddScopedSmokeTest) {
  ModelDirectives directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) + "/add_scoped_top.dmd.yaml"));

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
      FindResourceOrThrow(std::string(kTestDir) +
                          "/add_frame_without_model_namespace.dmd.yaml"));

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
      FindResourceOrThrow(std::string(kTestDir) +
                          "/test_backreferences.dmd.yaml"));

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
      FindResourceOrThrow(std::string(kTestDir) + "/inject_frames.dmd.yaml"));

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
  // This transform includes the translation from the parent frame to the world
  // and from the parent frame to the child frame (via add_weld::X_PC).
  EXPECT_TRUE(plant
      .GetFrameByName("base",
                      plant.GetModelInstanceByName("bottom_level_model"))
      .CalcPoseInWorld(*context)
      .translation()
      .isApprox(Vector3d(6, 9, 12)));
}

// Test collision filter groups in ModelDirectives.
GTEST_TEST(ProcessModelDirectivesTest, CollisionFilterGroupSmokeTest) {
  ModelDirectives directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) +
                          "/collision_filter_group.dmd.yaml"));

  // Ensure that we have a SceneGraph present so that we test relevant visual
  // pieces.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.);
  ProcessModelDirectives(directives, &plant,
                         nullptr, make_parser(&plant).get());

  // Make sure the plant is not finalized such that the Finalize() default
  // filtering has not taken into effect yet. This guarantees that the
  // collision filtering is applied due to the collision filter group parsing.
  ASSERT_FALSE(plant.is_finalized());

  std::set<CollisionPair> expected_filters = {
    // From group 'across_models'.
    {"model1::collision",             "model2::collision"},
    // From group 'nested_members'.
    {"model1::collision",             "nested::sub_model2::collision"},
    // From group 'nested_group'.
    {"model3::collision",             "nested::sub_model1::collision"},
    {"model3::collision",             "nested::sub_model2::collision"},
    // From group 'across_sub_models'.
    {"nested::sub_model1::collision", "nested::sub_model2::collision"},
  };
  VerifyCollisionFilters(scene_graph, expected_filters);
}

// Test collision filter groups in ModelDirectives.
GTEST_TEST(ProcessModelDirectivesTest, DefaultPositions) {
  ModelDirectives directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) +
                          "/default_positions.dmd.yaml"));

  MultibodyPlant<double> plant(0);
  ProcessModelDirectives(directives, &plant, nullptr,
                         make_parser(&plant).get());
  plant.Finalize();

  auto context = plant.CreateDefaultContext();
  const math::RigidTransformd X_WB(
      math::RollPitchYawd(5 * M_PI / 180, 6 * M_PI / 180, 7 * M_PI / 180),
      Eigen::Vector3d(1, 2, 3));
  EXPECT_TRUE(plant.GetFreeBodyPose(*context, plant.GetBodyByName("base"))
                  .IsNearlyEqualTo(X_WB, 1e-14));
  EXPECT_EQ(
      plant.GetJointByName<RevoluteJoint>("ShoulderJoint").get_angle(*context),
      0.1);
  EXPECT_EQ(
      plant.GetJointByName<RevoluteJoint>("ElbowJoint").get_angle(*context),
      0.2);
}

// Make sure we have good error messages.
GTEST_TEST(ProcessModelDirectivesTest, ErrorMessages) {
  // When the user gives a bogus filename, at minimum we must echo it back to
  // them so they know what failed.
  DRAKE_EXPECT_THROWS_MESSAGE(
      LoadModelDirectives("no-such-file.yaml"),
      ".*no-such-file.yaml.*");

  // User specifies a package that hasn't been properly registered.
  {
    MultibodyPlant<double> plant(0.0);
    ModelDirectives directives = LoadModelDirectives(
        FindResourceOrThrow(std::string(kTestDir) +
                            "/bad_package_uri.dmd.yaml"));
    DRAKE_EXPECT_THROWS_MESSAGE(
        ProcessModelDirectives(directives, &plant, nullptr,
                               make_parser(&plant).get()),
        ".*unknown package 'nonexistant'.*");
  }
}

}  // namespace
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
