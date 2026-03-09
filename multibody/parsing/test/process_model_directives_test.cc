#include "drake/multibody/parsing/process_model_directives.h"

#include <filesystem>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/text_logging.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace {

using drake::geometry::GeometryId;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using Eigen::Vector3d;
using std::optional;

const char* const kTestDir =
    "drake/multibody/parsing/test/process_model_directives_test";

void add_test_package(PackageMap* package_map) {
  const std::filesystem::path abspath_xml =
      FindResourceOrThrow(std::string(kTestDir) + "/package.xml");
  package_map->AddPackageXml(abspath_xml.string());
}

// Our unit test's package is not normally loaded; construct a parser that
// has it and can resolve package://process_model_directives_test urls.
std::unique_ptr<Parser> make_parser(
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr) {
  auto parser = std::make_unique<Parser>(plant, scene_graph);
  add_test_package(&(parser->package_map()));
  return parser;
}

using CollisionPair = SortedPair<std::string>;
void VerifyCollisionFilters(const geometry::SceneGraph<double>& scene_graph,
                            const std::set<CollisionPair>& expected_filters) {
  const auto& inspector = scene_graph.model_inspector();
  // Get the collision geometry ids.
  const std::vector<GeometryId> all_ids = inspector.GetAllGeometryIds();
  geometry::GeometrySet id_set(all_ids);
  auto collision_id_set =
      inspector.GetGeometryIds(id_set, geometry::Role::kProximity);
  std::vector<GeometryId> ids(collision_id_set.begin(), collision_id_set.end());
  const int num_links = ids.size();
  for (int m = 0; m < num_links; ++m) {
    const std::string& m_name = inspector.GetName(ids[m]);
    for (int n = m + 1; n < num_links; ++n) {
      const std::string& n_name = inspector.GetName(ids[n]);
      CollisionPair names{m_name, n_name};
      SCOPED_TRACE(fmt::format("{} vs {}", names.first(), names.second()));
      auto contains = [&expected_filters](const CollisionPair& key) {
        return expected_filters.contains(key);
      };
      EXPECT_EQ(inspector.CollisionFiltered(ids[m], ids[n]), contains(names));
    }
  }
}

// Simple smoke test of the most basic model directives.
GTEST_TEST(ProcessModelDirectivesTest, BasicSmokeTest) {
  ModelDirectives station_directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) + "/add_scoped_sub.dmd.yaml"));
  const MultibodyPlant<double> empty_plant(0.0);

  MultibodyPlant<double> plant(0.0);
  ProcessModelDirectives(station_directives, &plant, nullptr,
                         make_parser(&plant).get());
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

// Smoke test of the most basic model directives, now loading from string.
GTEST_TEST(ProcessModelDirectivesTest, FromString) {
  ModelDirectives station_directives =
      LoadModelDirectivesFromString(ReadFileOrThrow(FindResourceOrThrow(
          std::string(kTestDir) + "/add_scoped_sub.dmd.yaml")));

  const MultibodyPlant<double> empty_plant(0.0);

  MultibodyPlant<double> plant(0.0);
  ProcessModelDirectives(station_directives, &plant, nullptr,
                         make_parser(&plant).get());
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
  ProcessModelDirectives(directives, &plant, nullptr,
                         make_parser(&plant).get());
  plant.Finalize();
  auto diagram = builder.Build();

  // Helper lambda for checking existence of frame in model scope.
  auto check_frame = [&plant](const std::string instance,
                              const std::string frame) {
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
  ModelDirectives directives = LoadModelDirectives(FindResourceOrThrow(
      std::string(kTestDir) + "/add_frame_without_model_namespace.dmd.yaml"));

  // Ensure that we have a SceneGraph present so that we test relevant visual
  // pieces.
  DiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = AddMultibodyPlantSceneGraph(&builder, 0.);
  ProcessModelDirectives(directives, &plant, nullptr,
                         make_parser(&plant).get());
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

// Tests for rejection of non-deterministic frames.
GTEST_TEST(ProcessModelDirectivesTest, AddStochasticFrameBad) {
  // This nominal case works OK.
  std::string yaml = R"""(
directives:
- add_frame:
    name: my_frame
    X_PF:
      base_frame: world
)""";
  EXPECT_NO_THROW(LoadModelDirectivesFromString(yaml));

  // Setting a stochastic transform yields a failure.
  yaml += "      rotation: !Uniform {}\n";
  DRAKE_EXPECT_THROWS_MESSAGE(LoadModelDirectivesFromString(yaml),
                              ".*IsValid.*");
}

// Test backreference behavior in ModelDirectives.
GTEST_TEST(ProcessModelDirectivesTest, TestBackreferences) {
  ModelDirectives directives = LoadModelDirectives(FindResourceOrThrow(
      std::string(kTestDir) + "/test_backreferences.dmd.yaml"));

  // Ensure that we have a SceneGraph present so that we test relevant visual
  // pieces.
  DiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = AddMultibodyPlantSceneGraph(&builder, 0.);
  ProcessModelDirectives(directives, &plant, nullptr,
                         make_parser(&plant).get());
  plant.Finalize();
  auto diagram = builder.Build();

  // Weld joint for the model without a namespace is placed under simple_model
  // instead of world.
  EXPECT_TRUE(
      plant.HasJointNamed("simple_model_origin_welds_to_base",
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
  ProcessModelDirectives(directives, &plant, nullptr,
                         make_parser(&plant).get());
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
                  .GetFrameByName(
                      "base", plant.GetModelInstanceByName("mid_level_model"))
                  .CalcPoseInWorld(*context)
                  .translation()
                  .isApprox(Vector3d(1, 2, 3)));
  // This transform includes the translation from the parent frame to the world
  // and from the parent frame to the child frame (via add_weld::X_PC).
  EXPECT_TRUE(plant
                  .GetFrameByName("base", plant.GetModelInstanceByName(
                                              "bottom_level_model"))
                  .CalcPoseInWorld(*context)
                  .translation()
                  .isApprox(Vector3d(6, 9, 12)));
}

// Test collision filter groups in ModelDirectives.
GTEST_TEST(ProcessModelDirectivesTest, CollisionFilterGroupSmokeTest) {
  ModelDirectives directives = LoadModelDirectives(FindResourceOrThrow(
      std::string(kTestDir) + "/collision_filter_group.dmd.yaml"));

  // Ensure that we have a SceneGraph present so that we test relevant visual
  // pieces.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.);
  auto parser = make_parser(&plant);
  ProcessModelDirectives(directives, &plant, nullptr, parser.get());

  // Make sure the plant is not finalized such that the Finalize() default
  // filtering has not taken into effect yet. This guarantees that the
  // collision filtering is applied due to the collision filter group parsing.
  ASSERT_FALSE(plant.is_finalized());

  std::set<CollisionPair> expected_filters = {
      // From group 'across_models'.
      {"model1::collision", "model2::collision"},
      // From group 'nested_members'.
      {"model1::collision", "nested::sub_model2::collision"},
      // From group 'nested_group'.
      {"model3::collision", "nested::sub_model1::collision"},
      {"model3::collision", "nested::sub_model2::collision"},
      // From group 'across_sub_models'.
      {"nested::sub_model1::collision", "nested::sub_model2::collision"},
      // From composite group 'group_4567'.
      {"model4::collision", "model5::collision"},
      {"model4::collision", "model6::collision"},
      {"model4::collision", "model7::collision"},
      {"model5::collision", "model6::collision"},
      {"model5::collision", "model7::collision"},
      {"model6::collision", "model7::collision"},
  };
  VerifyCollisionFilters(scene_graph, expected_filters);

  // Verify parser-level collision filter reporting.
  CollisionFilterGroups expected_report;
  expected_report.AddGroup("across_models", {"model1::base", "model2::base"});
  expected_report.AddGroup("group_45", {"model4::base", "model5::base"});
  expected_report.AddGroup("group_4567", {"model4::base", "model5::base",
                                          "model6::base", "model7::base"});
  expected_report.AddGroup("group_67", {"model6::base", "model7::base"});
  expected_report.AddGroup(
      "nested::across_sub_models",
      {"nested::sub_model1::base", "nested::sub_model2::base"});
  expected_report.AddGroup("nested_group", {"model3::base"});
  expected_report.AddGroup("nested_members",
                           {"model1::base", "nested::sub_model2::base"});
  expected_report.AddExclusionPair({"across_models", "across_models"});
  expected_report.AddExclusionPair({"group_4567", "group_4567"});
  expected_report.AddExclusionPair(
      {"nested::across_sub_models", "nested::across_sub_models"});
  expected_report.AddExclusionPair(
      {"nested::across_sub_models", "nested_group"});
  expected_report.AddExclusionPair({"nested_members", "nested_members"});
  EXPECT_EQ(parser->GetCollisionFilterGroups(), expected_report);
}

// Test collision filter groups in ModelDirectives.
GTEST_TEST(ProcessModelDirectivesTest, CollisionFilterGroupNoSceneGraph) {
  ModelDirectives directives = LoadModelDirectives(FindResourceOrThrow(
      std::string(kTestDir) + "/collision_filter_group.dmd.yaml"));

  MultibodyPlant<double> plant(0.0);
  ASSERT_FALSE(plant.geometry_source_is_registered());
  DRAKE_EXPECT_NO_THROW(ProcessModelDirectives(directives, &plant, nullptr,
                                               make_parser(&plant).get()));
}

// Duplicate definition of collision filter groups raises an error.
GTEST_TEST(ProcessModelDirectivesTest, CollisionFilterGroupDuplicateGroups) {
  std::string kDmdYaml = R"""(
directives:

- add_model:
    name: model1
    file: package://process_model_directives_test/simple_model.sdf

- add_model:
    name: model2
    file: package://process_model_directives_test/simple_model.sdf

- add_collision_filter_group:
    name: bad_repeated
    members: [model1::base]
    ignored_collision_filter_groups: [bad_repeated]

- add_collision_filter_group:
    name: bad_repeated
    members: [model2::base]
    ignored_collision_filter_groups: [bad_repeated]
)""";

  ModelDirectives directives = LoadModelDirectivesFromString(kDmdYaml);

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.);
  DRAKE_EXPECT_THROWS_MESSAGE(
      ProcessModelDirectives(directives, &plant, nullptr,
                             make_parser(&plant).get()),
      ".*'bad_repeated'.*already.*defined.*");
}

// Test collision filter groups in ModelDirectives.
GTEST_TEST(ProcessModelDirectivesTest, DefaultPositions) {
  ModelDirectives directives = LoadModelDirectives(FindResourceOrThrow(
      std::string(kTestDir) + "/default_positions.dmd.yaml"));

  MultibodyPlant<double> plant(0);
  ProcessModelDirectives(directives, &plant, nullptr,
                         make_parser(&plant).get());
  plant.Finalize();

  const ModelInstanceIndex simple =
      plant.GetModelInstanceByName("simple_model");
  const ModelInstanceIndex simple_again =
      plant.GetModelInstanceByName("simple_model_again");

  auto context = plant.CreateDefaultContext();
  const math::RigidTransformd X_WB(
      math::RollPitchYawd(5 * M_PI / 180, 6 * M_PI / 180, 7 * M_PI / 180),
      Eigen::Vector3d(1, 2, 3));
  EXPECT_TRUE(CompareMatrices(
      plant.GetFreeBodyPose(*context, plant.GetBodyByName("base", simple))
          .GetAsMatrix34(),
      X_WB.GetAsMatrix34(), 1e-14));
  EXPECT_TRUE(CompareMatrices(
      plant.GetFreeBodyPose(*context, plant.GetBodyByName("base", simple_again))
          .GetAsMatrix34(),
      X_WB.GetAsMatrix34(), 1e-14));
  EXPECT_EQ(
      plant.GetJointByName<RevoluteJoint>("ShoulderJoint").get_angle(*context),
      0.1);
  EXPECT_EQ(
      plant.GetJointByName<RevoluteJoint>("ElbowJoint").get_angle(*context),
      0.2);
}

// Test that collision filter groups can contain both rigid and deformable
// bodies.
GTEST_TEST(ProcessModelDirectivesTest,
           CollisionFilterGroupMixRigidAndDeformable) {
  std::string kDmdYaml = R"""(
directives:

- add_model:
    name: rigid
    file: package://process_model_directives_test/simple_model.sdf

- add_model:
    name: deformable_1
    file: package://process_model_directives_test/deformable_model.sdf

- add_model:
    name: deformable_2
    file: package://process_model_directives_test/deformable_model.sdf

- add_collision_filter_group:
    name: rigid_and_deformable
    members: [rigid::base, deformable_1::body]
    ignored_collision_filter_groups: [rigid_and_deformable]

- add_collision_filter_group:
    name: deformable_and_deformable
    members: [deformable_1::body, deformable_2::body]
    ignored_collision_filter_groups: [deformable_and_deformable]
)""";

  ModelDirectives directives = LoadModelDirectivesFromString(kDmdYaml);

  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.01);
  auto parser = make_parser(&plant);
  ProcessModelDirectives(directives, &plant, nullptr, parser.get());

  // Make sure the plant is not finalized such that the Finalize() default
  // filtering has not taken into effect yet. This guarantees that the
  // collision filtering is applied due to the collision filter group parsing.
  ASSERT_FALSE(plant.is_finalized());

  std::set<CollisionPair> expected_filters = {
      // From group 'rigid_and_deformable'.
      {"rigid::collision", "deformable_1::body"},
      // From group 'deformable_and_deformable'.
      {"deformable_1::body", "deformable_2::body"},
  };
  VerifyCollisionFilters(scene_graph, expected_filters);
}

// Make sure we have good error messages.
GTEST_TEST(ProcessModelDirectivesTest, ErrorMessages) {
  // When the user gives a bogus filename, at minimum we must echo it back to
  // them so they know what failed.
  DRAKE_EXPECT_THROWS_MESSAGE(LoadModelDirectives("no-such-file.yaml"),
                              ".*no-such-file.yaml.*");

  // User specifies a package that hasn't been properly registered.
  {
    MultibodyPlant<double> plant(0.0);
    ModelDirectives directives = LoadModelDirectives(FindResourceOrThrow(
        std::string(kTestDir) + "/bad_package_uri.dmd.yaml"));
    DRAKE_EXPECT_THROWS_MESSAGE(
        ProcessModelDirectives(directives, &plant, nullptr,
                               make_parser(&plant).get()),
        ".*unknown package 'nonexistent'.*");
  }
}

// Test model directives failure to load welds with a deep nested child.
GTEST_TEST(ProcessModelDirectivesTest, DeepNestedChildWelds) {
  ModelDirectives directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) + "/deep_child_weld.dmd.yaml"));
  MultibodyPlant<double> plant(0.0);

  DRAKE_EXPECT_THROWS_MESSAGE(
      ProcessModelDirectives(directives, &plant, nullptr,
                             make_parser(&plant).get()),
      R"(.*Failure at .* in AddWeld\(\): condition 'found' failed.*)");
}

// Test model directives failure to load welds with a child to a
// deep nested frame.
GTEST_TEST(ProcessModelDirectivesTest, DeepNestedChildFrameWelds) {
  ModelDirectives directives = LoadModelDirectives(FindResourceOrThrow(
      std::string(kTestDir) + "/deep_child_frame_weld.dmd.yaml"));
  MultibodyPlant<double> plant(0.0);

  DRAKE_EXPECT_THROWS_MESSAGE(
      ProcessModelDirectives(directives, &plant, nullptr,
                             make_parser(&plant).get()),
      R"(.*Failure at .* in AddWeld\(\): condition 'found' failed.*)");
}

// Test that flattening is idempotent and semantically a no-op.
GTEST_TEST(ProcessModelDirectivesTest, Flatten) {
  std::vector<ModelInstanceInfo> deep_models;
  {
    const ModelDirectives directives = LoadModelDirectives(FindResourceOrThrow(
        std::string(kTestDir) + "/add_scoped_top.dmd.yaml"));
    MultibodyPlant<double> plant(0.0);
    std::unique_ptr<Parser> parser = make_parser(&plant);
    deep_models = ProcessModelDirectives(directives, make_parser(&plant).get());
  }

  std::vector<ModelInstanceInfo> flat_models;
  {
    const ModelDirectives directives = LoadModelDirectives(FindResourceOrThrow(
        std::string(kTestDir) + "/add_scoped_top.dmd.yaml"));
    MultibodyPlant<double> plant(0.0);
    std::unique_ptr<Parser> parser = make_parser(&plant);
    ModelDirectives flat_directives;
    FlattenModelDirectives(directives, parser->package_map(), &flat_directives);
    flat_models = ProcessModelDirectives(flat_directives, parser.get());
  }

  std::vector<ModelInstanceInfo> reflattened_models;
  {
    const ModelDirectives directives = LoadModelDirectives(FindResourceOrThrow(
        std::string(kTestDir) + "/add_scoped_top.dmd.yaml"));
    MultibodyPlant<double> plant(0.0);
    std::unique_ptr<Parser> parser = make_parser(&plant);
    ModelDirectives flat_directives;
    ModelDirectives reflattened_directives;  // To test idempotency.
    FlattenModelDirectives(directives, parser->package_map(), &flat_directives);
    FlattenModelDirectives(flat_directives, parser->package_map(),
                           &reflattened_directives);
    reflattened_models =
        ProcessModelDirectives(reflattened_directives, parser.get());
  }

  // If there were inconsistencies in the scoped names between directives,
  // e.g. frame names referring to nonexistent model scopes, then the
  // `ProcessModelDirectives` call above would have failed.  So we only need
  // to ensure that the models are present with identical names and we can be
  // reasonably sure the other named elements must have been correct.

  // Models from flattened directives have the same names as the originals.
  std::set<std::string> deep_names;
  for (const auto& info : deep_models) {
    deep_names.insert(info.model_name);
  }
  std::set<std::string> flat_names;
  for (const auto& info : flat_models) {
    flat_names.insert(info.model_name);
  }
  EXPECT_EQ(deep_names, flat_names);

  // Repeated flattening makes no difference (idempotency).
  std::set<std::string> reflattened_names;
  for (const auto& info : reflattened_models) {
    reflattened_names.insert(info.model_name);
  }
  EXPECT_EQ(flat_names, reflattened_names);
}

// Test adapted from
// https://github.com/RobotLocomotion/drake/pull/20757#issuecomment-1884115261
// to verify the expected result of flattening two copies of the same file
// containing a collision filter group.
GTEST_TEST(ProcessModelDirectivesTest, FlattenCollisionGroups) {
  PackageMap package_map;
  add_test_package(&package_map);

  // Load the directives hierarchy.
  const ModelDirectives directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) + "/unflattened_top.dmd.yaml"));

  // Flatten the directives hierarchy.
  ModelDirectives flat_directives;
  FlattenModelDirectives(directives, package_map, &flat_directives);

  // Load the expected result of flattening.
  const ModelDirectives expected_directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) + "/flattened.dmd.yaml"));

  // Check that the flattened directives are identical to the expected ones.
  const std::string flattened = yaml::SaveYamlString(flat_directives);
  const std::string expected = yaml::SaveYamlString(expected_directives);
  EXPECT_EQ(flattened, expected);

  // Check that both flattened and expected can be processed.
  {
    MultibodyPlant<double> plant(0.0);
    geometry::SceneGraph<double> scene_graph;
    std::unique_ptr<Parser> parser = make_parser(&plant, &scene_graph);
    std::vector<ModelInstanceInfo> added_models =
        ProcessModelDirectives(flat_directives, parser.get());
    ASSERT_EQ(added_models.size(), 4);
  }
  {
    MultibodyPlant<double> plant(0.0);
    geometry::SceneGraph<double> scene_graph;
    std::unique_ptr<Parser> parser = make_parser(&plant, &scene_graph);
    std::vector<ModelInstanceInfo> added_models =
        ProcessModelDirectives(expected_directives, parser.get());
    ASSERT_EQ(added_models.size(), 4);
  }

  // This test does not directly test collision filtering, as we know it
  // should work because the test dmd.yaml file contains collision filter
  // groups that reference one another by scoped name.  For testing against
  // collision filtering, see CollisionFilterGroupSmokeTest.
}

GTEST_TEST(ProcessModelDirectivesTest, FlattenWithWorld) {
  // Test that frames named "world" are unaffected by the model namespace.
  const std::string kDirectives = R"(
directives:
  - add_directives:
      file: package://process_model_directives_test/add_frame_without_model_namespace.dmd.yaml
      model_namespace: nested
)";
  const ModelDirectives directives = LoadModelDirectivesFromString(kDirectives);
  MultibodyPlant<double> plant(0.0);
  std::unique_ptr<Parser> parser = make_parser(&plant);
  ModelDirectives flat_directives;
  FlattenModelDirectives(directives, parser->package_map(), &flat_directives);
  EXPECT_EQ(
      flat_directives.directives[1].add_frame.value().X_PF.base_frame.value(),
      "world" /* Not "nested::world" */);
}

// Ensure that we incorporate weld offsets into produces model info.
GTEST_TEST(ProcessModelDirectivesTest, WeldOffset) {
  const ModelDirectives directives = LoadModelDirectives(FindResourceOrThrow(
      "drake/manipulation/util/test/panda_arm_and_hand.dmd.yaml"));
  MultibodyPlant<double> plant(0.0);
  std::vector<ModelInstanceInfo> added_models =
      ProcessModelDirectives(directives, make_parser(&plant).get());
  plant.Finalize();
  const std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();

  ASSERT_EQ(added_models.size(), 2);
  EXPECT_EQ(added_models[0].model_name, "panda");
  EXPECT_TRUE(added_models[0].X_PC.IsExactlyIdentity());

  EXPECT_EQ(added_models[1].model_name, "panda_hand");
  EXPECT_FALSE(added_models[1].X_PC.IsExactlyIdentity());

  const Frame<double>& frame_P =
      plant.GetFrameByName(added_models[1].parent_frame_name);
  const Frame<double>& frame_C =
      plant.GetFrameByName(added_models[1].child_frame_name);
  const math::RigidTransformd panda_hand_X_PC_expected =
      plant.CalcRelativeTransform(*context, frame_P, frame_C);
  EXPECT_TRUE(added_models[1].X_PC.IsExactlyEqualTo(panda_hand_X_PC_expected));
}

}  // namespace
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
