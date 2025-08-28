#include "drake/multibody/parsing/parser.h"

#include <filesystem>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/planning/robot_diagram_builder.h"

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(FileParserTest, BasicTest) {
  const std::string sdf_name =
      FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.sdf");
  const std::string urdf_name =
      FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.urdf");
  const std::string xml_name = FindResourceOrThrow(
      "drake/multibody/parsing/dm_control/suite/acrobot.xml");
  const std::string dmd_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/process_model_directives_test/"
      "acrobot.dmd.yaml");
  const std::string obj_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/meshes/box.obj");

  // Load from SDFormat.
  // Load the same model again with a name prefix.
  {
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    EXPECT_EQ(&dut.plant(), &plant);
    EXPECT_EQ(dut.scene_graph(), nullptr);
    EXPECT_EQ(dut.builder(), nullptr);
    EXPECT_EQ(dut.AddModels(sdf_name).size(), 1);
    const auto prefix_ids = Parser(&plant, "prefix").AddModels(sdf_name);
    EXPECT_EQ(prefix_ids.size(), 1);
    EXPECT_EQ(plant.GetModelInstanceName(prefix_ids[0]), "prefix::acrobot");
  }

  // Load from URDF.
  // Load the same model again with a name prefix.
  {
    MultibodyPlant<double> plant(0.0);
    geometry::SceneGraph<double> scene_graph;
    Parser dut(&plant, &scene_graph);
    EXPECT_EQ(&dut.plant(), &plant);
    EXPECT_EQ(dut.scene_graph(), &scene_graph);
    EXPECT_EQ(dut.builder(), nullptr);
    EXPECT_EQ(dut.AddModels(urdf_name).size(), 1);
    const auto prefix_ids = Parser(&plant, "prefix").AddModels(urdf_name);
    EXPECT_EQ(prefix_ids.size(), 1);
    EXPECT_EQ(plant.GetModelInstanceName(prefix_ids[0]), "prefix::acrobot");
  }

  // Load from Mujoco XML.
  // Load the same model again with a name prefix.
  {
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    const std::vector<ModelInstanceIndex> ids = dut.AddModels(xml_name);
    EXPECT_EQ(ids.size(), 1);
    EXPECT_EQ(plant.GetModelInstanceName(ids[0]), "acrobot");
    const auto prefix_ids = Parser(&plant, "prefix").AddModels(xml_name);
    EXPECT_EQ(prefix_ids.size(), 1);
    EXPECT_EQ(plant.GetModelInstanceName(prefix_ids[0]), "prefix::acrobot");
  }

  // Load from DMD.
  // Load the same model again with a name prefix.
  {
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    const std::vector<ModelInstanceIndex> ids = dut.AddModels(dmd_name);
    EXPECT_EQ(ids.size(), 1);
    EXPECT_EQ(plant.GetModelInstanceName(ids[0]), "acrobot");
    const auto prefix_ids = Parser(&plant, "prefix").AddModels(dmd_name);
    EXPECT_EQ(prefix_ids.size(), 1);
    EXPECT_EQ(plant.GetModelInstanceName(prefix_ids[0]), "prefix::acrobot");
  }

  // Load from OBJ.
  // Load the same model again with a name prefix.
  {
    // TODO(SeanCurtis-TRI): Break this "basic test" up into each extension
    // type. The shared infrastructure is negligible, but the cost of adding
    // a new extension is reduced by having each extension isolated in its own
    // test.
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    const std::vector<ModelInstanceIndex> ids = dut.AddModels(obj_name);
    EXPECT_EQ(ids.size(), 1);
    EXPECT_EQ(plant.GetModelInstanceName(ids[0]), "box");
    const auto prefix_ids = Parser(&plant, "prefix").AddModels(obj_name);
    EXPECT_EQ(prefix_ids.size(), 1);
    EXPECT_EQ(plant.GetModelInstanceName(prefix_ids[0]), "prefix::box");
  }
}

GTEST_TEST(FileParserTest, BuilderTest) {
  const std::string sdf_name =
      FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.sdf");

  {  // Pass the builder only (using RobotDiagramBuilder).
    planning::RobotDiagramBuilder<double> builder(0.0);
    Parser dut(&builder.builder());
    EXPECT_EQ(dut.builder(), &builder.builder());
    EXPECT_EQ(&dut.plant(), &builder.plant());
    EXPECT_EQ(dut.scene_graph(), &builder.scene_graph());
    auto ids = dut.AddModels(sdf_name);
    EXPECT_EQ(ids.size(), 1);
    EXPECT_EQ(builder.plant().GetModelInstanceName(ids[0]), "acrobot");
  }

  {  // Pass the builder only (the Diagram has only the plant).
    systems::DiagramBuilder<double> builder;
    auto plant = builder.AddSystem<MultibodyPlant<double>>(0.0);
    plant->set_name("plant");
    Parser dut(&builder);
    EXPECT_EQ(dut.builder(), &builder);
    EXPECT_EQ(&dut.plant(), plant);
    EXPECT_EQ(dut.scene_graph(), nullptr);
    auto ids = dut.AddModels(sdf_name);
    EXPECT_EQ(ids.size(), 1);
    EXPECT_EQ(plant->GetModelInstanceName(ids[0]), "acrobot");
  }

  {  // The Diagram has a plant named "random", but no SceneGraph. Passing just
     // the builder fails but if we pass the plant pointer it succeeds.
    systems::DiagramBuilder<double> builder;
    auto plant = builder.AddSystem<MultibodyPlant<double>>(0.0);
    plant->set_name("random");
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&builder),
        "DiagramBuilder does not contain a subsystem named plant.*");
    Parser dut(&builder, plant);
    EXPECT_EQ(dut.builder(), &builder);
    EXPECT_EQ(&dut.plant(), plant);
    EXPECT_EQ(dut.scene_graph(), nullptr);
    auto ids = dut.AddModels(sdf_name);
    EXPECT_EQ(ids.size(), 1);
    EXPECT_EQ(plant->GetModelInstanceName(ids[0]), "acrobot");
  }

  {  // The Diagram has a plant and scene_graph with non-default names. Passing
     // just the builder and scene_graph fails but if we pass the plant pointer
     // it succeeds.
    systems::DiagramBuilder<double> builder;
    auto plant = builder.AddSystem<MultibodyPlant<double>>(0.0);
    auto scene_graph = builder.AddSystem<geometry::SceneGraph<double>>();
    plant->set_name("random");
    scene_graph->set_name("arbitrary");
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&builder, nullptr, scene_graph),
        "DiagramBuilder does not contain a subsystem named plant.*");
    Parser dut(&builder, plant, scene_graph);
    EXPECT_EQ(dut.builder(), &builder);
    EXPECT_EQ(&dut.plant(), plant);
    EXPECT_EQ(dut.scene_graph(), scene_graph);
    EXPECT_EQ(plant->geometry_source_is_registered(), true);
    auto ids = dut.AddModels(sdf_name);
    EXPECT_EQ(ids.size(), 1);
    EXPECT_EQ(plant->GetModelInstanceName(ids[0]), "acrobot");
  }
}

// Load from SDF using a PackageMap URL.
GTEST_TEST(FileParserTest, UrlTest) {
  MultibodyPlant<double> plant(0.0);
  Parser dut(&plant);
  auto models = dut.AddModelsFromUrl(
      "package://drake/multibody/benchmarks/acrobot/acrobot.sdf");
  EXPECT_EQ(models.size(), 1);
  EXPECT_EQ(plant.GetModelInstanceName(models.at(0)), "acrobot");

  // Check the error message for unsupported schemes.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.AddModelsFromUrl("ftp://l33t.w4r3z/acrobotz.sdf"),
      ".*unsupported scheme.*");
}

GTEST_TEST(FileParserTest, BasicStringTest) {
  const std::string sdf_name =
      FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.sdf");
  const std::string urdf_name =
      FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.urdf");
  const std::string xml_name =
      FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.xml");
  const std::string dmd_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/process_model_directives_test/"
      "acrobot.dmd.yaml");

  // Load an SDF via string using plural method.
  {
    const std::string sdf_contents = ReadFileOrThrow(sdf_name);
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    const std::vector<ModelInstanceIndex> ids =
        dut.AddModelsFromString(sdf_contents, "sdf");
    EXPECT_EQ(ids.size(), 1);
    EXPECT_EQ(plant.GetModelInstanceName(ids[0]), "acrobot");
  }

  // Load an URDF via string using plural method.
  {
    const std::string urdf_contents = ReadFileOrThrow(urdf_name);
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    const std::vector<ModelInstanceIndex> ids =
        dut.AddModelsFromString(urdf_contents, "urdf");
    EXPECT_EQ(ids.size(), 1);
    EXPECT_EQ(plant.GetModelInstanceName(ids[0]), "acrobot");
  }

  // Load an MJCF via string using plural method.
  {
    const std::string xml_contents = ReadFileOrThrow(xml_name);
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    const std::vector<ModelInstanceIndex> ids =
        dut.AddModelsFromString(xml_contents, "xml");
    EXPECT_EQ(ids.size(), 1);
    EXPECT_EQ(plant.GetModelInstanceName(ids[0]), "acrobot");
  }

  // Load a DMD.YAML via string using plural method.
  {
    const std::string dmd_contents = ReadFileOrThrow(dmd_name);
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    const std::vector<ModelInstanceIndex> ids =
        dut.AddModelsFromString(dmd_contents, "dmd.yaml");
    EXPECT_EQ(ids.size(), 1);
    EXPECT_EQ(plant.GetModelInstanceName(ids[0]), "acrobot");
  }
}

// Try loading a file with two <model> elements, but without a <world>.
// This should always result in an error. For an example of a valid <world>
// with two <model> elements, refer to MultiModelViaWorldIncludesTest.
GTEST_TEST(FileParserTest, MultiModelErrorsTest) {
  const std::string sdf_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/two_models.sdf");
  MultibodyPlant<double> plant(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModels(sdf_name),
      R"([\s\S]*Root object can only contain one model.*)");
}

std::vector<std::string> GetModelInstanceNames(
    const MultibodyPlant<double>& plant,
    const std::vector<ModelInstanceIndex>& models) {
  std::vector<std::string> names;
  for (auto model : models) {
    names.push_back(plant.GetModelInstanceName(model));
  }
  return names;
}

GTEST_TEST(FileParserTest, MultiModelViaWorldIncludesTest) {
  const std::string sdf_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "world_with_directly_nested_models.sdf");
  MultibodyPlant<double> plant(0.0);
  const std::vector<ModelInstanceIndex> models =
      Parser(&plant).AddModels(sdf_name);
  const std::vector<std::string> model_names_actual =
      GetModelInstanceNames(plant, models);
  const std::vector<std::string> model_names_expected = {
      "parent_model",
      "parent_model::robot1",
      "parent_model::robot2",
  };
  EXPECT_EQ(model_names_actual, model_names_expected);
}

GTEST_TEST(FileParserTest, ExtensionMatchTest) {
  // An unknown extension is an error.
  // (Check both singular and plural overloads.)
  MultibodyPlant<double> plant(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(Parser(&plant).AddModels("acrobot.foo"),
                              ".*file.*\\.foo.* is not.*recognized.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Parser(&plant).AddModels("acrobot.foo"),
                              ".*file.*\\.foo.* is not.*recognized.*");

  // Uppercase extensions are accepted (i.e., still call the underlying SDF or
  // URDF parser, shown here by it generating a different exception message).
  DRAKE_EXPECT_THROWS_MESSAGE(Parser(&plant).AddModels("acrobot.SDF"),
                              "error: Error finding file.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModels("acrobot.URDF"),
      "/.*/acrobot.URDF:0: error: "
      "Failed to parse XML file: XML_ERROR_FILE_NOT_FOUND");
}

GTEST_TEST(FileParserTest, BadStringTest) {
  // Malformed SDF string is an error.
  MultibodyPlant<double> plant(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModelsFromString("bad", "sdf"),
      ".*Error parsing XML from string: Error=XML_ERROR_PARSING_TEXT.*");

  // Malformed URDF string is an error.
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModelsFromString("bad", "urdf"),
      "<literal-string>.urdf:1: error: "
      "Failed to parse XML string: XML_ERROR_PARSING_TEXT");

  // Malformed Mujoco string is an error.
  // TODO(#18055): Until the underlying parser supports diagnostic policy, the
  // error message matching here will be less than convincing.
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModelsFromString("bad", "xml"),
      ".*Failed to parse XML string: XML_ERROR_PARSING_TEXT");

  // Malformed DMD string is an error.
  // TODO(#18052): Until the underlying parser supports diagnostic policy, the
  // input needs to crafted to avoid reachable fatal assertions.
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModelsFromString("bad:", "dmd.yaml"), ".*YAML.*bad.*");

  // Syntactically well-formed DMD data, but semantically invalid.
  {
    // N.B. This directive is missing the required `name` attribute.
    constexpr char yaml[] =
        "directives:\n"
        "- add_model_instance:\n";
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&plant).AddModelsFromString(yaml, "dmd.yaml"),
        ".*IsValid.*failed.*");
  }

  // Unknown extension is an error.
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModelsFromString("<bad/>", "weird-ext"),
      ".*file.*\\.weird-ext.* is not.*recognized.*");
}

// Internally, all of the constructors forward to the same implementation. This
// test just confirms support for the various call signatures involving a model
// name prefix.
GTEST_TEST(FileParserTest, PrefixConstructors) {
  std::string model = "<robot name='r'><link name='a'/></robot>";
  MultibodyPlant<double> plant(0.0);
  geometry::SceneGraph<double> scene_graph;

  EXPECT_NO_THROW(Parser(&plant).AddModelsFromString(model, "urdf"));

  // Reload the same model via a different parser constructor. Catch the name
  // collision.
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant, &scene_graph).AddModelsFromString(model, "urdf"),
      ".*names must be unique.*");

  // Reload the same model, but use model_name_prefix to avoid name collisions.
  EXPECT_NO_THROW(Parser(&plant, "prefix1").AddModelsFromString(model, "urdf"));
  EXPECT_NO_THROW(Parser(&plant, &scene_graph, "prefix2")
                      .AddModelsFromString(model, "urdf"));
}

// If a non-Drake URDF or SDF file uses package URIs, this confirms that it is
// necessary to explicitly include the package in order to resolve the URIs.
GTEST_TEST(FileParserTest, PackageMapTest) {
  // We start with the world and default model instances (model_instance.h
  // explains why there are two).
  MultibodyPlant<double> plant(0.0);
  geometry::SceneGraph<double> scene_graph;
  Parser parser(&plant, &scene_graph);
  ASSERT_EQ(plant.num_model_instances(), 2);

  // Get the temporary directory.
  std::string temp_dir = temp_directory();

  // Get the SDF and mesh files.
  const std::string full_package_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/package.xml");
  const std::string full_sdf_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/sdfs/box.sdf");
  const std::string full_obj_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/meshes/box.obj");

  // Copy the relevant files to the temporary directory.
  const std::string sdf_path = temp_dir + "/sdfs";
  const std::string mesh_path = temp_dir + "/meshes";
  std::filesystem::create_directory({sdf_path});
  std::filesystem::create_directory({mesh_path});
  std::filesystem::copy(full_package_filename, temp_dir + "/package.xml");
  std::filesystem::copy(full_sdf_filename, sdf_path + "/box.sdf");
  std::filesystem::copy(full_obj_filename, mesh_path + "/box.obj");

  // Attempt to read in the SDF file without setting the package map first.
  const std::string new_sdf_filename = sdf_path + "/box.sdf";
  DRAKE_EXPECT_THROWS_MESSAGE(parser.AddModels(new_sdf_filename),
                              ".*error.*unknown package.*box_model.*");

  // Move the failed parse out of the way.
  plant.RenameModelInstance(plant.GetModelInstanceByName("box"), "broken");

  // Try again.
  parser.package_map().PopulateFromFolder(temp_dir);
  parser.AddModels(new_sdf_filename);
}

GTEST_TEST(FileParserTest, StrictParsing) {
  // If the choice of what causes warnings changes, this test data will need to
  // be updated. In this incarnation, the /robot/@version attribute provokes a
  // warning because it is ignored.
  std::string model_provokes_warning = R"""(
    <robot name='robot' version='0.99'>
      <link name='a'/>
    </robot>)""";
  std::string warning_pattern = ".*version.*ignored.*";

  {
    // Lax parser does not throw on warnings.
    MultibodyPlant<double> plant(0.0);
    geometry::SceneGraph<double> scene_graph;
    Parser parser(&plant, &scene_graph);
    EXPECT_NO_THROW(parser.AddModelsFromString(model_provokes_warning, "urdf"));
  }

  {
    // Strict parser *does* throw on warnings.
    MultibodyPlant<double> plant(0.0);
    geometry::SceneGraph<double> scene_graph;
    Parser parser(&plant, &scene_graph);
    parser.SetStrictParsing();
    DRAKE_EXPECT_THROWS_MESSAGE(
        parser.AddModelsFromString(model_provokes_warning, "urdf"),
        warning_pattern);
  }
}

GTEST_TEST(FileParserTest, AutoRenaming) {
  std::string model = "<robot name='robot'><link name='a'/></robot>";
  MultibodyPlant<double> plant(0.0);

  Parser parser(&plant);
  EXPECT_FALSE(parser.GetAutoRenaming());

  // Load a model.
  parser.AddModelsFromString(model, "urdf");
  EXPECT_TRUE(plant.HasModelInstanceNamed("robot"));
  // Auto renaming is off; fail to load it again.
  DRAKE_EXPECT_THROWS_MESSAGE(parser.AddModelsFromString(model, "urdf"),
                              ".*names must be unique.*");

  // Load it again with auto renaming.
  parser.SetAutoRenaming(true);
  EXPECT_TRUE(parser.GetAutoRenaming());
  parser.AddModelsFromString(model, "urdf");
  EXPECT_TRUE(plant.HasModelInstanceNamed("robot_1"));

  // Disable auto renaming and show repeat loading subsequently fails.
  parser.SetAutoRenaming(false);
  EXPECT_FALSE(parser.GetAutoRenaming());
  DRAKE_EXPECT_THROWS_MESSAGE(parser.AddModelsFromString(model, "urdf"),
                              ".*names must be unique.*");
}

// This is a regression test for issue #21316. The code is adapted from that
// where the problem was originally found.
GTEST_TEST(FileParserTest, InterleavedRenaming) {
  drake::systems::DiagramBuilder<double> builder;
  drake::multibody::MultibodyPlantConfig plant_config;
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(plant_config, &builder);

  // Choose a robot model with collision filters specified.
  auto model_file_url =
      "package://drake_models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf";
  const std::string model_name = "my_kuka_iiwa";

  // Load the same model multiple times.
  int kNumLoads = 2;
  // For the bug to manifest, the parser lifetime must encompass multiple parse
  // and rename steps.
  auto parser = drake::multibody::Parser(&plant);
  for (int k = 0; k < kNumLoads; ++k) {
    // Load the model and give it a name based on its index in the array.
    const std::string model_instance_name = fmt::format("{}{}", model_name, k);
    parser.SetAutoRenaming(true);
    // In the original symptom, the second parse attempt would trigger an
    // assertion.
    auto model_instance = parser.AddModelsFromUrl(model_file_url)[0];
    plant.RenameModelInstance(model_instance, model_instance_name);
  }

  // Expect filter groups from both models, using the model names in force
  // after renaming.
  CollisionFilterGroups expected;
  expected.AddGroup("my_kuka_iiwa0::iiwa_wrist",
                    {"my_kuka_iiwa0::iiwa_link_5", "my_kuka_iiwa0::iiwa_link_6",
                     "my_kuka_iiwa0::iiwa_link_7"});
  expected.AddGroup("my_kuka_iiwa1::iiwa_wrist",
                    {"my_kuka_iiwa1::iiwa_link_5", "my_kuka_iiwa1::iiwa_link_6",
                     "my_kuka_iiwa1::iiwa_link_7"});
  expected.AddExclusionPair(
      {"my_kuka_iiwa0::iiwa_wrist", "my_kuka_iiwa0::iiwa_wrist"});
  expected.AddExclusionPair(
      {"my_kuka_iiwa1::iiwa_wrist", "my_kuka_iiwa1::iiwa_wrist"});
  EXPECT_EQ(parser.GetCollisionFilterGroups(), expected);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
