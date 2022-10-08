#include "drake/multibody/parsing/parser.h"

#include <fstream>
#include <sstream>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace {

std::string ReadEntireFile(const std::string& file_name) {
  std::ifstream input(file_name);
  DRAKE_DEMAND(input.good());
  std::stringstream result;
  result << input.rdbuf();
  return result.str();
}

GTEST_TEST(FileParserTest, BasicTest) {
  const std::string sdf_name = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/acrobot.sdf");
  const std::string urdf_name = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/acrobot.urdf");
  const std::string xml_name = FindResourceOrThrow(
      "drake/multibody/parsing/dm_control/suite/acrobot.xml");
  const std::string dmd_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/process_model_directives_test/"
      "acrobot.dmd.yaml");

  // Load from SDF using plural method.
  // Add a second one with an overridden model_name.
  {
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    EXPECT_EQ(&dut.plant(), &plant);
    EXPECT_EQ(dut.AddAllModelsFromFile(sdf_name).size(), 1);
    dut.AddModelFromFile(sdf_name, "foo");
  }

  // Load from URDF using plural method.
  // Add a second one with an overridden model_name.
  {
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    EXPECT_EQ(dut.AddAllModelsFromFile(urdf_name).size(), 1);
    dut.AddModelFromFile(urdf_name, "foo");
  }

  // Load an SDF then a URDF.
  {
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    dut.AddModelFromFile(sdf_name, "foo");
    dut.AddModelFromFile(urdf_name, "bar");
  }

  // Load from XML using plural method.
  // Add a second one with an overridden model_name.
  {
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    const std::vector<ModelInstanceIndex> ids =
        dut.AddAllModelsFromFile(xml_name);
    EXPECT_EQ(ids.size(), 1);
    EXPECT_EQ(plant.GetModelInstanceName(ids[0]), "acrobot");
    const ModelInstanceIndex id = dut.AddModelFromFile(xml_name, "foo");
    EXPECT_EQ(plant.GetModelInstanceName(id), "foo");
  }

  // Load from DMD using plural method.
  // Using the singular method is always an error.
  {
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    const std::vector<ModelInstanceIndex> ids =
        dut.AddAllModelsFromFile(dmd_name);
    EXPECT_EQ(ids.size(), 1);
    EXPECT_EQ(plant.GetModelInstanceName(ids[0]), "acrobot");
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut.AddModelFromFile(dmd_name, "foo"),
        ".* always an error.*");
  }
}

GTEST_TEST(FileParserTest, BasicStringTest) {
  const std::string sdf_name = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/acrobot.sdf");
  const std::string urdf_name = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/acrobot.urdf");
  const std::string xml_name = FindResourceOrThrow(
      "drake/multibody/parsing/dm_control/suite/acrobot.xml");
  const std::string dmd_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/process_model_directives_test/"
      "acrobot.dmd.yaml");

  // Load an SDF via string.
  {
    const std::string sdf_contents = ReadEntireFile(sdf_name);
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    const ModelInstanceIndex id = dut.AddModelFromString(sdf_contents, "sdf");
    EXPECT_EQ(plant.GetModelInstanceName(id), "acrobot");
  }

  // Load an URDF via string.
  {
    const std::string urdf_contents = ReadEntireFile(urdf_name);
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    const ModelInstanceIndex id = dut.AddModelFromString(urdf_contents, "urdf");
    EXPECT_EQ(plant.GetModelInstanceName(id), "acrobot");
  }

  // Load an MJCF via string.
  {
    const std::string xml_contents = ReadEntireFile(xml_name);
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    const ModelInstanceIndex id = dut.AddModelFromString(xml_contents, "xml");
    EXPECT_EQ(plant.GetModelInstanceName(id), "acrobot");
  }

  // Load a DMD.YAML via string.
  // Using the singular method is always an error.
  {
    const std::string dmd_contents = ReadEntireFile(dmd_name);
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut.AddModelFromString(dmd_contents, "dmd.yaml"),
        ".* always an error.*");
  }
}

// Try loading a file with two <model> elements, but without a <world>.
// This should always result in an error. For an example of a valid <world>
// with two <model> elements, refer to MultiModelViaWorldIncludesTest.
GTEST_TEST(FileParserTest, MultiModelErrorsTest) {
  const std::string sdf_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/two_models.sdf");

  // Check the plural method.
  {
    MultibodyPlant<double> plant(0.0);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&plant).AddAllModelsFromFile(sdf_name),
        R"([\s\S]*Root object can only contain one model.*)");
  }

  // The singular method cannot load a two-model file.
  const char* const expected_error =
        R"([\s\S]*Root object can only contain one model.*)";

  // Check the singular method without model_name.
  {
    MultibodyPlant<double> plant(0.0);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&plant).AddModelFromFile(sdf_name),
        expected_error);
  }

  // Check the singular method with empty model_name.
  {
    MultibodyPlant<double> plant(0.0);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&plant).AddModelFromFile(sdf_name, ""),
        expected_error);
  }

  // Check the singular method with non-empty model_name.
  {
    MultibodyPlant<double> plant(0.0);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&plant).AddModelFromFile(sdf_name, "foo"),
        expected_error);
  }
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
      Parser(&plant).AddAllModelsFromFile(sdf_name);
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
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModelFromFile("acrobot.foo"),
      ".*file.*\\.foo.* is not.*recognized.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddAllModelsFromFile("acrobot.foo"),
      ".*file.*\\.foo.* is not.*recognized.*");

  // Uppercase extensions are accepted (i.e., still call the underlying SDF or
  // URDF parser, shown here by it generating a different exception message).
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModelFromFile("acrobot.SDF"),
      ".*Unable to read file.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModelFromFile("acrobot.URDF"),
      "/.*/acrobot.URDF:0: error: "
      "Failed to parse XML file: XML_ERROR_FILE_NOT_FOUND");
}

GTEST_TEST(FileParserTest, BadStringTest) {
  // Malformed SDF string is an error.
  MultibodyPlant<double> plant(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModelFromString("bad", "sdf"),
      ".*Unable to read SDF string.*");

  // Malformed URDF string is an error.
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModelFromString("bad", "urdf"),
      "<literal-string>.urdf:1: error: "
      "Failed to parse XML string: XML_ERROR_PARSING_TEXT");

  // Malformed Mujoco string is an error.
  // TODO(#18055): Until the underlying parser supports diagnostic policy, the
  // error message matching here will be less than convincing.
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModelFromString("bad", "xml"),
      "Failed to parse XML string: XML_ERROR_PARSING_TEXT");

  // Unknown extension is an error.
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModelFromString("<bad/>", "weird-ext"),
      ".*file.*\\.weird-ext.* is not.*recognized.*");
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
  filesystem::create_directory({sdf_path});
  filesystem::create_directory({mesh_path});
  filesystem::copy(full_package_filename, temp_dir + "/package.xml");
  filesystem::copy(full_sdf_filename, sdf_path + "/box.sdf");
  filesystem::copy(full_obj_filename, mesh_path + "/box.obj");

  // Attempt to read in the SDF file without setting the package map first.
  const std::string new_sdf_filename = sdf_path + "/box.sdf";
  DRAKE_EXPECT_THROWS_MESSAGE(parser.AddModelFromFile(new_sdf_filename),
      "error.*unknown package.*box_model.*");

  // Try again.
  parser.package_map().PopulateFromFolder(temp_dir);
  parser.AddModelFromFile(new_sdf_filename, "dummy" /* model name */);
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

  MultibodyPlant<double> plant(0.0);
  geometry::SceneGraph<double> scene_graph;
  Parser parser(&plant, &scene_graph);

  EXPECT_NO_THROW(
      parser.AddModelFromString(model_provokes_warning, "urdf", "lax"));

  parser.SetStrictParsing();
  DRAKE_EXPECT_THROWS_MESSAGE(
      parser.AddModelFromString(model_provokes_warning, "urdf", "strict"),
      warning_pattern);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
