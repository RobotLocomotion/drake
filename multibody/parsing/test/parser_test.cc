#include "drake/multibody/parsing/parser.h"

#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(FileParserTest, BasicTest) {
  const std::string sdf_name = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/acrobot.sdf");
  const std::string urdf_name = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/acrobot.urdf");

  // Load from SDF using plural method.
  // Add a second one with an overridden model_name.
  {
    MultibodyPlant<double> plant(0.0);
    Parser dut(&plant);
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
}

GTEST_TEST(FileParserTest, MultiModelTest) {
  const std::string sdf_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/two_models.sdf");

  // Check that the plural method loads two models.
  {
    MultibodyPlant<double> plant(0.0);
    EXPECT_EQ(Parser(&plant).AddAllModelsFromFile(sdf_name).size(), 2);
  }

  // The singular method cannot load a two-model file.
  const char* const expected_error =
      "(.*must have a single <model> element.*)";

  // Check the singular method without model_name.
  {
    MultibodyPlant<double> plant(0.0);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&plant).AddModelFromFile(sdf_name),
        std::exception, expected_error);
  }

  // Check the singular method with empty model_name.
  {
    MultibodyPlant<double> plant(0.0);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&plant).AddModelFromFile(sdf_name, ""),
        std::exception, expected_error);
  }

  // Check the singular method with non-empty model_name.
  {
    MultibodyPlant<double> plant(0.0);
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&plant).AddModelFromFile(sdf_name, "foo"),
        std::exception, expected_error);
  }
}

GTEST_TEST(FileParserTest, ExtensionMatchTest) {
  // An unknown extension is an error.
  // (Check both singular and plural overloads.)
  MultibodyPlant<double> plant(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModelFromFile("acrobot.foo"),
      std::exception,
      ".*file type '\\.foo' is not supported .*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddAllModelsFromFile("acrobot.foo"),
      std::exception,
      ".*file type '\\.foo' is not supported .*");

  // Uppercase extensions are accepted (i.e., still call the underlying SDF or
  // URDF parser, shown here by it generating a different exception message).
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModelFromFile("acrobot.SDF"),
      std::exception,
      ".*does not exist.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      Parser(&plant).AddModelFromFile("acrobot.URDF"),
      std::exception,
      ".*does not exist.*");
}

// If a Drake URDF or SDF file uses package URIs, this confirms that the attempt
// to add the model also loads its package.xml files by side effect.
GTEST_TEST(FileParserTest, FindDrakePackageWhenAdding) {
  using AddFunc = std::function<void(const std::string&, Parser*)>;
  // Function wrappers to facilitate testing all
  // {URDF, SDF} X {AddModel, AddAllModels} combinations.
  AddFunc add_all_models = [](const std::string& file_name, Parser* parser) {
    parser->AddAllModelsFromFile(file_name);
  };
  AddFunc add_model = [](const std::string& file_name, Parser* parser) {
    parser->AddModelFromFile(file_name);
  };

  for (const auto& add_func : {add_all_models, add_model}) {
    for (const auto file_name :
         {"drake/multibody/parsing/test/box_package/sdfs/box.sdf",
          "drake/multibody/parsing/test/box_package/urdfs/box.urdf"}) {
      MultibodyPlant<double> plant(0.0);
      geometry::SceneGraph<double> scene_graph;
      Parser parser(&plant, &scene_graph);
      EXPECT_EQ(parser.package_map().size(), 0);

      // Because the box.sdf references an obj via a package: URI, this would
      // throw if the package were not found.
      EXPECT_NO_THROW(add_func(FindResourceOrThrow(file_name), &parser));

      // Now we explicitly confirm the package map has been modified.
      EXPECT_EQ(parser.package_map().size(), 1);
      EXPECT_TRUE(parser.package_map().Contains("box_model"));
    }
  }
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
      std::runtime_error,
      ".*ERROR: Mesh file name could not be resolved from the provided uri.*");

  // Try again.
  parser.package_map().PopulateFromFolder(temp_dir);
  parser.AddModelFromFile(new_sdf_filename, "dummy" /* model name */);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
