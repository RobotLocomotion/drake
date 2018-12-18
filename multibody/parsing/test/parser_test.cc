#include "drake/multibody/parsing/parser.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
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
    MultibodyPlant<double> plant;
    Parser dut(&plant);
    EXPECT_EQ(dut.AddAllModelsFromFile(sdf_name).size(), 1);
    dut.AddModelFromFile(sdf_name, "foo");
  }

  // Load from URDF using plural method.
  // Add a second one with an overridden model_name.
  {
    MultibodyPlant<double> plant;
    Parser dut(&plant);
    EXPECT_EQ(dut.AddAllModelsFromFile(urdf_name).size(), 1);
    dut.AddModelFromFile(urdf_name, "foo");
  }

  // Load an SDF then a URDF.
  {
    MultibodyPlant<double> plant;
    Parser dut(&plant);
    dut.AddModelFromFile(sdf_name, "foo");
    dut.AddModelFromFile(urdf_name, "bar");
  }
}

GTEST_TEST(FileParserTest, MultiModelTest) {
  const std::string sdf_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/two_models.sdf");

  // Check that the plural method loads two models.
  {
    MultibodyPlant<double> plant;
    EXPECT_EQ(Parser(&plant).AddAllModelsFromFile(sdf_name).size(), 2);
  }

  // The singular method cannot load a two-model file.
  const char* const expected_error =
      "(.*must have a single <model> element.*)";

  // Check the singular method without model_name.
  {
    MultibodyPlant<double> plant;
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&plant).AddModelFromFile(sdf_name),
        std::exception, expected_error);
  }

  // Check the singular method with empty model_name.
  {
    MultibodyPlant<double> plant;
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&plant).AddModelFromFile(sdf_name, ""),
        std::exception, expected_error);
  }

  // Check the singular method with non-empty model_name.
  {
    MultibodyPlant<double> plant;
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&plant).AddModelFromFile(sdf_name, "foo"),
        std::exception, expected_error);
  }
}

GTEST_TEST(FileParserTest, ExtensionMatchTest) {
  // An unknown extension is an error.
  // (Check both singular and plural overloads.)
  MultibodyPlant<double> plant;
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

GTEST_TEST(FileParserTest, PackageMapTest) {
  // We start with the world and default model instances (model_instance.h
  // explains why there are two).
  MultibodyPlant<double> plant;
  Parser parser(&plant);
  ASSERT_EQ(plant.num_model_instances(), 2);

  const std::string full_sdf_filename = FindResourceOrThrow(
      "drake/automotive/models/prius/prius.sdf");
  const std::string package_path = FindResourceOrThrow(
      "drake/automotive/models/prius");

  // Attempt to read in the SDF file without setting the package map first.
  parser.AddModelFromFile(full_sdf_filename);

  // Try again.
  parser.package_map().PopulateFromFolder(package_path);
  parser.AddModelFromFile(full_sdf_filename);

  // Verify the number of model instances.
  EXPECT_EQ(plant.num_model_instances(), 3);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
