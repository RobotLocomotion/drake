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

}  // namespace
}  // namespace multibody
}  // namespace drake
