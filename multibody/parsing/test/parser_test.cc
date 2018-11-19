#include "drake/multibody/parsing/parser.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"

using drake::multibody::multibody_plant::MultibodyPlant;

namespace drake {
namespace multibody {
namespace parsing {
namespace test {
namespace {

GTEST_TEST(FileParserTest, BasicTest) {
  const std::string sdf_name = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/acrobot.sdf");
  const std::string urdf_name = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/acrobot.urdf");

  // Load acrobot once each from SDF and URDF.
  MultibodyPlant<double> plant;
  Parser dut(&plant);
  dut.AddModelFromFile(sdf_name, "bot1");
  dut.AddModelFromFile(urdf_name, "bot2");

  // Load again using the plural methods.
  EXPECT_EQ(dut.AddModelsFromFile(sdf_name, "bot3").size(), 1);
  EXPECT_EQ(dut.AddModelsFromFile(urdf_name, "bot4").size(), 1);

  // Load again using the default name.
  dut.AddModelFromFile(sdf_name);
}

GTEST_TEST(FileParserTest, MultiModelTest) {
  const std::string sdf_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/two_models.sdf");

  // Check the plural method without model_name.
  {
    MultibodyPlant<double> plant;
    EXPECT_EQ(Parser(&plant).AddModelsFromFile(sdf_name).size(), 2);
  }

  // Check the plural method with empty model_name.
  {
    MultibodyPlant<double> plant;
    EXPECT_EQ(Parser(&plant).AddModelsFromFile(sdf_name, {}).size(), 2);
  }

  // For the failures cases, we don't care which of two possible errors comes
  // back.
  const char* const allowed_errors =
      "(.*must have a single <model> element.*)|"
      "(.*contained multiple \\(2\\) models.*)";

  // Check the plural method with non-empty model_name.
  {
    MultibodyPlant<double> plant;
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&plant).AddModelsFromFile(sdf_name, "foo"),
        std::exception, allowed_errors);
  }

  // Check the singular method without model_name.
  {
    MultibodyPlant<double> plant;
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&plant).AddModelFromFile(sdf_name),
        std::exception, allowed_errors);
  }

  // Check the singular method with empty model_name.
  {
    MultibodyPlant<double> plant;
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&plant).AddModelFromFile(sdf_name, ""),
        std::exception, allowed_errors);
  }

  // Check the singular method with non-empty model_name.
  {
    MultibodyPlant<double> plant;
    DRAKE_EXPECT_THROWS_MESSAGE(
        Parser(&plant).AddModelFromFile(sdf_name, "foo"),
        std::exception, allowed_errors);
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
      Parser(&plant).AddModelsFromFile("acrobot.foo"),
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
}  // namespace test
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
