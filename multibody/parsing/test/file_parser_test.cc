#include "drake/multibody/parsing/file_parser.h"

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
  AddModelFromFile(sdf_name, "bot1", &plant);
  AddModelFromFile(urdf_name, "bot2", &plant);

  // Load again using the plural methods.
  EXPECT_EQ(AddModelsFromFile(sdf_name, "bot3", &plant).size(), 1);
  EXPECT_EQ(AddModelsFromFile(urdf_name, "bot4", &plant).size(), 1);

  // Load again using the default name.
  AddModelFromFile(sdf_name, &plant);
}

GTEST_TEST(FileParserTest, MultiModelTest) {
  const std::string sdf_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/two_models.sdf");

  // Check the plural overload without model_name.
  {
    MultibodyPlant<double> plant;
    EXPECT_EQ(AddModelsFromFile(sdf_name, &plant).size(), 2);
  }

  // Check the plural overload with empty model_name.
  {
    MultibodyPlant<double> plant;
    EXPECT_EQ(AddModelsFromFile(sdf_name, {}, &plant).size(), 2);
  }

  // For the failures cases, we don't care which of two possible errors comes
  // back.
  const char* const allowed_errors =
      "(.*must have a single <model> element.*)|"
      "(.*contained too many \\(2\\) models.*)";

  // Check the plural overload with non-empty model_name.
  {
    MultibodyPlant<double> plant;
    DRAKE_EXPECT_THROWS_MESSAGE(
        AddModelsFromFile(sdf_name, "foo", &plant),
        std::exception, allowed_errors);
  }

  // Check the singular overload without model_name.
  {
    MultibodyPlant<double> plant;
    DRAKE_EXPECT_THROWS_MESSAGE(
        AddModelFromFile(sdf_name, &plant),
        std::exception, allowed_errors);
  }

  // Check the singular overload with empty model_name.
  {
    MultibodyPlant<double> plant;
    DRAKE_EXPECT_THROWS_MESSAGE(
        AddModelFromFile(sdf_name, "", &plant),
        std::exception, allowed_errors);
  }

  // Check the singular overload with non-empty model_name.
  {
    MultibodyPlant<double> plant;
    DRAKE_EXPECT_THROWS_MESSAGE(
        AddModelFromFile(sdf_name, "foo", &plant),
        std::exception, allowed_errors);
  }
}

GTEST_TEST(FileParserTest, ExtensionMatchTest) {
  // An unknown extension is an error.
  // (Check both singular and plural overloads.)
  MultibodyPlant<double> plant;
  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromFile("acrobot.foo", &plant),
      std::exception,
      ".*file type '\\.foo' is not supported .*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelsFromFile("acrobot.foo", &plant),
      std::exception,
      ".*file type '\\.foo' is not supported .*");

  // Uppercase extensions are accepted (i.e., still call the underlying SDF or
  // URDF parser, shown here by it generating a different exception message).
  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromFile("acrobot.SDF", &plant),
      std::exception,
      ".*does not exist.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromFile("acrobot.URDF", &plant),
      std::exception,
      ".*does not exist.*");
}

}  // namespace
}  // namespace test
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
