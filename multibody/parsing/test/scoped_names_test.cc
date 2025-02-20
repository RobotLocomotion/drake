#include "drake/multibody/parsing/scoped_names.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace {

GTEST_TEST(ScopedNamesTest, GetScopedFrameByName) {
  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelsFromUrl(
      "package://drake/multibody/parsing/test/scoped_names_model.sdf");
  plant.Finalize();

  const char* full_name = "scoped_names_model::frame";
  const Frame<double>* const outer_frame = &plant.GetFrameByName(
      "frame", plant.GetModelInstanceByName("scoped_names_model"));
  EXPECT_EQ(GetScopedFrameByNameMaybe(plant, full_name), outer_frame);
  EXPECT_EQ(&GetScopedFrameByName(plant, full_name), outer_frame);

  full_name = "scoped_names_model::inner_model::inner_frame";
  const Frame<double>* const inner_frame = &plant.GetFrameByName(
      "inner_frame",
      plant.GetModelInstanceByName("scoped_names_model::inner_model"));
  EXPECT_EQ(GetScopedFrameByNameMaybe(plant, full_name), inner_frame);
  EXPECT_EQ(&GetScopedFrameByName(plant, full_name), inner_frame);

  full_name = "foo";
  EXPECT_EQ(GetScopedFrameByNameMaybe(plant, full_name), nullptr);
  DRAKE_EXPECT_THROWS_MESSAGE(GetScopedFrameByName(plant, full_name),
                              ".*no.*foo.*valid names .*world.*");

  full_name = "scoped_names_model::foo";
  EXPECT_EQ(GetScopedFrameByNameMaybe(plant, full_name), nullptr);
  DRAKE_EXPECT_THROWS_MESSAGE(GetScopedFrameByName(plant, full_name),
                              ".*no.*foo.*valid names .*base.*");

  full_name = "scoped_names_model::inner_model::foo";
  EXPECT_EQ(GetScopedFrameByNameMaybe(plant, full_name), nullptr);
  DRAKE_EXPECT_THROWS_MESSAGE(GetScopedFrameByName(plant, full_name),
                              ".*no.*foo.*valid names .*inner_frame.*");

  full_name = "bar_model::foo";
  EXPECT_EQ(GetScopedFrameByNameMaybe(plant, full_name), nullptr);
  DRAKE_EXPECT_THROWS_MESSAGE(
      GetScopedFrameByName(plant, full_name),
      ".*no.*bar_model.*instances are.*scoped_names_model.*");
}

GTEST_TEST(ScopedNamesTest, AutoDiffXd) {
  MultibodyPlant<AutoDiffXd> plant(0.0);
  EXPECT_NO_THROW(GetScopedFrameByName(plant, "world"));
  EXPECT_NO_THROW(GetScopedFrameByNameMaybe(plant, "world"));
}

GTEST_TEST(ScopedNamesTest, Expression) {
  MultibodyPlant<symbolic::Expression> plant(0.0);
  EXPECT_NO_THROW(GetScopedFrameByName(plant, "world"));
  EXPECT_NO_THROW(GetScopedFrameByNameMaybe(plant, "world"));
}

}  // namespace
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
