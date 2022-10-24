#include "drake/multibody/parsing/scoped_names.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace {

GTEST_TEST(ScopedNamesTest, GetScopedFrameByName) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/scoped_names_model.sdf");

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  parser.AddModelFromFile(full_name);
  plant.Finalize();

  ASSERT_EQ(
      &GetScopedFrameByName(plant, "scoped_names_model::frame"),
      &plant.GetFrameByName(
          "frame", plant.GetModelInstanceByName("scoped_names_model")));

  ASSERT_EQ(
      &GetScopedFrameByName(
          plant, "scoped_names_model::inner_model::inner_frame"),
      &plant.GetFrameByName(
          "inner_frame",
          plant.GetModelInstanceByName("scoped_names_model::inner_model")));
}

}  // namespace
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
