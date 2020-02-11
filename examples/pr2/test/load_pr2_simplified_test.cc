#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace pr2 {

// Tests if the model from pr2_simplified.urdf loads with no errors.
GTEST_TEST(LoadPr2SimplifiedTest, TestIfPr2SimplifiedLoads) {
  systems::DiagramBuilder<double> builder;
  multibody::MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0 /* time_step */);
  const std::string& pathname = FindResourceOrThrow(
      "drake/examples/pr2/models/pr2_description/urdf/pr2_simplified.urdf");
  multibody::Parser parser(&plant);
  parser.package_map().PopulateUpstreamToDrake(pathname);
  parser.AddModelFromFile(pathname);
  plant.Finalize();

  EXPECT_EQ(plant.num_actuators(), 28);
  EXPECT_EQ(plant.num_positions(), 28);
  EXPECT_EQ(plant.num_bodies(), 84);
}

}  // namespace pr2
}  // namespace examples
}  // namespace drake
