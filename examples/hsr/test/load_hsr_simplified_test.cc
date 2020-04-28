#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace hsr {

// Tests if the model from hsrb4s_simplified.urdf loads with no errors.
GTEST_TEST(LoadHsrSimplifiedTest, TestIfPr2SimplifiedLoads) {
  systems::DiagramBuilder<double> builder;
  multibody::MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0 /* time_step */);
  const std::string& pathname = FindResourceOrThrow(
      "drake/examples/hsr/models/urdfs/hsrb4s_simplified.urdf");
  multibody::Parser parser(&plant);
  parser.package_map().PopulateUpstreamToDrake(pathname);
  parser.AddModelFromFile(pathname);
  plant.Finalize();

  EXPECT_EQ(plant.num_actuators(), 13);
  EXPECT_EQ(plant.num_positions(), 20);
  // 40 links of robot plus one for the world body.
  EXPECT_EQ(plant.num_bodies(), 41);
}

}  // namespace hsr
}  // namespace examples
}  // namespace drake
