#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

// Tests that dual_iiwa14_polytope_collision.urdf can be parsed.
GTEST_TEST(DualIiwa14PolytopeCollisionTest, TestLoadTree) {
  const std::string kPath(FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "dual_iiwa14_polytope_collision.urdf"));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(kPath);
  plant.Finalize();

  // Each robot has 8 bodies and an end effector with 2 bodies. In addition,
  // there are two bodies that are not part of either robot: world and base.
  // Since there are two robots, there should be a total of
  // 2 * (8 + 2) + 2 = 22 bodies in the tree.
  EXPECT_EQ(plant.num_bodies(), 22);

  // MultibodyPlant always creates at least two model instances, one for the
  // world and one for a default model instance for unspecified modeling
  // elements. Finally, there is *one* model instance for the IIWAs.
  EXPECT_EQ(plant.num_model_instances(), 3);
}

}  // namespace
}  // namespace manipulation
}  // namespace drake
