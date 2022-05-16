#include <ostream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace {

struct TestCase {
  std::string urdf;
  int num_actuators{};
  int num_bodies{};
};

std::ostream& operator<<(std::ostream& os, const TestCase& self) {
  os << self.urdf << "," << self.num_actuators << "," << self.num_bodies;
  return os;
}

class JacoArmParamsTest
    : public ::testing::TestWithParam<TestCase> {};

// Tests that a model can be loaded into a MultibodyPlant. This unit
// test also verifies that the URDF can be parsed by the URDF parser.
TEST_P(JacoArmParamsTest, HasExpected) {
  const TestCase param = GetParam();
  const std::string urdf_path =
      "drake/manipulation/models/jaco_description/urdf/";
  const std::string urdf(FindResourceOrThrow(urdf_path + param.urdf));

  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(urdf);

  EXPECT_EQ(plant.num_actuators(), param.num_actuators);
  EXPECT_EQ(plant.num_bodies(), param.num_bodies);
}

std::vector<TestCase> GenerateTestCases() {
  return std::vector<TestCase>{
    {"j2n6s300.urdf", 9,
          6 /* robot */ +
          1 /* end effector */ +
          3 * 2 /* two per finger */ +
          3 /* world, root, base */ },
    {"j2s7s300.urdf", 10,
          7 /* robot */ +
          1 /* end effector */ +
          3 * 2 /* two per finger */ +
          3 /* world, root, base */ },
    {"j2s7s300_sphere_collision.urdf", 10,
          7 /* robot */ +
          1 /* end effector */ +
          3 * 2 /* two per finger */ +
          3 /* world, root, base */ },
    {"j2s7s300_arm.urdf", 7,
          7 /* robot */ +
          3 /* world, root, base */ },
    {"j2s7s300_arm_sphere_collision.urdf", 7,
          7 /* robot */ +
          3 /* world, root, base */ },
    {"j2s7s300_hand_sphere_collision.urdf", 3,
          1 /* end effector */ +
          3 * 2 /* two per finger */ +
          2 /* world, base */ },
    {"j2s7s300_hand.urdf", 3,
          1 /* end effector */ +
          3 * 2 /* two per finger */ +
          2 /* world, base */ },
        };
}

INSTANTIATE_TEST_SUITE_P(JacoArmTest, JacoArmParamsTest,
                         testing::ValuesIn(GenerateTestCases()));

}  // namespace
}  // namespace manipulation
}  // namespace drake
