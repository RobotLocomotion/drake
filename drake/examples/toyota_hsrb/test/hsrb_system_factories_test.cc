#include "drake/examples/toyota_hsrb/hsrb_diagram_factories.h"

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
// #include <typeinfo>

#include "gtest/gtest.h"

// #include "drake/common/eigen_matrix_compare.h"
// #include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
// #include "drake/system1/Simulation.h"
// #include "drake/system1/cascade_system.h"
// #include "drake/system1/robot_state_tap.h"
// #include "drake/multibody/rigid_body_system1/RigidBodySystem.h"

// using drake::CascadeSystem;
// using drake::RigidBodySystem;
// using Eigen::VectorXd;
// using drake::RobotStateTap;
// using drake::MatrixCompareType;

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

using std::endl;
using std::getline;
using std::ifstream;
using std::string;
using std::stringstream;

namespace drake {

using lcm::DrakeMockLcm;
using systems::RigidBodyPlant;

namespace examples {
namespace toyota_hsrb {
namespace {

string ReadTextFile(const string& file) {
  ifstream text_file(file);
  DRAKE_DEMAND(text_file.is_open());

  stringstream buffer;
  string line;
  while(getline(text_file, line)) {
      buffer << line << endl;
  }
  text_file.close();
  return buffer.str();
}

// Tests CreateHsrbPlantDiagram().
GTEST_TEST(ToyotaHsrbTests, TestCreatePlantAndVisualizerDiagram) {
  const string urdf_string = ReadTextFile(drake::GetDrakePath() +
  	  "/multibody/test/rigid_body_tree/two_dof_robot.urdf");
  const double penetration_stiffness = 4000;
  const double penetration_damping = 300;
  const double friction_coefficient = 10;

  RigidBodyPlant<double>* plant{nullptr};
  DrakeMockLcm lcm;

  std::unique_ptr<systems::Diagram<double>> dut =
      CreatePlantAndVisualizerDiagram(urdf_string, penetration_stiffness,
      penetration_damping, friction_coefficient, &lcm, &plant);
  EXPECT_NE(plant, nullptr);
}

// Tests CreateHsrbPlantDiagram().
GTEST_TEST(ToyotaHsrbTests, CreateConstantSourceToPlantDiagram) {
}


}  // namespace
}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake
