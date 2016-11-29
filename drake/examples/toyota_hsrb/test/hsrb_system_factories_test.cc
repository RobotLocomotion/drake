#include "drake/examples/toyota_hsrb/hsrb_diagram_factories.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/system.h"

using std::endl;
using std::getline;
using std::ifstream;
using std::string;
using std::stringstream;
using std::unique_ptr;

namespace drake {

using lcm::DrakeMockLcm;
using systems::Diagram;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::System;

namespace examples {
namespace toyota_hsrb {
namespace {

string ReadTextFile(const string& file) {
  ifstream text_file(file);
  DRAKE_DEMAND(text_file.is_open());

  stringstream buffer;
  string line;
  while (getline(text_file, line)) {
    buffer << line << endl;
  }
  text_file.close();
  return buffer.str();
}

// Tests CreatePlantAndVisualizerDiagram() and
// CreateConstantSourceToPlantDiagram(). Since these are factory methods, only
// the structure of the returned systems::Diagram objects are verified for
// correctness.
GTEST_TEST(ToyotaHsrbTests, TestDiagramFactories) {
  // A two DOF robot model is used instead of the Toyota HSRb since (1) the
  // factory methods being tested are general and should work with all URDFs and
  // (2) HSRb URDFs need to be generated using `xacro`, the latest version of
  // which is currently not supported by Drake.
  const string urdf_string =
      ReadTextFile(drake::GetDrakePath() +
                   "/multibody/test/rigid_body_tree/two_dof_robot.urdf");
  const double penetration_stiffness = 4000;
  const double penetration_damping = 300;
  const double friction_coefficient = 10;

  RigidBodyPlant<double>* plant{nullptr};
  DrakeMockLcm lcm;

  // The Device Under Test (DUT) #2 is a Diagram that connects a constant vector
  // source to the input of DUT #1.
  Diagram<double>* dut1_ptr{nullptr};
  unique_ptr<Diagram<double>> dut2;
  {
    // The Device Under Test #1 is a Diagram that contains a RigidBodyPlant
    // whose output is connected to a DrakeVisualizer.
    unique_ptr<Diagram<double>> dut1 = CreatePlantAndVisualizerDiagram(
        urdf_string, penetration_stiffness, penetration_damping,
        friction_coefficient, &lcm, &plant);

    dut1_ptr = dut1.get();
    ASSERT_NE(dut1_ptr, nullptr);
    ASSERT_NE(plant, nullptr);

    std::vector<const System<double>*> systems = dut1->GetSystems();
    EXPECT_EQ(systems.size(), 2u);
    const RigidBodyPlant<double>* rigid_body_plant =
        dynamic_cast<const RigidBodyPlant<double>*>(systems.at(0));
    const DrakeVisualizer* visualizer =
        dynamic_cast<const DrakeVisualizer*>(systems.at(1));
    ASSERT_NE(rigid_body_plant, nullptr);
    ASSERT_NE(visualizer, nullptr);

    dut2 = CreateConstantSourceToPlantDiagram(*plant, std::move(dut1));
  }
  ASSERT_NE(dut2, nullptr);

  // The DUT should not have any input ports since the plant's input port is
  // connected to a constant vector source, which has no inputs.
  EXPECT_EQ(dut2->get_num_input_ports(), 0);

  // The DUT should have one output port that's connected to DUT #1's output
  // port zero.
  EXPECT_EQ(dut2->get_num_output_ports(), 1);

  // 7 floating joint positions + 6 floating joint velocities + 2 joint
  // positions + 2 joint velocities = 17 states.
  const int kNumStates = 17;

  EXPECT_EQ(dut2->get_output_port(0).get_size(), kNumStates);
  EXPECT_EQ(plant->get_num_states(), kNumStates);
}

}  // namespace
}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake
