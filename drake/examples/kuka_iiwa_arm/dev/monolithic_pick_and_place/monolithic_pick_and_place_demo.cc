#include <memory>
#include <string>

#include <gflags/gflags.h>
#include "bot_core/robot_state_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/action_primitives_common.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/demo_diagram_builder.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_command.hpp"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

using robotlocomotion::robot_plan_t;

namespace drake {
using systems::RigidBodyPlant;
using systems::Simulator;

namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {
namespace {

int DoMain(void) {
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  auto plant = builder.template AddSystem<
      IiwaWsgPlantGeneratorsEstimatorsAndVisualizer<double>>(
      std::make_unique<IiwaWsgPlantGeneratorsEstimatorsAndVisualizer<double>>(
          &lcm));

  auto iiwa_base_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      kRobotBase, Eigen::Vector3d::Zero());

  RigidBodyTree<double> iiwa;
  parsers::urdf::AddModelInstanceFromUrdfFile(drake::GetDrakePath() + kIiwaUrdf,
                                              multibody::joints::kFixed,
                                              iiwa_base_frame, &iiwa);

  auto state_machine = builder.template AddSystem<StateMachineAndPrimitives>(
      iiwa, 0.01 /* Iiwa_action_primitive_rate */,
      0.01 /* Wsg_action_primtiive_rate */);

  builder.Connect(plant->get_output_port_box_robot_state_est_msg(),
                  state_machine->get_input_port_box_robot_state());
  builder.Connect(plant->get_output_port_wsg_status(),
                  state_machine->get_input_port_wsg_status());
  builder.Connect(plant->get_output_port_iiwa_robot_state_est_msg(),
                  state_machine->get_input_port_iiwa_robot_state());
  builder.Connect(state_machine->get_output_port_wsg_command(),
                  plant->get_input_port_wsg_plan());
  builder.Connect(state_machine->get_output_port_iiwa_command(),
                  plant->get_input_port_iiwa_plan());

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);
  simulator.Initialize();

  plant->InitializeIiwaPlan(
      Eigen::VectorXd::Zero(7),
      sys->GetMutableSubsystemContext(simulator.get_mutable_context(), plant));

  simulator.StepTo(FLAGS_simulation_sec);

  std::cout << "Demo completed.\n";

  return 0;
}

}  // namespace
}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, const char* argv[]) {
  drake::examples::kuka_iiwa_arm::monolithic_pick_and_place::DoMain();
  return 0;
}
