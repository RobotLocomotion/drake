/// @file
///
/// This demo sets up a position controlled and gravity compensated kinova
/// jaco robot within a simulation, to reach and hold a given joint space pose.
/// The robot is initialized with an (arbitrary) joint space pose, and is
/// controlled to track and hold a final (arbitrary) joint space pose.

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/kinova_jaco_arm/jaco_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/controllers/state_feedback_controller_base.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"



DEFINE_double(simulation_sec, 2, "Number of seconds to simulate.");

using Eigen::VectorXd;
using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {
namespace examples {
namespace kinova_jaco_arm {
namespace {

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);

  drake::lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder_;

  systems::RigidBodyPlant<double>* plant = nullptr;
  const std::string kUrdfPath = "/manipulation/models/jaco_description/urdf/"
      "j2n6s300.urdf";

  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::multibody::AddFlatTerrainToWorld(tree.get());
    CreateTreedFromFixedModelAtPose(kUrdfPath, tree.get());

    auto tree_sys =
        std::make_unique<systems::RigidBodyPlant<double>>(std::move(tree));
    plant = builder_.template AddSystem<systems::RigidBodyPlant<double>>(
        std::move(tree_sys));
    plant->set_name("plant");
  }

  // Creates and adds LCM publisher for visualization.
  auto visualizer =
      builder_.template AddSystem<systems::DrakeVisualizer>(
          plant->get_rigid_body_tree(), &lcm);

  // Adds a controller
  VectorX<double> jaco_kp, jaco_kd, jaco_ki;
  SetPositionControlledJacoGains(&jaco_kp, &jaco_ki, &jaco_kd);
  auto contrl_sys =
      std::make_unique<systems::InverseDynamicsController<double>>(
          GetDrakePath() + kUrdfPath, nullptr, jaco_kp, jaco_ki, jaco_kd,
          false /* no feedforward acceleration */);
  auto controller =
      builder_.template AddSystem<systems::InverseDynamicsController<double>>(
          std::move(contrl_sys));

  // Adds a constant source for desired state.
  VectorXd const_pos = VectorXd::Zero(18);
  const_pos(1) = 1.57; const_pos(2) = 2.0;

  systems::ConstantVectorSource<double>* const_src =
      builder_.AddSystem<systems::ConstantVectorSource<double>>(const_pos);

  const_src->set_name("constant_source");
  builder_.Connect(const_src->get_output_port(),
                  controller->get_input_port_desired_state());

  // Connects the state port to the controller.
  const auto& instance_state_output_port =
      plant->model_instance_state_output_port(
          RigidBodyTreeConstants::kFirstNonWorldModelInstanceId);
  builder_.Connect(instance_state_output_port,
                    controller->get_input_port_estimated_state());

  // Connects the controller torque output to plant.
  const auto& instance_torque_input_port =
      plant->model_instance_actuator_command_input_port(
          RigidBodyTreeConstants::kFirstNonWorldModelInstanceId);
  builder_.Connect(controller->get_output_port_control(),
                    instance_torque_input_port);

  // Connect the visualizer and build the diagram
  builder_.Connect(plant->get_output_port(0), visualizer->get_input_port(0));
  std::unique_ptr<systems::Diagram<double>> diagram = builder_.Build();

  systems::Simulator<double> simulator(*diagram);

  systems::Context<double>* jaco_context =
          diagram->GetMutableSubsystemContext(simulator.get_mutable_context(),
                                              plant);

  //  Set some initial conditions
  systems::VectorBase<double>* x0 =
  jaco_context->get_mutable_continuous_state_vector();

  x0->SetAtIndex(1, -1.57);
  x0->SetAtIndex(2, -1.57);

  simulator.Initialize();
  simulator.set_target_realtime_rate(0.5);

  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace kinova_jaco_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kinova_jaco_arm::DoMain();
}
