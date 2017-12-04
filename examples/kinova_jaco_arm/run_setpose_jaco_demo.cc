/// @file
///
/// This demo sets up a position controlled and gravity compensated kinova
/// jaco robot within a simulation, to reach and hold a given joint space pose.
/// The robot is initialized with an (arbitrary) joint space pose, and is
/// controlled to track and hold a final (arbitrary) joint space pose.

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/kinova_jaco_arm/jaco_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, 2, "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace kinova_jaco_arm {
namespace {

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);

  drake::lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  systems::RigidBodyPlant<double>* plant = nullptr;

  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::multibody::AddFlatTerrainToWorld(tree.get());
    CreateTreeFromFixedModelAtPose(
        FindResourceOrThrow(
            "drake/manipulation/models/jaco_description/urdf/j2n6s300.urdf"),
        tree.get());

    auto tree_sys =
        std::make_unique<systems::RigidBodyPlant<double>>(std::move(tree));
    plant =
        builder.AddSystem<systems::RigidBodyPlant<double>>(std::move(tree_sys));
    plant->set_name("plant");
  }

  // Creates and adds LCM publisher for visualization.
  auto visualizer = builder.AddSystem<systems::DrakeVisualizer>(
      plant->get_rigid_body_tree(), &lcm);

  // Adds a controller.
  VectorX<double> jaco_kp, jaco_kd, jaco_ki;
  SetPositionControlledJacoGains(&jaco_kp, &jaco_ki, &jaco_kd);
  auto control_sys =
      std::make_unique<systems::controllers::InverseDynamicsController<double>>(
          plant->get_rigid_body_tree().Clone(), jaco_kp, jaco_ki, jaco_kd,
          false /* no feedforward acceleration */);
  auto controller =
      builder
          .AddSystem<systems::controllers::InverseDynamicsController<double>>(
              std::move(control_sys));

  // Adds a constant source for desired state.
  Eigen::VectorXd const_pos = Eigen::VectorXd::Zero(kNumDofs * 2);
  const_pos(1) = 1.57;  // shoulder fore/aft angle, [rad]
  const_pos(2) = 2.0;   // elbow fore/aft angle, [rad]

  systems::ConstantVectorSource<double>* const_src =
      builder.AddSystem<systems::ConstantVectorSource<double>>(const_pos);

  const_src->set_name("constant_source");
  builder.Connect(const_src->get_output_port(),
                  controller->get_input_port_desired_state());

  // Connects the state port to the controller.
  static const int kInstanceId =
      RigidBodyTreeConstants::kFirstNonWorldModelInstanceId;
  const auto& state_out_port =
      plant->model_instance_state_output_port(kInstanceId);
  builder.Connect(state_out_port, controller->get_input_port_estimated_state());

  // Connects the controller torque output to plant.
  const auto& torque_input_port =
      plant->model_instance_actuator_command_input_port(kInstanceId);
  builder.Connect(controller->get_output_port_control(), torque_input_port);

  // Connects the visualizer and builds the diagram.
  builder.Connect(plant->get_output_port(0), visualizer->get_input_port(0));
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  systems::Context<double>& jaco_context = diagram->GetMutableSubsystemContext(
      *plant, &simulator.get_mutable_context());

  // Sets some (arbitrary) initial conditions.
  // See the @file docblock in jaco_common.h for joint index descriptions.
  systems::VectorBase<double>& x0 =
      jaco_context.get_mutable_continuous_state_vector();
  x0.SetAtIndex(1, -1.57);  // shoulder fore/aft
  x0.SetAtIndex(2, -1.57);  // elbow fore/aft

  simulator.Initialize();
  simulator.set_target_realtime_rate(1);

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
