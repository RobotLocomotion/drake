#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {

using geometry::SceneGraph;
using geometry::SourceId;
using lcm::DrakeLcm;
using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::MultibodyPlant;
using multibody::RevoluteJoint;
using systems::ImplicitEulerIntegrator;
using systems::RungeKutta3Integrator;
using systems::SemiExplicitEulerIntegrator;

namespace examples {
namespace multibody {
namespace acrobot {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_string(integration_scheme, "runge_kutta3",
              "Integration scheme to be used. Available options are:"
              "'runge_kutta3','implicit_euler','semi_explicit_euler'");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double simulation_time = FLAGS_simulation_time;

  // Make the desired maximum time step a fraction of the simulation time.
  const double max_time_step = simulation_time / 1000.0;

  // The target accuracy determines the size of the actual time steps taken
  // whenever a variable time step integrator is used.
  const double target_accuracy = 0.001;

  // Make and add the acrobot model.
  const AcrobotParameters acrobot_parameters;
  const MultibodyPlant<double>& acrobot = *builder.AddSystem(MakeAcrobotPlant(
      acrobot_parameters, true /* Finalize the plant */, &scene_graph));
  const RevoluteJoint<double>& shoulder =
      acrobot.GetJointByName<RevoluteJoint>(
          acrobot_parameters.shoulder_joint_name());
  const RevoluteJoint<double>& elbow =
      acrobot.GetJointByName<RevoluteJoint>(
          acrobot_parameters.elbow_joint_name());

  // A constant source for a zero applied torque at the elbow joint.
  double applied_torque(0.0);
  auto torque_source =
      builder.AddSystem<systems::ConstantVectorSource>(applied_torque);
  torque_source->set_name("Applied Torque");
  builder.Connect(torque_source->get_output_port(),
                  acrobot.get_actuation_input_port());

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(acrobot.get_source_id().has_value());

  builder.Connect(
      acrobot.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(acrobot.get_source_id().value()));

  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& acrobot_context =
      diagram->GetMutableSubsystemContext(acrobot, diagram_context.get());

  // Set initial angles. Velocities are left to the default zero values.
  shoulder.set_angle(&acrobot_context, 1.0);
  elbow.set_angle(&acrobot_context, 1.0);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  systems::IntegratorBase<double>* integrator{nullptr};
  if (FLAGS_integration_scheme == "implicit_euler") {
    integrator =
        &simulator.reset_integrator<ImplicitEulerIntegrator<double>>();
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    integrator =
        &simulator.reset_integrator<RungeKutta3Integrator<double>>();
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    integrator =
        &simulator.reset_integrator<SemiExplicitEulerIntegrator<double>>(
            max_time_step);
  } else {
    throw std::runtime_error(
        "Integration scheme '" + FLAGS_integration_scheme +
        "' not supported for this example.");
  }
  integrator->set_maximum_step_size(max_time_step);

  // Error control is only supported for variable time step integrators.
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(target_accuracy);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(simulation_time);

  // Some sanity checks:
  if (FLAGS_integration_scheme == "semi_explicit_euler") {
    DRAKE_DEMAND(integrator->get_fixed_step_mode() == true);
  }

  // Checks for variable time step integrators.
  if (!integrator->get_fixed_step_mode()) {
    // From IntegratorBase::set_maximum_step_size():
    // "The integrator may stretch the maximum step size by as much as 1% to
    // reach discrete event." Thus the 1.01 factor in this DRAKE_DEMAND.
    DRAKE_DEMAND(
        integrator->get_largest_step_size_taken() <= 1.01 * max_time_step);
    DRAKE_DEMAND(integrator->get_smallest_adapted_step_size_taken() <=
        integrator->get_largest_step_size_taken());
    DRAKE_DEMAND(
        integrator->get_num_steps_taken() >= simulation_time / max_time_step);
  }

  // Checks for fixed time step integrators.
  if (integrator->get_fixed_step_mode()) {
    DRAKE_DEMAND(integrator->get_num_derivative_evaluations() ==
        integrator->get_num_steps_taken());
    DRAKE_DEMAND(
        integrator->get_num_step_shrinkages_from_error_control() == 0);
  }

  // We made a good guess for max_time_step and therefore we expect no
  // failures when taking a time step.
  DRAKE_DEMAND(integrator->get_num_substep_failures() == 0);
  DRAKE_DEMAND(
      integrator->get_num_step_shrinkages_from_substep_failures() == 0);

  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple acrobot demo using Drake's MultibodyPlant,"
      "with SceneGraph visualization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::acrobot::do_main();
}
