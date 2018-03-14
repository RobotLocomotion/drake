#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/multibody/bouncing_ball/make_bouncing_ball_plant.h"
#include "drake/geometry/geometry_system.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/math/random_rotation.h"
#include "drake/multibody/multibody_tree/quaternion_floating_mobilizer.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_string(integration_scheme, "runge_kutta3",
              "Integration scheme to be used. Available options are:"
              "'runge_kutta3','implicit_euler','semi_explicit_euler'");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using geometry::GeometrySystem;
using geometry::SourceId;
using lcm::DrakeLcm;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::MultibodyTree;
using drake::multibody::QuaternionFloatingMobilizer;
using drake::systems::ImplicitEulerIntegrator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::Serializer;
using drake::systems::rendering::PoseBundleToDrawMessage;
using drake::systems::RungeKutta3Integrator;
using drake::systems::SemiExplicitEulerIntegrator;

int do_main() {
  systems::DiagramBuilder<double> builder;

  GeometrySystem<double>& geometry_system =
      *builder.AddSystem<GeometrySystem>();
  geometry_system.set_name("geometry_system");

  const double simulation_time = 20.00;

  // Make the desired maximum time step a fraction of the simulation time.
  const double max_time_step = 1e-4;

  // The target accuracy determines the size of the actual time steps taken
  // whenever a variable time step integrator is used.
  const double target_accuracy = 0.001;

  // Plant's parameters.
  const double radius = 0.05;   // m
  const double mass = 0.1;      // kg
  const double g = 9.81;        // m/s^2
  const double z0 = 0.3;        // Initial height.

  const double penetration_length = 1.0e-3;
  const double stiffness = mass * g / penetration_length;  // static equilibrium (under estimation)
  const double omega = sqrt(stiffness / mass);  // frequency
  const double damping_ratio = 1.0;  // not realy, but should be close.
  const double damping = damping_ratio * (1.0/omega)/penetration_length; // Approx critically damped.

  MultibodyPlant<double>& plant =
      *builder.AddSystem(MakeBouncingBallPlant(
          radius, mass, -g * Vector3d::UnitZ(), &geometry_system));
  const MultibodyTree<double>& model = plant.model();
  plant.set_contact_penalty_stiffness(stiffness);
  plant.set_contact_penalty_damping(damping);

  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  // Boilerplate used to connect the plant to a GeometrySystem for
  // visualization.
  DrakeLcm lcm;
  const PoseBundleToDrawMessage& converter =
      *builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem& publisher =
      *builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher.set_publish_period(1 / 60.0);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(
      plant.get_geometry_ids_output_port(),
      geometry_system.get_source_frame_id_port(
          plant.get_source_id().value()));
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      geometry_system.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(geometry_system.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  builder.Connect(geometry_system.get_pose_bundle_output_port(),
                  converter.get_input_port(0));
  builder.Connect(converter, publisher);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(geometry_system);

  // And build the Diagram:
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

#if 0
  const QuaternionFloatingMobilizer<double>& ball_mobilizer =
      model.GetFreeBodyMobilizerOrThrow(plant.GetBodyByName("Ball"));

  // Set initial angles. Velocities are left to the default zero values.
  ball_mobilizer.set_position(&plant_context, Vector3d(0, 0, z0));
#endif

  auto set_position = [&](
      const std::string& body_name, const Vector3d& p_WB) {
    const QuaternionFloatingMobilizer<double>& ball_mobilizer =
        model.GetFreeBodyMobilizerOrThrow(plant.GetBodyByName(body_name));
    ball_mobilizer.set_position(&plant_context, p_WB);
  };

  auto set_orientation = [&](
      const std::string& body_name, const Matrix3d& R_WB) {
    const QuaternionFloatingMobilizer<double>& ball_mobilizer =
        model.GetFreeBodyMobilizerOrThrow(plant.GetBodyByName(body_name));
    ball_mobilizer.SetFromRotationMatrix(&plant_context, R_WB);
  };

  // Set at height z0 with random orientation.
  std::mt19937 generator(41);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  model.SetDefaultContext(&plant_context);
  set_position("Ball", Vector3d(0.0, 0.0, z0));
  Matrix3d R_WB = math::UniformlyRandomRotmat(generator);
  //set_orientation("Ball", R_WB * AngleAxisd(M_PI, Vector3d::UnitX()).matrix());
  set_orientation("Ball", R_WB);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  systems::IntegratorBase<double>* integrator{nullptr};
  if (FLAGS_integration_scheme == "implicit_euler") {
    integrator =
        simulator.reset_integrator<ImplicitEulerIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    integrator =
        simulator.reset_integrator<RungeKutta3Integrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    integrator =
        simulator.reset_integrator<SemiExplicitEulerIntegrator<double>>(
            *diagram, max_time_step, &simulator.get_mutable_context());
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
  simulator.StepTo(simulation_time);

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
}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple acrobot demo using Drake's MultibodyTree,"
      "with GeometrySystem visualization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::multibody::bouncing_ball::do_main();
}
