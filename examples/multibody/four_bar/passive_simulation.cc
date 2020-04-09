#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {

using geometry::SceneGraph;
using lcm::DrakeLcm;

using Eigen::Vector2d;
using Eigen::Vector3d;
using geometry::Cylinder;
using geometry::Sphere;
using math::RigidTransformd;
using multibody::ExternallyAppliedSpatialForce;
using multibody::Frame;
using multibody::JointActuator;
using multibody::LinearBushingRollPitchYaw;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::RevoluteJoint;
using multibody::SpatialForce;
using systems::Context;
using systems::ImplicitEulerIntegrator;
using systems::InputPort;
using systems::RungeKutta3Integrator;
using systems::SemiExplicitEulerIntegrator;

namespace examples {
namespace multibody {
namespace four_bar {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

DEFINE_string(integration_scheme, "runge_kutta3",
              "Integration scheme to be used. Available options are:"
              "'runge_kutta3','implicit_euler','semi_explicit_euler'");

DEFINE_bool(asymmetric_model, true,
            "If 'true', the model 'asymmetric_linkage.sdf' will be loaded. "
            "Otherwise, the default model will be 'parallel_linkage.sdf'.");

DEFINE_bool(show_bushing, true,
            "If 'true', add visual geometry to show where the bushing "
            "weld is placed on the coupler link.");

DEFINE_bool(apply_force, false,
            "If 'true', applies an external spatial force of -200Nm in the "
            "world_x direction to the RockerCoupler link's origin.");

DEFINE_double(default_force_stiffness, 30000,
              "Desired force stiffness value for kx, ky, and kz of the "
              "LinearBushingRollPitchYaw ForceElement.");

DEFINE_double(default_force_damping, 800,
              "Desired force damping value for dx, dy, and dz of the "
              "LinearBushingRollPitchYaw ForceElement.");

DEFINE_double(default_torque_stiffness, 30000,
              "Desired torque stiffness value for k₀, k₁, and k₂ of the "
              "LinearBushingRollPitchYaw ForceElement.");

DEFINE_double(default_torque_damping, 800,
              "Desired torque damping value for d₀, d₁, and d₂ of the "
              "LinearBushingRollPitchYaw ForceElement.");

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

  // Make and add the four_bar model from an SDF model, choosing either the
  // parallel or asymmetric linkage.
  const std::string relative_name =
      FLAGS_asymmetric_model
          ? "drake/examples/multibody/four_bar/asymmetric_linkage.sdf"
          : "drake/examples/multibody/four_bar/parallel_linkage.sdf";

  const std::string full_name = FindResourceOrThrow(relative_name);
  MultibodyPlant<double>& four_bar = *builder.AddSystem<MultibodyPlant>(0.0);

  Parser parser(&four_bar, &scene_graph);
  parser.AddModelFromFile(full_name);

  // Grab the two coincident frames at the midpoint of the coupler link
  // One is attached to the Crank side, one to the Rocker side.
  // The two frames will be welded together with a LinearBushingRollPitchYaw
  // force element to approximate a closed-loop kinematic chain.
  const Frame<double>& crank_coupler_weld =
      four_bar.GetFrameByName("CrankCouplerWeld");

  const Frame<double>& rocker_coupler_weld =
      four_bar.GetFrameByName("RockerCouplerWeld");

  // TODO(joemasterjohn) come up with a way to estimate correct parameters
  //  for stiffness and damping constants. These are hand tuned to fit the
  //  linkages used in this example and make the simulation "look right".
  double k_xyz = FLAGS_default_force_stiffness;
  double d_xyz = FLAGS_default_force_stiffness;
  double k_012 = FLAGS_default_torque_stiffness;
  double d_012 = FLAGS_default_torque_damping;

  Vector3<double> torque_stiffness_constants = {k_xyz, k_xyz, k_xyz};
  Vector3<double> torque_damping_constants = {d_xyz, d_xyz, d_xyz};
  Vector3<double> force_stiffness_constants = {k_012, k_012, k_012};
  Vector3<double> force_damping_constants = {d_012, d_012, d_012};

  // Add a bushing force element between the Crank and the Rocker at their
  // coincident weld frames.
  four_bar.AddForceElement<LinearBushingRollPitchYaw>(
      crank_coupler_weld, rocker_coupler_weld, torque_stiffness_constants,
      torque_damping_constants, force_stiffness_constants,
      force_damping_constants);

  // If specified, visualize where the bushing element is placed
  if (FLAGS_show_bushing) {
    const Vector4<double> green(0.0, 1.0, 0.0, 1.0);
    // Visual for the Coupler bushing weld
    four_bar.RegisterVisualGeometry(
        four_bar.GetBodyByName("CrankCoupler"),
        // Place it at the weld point, -1m down the Z-axis of the CrankCoupler
        RigidTransformd(Vector3<double>(0.0, 0.0, -1.0)), Cylinder(0.055, 0.05),
        "Bushing Weld", green);
  }

  // We are done defining the model.
  four_bar.Finalize();

  // A constant source for a zero applied torque at the Crank joint.
  double applied_torque(0.0);
  auto torque_source =
      builder.AddSystem<systems::ConstantVectorSource>(applied_torque);
  torque_source->set_name("Applied Torque");
  builder.Connect(torque_source->get_output_port(),
                  four_bar.get_actuation_input_port());

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!four_bar.get_source_id());

  builder.Connect(
      four_bar.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(four_bar.get_source_id().value()));

  geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  auto diagram = builder.Build();

  // Create a context for this system and sub-context for the four bar system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& four_bar_context =
      diagram->GetMutableSubsystemContext(four_bar, diagram_context.get());

  // If we've loaded the parallel linkage model, set the initial conditions
  // so the model has some motion.
  if (!FLAGS_asymmetric_model) {
    RevoluteJoint<double>& crank_joint =
        four_bar.GetMutableJointByName<RevoluteJoint>("CrankJoint");
    RevoluteJoint<double>& rocker_joint =
        four_bar.GetMutableJointByName<RevoluteJoint>("RockerJoint");

    RevoluteJoint<double>& crank_coupler_joint =
        four_bar.GetMutableJointByName<RevoluteJoint>("CrankCouplerJoint");
    RevoluteJoint<double>& rocker_coupler_joint =
        four_bar.GetMutableJointByName<RevoluteJoint>("RockerCouplerJoint");

    // Set initial angles. Velocities are left to the default zero values.
    crank_joint.set_angle(&four_bar_context, M_PI_4);
    rocker_joint.set_angle(&four_bar_context, M_PI_4);
    crank_coupler_joint.set_angle(&four_bar_context, -M_PI_4);
    rocker_coupler_joint.set_angle(&four_bar_context, -M_PI_4);
  }

  // Apply an external spatial force to the origin of the "RockerCoupler" link.
  // Applied when the asymmetric linkage model is loaded should keep the system
  // at static equilibrium given the initial conditions in the model's SDF file.
  if (FLAGS_apply_force) {
    std::vector<ExternallyAppliedSpatialForce<double>> forces{
        ExternallyAppliedSpatialForce<double>{
            four_bar.GetBodyByName("RockerCoupler").index(), Vector3d::Zero(),
            SpatialForce<double>(Vector3d::Zero(),
                                 Vector3d(-200.0, 0.0, 0.0))}};

    four_bar.get_applied_spatial_force_input_port().FixValue(&four_bar_context,
                                                             forces);
  }

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  systems::IntegratorBase<double>* integrator{nullptr};
  if (FLAGS_integration_scheme == "implicit_euler") {
    integrator = &simulator.reset_integrator<ImplicitEulerIntegrator<double>>();
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    integrator = &simulator.reset_integrator<RungeKutta3Integrator<double>>();
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    integrator =
        &simulator.reset_integrator<SemiExplicitEulerIntegrator<double>>(
            max_time_step);
  } else {
    throw std::runtime_error("Integration scheme '" + FLAGS_integration_scheme +
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

  // If we're simulating the asymmetric model and applying the force, then the
  // system should be at equilibrium. Verify.
  if (FLAGS_asymmetric_model && FLAGS_apply_force) {
    double KE = four_bar.EvalKineticEnergy(four_bar_context);
    DRAKE_DEMAND(KE < 1.0e-4);
  }

  // Some sanity checks:
  if (FLAGS_integration_scheme == "semi_explicit_euler") {
    DRAKE_DEMAND(integrator->get_fixed_step_mode() == true);
  }

  // Checks for variable time step integrators.
  if (!integrator->get_fixed_step_mode()) {
    // From IntegratorBase::set_maximum_step_size():
    // "The integrator may stretch the maximum step size by as much as 1% to
    // reach discrete event." Thus the 1.01 factor in this DRAKE_DEMAND.
    DRAKE_DEMAND(integrator->get_largest_step_size_taken() <=
                 1.01 * max_time_step);
    DRAKE_DEMAND(integrator->get_smallest_adapted_step_size_taken() <=
                 integrator->get_largest_step_size_taken());
    DRAKE_DEMAND(integrator->get_num_steps_taken() >=
                 simulation_time / max_time_step);
  }

  // Checks for fixed time step integrators.
  if (integrator->get_fixed_step_mode()) {
    DRAKE_DEMAND(integrator->get_num_derivative_evaluations() ==
                 integrator->get_num_steps_taken());
    DRAKE_DEMAND(integrator->get_num_step_shrinkages_from_error_control() == 0);
  }

  // We made a good guess for max_time_step and therefore we expect no
  // failures when taking a time step.
  DRAKE_DEMAND(integrator->get_num_substep_failures() == 0);
  DRAKE_DEMAND(integrator->get_num_step_shrinkages_from_substep_failures() ==
               0);

  return 0;
}

}  // namespace
}  // namespace four_bar
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple four bar linkage demo using Drake's MultibodyPlant"
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::four_bar::do_main();
}
