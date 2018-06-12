#include <memory>

#include <gflags/gflags.h>
#include "fmt/ostream.h"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/primitives/sine.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {
namespace examples {
namespace simple_gripper {
namespace {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::lcm::DrakeLcm;
using drake::math::RollPitchYaw;
using drake::math::RotationMatrix;
using drake::multibody::Body;
using drake::multibody::multibody_plant::CoulombFriction;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::parsing::AddModelFromSdfFile;
using drake::multibody::PrismaticJoint;
using drake::multibody::UniformGravityFieldElement;
using drake::systems::ImplicitEulerIntegrator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::Serializer;
using drake::systems::rendering::PoseBundleToDrawMessage;
using drake::systems::RungeKutta2Integrator;
using drake::systems::RungeKutta3Integrator;
using drake::systems::SemiExplicitEulerIntegrator;
using drake::systems::Sine;

// TODO(amcastro-tri): Consider moving this large set of parameters to a
// configuration file (e.g. YAML).
DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation. [s].");

DEFINE_double(grip_width, 0.095,
              "The initial distance between the gripper fingers. [m].");

// Integration parameters:
DEFINE_string(integration_scheme, "implicit_euler",
              "Integration scheme to be used. Available options are: "
              "'semi_explicit_euler','runge_kutta2','runge_kutta3',"
              "'implicit_euler'");
DEFINE_double(max_time_step, 1.0e-3,
              "Maximum time step used for the integrators. [s]. "
              "If negative, a value based on parameter penetration_allowance "
              "is used.");
DEFINE_double(accuracy, 1.0e-2, "Sets the simulation accuracy for variable step"
              "size integrators with error control.");
DEFINE_bool(time_stepping, true, "If 'true', the plant is modeled as a "
    "discrete system with periodic updates of period 'max_time_step'."
    "If 'false', the plant is modeled as a continuous system.");

// Contact parameters
DEFINE_double(penetration_allowance, 1.0e-2,
              "Penetration allowance. [m]. "
              "See MultibodyPlant::set_penetration_allowance().");
DEFINE_double(v_stiction_tolerance, 1.0e-2,
              "The maximum slipping speed allowed during stiction. [m/s]");

// Pads parameters
DEFINE_int32(ring_samples, 8,
             "The number of spheres used to sample the pad ring");
DEFINE_double(ring_orient, 0, "Rotation of the pads around x-axis. [degrees]");
DEFINE_double(ring_static_friction, 1.0, "The coefficient of static friction "
              "for the ring pad.");
DEFINE_double(ring_dynamic_friction, 0.5, "The coefficient of dynamic friction "
              "for the ring pad.");

// Parameters for rotating the mug.
DEFINE_double(rx, 0, "The x-rotation of the mug around its origin - the center "
              "of its bottom. [degrees]. Extrinsic rotation order: X, Y, Z");
DEFINE_double(ry, 0, "The y-rotation of the mug around its origin - the center "
              "of its bottom. [degrees]. Extrinsic rotation order: X, Y, Z");
DEFINE_double(rz, 0, "The z-rotation of the mug around its origin - the center "
              "of its bottom. [degrees]. Extrinsic rotation order: X, Y, Z");

// Gripping force.
DEFINE_double(gripper_force, 10, "The force to be applied by the gripper. [N]. "
              "A value of 0 indicates a fixed grip width as set with option "
              "grip_width.");

// Parameters for shaking the mug.
DEFINE_double(amplitude, 0, "The amplitude of the harmonic oscillations "
              "carried out by the gripper. [m].");
DEFINE_double(frequency, 0, "The frequency of the harmonic oscillations "
              "carried out by the gripper. [Hz].");

// The pad was measured as a torus with the following major and minor radii.
const double kPadMajorRadius = 14e-3;  // 14 mm.
const double kPadMinorRadius = 6e-3;   // 6 mm.

// This uses the parameters of the ring to add collision geometries to a
// rigid body for a finger. The collision geometries, consisting of a set of
// small spheres, approximates a torus attached to the finger.
//
// @param[in] plant the MultiBodyPlant in which to add the pads.
// @param[in] scene_graph the associated SceneGraph.
// @param[in] pad_offset the ring offset along the x-axis in the finger
// coordinate frame, i.e., how far the ring protrudes from the center of the
// finger.
// @param[in] finger the Body representing the finger
void AddGripperPads(MultibodyPlant<double>* plant,
                    SceneGraph<double>* scene_graph,
                    const double pad_offset, const Body<double>& finger) {
  const int sample_count = FLAGS_ring_samples;
  const double sample_rotation = FLAGS_ring_orient * M_PI / 180.0;  // radians.
  const double d_theta = 2 * M_PI / sample_count;

  Vector3d p_FSo;  // Position of the sphere frame S in the finger frame F.
  // The finger frame is defined in simpler_gripper.sdf so that:
  //  - x axis pointing to the right of the gripper.
  //  - y axis pointing forward in the direction of the fingers.
  //  - z axis points up.
  //  - It's origin Fo is right at the geometric center of the finger.
  for (int i = 0; i < sample_count; ++i) {
    // The y-offset of the center of the torus in the finger frame F.
    const double torus_center_y_position_F = 0.0265;
    p_FSo(0) = pad_offset;  // Offset from the center of the gripper.
    p_FSo(1) =
        std::cos(d_theta * i + sample_rotation) * kPadMajorRadius +
            torus_center_y_position_F;
    p_FSo(2) = std::sin(d_theta * i + sample_rotation) * kPadMajorRadius;

    // Pose of the sphere frame S in the finger frame F.
    const Isometry3d X_FS = Isometry3d(Translation3d(p_FSo));

    CoulombFriction<double> friction(
        FLAGS_ring_static_friction, FLAGS_ring_static_friction);

    plant->RegisterCollisionGeometry(
        finger, X_FS, Sphere(kPadMinorRadius), friction, scene_graph);
  }
}

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  DRAKE_DEMAND(FLAGS_max_time_step > 0);

  MultibodyPlant<double>& plant =
      FLAGS_time_stepping ?
      *builder.AddSystem<MultibodyPlant>(FLAGS_max_time_step) :
      *builder.AddSystem<MultibodyPlant>();
  std::string full_name =
      FindResourceOrThrow("drake/examples/simple_gripper/simple_gripper.sdf");
  AddModelFromSdfFile(full_name, &plant, &scene_graph);

  full_name =
      FindResourceOrThrow("drake/examples/simple_gripper/simple_mug.sdf");
  AddModelFromSdfFile(full_name, &plant, &scene_graph);

  // Obtain the "translate_joint" axis so that we know the direction of the
  // forced motions. We do not apply gravity if motions are forced in the
  // vertical direction so that the gripper doesn't start free falling. See note
  // below on how we apply these motions. A better strategy would be using
  // constraints but we keep it simple for this demo.
  const PrismaticJoint<double>& translate_joint =
      plant.GetJointByName<PrismaticJoint>("translate_joint");
  const Vector3d axis = translate_joint.translation_axis();
  if (axis.isApprox(Vector3d::UnitZ())) {
    fmt::print("Gripper motions forced in the vertical direction.\n");
  } else if (axis.isApprox(Vector3d::UnitX())) {
    fmt::print("Gripper motions forced in the horizontal direction.\n");
    // Add gravity to the model.
    plant.AddForceElement<UniformGravityFieldElement>(
        -9.81 * Vector3<double>::UnitZ());
  } else {
    throw std::runtime_error(
        "Only horizontal or vertical motions of the gripper are supported for "
        "this example. The joint axis in the SDF file must either be the "
        "x-axis or the z-axis");
  }

  // Add the pads.
  const Body<double>& left_finger = plant.GetBodyByName("left_finger");
  const Body<double>& right_finger = plant.GetBodyByName("right_finger");

  // Pads offset from the center of a finger. pad_offset = 0 means the center of
  // the spheres is located right at the center of the finger.
  const double pad_offset = 0.0046;
  if (FLAGS_gripper_force == 0) {
    // We then fix everything to the right finger and leave the left finger
    // "free" with no applied forces (thus we see it not moving).
    const double finger_width = 0.007;  // From the visual in the SDF file.
    AddGripperPads(&plant, &scene_graph, -pad_offset, right_finger);
    AddGripperPads(&plant, &scene_graph,
                   -(FLAGS_grip_width + finger_width) + pad_offset,
                   right_finger);
  } else {
    AddGripperPads(&plant, &scene_graph, -pad_offset, right_finger);
    AddGripperPads(&plant, &scene_graph, +pad_offset, left_finger);
  }

  // Now the model is complete.
  plant.Finalize(&scene_graph);

  // Set how much penetration (in meters) we are willing to accept.
  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  plant.set_stiction_tolerance(FLAGS_v_stiction_tolerance);

  // If the user specifies a time step, we use that, otherwise estimate a
  // maximum time step based on the compliance of the contact model.
  // The maximum time step is estimated to resolve this time scale with at
  // least 30 time steps. Usually this is a good starting point for fixed step
  // size integrators to be stable.
  const double max_time_step =
      FLAGS_max_time_step > 0 ? FLAGS_max_time_step :
      plant.get_contact_penalty_method_time_scale() / 30;

  // Print maximum time step and the time scale introduced by the compliance in
  // the contact model as a reference to the user.
  fmt::print("Maximum time step = {:10.6f} s\n", max_time_step);
  fmt::print("Compliance time scale = {:10.6f} s\n",
             plant.get_contact_penalty_method_time_scale());

  // from simple_griper.sdf, there are two actuators. One actuator on the
  // prismatic joint named "finger_sliding_joint" to actuate the left finger and
  // a second actuator on the prismatic joint named "translate_joint" to impose
  // motions of the gripper.
  DRAKE_DEMAND(plant.num_actuators() == 2);
  DRAKE_DEMAND(plant.num_actuated_dofs() == 2);

  // Boilerplate used to connect the plant to a SceneGraph for
  // visualization.
  DrakeLcm lcm;
  const PoseBundleToDrawMessage& converter =
      *builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem& publisher =
      *builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  builder.Connect(scene_graph.get_pose_bundle_output_port(),
                  converter.get_input_port(0));
  builder.Connect(converter, publisher);

  // Sinusoidal force input. We want the gripper to follow a trajectory of the
  // form x(t) = X0 * sin(ω⋅t). By differentiating once, we can compute the
  // velocity initial condition, and by differentiating twice, we get the input
  // force we need to apply.
  // The mass of the mug is ignored.
  // TODO(amcastro-tri): add a PD controller to precisely control the
  // trajectory of the gripper. Even better, add a motion constraint when MBP
  // supports it.

  // The mass of the gripper in simple_gripper.sdf.
  // TODO(amcastro-tri): we should call MultibodyTree::CalcMass() here.
  const double mass = 1.0890;  // kg.
  const double omega = 2 * M_PI * FLAGS_frequency;  // rad/s.
  const double x0 = FLAGS_amplitude;  // meters.
  const double v0 = -x0 * omega;  // Velocity amplitude, initial velocity, m/s.
  const double a0 = omega * omega * x0;  // Acceleration amplitude, m/s².
  const double f0 = mass * a0;  // Force amplitude, Newton.
  fmt::print("Acceleration amplitude = {:8.4f} m/s²\n", a0);

  // Notice we are using the same Sine source to:
  //   1. Generate a harmonic forcing of the gripper with amplitude f0 and
  //      angular frequency omega.
  //   2. Impose a constant force to the left finger. That is, a harmonic
  //      forcing with "zero" frequency.
  const Vector2<double> amplitudes(f0, FLAGS_gripper_force);
  const Vector2<double> frequencies(omega, 0.0);
  const Vector2<double> phases(0.0, M_PI_2);
  const auto& harmonic_force = *builder.AddSystem<Sine>(
      amplitudes, frequencies, phases);

  builder.Connect(harmonic_force.get_output_port(0),
                  plant.get_actuation_input_port());

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(scene_graph);

  // And build the Diagram:
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Get joints so that we can set initial conditions.
  const PrismaticJoint<double>& finger_slider =
      plant.GetJointByName<PrismaticJoint>("finger_sliding_joint");

  // Set initial position of the left finger.
  finger_slider.set_translation(&plant_context, -FLAGS_grip_width);

  // Get mug body so we can set its initial pose.
  const Body<double>& mug = plant.GetBodyByName("main_body");

  // Initialize the mug pose to be right in the middle between the fingers.
  std::vector<Isometry3d> X_WB_all;
  plant.model().CalcAllBodyPosesInWorld(plant_context, &X_WB_all);
  const Vector3d& p_WBr = X_WB_all[right_finger.index()].translation();
  const Vector3d& p_WBl = X_WB_all[left_finger.index()].translation();
  const double mug_y_W = (p_WBr(1) + p_WBl(1)) / 2.0;

  Isometry3d X_WM;
  Vector3d rpy(FLAGS_rx * M_PI / 180,
               FLAGS_ry * M_PI / 180,
               (FLAGS_rz * M_PI / 180) + M_PI);
  X_WM.linear() = RotationMatrix<double>(RollPitchYaw<double>(rpy)).matrix();
  X_WM.translation() = Vector3d(0.0, mug_y_W, 0.0);
  plant.model().SetFreeBodyPoseOrThrow(mug, X_WM, &plant_context);

  // Set the initial height of the gripper and its initial velocity so that with
  // the applied harmonic forces it continues to move in a harmonic oscillation
  // around this initial position.
  translate_joint.set_translation(&plant_context, 0.0);
  translate_joint.set_translation_rate(&plant_context, v0);

  // Set up simulator.
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  systems::IntegratorBase<double>* integrator{nullptr};
  if (FLAGS_integration_scheme == "implicit_euler") {
    integrator =
        simulator.reset_integrator<ImplicitEulerIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "runge_kutta2") {
    integrator =
        simulator.reset_integrator<RungeKutta2Integrator<double>>(
            *diagram, max_time_step, &simulator.get_mutable_context());
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
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(FLAGS_accuracy);

  // The error controlled integrators might need to take very small time steps
  // to compute a solution to the desired accuracy. Therefore, to visualize
  // these very short transients, we publish every time step.
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);

  // Print some stats for variable time step integrators.
  fmt::print("Integrator stats:\n");
  fmt::print("Number of time steps taken = {:d}\n",
             integrator->get_num_steps_taken());
  if (!integrator->get_fixed_step_mode()) {
    fmt::print("Initial time step taken = {:10.6g} s\n",
               integrator->get_actual_initial_step_size_taken());
    fmt::print("Largest time step taken = {:10.6g} s\n",
               integrator->get_largest_step_size_taken());
    fmt::print("Smallest adapted step size = {:10.6g} s\n",
               integrator->get_smallest_adapted_step_size_taken());
  }

  return 0;
}

}  // namespace
}  // namespace simple_gripper
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "Demo used to exercise MultibodyPlant's contact modeling in a gripping "
      "scenario. SceneGraph is used for both visualization and contact "
      "handling. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::simple_gripper::do_main();
}
