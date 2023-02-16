#include <memory>
#include <string>

#include <fmt/ostream.h>
#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/sine.h"
#include "drake/visualization/visualization_config_functions.h"
namespace drake {
namespace examples {
namespace simple_gripper {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
using geometry::SceneGraph;
using geometry::Sphere;
using math::RigidTransformd;
using math::RollPitchYawd;
using multibody::Body;
using multibody::CoulombFriction;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::MultibodyPlantConfig;
using multibody::Parser;
using multibody::PrismaticJoint;
using systems::Sine;

// TODO(amcastro-tri): Consider moving this large set of parameters to a
// configuration file (e.g. YAML).
DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation. [s].");

DEFINE_double(grip_width, 0.095,
              "The initial distance between the gripper fingers. [m].");

DEFINE_double(
    mbp_discrete_update_period, 1.0E-3,
    "If this value is positive, the multibody plant is modeled as a discrete "
    "system, and the value specifies the fixed-time step period of discrete "
    "updates for the plant. If this value is zero, the plant is modeled as a "
    "continuous system. [s].");

// Contact parameters
DEFINE_double(penetration_allowance, 1.0e-2,
              "Penetration allowance. [m]. "
              "See MultibodyPlant::set_penetration_allowance().");
DEFINE_double(v_stiction_tolerance, 1.0e-2,
              "The maximum slipping speed allowed during stiction. [m/s].");

// Pads parameters
DEFINE_int32(ring_samples, 8,
             "The number of spheres used to sample the pad ring");
DEFINE_double(ring_orient, 0, "Rotation of the pads around x-axis. [degrees]");
DEFINE_double(ring_static_friction, 1.0,
              "The coefficient of static friction "
              "for the ring pad.");
DEFINE_double(ring_dynamic_friction, 0.5,
              "The coefficient of dynamic friction "
              "for the ring pad.");

// Parameters for rotating the mug.
DEFINE_double(rx, 0,
              "The x-rotation of the mug around its origin - the center "
              "of its bottom. [degrees]. Extrinsic rotation order: X, Y, Z");
DEFINE_double(ry, 0,
              "The y-rotation of the mug around its origin - the center "
              "of its bottom. [degrees]. Extrinsic rotation order: X, Y, Z");
DEFINE_double(rz, 0,
              "The z-rotation of the mug around its origin - the center "
              "of its bottom. [degrees]. Extrinsic rotation order: X, Y, Z");

// Grip force. The amount of force we want to apply on the manipuland.
DEFINE_double(grip_force, 10,
              "The force to be applied by the gripper. [N]. "
              "A value of 0 indicates a fixed grip width as set with option "
              "grip_width.");

// Parameters for shaking the mug.
DEFINE_double(amplitude, 0.15,
              "The amplitude of the harmonic oscillations "
              "carried out by the gripper. [m].");
DEFINE_double(frequency, 2.0,
              "The frequency of the harmonic oscillations "
              "carried out by the gripper. [Hz].");

DEFINE_string(discrete_solver, "sap",
              "Discrete contact solver. Options are: 'tamsi', 'sap'.");
DEFINE_double(
    coupler_gear_ratio, -1.0,
    "When using SAP, the left finger's position qₗ is constrained to qₗ = "
    "ρ⋅qᵣ, where qᵣ is the right finger's position and ρ is this "
    "coupler_gear_ration parameter (dimensionless). If TAMSI used, "
    "then the right finger is locked, only the left finger moves and this "
    "parameter is ignored.");

// The pad was measured as a torus with the following major and minor radii.
const double kPadMajorRadius = 14e-3;  // 14 mm.
const double kPadMinorRadius = 6e-3;   // 6 mm.

// This uses the parameters of the ring to add collision geometries to a
// rigid body for a finger. The collision geometries, consisting of a set of
// small spheres, approximates a torus attached to the finger.
//
// @param[in] plant the MultiBodyPlant in which to add the pads.
// @param[in] pad_offset the ring offset along the x-axis in the finger
// coordinate frame, i.e., how far the ring protrudes from the center of the
// finger.
// @param[in] finger the Body representing the finger
void AddGripperPads(MultibodyPlant<double>* plant, const double pad_offset,
                    const Body<double>& finger) {
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
    p_FSo(1) = std::cos(d_theta * i + sample_rotation) * kPadMajorRadius +
               torus_center_y_position_F;
    p_FSo(2) = std::sin(d_theta * i + sample_rotation) * kPadMajorRadius;

    // Pose of the sphere frame S in the finger frame F.
    const RigidTransformd X_FS(p_FSo);

    CoulombFriction<double> friction(FLAGS_ring_static_friction,
                                     FLAGS_ring_static_friction);

    plant->RegisterCollisionGeometry(finger, X_FS, Sphere(kPadMinorRadius),
                                     "collision" + std::to_string(i), friction);

    const Vector4<double> red(0.8, 0.2, 0.2, 1.0);
    plant->RegisterVisualGeometry(finger, X_FS, Sphere(kPadMinorRadius),
                                  "visual" + std::to_string(i), red);
  }
}

int do_main() {
  systems::DiagramBuilder<double> builder;

  DRAKE_DEMAND(FLAGS_simulator_max_time_step > 0);
  DRAKE_DEMAND(FLAGS_mbp_discrete_update_period >= 0);

  MultibodyPlantConfig plant_config;
  plant_config.time_step = FLAGS_mbp_discrete_update_period;
  plant_config.discrete_contact_solver = FLAGS_discrete_solver;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlant(plant_config, &builder);

  Parser parser(&plant);
  parser.AddModelsFromUrl(
      "package://drake/examples/simple_gripper/simple_gripper.sdf");
  parser.AddModelsFromUrl(
      "package://drake/examples/simple_gripper/simple_mug.sdf");

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
    plant.mutable_gravity_field().set_gravity_vector(Vector3d::Zero());
  } else if (axis.isApprox(Vector3d::UnitX())) {
    fmt::print("Gripper motions forced in the horizontal direction.\n");
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
  if (FLAGS_grip_force == 0) {
    // We then fix everything to the right finger and leave the left finger
    // "free" with no applied forces (thus we see it not moving).
    const double finger_width = 0.007;  // From the visual in the SDF file.
    AddGripperPads(&plant, -pad_offset, right_finger);
    AddGripperPads(&plant, -(FLAGS_grip_width + finger_width) + pad_offset,
                   right_finger);
  } else {
    AddGripperPads(&plant, -pad_offset, right_finger);
    AddGripperPads(&plant, +pad_offset, left_finger);
  }

  // Get joints so that we can set initial conditions.
  const PrismaticJoint<double>& left_slider =
      plant.GetJointByName<PrismaticJoint>("left_slider");
  const PrismaticJoint<double>& right_slider =
      plant.GetJointByName<PrismaticJoint>("right_slider");

  // TAMSI does not support general constraints. If using TAMSI, we simplify the
  // model to have the right finger locked.
  if (FLAGS_discrete_solver != "tamsi") {
    plant.AddCouplerConstraint(left_slider, right_slider,
                               FLAGS_coupler_gear_ratio);
  }

  // Now the model is complete.
  plant.Finalize();

  // Set how much penetration (in meters) we are willing to accept.
  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  plant.set_stiction_tolerance(FLAGS_v_stiction_tolerance);

  // If the user specifies a time step, we use that, otherwise estimate a
  // maximum time step based on the compliance of the contact model.
  // The maximum time step is estimated to resolve this time scale with at
  // least 30 time steps. Usually this is a good starting point for fixed step
  // size integrators to be stable.
  const double max_time_step =
      FLAGS_simulator_max_time_step > 0
          ? FLAGS_simulator_max_time_step
          : plant.get_contact_penalty_method_time_scale() / 30;

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

  visualization::AddDefaultVisualization(&builder);

  // Sinusoidal force input. We want the gripper to follow a trajectory of the
  // form x(t) = X0 * sin(ω⋅t). By differentiating once, we can compute the
  // velocity initial condition, and by differentiating twice, we get the input
  // force we need to apply.
  // The mass of the mug is ignored.
  // TODO(amcastro-tri): add a PD controller to precisely control the
  // trajectory of the gripper. Even better, add a motion constraint when MBP
  // supports it.

  // The mass of the gripper in simple_gripper.sdf.
  // TODO(amcastro-tri): we should call MultibodyPlant::CalcMass() here.
  const double mass = 1.0890;                       // kg.
  const double omega = 2 * M_PI * FLAGS_frequency;  // rad/s.
  const double x0 = FLAGS_amplitude;                // meters.
  const double v0 = -x0 * omega;  // Velocity amplitude, initial velocity, m/s.
  const double a0 = omega * omega * x0;  // Acceleration amplitude, m/s².
  const double f0 = mass * a0;           // Force amplitude, Newton.
  fmt::print("Acceleration amplitude = {:8.4f} m/s²\n", a0);

  // The actuation force that must be applied in order to attain a given grip
  // force depends on how the gripper is modeled.
  // When we model the gripper as having the right finger locked and only the
  // actuated left finger moves, it is clear that the actuation force directly
  // balances the grip force (as done when usint the TAMSI solver).
  // When we model the gripper with two moving fingers coupled to move together
  // with a constraint, the relationship is not as obvious. We can think of the
  // constrained mechanism as a system of pulleys and therefore we'd need to
  // consider a free-body diagram to find the relationship between forces. A
  // much simpler approach however is to consider virtual displacements. When
  // the object is in a steady grasp between the fingers, there is an equal and
  // opposite force on each finger of magnitude equal to the grip force, G (or
  // otherwise the mug would move). A small motion δqᵣ of the right finger
  // then leads to a motion δqₗ = ρ⋅δqᵣ of the left finger. If U is the
  // actuation on the right finger, the virtual work due to U is δWu = U⋅δqᵣ.
  // The virtual work due to the contact forces (G on each finger) is δWg =
  // -G⋅δqₗ + G⋅δqᵣ = G⋅(1-ρ)⋅δqᵣ. In equilibrium we must have δWu = δWg for
  // arbitrary δqᵣ and therefore we find U = G⋅(1-ρ). For instance, when ρ = -1
  // (fingers move in opposition) we find that U = 2 G and thus we must apply
  // twice as much force as the grip force. This makes sense if we consider a
  // system of pulleys for this mechanism.
  const double grip_actuation_force =
      FLAGS_discrete_solver == "tamsi"
          ? FLAGS_grip_force
          : (1.0 - FLAGS_coupler_gear_ratio) * FLAGS_grip_force;

  // Notice we are using the same Sine source to:
  //   1. Generate a harmonic forcing of the gripper with amplitude f0 and
  //      angular frequency omega.
  //   2. Impose a constant force to the left finger. That is, a harmonic
  //      forcing with "zero" frequency.
  const Vector2d amplitudes(f0, grip_actuation_force);
  const Vector2d frequencies(omega, 0.0);
  const Vector2d phases(0.0, M_PI_2);
  const auto& harmonic_force =
      *builder.AddSystem<Sine>(amplitudes, frequencies, phases);

  builder.Connect(harmonic_force.get_output_port(0),
                  plant.get_actuation_input_port());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set initial position of the fingers.
  const double finger_offset = FLAGS_grip_width / 2.0;
  left_slider.set_translation(&plant_context, -finger_offset);
  right_slider.set_translation(&plant_context, finger_offset);

  // Initialize the mug pose to be right in the middle between the fingers.
  const multibody::Body<double>& mug = plant.GetBodyByName("simple_mug");
  const Vector3d& p_WBr =
      plant.EvalBodyPoseInWorld(plant_context, right_finger).translation();
  const Vector3d& p_WBl =
      plant.EvalBodyPoseInWorld(plant_context, left_finger).translation();
  const double mug_y_W = (p_WBr(1) + p_WBl(1)) / 2.0;

  RigidTransformd X_WM(
      RollPitchYawd(FLAGS_rx * M_PI / 180, FLAGS_ry * M_PI / 180,
                    (FLAGS_rz * M_PI / 180) + M_PI),
      Vector3d(0.0, mug_y_W, 0.0));
  plant.SetFreeBodyPose(&plant_context, mug, X_WM);

  // Set the initial height of the gripper and its initial velocity so that with
  // the applied harmonic forces it continues to move in a harmonic oscillation
  // around this initial position.
  translate_joint.set_translation(&plant_context, 0.0);
  translate_joint.set_translation_rate(&plant_context, v0);

  if (FLAGS_discrete_solver == "tamsi") {
    drake::log()->warn(
        "discrete_solver = 'tamsi'. Since TAMSI does not support coupler "
        "constraints to model the coupling of the fingers, this simple example "
        "locks the right finger and only the left finger is allowed to move.");
    right_slider.Lock(&plant_context);
  }

  // Set up simulator.
  auto simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));

  simulator->AdvanceTo(FLAGS_simulation_time);

  // Add a line break before simulator statistics.
  fmt::print("\n");

  systems::PrintSimulatorStatistics(*simulator);
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
      "Launch meldis before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::simple_gripper::do_main();
}
