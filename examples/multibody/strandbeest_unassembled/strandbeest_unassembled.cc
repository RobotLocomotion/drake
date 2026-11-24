/* @file
A Strandbeest demo demonstrating MultibodyPlant's automatic modeling of closed
kinematic loops, applied to a mechanism described in an SDFormat file as
unassembled parts. Each leg, described in model/JansenParts.xacro, is Theo
Jansen's linkage built from straight bars and rigid triangles joined by pins,
each part laid out from a pin along a coordinate axis, so that describing it
takes no trigonometry. Each leg's pins form three kinematic loops, which
MultibodyPlant breaks by splitting links and closing each loop with a weld
constraint. The default model, model/TwelveLegs.sdf.xacro, has twelve legs.

The legs are then assembled by optimization: AssembleNearestConfiguration()
finds the configuration nearest the unassembled one that satisfies every weld.
That function, in assemble_nearest_configuration.h, is not specific to this
example; you can copy it for your own models. Next, each leg's crank is turned
to its phase on the crankshaft by a motor that the program adds for the
purpose, in a simulation, and locked there. Finally, the crankshaft turns and
the legs walk the machine along the ground. */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/multibody/strandbeest_unassembled/assemble_nearest_configuration.h"
#include "drake/geometry/collision_filter_declaration.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using examples::strandbeest_unassembled::AssembleNearestConfiguration;
using geometry::SceneGraph;
using math::RigidTransformd;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::BodyIndex;
using multibody::Frame;
using multibody::FrameIndex;
using multibody::Joint;
using multibody::JointActuatorIndex;
using multibody::JointIndex;
using multibody::Link;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::RevoluteJoint;
using solvers::MathematicalProgramResult;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::Simulator;
using systems::TrajectorySource;
using trajectories::PiecewisePolynomial;

namespace {

DEFINE_double(simulation_time, 10.0, "Duration of the simulation in seconds.");

DEFINE_double(time_step, 5e-2,
              "Discrete time step of the MultibodyPlant in seconds. Closing "
              "the loops takes constraints, which the discrete plant's SAP "
              "solver enforces. The faster the legs move, the smaller a step "
              "it takes to keep the loops closed. (A continuous plant, 0, "
              "would need the CENIC integrator, which is far too slow for the "
              "larger models; see Drake issue #25293.)");

DEFINE_string(model, "TwelveLegs",
              "Which model in this example's model directory to run: "
              "JansenLeg (one leg), TwoLegs, FourLegs, EightLegs, "
              "TwelveLegs, SixteenLegs, or TwentyLegs.");

DEFINE_double(crank_speed, 2.0,
              "Angular velocity, in rad/s, at which the crankshaft's velocity "
              "servo turns it once the legs are assembled. Positive is "
              "counterclockwise when viewed with x to the right and z up.");

DEFINE_double(assembly_playback_time, 3.0,
              "Seconds to spend showing the legs move from their drawn "
              "configuration to their first assembled one, so that it can be "
              "watched rather than being over within a single frame. Set to 0 "
              "to skip it. This is wall clock time and is unrelated to "
              "--simulator_target_realtime_rate.");

DEFINE_bool(interactive, true,
            "Stop three times, waiting for you to press Enter before going on: "
            "with the legs as defined, unassembled; once they are assembled "
            "with their cranks lined up; and once the cranks are turned to "
            "their phases. Set --nointeractive to run start to finish without "
            "stopping.");

constexpr char kModelDirectory[] =
    "package://drake/examples/multibody/strandbeest_unassembled/model/";
constexpr char kCrankshaftJointName[] = "chassis_crankshaft";
// Each leg's crank joint is named "<prefix>crankshaft_crank".
constexpr char kCrankJointSuffix[] = "crankshaft_crank";
constexpr double kAssembledTolerance = 1.0e-3;  // meters
// The gains of the motor that turns each crank to its phase. The proportional
// gain is stiff, so that the crank sags little under the weight of its leg:
// 4 mrad at most. The derivative gain damps the servo's response.
constexpr multibody::PdControllerGains kCrankMotorGains{.p = 1000.0, .d = 20.0};
// The peak speed of the crank that turns farthest to its phase. It averages
// two thirds of that, about as fast as the crankshaft turns when the legs
// walk. Then the motors have a while to settle.
constexpr double kCrankTurningSpeed = 3.0;  // rad/s
constexpr double kCrankSettlingTime = 0.1;  // seconds

/* Owns the shared state of this example. Run() comes first and is implemented
in line so that you can begin with the example's story; implementation details
follow it. */
class StrandbeestUnassembledDemo final {
 public:
  int Run() {
    if (!(FLAGS_simulation_time > 0.0)) {
      throw std::runtime_error("--simulation_time must be positive.");
    }

    // Build the MultibodyPlant and SceneGraph.
    auto [strandbeest, scene_graph] =
        AddMultibodyPlantSceneGraph(&builder_, FLAGS_time_step);
    strandbeest_ = &strandbeest;
    scene_graph_ = &scene_graph;

    // Load the closed-topology mechanism, unassembled, along with the geometry
    // that illustrates it. See model/TwoLegs.sdf.xacro, for example.
    const std::vector<ModelInstanceIndex> models =
        Parser(&builder_).AddModelsFromUrl(
            fmt::format("{}{}.sdf", kModelDirectory, FLAGS_model));
    model_instance_ = models.at(0);

    // CENIC does not integrate the plant's own contact model, so ask for
    // sampled output ports to make the plant report the forces of CENIC's
    // solve.
    if (FLAGS_time_step == 0.0 &&
        FLAGS_simulator_integration_scheme == "cenic") {
      strandbeest_->SetUseSampledOutputPorts(true);
    }

    // Give each crank a motor for turning it to its phase.
    AddCrankMotors();

    // We are done defining the model. Splitting links, retargeting joints,
    // and adding welds happens here.
    strandbeest_->Finalize();
    IgnoreContactWithinMachine();
    // Each loop costs one shadow link and one weld constraint. The primary and
    // shadow share the original link's mass evenly.
    std::cout << fmt::format(
        "Modeled {} kinematic loop(s) with {} constraint(s).\n",
        strandbeest_->num_loop_constraints(), strandbeest_->num_constraints());

    // Command the servos: the crankshaft's and the cranks' motors. Until we
    // say otherwise, the crankshaft's velocity servo holds it still, and each
    // crank's motor holds it at zero, as the leg is drawn.
    AddServoCommands();

    visualization::AddDefaultVisualization(&builder_);
    diagram_ = builder_.Build();

    // Create a context and simulator.
    std::unique_ptr<Context<double>> diagram_context =
        diagram_->CreateDefaultContext();
    simulator_ = MakeSimulatorFromGflags(*diagram_, std::move(diagram_context));

    // We set no initial conditions. Every joint angle defaults to zero, which
    // is the unassembled configuration of the model file's parts.
    simulator_->Initialize();
    FindLegs();
    const double initial_error = CalcLoopClosureError(plant_context());
    if (!(initial_error > kAssembledTolerance)) {
      throw std::runtime_error(fmt::format(
          "Expected an unassembled model, but its loop closure error is {} m.",
          initial_error));
    }
    std::cout << fmt::format("Loop closure error as defined: {:.4f} m\n",
                             initial_error);
    ShowAndWait(
        "The legs are unassembled. Drake broke each kinematic loop by\n"
        "splitting one of its parts into two links, the part and an undrawn\n"
        "shadow, and moving one of the part's pins onto the shadow. A weld\n"
        "constraint joins each shadow to its part, but it is not satisfied\n"
        "yet: the shadow lies away from its part, so the part's hole for the\n"
        "moved pin lies away from that pin.",
        "assemble the legs");

    // Assemble the legs, with their cranks lined up as the legs are drawn.
    Assemble();
    ShowAndWait(
        "The legs are assembled, each as it is drawn, with all the cranks\n"
        "lined up. Each shadow now coincides with the part it was split\n"
        "from, so every hole is on its pin.",
        "turn the cranks to their phases");

    // Turn each leg's crank to its phase on the crankshaft and lock it there.
    TurnCranksToPhases();
    ShowAndWait("Each crank is now locked at its phase on the crankshaft.",
                "turn the crankshaft and walk");

    // Turn the crankshaft, and let the legs walk the chassis along the ground.
    const Link<double>& chassis = strandbeest_->GetBodyByName("chassis");
    const Vector3d p_WC_start =
        chassis.EvalPoseInWorld(plant_context()).translation();
    StartWalking();
    simulator_->AdvanceTo(FLAGS_simulation_time);
    std::cout << fmt::format(
        "Loop closure error after simulating {} s: {:.3g} m\n",
        FLAGS_simulation_time, CalcLoopClosureError(plant_context()));
    const Vector3d p_WC =
        chassis.EvalPoseInWorld(plant_context()).translation();
    std::cout << fmt::format(
        "The strandbeest walked {:.2f} m, and its axle is {:.3f} m up.\n",
        (p_WC - p_WC_start).head<2>().norm(), p_WC.z());
    return 0;
  }

 private:
  // What assembly needs to know about each leg. The model file names every
  // part of a leg with the leg's prefix, e.g. "leg1_".
  struct Leg {
    std::string prefix;
    // The joint about which the leg's crank turns on the crankshaft; zero
    // when the crank points away from the leg, as the leg is drawn.
    const RevoluteJoint<double>* crank_joint{};
    // The crank joint angle at which the crank is to be locked.
    double phase{};
    // Where the crank's motor is in the servo commands; see GetServo().
    int motor{};
  };

  // Assembly.
  const Context<double>& plant_context() const;
  Context<double>& mutable_plant_context();
  void AddCrankMotors();
  void IgnoreContactWithinMachine();
  std::vector<const Frame<double>*> GetFramesOn(const Link<double>& link) const;
  void AddServoCommands();
  int GetServo(const Joint<double>& joint) const;
  double CalcLoopClosureError(const Context<double>& context) const;
  void FindLegs();
  void AssembleOrThrow(const std::string& what);
  void Assemble();
  void TurnCranksToPhases();
  void StartWalking();

  // Presentation.
  void ShowAndWait(const std::string& message, const std::string& next);
  void ShowMotionFrom(const VectorXd& q_start, double duration);

  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* strandbeest_{};
  SceneGraph<double>* scene_graph_{};
  ModelInstanceIndex model_instance_;
  // Feeds the commands of the servos in model_instance_: the crankshaft's
  // velocity servo and the cranks' motors.
  TrajectorySource<double>* servo_commands_{};
  int num_servos_{};
  // Where the crankshaft's servo is in the servo commands; see GetServo().
  int crankshaft_servo_{};
  std::vector<Leg> legs_;
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Simulator<double>> simulator_;
};

const Context<double>& StrandbeestUnassembledDemo::plant_context() const {
  return strandbeest_->GetMyContextFromRoot(simulator_->get_context());
}

Context<double>& StrandbeestUnassembledDemo::mutable_plant_context() {
  return strandbeest_->GetMyMutableContextFromRoot(
      &simulator_->get_mutable_context());
}

/* Gives each leg's crank joint a motor: an actuator with PD gains, a servo
that can turn the crank to its phase. A real Strandbeest's cranks are bolted to
its crankshaft, so these motors are assembly tooling, added here rather than
described by the model file. Once a crank reaches its phase, it is locked
there, and its motor has nothing more to do. */
void StrandbeestUnassembledDemo::AddCrankMotors() {
  for (JointIndex index : strandbeest_->GetJointIndices(model_instance_)) {
    const Joint<double>& joint = strandbeest_->get_joint(index);
    if (!joint.name().ends_with(kCrankJointSuffix)) continue;
    const JointActuatorIndex motor =
        strandbeest_->AddJointActuator(joint.name() + "_motor", joint).index();
    strandbeest_->get_mutable_joint_actuator(motor).set_controller_gains(
        kCrankMotorGains);
  }
}

/* Makes the machine's parts ignore contact with one another, so that only the
ground can touch them. Each leg lies in a single plane, so its bars cross one
another and would otherwise jam. The machine is everything not welded to
World. */
void StrandbeestUnassembledDemo::IgnoreContactWithinMachine() {
  const std::vector<const Link<double>*> grounded =
      strandbeest_->GetBodiesWeldedTo(strandbeest_->world_body());
  std::vector<const Link<double>*> machine;
  for (BodyIndex index(0); index < strandbeest_->num_bodies(); ++index) {
    const Link<double>& link = strandbeest_->get_body(index);
    if (std::find(grounded.begin(), grounded.end(), &link) == grounded.end()) {
      machine.push_back(&link);
    }
  }
  scene_graph_->collision_filter_manager().Apply(
      geometry::CollisionFilterDeclaration().ExcludeWithin(
          strandbeest_->CollectRegisteredGeometries(machine)));
}

/* Returns every frame fixed to `link`, its own link frame first. That includes
any frames that Finalize() added. */
std::vector<const Frame<double>*> StrandbeestUnassembledDemo::GetFramesOn(
    const Link<double>& link) const {
  std::vector<const Frame<double>*> frames{&link.body_frame()};
  for (FrameIndex index(0); index < strandbeest_->num_frames(); ++index) {
    const Frame<double>& frame = strandbeest_->get_frame(index);
    if (&frame.link() == &link && !frame.is_link_frame()) {
      frames.push_back(&frame);
    }
  }
  return frames;
}

/* Adds the source of the servos' commands, which feeds the plant's desired
state input for model_instance_. Each servo's command is a desired angle that
follows a trajectory, along with its rate of change, the desired angular
velocity. Until told otherwise, the source commands every servo to hold still
at zero. */
void StrandbeestUnassembledDemo::AddServoCommands() {
  num_servos_ = strandbeest_->num_actuators(model_instance_);
  crankshaft_servo_ =
      GetServo(strandbeest_->GetJointByName(kCrankshaftJointName));
  servo_commands_ = builder_.AddSystem<TrajectorySource<double>>(
      PiecewisePolynomial<double>(VectorXd::Zero(num_servos_)),
      /* output_derivative_order = */ 1);
  builder_.Connect(servo_commands_->get_output_port(),
                   strandbeest_->get_desired_state_input_port(model_instance_));
}

/* Returns where the servo that turns `joint` is in the servo commands: the
index of its desired angle among the num_servos_ desired angles, which the
desired angular velocities follow in the same order. The plant orders a model
instance's servos by their actuators' indices. */
int StrandbeestUnassembledDemo::GetServo(const Joint<double>& joint) const {
  const std::vector<JointActuatorIndex> servos =
      strandbeest_->GetJointActuatorIndices(model_instance_);
  for (int i = 0; i < std::ssize(servos); ++i) {
    if (&strandbeest_->get_joint_actuator(servos[i]).joint() == &joint) {
      return i;
    }
  }
  throw std::logic_error(fmt::format("Joint '{}' has no servo.", joint.name()));
}

/* Returns the loop closure error, in meters: how far the shadow links are from
coinciding with the links they were split from, as the largest distance between
where a primary and its shadow put any of the primary's frames. Every pin is
always connected, and so is every weld that the model file asked for; only the
weld constraints that join each shadow to its primary can be unsatisfied, and
this measures how far from satisfied they are. */
double StrandbeestUnassembledDemo::CalcLoopClosureError(
    const Context<double>& context) const {
  double error = 0.0;
  // Finalize() numbers the shadow links after all of the user's links.
  for (BodyIndex index(strandbeest_->num_user_bodies());
       index < strandbeest_->num_bodies(); ++index) {
    const Link<double>& shadow = strandbeest_->get_body(index);
    const Link<double>& primary = shadow.primary_link();
    const RigidTransformd X_PS = strandbeest_->CalcRelativeTransform(
        context, primary.body_frame(), shadow.body_frame());
    // The primary's frames F; for a bar, its origin and its two holes. The
    // shadow has a copy of each, at the same position p_PF in its own link
    // frame S, which is X_PS * p_PF in the primary's link frame P.
    for (const Frame<double>* frame : GetFramesOn(primary)) {
      const Vector3d p_PF = frame->GetFixedPoseInBodyFrame().translation();
      error = std::max(error, (X_PS * p_PF - p_PF).norm());
    }
  }
  return error;
}

/* Finds the legs in the model, by their cranks' joints
"<prefix>crankshaft_crank". The model file marks where each crank goes with
the frame "<prefix>crank_phase" on the crankshaft. */
void StrandbeestUnassembledDemo::FindLegs() {
  const std::string kCrankJoint = kCrankJointSuffix;
  for (JointIndex index : strandbeest_->GetJointIndices()) {
    const std::string& name = strandbeest_->get_joint(index).name();
    if (!name.ends_with(kCrankJoint)) continue;
    Leg leg{.prefix = name.substr(0, name.size() - kCrankJoint.size()),
            .crank_joint = &strandbeest_->GetJointByName<RevoluteJoint>(name)};

    // The phase is the angle, about the crank joint's axis, from the joint's
    // frame on the crankshaft to the phase marker frame. Both are on the
    // crankshaft, so this doesn't depend on the configuration.
    const math::RotationMatrixd R_JpP =
        strandbeest_
            ->CalcRelativeTransform(
                plant_context(), leg.crank_joint->frame_on_parent(),
                strandbeest_->GetFrameByName(leg.prefix + "crank_phase"))
            .rotation();
    const Eigen::AngleAxisd angle_axis = R_JpP.ToAngleAxis();
    leg.phase = angle_axis.angle() *
                angle_axis.axis().dot(leg.crank_joint->revolute_axis());
    leg.motor = GetServo(*leg.crank_joint);
    legs_.push_back(std::move(leg));
  }
}

/* Assembles the active parts of the model, holding locked joints, or throws
an error mentioning `what` was being assembled. */
void StrandbeestUnassembledDemo::AssembleOrThrow(const std::string& what) {
  const MathematicalProgramResult result =
      AssembleNearestConfiguration(*strandbeest_, &mutable_plant_context());
  if (!result.is_success()) {
    throw std::runtime_error(fmt::format(
        "Failed to assemble {}: {} reported {}.", what,
        result.get_solver_id().name(), result.get_solution_result()));
  }
}

/* Assembles the legs, with the crankshaft and every crank locked at zero, and
shows them moving there from their drawn configuration. Each leg is drawn as
it looks when its crank points away from it, which is when its crank joint is
at zero, so the nearest assembled configuration is the intended one for every
leg at once. A floating chassis is locked too, where the model file put it,
so that it waits in the air while the legs are assembled. The chassis,
crankshaft, and cranks are left locked. */
void StrandbeestUnassembledDemo::Assemble() {
  Context<double>& context = mutable_plant_context();
  const VectorXd q_unassembled = strandbeest_->GetPositions(context);

  for (BodyIndex index : strandbeest_->GetFloatingBaseBodies()) {
    strandbeest_->get_body(index).Lock(&context);
  }
  strandbeest_->GetJointByName(kCrankshaftJointName).Lock(&context);
  for (const Leg& leg : legs_) leg.crank_joint->Lock(&context);
  AssembleOrThrow("the legs with their cranks lined up");
  std::cout << fmt::format(
      "Assembled the legs with their cranks lined up;\n"
      "the loop closure error is {:.3g} m.\n",
      CalcLoopClosureError(context));
  ShowMotionFrom(q_unassembled, FLAGS_assembly_playback_time);
}

/* Unlocks each assembled leg's crank, turns it to its phase with its motor,
and locks it there, as if bolting it to the crankshaft. Meanwhile the
crankshaft and a floating chassis stay locked; afterward the crankshaft is
unlocked, and so is the chassis if there are enough legs to stand on. The
turning is simulated: each motor pulls its leg along, and the weld constraints
keep it assembled, as they would during any simulation. That is much faster
than reassembling the legs repeatedly with the optimizer. Because the motors
turn the cranks smoothly, nothing jumps, so the legs never stray far from
assembled, and an error-controlled integrator can take long steps. The cranks
stay locked from then on; only the crankshaft turns. */
void StrandbeestUnassembledDemo::TurnCranksToPhases() {
  Context<double>& context = mutable_plant_context();
  for (const Leg& leg : legs_) leg.crank_joint->Unlock(&context);

  // Until now, every servo has been commanded to hold still at zero. Command
  // each crank's motor to turn it from zero to its phase along a cubic that
  // starts and ends at rest, all together; the crank that turns farthest
  // peaks at kCrankTurningSpeed, 1.5 times its average. The new commands
  // start where the old ones are, so an output the source had computed from
  // the old ones is still right.
  MatrixXd angles = MatrixXd::Zero(num_servos_, 2);  // At the start and end.
  double farthest = 0.0;
  for (const Leg& leg : legs_) {
    angles(leg.motor, 1) = leg.phase;
    farthest = std::max(farthest, std::abs(leg.phase));
  }
  const double start_time = context.get_time();
  const double duration = 1.5 * farthest / kCrankTurningSpeed;
  if (duration > 0.0) {
    servo_commands_->UpdateTrajectory(PiecewisePolynomial<double>::CubicHermite(
        Vector2d(start_time, start_time + duration), angles,
        MatrixXd::Zero(num_servos_, 2)));
  }
  simulator_->AdvanceTo(start_time + duration + kCrankSettlingTime);

  double phase_error = 0.0;
  for (const Leg& leg : legs_) {
    phase_error = std::max(
        phase_error, std::abs(leg.crank_joint->get_angle(context) - leg.phase));
    leg.crank_joint->Lock(&context);
  }
  std::cout << fmt::format(
      "Turned the cranks to their phases, to within {:.2g} rad,\n"
      "in {:.2f} s of simulated time.\n",
      phase_error, context.get_time() - start_time);
  strandbeest_->GetJointByName(kCrankshaftJointName).Unlock(&context);
  // With fewer than eight legs, the machine can't stand, so leave its chassis
  // locked in the air, and let the legs walk in place.
  if (std::ssize(legs_) >= 8) {
    for (BodyIndex index : strandbeest_->GetFloatingBaseBodies()) {
      strandbeest_->get_body(index).Unlock(&context);
    }
  }

  const double error = CalcLoopClosureError(context);
  if (!(error < kAssembledTolerance)) {
    throw std::runtime_error(fmt::format(
        "The legs came apart while the cranks turned; the loop closure error "
        "is {} m.",
        error));
  }
}

/* Starts the clock over at zero, with the crankshaft's velocity servo
commanded to turn it at --crank_speed until --simulation_time. The servo's
proportional gain is zero, so only the rate of change of its commanded angle
matters. The cranks' motors stay commanded to their phases, though their locked
cranks no longer need them. */
void StrandbeestUnassembledDemo::StartWalking() {
  MatrixXd angles = MatrixXd::Zero(num_servos_, 2);  // At the start and end.
  for (const Leg& leg : legs_) angles.row(leg.motor).setConstant(leg.phase);
  angles(crankshaft_servo_, 1) = FLAGS_crank_speed * FLAGS_simulation_time;
  // Set the new commands before restarting the clock. Updating a
  // TrajectorySource doesn't change the Context, so an output the source had
  // computed at the current time would otherwise still be used; restarting
  // the clock makes the Context compute it again.
  servo_commands_->UpdateTrajectory(PiecewisePolynomial<double>::FirstOrderHold(
      Vector2d(0.0, FLAGS_simulation_time), angles));

  // Changing the time requires reinitializing the simulator, but suppress the
  // initialization events: the visualizers' events would delete everything
  // they've drawn and redraw it, making the screen flash. (The plant has no
  // initialization events.)
  simulator_->get_mutable_context().SetTime(0.0);
  simulator_->Initialize({.suppress_initialization_events = true});
}

// The rest of the example's code supports the pedagogical presentation. None
// of it is needed merely to model a closed topology.

constexpr double kPlaybackFps = 30.0;

/* Shows the current configuration and, unless --nointeractive, waits for the
user to press Enter before doing whatever `next` describes. */
void StrandbeestUnassembledDemo::ShowAndWait(const std::string& message,
                                             const std::string& next) {
  diagram_->ForcedPublish(simulator_->get_context());
  std::cout << fmt::format("\n{}\n", message);
  if (FLAGS_interactive) {
    std::cout << "Press Enter to " << next << " . . . " << std::flush;
    std::string line;
    std::getline(std::cin, line);  // Just eats the line; EOF is fine too.
  }
  // Do not count time spent waiting as time the simulator must make up.
  simulator_->ResetStatistics();
}

/* Shows the model moving from `q_start` to its current configuration over
`duration` seconds of wall time, then leaves it there. The optimizer reports
only where a solve ends up, not how it gets there, so this simply interpolates
the positions; the holes close in on their pins as it goes. */
void StrandbeestUnassembledDemo::ShowMotionFrom(const VectorXd& q_start,
                                                double duration) {
  Context<double>& context = mutable_plant_context();
  const VectorXd q_end = strandbeest_->GetPositions(context);
  if (duration > 0.0) {
    const int num_frames =
        std::max(2, static_cast<int>(duration * kPlaybackFps));
    const auto frame_period = std::chrono::duration<double>(1.0 / kPlaybackFps);
    for (int frame = 0; frame < num_frames; ++frame) {
      const double s = static_cast<double>(frame) / (num_frames - 1);
      strandbeest_->SetPositions(&context, q_start + s * (q_end - q_start));
      diagram_->ForcedPublish(simulator_->get_context());
      std::this_thread::sleep_for(frame_period);
    }
  }
  strandbeest_->SetPositions(&context, q_end);
}

}  // namespace
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A Strandbeest demo demonstrating MultibodyPlant's automatic modeling "
      "of closed kinematic loops, with assembly, for model files that "
      "describe the legs as unassembled parts. Use --model to choose how many "
      "legs. Open the indicated URL to see the Meshcat visualization.");
  // Changes the default realtime rate to 1X, so the visualization looks
  // realistic. Otherwise, it finishes too fast. Users can still change it on
  // command line, e.g., "--simulator_target_realtime_rate=0.5" to slow it down.
  FLAGS_simulator_target_realtime_rate = 1.0;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::StrandbeestUnassembledDemo().Run();
}
