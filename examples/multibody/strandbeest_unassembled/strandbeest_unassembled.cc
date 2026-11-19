/* @file
A Strandbeest leg demo demonstrating MultibodyPlant's automatic modeling of
closed kinematic loops, applied to a mechanism described in an SDFormat file as
unassembled parts. The leg, described in model/JansenLeg.sdf.xacro, is Theo
Jansen's linkage built entirely from straight bars and pins, each bar laid out
along a coordinate axis, so that describing it takes no trigonometry. Its pins
form five kinematic loops, which MultibodyPlant breaks by splitting links and
closing each loop with a weld constraint.

The leg is then assembled by optimization: AssembleNearestConfiguration() finds
the configuration nearest the unassembled one that satisfies every weld. That
function is not specific to this example; you can copy it for your own models.
Finally, the crank drives the assembled leg through its stride.

For pedagogical purposes, the example also visualizes the shadow links that
Drake creates to model the loops; a shadow does not exist until the model is
finalized, so it cannot be drawn by the model file. Refer to README.md for more
details. */

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

#include "drake/common/drake_throw.h"
#include "drake/geometry/collision_filter_declaration.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"
#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using geometry::Cylinder;
using geometry::FrameId;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::Rgba;
using geometry::Role;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using geometry::Shape;
using geometry::SourceId;
using geometry::Sphere;
using math::RigidTransformd;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::BodyIndex;
using multibody::Frame;
using multibody::FrameIndex;
using multibody::Joint;
using multibody::JointIndex;
using multibody::Link;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::PositionConstraint;
using multibody::RevoluteJoint;
using solvers::MathematicalProgram;
using solvers::MathematicalProgramResult;
using solvers::VectorXDecisionVariable;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::Simulator;

namespace {

DEFINE_double(simulation_time, 10.0, "Duration of the simulation in seconds.");

DEFINE_double(time_step, 1.0e-3,
              "Discrete time step of the MultibodyPlant in seconds, or 0 for a "
              "continuous plant. Closing the loops takes constraints, so this "
              "needs a solver that supports them: discrete SAP, which is the "
              "default here, or the continuous CENIC integrator, via "
              "--time_step=0 --simulator_integration_scheme=cenic.");

DEFINE_string(model, "TwelveLegs",
              "Which model in this example's model directory to run: "
              "JansenLeg (one leg), TwoLegs, FourLegs, EightLegs, or "
              "TwelveLegs.");

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
            "Show the legs as defined and then again once they are assembled, "
            "waiting for you to press Enter before going on each time. Set "
            "--nointeractive to run start to finish without stopping.");

DEFINE_bool(show_shadows, false,
            "Draw the shadow links that Drake creates to model the kinematic "
            "loops, each as a translucent, fatter copy of the bar it was split "
            "from, carrying the pin that was moved onto it. By default, just "
            "the bars and pins are drawn, as the model file describes them.");

constexpr char kModelDirectory[] =
    "package://drake/examples/multibody/strandbeest_unassembled/model/";
constexpr char kCrankshaftJointName[] = "chassis_crankshaft";
constexpr double kAssembledTolerance = 1.0e-3;  // meters
// How fast the cranks turn to their phases, about as fast as the crankshaft
// turns when the legs walk, and how long the legs then have to settle.
constexpr double kCrankTurningSpeed = 2.0;  // rad/s
constexpr double kSettlingTime = 0.05;      // seconds

/* Assembles a model whose constraints are not yet satisfied, by moving the
generalized positions q in `plant_context` to the nearest configuration (the
smallest sum of squared changes to q) at which every active weld constraint in
`plant` is satisfied. That includes the welds that Finalize() adds to close
kinematic loops, as well as any you added yourself. Locked joints stay where
they are, joints stay within their position limits, and quaternions stay unit
length. The welds are satisfied to within about `tolerance` meters.

A mechanism with kinematic loops can usually be assembled in more than one way.
This function finds the way nearest the starting configuration, so describe
your model so that it starts near the configuration you intend.

Returns the solver's result. On failure, the positions in `plant_context` are
left as they were.

@throws std::exception if `plant` has constraints other than welds. Satisfying
the other kinds too is future work.

This is defined below the example's own code, in a section of its own. */
MathematicalProgramResult AssembleNearestConfiguration(
    const MultibodyPlant<double>& plant, Context<double>* plant_context,
    double tolerance = 1.0e-6);

/* Moves locked joints of an assembled model to new positions, keeping it
assembled, by simulating it while moving them a little each time step. The
joints stay locked, so the solver takes each small move as given, and pulls the
rest of the model along to keep its constraints satisfied, as it would while
simulating anything else. Because the model never strays far from assembled, it
stays assembled the way it was.

`plant` is simulated by `simulator`, which must already be initialized.
`targets` gives each joint to move, which must have a single position and be
locked, and the position to move it to. They all move together, the farthest
at `max_speed` (position units per second of simulated time). Then the
simulation goes on for `settling_time` seconds more, to let the solver correct
the small constraint errors the last moves left, and finally the velocities are
set to zero. The simulator's clock advances by the time all this takes; if it is
paced in real time and visualized, you can watch.

This is defined below the example's own code, with the function above. */
void SimulateWhileMovingJoints(
    const MultibodyPlant<double>& plant, Simulator<double>* simulator,
    const std::vector<std::pair<const Joint<double>*, double>>& targets,
    double max_speed, double settling_time);

/* Owns the shared state of this example. Run() comes first and is implemented
in line so that you can begin with the example's story; implementation details
follow it. */
class StrandbeestUnassembledDemo final {
 public:
  int Run() {
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

    // We are done defining the model. Splitting links, retargeting joints,
    // and adding welds happens here.
    strandbeest_->Finalize();
    IgnoreContactWithinMachine();
    ReportAndIllustrateModeledLoops();

    visualization::AddDefaultVisualization(&builder_);
    diagram_ = builder_.Build();

    // Create a context and simulator.
    std::unique_ptr<Context<double>> diagram_context =
        diagram_->CreateDefaultContext();
    simulator_ = MakeSimulatorFromGflags(*diagram_, std::move(diagram_context));

    // The crankshaft's velocity servo holds it still until the legs are
    // assembled.
    SetCrankSpeed(0.0);

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
        std::string(
            "The legs are unassembled. Drake broke each kinematic loop by "
            "splitting one of its bars into two links, the bar and a shadow, "
            "and moving one of the bar's pins onto the shadow. Every "
            "pin is connected, but each shadow lies away from its bar, so the "
            "weld constraint that joins the two is not satisfied yet.") +
            (FLAGS_show_shadows
                 ? ""
                 : " (Run with --show_shadows to see the shadows.)"),
        "assemble the legs");

    // Assemble the legs, with their cranks lined up as the legs are drawn.
    Assemble();
    ShowAndWait(
        "The legs are assembled, each as it is drawn, with all the cranks "
        "lined up. Each shadow now surrounds the bar it was split from, "
        "together reconstituting the original bar's properties.",
        "turn the cranks to their phases");

    // Turn each leg's crank to its phase on the crankshaft and lock it there.
    TurnCranksToPhases();
    PrintPinLocations();
    ShowAndWait("Each crank is now locked at its phase on the crankshaft.",
                "turn the crankshaft and walk");

    // Turn the crankshaft, and let the legs walk the chassis along the ground.
    const Link<double>& chassis = strandbeest_->GetBodyByName("chassis");
    const Vector3d p_WC_start =
        chassis.EvalPoseInWorld(plant_context()).translation();
    SetCrankSpeed(FLAGS_crank_speed);
    simulator_->AdvanceTo(FLAGS_simulation_time);
    std::cout << fmt::format(
        "Loop closure error after simulating {} s: {:.3g} m\n",
        FLAGS_simulation_time, CalcLoopClosureError(plant_context()));
    const Vector3d p_WC =
        chassis.EvalPoseInWorld(plant_context()).translation();
    std::cout << fmt::format(
        "The strandbeest walked {:.2f} m, and its axle is {:.3f} m up.\n",
        (p_WC - p_WC_start).head<2>().norm(), p_WC.z());
    PrintSimulatorStatistics(*simulator_);
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
  };

  // Assembly.
  const Context<double>& plant_context() const;
  Context<double>& mutable_plant_context();
  void IgnoreContactWithinMachine();
  void SetCrankSpeed(double speed);
  double CalcLoopClosureError(const Context<double>& context) const;
  void FindLegs();
  void AssembleOrThrow(const std::string& what);
  void Assemble();
  void TurnCranksToPhases();

  // Presentation.
  void ReportAndIllustrateModeledLoops();
  void PrintPinLocations() const;
  void ShowAndWait(const std::string& message, const std::string& next);
  void ShowMotionFrom(const VectorXd& q_start, double duration);

  // Low-level illustration support.
  std::vector<const Frame<double>*> GetFramesOn(const Link<double>& link) const;
  void DrawShadowLink(const Link<double>& shadow, const std::string& held_hole);
  void HidePinVisual(const Link<double>& link, const std::string& hole);

  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* strandbeest_{};
  SceneGraph<double>* scene_graph_{};
  ModelInstanceIndex model_instance_;
  // A link that Finalize() split in two to break a kinematic loop.
  struct SplitLink {
    const Link<double>* primary{};
    const Link<double>* shadow{};
    // The positions of the primary's frames in its link frame P; for a bar,
    // its origin and its two holes. The shadow has copies of them at the same
    // positions in its own link frame.
    std::vector<Vector3d> p_PF;
    // The primary's frame for the end of the joint that Finalize() moved onto
    // the shadow; for a bar, one of its holes.
    const Frame<double>* held_hole{};
  };
  std::vector<SplitLink> split_links_;
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

/* Sets the desired angular velocity of the crankshaft's velocity servo. The
servo has no proportional gain, so the desired angle is ignored. */
void StrandbeestUnassembledDemo::SetCrankSpeed(double speed) {
  strandbeest_->get_desired_state_input_port(model_instance_)
      .FixValue(&mutable_plant_context(), Vector2d(0.0, speed));
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
  for (const SplitLink& split : split_links_) {
    const RigidTransformd X_PS = strandbeest_->CalcRelativeTransform(
        context, split.primary->body_frame(), split.shadow->body_frame());
    for (const Vector3d& p_PF : split.p_PF) {
      // The shadow's copy of F is at p_PF in S, which is X_PS * p_PF in P.
      error = std::max(error, (X_PS * p_PF - p_PF).norm());
    }
  }
  return error;
}

/* Finds the legs in the model, by their cranks' joints
"<prefix>crankshaft_crank". The model file marks where each crank goes with
the frame "<prefix>crank_phase" on the crankshaft. */
void StrandbeestUnassembledDemo::FindLegs() {
  const std::string kCrankJoint = "crankshaft_crank";
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
      "Assembled the legs with their cranks lined up; the loop closure error "
      "is {:.3g} m.\n",
      CalcLoopClosureError(context));
  ShowMotionFrom(q_unassembled, FLAGS_assembly_playback_time);
}

/* Turns each assembled leg's crank, still locked, to its phase, as if bolting
it to the crankshaft there, then unlocks the crankshaft and lets go of a
floating chassis. The turning is
simulated, so the legs stay assembled as they would during any simulation; that
is much faster than reassembling them repeatedly with the optimizer. The cranks
stay locked from then on; only the crankshaft turns. */
void StrandbeestUnassembledDemo::TurnCranksToPhases() {
  Context<double>& context = mutable_plant_context();
  std::vector<std::pair<const Joint<double>*, double>> phases;
  for (const Leg& leg : legs_) phases.emplace_back(leg.crank_joint, leg.phase);
  const double start_time = simulator_->get_context().get_time();
  SimulateWhileMovingJoints(*strandbeest_, simulator_.get(), phases,
                            kCrankTurningSpeed, kSettlingTime);
  std::cout << fmt::format(
      "Turned the cranks to their phases in {:.2f} s of simulated time.\n",
      simulator_->get_context().get_time() - start_time);
  strandbeest_->GetJointByName(kCrankshaftJointName).Unlock(&context);
  for (BodyIndex index : strandbeest_->GetFloatingBaseBodies()) {
    strandbeest_->get_body(index).Unlock(&context);
  }

  const double error = CalcLoopClosureError(context);
  if (!(error < kAssembledTolerance)) {
    throw std::runtime_error(fmt::format(
        "The legs came apart while the cranks turned; the loop closure error "
        "is {} m.",
        error));
  }

  // Start the simulation from here, at time zero.
  simulator_->get_mutable_context().SetTime(0.0);
  simulator_->Initialize();
}

// The rest of the example's code supports the pedagogical presentation. None
// of it is needed merely to model a closed topology.

constexpr double kShadowAlpha = 0.33;
constexpr double kPlaybackFps = 30.0;
constexpr double kShadowFattening = 1.5;

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

/* Returns a copy of `shape` that is kShadowFattening times as fat: a cylinder's
or sphere's radius grows, but a cylinder's length does not. The model file uses
no other shapes; they are copied unchanged. */
std::unique_ptr<Shape> MakeFatter(const Shape& shape) {
  if (const auto* cylinder = dynamic_cast<const Cylinder*>(&shape)) {
    return std::make_unique<Cylinder>(kShadowFattening * cylinder->radius(),
                                      cylinder->length());
  }
  if (const auto* sphere = dynamic_cast<const Sphere*>(&shape)) {
    return std::make_unique<Sphere>(kShadowFattening * sphere->radius());
  }
  return shape.Clone();
}

/* Draws `shadow` as a faint, fatter copy of the bar it was split from, by
copying that bar's illustration geometry, except for the pins at its holes. The
fatter shadow surrounds its bar once the legs are assembled, rather than hiding
inside it, and is drawn in the bar's colors, but translucent. The shadow is
connected only by the one pin that Finalize() moved onto it, at its hole
`held_hole`, and by the weld to its primary at their origins, so the shadow
shows only that pin. A hole at its other end will lie on another pin once the
legs are assembled, but it is not connected there. The model file names each
hole's pin visual "<hole>_pin".

A shadow does not exist until Finalize(), so it cannot be drawn by the model
file, and its geometry must be registered directly with SceneGraph rather than
through MultibodyPlant::RegisterVisualGeometry(). */
void StrandbeestUnassembledDemo::DrawShadowLink(const Link<double>& shadow,
                                                const std::string& held_hole) {
  const SceneGraphInspector<double>& inspector =
      scene_graph_->model_inspector();
  const SourceId source_id = strandbeest_->get_source_id().value();
  const FrameId primary_frame_id =
      strandbeest_->GetBodyFrameIdOrThrow(shadow.primary_link().index());
  const FrameId shadow_frame_id =
      strandbeest_->GetBodyFrameIdOrThrow(shadow.index());
  for (const GeometryId id :
       inspector.GetGeometries(primary_frame_id, Role::kIllustration)) {
    const std::string& name = inspector.GetName(id);
    if (name.ends_with("_pin") && !name.ends_with(held_hole + "_pin")) {
      continue;
    }
    const IllustrationProperties* properties =
        inspector.GetIllustrationProperties(id);
    const Rgba color = properties->GetPropertyOrDefault(
        "phong", "diffuse", Rgba(0.9, 0.9, 0.9, 1.0));
    auto instance = std::make_unique<GeometryInstance>(
        inspector.GetPoseInFrame(id), MakeFatter(inspector.GetShape(id)),
        fmt::format("{}_shadow", name));
    instance->set_illustration_properties(
        geometry::MakePhongIllustrationProperties(
            Vector4d(color.r(), color.g(), color.b(), kShadowAlpha)));
    scene_graph_->RegisterGeometry(source_id, shadow_frame_id,
                                   std::move(instance));
  }
}

void StrandbeestUnassembledDemo::ReportAndIllustrateModeledLoops() {
  // Each loop costs one shadow link and one weld constraint. The primary and
  // shadow share the original link's mass evenly.
  std::cout << fmt::format(
      "Modeled {} kinematic loop(s) with {} constraint(s).\n",
      strandbeest_->num_loop_constraints(), strandbeest_->num_constraints());
  // Finalize() numbers the shadow links after all of the user's links.
  for (BodyIndex index(strandbeest_->num_user_bodies());
       index < strandbeest_->num_bodies(); ++index) {
    const Link<double>& shadow = strandbeest_->get_body(index);
    const Link<double>& primary = shadow.primary_link();
    SplitLink split{.primary = &primary, .shadow = &shadow};
    for (const Frame<double>* frame : GetFramesOn(primary)) {
      split.p_PF.push_back(frame->GetFixedPoseInBodyFrame().translation());
    }

    // Besides its link frame, a shadow has just one frame: the copy, made by
    // Finalize(), of the primary's frame for the end of the pin it moved onto
    // the shadow. The copy is where the original is, so find which of the
    // primary's holes it copies.
    for (const Frame<double>* shadow_frame : GetFramesOn(shadow)) {
      if (shadow_frame->is_link_frame()) continue;
      const Vector3d p_SH =
          shadow_frame->GetFixedPoseInBodyFrame().translation();
      for (const Frame<double>* frame : GetFramesOn(primary)) {
        if (!frame->is_link_frame() &&
            frame->GetFixedPoseInBodyFrame().translation() == p_SH) {
          split.held_hole = frame;
        }
      }
    }
    DRAKE_THROW_UNLESS(split.held_hole != nullptr);

    if (FLAGS_show_shadows) DrawShadowLink(shadow, split.held_hole->name());
    split_links_.push_back(std::move(split));
  }

  // Without the shadows, leave the model file's geometry as it is.
  if (!FLAGS_show_shadows) return;

  // Now that the shadows have copied them, stop drawing the primaries' pins
  // that moved to the shadows: a hole whose joints all moved no longer
  // connects its bar to anything. (Only one joint moves to each shadow, so a
  // hole that also holds other joints stays connected.)
  for (const SplitLink& split : split_links_) {
    const Frame<double>& hole = *split.held_hole;
    int joints_at_hole = 0;
    for (JointIndex index : strandbeest_->GetJointIndices()) {
      const Joint<double>& joint = strandbeest_->get_joint(index);
      if (&joint.frame_on_parent() == &hole ||
          &joint.frame_on_child() == &hole) {
        ++joints_at_hole;
      }
    }
    const auto joints_moved =
        std::count_if(split_links_.begin(), split_links_.end(),
                      [&hole](const SplitLink& other) {
                        return other.held_hole == &hole;
                      });
    if (joints_at_hole == joints_moved) {
      HidePinVisual(*split.primary, hole.name());
    }
  }
}

/* Stops illustrating `link`'s pin visual at its hole `hole`, if it is still
illustrated. The model file names each hole's pin visual "<hole>_pin". Only the
visual's illustration role is removed; the geometry itself still belongs to
MultibodyPlant, which registered it. */
void StrandbeestUnassembledDemo::HidePinVisual(const Link<double>& link,
                                               const std::string& hole) {
  const SceneGraphInspector<double>& inspector =
      scene_graph_->model_inspector();
  const SourceId source_id = strandbeest_->get_source_id().value();
  for (const GeometryId id : inspector.GetGeometries(
           strandbeest_->GetBodyFrameIdOrThrow(link.index()),
           Role::kIllustration)) {
    if (inspector.GetName(id).ends_with(hole + "_pin")) {
      scene_graph_->RemoveRole(source_id, id, Role::kIllustration);
    }
  }
}

/* Prints where each assembled leg's pins are, relative to the crankshaft axle
O and expressed in the chassis frame, along with the crankshaft angle. Drake
found these; describing the legs needed only the bars' lengths. */
void StrandbeestUnassembledDemo::PrintPinLocations() const {
  const Frame<double>& O = strandbeest_->GetFrameByName("chassis_O");
  std::cout << fmt::format(
      "Assembled pin locations (x, z) relative to the crankshaft axle, with "
      "the crankshaft at {:.0f} degrees:\n",
      strandbeest_->GetJointByName<RevoluteJoint>(kCrankshaftJointName)
              .get_angle(plant_context()) *
          180 / M_PI);
  for (const Leg& leg : legs_) {
    std::cout << fmt::format("  {}\n", leg.prefix);
    for (const auto& [pin, frame_name] :
         std::vector<std::pair<std::string, std::string>>{{"X", "crank_X"},
                                                          {"Y", "bracket_Y"},
                                                          {"U1", "b_U1"},
                                                          {"U2", "d_U2"},
                                                          {"L1", "c_L1"},
                                                          {"L2", "g_L2"},
                                                          {"F", "i_F"}}) {
      const Vector3d p_OP =
          strandbeest_
              ->CalcRelativeTransform(
                  plant_context(), O,
                  strandbeest_->GetFrameByName(leg.prefix + frame_name))
              .translation();
      std::cout << fmt::format("    {:2} ({:7.4f}, {:7.4f}) m\n", pin, p_OP.x(),
                               p_OP.z());
    }
  }
}

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
the positions; the shadows close in on their bars as it goes. */
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

// =============================================================================
// A reusable assembler. Nothing below is specific to this example.
// =============================================================================

MathematicalProgramResult AssembleNearestConfiguration(
    const MultibodyPlant<double>& plant, Context<double>* plant_context,
    double tolerance) {
  if (plant.num_constraints() != plant.num_weld_constraints()) {
    throw std::logic_error(
        "AssembleNearestConfiguration() can satisfy only weld constraints.");
  }
  const VectorXd q_start = plant.GetPositions(*plant_context);
  const int nq = plant.num_positions();

  MathematicalProgram prog;
  const VectorXDecisionVariable q = prog.NewContinuousVariables(nq, "q");

  // Look for the assembled configuration nearest the starting one.
  prog.AddQuadraticErrorCost(MatrixXd::Identity(nq, nq), q_start, q);

  // Keep the joints within their limits and the locked joints where they are,
  // and keep any quaternions unit length.
  prog.AddBoundingBoxConstraint(plant.GetPositionLowerLimits(),
                                plant.GetPositionUpperLimits(), q);
  for (JointIndex index : plant.GetJointIndices()) {
    const Joint<double>& joint = plant.get_joint(index);
    if (joint.is_locked(*plant_context)) {
      const int start = joint.position_start();
      const int count = joint.num_positions();
      prog.AddBoundingBoxConstraint(q_start.segment(start, count),
                                    q_start.segment(start, count),
                                    q.segment(start, count));
    }
  }
  multibody::AddUnitQuaternionConstraintOnPlant(plant, q, &prog);

  // A weld constraint holds frame P, fixed on body A, coincident with frame Q,
  // fixed on body B. We impose that by requiring three points R to coincide,
  // each at the same coordinates r in both P and Q: the frames' origins, and
  // points a meter along their x and y axes. When three points that are not
  // in a line coincide, so do the frames.
  //
  // Each point's coordinates are held within `tolerance` rather than exactly.
  // A weld amounts to six conditions, but in a planar mechanism three of them
  // hold automatically, so stating them all as equalities would be redundant.
  // Some solvers (IPOPT, for one) give up on a problem with more equalities
  // than unknowns; narrow inequalities like these don't trouble them.
  //
  // N.B. MultibodyPlant does not yet offer a public way to ask for the frames
  // of its weld constraints, so this uses get_weld_constraint_specs(), which is
  // marked "(Internal use only)".
  const Vector3d slack = Vector3d::Constant(tolerance);
  for (const auto& [id, weld] : plant.get_weld_constraint_specs()) {
    if (!plant.GetConstraintActiveStatus(*plant_context, id)) continue;
    const Frame<double>& A = plant.get_body(weld.body_A).body_frame();
    const Frame<double>& B = plant.get_body(weld.body_B).body_frame();
    for (const Vector3d& r :
         {Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 1, 0)}) {
      const Vector3d p_AR = weld.X_AP * r;  // Where R should be on body A.
      const Vector3d p_BR = weld.X_BQ * r;  // Where R is on body B.
      prog.AddConstraint(
          std::make_shared<PositionConstraint>(
              &plant, A, p_AR - slack, p_AR + slack, B, p_BR, plant_context),
          q);
    }
  }

  const MathematicalProgramResult result = solvers::Solve(prog, q_start);
  // The constraints were evaluated using `plant_context`, so it holds whatever
  // positions were tried last. Set them explicitly either way.
  plant.SetPositions(plant_context,
                     result.is_success() ? result.GetSolution(q) : q_start);
  return result;
}

void SimulateWhileMovingJoints(
    const MultibodyPlant<double>& plant, Simulator<double>* simulator,
    const std::vector<std::pair<const Joint<double>*, double>>& targets,
    double max_speed, double settling_time) {
  Context<double>& root_context = simulator->get_mutable_context();
  Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(&root_context);

  // Where each joint starts, and how far the farthest one has to go.
  std::vector<double> starts;
  double farthest = 0.0;
  for (const auto& [joint, target] : targets) {
    if (joint->num_positions() != 1 || !joint->is_locked(plant_context)) {
      throw std::logic_error(fmt::format(
          "SimulateWhileMovingJoints() can move only locked joints with a "
          "single position, and joint '{}' isn't one.",
          joint->name()));
    }
    starts.push_back(joint->GetOnePosition(plant_context));
    farthest = std::max(farthest, std::abs(target - starts.back()));
  }

  // Move the joints once per time step of a discrete plant. A continuous plant
  // has no time step, so move them every millisecond.
  const double time_step = plant.is_discrete() ? plant.time_step() : 1.0e-3;
  const int num_steps =
      static_cast<int>(std::ceil(farthest / (max_speed * time_step)));
  for (int step = 1; step <= num_steps; ++step) {
    const double fraction = static_cast<double>(step) / num_steps;
    for (int i = 0; i < std::ssize(targets); ++i) {
      const auto& [joint, target] = targets[i];
      joint->SetPositions(
          &plant_context,
          Vector1d(starts[i] + fraction * (target - starts[i])));
    }
    simulator->AdvanceTo(root_context.get_time() + time_step);
  }
  simulator->AdvanceTo(root_context.get_time() + settling_time);
  plant.SetVelocities(&plant_context, VectorXd::Zero(plant.num_velocities()));
}

}  // namespace
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A Strandbeest leg demo demonstrating MultibodyPlant's automatic "
      "modeling of closed kinematic loops, with assembly, for a model file "
      "that describes the leg as unassembled parts. Open the indicated URL to "
      "see the Meshcat visualization.");
  // Changes the default realtime rate to 1X, so the visualization looks
  // realistic. Otherwise, it finishes too fast. Users can still change it on
  // command line, e.g., "--simulator_target_realtime_rate=0.5" to slow it down.
  FLAGS_simulator_target_realtime_rate = 1.0;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::StrandbeestUnassembledDemo().Run();
}
