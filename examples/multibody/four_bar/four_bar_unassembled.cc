/* @file
A four bar linkage demo demonstrating MultibodyPlant's automatic modeling of
closed kinematic loops, applied to an unassembled model parsed from an SDFormat
file. This is four_bar_auto.cc with the linkage and its geometry described in
four_bar_unassembled.sdf rather than built through the C++ API. The model file
uses Drake's <drake:parent_frame> and <drake:child_frame> joint extensions to
give each joint two independent frames; without them, a model parsed from a
file is always assembled. The linkage is assembled using an existing solver,
and then simulated. For pedagogical purposes, the example also visualizes the
shadow link and weld constraint that Drake creates to model the loop; a shadow
does not exist until the model is finalized, so it cannot be drawn by the model
file. Refer to README.md for more details and comparisons with the other
four-bar examples. */
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

#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using geometry::Box;
using geometry::Cylinder;
using geometry::FrameId;
using geometry::GeometryInstance;
using geometry::SceneGraph;
using geometry::Shape;
using geometry::SourceId;
using math::RigidTransformd;
using math::RotationMatrixd;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::BodyIndex;
using multibody::Frame;
using multibody::Joint;
using multibody::Link;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::RevoluteJoint;
using systems::Context;
using systems::DiagramBuilder;
using systems::EventStatus;
using systems::Simulator;

namespace {

DEFINE_double(simulation_time, 10.0, "Duration of the simulation in seconds.");

DEFINE_double(time_step, 1.0e-3,
              "Discrete time step of the MultibodyPlant in seconds, or 0 for a "
              "continuous plant. Closing the loop takes a constraint, so this "
              "needs a solver that supports one: discrete SAP, which is the "
              "default here, or the continuous CENIC integrator, via "
              "--time_step=0 --simulator_integration_scheme=cenic.");

DEFINE_double(applied_torque, 0.0,
              "Constant torque applied to the world_driver joint, in N·m.");

DEFINE_double(driver_angle, 0.0,
              "Angle in radians at which to hold the world_driver joint while "
              "the linkage is assembled, so that the mechanism assembles "
              "around it.");

DEFINE_double(assembly_playback_time, 3.0,
              "Seconds to spend replaying the assembly in the visualizer, so "
              "that it can be watched rather than being over within a single "
              "frame. Set to 0 to skip the replay. This is wall clock time and "
              "is unrelated to --simulator_target_realtime_rate.");

DEFINE_bool(interactive, true,
            "Show the linkage as defined and then again once it is assembled, "
            "waiting for you to press Enter before going on each time. Set "
            "--nointeractive to run start to finish without stopping.");

constexpr double kAssembledTolerance = 1.0e-3;  // meters

/* Owns the shared state of this example. Run() comes first and is implemented
in line so that you can begin with the example's story; implementation details
follow it. */
class FourBarUnassembledDemo final {
 public:
  int Run() {
    // Build the MultibodyPlant and SceneGraph.
    auto [four_bar, scene_graph] =
        AddMultibodyPlantSceneGraph(&builder_, FLAGS_time_step);
    four_bar_ = &four_bar;
    scene_graph_ = &scene_graph;

    // Load the closed-topology mechanism, unassembled, along with the geometry
    // that illustrates it. See four_bar_unassembled.sdf.
    Parser(&builder_).AddModelsFromUrl(
        "package://drake/examples/multibody/four_bar/four_bar_unassembled.sdf");

    // We are done defining the model. Opt in to automatic modeling of closed
    // topologies (otherwise Finalize() would throw). Splitting a link,
    // retargeting a joint, and adding a weld happens here.
    four_bar_->SetEnableLoopTopology(true);
    four_bar_->Finalize();
    if (four_bar_->num_loop_constraints() != 1) {
      throw std::runtime_error(
          fmt::format("Expected one modeled loop, but found {}.",
                      four_bar_->num_loop_constraints()));
    }
    ReportAndIllustrateModeledLoops();

    visualization::AddDefaultVisualization(&builder_);
    diagram_ = builder_.Build();

    // Create a context and simulator.
    std::unique_ptr<Context<double>> diagram_context =
        diagram_->CreateDefaultContext();
    simulator_ = MakeSimulatorFromGflags(*diagram_, std::move(diagram_context));

    // Apply a constant torque to the only actuated joint.
    four_bar_->get_actuation_input_port().FixValue(&mutable_plant_context(),
                                                   FLAGS_applied_torque);

    // We set no initial conditions. Every joint angle defaults to zero, which
    // is the unassembled configuration shown in the model file.
    simulator_->Initialize();
    const double initial_error = CalcLoopClosureError(plant_context());
    if (!(initial_error > kAssembledTolerance)) {
      throw std::runtime_error(fmt::format(
          "Expected an unassembled model, but its loop closure error is {} m.",
          initial_error));
    }
    std::cout << fmt::format("Loop closure error as defined: {:.4f} m\n",
                             initial_error);
    ShowAndWait(
        "The linkage is unassembled: the coupler and its pale shadow are "
        "apart, so the weld constraint holding the two halves of the split "
        "link together is not satisfied yet. Every joint already is -- look "
        "for each one's two pins sitting together. ",
        "assemble the linkage");

    const AssemblyResult assembly = Assemble(FLAGS_driver_angle);
    std::cout << fmt::format(
        "Loop closure error after assembling for {:.4f} s: {:.3g} m\n",
        assembly.simulation_time, CalcLoopClosureError(plant_context()));
    std::cout << fmt::format(
        "Assembly took {} steps; replaying them over {} s.\n",
        std::ssize(assembly.path) - 1, FLAGS_assembly_playback_time);

    // Rewind and replay, so that the assembly can actually be seen happening.
    ReplayAssembly(assembly.path, FLAGS_assembly_playback_time);
    ShowAndWait(
        "The linkage is assembled: the shadow now lies on top of the coupler "
        "it was split from, together reconstituting the original link's "
        "properties.",
        "simulate");

    // Simulate the assembled mechanism swinging under gravity.
    simulator_->AdvanceTo(FLAGS_simulation_time);
    std::cout << fmt::format(
        "Loop closure error after simulating {} s: {:.3g} m\n",
        FLAGS_simulation_time, CalcLoopClosureError(plant_context()));
    PrintSimulatorStatistics(*simulator_);
    return 0;
  }

 private:
  struct AssemblyStep {
    VectorXd q;
    double error{};
  };

  struct AssemblyResult {
    double simulation_time{};
    std::vector<AssemblyStep> path;
  };

  // Assembly.
  const Context<double>& plant_context() const;
  Context<double>& mutable_plant_context();
  double CalcLoopClosureError(const Context<double>& context) const;
  AssemblyResult Assemble(double driver_angle);

  // Presentation.
  void ReportAndIllustrateModeledLoops();
  void ShowAndWait(const std::string& message, const std::string& next);
  void ReplayAssembly(const std::vector<AssemblyStep>& path, double duration);

  // Low-level illustration support.
  static Vector4d WeldFrameColor();
  static std::pair<RigidTransformd, Box> MakeBar(const Vector3d& p_LStart,
                                                 double length,
                                                 const Vector3d& direction,
                                                 double transverse_scale = 1.0);
  static std::pair<RigidTransformd, Cylinder> MakePivot(const Vector3d& p_LP,
                                                        double radius);
  static std::pair<RigidTransformd, Cylinder> MakeFrameAxis(
      const RigidTransformd& X_LF, int axis, double radius_scale = 1.0);
  void DrawShadowLink(const Link<double>& shadow);

  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* four_bar_{};
  SceneGraph<double>* scene_graph_{};
  std::unique_ptr<systems::Diagram<double>> diagram_;
  std::unique_ptr<Simulator<double>> simulator_;
};

const Context<double>& FourBarUnassembledDemo::plant_context() const {
  return four_bar_->GetMyContextFromRoot(simulator_->get_context());
}

Context<double>& FourBarUnassembledDemo::mutable_plant_context() {
  return four_bar_->GetMyMutableContextFromRoot(
      &simulator_->get_mutable_context());
}

/* Returns the loop closure error, in meters. The two frames of the retargeted
joint are no longer held together by that joint, and the distance between them
is the loop closure error. */
double FourBarUnassembledDemo::CalcLoopClosureError(
    const Context<double>& context) const {
  const Joint<double>& loop_joint = four_bar_->GetJointByName("coupler_rocker");
  return four_bar_
      ->CalcRelativeTransform(context, loop_joint.frame_on_parent(),
                              loop_joint.frame_on_child())
      .translation()
      .norm();
}

/* Assembles the linkage with the driver held at `driver_angle`. Nothing here
solves the loop closure equations; the solver pulls the two halves of the split
link together, and this function merely steps until they have arrived. On
return, the linkage is assembled and at rest, the driver is released, and the
simulator clock is reset to zero. */
FourBarUnassembledDemo::AssemblyResult FourBarUnassembledDemo::Assemble(
    double driver_angle) {
  Context<double>& context = mutable_plant_context();
  const RevoluteJoint<double>& driver_joint =
      four_bar_->GetJointByName<RevoluteJoint>("world_driver");
  driver_joint.set_angle(&context, driver_angle);
  driver_joint.Lock(&context);

  AssemblyResult result;
  // The monitor below runs after each step, so record the starting point here.
  result.path.push_back(
      {four_bar_->GetPositions(context), CalcLoopClosureError(context)});

  const double kAssemblyDeadline = 0.5;  // seconds
  simulator_->set_monitor([this, &result](const Context<double>& root_context) {
    const Context<double>& plant_context =
        four_bar_->GetMyContextFromRoot(root_context);
    const double error = CalcLoopClosureError(plant_context);
    result.path.push_back({four_bar_->GetPositions(plant_context), error});
    if (error < kAssembledTolerance) {
      return EventStatus::ReachedTermination(diagram_.get(), "loop closed");
    }
    return EventStatus::Succeeded();
  });
  simulator_->AdvanceTo(kAssemblyDeadline);
  simulator_->clear_monitor();
  result.simulation_time = simulator_->get_context().get_time();
  const double final_error = CalcLoopClosureError(context);
  if (!(final_error < kAssembledTolerance)) {
    throw std::runtime_error(fmt::format(
        "Failed to assemble the linkage within {} s; its loop closure error "
        "is {} m.",
        kAssemblyDeadline, final_error));
  }

  // Release the driver and hand back a clean assembled initial condition.
  driver_joint.Unlock(&context);
  four_bar_->SetVelocities(&context,
                           VectorXd::Zero(four_bar_->num_velocities()));
  simulator_->get_mutable_context().SetTime(0.0);
  simulator_->Initialize();
  return result;
}

// The rest of the file supports the pedagogical visualization. None of it is
// needed merely to model a closed topology. All the geometry other than the
// shadow link's is in the model file; these values match it.

constexpr double kBarWidth = 0.2;      // In the plane of motion.
constexpr double kBarThickness = 0.1;  // Out of the plane of motion, i.e. y.
constexpr double kPivotRadius = 0.02;  // The child end of a joint.
constexpr double kFatPivotRadius = 2 * kPivotRadius;  // The parent end.
constexpr double kPivotLength = 2 * kBarThickness;    // Pokes out either side.
constexpr double kShadowAlpha = 0.33;
constexpr double kShadowScale = 0.75;
constexpr double kPlaybackFps = 30.0;
constexpr double kFrameAxisLength = 0.25;   // meters
constexpr double kFrameAxisRadius = 0.006;  // meters

Vector4d FourBarUnassembledDemo::WeldFrameColor() {
  return Vector4d(0.833, 0.333, 0, 1);  // A dark orange.
}

std::pair<RigidTransformd, Box> FourBarUnassembledDemo::MakeBar(
    const Vector3d& p_LStart, double length, const Vector3d& direction,
    double transverse_scale) {
  const bool along_x = direction.x() != 0.0;
  // The long side is not scaled, so the shadow runs exactly as far as the link
  // from which it was split.
  const double long_side = length - kBarWidth;
  const double width = transverse_scale * kBarWidth;
  const double thickness = transverse_scale * kBarThickness;
  return {
      RigidTransformd(p_LStart + 0.5 * length * direction),
      Box(along_x ? long_side : width, thickness, along_x ? width : long_side)};
}

std::pair<RigidTransformd, Cylinder> FourBarUnassembledDemo::MakePivot(
    const Vector3d& p_LP, double radius) {
  return {RigidTransformd(RotationMatrixd::MakeXRotation(M_PI_2), p_LP),
          Cylinder(radius, kPivotLength)};
}

std::pair<RigidTransformd, Cylinder> FourBarUnassembledDemo::MakeFrameAxis(
    const RigidTransformd& X_LF, int axis, double radius_scale) {
  const RotationMatrixd R_FG =
      axis == 0   ? RotationMatrixd::MakeYRotation(M_PI_2)
      : axis == 1 ? RotationMatrixd::MakeXRotation(M_PI_2)
                  : RotationMatrixd::Identity();
  const Vector3d p_FG = 0.5 * kFrameAxisLength * Vector3d::Unit(axis);
  return {X_LF * RigidTransformd(R_FG, p_FG),
          Cylinder(radius_scale * kFrameAxisRadius, kFrameAxisLength)};
}

/* Draws `shadow` as a faint, slightly slimmer copy of the coupler, running
between the coupler's Cd and Cr frames as the model file defines them. A shadow
does not exist until Finalize(), so it cannot be drawn by the model file, and
its geometry must be registered directly with SceneGraph rather than through
MultibodyPlant::RegisterVisualGeometry(). */
void FourBarUnassembledDemo::DrawShadowLink(const Link<double>& shadow) {
  const Vector4d pale_blue(0.6, 0.6, 1, kShadowAlpha);
  const SourceId source_id = four_bar_->get_source_id().value();
  const FrameId frame_id = four_bar_->GetBodyFrameIdOrThrow(shadow.index());
  auto add_geometry = [&](const RigidTransformd& X_LG, const Shape& shape,
                          const std::string& name, const Vector4d& color) {
    auto instance = std::make_unique<GeometryInstance>(X_LG, shape, name);
    instance->set_illustration_properties(
        geometry::MakePhongIllustrationProperties(color));
    scene_graph_->RegisterGeometry(source_id, frame_id, std::move(instance));
  };

  const Vector3d p_CoCd =
      four_bar_->GetFrameByName("Cd").GetFixedPoseInBodyFrame().translation();
  const Vector3d p_CoCr =
      four_bar_->GetFrameByName("Cr").GetFixedPoseInBodyFrame().translation();
  const double coupler_length = (p_CoCr - p_CoCd).norm();
  const auto [X_LB, bar] =
      MakeBar(p_CoCd, coupler_length, Vector3d::UnitX(), kShadowScale);
  add_geometry(X_LB, bar, shadow.name() + "_bar", pale_blue);
  const auto [X_LP, parent_pin] = MakePivot(p_CoCr, kFatPivotRadius);
  add_geometry(X_LP, parent_pin, shadow.name() + "_parent_pin", pale_blue);

  // Draw the shadow's origin exactly like the coupler's Co frame, so that the
  // two orange triads become one when the weld is satisfied.
  for (int axis = 0; axis < 3; ++axis) {
    const auto [X_LG, arm] = MakeFrameAxis(RigidTransformd(), axis, 2.0);
    add_geometry(X_LG, arm,
                 fmt::format("{}_origin_{}_axis", shadow.name(), "xyz"[axis]),
                 WeldFrameColor());
  }
}

void FourBarUnassembledDemo::ReportAndIllustrateModeledLoops() {
  // Each loop costs one shadow link and one weld constraint. The primary and
  // shadow share the original link's mass evenly.
  std::cout << fmt::format(
      "Modeled {} kinematic loop(s) with {} constraint(s).\n",
      four_bar_->num_loop_constraints(), four_bar_->num_constraints());
  for (BodyIndex index(0); index < four_bar_->num_bodies(); ++index) {
    const Link<double>& body = four_bar_->get_body(index);
    if (body.is_ephemeral()) {
      std::cout << fmt::format(
          "  shadow link '{}' was added to break a loop.\n", body.name());
      DrawShadowLink(body);
    }
  }
}

/* Shows the current configuration and, unless --nointeractive, waits for the
user to press Enter before doing whatever `next` describes. */
void FourBarUnassembledDemo::ShowAndWait(const std::string& message,
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

/* Replays the recorded assembly in the visualizer over `duration` seconds of
wall time, then restores the assembled configuration. Frames are selected by
remaining loop closure error, so playback does not depend on solver step size.
*/
void FourBarUnassembledDemo::ReplayAssembly(
    const std::vector<AssemblyStep>& path, double duration) {
  if (duration <= 0.0 || std::ssize(path) < 2) return;

  Context<double>& context = mutable_plant_context();
  const VectorXd q_assembled = four_bar_->GetPositions(context);
  const double error_start = path.front().error;
  const int num_frames = std::max(2, static_cast<int>(duration * kPlaybackFps));
  const auto frame_period = std::chrono::duration<double>(1.0 / kPlaybackFps);

  for (int frame = 0; frame < num_frames; ++frame) {
    const double error =
        error_start * (1.0 - static_cast<double>(frame) / (num_frames - 1));
    int i = 0;
    while (i + 2 < std::ssize(path) && path[i + 1].error > error) ++i;
    const double e0 = path[i].error;
    const double e1 = path[i + 1].error;
    const double w =
        e0 > e1 ? std::clamp((e0 - error) / (e0 - e1), 0.0, 1.0) : 1.0;
    four_bar_->SetPositions(&context,
                            path[i].q + w * (path[i + 1].q - path[i].q));

    diagram_->ForcedPublish(simulator_->get_context());
    std::this_thread::sleep_for(frame_period);
  }

  four_bar_->SetPositions(&context, q_assembled);
}

}  // namespace
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A four bar linkage demo demonstrating MultibodyPlant's automatic "
      "modeling of closed kinematic loops, with assembly, for an unassembled "
      "model file. Open the indicated URL to see the Meshcat visualization.");
  // Changes the default realtime rate to 1X, so the visualization looks
  // realistic. Otherwise, it finishes too fast. Users can still change it on
  // command line, e.g., "--simulator_target_realtime_rate=0.5" to slow it down.
  FLAGS_simulator_target_realtime_rate = 1.0;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::FourBarUnassembledDemo().Run();
}
