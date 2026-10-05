/* @file
A four bar linkage demo demonstrating MultibodyPlant's automatic modeling of
closed kinematic loops. The linkage is defined as an unassembled loop of four
revolute joints, assembled using an existing solver, and then simulated. For
pedagogical purposes, the example also visualizes the shadow link and weld
constraint that Drake creates to model the loop. Refer to README.md for more
details and comparisons with the other four-bar examples. */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
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
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/spatial_inertia.h"
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
using multibody::FixedOffsetFrame;
using multibody::Frame;
using multibody::Joint;
using multibody::Link;
using multibody::MultibodyPlant;
using multibody::RevoluteJoint;
using multibody::SpatialInertia;
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

/* Owns the shared state of this example. Run() comes first and is implemented
in line so that you can begin with the example's story; implementation details
follow it. */
class FourBarAutoDemo final {
 public:
  int Run() {
    // Build the MultibodyPlant and SceneGraph.
    auto [four_bar, scene_graph] =
        AddMultibodyPlantSceneGraph(&builder_, FLAGS_time_step);
    four_bar_ = &four_bar;
    scene_graph_ = &scene_graph;

    // Construct the closed-topology mechanism, unassembled.
    BuildFourBarLinkage();

    // This geometry is cosmetic only, and must be added before Finalize().
    AddFourBarIllustration();

    // We are done defining the model. Opt in to automatic modeling of closed
    // topologies (otherwise Finalize() would throw). Splitting a link,
    // retargeting a joint, and adding a weld happens here.
    four_bar_->SetEnableLoopTopology(true);
    four_bar_->Finalize();
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
    // is the unassembled configuration shown in the model description below.
    simulator_->Initialize();
    std::cout << fmt::format("Loop closure error as defined: {:.4f} m\n",
                             CalcLoopClosureError(plant_context()));
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

  // Model construction and assembly.
  void BuildFourBarLinkage();
  const Frame<double>& AddFrame(const Link<double>& link,
                                const std::string& name, const Vector3d& p_LF);
  const Context<double>& plant_context() const;
  Context<double>& mutable_plant_context();
  double CalcLoopClosureError(const Context<double>& context) const;
  AssemblyResult Assemble(double driver_angle);

  // Presentation.
  void AddFourBarIllustration();
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
  void DrawBar(const Link<double>& link, const Vector3d& p_LStart,
               double length, const Vector3d& direction, const Vector4d& color);
  void DrawPivot(const Link<double>& link, const Vector3d& p_LP,
                 const std::string& name, double radius, const Vector4d& color);
  void DrawFrame(const Frame<double>& frame, const Vector4d& color,
                 const std::string& name, double radius_scale = 1.0);
  void DrawParentPin(const Frame<double>& parent_frame, const Vector4d& color,
                     const std::string& name);
  void DrawChildPin(const Frame<double>& child_frame, const Vector4d& color,
                    const std::string& name);
  void DrawJoint(const Frame<double>& parent_frame,
                 const Vector4d& parent_color, const Frame<double>& child_frame,
                 const Vector4d& child_color, const std::string& name);
  void DrawShadowLink(const Link<double>& shadow);

  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* four_bar_{};
  SceneGraph<double>* scene_graph_{};
  std::unique_ptr<systems::Diagram<double>> diagram_;
  std::unique_ptr<Simulator<double>> simulator_;
};

/* The three moving links (driver, coupler, rocker) plus World and four revolute
joints are laid out below as they are defined, unassembled and conveniently
lined up with the World frame axes. "*" marks the connection points, each of
which is a frame on a link. The frame names start with a capital letter
matching the link to which they are fixed and a lower case letter for the
connected link, except that Do and Ro are monogram names for the driver and
rocker link frame origins.

The coupler's link frame Co sits at the middle of the bar rather than at either
connection point, so both of its connections, Cd and Cr, are offset frames. Co
is where the loop-closing weld constraint ends up. All these frames are aligned
with World as drawn.

                                                      * Rc
                                                      |
                                                      |
                     coupler C                        |
     Cd *-------------------------- Co ------------------------* Cr
                            4.8m 1kg                  |
        * Dc                                          |
        |                                    rocker R | 2m
        |                                             | 2kg
    1m  | driver D                                    |
    1kg |                                             |      Wz
        |                                             |      |  Wy
        * Do                                       Ro *      | /
                                                             +----- Wx
        *====================== Wo ===================*
       Wd         2m         World W         2m       Wr

In parent-child order, the joints connect Wd-Do, Wr-Ro, Dc-Cd and Cr-Rc.
Link mass centers are at their midpoints. Lengths and masses are shown;
inertias are those of thin rods (mass * length² / 12). All four revolute
joints have their axes in the -y direction (towards you), so the mechanism
moves in the World x-z plane and swings under Drake's usual -z gravity.

This configuration does not satisfy loop closure. Drake automatically adds a
shadow link to break the loop and retargets one joint to it, making a tree. It
then adds a weld constraint between the shadow and its primary link. The
initial configuration does not satisfy that weld, so the linkage must be
assembled before it can be simulated. */

constexpr double kGroundLength = 4.0;   // Wd to Wr.
constexpr double kDriverLength = 1.0;   // Do to Dc.
constexpr double kCouplerLength = 4.8;  // Cd to Cr, with Co at the middle.
constexpr double kRockerLength = 2.0;   // Ro to Rc.
constexpr double kDriverMass = 1.0;     // kg
constexpr double kCouplerMass = 1.0;    // kg
constexpr double kRockerMass = 2.0;     // kg

/* Adds a frame named `name`, fixed to `link` at `p_LF` and aligned with the
link frame, and returns it. */
const Frame<double>& FourBarAutoDemo::AddFrame(const Link<double>& link,
                                               const std::string& name,
                                               const Vector3d& p_LF) {
  return four_bar_->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
      name, link, RigidTransformd(p_LF)));
}

/* Adds the links, frames, and joints shown above. Nothing here says anything
about a spanning tree: the four joints simply form a loop, and MultibodyPlant
decides how to model it. */
void FourBarAutoDemo::BuildFourBarLinkage() {
  // The ground "link" of the linkage is World itself; Wd and Wr are the frames
  // where the linkage attaches to it.
  const Link<double>& world = four_bar_->world_body();
  const Frame<double>& Wd =
      AddFrame(world, "Wd", Vector3d(-0.5 * kGroundLength, 0, 0));
  const Frame<double>& Wr =
      AddFrame(world, "Wr", Vector3d(0.5 * kGroundLength, 0, 0));

  // The driver, a thin rod from its link frame Do along +z to Dc.
  const Link<double>& driver = four_bar_->AddRigidBody(
      "driver", SpatialInertia<double>::ThinRodWithMassAboutEnd(
                    kDriverMass, kDriverLength, Vector3d::UnitZ()));
  const Frame<double>& Do = driver.body_frame();
  const Frame<double>& Dc =
      AddFrame(driver, "Dc", Vector3d(0, 0, kDriverLength));

  // The coupler runs along x from Cd to Cr, with its link frame Co at the
  // middle. Its inertia is therefore that of a rod about its center of mass.
  const Link<double>& coupler = four_bar_->AddRigidBody(
      "coupler", SpatialInertia<double>::ThinRodWithMass(
                     kCouplerMass, kCouplerLength, Vector3d::UnitX()));
  const Frame<double>& Cd =
      AddFrame(coupler, "Cd", Vector3d(-0.5 * kCouplerLength, 0, 0));
  const Frame<double>& Cr =
      AddFrame(coupler, "Cr", Vector3d(0.5 * kCouplerLength, 0, 0));

  // The rocker, a thin rod from its link frame Ro along +z to Rc.
  const Link<double>& rocker = four_bar_->AddRigidBody(
      "rocker", SpatialInertia<double>::ThinRodWithMassAboutEnd(
                    kRockerMass, kRockerLength, Vector3d::UnitZ()));
  const Frame<double>& Ro = rocker.body_frame();
  const Frame<double>& Rc =
      AddFrame(rocker, "Rc", Vector3d(0, 0, kRockerLength));

  // The four revolute joints connect the frame pairs shown above, with q = 0
  // when each pair is coincident. Only the first joint is actuated.
  const Vector3d axis = -Vector3d::UnitY();  // Out of the page, towards you.
  const RevoluteJoint<double>& world_driver = four_bar_->AddJoint(
      std::make_unique<RevoluteJoint<double>>("world_driver", Wd, Do, axis));
  four_bar_->AddJoint(
      std::make_unique<RevoluteJoint<double>>("world_rocker", Wr, Ro, axis));
  four_bar_->AddJoint(
      std::make_unique<RevoluteJoint<double>>("driver_coupler", Dc, Cd, axis));
  four_bar_->AddJoint(
      std::make_unique<RevoluteJoint<double>>("coupler_rocker", Cr, Rc, axis));
  four_bar_->AddJointActuator("driver_torque", world_driver);
}

const Context<double>& FourBarAutoDemo::plant_context() const {
  return four_bar_->GetMyContextFromRoot(simulator_->get_context());
}

Context<double>& FourBarAutoDemo::mutable_plant_context() {
  return four_bar_->GetMyMutableContextFromRoot(
      &simulator_->get_mutable_context());
}

/* Returns the loop closure error, in meters. The two frames of the retargeted
joint are no longer held together by that joint, and the distance between them
is the loop closure error. */
double FourBarAutoDemo::CalcLoopClosureError(
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
FourBarAutoDemo::AssemblyResult FourBarAutoDemo::Assemble(double driver_angle) {
  Context<double>& context = mutable_plant_context();
  const RevoluteJoint<double>& driver_joint =
      four_bar_->GetJointByName<RevoluteJoint>("world_driver");
  driver_joint.set_angle(&context, driver_angle);
  driver_joint.Lock(&context);

  AssemblyResult result;
  // The monitor below runs after each step, so record the starting point here.
  result.path.push_back(
      {four_bar_->GetPositions(context), CalcLoopClosureError(context)});

  const double kAssembledTolerance = 1.0e-3;  // meters
  const double kAssemblyDeadline = 0.5;       // seconds
  simulator_->set_monitor([this, kAssembledTolerance,
                           &result](const Context<double>& root_context) {
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

  // Release the driver and hand back a clean assembled initial condition.
  driver_joint.Unlock(&context);
  four_bar_->SetVelocities(&context,
                           VectorXd::Zero(four_bar_->num_velocities()));
  simulator_->get_mutable_context().SetTime(0.0);
  simulator_->Initialize();
  return result;
}

// The rest of the file supports the pedagogical visualization. None of it is
// needed merely to model a closed topology.

constexpr double kBarWidth = 0.2;      // In the plane of motion.
constexpr double kBarThickness = 0.1;  // Out of the plane of motion, i.e. y.
constexpr double kPivotRadius = 0.02;  // The child end of a joint.
constexpr double kFatPivotRadius = 2 * kPivotRadius;  // The parent end.
constexpr double kParentPinAlpha = 0.5;  // See through it to the child's pin.
constexpr double kPivotLength = 2 * kBarThickness;  // Pokes out either side.
constexpr double kCouplerAlpha = 0.5;
constexpr double kShadowAlpha = 0.33;
constexpr double kShadowScale = 0.75;
constexpr double kPlaybackFps = 30.0;
constexpr double kFrameAxisLength = 0.25;   // meters
constexpr double kFrameAxisRadius = 0.006;  // meters

Vector4d FourBarAutoDemo::WeldFrameColor() {
  return Vector4d(0.833, 0.333, 0, 1);  // A dark orange.
}

std::pair<RigidTransformd, Box> FourBarAutoDemo::MakeBar(
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

std::pair<RigidTransformd, Cylinder> FourBarAutoDemo::MakePivot(
    const Vector3d& p_LP, double radius) {
  return {RigidTransformd(RotationMatrixd::MakeXRotation(M_PI_2), p_LP),
          Cylinder(radius, kPivotLength)};
}

std::pair<RigidTransformd, Cylinder> FourBarAutoDemo::MakeFrameAxis(
    const RigidTransformd& X_LF, int axis, double radius_scale) {
  const RotationMatrixd R_FG =
      axis == 0   ? RotationMatrixd::MakeYRotation(M_PI_2)
      : axis == 1 ? RotationMatrixd::MakeXRotation(M_PI_2)
                  : RotationMatrixd::Identity();
  const Vector3d p_FG = 0.5 * kFrameAxisLength * Vector3d::Unit(axis);
  return {X_LF * RigidTransformd(R_FG, p_FG),
          Cylinder(radius_scale * kFrameAxisRadius, kFrameAxisLength)};
}

void FourBarAutoDemo::DrawBar(const Link<double>& link,
                              const Vector3d& p_LStart, double length,
                              const Vector3d& direction,
                              const Vector4d& color) {
  const auto [X_LB, bar] = MakeBar(p_LStart, length, direction);
  four_bar_->RegisterVisualGeometry(link, X_LB, bar, link.name() + "_bar",
                                    color);
}

void FourBarAutoDemo::DrawPivot(const Link<double>& link, const Vector3d& p_LP,
                                const std::string& name, double radius,
                                const Vector4d& color) {
  const auto [X_LP, pivot] = MakePivot(p_LP, radius);
  four_bar_->RegisterVisualGeometry(link, X_LP, pivot, name, color);
}

void FourBarAutoDemo::DrawFrame(const Frame<double>& frame,
                                const Vector4d& color, const std::string& name,
                                double radius_scale) {
  const RigidTransformd X_LF = frame.GetFixedPoseInBodyFrame();
  for (int axis = 0; axis < 3; ++axis) {
    const auto [X_LG, arm] = MakeFrameAxis(X_LF, axis, radius_scale);
    four_bar_->RegisterVisualGeometry(
        frame.link(), X_LG, arm, fmt::format("{}_{}_axis", name, "xyz"[axis]),
        color);
  }
}

void FourBarAutoDemo::DrawParentPin(const Frame<double>& parent_frame,
                                    const Vector4d& color,
                                    const std::string& name) {
  Vector4d translucent_color = color;
  translucent_color[3] = kParentPinAlpha;
  DrawPivot(parent_frame.link(),
            parent_frame.GetFixedPoseInBodyFrame().translation(),
            name + "_parent_pin", kFatPivotRadius, translucent_color);
}

void FourBarAutoDemo::DrawChildPin(const Frame<double>& child_frame,
                                   const Vector4d& color,
                                   const std::string& name) {
  DrawPivot(child_frame.link(),
            child_frame.GetFixedPoseInBodyFrame().translation(),
            name + "_child_pin", kPivotRadius, color);
}

void FourBarAutoDemo::DrawJoint(const Frame<double>& parent_frame,
                                const Vector4d& parent_color,
                                const Frame<double>& child_frame,
                                const Vector4d& child_color,
                                const std::string& name) {
  DrawParentPin(parent_frame, parent_color, name);
  DrawChildPin(child_frame, child_color, name);
}

void FourBarAutoDemo::AddFourBarIllustration() {
  const Vector4d green(0, 1, 0, 1), red(1, 0, 0, 1),
      blue(0, 0, 1, kCouplerAlpha), yellow(1, 1, 0, 1);

  const Link<double>& world = four_bar_->world_body();
  const Link<double>& driver = four_bar_->GetBodyByName("driver");
  const Link<double>& coupler = four_bar_->GetBodyByName("coupler");
  const Link<double>& rocker = four_bar_->GetBodyByName("rocker");
  const Frame<double>& Wd = four_bar_->GetFrameByName("Wd");
  const Frame<double>& Wr = four_bar_->GetFrameByName("Wr");
  const Frame<double>& Dc = four_bar_->GetFrameByName("Dc");
  const Frame<double>& Cd = four_bar_->GetFrameByName("Cd");
  const Frame<double>& Cr = four_bar_->GetFrameByName("Cr");
  const Frame<double>& Rc = four_bar_->GetFrameByName("Rc");

  DrawBar(world, Vector3d(-0.5 * kGroundLength, 0, 0), kGroundLength,
          Vector3d::UnitX(), green);
  DrawBar(driver, Vector3d::Zero(), kDriverLength, Vector3d::UnitZ(), red);
  DrawBar(coupler, Vector3d(-0.5 * kCouplerLength, 0, 0), kCouplerLength,
          Vector3d::UnitX(), blue);
  DrawBar(rocker, Vector3d::Zero(), kRockerLength, Vector3d::UnitZ(), yellow);

  // Every joint is drawn as a translucent parent pin around a child pin.
  DrawJoint(Wd, green, driver.body_frame(), red, "world_driver");
  DrawJoint(Wr, green, rocker.body_frame(), yellow, "world_rocker");
  DrawJoint(Dc, red, Cd, blue, "driver_coupler");

  // The coupler_rocker joint will move from the coupler to its shadow during
  // Finalize(), so draw only its child end here. Its parent end is added to the
  // shadow later.
  DrawChildPin(Rc, yellow, "coupler_rocker");

  // Orange marks the coupler frame and the matching frame on its shadow; blue
  // marks the coupler's far end.
  DrawFrame(coupler.body_frame(), WeldFrameColor(), "Co", 2.0);
  DrawFrame(Cr, blue, "Cr");
}

/* Draws `shadow` as a faint, slightly slimmer copy of the coupler. A shadow
does not exist until Finalize(), so its geometry must be registered directly
with SceneGraph rather than through MultibodyPlant::RegisterVisualGeometry(). */
void FourBarAutoDemo::DrawShadowLink(const Link<double>& shadow) {
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

  const Vector3d p_CoCd(-0.5 * kCouplerLength, 0, 0);
  const auto [X_LB, bar] =
      MakeBar(p_CoCd, kCouplerLength, Vector3d::UnitX(), kShadowScale);
  add_geometry(X_LB, bar, shadow.name() + "_bar", pale_blue);
  const auto [X_LP, parent_pin] =
      MakePivot(p_CoCd + kCouplerLength * Vector3d::UnitX(), kFatPivotRadius);
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

void FourBarAutoDemo::ReportAndIllustrateModeledLoops() {
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
void FourBarAutoDemo::ShowAndWait(const std::string& message,
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
void FourBarAutoDemo::ReplayAssembly(const std::vector<AssemblyStep>& path,
                                     double duration) {
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
      "modeling of closed kinematic loops, with assembly. Open the indicated "
      "URL to see the Meshcat visualization.");
  // Changes the default realtime rate to 1X, so the visualization looks
  // realistic. Otherwise, it finishes too fast. Users can still change it on
  // command line, e.g., "--simulator_target_realtime_rate=0.5" to slow it down.
  FLAGS_simulator_target_realtime_rate = 1.0;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::FourBarAutoDemo().Run();
}
