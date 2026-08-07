/* @file
A four bar linkage demo demonstrating MultibodyPlant's automatic modeling of
closed kinematic loops. It shows:
  - How to define a four bar linkage as a plain loop of four joints, with no
    attempt to specify a spanning tree and no stand-in for the "extra" joint.
  - How to opt in to automatic loop modeling with
    `MultibodyPlant::SetEnableLoopTopology()`, and report what the resulting
    model looks like.
  - That there is no need to solve for an assembled configuration to start
    from: the linkage is defined here unassembled, and the loop-closing
    constraint that Finalize() added assembles it within the first few
    milliseconds, with the input joint locked to hold it at the angle it was
    defined with. The configurations it passes through on the way are recorded
    and replayed (see --assembly_playback_time), since otherwise the assembly
    is over before it can be seen.

  Closing the loop takes a constraint, so this model asks for a solver that can
  enforce one, and it does not care which. It runs discrete under SAP, which is
  the default here, and continuous under CENIC:
      --time_step=0 --simulator_integration_scheme=cenic

  Compare four_bar_with_bushing.cc, which models the same kind of mechanism
  with one joint left out and replaced by a LinearBushingRollPitchYaw force
  element.

  The linkage is built with the C++ API rather than parsed from a model file
  because no model file can describe an unassembled loop: an SDF joint has a
  single frame, from which both of Drake's two joint frames are derived, so they
  are coincident at q = 0 no matter what poses the links are given. Building the
  joints here sets those two frames independently, which is what lets the figure
  below be the model as defined, unassembled q = 0 and all. Compare four_bar.cc,
  which parses the same kind of mechanism from a model file and therefore has to
  start out assembled.

  Refer to README.md for more details.

  The three moving links (driver, coupler, rocker) plus World and four revolute
  joints are laid out below as they are defined, unassembled and conveniently
  lined up with the World frame axes. "*" marks the connection points, each of
  which is a frame on a link. The frame names start with a capital letter
  matching the link to which they are fixed and a lower case letter for the
  connected link, except that Do and Ro are just monogram names for the driver
  and rocker link frames themselves, which is where we put those connection
  points.

  The coupler's link frame Co sits at the middle of the bar rather than at
either connection point, so both of its connections, Cd and Cr, are offset
frames. Co is where the loop-closing weld constraint ends up -- that weld holds
the shadow copy's link frame to this one -- so putting Co at the middle puts the
  weld at the middle of the link being split, where it is easier to see (we
  draw frames there), rather than off at one end. All these frames are aligned
  with World as drawn.

                                                      * Rc
                                                      |
                                                      |
                     coupler C       Co               |
     Cd *---------------------------*--------------------------* Cr
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
  moves in the World x-z plane, hence swings under Drake's usual -z gravity.

  Note that this configuration does not satisfy loop closure. Drake will
  automatically add a "shadow" link to break the loop, and one of the joints
  will be retargeted to the shadow link. That way all the joints are trivially
  assembled into a tree. Then it will add a weld constraint between the shadow
  link and the original link from which it was split. In the initial
  configuration, this weld is not satisfied, so we must assemble the linkage
  before we can simulate it.
*/
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

namespace examples {
namespace multibody {
namespace four_bar {
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

// The mechanism, as drawn in the figure at the top of this file. Each link is a
// thin rod along its own z axis, except the coupler which runs along x.
constexpr double kGroundLength = 4.0;   // Wd to Wr.
constexpr double kDriverLength = 1.0;   // Do to Dc.
constexpr double kCouplerLength = 4.8;  // Cd to Cr, with Co at the middle.
constexpr double kRockerLength = 2.0;   // Ro to Rc.
constexpr double kDriverMass = 1.0;     // kg
constexpr double kCouplerMass = 1.0;    // kg
constexpr double kRockerMass = 2.0;     // kg

// The visual geometry is cosmetic only: each link is drawn as a skinny box
// running between its two connection points, with a cylinder marking each of
// those points. Everything is drawn in the y = 0 plane the mechanism actually
// moves in, so links do overlap on screen where they are joined; the bars are
// drawn a little short of their pivots to keep that overlap small.
constexpr double kBarWidth = 0.2;      // In the plane of motion.
constexpr double kBarThickness = 0.1;  // Out of the plane of motion, i.e. y.
constexpr double kPivotRadius = 0.02;  // The child end of a joint.
constexpr double kFatPivotRadius = 2 * kPivotRadius;  // The parent end.
constexpr double kParentPinAlpha = 0.5;  // See through it to the child's pin.
constexpr double kPivotLength = 2 * kBarThickness;  // Pokes out either side.

// The coupler is the link that gets split in two to break the loop, so it is
// drawn see-through, and its shadow copy in a fainter shade of the same blue.
// Neither copy looks solid on its own; once the weld constraint has brought
// the two together they should read as the single solid link they were defined
// to be. See DrawShadowLink().
constexpr double kCouplerAlpha = 0.5;
constexpr double kShadowAlpha = 0.33;

// The shadow's bar is drawn a little slimmer than the coupler's -- the same
// length, but this fraction of its width and thickness -- so that once the two
// have come together the shadow is still there to see, nested just inside the
// coupler, rather than fighting it for the same surface.
constexpr double kShadowScale = 0.75;

// Frame rate of the assembly replay; see ReplayAssembly().
constexpr double kPlaybackFps = 30.0;

// The triad marking a frame; see DrawFrame().
constexpr double kFrameAxisLength = 0.25;   // meters
constexpr double kFrameAxisRadius = 0.006;  // meters

// The color of the two frames the loop-closing weld holds together: the
// coupler's link frame Co and its shadow copy's link frame. They are drawn
// identically -- same color, same size, opaque -- rather than in their own
// links' colors, both so that they stand out against the blues around them and
// so that they read as the single frame the weld says they are once it is
// satisfied. Defined here, in one place, so the two cannot drift apart.
Vector4d WeldFrameColor() {
  return Vector4d(0.833, 0.333, 0, 1);  // A dark orange.
}

// Returns the box that draws a link as a skinny bar running from `p_LStart`
// along `direction` (+x or +z), stopping just short of the connection point
// `length` away, along with its pose in the link frame. Bars whose link frame
// sits at one end pass p_LStart = 0; the ground and the coupler have their link
// frames at the middle instead, and pass half a length back along `direction`.
// `transverse_scale` slims the bar in the two directions across its length,
// leaving the length itself alone. This is split out from DrawBar() below
// because the shadow link is drawn with these same shapes but through a
// different API; see DrawShadowLink().
std::pair<RigidTransformd, Box> MakeBar(const Vector3d& p_LStart, double length,
                                        const Vector3d& direction,
                                        double transverse_scale = 1.0) {
  const bool along_x = direction.x() != 0.0;
  // Note that the long side is measured off the unscaled width, so that a
  // slimmed bar still runs exactly as far as the one it is a copy of.
  const double long_side = length - kBarWidth;
  const double width = transverse_scale * kBarWidth;
  const double thickness = transverse_scale * kBarThickness;
  return {
      RigidTransformd(p_LStart + 0.5 * length * direction),
      Box(along_x ? long_side : width, thickness, along_x ? width : long_side)};
}

// Returns the cylinder that draws a pivot pin at `p_LP`, along with its pose in
// the link frame. The revolute axes are all along y, while a cylinder's own
// axis is z, so we have to rotate it to match.
std::pair<RigidTransformd, Cylinder> MakePivot(const Vector3d& p_LP,
                                               double radius) {
  return {RigidTransformd(RotationMatrixd::MakeXRotation(M_PI_2), p_LP),
          Cylinder(radius, kPivotLength)};
}

// Returns the cylinder that draws the `axis`-th (0 = x, 1 = y, 2 = z) arm of
// the triad marking a frame whose pose in the link frame is `X_LF`, along with
// its own pose in the link frame. `radius_scale` slims the arm without
// shortening it, so that one triad can nest inside another drawn at the same
// place. Split out from DrawFrame() below because the shadow link's triad is
// drawn with these same shapes but through a different API; see
// DrawShadowLink().
std::pair<RigidTransformd, Cylinder> MakeFrameAxis(const RigidTransformd& X_LF,
                                                   int axis,
                                                   double radius_scale = 1.0) {
  // A cylinder's own axis is +z, so rotate it onto the axis being drawn (the z
  // axis needs no rotation), then shift it half its length so that it runs from
  // F's origin rather than being centered there.
  const RotationMatrixd R_FG =
      axis == 0   ? RotationMatrixd::MakeYRotation(M_PI_2)
      : axis == 1 ? RotationMatrixd::MakeXRotation(M_PI_2)
                  : RotationMatrixd::Identity();
  const Vector3d p_FG = 0.5 * kFrameAxisLength * Vector3d::Unit(axis);
  return {X_LF * RigidTransformd(R_FG, p_FG),
          Cylinder(radius_scale * kFrameAxisRadius, kFrameAxisLength)};
}

// Draws `link` as a skinny box running from `p_LStart` along `direction`
// (+x or +z), stopping just short of the connection point `length` away.
void DrawBar(const Link<double>& link, const Vector3d& p_LStart, double length,
             const Vector3d& direction, const Vector4d& color,
             MultibodyPlant<double>* plant) {
  const auto [X_LB, bar] = MakeBar(p_LStart, length, direction);
  plant->RegisterVisualGeometry(link, X_LB, bar, link.name() + "_bar", color);
}

// Draws a pivot pin on `link` at `p_LP`.
void DrawPivot(const Link<double>& link, const Vector3d& p_LP,
               const std::string& name, double radius, const Vector4d& color,
               MultibodyPlant<double>* plant) {
  const auto [X_LP, pivot] = MakePivot(p_LP, radius);
  plant->RegisterVisualGeometry(link, X_LP, pivot, name, color);
}

// Draws frame `F` as a triad, each axis a cylinder running from F's origin out
// along that axis. The triad is fixed to whichever link F is fixed to, so it
// moves with that link. All three axes are drawn in `color`, meant to be the
// color of that link: what matters here is WHICH LINK the frame belongs to, and
// the conventional red/green/blue-per-axis coloring would say nothing about
// that while clashing with the colors the links are already drawn in.
//
// Drake has no C++ frame-drawing service to call here. The equivalent exists
// only in Python, as pydrake.visualization.AddFrameTriadIllustration, which is
// written in Python rather than bound from C++; this is that recipe, minus its
// per-axis coloring.
void DrawFrame(const Frame<double>& F, const Vector4d& color,
               const std::string& name, MultibodyPlant<double>* plant,
               double radius_scale = 1.0) {
  const RigidTransformd X_LF = F.GetFixedPoseInBodyFrame();
  for (int axis = 0; axis < 3; ++axis) {
    const auto [X_LG, arm] = MakeFrameAxis(X_LF, axis, radius_scale);
    plant->RegisterVisualGeometry(F.link(), X_LG, arm,
                                  fmt::format("{}_{}_axis", name, "xyz"[axis]),
                                  color);
  }
}

// A joint is drawn as a pin through the two links it connects: a fat cylinder
// at its parent end and a smaller one at its child end, each in `color`, meant
// to be the color of the link that end is fixed to, so you can see at a glance
// which link each end of a joint belongs to. The fat one is drawn translucent
// so that the small one nested inside it shows through. The two ends get their
// own functions here because they need not end up on the same pair of links --
// see BuildFourBarLinkage().

// Draws the parent end of the joint named `name`, on the link that frame `Jp`
// is fixed to.
void DrawParentPin(const Frame<double>& Jp, const Vector4d& color,
                   const std::string& name, MultibodyPlant<double>* plant) {
  Vector4d translucent_color = color;
  translucent_color[3] = kParentPinAlpha;
  DrawPivot(Jp.link(), Jp.GetFixedPoseInBodyFrame().translation(),
            name + "_parent_pin", kFatPivotRadius, translucent_color, plant);
}

// Draws the child end of the joint named `name`, on the link that frame `Jc`
// is fixed to.
void DrawChildPin(const Frame<double>& Jc, const Vector4d& color,
                  const std::string& name, MultibodyPlant<double>* plant) {
  DrawPivot(Jc.link(), Jc.GetFixedPoseInBodyFrame().translation(),
            name + "_child_pin", kPivotRadius, color, plant);
}

// Draws both ends of the joint named `name`. The joints here are all drawn
// alike because they are all modeled alike -- nothing about the way this
// linkage is defined singles any one of them out. And every joint's two pins
// stay together at all times, in the unassembled configuration as much as in
// the assembled one: no joint is ever broken here. Breaking a loop splits a
// LINK instead, dividing the joints that link carries between the primary and
// its shadow; see DrawShadowLink().
void DrawJoint(const Frame<double>& Jp, const Vector4d& parent_color,
               const Frame<double>& Jc, const Vector4d& child_color,
               const std::string& name, MultibodyPlant<double>* plant) {
  DrawParentPin(Jp, parent_color, name, plant);
  DrawChildPin(Jc, child_color, name, plant);
}

// Draws `shadow` -- an ephemeral shadow link that Finalize() added in order to
// break the loop -- as a faint copy of the coupler's own blue: a slightly
// slimmer bar of `length` running along `direction`, just like the link it is
// a copy of, plus the one joint that came with it. Watching it is the point of
// drawing it at all, because the shadow is where the loop breaking is actually
// visible. A split link's joints are divided up between the two copies -- here
// the coupler keeps the joint at its Cd end, the driver_coupler, while the
// joint at its Cr end, the coupler_rocker, moved over to the shadow -- and it
// is the weld constraint's job to hold the two copies together so that they
// act like the one link they were defined to be. So the shadow bar is pinned
// to the rocker from the outset, exactly as the coupler_rocker joint demands,
// and what you see where the two blues are apart is that weld not yet
// satisfied. The weld acts at Co, the middle of both copies, so that is where
// to watch the two of them come together.
//
// MultibodyPlant::RegisterVisualGeometry() can't do this job: it is a
// pre-Finalize call, and a shadow link does not exist until Finalize() creates
// it. We can still register illustration geometry ourselves by going straight
// to SceneGraph, using the plant's own geometry source and the SceneGraph frame
// that Finalize() registered for the shadow -- so the plant goes on posing this
// geometry along with everything else it drew. Note that we can reuse the
// primary's geometry poses verbatim, because a shadow link's frame coincides
// with its primary's by construction.
void DrawShadowLink(const MultibodyPlant<double>& plant,
                    const Link<double>& shadow, const Vector3d& p_LStart,
                    double length, const Vector3d& direction,
                    SceneGraph<double>* scene_graph) {
  // A paler, fainter version of the coupler's blue, so that the two copies
  // look like one solid link where they overlap.
  const Vector4d pale_blue(0.6, 0.6, 1, kShadowAlpha);

  const SourceId source_id = plant.get_source_id().value();
  const FrameId frame_id = plant.GetBodyFrameIdOrThrow(shadow.index());
  auto add_geometry = [&](const RigidTransformd& X_LG, const Shape& shape,
                          const std::string& name, const Vector4d& color) {
    auto instance = std::make_unique<GeometryInstance>(X_LG, shape, name);
    instance->set_illustration_properties(
        geometry::MakePhongIllustrationProperties(color));
    scene_graph->RegisterGeometry(source_id, frame_id, std::move(instance));
  };

  // The bar, and then the one joint this copy carries: the parent end at the
  // far end of the bar. Nothing is drawn at the near end -- the joint that
  // attaches there stayed behind on the primary. Only the bar is slimmed; the
  // pin is left the same size as every other joint's, since it has no
  // counterpart on the coupler to overlap with.
  const auto [X_LB, bar] = MakeBar(p_LStart, length, direction, kShadowScale);
  add_geometry(X_LB, bar, shadow.name() + "_bar", pale_blue);
  const auto [X_LP, parent_pin] =
      MakePivot(p_LStart + length * direction, kFatPivotRadius);
  add_geometry(X_LP, parent_pin, shadow.name() + "_parent_pin", pale_blue);

  // Finally a triad at this copy's own origin, the twin of the one drawn at the
  // coupler's Co. THE WELD CONSTRAINT IS EXACTLY THE STATEMENT THAT THESE TWO
  // FRAMES ARE THE SAME FRAME, so the gap between the two triads is the weld
  // error itself: they start apart and end up exactly on top of one another.
  // The pose is identity because a shadow's link frame coincides with its
  // primary's by construction -- which is also why the weld's two offsets are
  // both identity. Unlike the bar, this is NOT slimmed and does not take the
  // shadow's pale blue: it is drawn exactly like the coupler's Co triad, so
  // that when the weld is satisfied the two become one orange frame.
  for (int axis = 0; axis < 3; ++axis) {
    const auto [X_LG, arm] = MakeFrameAxis(RigidTransformd(), axis, 2.0);
    add_geometry(X_LG, arm,
                 fmt::format("{}_origin_{}_axis", shadow.name(), "xyz"[axis]),
                 WeldFrameColor());
  }
}

// Shows the configuration in `context` in the visualizer, says what there is to
// see, and then (unless --nointeractive) waits for the user to press Enter
// before doing whatever `next` describes.
void ShowAndWait(const systems::Diagram<double>& diagram,
                 Simulator<double>* simulator, const std::string& message,
                 const std::string& next) {
  diagram.ForcedPublish(simulator->get_context());
  fmt::print("\n{}\n", message);
  if (FLAGS_interactive) {
    std::cout << "Press Enter to " << next << " . . . " << std::flush;
    std::string line;
    std::getline(std::cin, line);  // Just eats the line; EOF is fine too.
  }
  // Restart the simulator's real time reference point. It throttles itself to
  // --simulator_target_realtime_rate (1, i.e. real time, by default) by
  // comparing simulated time against wall clock time elapsed since Initialize()
  // or ResetStatistics(); without this it would count the time we just spent
  // waiting as time it has to make up, and then run flat out to catch up. Note
  // that this also zeroes the step counters, so the statistics printed at the
  // end are those of the final simulation phase.
  simulator->ResetStatistics();
}

// Adds a frame named `name`, fixed to `link` at `p_LF` and aligned with the
// link frame, and returns it.
const Frame<double>& AddFrame(const Link<double>& link, const std::string& name,
                              const Vector3d& p_LF,
                              MultibodyPlant<double>* plant) {
  return plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
      name, link, RigidTransformd(p_LF)));
}

// Adds the links, frames and joints of the figure at the top of this file to
// `plant`, along with cosmetic visual geometry. Note that nothing here says
// anything about a spanning tree: the four joints simply form a loop, and
// MultibodyPlant decides how to model it.
void BuildFourBarLinkage(MultibodyPlant<double>* plant) {
  // One color per link, World included. The coupler's blue is partly
  // transparent because that is the link that gets split; its shadow copy is
  // drawn in a fainter shade of it later on, by DrawShadowLink().
  const Vector4d green(0, 1, 0, 1), red(1, 0, 0, 1),
      blue(0, 0, 1, kCouplerAlpha), yellow(1, 1, 0, 1);

  // The ground "link" of the linkage is World itself; Wd and Wr are the frames
  // where the linkage attaches to it. Wo is at the middle, so the bar starts
  // half a length back from Wd.
  const Vector3d p_WoWd(-0.5 * kGroundLength, 0, 0);
  const Link<double>& world = plant->world_body();
  const Frame<double>& Wd = AddFrame(world, "Wd", p_WoWd, plant);
  const Frame<double>& Wr =
      AddFrame(world, "Wr", Vector3d(0.5 * kGroundLength, 0, 0), plant);
  DrawBar(world, p_WoWd, kGroundLength, Vector3d::UnitX(), green, plant);

  // The driver, a thin rod from Do along +z to Dc.
  const Link<double>& driver = plant->AddRigidBody(
      "driver", SpatialInertia<double>::ThinRodWithMassAboutEnd(
                    kDriverMass, kDriverLength, Vector3d::UnitZ()));
  const Frame<double>& Do = driver.body_frame();  // Do is the driver frame.
  const Frame<double>& Dc =
      AddFrame(driver, "Dc", Vector3d(0, 0, kDriverLength), plant);
  DrawBar(driver, Vector3d::Zero(), kDriverLength, Vector3d::UnitZ(), red,
          plant);

  // The coupler, a thin rod running along x from Cd to Cr, with its link frame
  // Co at the MIDDLE rather than at either end. That is what puts the loop
  // closing weld at the middle of this link: the weld holds the shadow copy's
  // link frame to this one, and a shadow's link frame coincides with its
  // primary's by construction. Note the matching change of inertia -- a rod
  // about its center of mass, not about one of its ends.
  const Vector3d p_CoCd(-0.5 * kCouplerLength, 0, 0);
  const Link<double>& coupler = plant->AddRigidBody(
      "coupler", SpatialInertia<double>::ThinRodWithMass(
                     kCouplerMass, kCouplerLength, Vector3d::UnitX()));
  const Frame<double>& Co = coupler.body_frame();  // Co is the coupler frame.
  const Frame<double>& Cd = AddFrame(coupler, "Cd", p_CoCd, plant);
  const Frame<double>& Cr =
      AddFrame(coupler, "Cr", Vector3d(0.5 * kCouplerLength, 0, 0), plant);
  DrawBar(coupler, p_CoCd, kCouplerLength, Vector3d::UnitX(), blue, plant);

  // The rocker, a thin rod from Ro along +z to Rc.
  const Link<double>& rocker = plant->AddRigidBody(
      "rocker", SpatialInertia<double>::ThinRodWithMassAboutEnd(
                    kRockerMass, kRockerLength, Vector3d::UnitZ()));
  const Frame<double>& Ro = rocker.body_frame();  // Ro is the rocker frame.
  const Frame<double>& Rc =
      AddFrame(rocker, "Rc", Vector3d(0, 0, kRockerLength), plant);
  DrawBar(rocker, Vector3d::Zero(), kRockerLength, Vector3d::UnitZ(), yellow,
          plant);

  // The four revolute joints, each connecting the two frames named for it in
  // the figure, with q = 0 when those two frames are coincident. Only the first
  // one is actuated; the rest are passive.
  const Vector3d axis = -Vector3d::UnitY();  // Out of the page, towards you.
  const RevoluteJoint<double>& world_driver = plant->AddJoint(
      std::make_unique<RevoluteJoint<double>>("world_driver", Wd, Do, axis));
  plant->AddJoint(
      std::make_unique<RevoluteJoint<double>>("world_rocker", Wr, Ro, axis));
  plant->AddJoint(
      std::make_unique<RevoluteJoint<double>>("driver_coupler", Dc, Cd, axis));
  plant->AddJoint(
      std::make_unique<RevoluteJoint<double>>("coupler_rocker", Cr, Rc, axis));
  plant->AddJointActuator("driver_torque", world_driver);

  // A pin for each of those four joints, drawn from the same pair of frames the
  // joint was defined with, so that what you see is what was asked for. Each
  // end takes the color of the link it is fixed to.
  DrawJoint(Wd, green, Do, red, "world_driver", plant);
  DrawJoint(Wr, green, Ro, yellow, "world_rocker", plant);
  DrawJoint(Dc, red, Cd, blue, "driver_coupler", plant);

  // The last joint gets only its child end here. Its parent end is at Cr, on
  // the coupler, and the coupler is the link that gets split in two to break
  // this loop (the report in do_main() names it). A split link's joints are
  // divided up between the two copies, and this is the one that ends up on the
  // shadow, so its parent pin is drawn there instead -- see DrawShadowLink().
  // The coupler keeps the joint at its Cd end, the driver_coupler above.
  DrawChildPin(Rc, yellow, "coupler_rocker", plant);

  // Draw Co, the coupler's own link frame, at the middle of the bar. This is
  // the frame the loop-closing weld constrains: DrawShadowLink() draws the
  // shadow copy's link frame the same way, and the weld says those two frames
  // are one and the same. So the two triads drifting apart and then landing on
  // top of one another IS the weld, made visible.
  DrawFrame(Co, WeldFrameColor(), "Co", plant, 2.0);

  // Draw Cr as well, on the coupler. This is where the primary and its shadow
  // visibly tell different stories: the loop joint acts on the SHADOW's copy of
  // Cr, which is pinned to the rocker's Rc from t = 0, while the triad here
  // marks where the coupler's own Cr is. The gap between this triad and the
  // coupler_rocker pin is exactly the loop closure error that do_main()
  // reports, and it closes as the linkage assembles.
  DrawFrame(Cr, blue, "Cr", plant);
}

// Returns the loop closure error, in meters. The loop joint is the one the
// modeler chose to retarget onto the shadow link: its mobilizer moves the
// shadow, not the user's coupler link, and it is the weld constraint that keeps
// the shadow and the coupler together. So the two frames we named for that
// joint are NOT held together by the joint any more, and the distance between
// them is exactly the loop closure error: large in the unassembled starting
// configuration, and driven to (nearly) zero as the model assembles.
double CalcLoopClosureError(const MultibodyPlant<double>& four_bar,
                            const Context<double>& context) {
  const Joint<double>& loop_joint = four_bar.GetJointByName("coupler_rocker");
  return four_bar
      .CalcRelativeTransform(context, loop_joint.frame_on_parent(),
                             loop_joint.frame_on_child())
      .translation()
      .norm();
}

// One configuration the linkage passed through on its way to being assembled,
// along with the loop closure error there. Assemble() records these so that
// ReplayAssembly() can show them; see the comment there for why they have to be
// recorded and replayed rather than just watched as they happen.
struct AssemblyStep {
  VectorXd q;
  double error{};
};

// Assembles the linkage, with the driver held at `driver_angle` radians, and
// returns how long assembly took in seconds of simulation time. The
// configurations it passes through on the way are appended to `path`.
//
// Note that nothing here solves the loop closure equations. The weld constraint
// that Finalize() added to close the loop does all of the work: it pulls the
// two halves of the split link together over the first few steps, and all this
// function does is step until they have arrived.
//
// The driver is locked while that happens. Assembly is dynamic, so every
// unlocked joint moves as the loop closes, and the driver would otherwise
// arrive at whatever angle the transient left it at. Locking it makes the
// outcome the one asked for: the driver stays at `driver_angle` and the rest of
// the linkage assembles around it.
//
// On return the linkage is assembled and at rest, the driver is released, and
// the simulator's clock is back at zero, so that the caller can simulate from
// the assembled configuration as though it had been the starting point all
// along.
double Assemble(const systems::Diagram<double>& diagram,
                Simulator<double>* simulator,
                const MultibodyPlant<double>& four_bar,
                Context<double>* four_bar_context, double driver_angle,
                std::vector<AssemblyStep>* path) {
  const RevoluteJoint<double>& driver_joint =
      four_bar.GetJointByName<RevoluteJoint>("world_driver");
  driver_joint.set_angle(four_bar_context, driver_angle);
  driver_joint.Lock(four_bar_context);

  // The starting configuration. The monitor below runs after each step, so it
  // would otherwise miss the one configuration nobody stepped into.
  path->push_back({four_bar.GetPositions(*four_bar_context),
                   CalcLoopClosureError(four_bar, *four_bar_context)});

  // Step until the loop closure error is down to kAssembledTolerance, giving up
  // at kAssemblyDeadline in case it never gets there.
  const double kAssembledTolerance = 1.0e-3;  // meters
  const double kAssemblyDeadline = 0.5;       // seconds
  simulator->set_monitor([&four_bar, &diagram, kAssembledTolerance,
                          path](const Context<double>& root_context) {
    const Context<double>& plant_context =
        four_bar.GetMyContextFromRoot(root_context);
    const double error = CalcLoopClosureError(four_bar, plant_context);
    path->push_back({four_bar.GetPositions(plant_context), error});
    if (error < kAssembledTolerance) {
      return EventStatus::ReachedTermination(&diagram, "loop closed");
    }
    return EventStatus::Succeeded();
  });
  simulator->AdvanceTo(kAssemblyDeadline);
  simulator->clear_monitor();
  const double assembly_time = simulator->get_context().get_time();

  // Release the driver now that the linkage is assembled, so that the whole
  // mechanism is free to move.
  driver_joint.Unlock(four_bar_context);

  // Hand back the assembled configuration as a clean starting point. Assembly
  // took real motion over real time, so the linkage arrived here already moving
  // and with the clock a few milliseconds in. Zero both. The positions are
  // deliberately left alone -- they are the assembled configuration we just
  // worked to reach.
  four_bar.SetVelocities(four_bar_context,
                         VectorXd::Zero(four_bar.num_velocities()));
  simulator->get_mutable_context().SetTime(0.0);

  // Initialize() is required after changing the time, not merely advisable: the
  // time-triggered events have to be recalculated in case one is due at the new
  // starting time, and AdvanceTo() throws if the call is missing.
  simulator->Initialize();

  return assembly_time;
}

// Replays the recorded assembly in the visualizer, spread over `duration`
// seconds of wall clock, and leaves the linkage back in its assembled
// configuration. Does nothing if `duration` is not positive.
//
// WHY REPLAY IT RATHER THAN JUST WATCH IT HAPPEN. Assembly is not a motion the
// mechanism performs; it is the solver enforcing the loop-closing constraint,
// and it is over within a few milliseconds of simulated time -- comfortably
// less than one visualizer frame, so hardly any of it is ever published.
//
// How the solver gets there is its own business, and it varies: the discrete
// SAP solver takes a handful of steps, while the continuous CENIC integrator
// takes many more. Neither count is something this file chooses, and neither is
// a duration you can dial. Making the time step smaller does not help, and with
// SAP does not even change the step count -- the loop closes in the same number
// of steps, ending at the same loop closure error, whether the step is 1e-2 s
// or 1e-4 s, because the weld is rigid and SAP stiffens it in proportion as the
// step shrinks. So the only reliable way to see the assembly is to keep the
// configurations and show them afterwards, which is what this does.
//
// The playback timeline is therefore openly cosmetic, and is deliberately built
// so as NOT to depend on how many steps the solver took or how they were
// spaced: frames are placed by how much loop closure error is left rather than
// by step, so the two halves of the split link close on each other steadily and
// the replay looks the same whichever solver produced it. Replaying the
// recorded steps at equal times would be a truer time-lapse of that particular
// solver, but the steps are not evenly spaced in the distance they cover --
// most of the closing happens in the first few -- so it looks like a snap
// followed by nothing.
void ReplayAssembly(const systems::Diagram<double>& diagram,
                    Simulator<double>* simulator,
                    const MultibodyPlant<double>& four_bar,
                    Context<double>* four_bar_context,
                    const std::vector<AssemblyStep>& path, double duration) {
  if (duration <= 0.0 || std::ssize(path) < 2) return;

  const VectorXd q_assembled = four_bar.GetPositions(*four_bar_context);
  const double error_start = path.front().error;
  const int num_frames = std::max(2, static_cast<int>(duration * kPlaybackFps));
  const auto frame_period = std::chrono::duration<double>(1.0 / kPlaybackFps);

  for (int frame = 0; frame < num_frames; ++frame) {
    // Walk the error down to zero in equal increments, and ask the recording
    // where the linkage was when it had that much error left to go.
    const double error =
        error_start * (1.0 - static_cast<double>(frame) / (num_frames - 1));
    int i = 0;
    while (i + 2 < std::ssize(path) && path[i + 1].error > error) ++i;
    // Interpolate across the bracketing pair. Steps that made no progress
    // (which the halving never quite does, but guard anyway) contribute their
    // endpoint rather than dividing by zero.
    const double e0 = path[i].error, e1 = path[i + 1].error;
    const double w =
        e0 > e1 ? std::clamp((e0 - error) / (e0 - e1), 0.0, 1.0) : 1.0;
    four_bar.SetPositions(four_bar_context,
                          path[i].q + w * (path[i + 1].q - path[i].q));

    diagram.ForcedPublish(simulator->get_context());
    std::this_thread::sleep_for(frame_period);
  }

  // Put back the configuration the caller handed us. Only positions were ever
  // touched here, and the clock never moved, so there is nothing else to undo
  // and no need to re-Initialize() the simulator.
  four_bar.SetPositions(four_bar_context, q_assembled);
}

int do_main() {
  // Build the MultibodyPlant and SceneGraph. The loop is closed with a weld
  // constraint, so all this model asks of the solver is that it can enforce
  // one. Discrete plants use SAP, which can: every discrete contact model
  // approximation but TAMSI uses SAP, and the default (kLagged) is one of
  // those, so there is nothing to select here.
  DiagramBuilder<double> builder;
  auto [four_bar, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder, FLAGS_time_step);

  // Opt in to automatic modeling of closed topologies. Without this, Finalize()
  // below would throw: this model's joints form a loop, and modeling one takes
  // machinery (a split link and a constraint to hold the halves together) that
  // MultibodyPlant will not introduce behind your back.
  four_bar.SetEnableLoopTopology(true);

  BuildFourBarLinkage(&four_bar);

  // Note that the linkage moves in the World x-z plane, which contains Drake's
  // default -z gravity, so there is nothing to adjust there.

  // We are done defining the model. Finalize is where the loop modeling
  // happens.
  four_bar.Finalize();

  // Report what the automatic modeling did. Each loop costs one split link --
  // the original ("primary") link and an ephemeral "shadow" copy of it, sharing
  // the original's mass evenly -- plus one weld constraint holding the two
  // copies together.
  fmt::print("Modeled {} kinematic loop(s) with {} constraint(s).\n",
             four_bar.num_loop_constraints(), four_bar.num_constraints());
  // The shadow link gets drawn here rather than in BuildFourBarLinkage() above,
  // since it did not exist until Finalize(). The modeler split the coupler (see
  // the name reported below), so it is the coupler's geometry we copy.
  for (BodyIndex index(0); index < four_bar.num_bodies(); ++index) {
    const Link<double>& body = four_bar.get_body(index);
    if (body.is_ephemeral()) {
      fmt::print("  shadow link '{}' was added to break a loop.\n",
                 body.name());
      DrawShadowLink(four_bar, body, Vector3d(-0.5 * kCouplerLength, 0, 0),
                     kCouplerLength, Vector3d::UnitX(), &scene_graph);
    }
  }

  visualization::AddDefaultVisualization(&builder);

  auto diagram = builder.Build();

  // Create a context for this system and sub-context for the four bar system.
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  Context<double>& four_bar_context =
      four_bar.GetMyMutableContextFromRoot(diagram_context.get());

  // A constant source for applied torque at the world_driver joint. This is the
  // only actuated joint in the model.
  four_bar.get_actuation_input_port().FixValue(&four_bar_context,
                                               FLAGS_applied_torque);

  // Note that we set no initial conditions at all. The default context has
  // every joint angle at zero, which is the unassembled configuration drawn in
  // the figure at the top of this file. There is no need to solve the loop
  // closure equations for an assembled configuration to start from: the weld
  // constraint closing the loop pulls the two halves of the split link
  // together over the first few steps, assembling the linkage, after which it
  // swings under gravity.

  std::unique_ptr<Simulator<double>> simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));

  // Show the linkage as defined, before anything has moved.
  simulator->Initialize();
  fmt::print("Loop closure error as defined: {:.4f} m\n",
             CalcLoopClosureError(four_bar, four_bar_context));
  ShowAndWait(*diagram, simulator.get(),
              "The linkage is unassembled: the coupler and its pale shadow copy"
              " are apart, so the weld constraint holding the two halves of the"
              " split link together is not satisfied yet. Every joint already"
              " is -- look for each one's two pins sitting together.",
              "assemble the linkage");

  std::vector<AssemblyStep> assembly_path;
  const double assembly_time =
      Assemble(*diagram, simulator.get(), four_bar, &four_bar_context,
               FLAGS_driver_angle, &assembly_path);

  fmt::print("Loop closure error after assembling for {:.4f} s: {:.3g} m\n",
             assembly_time, CalcLoopClosureError(four_bar, four_bar_context));
  fmt::print("Assembly took {} steps; replaying them over {} s.\n",
             std::ssize(assembly_path) - 1, FLAGS_assembly_playback_time);

  // Rewind and replay, so that the assembly can actually be seen happening.
  ReplayAssembly(*diagram, simulator.get(), four_bar, &four_bar_context,
                 assembly_path, FLAGS_assembly_playback_time);
  ShowAndWait(*diagram, simulator.get(),
              "The linkage is assembled: the shadow now lies on top of the"
              " coupler it is a copy of, the two of them together looking like"
              " the one solid link they were defined to be.",
              "simulate");

  // Simulate the assembled mechanism swinging under gravity.
  simulator->AdvanceTo(FLAGS_simulation_time);

  fmt::print("Loop closure error after simulating {} s: {:.3g} m\n",
             FLAGS_simulation_time,
             CalcLoopClosureError(four_bar, four_bar.GetMyContextFromRoot(
                                                simulator->get_context())));

  // Print some useful statistics.
  PrintSimulatorStatistics(*simulator);

  return 0;
}

}  // namespace
}  // namespace four_bar
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A four bar linkage demo demonstrating MultibodyPlant's automatic "
      "modeling of closed kinematic loops. Launch meldis before running this "
      "example.");
  // Changes the default realtime rate to 1.0, so the visualization looks
  // realistic. Otherwise, it finishes so fast that we can't appreciate the
  // motion. Users can still change it on command-line, e.g. "
  // --simulator_target_realtime_rate=0.5" to slow it down.
  FLAGS_simulator_target_realtime_rate = 1.0;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::four_bar::do_main();
}
