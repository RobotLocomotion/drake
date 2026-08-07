/* Tests for automatic handling of closed-topology (looped) systems, enabled via
MultibodyPlant::SetEnableLoopTopology(). When enabled, Finalize() breaks each
kinematic loop using the shadow links and loop constraints produced by the
underlying LinkJointGraph/SpanningForest. This file tests that
 - adding shadow links splits mass properties properly among the primary
   link and all of its shadows,
 - splitting a primary link into more than two pieces works,
 - loop joints are correctly retargeted from a frame on the primary to
   a corresponding frame on the shadow,
 - the loop weld constraints are added to the plant and correctly joint
   the primary and shadow links together at their origins, and
 - the activate/deactivate parameters are properly allocated for the
   loop weld constraints and work properly.

Most tests use a planar four-bar linkage (three moving links -- driver, coupler,
rocker -- plus World, connected by four revolute joints) which forms a single
kinematic loop. See examples/multibody/four_bar for runnable examples of similar
mechanisms. The multiple-shadow links test uses a double loop that gives one
primary link two shadows. */

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/kinematics_vector.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/autodiff.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/constraint_specs.h"
#include "drake/multibody/plant/internal_geometry_names.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/shadow_frame.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace {

/* A planar four-bar linkage described with no spanning tree specified, i.e. as
a raw loop. There are three moving links (driver, coupler, rocker) plus World
and four revolute joints; the linkage moves in the World x-z plane (so the
default -z gravity lies in its plane of motion) and all four revolute axes point
in +y, into the page as drawn below.

The linkage is a parallelogram: the driver and the rocker are the same length,
so the coupler stays parallel to the ground and every pose in the file below is
exact. Note that this model is _assembled_ as written -- it has to be, since an
SDF joint has a single joint frame from which both of Drake's joint frames (on
parent and on child) are derived, making a parsed model loop-consistent at q = 0
by construction. Watching a loop close from an unassembled start needs the C++
API; see examples/multibody/four_bar.

Each link's frame origin is at its inboard pivot; each mass center is at the
link's midpoint, with the rotational inertia of a uniform density thin rod
(mL^2/12).

      Dc,Co *==============================* Cr,Rc
            |          coupler C           |
            |           2m, 1kg            |
   driver D |                              | rocker R
    1m, 1kg |                              | 1m, 2kg
            |                              |
      Wd,Do *------------- Wo -------------* Wr,Ro          Wz
                      World 2m                              |  Wy
                                                            | /
                                                            +----- Wx

In parent-child order the joints connect World-Do, World-Ro, Dc-Co, Cr-Rc. Each
label pair above marks a pivot, where the two named frames coincide in this
assembled configuration; Wd and Wr are the World-fixed pivots, which need no
frames of their own -- because each child link's origin sits at its pivot,
naming <parent>world</parent> puts the joint's World-side frame at the child
origin. */
constexpr char kFourBarLoopSdf[] = R"""(
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="four_bar_loop">
    <link name="driver">
      <pose>-1 0 0 0 0 0</pose>
      <inertial>
        <pose>0 0 0.5 0 0 0</pose>
        <mass>1</mass>
        <inertia>
          <ixx>0.0833333333333333</ixx>
          <iyy>0.0833333333333333</iyy>
          <izz>0</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </inertia>
      </inertial>
    </link>
    <frame name="Dc" attached_to="driver">
      <pose relative_to="driver">0 0 1 0 0 0</pose>
    </frame>
    <link name="rocker">
      <pose>1 0 0 0 0 0</pose>
      <inertial>
        <pose>0 0 0.5 0 0 0</pose>
        <mass>2</mass>
        <inertia>
          <ixx>0.1666666666666667</ixx>
          <iyy>0.1666666666666667</iyy>
          <izz>0</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </inertia>
      </inertial>
    </link>
    <frame name="Rc" attached_to="rocker">
      <pose relative_to="rocker">0 0 1 0 0 0</pose>
    </frame>
    <link name="coupler">
      <pose>-1 0 1 0 0 0</pose>
      <inertial>
        <pose>1 0 0 0 0 0</pose>
        <mass>1</mass>
        <inertia>
          <ixx>0</ixx>
          <iyy>0.3333333333333333</iyy>
          <izz>0.3333333333333333</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </inertia>
      </inertial>
    </link>
    <frame name="Cr" attached_to="coupler">
      <pose relative_to="coupler">2 0 0 0 0 0</pose>
    </frame>
    <joint name="world_driver" type="revolute">
      <parent>world</parent>
      <child>driver</child>
      <axis><xyz expressed_in="__model__">0 1 0</xyz></axis>
    </joint>
    <joint name="world_rocker" type="revolute">
      <parent>world</parent>
      <child>rocker</child>
      <axis><xyz expressed_in="__model__">0 1 0</xyz></axis>
    </joint>
    <joint name="driver_coupler" type="revolute">
      <parent>Dc</parent>
      <child>coupler</child>
      <axis><xyz expressed_in="__model__">0 1 0</xyz></axis>
    </joint>
    <joint name="coupler_rocker" type="revolute">
      <parent>Cr</parent>
      <child>Rc</child>
      <axis><xyz expressed_in="__model__">0 1 0</xyz></axis>
    </joint>
  </model>
</sdf>
)""";

/* A four-bar loop model with automatic loop modeling enabled: a MultibodyPlant
and a SceneGraph, with the driver and the coupler (the link we know gets split;
see CouplerIsSplit) each given one visual and one collision geometry. The plant
is left _unfinalized_, since Finalize() is what does the loop modeling these
tests are about; each test finalizes when it is ready. The DiagramBuilder is
kept only to own the two systems -- the tests work with the plant and its
Context directly rather than building a diagram. */
class FourBar {
 public:
  explicit FourBar(double time_step = 0.0 /* continuous */) {
    plant_ = &AddMultibodyPlantSceneGraph(&builder_, time_step).plant;
    MultibodyPlant<double>& plant = *plant_;
    plant.SetEnableLoopTopology(true);
    Parser(&plant).AddModelsFromString(kFourBarLoopSdf, "sdf");
    for (const std::string name : {"driver", "coupler", "rocker"}) {
      const Link<double>& link = plant.GetBodyByName(name);
      plant.RegisterVisualGeometry(link, math::RigidTransformd(),
                                   geometry::Sphere(0.1), name + "_visual");
      plant.RegisterCollisionGeometry(
          link, math::RigidTransformd(), geometry::Sphere(0.1),
          name + "_collision", CoulombFriction<double>(1.0, 1.0));
    }
  }

  MultibodyPlant<double>& plant() { return *plant_; }

 private:
  systems::DiagramBuilder<double> builder_;
  MultibodyPlant<double>* plant_{};
};

/* Ensure there is one ephemeral (shadow) link in `plant`, i.e., the
link created when the coupler is split at Finalize() - and return it. */
const Link<double>& GetSoleShadowLink(const MultibodyPlant<double>& plant) {
  std::vector<LinkIndex> shadows;
  for (LinkIndex i(0); i < plant.num_bodies(); ++i) {
    if (plant.get_body(i).is_ephemeral()) shadows.push_back(i);
  }
  EXPECT_EQ(shadows.size(), 1);
  // Shadow links are guaranteed to have higher indexes than any user link.
  EXPECT_EQ(shadows[0], LinkIndex(plant.num_bodies() - 1));
  return plant.get_body(shadows[0]);
}

/* The four-bar's kinematic loop is handled by splitting the coupler link's mass
equally into a primary and shadow link. The coupler link is split because the
SpanningForest minimizes the maximum branch length (hence it splits the middle
link): the coupler's primary link is reached via the driver and the coupler's
shadow link is reached via the rocker, yielding two length-2 branches. The
primary link is the coupler except half of its mass properties are given to
the coupler shadow link (which is named coupler$1). */
GTEST_TEST(ClosedTopologyTest, CouplerIsSplit) {
  FourBar four_bar;
  MultibodyPlant<double>& plant = four_bar.plant();
  plant.Finalize();

  // World + driver + coupler + rocker + the coupler's shadow.
  EXPECT_EQ(plant.num_bodies(), 5);

  // There is exactly one shadow, and it is the coupler's.
  const Link<double>& shadow = GetSoleShadowLink(plant);
  EXPECT_EQ(shadow.name(), "coupler$1");
  EXPECT_TRUE(shadow.is_ephemeral());

  // Every user-defined link remains non-ephemeral.
  for (const char* name : {"driver", "coupler", "rocker"}) {
    EXPECT_FALSE(plant.GetBodyByName(name).is_ephemeral());
  }
}

/* Objects auto-created during Finalize() have to be marked as ephemeral, per
the MultibodyElement::is_ephemeral() contract. That includes link frames which
are co-created implicitly whenever a link is auto-created (in contrast to an
explicit user call to "add frame"). */
GTEST_TEST(ClosedTopologyTest, ShadowLinkFramesAreEphemeral) {
  FourBar four_bar;
  MultibodyPlant<double>& plant = four_bar.plant();
  plant.Finalize();
  const Link<double>& shadow = GetSoleShadowLink(plant);
  EXPECT_TRUE(shadow.body_frame().is_ephemeral());

  // Since shadow links are auto-created (not user-created), every frame
  // fixed to a shadow link is auto-created during Finalize(): the
  // shadow's own link frame, plus the frames the retargeted loop joint's
  // mobilizer needed on the shadow side.
  int num_shadow_frames = 0;
  for (FrameIndex index(0); index < plant.num_frames(); ++index) {
    const Frame<double>& frame = plant.get_frame(index);
    if (frame.body().index() != shadow.index()) continue;
    ++num_shadow_frames;
    EXPECT_TRUE(frame.is_ephemeral()) << frame.name();
  }
  EXPECT_GE(num_shadow_frames, 2);

  // The user's links keep non-ephemeral link frames.
  EXPECT_FALSE(plant.world_body().body_frame().is_ephemeral());
  for (const char* name : {"driver", "coupler", "rocker"}) {
    EXPECT_FALSE(plant.GetBodyByName(name).body_frame().is_ephemeral());
  }

  // Scalar conversion must carry the flags over; unlike the pre-finalize path,
  // it creates the shadow's link frame by cloning rather than by adding a link.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
      systems::System<double>::ToAutoDiffXd(plant);
  const Link<AutoDiffXd>& shadow_ad = plant_ad->GetBodyByName("coupler$1");
  EXPECT_TRUE(shadow_ad.is_ephemeral());
  EXPECT_TRUE(shadow_ad.body_frame().is_ephemeral());
  EXPECT_FALSE(plant_ad->GetBodyByName("coupler").body_frame().is_ephemeral());
}

/* Ensure the coupler link's mass properties is split evenly with its shadow
link. Note: The algorithm used here splits individual links, not mobods.
(Reminder: mobod mass properties may include multiple fused links). Here, the
coupler primary link and the coupler's shadow link coupler$1 each carry half of
the coupler's inertia (and are identical to each other, since their link frames
coincide). The user-facing per-link (default) mass is unchanged for the coupler
(its full declared 1 kg) while the shadow link's default reflects its half
share; the effective (split) inertia checked here is what drives the dynamics.
Also ensure the unsplit links (driver and rocker) are unaffected. */
GTEST_TEST(ClosedTopologyTest, ShadowMassIsSplitEvenly) {
  constexpr double kTol = 1e-14;
  FourBar four_bar;
  MultibodyPlant<double>& plant = four_bar.plant();
  plant.Finalize();
  auto context = plant.CreateDefaultContext();

  const Link<double>& coupler = plant.GetBodyByName("coupler");
  const Link<double>& shadow = plant.GetBodyByName("coupler$1");

  // The user-facing (model default) mass of the coupler is untouched (its full
  // declared 1 kg). The shadow's default mass properties reflect its share of
  // the even split: half the coupler's mass, with the same com and unit inertia
  // (hence half the rotational inertia). This default is informational only;
  // the effective split inertia checked below is what drives the dynamics.
  EXPECT_EQ(coupler.default_mass(), 1.0);
  EXPECT_EQ(shadow.default_mass(), 0.5);
  EXPECT_TRUE(
      CompareMatrices(shadow.default_com(), coupler.default_com(), kTol));
  EXPECT_TRUE(CompareMatrices(
      shadow.default_rotational_inertia().CopyToFullMatrix3(),
      0.5 * coupler.default_rotational_inertia().CopyToFullMatrix3(), kTol));

  // Ask each link for its own effective spatial inertia about its origin Lo,
  // expressed in its link frame L.
  auto M_LLo_L = [&](const Link<double>& link) {
    // This function reads the loop-split value from the FrameBodyPoseCache.
    return link.CalcSpatialInertiaInBodyFrame(*context);
  };
  const SpatialInertia<double> M_LLo_L_coupler = M_LLo_L(coupler);
  const SpatialInertia<double> M_LLo_L_shadow = M_LLo_L(shadow);

  // The coupler's primary link and its shadow link each carry half of
  // the coupler's 1 kg, have colocated centers of mass, and have
  // equal rotational inertia (their link frames coincide).
  EXPECT_NEAR(M_LLo_L_coupler.get_mass(), 0.5, kTol);
  EXPECT_NEAR(M_LLo_L_shadow.get_mass(), 0.5, kTol);
  EXPECT_TRUE(CompareMatrices(M_LLo_L_shadow.get_com(),
                              M_LLo_L_coupler.get_com(), kTol));
  EXPECT_TRUE(CompareMatrices(
      M_LLo_L_shadow.CalcRotationalInertia().CopyToFullMatrix3(),
      M_LLo_L_coupler.CalcRotationalInertia().CopyToFullMatrix3(), kTol));

  // The coupler's primary link and its shadow link each have half the
  // user-declared coupler's rotational inertia about Lo, expressed in L.
  const Matrix3<double> half_I_LLo_L =
      0.5 * coupler.default_rotational_inertia().CopyToFullMatrix3();
  EXPECT_TRUE(CompareMatrices(
      M_LLo_L_coupler.CalcRotationalInertia().CopyToFullMatrix3(), half_I_LLo_L,
      kTol));
  EXPECT_TRUE(CompareMatrices(
      M_LLo_L_shadow.CalcRotationalInertia().CopyToFullMatrix3(), half_I_LLo_L,
      kTol));

  // The unsplit links keep their full mass (driver 1 kg, rocker 2 kg).
  EXPECT_NEAR(M_LLo_L(plant.GetBodyByName("driver")).get_mass(), 1.0, kTol);
  EXPECT_NEAR(M_LLo_L(plant.GetBodyByName("rocker")).get_mass(), 2.0, kTol);
}

/* A shadow link has no spatial-inertia parameter of its own; its effective mass
properties are sourced from the FrameBodyPoseCache (mirroring its primary's
share of the split). Consequently the per-body context accessors report the
split value for both the primary and the shadow, and system-wide mass
aggregates (which loop over all links, shadows included) count the physical mass
exactly once rather than double-counting the shadow. */
GTEST_TEST(ClosedTopologyTest, ShadowHasNoIndependentInertiaParameter) {
  constexpr double kTol = 1e-14;
  FourBar four_bar;
  MultibodyPlant<double>& plant = four_bar.plant();
  plant.Finalize();
  auto context = plant.CreateDefaultContext();

  const Link<double>& coupler = plant.GetBodyByName("coupler");
  const Link<double>& shadow = plant.GetBodyByName("coupler$1");

  // The context accessor reports each copy's split share (0.5 kg), not the
  // coupler's full declared 1 kg. (default_mass() still reports the declared
  // 1 kg for the primary; see ShadowMassIsSplitEvenly.)
  EXPECT_NEAR(coupler.get_mass(*context), 0.5, kTol);
  EXPECT_NEAR(shadow.get_mass(*context), 0.5, kTol);

  // Ensure the system's total mass properly accounts for the split-link
  // coupler (0.5 * primary_link_mass + 0.5 * shadow_link_mass = 1 kg),
  // so total mass is: driver 1 kg + rocker 2 kg + coupler 1 kg = 4 kg.
  EXPECT_NEAR(plant.CalcTotalMass(*context), 4.0, kTol);
}

/* Because the split is recomputed in the FrameBodyPoseCache from the primary's
parameter, changing the primary's mass at runtime re-splits across the shadow
automatically -- the primary is the single source of truth. Setting mass on the
shadow directly is disallowed. */
GTEST_TEST(ClosedTopologyTest, RuntimeMassChangeReSplitsAndShadowIsReadOnly) {
  constexpr double kTol = 1e-14;
  FourBar four_bar;
  MultibodyPlant<double>& plant = four_bar.plant();
  plant.Finalize();
  auto context = plant.CreateDefaultContext();

  const Link<double>& coupler = plant.GetBodyByName("coupler");
  const Link<double>& shadow = plant.GetBodyByName("coupler$1");

  // Set the coupler's (physical) mass to 3 kg; the split follows to 1.5 kg on
  // each of the coupler and its shadow, and the total tracks accordingly
  // (driver 1 + rocker 2 + coupler 3 = 6 kg).
  coupler.SetMass(context.get(), 3.0);
  EXPECT_NEAR(coupler.get_mass(*context), 1.5, kTol);
  EXPECT_NEAR(shadow.get_mass(*context), 1.5, kTol);
  EXPECT_NEAR(plant.CalcTotalMass(*context), 6.0, kTol);

  // The shadow's mass properties are not independently settable.
  DRAKE_EXPECT_THROWS_MESSAGE(shadow.SetMass(context.get(), 1.0),
                              ".*coupler\\$1.*ephemeral shadow link.*");
}

/* A shadow link carries no geometry of its own. It is an internal modeling
artifact coincident with its primary link, but it must still have an (empty)
entry in the plant's per-body geometry arrays, which are indexed by BodyIndex
and so must stay dense over num_bodies(). Shadow links are created inside
MultibodyTree::Finalize() rather than by MultibodyPlant::AddRigidBody() (which
is what normally extends those arrays), so Finalize() has to extend them. */
GTEST_TEST(ClosedTopologyTest, ShadowLinkHasEmptyGeometryEntries) {
  FourBar four_bar;
  MultibodyPlant<double>& plant = four_bar.plant();
  plant.Finalize();

  const Link<double>& coupler = plant.GetBodyByName("coupler");
  const Link<double>& shadow = plant.GetBodyByName("coupler$1");

  // The primary link keeps the geometry registered on it.
  EXPECT_EQ(plant.GetVisualGeometriesForBody(coupler).size(), 1);
  EXPECT_EQ(plant.GetCollisionGeometriesForBody(coupler).size(), 1);

  // The shadow link has an empty entry. Without that empty entry,
  // these two lookups would read past the end of the per-body arrays.
  EXPECT_TRUE(plant.GetVisualGeometriesForBody(shadow).empty());
  EXPECT_TRUE(plant.GetCollisionGeometriesForBody(shadow).empty());

  // Breaking the loop doesn't invent geometry: driver, coupler, rocker have
  // one visual and one collision geometry each.
  EXPECT_EQ(plant.num_visual_geometries(), 3);
  EXPECT_EQ(plant.num_collision_geometries(), 3);

  // The per-body arrays are copied wholesale during scalar conversion, so the
  // converted plant must agree with the tree it carries as well.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
      systems::System<double>::ToAutoDiffXd(plant);
  EXPECT_EQ(plant_ad->num_bodies(), plant.num_bodies());
  EXPECT_TRUE(
      plant_ad->GetVisualGeometriesForBody(plant_ad->GetBodyByName("coupler$1"))
          .empty());
  EXPECT_TRUE(
      plant_ad
          ->GetCollisionGeometriesForBody(plant_ad->GetBodyByName("coupler$1"))
          .empty());
}

/* A shadow link does, however, get a SceneGraph frame of its own. It is a body
like any other: it has a pose, which agrees with its primary's only to the
extent that the loop-closing weld constraint is satisfied -- an unassembled
model can start far from that. Nothing hangs geometry on that frame (a shadow
carries none, per the test above), but SceneGraph is where a body's pose is
published, and a consumer that walks the bodies looking for something to draw
has nothing to attach to without it. Like the geometry arrays above, this is
bookkeeping that AddRigidBody() would normally do, so Finalize() has to. */
GTEST_TEST(ClosedTopologyTest, ShadowLinkHasItsOwnSceneGraphFrame) {
  FourBar four_bar;
  MultibodyPlant<double>& plant = four_bar.plant();
  plant.Finalize();

  const Link<double>& coupler = plant.GetBodyByName("coupler");
  const Link<double>& shadow = plant.GetBodyByName("coupler$1");

  const std::optional<geometry::FrameId> shadow_frame =
      plant.GetBodyFrameIdIfExists(shadow.index());
  ASSERT_TRUE(shadow_frame.has_value());

  // It is the shadow's own frame, not a second reference to its primary's ...
  const geometry::FrameId coupler_frame =
      plant.GetBodyFrameIdOrThrow(coupler.index());
  EXPECT_NE(*shadow_frame, coupler_frame);

  // ... and it maps back to the shadow link.
  EXPECT_EQ(plant.GetBodyFromFrameId(*shadow_frame), &shadow);

  // The pose port reports that frame, with a pose that really is the shadow's
  // own. To be sure we aren't just looking at a copy of the primary's pose, we
  // move the loop's two branches independently: the driver carries the primary
  // coupler, while the rocker carries the shadow.
  std::unique_ptr<systems::Context<double>> context =
      plant.CreateDefaultContext();
  plant.GetJointByName<RevoluteJoint>("world_driver")
      .set_angle(context.get(), 0.3);
  plant.GetJointByName<RevoluteJoint>("world_rocker")
      .set_angle(context.get(), -0.4);

  const geometry::FramePoseVector<double>& poses =
      plant.get_geometry_pose_output_port()
          .Eval<geometry::FramePoseVector<double>>(*context);
  ASSERT_TRUE(poses.has_id(*shadow_frame));
  EXPECT_TRUE(
      poses.value(*shadow_frame)
          .IsExactlyEqualTo(plant.EvalBodyPoseInWorld(*context, shadow)));
  EXPECT_FALSE(poses.value(*shadow_frame)
                   .IsNearlyEqualTo(poses.value(coupler_frame), 1e-10));

  // Frame ids are copied wholesale during scalar conversion (where Finalize()
  // does not re-run), so the shadow keeps the frame it was given here.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
      systems::System<double>::ToAutoDiffXd(plant);
  EXPECT_EQ(plant_ad->GetBodyFrameIdIfExists(shadow.index()), shadow_frame);
}

/* Several post-finalize consumers (visualization helpers in particular) walk
every BodyIndex in [0, num_bodies()) and ask the plant for that body's
geometry. Before shadow links were given per-body geometry entries, those walks
read past the end of the arrays once a loop had been broken. Note that the
out-of-range read is undefined behavior rather than an exception, so this test
earns its keep in debug builds (where the density assertions in the accessors
fire) and under the memory sanitizers. */
GTEST_TEST(ClosedTopologyTest, WalkingEveryBodyForGeometryStaysInRange) {
  FourBar four_bar;
  MultibodyPlant<double>& plant = four_bar.plant();
  plant.Finalize();

  // Summing per-body counts over all bodies (shadows included) must reproduce
  // the plant-wide totals -- i.e. every body has an entry and no geometry is
  // counted twice.
  int num_visual = 0;
  int num_collision = 0;
  for (BodyIndex i(0); i < plant.num_bodies(); ++i) {
    const Link<double>& link = plant.get_body(i);
    num_visual += ssize(plant.GetVisualGeometriesForBody(link));
    num_collision += ssize(plant.GetCollisionGeometriesForBody(link));
  }
  EXPECT_EQ(num_visual, plant.num_visual_geometries());
  EXPECT_EQ(num_collision, plant.num_collision_geometries());

  // GeometryNames performs exactly that walk; it is the path taken by contact
  // visualization (see also ContactResultsToLcmSystem).
  internal::GeometryNames geometry_names;
  EXPECT_NO_THROW(geometry_names.ResetBasic(plant));
}

/* This is the same floating double loop as SpanningForest.DoubleLoop. The
middle branch reaches link6 first; when the other two branches subsequently
reach it, link6 is split twice to close the two loops:

     link2 ------> link5
       ^             |
       |             v
     link1 -> link3 -> link6
       |             ^
       v             |
     link4 ------> link7

After Finalize(), link6, link6$1, and link6$2 each carry one third of link6's
mass properties. */
GTEST_TEST(ClosedTopologyTest, OneLinkCanHaveTwoShadows) {
  constexpr double kTol = 1e-14;
  constexpr double kLink6Mass = 6.0;
  MultibodyPlant<double> plant(0.0 /* continuous */);
  plant.SetEnableLoopTopology(true);

  const SpatialInertia<double> unit_inertia =
      SpatialInertia<double>::MakeUnitary();
  const SpatialInertia<double> link6_inertia =
      SpatialInertia<double>::MakeFromCentralInertia(
          kLink6Mass, Vector3<double>(0.1, -0.2, 0.3),
          kLink6Mass * UnitInertia<double>::SolidSphere(0.4));

  // Index bodies from 1 to match the LinkIndex values in DoubleLoop.
  for (int i = 1; i <= 7; ++i) {
    const Link<double>& link = plant.AddRigidBody(
        "link" + std::to_string(i), i == 6 ? link6_inertia : unit_inertia);
    EXPECT_EQ(link.index(), LinkIndex(i));
  }

  // Preserve DoubleLoop's joint order; it determines branch and shadow order.
  const std::vector<std::pair<int, int>> joints{{1, 2}, {1, 4}, {1, 3}, {2, 5},
                                                {4, 7}, {3, 6}, {5, 6}, {7, 6}};
  for (int i = 0; i < ssize(joints); ++i) {
    const auto [parent, child] = joints[i];
    plant.AddJoint<RevoluteJoint>(
        "joint" + std::to_string(i), plant.get_body(LinkIndex(parent)), {},
        plant.get_body(LinkIndex(child)), {}, Vector3<double>::UnitZ());
  }

  plant.Finalize();

  // Before testing MultibodyPlant's handling of multiple shadows, confirm our
  // assumption about the topology produced for this graph. These are fatal
  // checks because the remaining test is meaningful only if link6 was split
  // twice as we are expecting, with the expected number of links and joints.
  ASSERT_NO_THROW(plant.GetBodyByName("link6$2"));
  // World + seven user links + two shadows; the ninth joint is the ephemeral
  // floating joint that attaches link1 to World.
  ASSERT_EQ(plant.num_bodies(), 10);
  ASSERT_EQ(plant.num_joints(), 9);

  // We got the topology we're expecting.
  const LinkIndex primary_index(6);
  const LinkIndex shadow1_index(8);
  const LinkIndex shadow2_index(9);
  auto context = plant.CreateDefaultContext();

  const Link<double>& primary = plant.GetBodyByName("link6");
  const Link<double>& shadow1 = plant.GetBodyByName("link6$1");
  const Link<double>& shadow2 = plant.GetBodyByName("link6$2");
  EXPECT_EQ(primary.index(), primary_index);
  EXPECT_EQ(shadow1.index(), shadow1_index);
  EXPECT_EQ(shadow2.index(), shadow2_index);
  EXPECT_FALSE(primary.is_ephemeral());
  EXPECT_TRUE(shadow1.is_ephemeral());
  EXPECT_TRUE(shadow2.is_ephemeral());
  EXPECT_NE(primary.mobod_index(), shadow1.mobod_index());
  EXPECT_NE(primary.mobod_index(), shadow2.mobod_index());
  EXPECT_NE(shadow1.mobod_index(), shadow2.mobod_index());

  // The primary retains the user's declared defaults. Each shadow's defaults
  // report its one-third share, including the rotational inertia about Lo.
  EXPECT_EQ(primary.default_mass(), kLink6Mass);
  for (const Link<double>* shadow : {&shadow1, &shadow2}) {
    EXPECT_NEAR(shadow->default_mass(), kLink6Mass / 3.0, kTol);
    EXPECT_TRUE(
        CompareMatrices(shadow->default_com(), primary.default_com(), kTol));
    EXPECT_TRUE(CompareMatrices(
        shadow->default_rotational_inertia().CopyToFullMatrix3(),
        primary.default_rotational_inertia().CopyToFullMatrix3() / 3.0, kTol));
  }

  // Effective mass properties are split equally among the primary and its two
  // shadows. Summing over the plant still counts link6's physical mass once.
  const Matrix3<double> one_third_I_LLo_L =
      primary.default_rotational_inertia().CopyToFullMatrix3() / 3.0;
  for (const Link<double>* copy : {&primary, &shadow1, &shadow2}) {
    const SpatialInertia<double> M_LLo_L =
        copy->CalcSpatialInertiaInBodyFrame(*context);
    EXPECT_NEAR(M_LLo_L.get_mass(), kLink6Mass / 3.0, kTol);
    EXPECT_TRUE(
        CompareMatrices(M_LLo_L.get_com(), primary.default_com(), kTol));
    EXPECT_TRUE(
        CompareMatrices(M_LLo_L.CalcRotationalInertia().CopyToFullMatrix3(),
                        one_third_I_LLo_L, kTol));
  }
  EXPECT_NEAR(plant.CalcTotalMass(*context), 12.0, kTol);

  // As in RuntimeMassChangeReSplitsAndShadowIsReadOnly, changing the primary's
  // mass at runtime re-splits its mass properties. Here the new 12 kg mass and
  // correspondingly scaled rotational inertia are divided among three copies.
  constexpr double kUpdatedLink6Mass = 12.0;
  primary.SetMass(context.get(), kUpdatedLink6Mass);
  for (const Link<double>* copy : {&primary, &shadow1, &shadow2}) {
    const SpatialInertia<double> M_LLo_L =
        copy->CalcSpatialInertiaInBodyFrame(*context);
    EXPECT_NEAR(M_LLo_L.get_mass(), kUpdatedLink6Mass / 3.0, kTol);
    EXPECT_TRUE(
        CompareMatrices(M_LLo_L.get_com(), primary.default_com(), kTol));
    EXPECT_TRUE(CompareMatrices(
        M_LLo_L.CalcRotationalInertia().CopyToFullMatrix3(),
        (kUpdatedLink6Mass / kLink6Mass) * one_third_I_LLo_L, kTol));
  }
  EXPECT_NEAR(plant.CalcTotalMass(*context), 18.0, kTol);
}

/* Loop breaking retargets exactly one end of exactly one joint onto a shadow
link (in general, one per shadow link). Identifies that joint by looking for a
substituted frame, and reports which end was moved. */
struct RetargetedJoint {
  const Joint<double>* joint{};
  bool moved_parent{};  // Which end of the joint got moved to the shadow.
};

RetargetedJoint GetSoleRetargetedJoint(const MultibodyPlant<double>& plant) {
  std::vector<RetargetedJoint> found;
  for (JointIndex index : plant.GetJointIndices()) {
    const Joint<double>& joint = plant.get_joint(index);
    const bool moved_parent =
        &joint.effective_frame_on_parent() != &joint.frame_on_parent();
    const bool moved_child =
        &joint.effective_frame_on_child() != &joint.frame_on_child();
    // A joint has at most one end on any one link, so at most one of its ends
    // can have been moved to a given shadow of that link.
    EXPECT_FALSE(moved_parent && moved_child) << joint.name();
    if (moved_parent || moved_child) found.push_back({&joint, moved_parent});
  }
  EXPECT_EQ(found.size(), 1);
  return found.empty() ? RetargetedJoint{} : found.front();
}

/* Loop breaking cuts the coupler into a primary and a shadow link, which means
one of the two joints attached to the coupler must be re-aimed at the shadow.
The forest reaches the primary through the driver, so it is the coupler end of
the "coupler_rocker" joint that has to move; its mobilizer must then take its
outboard frame from the shadow link rather than from the coupler.

We check that structurally rather than by comparing poses. On this model the
substitution is numerically a no-op: a shadow's link frame coincides with its
primary's, and neither the coupler nor its shadow is fused into a welded
composite, so the substituted frame's pose in its body frame is identical to
the user frame's. Only the structure -- which link the mobilizer's frame is
fixed to -- distinguishes a retargeted joint from one that was left pointing at
the primary. (In a debug build BodyNodeImpl also asserts this invariant,
frame_M.body() == body_B, but we want a check that holds in every build.) */
GTEST_TEST(ClosedTopologyTest, LoopJointIsRetargetedToTheShadowLink) {
  FourBar four_bar;
  MultibodyPlant<double>& plant = four_bar.plant();
  plant.Finalize();
  const Link<double>& coupler = plant.GetBodyByName("coupler");
  const Link<double>& rocker = plant.GetBodyByName("rocker");
  const Link<double>& shadow = GetSoleShadowLink(plant);

  // It is the coupler_rocker joint's parent (coupler) end that got moved.
  const RetargetedJoint retargeted = GetSoleRetargetedJoint(plant);
  ASSERT_NE(retargeted.joint, nullptr);
  const Joint<double>& loop_joint = *retargeted.joint;
  EXPECT_EQ(loop_joint.name(), "coupler_rocker");
  EXPECT_TRUE(retargeted.moved_parent);

  // The user's view of the joint is unaffected: it still connects the coupler
  // to the rocker, through the frames the parser created on those links.
  EXPECT_EQ(loop_joint.parent_body().index(), coupler.index());
  EXPECT_EQ(loop_joint.child_body().index(), rocker.index());
  EXPECT_EQ(loop_joint.frame_on_parent().body().index(), coupler.index());
  EXPECT_EQ(loop_joint.frame_on_child().body().index(), rocker.index());

  // The substitute frame is an ephemeral ShadowFrame fixed to the shadow link,
  // with the user's frame on the primary as its source (so it has no
  // independent pose of its own; see the next test).
  const auto* shadow_frame = dynamic_cast<const internal::ShadowFrame<double>*>(
      &loop_joint.effective_frame_on_parent());
  ASSERT_NE(shadow_frame, nullptr);
  EXPECT_EQ(shadow_frame->body().index(), shadow.index());
  EXPECT_EQ(&shadow_frame->source_frame(), &loop_joint.frame_on_parent());
  EXPECT_TRUE(shadow_frame->is_ephemeral());

  // The mobilizer modeling this joint moves the shadow link, not the coupler.
  // The shadow is reached through the rocker, so this mobilizer is reversed
  // with respect to its joint: the rocker is inboard and the shadow outboard.
  // Note that the mobilizer's own frames need not be the joint's frames -- a
  // revolute joint inserts an offset frame when it has to align its axis with a
  // mobilizer axis, as the reversal here forces it to -- so what matters is
  // that the outboard frame ends up fixed to the shadow link, which it does
  // because it chains off the ShadowFrame.
  const internal::Mobilizer<double>& mobilizer = loop_joint.GetMobilizerInUse();
  EXPECT_EQ(mobilizer.inboard_body().index(), rocker.index());
  EXPECT_EQ(mobilizer.outboard_body().index(), shadow.index());
  EXPECT_EQ(mobilizer.outboard_frame().body().index(), shadow.index());

  // Nothing else is touched: the remaining joints, including the coupler's
  // other joint (driver_coupler, which stays on the primary), keep the frames
  // and bodies the user gave them. GetSoleRetargetedJoint() already checked
  // that no other joint has a substituted frame; check the bodies too.
  for (const char* name : {"world_driver", "world_rocker", "driver_coupler"}) {
    const Joint<double>& joint = plant.GetJointByName(name);
    EXPECT_EQ(joint.frame_on_parent().body().index(),
              joint.parent_body().index());
    EXPECT_EQ(joint.frame_on_child().body().index(),
              joint.child_body().index());
  }
  EXPECT_EQ(plant.GetJointByName("driver_coupler").child_body().index(),
            coupler.index());
}

/* The substitute frame on the shadow link has no pose parameter of its own; it
delegates to the user's frame on the primary link. So moving the user's frame at
runtime must carry the shadow-side frame along with it -- had we instead
snapshotted the offset during Finalize(), the two would silently disagree and
the joint would no longer connect what the user asked it to. This is a
parameter-level relationship, so we can see it without evaluating kinematics. */
GTEST_TEST(ClosedTopologyTest, MovingThePrimaryFrameMovesTheShadowFrame) {
  constexpr double kTol = 1e-14;
  FourBar four_bar;
  MultibodyPlant<double>& plant = four_bar.plant();
  plant.Finalize();
  auto context = plant.CreateDefaultContext();

  const Link<double>& shadow = GetSoleShadowLink(plant);
  const Joint<double>& loop_joint = plant.GetJointByName("coupler_rocker");
  // The parser materialized this joint's frame on the coupler as a
  // FixedOffsetFrame, whose offset is a Context parameter we can change.
  const auto& Jp = dynamic_cast<const FixedOffsetFrame<double>&>(
      loop_joint.frame_on_parent());

  // Anchor the checks below to frames that really are on the shadow link;
  // otherwise a missing substitution would leave us comparing the user's frame
  // to itself and everything would agree vacuously.
  const Frame<double>& shadow_frame = loop_joint.effective_frame_on_parent();
  ASSERT_EQ(shadow_frame.body().index(), shadow.index());
  // The mobilizer's outboard frame chains off the shadow frame, so it must
  // track the change too.
  const Frame<double>& frame_M =
      loop_joint.GetMobilizerInUse().outboard_frame();
  ASSERT_EQ(frame_M.body().index(), shadow.index());

  // The shadow-side frame starts out coincident with the user's frame. Each
  // pose is measured in its own link frame, and those two link frames coincide,
  // so coincident frames have equal poses.
  const math::RigidTransformd X_CJp = Jp.CalcPoseInBodyFrame(*context);
  EXPECT_TRUE(CompareMatrices(
      shadow_frame.CalcPoseInBodyFrame(*context).GetAsMatrix34(),
      X_CJp.GetAsMatrix34(), kTol));
  EXPECT_TRUE(
      CompareMatrices(frame_M.CalcPoseInBodyFrame(*context).translation(),
                      X_CJp.translation(), kTol));

  // Move the user's frame on the primary coupler to a new offset.
  const math::RigidTransformd X_PJp_new(math::RollPitchYawd(0.1, -0.2, 0.3),
                                        Vector3<double>(3.5, 0.25, -0.75));
  Jp.SetPoseInParentFrame(context.get(), X_PJp_new);
  const math::RigidTransformd X_CJp_new = Jp.CalcPoseInBodyFrame(*context);
  ASSERT_FALSE(
      CompareMatrices(X_CJp_new.GetAsMatrix34(), X_CJp.GetAsMatrix34(), kTol))
      << "the test moved the frame nowhere";

  // With no re-finalization, both the shadow frame and the mobilizer frame
  // chained off it report the new offset.
  EXPECT_TRUE(CompareMatrices(
      shadow_frame.CalcPoseInBodyFrame(*context).GetAsMatrix34(),
      X_CJp_new.GetAsMatrix34(), kTol));
  EXPECT_TRUE(
      CompareMatrices(frame_M.CalcPoseInBodyFrame(*context).translation(),
                      X_CJp_new.translation(), kTol));
}

/* Scalar conversion clones the mobilizers and the shadow-side frames rather
than rebuilding them (it does not re-run the joint-modeling step), so the
converted plant must come out modeling the broken loop the same way, with the
delegation intact. */
GTEST_TEST(ClosedTopologyTest, RetargetingSurvivesScalarConversion) {
  constexpr double kTol = 1e-14;
  FourBar four_bar;
  MultibodyPlant<double>& plant = four_bar.plant();
  plant.Finalize();
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
      systems::System<double>::ToAutoDiffXd(plant);

  const Link<AutoDiffXd>& shadow_ad = plant_ad->GetBodyByName("coupler$1");
  EXPECT_TRUE(shadow_ad.is_ephemeral());

  const Joint<AutoDiffXd>& loop_joint_ad =
      plant_ad->GetJointByName("coupler_rocker");
  const auto* shadow_frame_ad =
      dynamic_cast<const internal::ShadowFrame<AutoDiffXd>*>(
          &loop_joint_ad.effective_frame_on_parent());
  ASSERT_NE(shadow_frame_ad, nullptr);
  EXPECT_EQ(shadow_frame_ad->body().index(), shadow_ad.index());
  // The clone's source frame is the clone's own frame_on_parent(), not an
  // alias back into the double plant.
  EXPECT_EQ(&shadow_frame_ad->source_frame(), &loop_joint_ad.frame_on_parent());
  EXPECT_EQ(&loop_joint_ad.effective_frame_on_child(),
            &loop_joint_ad.frame_on_child());

  const internal::Mobilizer<AutoDiffXd>& mobilizer_ad =
      loop_joint_ad.GetMobilizerInUse();
  EXPECT_EQ(mobilizer_ad.outboard_body().index(), shadow_ad.index());
  EXPECT_EQ(mobilizer_ad.outboard_frame().body().index(), shadow_ad.index());

  // The cloned frame still delegates rather than holding a copy of the offset.
  auto context_ad = plant_ad->CreateDefaultContext();
  const auto& Jp_ad = dynamic_cast<const FixedOffsetFrame<AutoDiffXd>&>(
      loop_joint_ad.frame_on_parent());
  Jp_ad.SetPoseInParentFrame(
      context_ad.get(),
      math::RigidTransform<AutoDiffXd>(Vector3<AutoDiffXd>(3.5, 0.25, -0.75)));
  EXPECT_TRUE(CompareMatrices(
      math::ExtractValue(
          shadow_frame_ad->CalcPoseInBodyFrame(*context_ad).GetAsMatrix34()),
      math::ExtractValue(
          Jp_ad.CalcPoseInBodyFrame(*context_ad).GetAsMatrix34()),
      kTol));
}

/* Returns the sole ephemeral weld constraint spec in `plant`, i.e. the one
Finalize() added to close the loop. */
const internal::WeldConstraintSpec& GetSoleLoopConstraintSpec(
    const MultibodyPlant<double>& plant) {
  std::vector<const internal::WeldConstraintSpec*> found;
  for (const auto& [id, spec] : plant.get_weld_constraint_specs()) {
    EXPECT_EQ(spec.id, id);
    if (spec.is_ephemeral) found.push_back(&spec);
  }
  EXPECT_EQ(found.size(), 1);
  static const internal::WeldConstraintSpec kEmpty;
  return found.empty() ? kEmpty : *found.front();
}

/* Breaking the loop leaves the coupler and its shadow as two disconnected
copies of the same link; a weld constraint between them is what actually closes
the loop. The user didn't ask for that constraint, so it is marked ephemeral,
and because both copies share a link frame it welds the two body frames
directly (no offsets). */
GTEST_TEST(ClosedTopologyTest, LoopIsClosedByAnEphemeralWeldConstraint) {
  // Nothing exists before Finalize(): the loop constraints are a product of
  // forest building, and the user added no constraints of their own.
  FourBar four_bar;
  MultibodyPlant<double>& plant = four_bar.plant();
  EXPECT_EQ(plant.num_loop_constraints(), 0);
  EXPECT_EQ(plant.num_constraints(), 0);

  plant.Finalize();

  // The single loop of the four-bar costs a single weld constraint, which is
  // reported by the plant-wide count like any other.
  EXPECT_EQ(plant.num_loop_constraints(), 1);
  EXPECT_EQ(plant.num_weld_constraints(), 1);
  EXPECT_EQ(plant.num_constraints(), 1);
  EXPECT_EQ(plant.GetConstraintIds().size(), 1);

  const internal::WeldConstraintSpec& spec = GetSoleLoopConstraintSpec(plant);
  EXPECT_EQ(plant.GetConstraintIds().at(0), spec.id);

  // The primary link is body A (the parent), which fixes the sign convention
  // for the constraint's multipliers.
  const Link<double>& coupler = plant.GetBodyByName("coupler");
  const Link<double>& shadow = GetSoleShadowLink(plant);
  EXPECT_EQ(spec.body_A, coupler.index());
  EXPECT_EQ(spec.body_B, shadow.index());

  // The welded frames are the two body frames themselves: a shadow's link
  // frame coincides with its primary's by construction, so both offsets are
  // the identity.
  EXPECT_TRUE(spec.X_AP.IsExactlyIdentity());
  EXPECT_TRUE(spec.X_BQ.IsExactlyIdentity());
}

/* An ephemeral loop constraint is an ordinary weld constraint in every respect
except that the user didn't add it, so it has to coexist with -- and be
distinguishable from -- the user's own weld constraints. */
GTEST_TEST(ClosedTopologyTest, LoopConstraintCoexistsWithUserConstraints) {
  FourBar four_bar;
  MultibodyPlant<double>& plant = four_bar.plant();
  const MultibodyConstraintId user_id = plant.AddWeldConstraint(
      plant.GetBodyByName("driver"), math::RigidTransformd(),
      plant.GetBodyByName("rocker"), math::RigidTransformd());
  EXPECT_EQ(plant.num_weld_constraints(), 1);
  plant.Finalize();

  // The user's weld plus the loop-closing weld.
  EXPECT_EQ(plant.num_weld_constraints(), 2);
  EXPECT_EQ(plant.num_constraints(), 2);
  EXPECT_EQ(plant.num_loop_constraints(), 1);

  // The user's constraint is untouched and is not confused for an ephemeral
  // one; GetSoleLoopConstraintSpec() checks the converse, that exactly one of
  // the two is ephemeral.
  EXPECT_FALSE(plant.get_weld_constraint_specs(user_id).is_ephemeral);
  EXPECT_NE(GetSoleLoopConstraintSpec(plant).id, user_id);
}

/* Loop constraints are created by Finalize(), which a scalar-converted plant
does not re-run: it copies the constraint specs and finalizes only the plant
itself. So the converted plant must come out with the same constraints, under
the same ids (which the Context's active status map is keyed on). */
GTEST_TEST(ClosedTopologyTest, LoopConstraintSurvivesScalarConversion) {
  FourBar four_bar;
  MultibodyPlant<double>& plant = four_bar.plant();
  plant.Finalize();
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
      systems::System<double>::ToAutoDiffXd(plant);

  // In particular, no second copy of the loop weld was added on the way.
  EXPECT_EQ(plant_ad->num_weld_constraints(), 1);
  EXPECT_EQ(plant_ad->num_constraints(), 1);

  // The graph (and hence the loop constraint count, which is sourced from it)
  // is cloned along with the tree.
  EXPECT_EQ(plant_ad->num_loop_constraints(), 1);

  const internal::WeldConstraintSpec& spec = GetSoleLoopConstraintSpec(plant);
  const auto& specs_ad = plant_ad->get_weld_constraint_specs();
  ASSERT_TRUE(specs_ad.contains(spec.id));
  const internal::WeldConstraintSpec& spec_ad = specs_ad.at(spec.id);
  EXPECT_TRUE(spec_ad.is_ephemeral);
  EXPECT_EQ(spec_ad.body_A, spec.body_A);
  EXPECT_EQ(spec_ad.body_B, spec.body_B);
}

/* A loop constraint has to reach the solver like any other constraint. That
requires it to be in place before Finalize() declares the constraint
parameters, since the solver looks up each constraint's active status by id as
it builds its model. It is a live constraint in that model, so its status is
also settable -- deactivating it re-opens the loop, which is useful for
debugging assembly problems, at the cost of leaving the shadow link dangling
(with its share of the split mass) on the loop joint. */
GTEST_TEST(ClosedTopologyTest, LoopConstraintReachesTheDiscreteSolver) {
  FourBar four_bar(0.01 /* discrete, SAP */);
  MultibodyPlant<double>& plant = four_bar.plant();
  plant.Finalize();
  EXPECT_EQ(plant.get_discrete_contact_solver(), DiscreteContactSolver::kSap);
  auto context = plant.CreateDefaultContext();

  const MultibodyConstraintId id = GetSoleLoopConstraintSpec(plant).id;

  // Like any constraint it starts out active, and the discrete update -- which
  // is where SAP consults that status and assembles the constraint -- runs.
  EXPECT_TRUE(plant.GetConstraintActiveStatus(*context, id));
  auto state = plant.AllocateDiscreteVariables();
  EXPECT_NO_THROW(
      plant.CalcForcedDiscreteVariableUpdate(*context, state.get()));

  // Deactivating it is allowed, and SAP then skips it.
  plant.SetConstraintActiveStatus(context.get(), id, false);
  EXPECT_FALSE(plant.GetConstraintActiveStatus(*context, id));
  EXPECT_NO_THROW(
      plant.CalcForcedDiscreteVariableUpdate(*context, state.get()));
}

/* Continuous time (with anything but the CENIC integrator, which computes its
dynamics by a different route) does not support constraints -- and a looped
model has them whether or not the user asked for any. So the complaint has to
say where they came from; otherwise it tells the user to go remove constraints
they never added. */
GTEST_TEST(ClosedTopologyTest, ContinuousDynamicsSaysWhereLoopConstraintsCame) {
  FourBar four_bar;  // Continuous.
  MultibodyPlant<double>& plant = four_bar.plant();
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  ASSERT_EQ(plant.num_constraints(), plant.num_loop_constraints());

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.EvalTimeDerivatives(*context),
      ".*does not support constraints, but this model has 1 constraint, which "
      "Finalize\\(\\) added in order to close the kinematic loops.*");

  // With constraints from both sources, the message accounts for both.
  FourBar mixed_four_bar;
  MultibodyPlant<double>& mixed_plant = mixed_four_bar.plant();
  mixed_plant.AddWeldConstraint(
      mixed_plant.GetBodyByName("driver"), math::RigidTransformd(),
      mixed_plant.GetBodyByName("rocker"), math::RigidTransformd());
  mixed_plant.Finalize();
  auto mixed_context = mixed_plant.CreateDefaultContext();

  DRAKE_EXPECT_THROWS_MESSAGE(
      mixed_plant.EvalTimeDerivatives(*mixed_context),
      ".*this model has 2 constraints, 1 of which Finalize\\(\\) added in "
      "order to close the kinematic loops.*");
}

/* A closed loop need not be anchored to World at all -- think of a linkage
floating in space. The forest then has to choose one of the loop's own links to
serve as a base body and give it an ephemeral floating joint, even though that
link is already named as a child by one of the loop's joints. This case
previously caused a bug; here we verify it works properly.

The model is the simplest possible loop: three links in a cycle, nothing welded
to World. */
GTEST_TEST(ClosedTopologyTest, UnanchoredLoopGetsAFloatingBaseBody) {
  MultibodyPlant<double> plant(0.0 /* continuous */);
  plant.SetEnableLoopTopology(true);
  const Link<double>& link_a = plant.AddRigidBody(
      "a", SpatialInertia<double>::SolidCubeWithMass(1.0, 0.1));
  const Link<double>& link_b = plant.AddRigidBody(
      "b", SpatialInertia<double>::SolidCubeWithMass(1.0, 0.1));
  const Link<double>& link_c = plant.AddRigidBody(
      "c", SpatialInertia<double>::SolidCubeWithMass(1.0, 0.1));
  const Vector3<double> axis = Vector3<double>::UnitZ();
  plant.AddJoint<RevoluteJoint>("a_b", link_a, {}, link_b, {}, axis);
  plant.AddJoint<RevoluteJoint>("b_c", link_b, {}, link_c, {}, axis);
  plant.AddJoint<RevoluteJoint>("c_a", link_c, {}, link_a, {}, axis);

  EXPECT_NO_THROW(plant.Finalize());

  // The loop was broken the usual way, and the assembly floats: 6 dofs for the
  // floating base plus one per revolute joint.
  EXPECT_EQ(plant.num_loop_constraints(), 1);
  // Which of the three links the forest chooses to split is not what this test
  // is about, so just check that one of them was.
  const std::string shadow_name = GetSoleShadowLink(plant).name();
  EXPECT_TRUE(shadow_name == "a$1" || shadow_name == "b$1" ||
              shadow_name == "c$1");
  EXPECT_EQ(plant.num_velocities(), 6 + 3);

  // Exactly one of the loop's links is the floating base body, and it is the
  // child of a user joint (which is what used to trigger the bug).
  std::vector<BodyIndex> floating;
  for (const Link<double>* link : {&link_a, &link_b, &link_c}) {
    if (link->is_floating_base_body()) floating.push_back(link->index());
  }
  ASSERT_EQ(floating.size(), 1);
  const Link<double>& base = plant.get_body(floating.at(0));
  // It is also the child link of one of the loop's user joints -- precisely the
  // situation that used to trip the DRAKE_DEMAND.
  int user_joints_naming_base_as_child = 0;
  for (const char* name : {"a_b", "b_c", "c_a"}) {
    if (plant.GetJointByName(name).child_body().index() == base.index()) {
      ++user_joints_naming_base_as_child;
    }
  }
  EXPECT_EQ(user_joints_naming_base_as_child, 1);

  // The default pose of that body must route through its ephemeral floating
  // joint -- that routing is exactly what the repaired loop sets up.
  const math::RigidTransformd X_WB(math::RollPitchYawd(0.1, 0.2, 0.3),
                                   Vector3<double>(1.0, 2.0, 3.0));
  plant.SetDefaultFloatingBaseBodyPose(base, X_WB);
  EXPECT_TRUE(CompareMatrices(
      plant.GetDefaultFloatingBaseBodyPose(base).GetAsMatrix34(),
      X_WB.GetAsMatrix34(), 1e-14));
  auto context = plant.CreateDefaultContext();
  EXPECT_TRUE(
      CompareMatrices(plant.EvalBodyPoseInWorld(*context, base).GetAsMatrix34(),
                      X_WB.GetAsMatrix34(), 1e-14));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
