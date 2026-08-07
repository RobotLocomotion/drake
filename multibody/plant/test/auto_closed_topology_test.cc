/* Tests for automatic handling of closed-topology (looped) systems, enabled via
MultibodyPlant::SetEnableLoopTopology(). When enabled, Finalize() breaks each
kinematic loop using the shadow links and loop constraints produced by the
underlying LinkJointGraph/SpanningForest. This file tests that adding shadow
links splits mass properties properly among the primary link and all of its
shadows.

Most tests use a planar four-bar linkage (three moving links -- driver, coupler,
rocker -- plus World, connected by four revolute joints) which forms a single
kinematic loop. See examples/multibody/four_bar for runnable examples of similar
mechanisms. The final test uses a double loop that gives one primary link two
shadows. */

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/internal_geometry_names.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

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

/* Builds and finalizes the four-bar loop plant, with automatic handling of
kinematic loops enabled. */
std::unique_ptr<MultibodyPlant<double>> MakeFourBarPlant() {
  auto plant = std::make_unique<MultibodyPlant<double>>(0.0 /* continuous */);
  plant->SetEnableLoopTopology(true);
  Parser(plant.get()).AddModelsFromString(kFourBarLoopSdf, "sdf");
  plant->Finalize();
  return plant;
}

/* Builds and finalizes the four-bar loop plant, with automatic handling
of kinematic loops enabled. Registers the plant as a geometry source for
`scene_graph` and gives the driver, coupler, and rocker one arbitrary visual and
one arbitrary collision geometry apiece. (This is to test that shadow links
don't interfere with geometry access so the particular geometry doesn't
matter.) */
std::unique_ptr<MultibodyPlant<double>> MakeFourBarPlantWithGeometry(
    geometry::SceneGraph<double>* scene_graph) {
  auto plant = std::make_unique<MultibodyPlant<double>>(0.0 /* continuous */);
  plant->SetEnableLoopTopology(true);
  plant->RegisterAsSourceForSceneGraph(scene_graph);
  Parser(plant.get()).AddModelsFromString(kFourBarLoopSdf, "sdf");
  for (const std::string name : {"driver", "coupler", "rocker"}) {
    const Link<double>& link = plant->GetBodyByName(name);
    plant->RegisterVisualGeometry(link, math::RigidTransformd(),
                                  geometry::Sphere(0.1), name + "_visual");
    plant->RegisterCollisionGeometry(link, math::RigidTransformd(),
                                     geometry::Sphere(0.1), name + "_collision",
                                     CoulombFriction<double>(1.0, 1.0));
  }
  plant->Finalize();
  return plant;
}

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
  std::unique_ptr<MultibodyPlant<double>> plant = MakeFourBarPlant();

  // World + driver + coupler + rocker + the coupler's shadow.
  EXPECT_EQ(plant->num_bodies(), 5);

  // There is exactly one shadow, and it is the coupler's.
  const Link<double>& shadow = GetSoleShadowLink(*plant);
  EXPECT_EQ(shadow.name(), "coupler$1");
  EXPECT_TRUE(shadow.is_ephemeral());

  // Every user-defined link remains non-ephemeral.
  for (const char* name : {"driver", "coupler", "rocker"}) {
    EXPECT_FALSE(plant->GetBodyByName(name).is_ephemeral());
  }
}

/* Objects auto-created during Finalize() have to be marked as ephemeral, per
the MultibodyElement::is_ephemeral() contract. That includes link frames which
are co-created implicitly whenever a link is auto-created (in contrast to an
explicit user call to "add frame"). */
GTEST_TEST(ClosedTopologyTest, ShadowLinkFramesAreEphemeral) {
  std::unique_ptr<MultibodyPlant<double>> plant = MakeFourBarPlant();
  const Link<double>& shadow = GetSoleShadowLink(*plant);
  EXPECT_TRUE(shadow.body_frame().is_ephemeral());

  // Since shadow links are auto-created (not user-created), every frame
  // fixed to a shadow link is auto-created during Finalize(). For now, that
  // is just the shadow's own link frame; when loop joints get retargeted
  // onto the shadow they will bring their mobilizer frames along too.
  int num_shadow_frames = 0;
  for (FrameIndex index(0); index < plant->num_frames(); ++index) {
    const Frame<double>& frame = plant->get_frame(index);
    if (frame.body().index() != shadow.index()) continue;
    ++num_shadow_frames;
    EXPECT_TRUE(frame.is_ephemeral()) << frame.name();
  }
  EXPECT_GE(num_shadow_frames, 1);

  // The user's links keep non-ephemeral link frames.
  EXPECT_FALSE(plant->world_body().body_frame().is_ephemeral());
  for (const char* name : {"driver", "coupler", "rocker"}) {
    EXPECT_FALSE(plant->GetBodyByName(name).body_frame().is_ephemeral());
  }

  // Scalar conversion must carry the flags over; unlike the pre-finalize path,
  // it creates the shadow's link frame by cloning rather than by adding a link.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
      systems::System<double>::ToAutoDiffXd(*plant);
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
  std::unique_ptr<MultibodyPlant<double>> plant = MakeFourBarPlant();
  auto context = plant->CreateDefaultContext();

  const Link<double>& coupler = plant->GetBodyByName("coupler");
  const Link<double>& shadow = plant->GetBodyByName("coupler$1");

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
  EXPECT_NEAR(M_LLo_L(plant->GetBodyByName("driver")).get_mass(), 1.0, kTol);
  EXPECT_NEAR(M_LLo_L(plant->GetBodyByName("rocker")).get_mass(), 2.0, kTol);
}

/* A shadow link has no spatial-inertia parameter of its own; its effective mass
properties are sourced from the FrameBodyPoseCache (mirroring its primary's
share of the split). Consequently the per-body context accessors report the
split value for both the primary and the shadow, and system-wide mass
aggregates (which loop over all links, shadows included) count the physical mass
exactly once rather than double-counting the shadow. */
GTEST_TEST(ClosedTopologyTest, ShadowHasNoIndependentInertiaParameter) {
  constexpr double kTol = 1e-14;
  std::unique_ptr<MultibodyPlant<double>> plant = MakeFourBarPlant();
  auto context = plant->CreateDefaultContext();

  const Link<double>& coupler = plant->GetBodyByName("coupler");
  const Link<double>& shadow = plant->GetBodyByName("coupler$1");

  // The context accessor reports each copy's split share (0.5 kg), not the
  // coupler's full declared 1 kg. (default_mass() still reports the declared
  // 1 kg for the primary; see ShadowMassIsSplitEvenly.)
  EXPECT_NEAR(coupler.get_mass(*context), 0.5, kTol);
  EXPECT_NEAR(shadow.get_mass(*context), 0.5, kTol);

  // Ensure the system's total mass properly accounts for the split-link
  // coupler (0.5 * primary_link_mass + 0.5 * shadow_link_mass = 1 kg),
  // so total mass is: driver 1 kg + rocker 2 kg + coupler 1 kg = 4 kg.
  EXPECT_NEAR(plant->CalcTotalMass(*context), 4.0, kTol);
}

/* Because the split is recomputed in the FrameBodyPoseCache from the primary's
parameter, changing the primary's mass at runtime re-splits across the shadow
automatically -- the primary is the single source of truth. Setting mass on the
shadow directly is disallowed. */
GTEST_TEST(ClosedTopologyTest, RuntimeMassChangeReSplitsAndShadowIsReadOnly) {
  constexpr double kTol = 1e-14;
  std::unique_ptr<MultibodyPlant<double>> plant = MakeFourBarPlant();
  auto context = plant->CreateDefaultContext();

  const Link<double>& coupler = plant->GetBodyByName("coupler");
  const Link<double>& shadow = plant->GetBodyByName("coupler$1");

  // Set the coupler's (physical) mass to 3 kg; the split follows to 1.5 kg on
  // each of the coupler and its shadow, and the total tracks accordingly
  // (driver 1 + rocker 2 + coupler 3 = 6 kg).
  coupler.SetMass(context.get(), 3.0);
  EXPECT_NEAR(coupler.get_mass(*context), 1.5, kTol);
  EXPECT_NEAR(shadow.get_mass(*context), 1.5, kTol);
  EXPECT_NEAR(plant->CalcTotalMass(*context), 6.0, kTol);

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
  geometry::SceneGraph<double> scene_graph;
  std::unique_ptr<MultibodyPlant<double>> plant =
      MakeFourBarPlantWithGeometry(&scene_graph);

  const Link<double>& coupler = plant->GetBodyByName("coupler");
  const Link<double>& shadow = plant->GetBodyByName("coupler$1");

  // The primary link keeps the geometry registered on it.
  EXPECT_EQ(plant->GetVisualGeometriesForBody(coupler).size(), 1);
  EXPECT_EQ(plant->GetCollisionGeometriesForBody(coupler).size(), 1);

  // The shadow link has an empty entry. Without that empty entry,
  // these two lookups would read past the end of the per-body arrays.
  EXPECT_TRUE(plant->GetVisualGeometriesForBody(shadow).empty());
  EXPECT_TRUE(plant->GetCollisionGeometriesForBody(shadow).empty());

  // Breaking the loop doesn't invent geometry: driver, coupler, rocker have
  // one visual and one collision geometry each.
  EXPECT_EQ(plant->num_visual_geometries(), 3);
  EXPECT_EQ(plant->num_collision_geometries(), 3);

  // A shadow gets no SceneGraph frame of its own; reporting a pose for it would
  // publish a spurious duplicate of its primary. That's safe precisely because
  // the frame id table is map-keyed and documented to tolerate bodies with no
  // frame -- unlike the dense per-body arrays checked above.
  EXPECT_TRUE(plant->GetBodyFrameIdIfExists(coupler.index()).has_value());
  EXPECT_FALSE(plant->GetBodyFrameIdIfExists(shadow.index()).has_value());

  // The per-body arrays are copied wholesale during scalar conversion, so the
  // converted plant must agree with the tree it carries as well.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
      systems::System<double>::ToAutoDiffXd(*plant);
  EXPECT_EQ(plant_ad->num_bodies(), plant->num_bodies());
  EXPECT_TRUE(
      plant_ad->GetVisualGeometriesForBody(plant_ad->GetBodyByName("coupler$1"))
          .empty());
  EXPECT_TRUE(
      plant_ad
          ->GetCollisionGeometriesForBody(plant_ad->GetBodyByName("coupler$1"))
          .empty());
}

/* Several post-finalize consumers (visualization helpers in particular) walk
every BodyIndex in [0, num_bodies()) and ask the plant for that body's
geometry. Before shadow links were given per-body geometry entries, those walks
read past the end of the arrays once a loop had been broken. Note that the
out-of-range read is undefined behavior rather than an exception, so this test
earns its keep in debug builds (where the density assertions in the accessors
fire) and under the memory sanitizers. */
GTEST_TEST(ClosedTopologyTest, WalkingEveryBodyForGeometryStaysInRange) {
  geometry::SceneGraph<double> scene_graph;
  std::unique_ptr<MultibodyPlant<double>> plant =
      MakeFourBarPlantWithGeometry(&scene_graph);

  // Summing per-body counts over all bodies (shadows included) must reproduce
  // the plant-wide totals -- i.e. every body has an entry and no geometry is
  // counted twice.
  int num_visual = 0;
  int num_collision = 0;
  for (BodyIndex i(0); i < plant->num_bodies(); ++i) {
    const Link<double>& link = plant->get_body(i);
    num_visual += ssize(plant->GetVisualGeometriesForBody(link));
    num_collision += ssize(plant->GetCollisionGeometriesForBody(link));
  }
  EXPECT_EQ(num_visual, plant->num_visual_geometries());
  EXPECT_EQ(num_collision, plant->num_collision_geometries());

  // GeometryNames performs exactly that walk; it is the path taken by contact
  // visualization (see also ContactResultsToLcmSystem).
  internal::GeometryNames geometry_names;
  EXPECT_NO_THROW(geometry_names.ResetBasic(*plant));
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
  // assumptions about the topology produced for this graph. These are fatal
  // checks because the remaining test is meaningful only if link6 was split
  // twice in the same way as in SpanningForest.DoubleLoop.
  const LinkIndex primary_index(6);
  const LinkIndex shadow1_index(8);
  const LinkIndex shadow2_index(9);
  const internal::LinkJointGraph& graph = plant.graph();
  ASSERT_EQ(graph.num_user_links(), 8);  // Includes World.
  ASSERT_EQ(graph.num_links(), 10);
  ASSERT_EQ(graph.num_user_joints(), 8);
  ASSERT_EQ(graph.num_joints(), 9);
  ASSERT_EQ(ssize(graph.loop_constraints()), 2);
  const internal::LinkJointGraph::Link& graph_primary =
      graph.link_by_index(primary_index);
  const internal::LinkJointGraph::Link& graph_shadow1 =
      graph.link_by_index(shadow1_index);
  const internal::LinkJointGraph::Link& graph_shadow2 =
      graph.link_by_index(shadow2_index);
  ASSERT_EQ(graph_primary.num_shadows(), 2);
  ASSERT_EQ(graph_primary.shadow_links(),
            (std::vector{shadow1_index, shadow2_index}));
  ASSERT_TRUE(graph_shadow1.is_shadow());
  ASSERT_TRUE(graph_shadow2.is_shadow());
  ASSERT_EQ(graph_shadow1.primary_link(), primary_index);
  ASSERT_EQ(graph_shadow2.primary_link(), primary_index);
  ASSERT_EQ(graph.loop_constraints()[0].primary_link(), primary_index);
  ASSERT_EQ(graph.loop_constraints()[0].shadow_link(), shadow1_index);
  ASSERT_EQ(graph.loop_constraints()[1].primary_link(), primary_index);
  ASSERT_EQ(graph.loop_constraints()[1].shadow_link(), shadow2_index);
  ASSERT_EQ(graph.forest().num_mobods(), 10);
  ASSERT_EQ(graph.forest().num_trees(), 1);
  ASSERT_EQ(graph.forest().trees(internal::TreeIndex(0)).num_mobods(), 9);
  ASSERT_EQ(graph.forest().trees(internal::TreeIndex(0)).height(), 4);

  // With those topology assumptions established, check MultibodyPlant's
  // representation of the shadows and their mass properties.
  auto context = plant.CreateDefaultContext();

  // World + seven user links + two shadows; the ninth joint is the ephemeral
  // floating joint that attaches link1 to World.
  EXPECT_EQ(plant.num_bodies(), 10);
  EXPECT_EQ(plant.num_joints(), 9);

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

}  // namespace
}  // namespace multibody
}  // namespace drake
