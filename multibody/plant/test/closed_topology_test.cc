/* Tests for automatic modeling of closed-topology (looped) systems, enabled via
MultibodyPlant::SetAllowLoopTopology(). When enabled, Finalize() breaks each
kinematic loop using the shadow links and loop constraints produced by the
underlying LinkJointGraph/SpanningForest. This file focuses on the ephemeral
shadow links and their (evenly-split) mass properties.

The test model is a planar four-bar linkage (three moving links -- driver,
coupler, rocker -- plus World, connected by four revolute joints) which forms a
single kinematic loop. See examples/multibody/four_bar/dev/four_bar_loop.sdf. */

#include <limits>
#include <memory>
#include <string>
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
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

/* A planar four-bar linkage described with no spanning tree specified, i.e. as
a raw loop. Taken from examples/multibody/four_bar/dev/four_bar_loop.sdf.
There are three moving links (driver, coupler, rocker) plus World and four
revolute joints; all revolute axes point in +z (out of the page). Below, the
links are drawn unassembled and lined up with the World axes; "*" marks the
connection points and the attached frames are named.

                                                        * Rc
                                                        |
                                                        |
                             coupler C                  |
       Co *------------------------------------------------------* Cr
                              4.8m 1kg                  |
          * Dc                                          |
          |                                    rocker R | 2m
          |                                             | 2kg
      1m  | driver D                                    |
      1kg |                                             |      Wy
          |                                             |      |
          * Do                                       Ro *      |
                                                               +----- Wx
          *====================== Wo ===================*     /
         Wd         2m          world2 W      2m        Wr    Wz
                                 ###
                                World

In parent-child order the joints connect Wd-Do, Wr-Ro, Dc-Co, Cr-Rc. link
mass centers are at their midpoints; inertias are those of thin rods
(ML²/12). link masses: driver 1 kg, coupler 1 kg, rocker 2 kg.

One change from the source SDF lets this parse standalone: rather than
attaching the World-fixed frames Wd/Wr directly to "world" (which
libsdformat's frame graph won't resolve for a bare model), we add a "world2"
link welded to World and attach the frames to "world2" instead. */
constexpr char kFourBarLoopSdf[] = R"""(
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="four_bar_loop">
    <link name="world2">
      <!-- Explicit zero mass properties so this parse-helper link (welded to
      World) contributes nothing to system-wide mass aggregates in the tests.
      Without an <inertial> block sdformat would default it to 1 kg. -->
      <inertial>
        <mass>0</mass>
        <inertia>
          <ixx>0</ixx> <iyy>0</iyy> <izz>0</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </inertia>
      </inertial>
    </link>
    <joint name="world2_weld" type="fixed">
      <parent>world</parent>
      <child>world2</child>
    </joint>
    <frame name="Wd" attached_to="world2">
      <pose relative_to="world2">-2 0 0 0 0 0</pose>
    </frame>
    <frame name="Wr" attached_to="world2">
      <pose relative_to="world2">2 0 0 0 0 0</pose>
    </frame>
    <link name="driver">
      <inertial>
        <pose>0 0.5 0 0 0 0</pose>
        <mass>1</mass>
        <inertia>
          <ixx>0.0833333333333333</ixx>
          <iyy>0</iyy>
          <izz>0.0833333333333333</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </inertia>
      </inertial>
    </link>
    <frame name="Dc" attached_to="driver">
      <pose relative_to="driver">0 1 0 0 0 0</pose>
    </frame>
    <link name="rocker">
      <inertial>
        <pose>0 1 0 0 0 0</pose>
        <mass>2</mass>
        <inertia>
          <ixx>0.6666666666666667</ixx>
          <iyy>0</iyy>
          <izz>0.6666666666666667</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </inertia>
      </inertial>
    </link>
    <frame name="Rc" attached_to="rocker">
      <pose relative_to="rocker">0 2 0 0 0 0</pose>
    </frame>
    <link name="coupler">
      <inertial>
        <pose>2.4 0 0 0 0 0</pose>
        <mass>1</mass>
        <inertia>
          <ixx>0</ixx>
          <iyy>1.92</iyy>
          <izz>1.92</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </inertia>
      </inertial>
    </link>
    <frame name="Cr" attached_to="coupler">
      <pose relative_to="coupler">4.8 0 0 0 0 0</pose>
    </frame>
    <joint name="world_driver" type="revolute">
      <parent>Wd</parent>
      <child>driver</child>
      <axis><xyz expressed_in="__model__">0 0 1</xyz></axis>
    </joint>
    <joint name="world_rocker" type="revolute">
      <parent>Wr</parent>
      <child>rocker</child>
      <axis><xyz expressed_in="__model__">0 0 1</xyz></axis>
    </joint>
    <joint name="driver_coupler" type="revolute">
      <parent>Dc</parent>
      <child>coupler</child>
      <axis><xyz expressed_in="__model__">0 0 1</xyz></axis>
    </joint>
    <joint name="coupler_rocker" type="revolute">
      <parent>Cr</parent>
      <child>Rc</child>
      <axis><xyz expressed_in="__model__">0 0 1</xyz></axis>
    </joint>
  </model>
</sdf>
)""";

/* Builds the four-bar loop plant with automatic loop modeling enabled and
finalizes it. */
std::unique_ptr<MultibodyPlant<double>> MakeFourBarPlant() {
  auto plant = std::make_unique<MultibodyPlant<double>>(0.0 /* continuous */);
  plant->SetAllowLoopTopology(true);
  Parser(plant.get()).AddModelsFromString(kFourBarLoopSdf, "sdf");
  plant->Finalize();
  return plant;
}

/* As above, but registers the plant as a geometry source for `scene_graph` and
gives the driver and the coupler (the link we know gets split; see
CouplerIsSplit) one visual and one collision geometry apiece. */
std::unique_ptr<MultibodyPlant<double>> MakeFourBarPlantWithGeometry(
    geometry::SceneGraph<double>* scene_graph) {
  auto plant = std::make_unique<MultibodyPlant<double>>(0.0 /* continuous */);
  plant->SetAllowLoopTopology(true);
  plant->RegisterAsSourceForSceneGraph(scene_graph);
  Parser(plant.get()).AddModelsFromString(kFourBarLoopSdf, "sdf");
  for (const std::string name : {"driver", "coupler"}) {
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

/* Returns the single ephemeral (shadow) link in `plant`. We'll hunt for it here
and then verify below that it meets our expectations (which is that the coupler
was split). */
const Link<double>& GetSoleShadowLink(const MultibodyPlant<double>& plant) {
  std::vector<BodyIndex> shadows;
  for (BodyIndex i(0); i < plant.num_bodies(); ++i) {
    if (plant.get_body(i).is_ephemeral()) shadows.push_back(i);
  }
  EXPECT_EQ(shadows.size(), 1);
  return plant.get_body(shadows.at(0));
}

/* The four-bar loop is modeled by splitting one link into primary and
shadow links. Because the SpanningForest minimizes the maximum branch length, it
breaks the loop by splitting the _middle_ link, the coupler: the primary
(coupler) is reached via the driver and the shadow (coupler$1) via the rocker,
yielding two length-2 branches. We therefore know a priori that primary =
coupler and shadow = coupler$1, and the tests below rely on that so we verify
here. */
GTEST_TEST(ClosedTopologyTest, CouplerIsSplit) {
  std::unique_ptr<MultibodyPlant<double>> plant = MakeFourBarPlant();

  // World + world2 + driver + coupler + rocker + the coupler's shadow.
  EXPECT_EQ(plant->num_bodies(), 6);

  // There is exactly one shadow, and it is the coupler's.
  const Link<double>& shadow = GetSoleShadowLink(*plant);
  EXPECT_EQ(shadow.name(), "coupler$1");
  EXPECT_TRUE(shadow.is_ephemeral());

  // Every user-defined link remains non-ephemeral.
  for (const char* name : {"world2", "driver", "coupler", "rocker"}) {
    EXPECT_FALSE(plant->GetBodyByName(name).is_ephemeral());
  }
}

/* The coupler's mass properties are split evenly with its shadow. We check the
per-link spatial inertia M_LLo_L (about the link's origin Lo, expressed in the
link frame L), obtained by asking the plant for that one link's spatial inertia.
The per-link inertia is the right quantity because welded-together links can be
fused onto a single mobod, in which case one mobod carries the summed inertia of
several links. Here the coupler link and the coupler$1 link each carry half of
the coupler's inertia (and are identical to each other, since their link frames
coincide). The user-facing per-link (default) mass is unchanged for the coupler
(its full declared 1 kg) while the ephemeral shadow's default reflects its half
share; the effective (split) inertia checked here is what drives the dynamics.
The unsplit links (driver and rocker) are unaffected. */
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
  // expressed in its link frame L (M_LLo_L == M_BBo_B, since the link frame is
  // the body frame). This public accessor reads the loop-split value from the
  // FrameBodyPoseCache.
  auto M_LLo_L = [&](const Link<double>& link) {
    return link.CalcSpatialInertiaInBodyFrame(*context);
  };
  const SpatialInertia<double> M_LLo_L_coupler = M_LLo_L(coupler);
  const SpatialInertia<double> M_LLo_L_shadow = M_LLo_L(shadow);

  // The coupler link and its shadow each carry half of the coupler's 1 kg, with
  // otherwise identical spatial inertia (their link frames coincide).
  EXPECT_NEAR(M_LLo_L_coupler.get_mass(), 0.5, kTol);
  EXPECT_NEAR(M_LLo_L_shadow.get_mass(), 0.5, kTol);
  EXPECT_TRUE(CompareMatrices(M_LLo_L_shadow.get_com(),
                              M_LLo_L_coupler.get_com(), kTol));
  EXPECT_TRUE(CompareMatrices(
      M_LLo_L_shadow.CalcRotationalInertia().CopyToFullMatrix3(),
      M_LLo_L_coupler.CalcRotationalInertia().CopyToFullMatrix3(), kTol));

  // Moreover, each is exactly half of the coupler's declared (default)
  // rotational inertia about Lo, expressed in L.
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

  // The aggregate must count the coupler's mass exactly once (0.5 primary +
  // 0.5 shadow = 1 kg), not double-count the shadow. The physical mass is
  // driver 1 + rocker 2 + coupler 1 = 4 kg (the world2 helper link is
  // massless).
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
  // (driver 1 + rocker 2 + coupler 3 = 6 kg; the world2 helper link is
  // massless).
  coupler.SetMass(context.get(), 3.0);
  EXPECT_NEAR(coupler.get_mass(*context), 1.5, kTol);
  EXPECT_NEAR(shadow.get_mass(*context), 1.5, kTol);
  EXPECT_NEAR(plant->CalcTotalMass(*context), 6.0, kTol);

  // The shadow's mass properties are not independently settable.
  DRAKE_EXPECT_THROWS_MESSAGE(shadow.SetMass(context.get(), 1.0),
                              ".*coupler\\$1.*ephemeral shadow link.*");
}

/* A shadow link carries no geometry of its own -- it is an internal modeling
artifact coincident with its primary -- but it must still have an (empty) entry
in the plant's per-body geometry arrays, which are indexed by BodyIndex and so
must stay dense over num_bodies(). Shadow links are created inside
MultibodyTree::Finalize() rather than by MultibodyPlant::AddRigidBody() (which
is what normally extends those arrays), so Finalize() has to extend them. */
GTEST_TEST(ClosedTopologyTest, ShadowLinkHasEmptyGeometryEntries) {
  geometry::SceneGraph<double> scene_graph;
  std::unique_ptr<MultibodyPlant<double>> plant =
      MakeFourBarPlantWithGeometry(&scene_graph);

  const Link<double>& coupler = plant->GetBodyByName("coupler");
  const Link<double>& shadow = plant->GetBodyByName("coupler$1");

  // The primary keeps the geometry registered on it ...
  EXPECT_EQ(plant->GetVisualGeometriesForBody(coupler).size(), 1);
  EXPECT_EQ(plant->GetCollisionGeometriesForBody(coupler).size(), 1);

  // ... and its shadow has an entry, which is empty. Without that entry these
  // two lookups would read past the end of the per-body arrays.
  EXPECT_TRUE(plant->GetVisualGeometriesForBody(shadow).empty());
  EXPECT_TRUE(plant->GetCollisionGeometriesForBody(shadow).empty());

  // Breaking the loop doesn't invent geometry: driver and coupler, one visual
  // and one collision geometry each.
  EXPECT_EQ(plant->num_visual_geometries(), 2);
  EXPECT_EQ(plant->num_collision_geometries(), 2);

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

}  // namespace
}  // namespace multibody
}  // namespace drake
