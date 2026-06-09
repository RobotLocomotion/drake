/* Tests that mass properties are identical whether welded-together links are
modeled with explicit weld joints or combined into a single composite
mobilized body. The test builds two identical models that differ only in whether
SetCombineWeldedBodies() is enabled. */

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/rotational_inertia.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

using math::RigidTransformd;
using math::RotationMatrixd;
using systems::Context;

// Tolerance for numerical comparisons.
constexpr double kTolerance = 32 * std::numeric_limits<double>::epsilon();

// Holds one version of the test model (either explicit welds or composites)
// along with its context, ready for kinematics queries.
struct TestModel {
  std::unique_ptr<MultibodyPlant<double>> plant;
  std::unique_ptr<Context<double>> context;
  const RevoluteJoint<double>* revolute{};
  const RigidBody<double>* link1{};
  const RigidBody<double>* link2{};
  const RigidBody<double>* link3{};
  const RigidBody<double>* link4{};
};

/* Builds a test model with the topology:

                          Link3
                            ^
                            |             y
                          [weld]          ^
                            |             |
         Link1 --[weld]--> Link2          +----> x
          /                              /
    [revolute z] (angle θ)              z
        /
      World --[weld]--> Link4

Link2 is offset +1 m in x from Link1's frame.
Link3 is offset +1 m in y from Link2's frame.
Link4 is offset +4 m in x from World frame.

The positions of the link origins from World origin Wo, expressed in World are:
  Link1: (0, 0, 0)
  Link2: (cos θ, sin θ, 0)
  Link3: (cos θ − sin θ, sin θ + cos θ, 0)
  Link4: (4, 0, 0)

With combine_welded_bodies = false, five mobilized bodies are created,
(World, Link1 via revolute, Link2 via weld, Link3 via weld, Link 4 via weld).
With combine_welded_bodies = true two mobilized bodies are created,
(World with link4 and one composite mobilized body with Link1, Link2, Link3). */
TestModel MakeModel(bool combine_welded_bodies) {
  TestModel m;
  m.plant = std::make_unique<MultibodyPlant<double>>(0.0 /* continuous */);
  m.plant->SetCombineWeldedBodies(combine_welded_bodies);

  // To facilitate an analytical solution, each link has a trivial inertia,
  // namely a 1 kg solid cube, 0.1 m per side.
  const SpatialInertia<double> M =
      SpatialInertia<double>::SolidCubeWithMass(1.0, 0.1);

  // Add the four links.
  m.link1 = &m.plant->AddRigidBody("Link1", M);
  m.link2 = &m.plant->AddRigidBody("Link2", M);
  m.link3 = &m.plant->AddRigidBody("Link3", M);
  m.link4 = &m.plant->AddRigidBody("Link4", M);

  // Revolute joint (z-axis): World to Link1, with Link1 frame at world frame.
  m.revolute = &m.plant->AddJoint<RevoluteJoint>(
      "revolute", m.plant->world_body(), RigidTransformd{}, *m.link1,
      RigidTransformd{}, Vector3<double>::UnitZ());

  // Weld Link2 to Link1, with Link2's joint frame offset +1 m in x from Link1.
  const RigidTransformd X_1to2(Vector3<double>(1.0, 0.0, 0.0));
  m.plant->AddJoint<WeldJoint>("weld12", *m.link1, X_1to2, *m.link2,
                               RigidTransformd{}, RigidTransformd{});

  // Weld Link3 to Link2, with Link3's joint frame offset +1 m in y from Link2.
  // Using y (not x) avoids a linear layout that could mask transform bugs.
  const RigidTransformd X_2to3(Vector3<double>(0.0, 1.0, 0.0));
  m.plant->AddJoint<WeldJoint>("weld23", *m.link2, X_2to3, *m.link3,
                               RigidTransformd{}, RigidTransformd{});

  // Weld Link4 to World, with Link4's joint frame offset +4 m in x from World.
  const RigidTransformd X_Wto4(Vector3<double>(4.0, 0.0, 0.0));
  m.plant->AddJoint<WeldJoint>("weldW4", m.plant->world_body(), X_Wto4,
                               *m.link4, RigidTransformd{}, RigidTransformd{});
  m.plant->Finalize();
  m.context = m.plant->CreateDefaultContext();

  // Sanity check: Both models should have the same number of bodies (links),
  // joints, and number of states (albeit different number of mobilized bodies).
  EXPECT_EQ(m.plant->num_bodies(), 5);      // World + 4 links.
  EXPECT_EQ(m.plant->num_joints(), 4);      // 1 revolute + 3 welds.
  EXPECT_EQ(m.plant->num_positions(), 1);   // 1 revolute angle.
  EXPECT_EQ(m.plant->num_velocities(), 1);  // 1 revolute angular rate.
  const internal::MultibodyTree<double>& tree = GetInternalTree(*m.plant);
  EXPECT_EQ(tree.num_mobods(), combine_welded_bodies ? 2 : 5);
  EXPECT_EQ(tree.num_mobilizers(), combine_welded_bodies ? 2 : 5);
  // Note: num_mobilizers() == num_mobods() because the World body gets a dummy
  // weld mobilizer at index 0 to keep mobilizer/body-node indexing identical.

  // Sanity check: Some information in the SpanningForest should be the same,
  // whether or not the welded links are combined.
  const internal::SpanningForest& forest = tree.forest();
  const internal::SpanningForest::Mobod& mobod_0 =
      forest.mobods(internal::MobodIndex(0));
  const internal::SpanningForest::Mobod& mobod_1 =
      forest.mobods(internal::MobodIndex(1));
  EXPECT_TRUE(mobod_0.is_world());      // World is mobod(0) and LinkOrdinal(0).
  EXPECT_TRUE(mobod_1.is_base_body());  // Connects to World via revolute joint.
  EXPECT_EQ(mobod_0.active_link_ordinal(), LinkOrdinal(0));
  EXPECT_EQ(mobod_1.active_link_ordinal(), LinkOrdinal(1));

  // Ensure Mobods (mobilized bodies) have the proper follower link ordinals.
  // Each Mobod should have an active link ordinal. If welded links have
  // been combined, then there are additional follower link ordinals.
  if (combine_welded_bodies) {
    EXPECT_EQ(mobod_0.follower_link_ordinals(),
              (std::vector{LinkOrdinal(0), LinkOrdinal(4)}));
    EXPECT_EQ(mobod_1.follower_link_ordinals(),
              (std::vector{LinkOrdinal(1), LinkOrdinal(2), LinkOrdinal(3)}));
  } else {
    EXPECT_EQ(mobod_0.follower_link_ordinals(), (std::vector{LinkOrdinal(0)}));
    EXPECT_EQ(mobod_1.follower_link_ordinals(), (std::vector{LinkOrdinal(1)}));
  }
  return m;
}

// Sets the revolute joint angle (q) and angular velocity (v) in the model.
void SetState(const TestModel& m, double angle_rad, double angular_vel) {
  m.revolute->set_angle(m.context.get(), angle_rad);
  m.revolute->set_angular_rate(m.context.get(), angular_vel);
}

/* Ensure the composite mobilized body's combined spatial inertia for Link123
(links 1, 2, 3) is computed correctly. The 1x1 mass matrix for this 1-DOF
model directly depends on the spatial inertia of Link123, so we also verify:
  (a) The mass matrix is identical between the explicit-weld and composite
      models at several configurations.
  (b) The mass matrix matches the analytically computed value.

Analytical derivation
---------------------
The model has 4 links, Linki (i=1,2,3,4), each a 1 kg solid cube of side 0.1 m.
Links 1,2,3 are welded together and are connected to World via a revolute joint
(z-axis at World origin). Link4 is welded directly to World. The link body-frame
origins are coincident with their associated centers of mass, located with:
  Link1 origin from World origin: p₁ = (0, 0, 0) expressed in World.
  Link2 origin from Link1 origin: p₂ = (1, 0, 0) expressed in Link1.
  Link3 origin from Link1 origin: p₃ = (1, 1, 0) expressed in Link1.
  Link4 origin from World origin: p₄ = (4, 0, 0) expressed in World.

For a solid cube of mass m and side a, its moment of inertia about any axis
through its COM is m*a²/6. The parallel axis theorem calculates each cube's
moment of inertia about the revolute's z-axis via: Iᵢ = m*a²/6 + m*(dᵢ)²,
where dᵢ (i=1,2,3) is the distance between each cube's COM and the revolute's
z-axis. Iᵢ is independent of joint angle because the composite is welded
together.

  Link1: I₁ = 1*(0.1)²/6 + 1*0²    = 1/600 + 0   (d² = 0)
  Link2: I₂ = 1*(0.1)²/6 + 1*1²    = 1/600 + 1   (d² = 1)
  Link3: I₃ = 1*(0.1)²/6 + 1*√2²   = 1/600 + 2   (d² = 2)
  Total: Iₜ = 3/600 + 3 = 1/200 + 3 = 3.005 kg·m² */
GTEST_TEST(CompositeTest, CompositeSpatialInertia) {
  const TestModel explicit_model = MakeModel(false /* no combining */);
  const TestModel composite_model = MakeModel(true /* combine welds */);

  const double m = 1.0, a = 0.1;  // mass m and side-length a of solid cubes.
  const Frame<double>& world_frame = explicit_model.plant->world_frame();
  const RigidBody<double>* explicit_links[] = {
      explicit_model.link1, explicit_model.link2, explicit_model.link3,
      explicit_model.link4};
  const RigidBody<double>* composite_links[] = {
      composite_model.link1, composite_model.link2, composite_model.link3,
      composite_model.link4};

  // The mass matrix is configuration-independent for this model (see above),
  // but we check at several angles to guard against future changes.
  const std::vector<double> angles = {0.0, M_PI / 6, M_PI / 4, -M_PI / 3};
  for (double angle : angles) {
    SetState(explicit_model, angle, 0.0);
    SetState(composite_model, angle, 0.0);

    // Verify Link123's summed spatial inertia does not depend on whether they
    // are a composite body.
    SpatialInertia<double> M_EWo_W = explicit_model.plant->CalcSpatialInertia(
        *explicit_model.context, world_frame,
        {explicit_model.link1->index(), explicit_model.link2->index(),
         explicit_model.link3->index()});
    SpatialInertia<double> M_CWo_W = composite_model.plant->CalcSpatialInertia(
        *composite_model.context, world_frame,
        {composite_model.link1->index(), composite_model.link2->index(),
         composite_model.link3->index()});
    EXPECT_TRUE(CompareMatrices(M_EWo_W.CopyToFullMatrix6(),
                                M_CWo_W.CopyToFullMatrix6(), kTolerance,
                                MatrixCompareType::relative))
        << "Link123 spatial inertia mismatch at angle = " << angle;

    // Ensure that individual link spatial inertias are reported correctly
    // regardless of whether they were fused.
    for (int i = 0; i < 4; ++i) {
      const RigidBody<double>* explicit_linki = explicit_links[i];
      const RigidBody<double>* composite_linki = composite_links[i];
      M_EWo_W = explicit_model.plant->CalcSpatialInertia(
          *explicit_model.context, world_frame, {explicit_linki->index()});
      M_CWo_W = composite_model.plant->CalcSpatialInertia(
          *composite_model.context, world_frame, {composite_linki->index()});
      EXPECT_TRUE(CompareMatrices(M_EWo_W.CopyToFullMatrix6(),
                                  M_CWo_W.CopyToFullMatrix6(), kTolerance,
                                  MatrixCompareType::relative))
          << "Spatial inertia mismatch: link" << i + 1
          << " at angle = " << angle;

      // Since link4 is welded to world, special-case calculations are used. For
      // this special case, also compare link4 results to an analytical value.
      if (i == 3) {
        const Vector3<double> p_WoL4o_W(4.0, 0.0, 0.0);
        SpatialInertia<double> M_L4Wo_W_expected =
            SpatialInertia<double>::SolidCubeWithMass(m, a).Shift(-p_WoL4o_W);
        EXPECT_TRUE(CompareMatrices(M_L4Wo_W_expected.CopyToFullMatrix6(),
                                    M_CWo_W.CopyToFullMatrix6(), kTolerance,
                                    MatrixCompareType::relative))
            << "Inaccurate link4 spatial inertia at angle = " << angle;
      }
    }

    // Ensure the mass matrix does not depend on welded links being combined.
    MatrixX<double> M_explicit(1, 1), M_composite(1, 1);
    explicit_model.plant->CalcMassMatrix(*explicit_model.context, &M_explicit);
    composite_model.plant->CalcMassMatrix(*composite_model.context,
                                          &M_composite);
    EXPECT_TRUE(CompareMatrices(M_explicit, M_composite, kTolerance,
                                MatrixCompareType::relative))
        << "Mass matrix mismatch at angle = " << angle;

    // Ensure the mass matrix matches the analytical value.
    const double Izz = m * a * a / 6.0;  // Izz of one cube about its COM.
    const double M_expected = (Izz + m * 0.0) +  // Link1: d² = 0
                              (Izz + m * 1.0) +  // Link2: d² = 1² = 1
                              (Izz + m * 2.0);   // Link3: d² = √2² = 2
    EXPECT_NEAR(M_composite(0, 0), M_expected, kTolerance)
        << "Mass matrix analytical mismatch at angle = " << angle;
  }
}

/* Tests that CalcFrameBodyPoses() computes the correct composite mass
properties by exercising every code path (including reverse welds):
- Pass 1: non-identity frame poses X_LF, where link frame L and frame F are
  both rigidly attached to the same body (so X_LF is constant). When a
  weld joint is NOT reversed, F is the inboard frame on the parent body, whereas
  for a reversed weld, F is the inboard frame on the child body.
- Pass 2: poses X_BL, where mobod (mobilized body) frame B and link frame L
  are both rigidly attached to the same body (so X_BL is constant). If L is the
  composite body's "active" link, frame B is frame L and X_BL is identity. In
  general, X_BL is non-identity for each "follower" link L welded into the
  composite body B.
- Pass 3: mass property accumulation with both shifting and re-expressing.

The strategy is to build two versions of the same physical system -- one with
composites enabled and one without -- then verify that the mass matrix and
gravity generalized forces agree. The mass matrix depends on M_BBo_B (the
composite spatial inertia computed in Pass 3), while gravity forces also
depend on p_BoLcm_B (each follower link's center of mass offset from Bo).

Topology:
                             LinkC
                              ^
                              |  (reversed weld: child=LinkB, parent=LinkC)
                              |
    LinkA --[normal weld]--> LinkB
      /
  [revolute y]
     /
   World
*/
GTEST_TEST(CompositeTest, CalcFrameBodyPosesAllPaths) {
  // Normal (not reversed) weld between parent=LinkA → child=LinkB.
  // Weld joint parent frame Jp is offset from LinkA's frame A by
  //      (+0.5, 0, +0.3) with a 30-degree rotation about x.
  // Weld joint child frame Jc is offset from LinkB's frame B by (0, +0.2, 0).
  // X_JpJc = identity (coincident joint parent Jp and joint child Jp frames).
  const RigidTransformd X_AJp(RotationMatrixd::MakeXRotation(M_PI / 6),
                              Vector3<double>(0.5, 0.0, 0.3));
  const RigidTransformd X_BJc(Vector3<double>(0.0, 0.2, 0.0));
  const RigidTransformd X_JpJc_AB;  // Identity transform.

  // Reversed weld between parent=LinkC → child=LinkB (but LinkB is inboard).
  // "Reversed" means linkB is the inboard link and LinkC is the outboard
  // link. Weld joint parent frame Jp is offset from LinkC's frame C by (0, 0,
  // +0.4). Weld joint child frame Jc is offset from LinkB's frame B by (0.6,
  // 0, 0). X_JpJc = translation(0.1, 0.2, 0.3) -- frames Jp and Jc are not
  // coincident.
  const RigidTransformd X_CJp(Vector3<double>(0.0, 0.0, 0.4));
  const RigidTransformd X_BJc2(Vector3<double>(0.6, 0.0, 0.0));
  const RigidTransformd X_JpJc_CB(Vector3<double>(0.1, 0.2, 0.3));

  // For robust testing, each link has a distinct spatial inertia with
  // non-zero products of inertia and non-zero center of mass offsets.
  const SpatialInertia<double> M_AAo_A =
      SpatialInertia<double>::MakeFromCentralInertia(
          2.0, Vector3<double>(0.1, -0.05, 0.02),
          RotationalInertia<double>(0.03, 0.05, 0.04, 0.001, -0.002, 0.0015));
  const SpatialInertia<double> M_BBo_B =
      SpatialInertia<double>::MakeFromCentralInertia(
          1.5, Vector3<double>(-0.08, 0.04, 0.06),
          RotationalInertia<double>(0.05, 0.06, 0.04, -0.001, 0.002, 0.001));
  const SpatialInertia<double> M_CCo_C =
      SpatialInertia<double>::MakeFromCentralInertia(
          3.0, Vector3<double>(0.0, 0.1, -0.07),
          RotationalInertia<double>(0.08, 0.04, 0.07, 0.002, 0.001, -0.003));

  // --- Helper lambda to build a model. ---
  auto make_model = [&](bool combine) {
    auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
    plant->SetCombineWeldedBodies(combine);

    const auto& linkA = plant->AddRigidBody("LinkA", M_AAo_A);
    const auto& linkB = plant->AddRigidBody("LinkB", M_BBo_B);
    const auto& linkC = plant->AddRigidBody("LinkC", M_CCo_C);

    // Revolute: World → LinkA about y.
    plant->AddJoint<RevoluteJoint>("revolute", plant->world_body(),
                                   RigidTransformd{}, linkA, RigidTransformd{},
                                   Vector3<double>::UnitY());

    // Normal weld: LinkA(parent) → LinkB(child).
    plant->AddJoint<WeldJoint>("weld_AB", linkA, X_AJp, linkB, X_BJc,
                               X_JpJc_AB);

    // Reversed weld: LinkC(parent) → LinkB(child). Because LinkB is already
    // connected to the tree via LinkA, the spanning forest will traverse this
    // joint from LinkB (inboard) to LinkC (outboard), making it "reversed".
    plant->AddJoint<WeldJoint>("weld_CB", linkC, X_CJp, linkB, X_BJc2,
                               X_JpJc_CB);

    plant->Finalize();
    auto context = plant->CreateDefaultContext();

    return std::pair{std::move(plant), std::move(context)};
  };

  // --- Build both models. ---
  auto [plant_nc, context_nc] = make_model(false);  // no composites
  auto [plant_c, context_c] = make_model(true);     // composites

  const auto& tree_nc = GetInternalTree(*plant_nc);
  const auto& tree_c = GetInternalTree(*plant_c);

  // Sanity check: Both models should have the same number of bodies (World +
  // 3 links), number of joints (1 revolute + 2 welds), and number of states
  // (1 revolute angle and 1 revolute angular rate), but they should differ in
  // the number of mobilized bodies.
  EXPECT_EQ(plant_nc->num_bodies(), plant_c->num_bodies());
  EXPECT_EQ(plant_nc->num_joints(), plant_c->num_joints());
  EXPECT_EQ(plant_nc->num_positions(), plant_c->num_positions());
  EXPECT_EQ(plant_nc->num_velocities(), plant_c->num_velocities());
  ASSERT_EQ(tree_nc.num_mobods(), 4);  // World + LinkA + LinkB + LinkC.
  ASSERT_EQ(tree_c.num_mobods(),
            2);  // World + composite body (links A, B, C).

  const auto& rev_nc = plant_nc->GetJointByName<RevoluteJoint>("revolute");
  const auto& rev_c = plant_c->GetJointByName<RevoluteJoint>("revolute");

  for (const double angle : {M_PI / 5, -M_PI / 3}) {
    rev_nc.set_angle(context_nc.get(), angle);
    rev_c.set_angle(context_c.get(), angle);

    // The mass matrix (1×1) should agree between the two models. This
    // directly validates that CalcFrameBodyPoses produced the correct
    // composite body inertia (M_BBo_B), since the mass matrix is computed
    // from it.
    MatrixX<double> mass_nc(1, 1), mass_c(1, 1);
    plant_nc->CalcMassMatrix(*context_nc, &mass_nc);
    plant_c->CalcMassMatrix(*context_c, &mass_c);
    EXPECT_TRUE(CompareMatrices(mass_nc, mass_c, kTolerance,
                                MatrixCompareType::relative))
        << "Mass matrix mismatch at angle = " << angle;

    // Gravity generalized forces should agree (exercises AccumulateGravity
    // which depends on p_BoLcm_B computed in CalcFrameBodyPoses).
    const VectorX<double> tau_g_nc =
        plant_nc->CalcGravityGeneralizedForces(*context_nc);
    const VectorX<double> tau_g_c =
        plant_c->CalcGravityGeneralizedForces(*context_c);
    EXPECT_TRUE(CompareMatrices(tau_g_nc, tau_g_c, kTolerance,
                                MatrixCompareType::relative))
        << "Gravity generalized forces mismatch at angle = " << angle;
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
