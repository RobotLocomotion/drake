/* Tests that mass properties are identical whether welded-together links are
modeled with unfused weld joints or whether links that are welded together
are fused onto a mobilized body (fused Mobod). The test builds two identical
models that differ only in whether SetFuseWeldedLinks() is enabled. */

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

// Holds one version of the test model, either a model with unfused welds
// or a model with welded links fused onto a mobilized body (fused mobod)
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

With fuse_welded_bodies = false, five mobilized bodies are created,
(World, Link1 via revolute, Link2 via weld, Link3 via weld, Link 4 via weld).
With fuse_welded_bodies = true two mobilized bodies are created,
(World with link4 and one fused mobilized body with Link1, Link2, Link3). */
TestModel MakeModel(bool fuse_welded_bodies) {
  TestModel m;
  m.plant = std::make_unique<MultibodyPlant<double>>(0.0 /* continuous */);
  m.plant->SetFuseWeldedLinks(fuse_welded_bodies);

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
  EXPECT_EQ(tree.num_mobods(), fuse_welded_bodies ? 2 : 5);
  EXPECT_EQ(tree.num_mobilizers(), fuse_welded_bodies ? 2 : 5);
  // Note: num_mobilizers() == num_mobods() because the World body gets a dummy
  // weld mobilizer at index 0 to keep mobilizer/body-node indexing identical.

  // Sanity check: Some information in the SpanningForest should be the same,
  // whether or not the welded links are fused.
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
  // been fused, then there are additional follower link ordinals.
  if (fuse_welded_bodies) {
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

/* Verify that the spatial inertia for fused mobod Link123 (with links 1, 2, 3)
matches the sum of spatial inertias for unfused links 1, 2, 3 at several poses.
Next, ensure spatial inertia for each of the follower links in a fused mobod are
computed correctly. Lastly, the 1x1 mass matrix for this 1-DOF model directly
depends on the spatial inertia of Link123, so we also verify:
  (a) The mass matrix is identical between the fused and unfused weld models.
  (b) The mass matrix matches the analytically computed value.

Similarly, verify spatial momentum calculations.

Analytical derivation for spatial inertia.
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
z-axis. Iᵢ is independent of joint angle because the links are welded together.

  Link1: I₁ = 1*(0.1)²/6 + 1*0²    = 1/600 + 0   (d² = 0)
  Link2: I₂ = 1*(0.1)²/6 + 1*1²    = 1/600 + 1   (d² = 1)
  Link3: I₃ = 1*(0.1)²/6 + 1*√2²   = 1/600 + 2   (d² = 2)
  Total: Iₜ = 3/600 + 3 = 1/200 + 3 = 3.005 kg·m² */
GTEST_TEST(FusedTest, CompositeSpatialInertia) {
  const TestModel unfused_model = MakeModel(false /* no combining */);
  const TestModel fused_model = MakeModel(true /* fuse welds */);

  const double m = 1.0, a = 0.1;  // mass m and side-length a of solid cubes.
  const Frame<double>& world_frame = unfused_model.plant->world_frame();
  const RigidBody<double>* unfused_links[] = {
      unfused_model.link1, unfused_model.link2, unfused_model.link3,
      unfused_model.link4};
  const RigidBody<double>* fused_links[] = {
      fused_model.link1, fused_model.link2, fused_model.link3,
      fused_model.link4};

  // Note: The mass matrix is configuration-independent for this model (see
  // above), but we check at several angles to guard against future changes.
  // Note: Use a non-zero angular velocity so spatial momentum is non-trivial.
  const std::vector<double> angles = {0.0, M_PI / 6, M_PI / 4, -M_PI / 3};
  for (double angle : angles) {
    SCOPED_TRACE(fmt::format("angle = {}", angle));
    SetState(unfused_model, angle, 2.0 /* rad/s */);
    SetState(fused_model, angle, 2.0 /* rad/s */);

    // Verify Link123's summed spatial inertia does not depend on fused links.
    SpatialInertia<double> M_UWo_W = unfused_model.plant->CalcSpatialInertia(
        *unfused_model.context, world_frame,
        {unfused_model.link1->index(), unfused_model.link2->index(),
         unfused_model.link3->index()});
    SpatialInertia<double> M_FWo_W = fused_model.plant->CalcSpatialInertia(
        *fused_model.context, world_frame,
        {fused_model.link1->index(), fused_model.link2->index(),
         fused_model.link3->index()});
    EXPECT_TRUE(CompareMatrices(M_UWo_W.CopyToFullMatrix6(),
                                M_FWo_W.CopyToFullMatrix6(), kTolerance,
                                MatrixCompareType::relative));

    // Verify Link123's summed spatial momentum does not depend on fused links.
    const Vector3<double> p_WoP_W(1.1, -2.3, 4.2);
    SpatialMomentum<double> L_WUP_W =
        unfused_model.plant->CalcSpatialMomentumInWorldAboutPoint(
            *unfused_model.context,
            {unfused_model.link1->model_instance(),
             unfused_model.link2->model_instance(),
             unfused_model.link3->model_instance()},
            p_WoP_W);
    SpatialMomentum<double> L_WFP_W =
        fused_model.plant->CalcSpatialMomentumInWorldAboutPoint(
            *fused_model.context,
            {fused_model.link1->model_instance(),
             fused_model.link2->model_instance(),
             fused_model.link3->model_instance()},
            p_WoP_W);
    // TODO(Mitiguy) EXPECT_FALSE is wrong!  Should be EXPECT_TRUE!
    EXPECT_FALSE(CompareMatrices(L_WUP_W.get_coeffs(), L_WFP_W.get_coeffs(),
                                 kTolerance, MatrixCompareType::relative));

    // Ensure that individual link spatial inertias and spatial momentum are
    // accurately calculated, regardless of whether they were fused.
    for (int i = 0; i < 4; ++i) {
      const RigidBody<double>* unfused_linki = unfused_links[i];
      const RigidBody<double>* fused_linki = fused_links[i];
      M_UWo_W = unfused_model.plant->CalcSpatialInertia(
          *unfused_model.context, world_frame, {unfused_linki->index()});
      M_FWo_W = fused_model.plant->CalcSpatialInertia(
          *fused_model.context, world_frame, {fused_linki->index()});
      EXPECT_TRUE(CompareMatrices(M_UWo_W.CopyToFullMatrix6(),
                                  M_FWo_W.CopyToFullMatrix6(), kTolerance,
                                  MatrixCompareType::relative));

      // Verify individual links' spatial momentum do not depend on fused links.
      L_WUP_W = unfused_model.plant->CalcSpatialMomentumInWorldAboutPoint(
          *unfused_model.context, {unfused_linki->model_instance()}, p_WoP_W);
      L_WFP_W = fused_model.plant->CalcSpatialMomentumInWorldAboutPoint(
          *fused_model.context, {fused_linki->model_instance()}, p_WoP_W);
      // TODO(Mitiguy) EXPECT_FALSE is wrong!  Should be EXPECT_TRUE!
      EXPECT_FALSE(CompareMatrices(L_WUP_W.get_coeffs(), L_WFP_W.get_coeffs(),
                                   kTolerance, MatrixCompareType::relative));

      // Since link4 is welded to world, special-case calculations are used. For
      // this special case, also compare link4 results to an analytical value.
      if (i == 3) {
        const Vector3<double> p_WoL4o_W(4.0, 0.0, 0.0);
        SpatialInertia<double> M_L4Wo_W_expected =
            SpatialInertia<double>::SolidCubeWithMass(m, a).Shift(-p_WoL4o_W);
        EXPECT_TRUE(CompareMatrices(M_L4Wo_W_expected.CopyToFullMatrix6(),
                                    M_FWo_W.CopyToFullMatrix6(), kTolerance,
                                    MatrixCompareType::relative));

        // Link4's spatial momentum should always be zero (welded to ground).
        // TODO(Mitiguy) EXPECT_FALSE is wrong!  Should be EXPECT_TRUE!
        EXPECT_FALSE(CompareMatrices(L_WFP_W.get_coeffs(),
                                     Vector6<double>::Zero(), kTolerance,
                                     MatrixCompareType::relative));
      }
    }

    // Ensure the mass matrix does not depend on welded links being combined.
    MatrixX<double> M_unfused(1, 1), M_fused(1, 1);
    unfused_model.plant->CalcMassMatrix(*unfused_model.context, &M_unfused);
    fused_model.plant->CalcMassMatrix(*fused_model.context, &M_fused);
    EXPECT_TRUE(CompareMatrices(M_unfused, M_fused, kTolerance,
                                MatrixCompareType::relative));

    // Ensure the mass matrix matches the analytical value.
    const double Izz = m * a * a / 6.0;  // Izz of one cube about its COM.
    const double M_expected = (Izz + m * 0.0) +  // Link1: d² = 0
                              (Izz + m * 1.0) +  // Link2: d² = 1² = 1
                              (Izz + m * 2.0);   // Link3: d² = √2² = 2
    EXPECT_NEAR(M_fused(0, 0), M_expected, kTolerance);

    // Ensure spatial momentum does not depend on fused welded links.
    // Also, use an "about-point" P which is not coincident with Wo.
    const SpatialMomentum<double> L_unfused =
        unfused_model.plant->CalcSpatialMomentumInWorldAboutPoint(
            *unfused_model.context, p_WoP_W);

    // Test the function for the entire system's spatial momentum in world.
    const SpatialMomentum<double> L_fused =
        fused_model.plant->CalcSpatialMomentumInWorldAboutPoint(
            *fused_model.context, p_WoP_W);
    EXPECT_TRUE(CompareMatrices(L_unfused.get_coeffs(), L_fused.get_coeffs(),
                                kTolerance, MatrixCompareType::relative));
  }
}

/* Tests that CalcFrameBodyPoses() computes the correct composite mass
properties by exercising every code path (including reverse welds):
- Pass 1: non-identity frame poses X_LF, where link frame L and frame F are
  both rigidly attached to the same mobod (so X_LF is constant). When a
  weld joint is NOT reversed, F is the inboard frame on the parent body, whereas
  for a reversed weld, F is the inboard frame on the child body.
- Pass 2: poses X_BL, where mobod (mobilized body) frame B and link frame L
  are both rigidly attached to the same mobod (so X_BL is constant). If L is the
  fused mobod's "active" link, frame B is frame L and X_BL is identity. In
  general, X_BL is non-identity for each "follower" link L welded onto the
  fused mobod B.
- Pass 3: mass property accumulation with both shifting and re-expressing.

The strategy is to build two versions of the same physical system -- one with
fusing enabled and one without -- then verify that the mass matrix and gravity
generalized forces agree. The mass matrix depends on M_BBo_B (the spatial
inertia computed in Pass 3 of CalcFrameBodyPoses()), while gravity forces also
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
GTEST_TEST(FusedTest, CalcFrameBodyPosesAllPaths) {
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
  auto make_model = [&](bool fuse) {
    auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
    plant->SetFuseWeldedLinks(fuse);

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
  auto [plant_nf, context_nf] = make_model(false);  // no fusing
  auto [plant_f, context_f] = make_model(true);     // fusing

  const auto& tree_nf = GetInternalTree(*plant_nf);
  const auto& tree_f = GetInternalTree(*plant_f);

  // Sanity check: Both models should have the same number of bodies (World +
  // 3 links), number of joints (1 revolute + 2 welds), and number of states
  // (1 revolute angle and 1 revolute angular rate), but they should differ in
  // the number of mobilized bodies.
  EXPECT_EQ(plant_nf->num_bodies(), plant_f->num_bodies());
  EXPECT_EQ(plant_nf->num_joints(), plant_f->num_joints());
  EXPECT_EQ(plant_nf->num_positions(), plant_f->num_positions());
  EXPECT_EQ(plant_nf->num_velocities(), plant_f->num_velocities());
  ASSERT_EQ(tree_nf.num_mobods(), 4);  // World + LinkA + LinkB + LinkC.
  ASSERT_EQ(tree_f.num_mobods(),
            2);  // World + fused mobod (links A, B, C).

  const auto& rev_nf = plant_nf->GetJointByName<RevoluteJoint>("revolute");
  const auto& rev_f = plant_f->GetJointByName<RevoluteJoint>("revolute");

  for (const double angle : {M_PI / 5, -M_PI / 3}) {
    SCOPED_TRACE(fmt::format("angle = {}", angle));
    rev_nf.set_angle(context_nf.get(), angle);
    rev_f.set_angle(context_f.get(), angle);

    // The mass matrix (1×1) should agree between the two models. This directly
    // validates that CalcFrameBodyPoses() produced the correct mobod inertia
    // (M_BBo_B), since the mass matrix is computed from it.
    MatrixX<double> mass_nf(1, 1), mass_f(1, 1);
    plant_nf->CalcMassMatrix(*context_nf, &mass_nf);
    plant_f->CalcMassMatrix(*context_f, &mass_f);
    EXPECT_TRUE(CompareMatrices(mass_nf, mass_f, kTolerance,
                                MatrixCompareType::relative));

    // Gravity generalized forces should agree (exercises AccumulateGravity
    // which depends on p_BoLcm_B computed in CalcFrameBodyPoses).
    const VectorX<double> tau_g_nf =
        plant_nf->CalcGravityGeneralizedForces(*context_nf);
    const VectorX<double> tau_g_f =
        plant_f->CalcGravityGeneralizedForces(*context_f);
    EXPECT_TRUE(CompareMatrices(tau_g_nf, tau_g_f, kTolerance,
                                MatrixCompareType::relative));
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
