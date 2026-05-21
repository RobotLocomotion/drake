/* Tests that link kinematics (poses and spatial velocities) are identical
whether welded-together links are modeled with explicit weld joints or combined
into a single composite mobilized body.

The test builds two identical models that differ only in whether
SetCombineWeldedBodies() is enabled. In both cases, the link poses and
velocities must be the same, since composites are kinematically equivalent to
their explicit-weld counterparts. */

#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
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
constexpr double kTolerance = 8 * std::numeric_limits<double>::epsilon();

// Holds one version of the test model (either explicit welds or composites)
// along with its context, ready for kinematics queries.
struct TestModel {
  std::unique_ptr<MultibodyPlant<double>> plant;
  std::unique_ptr<Context<double>> context;
  const RevoluteJoint<double>* revolute{};
  const RigidBody<double>* link1{};
  const RigidBody<double>* link2{};
  const RigidBody<double>* link3{};
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
      World

where Link2 is offset +1 m in x from Link1's frame and Link3 is offset +1 m in y
from Link2's frame.

At joint angle θ, the link origins in World are:
  Link1: (0, 0, 0)
  Link2: (cos θ, sin θ, 0)
  Link3: (cos θ − sin θ, sin θ + cos θ, 0)

With combine_welded_bodies=false, four mobilized bodies are created (World,
Link1 via revolute, Link2 and Link3 each via weld). With
combine_welded_bodies=true we get World plus a single composite mobilized body
that contains all three links. */
TestModel MakeModel(bool combine_welded_bodies) {
  TestModel m;
  m.plant = std::make_unique<MultibodyPlant<double>>(0.0 /* continuous */);
  m.plant->SetCombineWeldedBodies(combine_welded_bodies);

  // Give each link a non-trivial inertia: a 1 kg solid cube, 0.1 m per side.
  const SpatialInertia<double> M =
      SpatialInertia<double>::SolidCubeWithMass(1.0, 0.1);

  // Add the three links.
  m.link1 = &m.plant->AddRigidBody("Link1", M);
  m.link2 = &m.plant->AddRigidBody("Link2", M);
  m.link3 = &m.plant->AddRigidBody("Link3", M);

  // Revolute joint: World → Link1, rotating about z.
  m.revolute = &m.plant->AddJoint<RevoluteJoint>(
      "revolute", m.plant->world_body(), RigidTransformd{}, *m.link1,
      RigidTransformd{}, Vector3<double>::UnitZ());

  // Weld Link2 to Link1, with Link2's joint frame offset +1 m in x from Link1.
  const RigidTransformd X_1to2(Vector3<double>(1.0, 0.0, 0.0));
  m.plant->AddJoint<WeldJoint>("weld12", *m.link1, X_1to2, *m.link2,
                               RigidTransformd{}, RigidTransformd{});

  // Weld Link3 to Link2, with Link3's joint frame offset +1 m in y from Link2.
  // Using y (not x) avoids a collinear layout that could mask transform bugs.
  const RigidTransformd X_2to3(Vector3<double>(0.0, 1.0, 0.0));
  m.plant->AddJoint<WeldJoint>("weld23", *m.link2, X_2to3, *m.link3,
                               RigidTransformd{}, RigidTransformd{});

  m.plant->Finalize();
  m.context = m.plant->CreateDefaultContext();

  // Sanity check that we got what we asked for. Both models have the bodies
  // (links), joints, and number of states in common, but they should differ in
  // the number of mobilized bodies.
  EXPECT_EQ(m.plant->num_bodies(), 4);  // world + 3 links
  EXPECT_EQ(m.plant->num_joints(), 3);
  EXPECT_EQ(m.plant->num_positions(), 1);
  EXPECT_EQ(m.plant->num_velocities(), 1);

  const internal::MultibodyTree<double>& tree = GetInternalTree(*m.plant);
  EXPECT_EQ(tree.num_mobods(), combine_welded_bodies ? 2 : 4);

  if (combine_welded_bodies) {
    const internal::SpanningForest& forest = tree.forest();
    const internal::SpanningForest::Mobod& mobod_1 =
        forest.mobods(internal::MobodIndex(1));
    EXPECT_EQ(mobod_1.follower_link_ordinals(),
              (std::vector{LinkOrdinal(1), LinkOrdinal(2), LinkOrdinal(3)}));
    EXPECT_EQ(mobod_1.active_link_ordinal(), LinkOrdinal(1));
  }
  return m;
}

// Sets the revolute joint angle (q) and angular velocity (v) in the model.
void SetState(const TestModel& m, double angle_rad, double angular_vel) {
  m.revolute->set_angle(m.context.get(), angle_rad);
  m.revolute->set_angular_rate(m.context.get(), angular_vel);
}

/* Tests that link poses and spatial velocities are identical whether welds are
modeled explicitly or combined into a composite mobilized body. This doesn't
check that the values are right, just that they match. Other tests below will
check some numerical results. */
GTEST_TEST(CompositeTest, LinkKinematicsMatchExplicitWelds) {
  const TestModel explicit_model = MakeModel(false /* no combining */);
  const TestModel composite_model = MakeModel(true /* combine welds */);

  // Test at several non-trivial (q, v) configurations.
  const std::vector<std::pair<double, double>> configs = {
      {0.0, 0.0},      {M_PI / 6, 0.5},  {M_PI / 4, -1.2},
      {M_PI / 2, 2.0}, {-M_PI / 3, 0.7},
  };

  for (const auto& [angle, omega] : configs) {
    SetState(explicit_model, angle, omega);
    SetState(composite_model, angle, omega);

    for (const auto& [link_name, explicit_link, composite_link] :
         std::vector<std::tuple<const char*, const RigidBody<double>*,
                                const RigidBody<double>*>>{
             {"Link1", explicit_model.link1, composite_model.link1},
             {"Link2", explicit_model.link2, composite_model.link2},
             {"Link3", explicit_model.link3, composite_model.link3},
         }) {
      // Compare world poses.
      const RigidTransformd& X_WL_explicit =
          explicit_link->EvalPoseInWorld(*explicit_model.context);
      const RigidTransformd& X_WL_composite =
          composite_link->EvalPoseInWorld(*composite_model.context);

      EXPECT_TRUE(X_WL_explicit.IsNearlyEqualTo(X_WL_composite, kTolerance))
          << "Pose mismatch for " << link_name << " at angle=" << angle;

      // Compare world spatial velocities.
      const SpatialVelocity<double>& V_WL_explicit =
          explicit_link->EvalSpatialVelocityInWorld(*explicit_model.context);
      const SpatialVelocity<double>& V_WL_composite =
          composite_link->EvalSpatialVelocityInWorld(*composite_model.context);

      EXPECT_TRUE(CompareMatrices(V_WL_explicit.get_coeffs(),
                                  V_WL_composite.get_coeffs(), kTolerance,
                                  MatrixCompareType::relative))
          << "Velocity mismatch for " << link_name << " at angle=" << angle
          << " omega=" << omega;
    }
  }
}

/* Tests that at the zero configuration, Link1 is at the World origin, Link2 is
1 m in +x, and Link3 is 1 m in +x and 1 m in +y. */
GTEST_TEST(CompositeTest, ZeroConfigurationPoses) {
  for (bool combine : {false, true}) {
    SCOPED_TRACE(combine ? "composite" : "explicit welds");
    const TestModel m = MakeModel(combine);
    SetState(m, 0.0, 0.0);

    // Link1 at origin.
    const RigidTransformd& X_WL1 = m.link1->EvalPoseInWorld(*m.context);
    EXPECT_TRUE(X_WL1.IsNearlyEqualTo(RigidTransformd::Identity(), kTolerance));

    // Link2 at (1, 0, 0), no rotation.
    const RigidTransformd& X_WL2 = m.link2->EvalPoseInWorld(*m.context);
    EXPECT_TRUE(X_WL2.IsNearlyEqualTo(
        RigidTransformd(Vector3<double>(1.0, 0.0, 0.0)), kTolerance));

    // Link3 at (1, 1, 0): Link2's y-axis at θ=0 is World +y, no rotation.
    const RigidTransformd& X_WL3 = m.link3->EvalPoseInWorld(*m.context);
    EXPECT_TRUE(X_WL3.IsNearlyEqualTo(
        RigidTransformd(Vector3<double>(1.0, 1.0, 0.0)), kTolerance));
  }
}

/* Tests that at 90-degree rotation, the expected rotated positions are correct
regardless of whether welds are combined.

At θ=π/2 the Link1 frame is rotated 90° about z, so Link1's x-axis points
in World +y. Link3's offset (+1 in Link2's y) then points in World -x.
  Link1: (0, 0, 0)
  Link2: cos(π/2)·x̂ + sin(π/2)·ŷ = (0, 1, 0)
  Link3: Link2 origin + R_WL2 * (0,1,0) = (0,1,0) + (-1,0,0) = (-1, 1, 0) */
GTEST_TEST(CompositeTest, NinetyDegreePoses) {
  for (bool combine : {false, true}) {
    SCOPED_TRACE(combine ? "composite" : "explicit welds");
    const TestModel m = MakeModel(combine);
    SetState(m, M_PI / 2, 0.0);

    // All links share Link1's rotation: 90° about z.
    const math::RotationMatrixd R_W90z =
        math::RotationMatrixd::MakeZRotation(M_PI / 2);

    // Link1's origin stays at the world origin.
    const RigidTransformd& X_WL1 = m.link1->EvalPoseInWorld(*m.context);
    EXPECT_TRUE(X_WL1.IsNearlyEqualTo(
        RigidTransformd(R_W90z, Vector3<double>::Zero()), kTolerance));

    // Link2's origin is 1 m along Link1's x-axis → (0, 1, 0) in World.
    const RigidTransformd& X_WL2 = m.link2->EvalPoseInWorld(*m.context);
    EXPECT_TRUE(X_WL2.IsNearlyEqualTo(
        RigidTransformd(R_W90z, Vector3<double>(0.0, 1.0, 0.0)), kTolerance));

    // Link3's origin is 1 m along Link2's y-axis. After 90° rotation, Link2's
    // y-axis points in World -x, so Link3 is at (0,1,0) + (-1,0,0) = (-1,1,0).
    const RigidTransformd& X_WL3 = m.link3->EvalPoseInWorld(*m.context);
    EXPECT_TRUE(X_WL3.IsNearlyEqualTo(
        RigidTransformd(R_W90z, Vector3<double>(-1.0, 1.0, 0.0)), kTolerance));
  }
}

/* Tests that spatial velocities are correct for a spinning configuration
regardless of whether welds are combined. At angle θ with angular rate ω, each
link has:
  rotational velocity: ω * ẑ (World)
  translational velocity at its origin: ω * ẑ × p_WLi
where p_WLi is the world position of the iᵗʰ link's origin. */
GTEST_TEST(CompositeTest, SpatialVelocities) {
  const double angle = M_PI / 4;
  const double omega = 2.0;

  for (bool combine : {false, true}) {
    SCOPED_TRACE(combine ? "composite" : "explicit welds");
    const TestModel m = MakeModel(combine);
    SetState(m, angle, omega);

    // The angular velocity of all links is ω * ẑ.
    const Vector3<double> w_expected = omega * Vector3<double>::UnitZ();
    for (const RigidBody<double>* link : {m.link1, m.link2, m.link3}) {
      const SpatialVelocity<double>& V_WL =
          link->EvalSpatialVelocityInWorld(*m.context);
      EXPECT_TRUE(CompareMatrices(V_WL.rotational(), w_expected, kTolerance,
                                  MatrixCompareType::relative));
    }

    // Translational velocity of Link1's origin is zero (it's at World origin).
    const SpatialVelocity<double>& V_WL1 =
        m.link1->EvalSpatialVelocityInWorld(*m.context);
    EXPECT_TRUE(CompareMatrices(V_WL1.translational(), Vector3<double>::Zero(),
                                kTolerance, MatrixCompareType::relative));

    // The translational velocity of Link2's origin is ω × p_WL2.
    // p_WL2 = R_W_L1 * [1, 0, 0] = [cos θ, sin θ, 0].
    // v_WL2 = ω ẑ × [cos θ, sin θ, 0] = ω[-sin θ, cos θ, 0].
    const Vector3<double> v_WL2o_expected =
        omega * Vector3<double>(-std::sin(angle), std::cos(angle), 0.0);
    const SpatialVelocity<double>& V_WL2 =
        m.link2->EvalSpatialVelocityInWorld(*m.context);
    EXPECT_TRUE(CompareMatrices(V_WL2.translational(), v_WL2o_expected,
                                kTolerance, MatrixCompareType::relative));

    // Link3's origin is at R_WL1 * [1,0,0] + R_WL1 * [0,1,0]
    //   = [cos θ - sin θ, sin θ + cos θ, 0].
    // v_WL3 = ω ẑ × p_WL3
    //       = ω ẑ × [cos θ - sin θ, sin θ + cos θ, 0]
    //       = ω[-(sin θ + cos θ), cos θ - sin θ, 0].
    const Vector3<double> v_WL3o_expected =
        omega * Vector3<double>(-(std::sin(angle) + std::cos(angle)),
                                std::cos(angle) - std::sin(angle), 0.0);
    const SpatialVelocity<double>& V_WL3 =
        m.link3->EvalSpatialVelocityInWorld(*m.context);
    EXPECT_TRUE(CompareMatrices(V_WL3.translational(), v_WL3o_expected,
                                kTolerance, MatrixCompareType::relative));
  }
}

/* Tests that the composite mobilized body's combined spatial inertia is
computed correctly. The mass matrix (a 1×1 matrix for this 1-DOF model) directly
depends on the combined spatial inertia of the composite, so we verify:
  (a) The mass matrix is identical between the explicit-weld and composite
      models at several configurations.
  (b) The mass matrix matches the analytically computed value.

Analytical derivation
---------------------
The model has a single revolute joint (z-axis at World origin) with three welded
links, each a 1 kg solid cube of side 0.1 m. Their body-frame origins (= centers
of mass) are at:
  Link1: p = (0, 0, 0)   — at the joint
  Link2: p = (1, 0, 0)   in Link1's frame
  Link3: p = (1, 1, 0)   in Link1's frame (1 m in x then 1 m in Link2's y)

For a solid cube of mass m and side a, the moment of inertia about any axis
through the COM equals m*a²/6. The moment of inertia about the revolute axis (z
through world origin) follows from the parallel axis theorem: I = m*a²/6 +
m*d², where d is the distance from the COM to the z-axis.

The result is independent of joint angle because:
  - For a cube, Ixx = Iyy = Izz, so rotation doesn't change its Izz.
  - The distance from each COM to the z-axis is preserved by rotation about z.

  Link1: I₁ = 1*(0.1)²/6 + 1*0²    = 1/600 + 0   (d² = 0)
  Link2: I₂ = 1*(0.1)²/6 + 1*1²    = 1/600 + 1   (d² = 1)
  Link3: I₃ = 1*(0.1)²/6 + 1*√2²   = 1/600 + 2   (d² = 2)
  Total: M = 3/600 + 3 = 1/200 + 3 = 3.005 kg·m² */
GTEST_TEST(CompositeTest, CompositeSpatialInertia) {
  const TestModel explicit_model = MakeModel(false /* no combining */);
  const TestModel composite_model = MakeModel(true /* combine welds */);

  // The mass matrix is configuration-independent for this model (see above),
  // but we check at several angles to guard against future changes.
  const std::vector<double> angles = {0.0, M_PI / 6, M_PI / 4, -M_PI / 3};

  for (double angle : angles) {
    SetState(explicit_model, angle, 0.0);
    SetState(composite_model, angle, 0.0);

    MatrixX<double> M_explicit(1, 1), M_composite(1, 1);
    explicit_model.plant->CalcMassMatrix(*explicit_model.context, &M_explicit);
    composite_model.plant->CalcMassMatrix(*composite_model.context,
                                          &M_composite);

    EXPECT_TRUE(CompareMatrices(M_explicit, M_composite, kTolerance,
                                MatrixCompareType::relative))
        << "Mass matrix mismatch at angle=" << angle;

    // Verify the analytical value.
    const double a = 0.1;                     // cube side
    const double m = 1.0;                     // mass per link
    const double I_cube_z = m * a * a / 6.0;  // Izz of one cube about its COM
    const double M_expected = (I_cube_z + m * 0.0) +  // Link1: d² = 0
                              (I_cube_z + m * 1.0) +  // Link2: d² = 1² = 1
                              (I_cube_z + m * 2.0);   // Link3: d² = √2² = 2
    EXPECT_NEAR(M_composite(0, 0), M_expected, kTolerance)
        << "Mass matrix analytical mismatch at angle=" << angle;
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
