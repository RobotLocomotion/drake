#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

class LinearBushingRollPitchYawTester : public ::testing::Test {
 public:
  void SetUp() override {
    // Create an empty model.
    auto model = std::make_unique<MultibodyTree<double>>();

    // In preparation to add a rigid body B to the model, form a spatial inertia
    // containing B's mass and inertia properties about Bcm, expressed in B.
    const RotationalInertia<double> I_BBcm_B(Ixx_, Iyy_, Izz_,
                                             Ixy_, Ixz_, Iyz_);
    const SpatialInertia<double> M_BBo_B =
        SpatialInertia<double>::MakeFromCentralInertia(mB_, p_BoBcm_, I_BBcm_B);

    bodyA_ = &(model->world_body());
    bodyB_ = &(model->AddRigidBody("BodyB", M_BBo_B));

    // Calculate "reasonable" torque and force stiffness and damping constants,
    // where "reasonable" means a critical damping ratio ζ = 0.1 and damped
    // natural periods of a few seconds.
    Vector3<double> torque_stiffness_constants;
    Vector3<double> torque_damping_constants;
    Vector3<double> force_stiffness_constants;
    Vector3<double> force_damping_constants;
    CalcStiffnessAndDampingConstants(&torque_stiffness_constants,
                                     &torque_damping_constants,
                                     &force_stiffness_constants,
                                     &force_damping_constants);

    // Form frameAb, the frame Aʙ of body `A` that connects to the bushing.
    // Form frameBa, the frame Bᴀ of body `B` that connects to the bushing.
    const Frame<double>& frameAb = bodyA_->body_frame();
    const Frame<double>& frameBa = bodyB_->body_frame();

    // Allow relative motion between bodies A and B by adding a mobilizer
    // between frame Aʙ of body A and frame Bᴀ of body B.
    mobilizer_ = &model->AddMobilizer(
       std::make_unique<QuaternionFloatingMobilizer<double>>(frameAb, frameBa));

    // Add a bushing force element between frame Aʙ of body A and frame Bᴀ of B.
    bushing_ = &(model->AddForceElement<LinearBushingRollPitchYaw>(
                              frameAb, frameBa,
                              torque_stiffness_constants,
                              torque_damping_constants,
                              force_stiffness_constants,
                              force_damping_constants));

    // By default, this model has no uniform gravitational force.
    const Vector3<double> gravity(0, 0, 0);
    model->mutable_gravity_field().set_gravity_vector(gravity);

    // Finalize the model and create a default context for this system.
    model->Finalize();
    mbtree_system_ =
        std::make_unique<MultibodyTreeSystem<double>>(std::move(model));
    context_ = mbtree_system_->CreateDefaultContext();
  }

 protected:
  std::unique_ptr<MultibodyTreeSystem<double>> mbtree_system_;
  std::unique_ptr<systems::Context<double>> context_;
  const QuaternionFloatingMobilizer<double>* mobilizer_{nullptr};

  const RigidBody<double>* bodyA_{nullptr};  // World body (inboard to bushing).
  const RigidBody<double>* bodyB_{nullptr};  // Body attached to bushing.
  const LinearBushingRollPitchYaw<double>* bushing_{nullptr};

  // Mass and inertia parameters for this model.
  const double mB_ = 1;                       // B's mass in kilograms (kg).
  const double Ixx_ = 2, Iyy_ = 3, Izz_ = 4;  // B's moments  of inertia (m²).
  const double Ixy_ = 0, Ixz_ = 0, Iyz_ = 0;  // B's products of inertia (m²).
  const Vector3<double> p_BoBcm_{0, 0, 0};    // Position from Bo to Bcm (m).

 private:
  // Calculates "reasonable" torque and force stiffness and damping constants,
  // where "reasonable" means a critical damping ratio ζ = 0.1 and damped
  // natural periods of a few seconds.
  // @param[out] torque_stiffness_constants For torque τ, the stiffness
  //   constants `[k₀, k₁, k₂]` associated with angles `[q₀, q₁, q₂]`.
  // @param[out] torque_damping_constants For torque τ, the damping
  //   constants `[b₀, b₁, b₂]` associated with angular rates `[q̇₀, q̇₁, q̇₂]`.
  // @param[out] force_stiffness_constants For force f, the stiffness constants
  //   `[kx, ky, kz]` associated with translational displacement `[x, y, z]`
  // @param[out] force_damping_constants For force f, the damping constants
  //   `[bx, by, bz]` associated with translational rates `[ẋ, ẏ, ż]`.
  // @see linear_bushing_roll_pitch_yaw.h for definitions of k₀, b₀, q₀, x, etc.
  void CalcStiffnessAndDampingConstants(
      Vector3<double>* torque_stiffness_constants,
      Vector3<double>* torque_damping_constants,
      Vector3<double>* force_stiffness_constants,
      Vector3<double>* force_damping_constants) const {
    DRAKE_ASSERT(torque_stiffness_constants != nullptr);
    DRAKE_ASSERT(torque_damping_constants != nullptr);
    DRAKE_ASSERT(force_stiffness_constants != nullptr);
    DRAKE_ASSERT(force_damping_constants != nullptr);
    // For certain small rotations, the natural rotational frequencies are
    // ωₙ₀ ≈ sqrt(k₀ / Ixx),  ωₙ₁ ≈ sqrt(k₁ / Iyy),  ωₙ₂ ≈ sqrt(k₂ / Izz).
    // Hence, k₀ = Ixx * ωₙ₀²,  k₁ = Iyy * ωₙ₁²,  k₂ = Izz * ωₙ₂².
    // The natural frequencies associated with translation are
    // ωₙx ≈ sqrt(kx / m),  ωₙy ≈ sqrt(ky / m),  ωₙz ≈ sqrt(kz / m).
    // Hence, kx = m * ωₙx²,  ky = m * ωₙy²,  kz = m * ωₙz².
    //
    // For this analysis, a critical damping ratio ζ = 0.1 was chosen with a
    // damped period of vibration τᴅᴀᴍᴩ ≈ 1 second for rotational motions and
    // damped periods of translation vibration of τx ≈ 2 s, τy ≈ 3 s, τz ≈ 4 s.
    // Use τᴅᴀᴍᴩ = 2π/ωᴅᴀᴍᴩ, ωᴅᴀᴍᴩ = ωₙ√(1-ζ²) to find ωₙ = ωᴅᴀᴍᴩ / √(1-ζ²).
    // Since ζ = b /(2 √(m k)), the associated damping constant b = 2 ζ √(m k).
    // ------------ Torque stiffness and damping calculations ------------
    const double tau_rotate = 1;                                 // seconds
    const double wDamped_rotate = 2 * M_PI / tau_rotate;         // rad/sec
    const double zeta = 0.1;                                     // No units
    const double wn_rotate = wDamped_rotate / std::sqrt(1 - zeta * zeta);
    const double k0 = Ixx_ * wn_rotate * wn_rotate;              // N*m/rad
    const double k1 = Iyy_ * wn_rotate * wn_rotate;              // N*m/rad
    const double k2 = Izz_ * wn_rotate * wn_rotate;              // N*m/rad
    const double b0 = 2 * zeta * std::sqrt(Ixx_ * k0);           // N*m*s/rad
    const double b1 = 2 * zeta * std::sqrt(Iyy_ * k1);           // N*m*s/rad
    const double b2 = 2 * zeta * std::sqrt(Izz_ * k2);           // N*m*s/rad
    *torque_stiffness_constants = Vector3<double>(k0, k1, k2);
    *torque_damping_constants = Vector3<double>(b0, b1, b2);

    // ------------ Force stiffness and damping calculations -------------
    const double tauX = 2, tauY = 3, tauZ = 3;                   // seconds
    const double wDampedX = 2 * M_PI / tauX;                     // rad/sec
    const double wDampedY = 2 * M_PI / tauY;                     // rad/sec
    const double wDampedZ = 2 * M_PI / tauZ;                     // rad/sec
    const double wnX = wDampedX / std::sqrt(1 - zeta * zeta);    // rad/sec
    const double wnY = wDampedY / std::sqrt(1 - zeta * zeta);    // rad/sec
    const double wnZ = wDampedZ / std::sqrt(1 - zeta * zeta);    // rad/sec
    const double kx = mB_ * wnX * wnX;                           // N/m
    const double ky = mB_ * wnY * wnY;                           // N/m
    const double kz = mB_ * wnZ * wnZ;                           // N/m
    const double bx = 2 * zeta * std::sqrt(mB_ * kx);            // N*s/m
    const double by = 2 * zeta * std::sqrt(mB_ * ky);            // N*s/m
    const double bz = 2 * zeta * std::sqrt(mB_ * kz);            // N*s/m
    *force_stiffness_constants = Vector3<double>(kx, ky, kz);
    *force_damping_constants = Vector3<double>(bx, by, bz);
  }
};

TEST_F(LinearBushingRollPitchYawTester, ConstructionAndAccessors) {
  EXPECT_EQ(bushing_->bodyA().index(), bodyA_->index());
  EXPECT_EQ(bushing_->bodyB().index(), bodyB_->index());
}

// Verify the torques, force, potential energy, and power calculations are 0
// when body B has an identity pose and zero motion in world W.
TEST_F(LinearBushingRollPitchYawTester, BaselineIdentityPoseAtRest) {
  systems::Context<double>& context = *(context_.get());

  const math::RigidTransform<double> X_AbBa =
      bushing_->CalcBushingRigidTransform(context);
  EXPECT_TRUE(X_AbBa.IsExactlyIdentity());

  const math::RollPitchYaw<double> rpy =
      bushing_->CalcBushingRollPitchYawAngles(context);
  EXPECT_EQ(rpy.vector(), Vector3<double>::Zero());

  const SpatialVelocity<double> V_AbBa =
      bushing_->CalcBushingSpatialVelocity(context);
  EXPECT_EQ(V_AbBa.get_coeffs(), SpatialVelocity<double>::Zero().get_coeffs());

  const SpatialForce<double> F_Ab_Ab =
      bushing_->CalcBushingSpatialForceOnAb(context);
  EXPECT_EQ(F_Ab_Ab.get_coeffs(), SpatialVelocity<double>::Zero().get_coeffs());

  // TODO(Mitiguy) Fix CalcPotentialEnergy() to avoid need to form pc.
  //  Similarly, fix CalcConservativePower() to avoid need for pc and vc, etc.
  const MultibodyTree<double>& mbt = GetInternalTree(*mbtree_system_);
  const PositionKinematicsCache<double>& pc =
      mbt.EvalPositionKinematics(context);
  const VelocityKinematicsCache<double>& vc =
      mbt.EvalVelocityKinematics(context);

  const double potential_energy = bushing_->CalcPotentialEnergy(context, pc);
  EXPECT_EQ(potential_energy, 0);

  const double conservative_power =
      bushing_->CalcConservativePower(context, pc, vc);
  EXPECT_EQ(conservative_power, 0);

  const double non_conservative_power =
    bushing_->CalcNonConservativePower(context, pc, vc);
  EXPECT_EQ(non_conservative_power, 0);
}


// Verify the torques, force, potential energy, and power calculations
// when body B has a non-identity pose and zero motion in world W.
TEST_F(LinearBushingRollPitchYawTester, NonIdentityPoseAtRest) {
  systems::Context<double>* context = context_.get();

  // Use the mobilizer to set body B's orientation in World.
  const double q0 = M_PI / 6;  // q0 = SpaceXYZ roll angle (rotation about X).
  const math::RollPitchYaw<double> rpy(q0, 0, 0);
  const math::RotationMatrix<double> R_WB(rpy);
  mobilizer_->SetFromRotationMatrix(context, R_WB.matrix());

  // Use the mobilizer to set Bo's position from Wo expressed in world W.
  const Vector3<double> p_WoBo_W(1.0, 2.0, 3.0);
  mobilizer_->set_position(context, p_WoBo_W);

  // Check that the mobilizer sets the bushing's configuration as expected.
  const math::RigidTransform<double> X_AbBa =
    bushing_->CalcBushingRigidTransform(*context);
  const math::RigidTransform<double> X_AbBa_expected(R_WB, p_WoBo_W);
  EXPECT_TRUE(X_AbBa.IsNearlyEqualTo(X_AbBa_expected, 2 * kEpsilon));

  // Ensure CalcBushingRollPitchYawAngles() gives the expected results.
  const math::RollPitchYaw<double> rpy_expected =
    bushing_->CalcBushingRollPitchYawAngles(*context);
  EXPECT_TRUE(rpy.IsNearlySameOrientation(rpy_expected, 2 * kEpsilon));

  // Check that setting the mobilizer's pose did not change the motion.
  const SpatialVelocity<double> V_AbBa =
      bushing_->CalcBushingSpatialVelocity(*context);
  EXPECT_EQ(V_AbBa.get_coeffs(), SpatialVelocity<double>::Zero().get_coeffs());

  // Check that the bushing torque depends only on pose (not motion).
  const SpatialForce<double> F_Ab_Ab =
    bushing_->CalcBushingSpatialForceOnAb(*context);
  const Vector3<double> k012 = bushing_->torque_stiffness_constants();
  const double k0_q0 = k012(0) * q0;

#if 0
  const Vector3<double> t_Ab_Ab_expected(k0_q0, 0, 0);
  EXPECT_TRUE(CompareMatrices(F_Ab_Ab.rotational(), t_Ab_Ab_expected,
                              2 * kEpsilon, MatrixCompareType::relative));
#endif

  // Check that the bushing force depends only on pose (not motion).
  const Vector3<double> kxyz = bushing_->force_stiffness_constants();
  const Vector3<double> xyz = X_AbBa.translation();
  const Vector3<double> f_Abo_Ab_expected = kxyz.cwiseProduct(xyz);
  EXPECT_TRUE(CompareMatrices(F_Ab_Ab.translational(), f_Abo_Ab_expected,
                              2 * kEpsilon, MatrixCompareType::relative));

  // TODO(Mitiguy) Try to avoid forming position/velocity cache.
  const MultibodyTree<double>& mbt = GetInternalTree(*mbtree_system_);
  const PositionKinematicsCache<double>& pc =
      mbt.EvalPositionKinematics(*context);
  const VelocityKinematicsCache<double>& vc =
      mbt.EvalVelocityKinematics(*context);

  const double potential_energy = bushing_->CalcPotentialEnergy(*context, pc);
  const double force_potential_energy_expected =
      0.5 * kxyz.dot(xyz.cwiseProduct(xyz));
  const double torque_potential_energy_expected = 0.5 * k0_q0 * q0;
  const double potential_energy_expected = force_potential_energy_expected
                                         + torque_potential_energy_expected;
  const double scaled_epsilon = 2 * kEpsilon * potential_energy_expected;
  EXPECT_NEAR(potential_energy, potential_energy_expected, scaled_epsilon);

  const double conservative_power =
      bushing_->CalcConservativePower(*context, pc, vc);
  EXPECT_EQ(conservative_power, 0);

  const double non_conservative_power =
      bushing_->CalcNonConservativePower(*context, pc, vc);
  EXPECT_EQ(non_conservative_power, 0);
}


// Verify the torques, force, potential energy, and power calculations
// when body B has an identity pose and non-zero motion in world W.
TEST_F(LinearBushingRollPitchYawTester, IdentityPoseButMoving) {
  systems::Context<double>* context = context_.get();

  // Use the mobilizer to set body B's orientation in World.
  // Use the mobilizer to set Bo's position from Wo expressed in world W.
  const math::RotationMatrix<double> R_WB;  // Identity rotation matrix.
  mobilizer_->SetFromRotationMatrix(context, R_WB.matrix());
  mobilizer_->set_position(context, Vector3<double>::Zero());

  // Check that the mobilizer sets the bushing's configuration as expected.
  const math::RigidTransform<double> X_AbBa =
      bushing_->CalcBushingRigidTransform(*context);
  EXPECT_TRUE(X_AbBa.IsExactlyIdentity());

  // Use the mobilizer to set B's angular velocity in world W expressed in W.
  // Use the mobilizer to set Bo's translational velocity in W expressed in W.
  const Vector3<double> w_WB_W(0.1, 0, 0);
  const Vector3<double> v_WBo_W(0.4, 0.5, 0.6);
  mobilizer_->set_angular_velocity(context, w_WB_W);
  mobilizer_->set_translational_velocity(context, v_WBo_W);

  // Check that the mobilizer sets the bushing's motion as expected.
  const SpatialVelocity<double> V_AbBa =
    bushing_->CalcBushingSpatialVelocity(*context);
  EXPECT_EQ(V_AbBa.rotational(), w_WB_W);
  EXPECT_EQ(V_AbBa.translational(), v_WBo_W);

  // Check that the bushing torque depends only on motion (not pose).
  const SpatialForce<double> F_Ab_Ab =
      bushing_->CalcBushingSpatialForceOnAb(*context);
  const Vector3<double> b012 = bushing_->torque_damping_constants();
  const Vector3<double> q012Dt =
      bushing_->CalcBushingRollPitchYawAngleRates(*context);
  const double b0_q0Dt = b012(0) * q012Dt(0);
  const Vector3<double> t_Ab_Ab_expected(b0_q0Dt, 0, 0);
  EXPECT_TRUE(CompareMatrices(F_Ab_Ab.rotational(), t_Ab_Ab_expected,
                              2 * kEpsilon, MatrixCompareType::relative));

  // Check that the bushing force depends only on pose (not motion).
  const Vector3<double> bxyz = bushing_->force_damping_constants();
  const Vector3<double> xyzDt = V_AbBa.translational();
  const Vector3<double> f_Abo_Ab_expected = bxyz.cwiseProduct(xyzDt);
  EXPECT_TRUE(CompareMatrices(F_Ab_Ab.translational(), f_Abo_Ab_expected,
                              2 * kEpsilon, MatrixCompareType::relative));

  const MultibodyTree<double>& mbt = GetInternalTree(*mbtree_system_);
  const PositionKinematicsCache<double>& pc =
      mbt.EvalPositionKinematics(*context);
  const VelocityKinematicsCache<double>& vc =
      mbt.EvalVelocityKinematics(*context);

  const double potential_energy = bushing_->CalcPotentialEnergy(*context, pc);
  EXPECT_EQ(potential_energy, 0);

  const double conservative_power =
      bushing_->CalcConservativePower(*context, pc, vc);
  EXPECT_EQ(conservative_power, 0);

  const double non_conservative_power =
      bushing_->CalcNonConservativePower(*context, pc, vc);
  const double non_conservative_power_expected = -(F_Ab_Ab.dot(V_AbBa));
  const double scaled_epsilon = -2 * kEpsilon * non_conservative_power_expected;
  EXPECT_NEAR(non_conservative_power, non_conservative_power_expected,
              scaled_epsilon);
}


}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
