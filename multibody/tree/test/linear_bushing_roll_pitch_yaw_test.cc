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

// constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

class LinearBushingRollPitchYawTester : public ::testing::Test {
 public:
  void SetUp() override {
    // Create an empty model.
    auto model = std::make_unique<MultibodyTree<double>>();

    // In preparation to add a rigid body B to the model, form a spatial inertia
    // containing B's mass and inertia properties about Bcm, expressed in B.
    const UnitInertia<double> G_BBcm_B(Ixx_, Iyy_, Izz_, Ixy_, Ixz_, Iyz_);
    const SpatialInertia<double> M_BBcm_B(mB_, p_BoBcm_, G_BBcm_B);

    bodyA_ = &(model->world_body());
    bodyB_ = &(model->AddRigidBody("BodyB", M_BBcm_B));

    // Calculate rotational and translational stiffness and damping.
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

    std::unique_ptr<QuaternionFloatingMobilizer<double>> mobilizer =
        std::make_unique<QuaternionFloatingMobilizer<double>>(frameAb, frameBa);
    model->AddMobilizer(std::move(mobilizer));

    // Add a bushing force element to the plane.
    bushing_ = &(model->AddForceElement<LinearBushingRollPitchYaw>(
                              frameAb, frameBa,
                              torque_stiffness_constants,
                              torque_damping_constants,
                              force_stiffness_constants,
                              force_damping_constants));

    // Model with zero uniform gravitational force on each massive object.
    const Vector3<double> gravity(0, 0, 0);
    model->mutable_gravity_field().set_gravity_vector(gravity);

    // Modeling is done.  Create a context for this system.
    model->Finalize();
    mbtree_system_ =
        std::make_unique<MultibodyTreeSystem<double>>(std::move(model));
    context_ = mbtree_system_->CreateDefaultContext();
  }

 protected:
  std::unique_ptr<MultibodyTreeSystem<double>> mbtree_system_;
  std::unique_ptr<systems::Context<double>> context_;

  const RigidBody<double>* bodyA_{nullptr};  // World body (inboard to bushing).
  const RigidBody<double>* bodyB_{nullptr};  // Rigid body attached to bushing.
  const LinearBushingRollPitchYaw<double>* bushing_{nullptr};

  // Mass and inertia parameters for this model.
  const double mB_ = 1;                       // B's mass in kilograms (kg)
  const double Ixx_ = 2, Iyy_ = 3, Izz_ = 4;  // Unit moments  of inertia (m²)
  const double Ixy_ = 0, Ixz_ = 0, Iyz_ = 0;  // Unit products of inertia (m²)
  const Vector3<double> p_BoBcm_{0, 0, 0};    // Position from Bo to Bcm (m)

 private:
  // @param[out] torque_stiffness_constants For torque τ, the stiffness
  //   constants `[k₀, k₁, k₂]` associated with angles `[q₀, q₁, q₂]`.
  // @param[out] torque_damping_constants For torque τ, the damping
  //   constants `[b₀, b₁, b₂]` associated with angular rates `[q̇₀, q̇₁, q̇₂]`.
  // @param[out] force_stiffness_constants For force f, the stiffness constants
  //   `[kx, ky, kz]` associated with translational displacement `[x, y, z]`
  // @param[out] force_damping_constants For force f, the damping constants
  //   `[bx, by, bz]` associated with translational rates `[ẋ, ẏ, ż]`.
  // @note For small rotations, the natural rotational frequencies are
  // ωₙ₀ ≈ sqrt(k0 / Ixx),  ωₙ₁ ≈ sqrt(k1 / Iyy),  ωₙ₂ ≈ sqrt(k2 / Izz).
  // The natural frequencies associated with translation are
  // ωₙx ≈ sqrt(kx / m),  ωₙy ≈ sqrt(ky / m),  ωₙz ≈ sqrt(kz / m).
  // For this analysis, a critical damping ratio ζ = 0.1 was chosen with a
  // damped period of vibration τᴅᴀᴍᴩ ≈ 1 second for rotational motions and
  // damped periods of translation vibration of τx ≈ 2 s, τy ≈ 3 s, τz ≈ 4 s.
  // Since τᴅᴀᴍᴩ = 2π/(ωₙ √(1-ζ²)) the associated stiffness is calculated as
  // ωₙ = 2π/(τᴅᴀᴍᴩ √(1-ζ²)).  Since ζ = b /(2 √(m k)), the associated damping
  // constant is calculated as b = 2 ζ √(m k).
  // @see the class documentation in linear_bushing_roll_pitch_yaw.h.
  void CalcStiffnessAndDampingConstants(
      Vector3<double>* torque_stiffness_constants,
      Vector3<double>* torque_damping_constants,
      Vector3<double>* force_stiffness_constants,
      Vector3<double>* force_damping_constants) const {
    DRAKE_ASSERT(torque_stiffness_constants != nullptr);
    DRAKE_ASSERT(torque_damping_constants != nullptr);
    DRAKE_ASSERT(force_stiffness_constants != nullptr);
    DRAKE_ASSERT(force_damping_constants != nullptr);

    // --------- Rotational stiffness and damping calculations ------------
    const double tauRotate = 1;                                  // seconds
    const double zeta = 0.1;                                     // No units
    const double z1 = 2 * M_PI / std::sqrt(1 - zeta * zeta);     // No units
    const double wnRotate = z1 / tauRotate;                      // rad/sec
    const double k0 = Ixx_ * wnRotate * wnRotate;                // N*m/rad
    const double k1 = Iyy_ * wnRotate * wnRotate;                // N*m/rad
    const double k2 = Izz_ * wnRotate * wnRotate;                // N*m/rad
    const double b0 = 2 * zeta * std::sqrt(Ixx_ * k0);           // N*m*s/rad
    const double b1 = 2 * zeta * std::sqrt(Iyy_ * k1);           // N*m*s/rad
    const double b2 = 2 * zeta * std::sqrt(Izz_ * k2);           // N*m*s/rad
    *torque_stiffness_constants = Vector3<double>(k0, k1, k2);
    *torque_damping_constants = Vector3<double>(b0, b1, b2);

    // --------- Translational stiffness and damping calculations --------
    const double tauX = 2, tauY = 3, tauZ = 3;                       // seconds
    const double wnX = z1 / tauX, wnY = z1 / tauY, wnZ = z1 / tauZ;  // rad/sec
    const double kx = mB_ * wnX * wnX;                               // N/m
    const double ky = mB_ * wnY * wnY;                               // N/m
    const double kz = mB_ * wnZ * wnZ;                               // N/m
    const double bx = 2 * zeta * std::sqrt(mB_ * kx);                // N*s/m
    const double by = 2 * zeta * std::sqrt(mB_ * ky);                // N*s/m
    const double bz = 2 * zeta * std::sqrt(mB_ * kz);                // N*s/m
    *force_stiffness_constants = Vector3<double>(kx, ky, kz);
    *force_damping_constants = Vector3<double>(bx, by, bz);
  }

  // @param[out] solid_block_dimensions, the dimensions of the uniform-density
  //   solid block whose mass and inertia properties correspond to the ones
  //   specified in this model.
  void CalcUniformDensitySolidBlockDimensions(
      Vector3<double>* solid_block_dimensions) const {
    DRAKE_ASSERT(solid_block_dimensions != nullptr);
    const double z = std::sqrt((Ixx_ + Iyy_ - Izz_) * 6 / mB_);
    const double y = std::sqrt((Ixx_ + Izz_ - Iyy_) * 6 / mB_);
    const double x = std::sqrt((Iyy_ + Izz_ - Ixx_) * 6 / mB_);
    *solid_block_dimensions = Vector3<double>(x, y, z);
  }
};

TEST_F(LinearBushingRollPitchYawTester, ConstructionAndAccessors) {
  EXPECT_EQ(bushing_->bodyA().index(), bodyA_->index());
  EXPECT_EQ(bushing_->bodyB().index(), bodyB_->index());
}

// Verify that the model's baseline state (pose and motion) are correct and
// correspond to zero torques, zero force, zero potential energy, zero power.
TEST_F(LinearBushingRollPitchYawTester, BaselinePoseAtRest) {
  const systems::Context<double>& context = *(context_.get());

  const math::RigidTransformd X_AbBa =
      bushing_->CalcBushingRigidTransform(context);
  EXPECT_TRUE(X_AbBa.IsExactlyIdentity());

  const math::RollPitchYaw<double> rpy =
      bushing_->CalcBushingRollPitchYawAngles(context);
  EXPECT_EQ(rpy.vector(), Vector3<double>::Zero());

  const SpatialVelocity<double> V_AbBa =
      bushing_->CalcBushingSpatialVelocity(context);
  EXPECT_EQ(V_AbBa.get_coeffs(), SpatialVelocity<double>::Zero().get_coeffs());

  const SpatialForce<double> F_Ab_Ab =
      bushing_->CalcBushingResultantSpatialForceOnAbo(context);
  EXPECT_EQ(F_Ab_Ab.get_coeffs(), SpatialVelocity<double>::Zero().get_coeffs());

  // TODO(Mitiguy) Try to avoid forming position/velocity cache.
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


}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
