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

    // Calculate "reasonable" torque and force stiffness/damping constants,
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


  // Compares the Drake results whose those calculated by MotionGenesis.
  // @param[in] rpy roll, pitch, yaw angles relating frames Aʙ and Bᴀ.
  // @param[in] w_AbBa frame Bᴀ's angular velocity in frame Aʙ, expressed in Aʙ.
  // @param[in] p_AbBa position vector from Aʙₒ to Bᴀₒ, expressed in frame Aʙ.
  // @param[in] v_AbBa point Bᴀₒ's velocity in frame Aʙ, expressed in frame Aʙ.
  // @see linear_bushing_roll_pitch_yaw.h for definitions of q₀, x, ẋ, etc.
  void CompareToMotionGenesisResults(const math::RollPitchYaw<double>& rpy,
                                     const Vector3<double>& w_AbBa,
                                     const Vector3<double>& p_AbBa,
                                     const Vector3<double>& v_AbBa) const {
    //-------------- Start of MotionGenesis calculations -------------
    // Get the bushing's torque and force stiffness/damping constants.
    const Vector3<double>& k012 = bushing_->torque_stiffness_constants();
    const Vector3<double>& b012 = bushing_->torque_damping_constants();
    const Vector3<double>& kxyz = bushing_->force_stiffness_constants();
    const Vector3<double>& bxyz = bushing_->force_damping_constants();
    const double b0 = b012(0), b1 = b012(1), b2 = b012(2);  // N*m*s/rad
    const double bx = bxyz(0), by = bxyz(1), bz = bxyz(2);  // N*s/m
    const double k0 = k012(0), k1 = k012(1), k2 = k012(2);  // N*m/rad
    const double kx = kxyz(0), ky = kxyz(1), kz = kxyz(2);  // N/m

    // Get the bushing's configuration and motion.
    const double q0 = rpy.roll_angle();                              // rad
    const double q1 = rpy.pitch_angle();                             // rad
    const double q2 = rpy.yaw_angle();                               // rad
    const double wx = w_AbBa(0), wy = w_AbBa(1), wz = w_AbBa(2);     // rad/sec
    const double x = p_AbBa(0), y = p_AbBa(1), z = p_AbBa(2);        // m
    const double xDt = v_AbBa(0), yDt = v_AbBa(1), zDt = v_AbBa(2);  // m/s

    const double c1 = cos(q1), s1 = sin(q1), tan1 = s1 / c1;
    const double c2 = cos(q2), s2 = sin(q2);
    const double q0Dt = c2 / c1 * wx + s2 / c1 * wy;
    const double q1Dt = -s2 * wx + c2 * wy;
    const double q2Dt = c2 * tan1 * wx + s2 * tan1 * wy + wz;
    const double Fx = -(kx * x + bx * xDt);
    const double Fy = -(ky * y + by * yDt);
    const double Fz = -(kz * z + bz * zDt);
    const double T0 = -(k0 * q0 + b0 * q0Dt);
    const double T1 = -(k1 * q1 + b1 * q1Dt);
    const double T2 = -(k2 * q2 + b2 * q2Dt);
    const double Tx = c2 / c1 * T0 - s2 * T1 + c2 * tan1 * T2;
    const double Ty = s2 / c1 * T0 + c2 * T1 + s2 * tan1 * T2;
    const double Tz = T2;
    const double MBox =  Tx + 0.5*Fy*z - 0.5*Fz*y;
    const double MBoy =  Ty + 0.5*Fz*x - 0.5*Fx*z;
    const double MBoz =  Tz + 0.5*Fx*y - 0.5*Fy*x;
    const double MAox = -Tx + 0.5*Fy*z - 0.5*Fz*y;
    const double MAoy = -Ty + 0.5*Fz*x - 0.5*Fx*z;
    const double MAoz = -Tz + 0.5*Fx*y - 0.5*Fy*x;
    const double potentialTranslation = 0.5 * kx * pow(x, 2)
                                      + 0.5 * ky * pow(y, 2)
                                      + 0.5 * kz * pow(z, 2);
    const double potentialRotation = 0.5 * k0 * pow(q0, 2)
                                   + 0.5 * k1 * pow(q1, 2)
                                   + 0.5 * k2 * pow(q2, 2);
    const double potentialEnergy_MG = potentialRotation + potentialTranslation;
    const double powerConservative_MG = -k0 * q0 * q0Dt - kx * x * xDt
                                       - k1 * q1 * q1Dt - ky * y * yDt
                                       - k2 * q2 * q2Dt - kz * z * zDt;
    const double powerDissipation_MG = -b0 * pow(q0Dt, 2) - bx * pow(xDt, 2)
                                      - b1 * pow(q1Dt, 2) - by * pow(yDt, 2)
                                      - b2 * pow(q2Dt, 2) - bz * pow(zDt, 2);
    const double powerExtra_MG = 0.5*Fx*(y*wz-z*wy)
                               + 0.5*Fy*(z*wx-x*wz)
                               + 0.5*Fz*(x*wy-y*wx);
    const double powerNonconservative_MG = powerDissipation_MG - powerExtra_MG;
    //---------------- End of MotionGenesis calculations -------------

    // Harvest the context for Drake information.
    systems::Context<double>* context = context_.get();

    // Use the mobilizer to set frame Bᴀ's orientation in frame Aʙ.
    // Use the mobilizer to set Bᴀo's position from Aʙo expressed in frame Aʙ.
    const math::RotationMatrix<double> R_AbBa(rpy);
    mobilizer_->SetFromRotationMatrix(context, R_AbBa);
    mobilizer_->set_position(context, p_AbBa);

    // Verify the mobilizer sets the bushing's configuration correctly.
    const math::RigidTransform<double> X_AbBa =
        bushing_->CalcBushingRigidTransform(*context);
    const math::RigidTransform<double> X_AbBa_expected(R_AbBa, p_AbBa);
    EXPECT_TRUE(X_AbBa.IsNearlyEqualTo(X_AbBa_expected, 4 * kEpsilon));

    // Ensure CalcBushingRollPitchYawAngles() gives the expected results.
    const math::RollPitchYaw<double> bushing_rpy =
        bushing_->CalcBushingRollPitchYawAngles(*context);
    EXPECT_TRUE(bushing_rpy.IsNearlySameOrientation(rpy, 4 * kEpsilon));

    // Use the mobilizer to set Bᴀ's angular velocity in Aʙ expressed in Aʙ.
    // Similarly, set Bᴀo's translational velocity in Aʙ expressed in Aʙ.
    mobilizer_->set_angular_velocity(context, w_AbBa);
    mobilizer_->set_translational_velocity(context, v_AbBa);

    // Verify the mobilizer sets the bushing's motion correctly.
    const SpatialVelocity<double> V_AbBa =
        bushing_->CalcBushingSpatialVelocity(*context);
    EXPECT_EQ(V_AbBa.rotational(), w_AbBa);
    EXPECT_EQ(V_AbBa.translational(), v_AbBa);

    // Verify the bushing's spatial force (torque and force) calculation at Bc.
    const SpatialForce<double> F_Bc_Ab =
        bushing_->CalcBushingSpatialForceOnBc(*context);
    const Vector3<double> t_Bc_Ab = F_Bc_Ab.rotational();
    const Vector3<double> f_Bc_Ab = F_Bc_Ab.translational();
    const Vector3<double> t_Bc_Ab_expected(Tx, Ty, Tz);
    const Vector3<double> f_Bc_Ab_expected(Fx, Fy, Fz);
    EXPECT_TRUE(CompareMatrices(t_Bc_Ab, t_Bc_Ab_expected, 4 * kEpsilon,
                                MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(f_Bc_Ab, f_Bc_Ab_expected, 4 * kEpsilon,
                                MatrixCompareType::relative));

    // Verify the bushing's spatial force (torque and force) calculation at Bᴀₒ.
    const SpatialForce<double> F_Ba_Ab =
        bushing_->CalcBushingSpatialForceOnBa(*context);
    const Vector3<double> t_Ba_Ab = F_Ba_Ab.rotational();
    const Vector3<double> f_Ba_Ab = F_Ba_Ab.translational();
    const Vector3<double> t_Ba_Ab_expected(MBox, MBoy, MBoz);
    const Vector3<double> f_Ba_Ab_expected(Fx, Fy, Fz);
    EXPECT_TRUE(CompareMatrices(t_Ba_Ab, t_Ba_Ab_expected, 4 * kEpsilon,
                                MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(f_Ba_Ab, f_Ba_Ab_expected, 4 * kEpsilon,
                                MatrixCompareType::relative));

    // Verify the bushing's spatial force (torque and force) calculation at Aʙₒ.
    const SpatialForce<double> F_Ab_Ab =
        bushing_->CalcBushingSpatialForceOnAb(*context);
    const Vector3<double> t_Ab_Ab = F_Ab_Ab.rotational();
    const Vector3<double> f_Ab_Ab = F_Ab_Ab.translational();
    const Vector3<double> t_Ab_Ab_expected(MAox, MAoy, MAoz);
    const Vector3<double> f_Ab_Ab_expected(-Fx, -Fy, -Fz);
    EXPECT_TRUE(CompareMatrices(t_Ab_Ab, t_Ab_Ab_expected, 4 * kEpsilon,
                                MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(f_Ab_Ab, f_Ab_Ab_expected, 4 * kEpsilon,
                                MatrixCompareType::relative));

    // TODO(Mitiguy) Fix CalcPotentialEnergy(), CalcConservativePower(), etc.
    //  to avoid the need to form pc and vc.
    const MultibodyTree<double>& mbt = GetInternalTree(*mbtree_system_);
    const PositionKinematicsCache<double>& pc =
        mbt.EvalPositionKinematics(*context);
    const VelocityKinematicsCache<double>& vc =
        mbt.EvalVelocityKinematics(*context);

    // Verify the bushing's potential energy calculation.
    const double potential_energy = bushing_->CalcPotentialEnergy(*context, pc);
    double scaled_epsilon = std::abs(potentialEnergy_MG) * 8 * kEpsilon;
    EXPECT_NEAR(potential_energy, potentialEnergy_MG, scaled_epsilon);

    // Verify the bushing's conservative power calculation.
    const double conservative_power =
        bushing_->CalcConservativePower(*context, pc, vc);
    scaled_epsilon = std::abs(powerConservative_MG) * 8 * kEpsilon;
    EXPECT_NEAR(conservative_power, powerConservative_MG, scaled_epsilon);

    // Verify the bushing's non-conservative power calculation.
    const double non_conservative_power =
        bushing_->CalcNonConservativePower(*context, pc, vc);
    scaled_epsilon = std::abs(powerDissipation_MG) * 8 * kEpsilon;
    EXPECT_NEAR(non_conservative_power, powerNonconservative_MG,
                scaled_epsilon);
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
  // Calculates "reasonable" torque and force stiffness/damping constants,
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
    // ------------ Calculate torque stiffness/damping constants --------------
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

    // ------------ Calculate force stiffness/damping constants ---------------
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

// Verify calculations when body B has given pose and zero motion in world W.
TEST_F(LinearBushingRollPitchYawTester, NonIdentityPoseAtRest) {
  const Vector3<double> w_AbBa(0, 0, 0);
  const Vector3<double> v_AbBa(0, 0, 0);

  // Test with both zero and non-zero position vectors.
  const Vector3<double> p_AbBa(1, 2, 3);
  const Vector3<double> p_zero(0, 0, 0);

  // Test identity orientation.
  math::RollPitchYaw<double> rpy(0, 0, 0);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_zero, v_AbBa);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_AbBa, v_AbBa);

  // Test for non-zero roll only.
  rpy.set(M_PI / 6, 0, 0);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_zero, v_AbBa);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_AbBa, v_AbBa);

  // Test for non-zero pitch only.
  rpy.set(0, M_PI / 3, 0);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_zero, v_AbBa);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_AbBa, v_AbBa);

  // Test for non-zero yaw only.
  rpy.set(0, 0, 2 * M_PI / 3);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_zero, v_AbBa);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_AbBa, v_AbBa);

  // Test for non-zero roll, pitch, and yaw.
  rpy.set(M_PI / 6, M_PI / 3, 2 * M_PI / 3);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_zero, v_AbBa);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_AbBa, v_AbBa);
}

// Verify calculations when body B has identity pose and motion in world W.
TEST_F(LinearBushingRollPitchYawTester, IdentityPoseButMoving) {
  const math::RollPitchYaw<double> rpy(0, 0, 0);
  const Vector3<double> p_AbBa(0, 0, 0);

  // Test with both zero and non-zero angular and translational velocities.
  const Vector3<double> w_zero(0, 0, 0);
  const Vector3<double> v_zero(0, 0, 0);
  const Vector3<double> v_AbBa(3, 2, 1);

  // Test simple angular velocity (x-direction).
  Vector3<double> w_AbBa(2, 0, 0);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_AbBa, v_zero);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_AbBa, v_AbBa);

  // Test simple angular velocity (y-direction).
  w_AbBa = Vector3<double>(0, 3, 0);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_AbBa, v_zero);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_AbBa, v_AbBa);

  // Test simple angular velocity (z-direction).
  w_AbBa = Vector3<double>(0, 0, 4);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_AbBa, v_zero);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_AbBa, v_AbBa);

  // Test non-simple angular velocity.
  w_AbBa = Vector3<double>(2, 3, 4);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_AbBa, v_zero);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_AbBa, v_AbBa);
}

// Verify calculations when body B has non-zero pose/motion in world W.
TEST_F(LinearBushingRollPitchYawTester, NonIdentityPoseAndMoving) {
  const math::RollPitchYaw<double> rpy(0.1, 0.2, 0.3);
  const Vector3<double> p_AbBa(0.4, 0.5, 0.6);
  const Vector3<double> w_AbBa(0.7, 0.8, 0.9);
  const Vector3<double> v_AbBa(1.1, 1.2, 1.3);
  CompareToMotionGenesisResults(rpy, w_AbBa, p_AbBa, v_AbBa);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
