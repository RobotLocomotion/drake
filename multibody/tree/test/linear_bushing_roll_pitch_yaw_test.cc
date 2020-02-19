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
    // TODO(Mitiguy) Per issue #12739, consider using MultibodyPlant not MBT.
    auto model = std::make_unique<MultibodyTree<double>>();

    // Before adding a rigid link C to the model, form a spatial inertia that
    // contains C's mass and inertia properties about Ccm, expressed in C.
    // Note: Point Co (C's origin) is coincident with CCm (C's center of mass).
    const RotationalInertia<double> I_CCcm_C(Ixx_, Iyy_, Izz_,
                                             Ixy_, Ixz_, Iyz_);
    const SpatialInertia<double> M_CCo_C =
        SpatialInertia<double>::MakeFromCentralInertia(mC_, p_CoCcm_, I_CCcm_C);

    // Assign the two bodies (links) that are attached to the bushing.
    // Designate frameA as the frame of bodyA (world) connected to the bushing.
    // Designate frameC as the frame of bodyC (body)  connected to the bushing.
    bodyA_ = &(model->world_body());
    bodyC_ = &(model->AddRigidBody("BodyC", M_CCo_C));
    const Frame<double>& frameA = bodyA_->body_frame();
    const Frame<double>& frameC = bodyC_->body_frame();

    // Allow relative motion between bodies A and C by adding a 6-DOF mobilizer
    // between frameA of bodyA (world) and frameC of bodyC.
    mobilizer_ = &model->AddMobilizer(
        std::make_unique<QuaternionFloatingMobilizer<double>>(frameA, frameC));

    // Calculate "reasonable" torque and force stiffness/damping constants,
    // where "reasonable" means a critical damping ratio ζ = 0.1 for each of the
    // six modes and a damped natural period of a few seconds for each mode.
    Vector3<double> torque_stiffness_constants;
    Vector3<double> torque_damping_constants;
    Vector3<double> force_stiffness_constants;
    Vector3<double> force_damping_constants;
    CalcReasonableStiffnessAndDampingConstants(&torque_stiffness_constants,
                                               &torque_damping_constants,
                                               &force_stiffness_constants,
                                               &force_damping_constants);

    // Add a bushing force element between frame A and frame C.
    bushing_ = &(model->AddForceElement<LinearBushingRollPitchYaw>(
                              frameA, frameC,
                              torque_stiffness_constants,
                              torque_damping_constants,
                              force_stiffness_constants,
                              force_damping_constants));

    // By default, this model has no uniform gravitational force.
    const Vector3<double> gravity(0, 0, 0);
    model->mutable_gravity_field().set_gravity_vector(gravity);

    // Finalize the model and create a default context for this system.
    // TODO(Mitiguy) Per issue #12739, consider using MultibodyPlant not MBT.
    model->Finalize();
    mbtree_system_ =
        std::make_unique<MultibodyTreeSystem<double>>(std::move(model));
    context_ = mbtree_system_->CreateDefaultContext();
  }

  // Compare the Drake results with those calculated via by-hand/MotionGenesis.
  // @param[in] rpy roll, pitch, yaw angles relating frames A and C.
  // @param[in] p_AoCo_A position vector from Aₒ to Cₒ, expressed in frame A.
  // @param[in] w_AC_A frame C's angular velocity in frame A, expressed in A.
  // @param[in] v_ACo_A point Cₒ's velocity in frame A, expressed in frame A.
  // @param[in] f_C_C_expected_simple Expected net force from the bushing on
  //   frame C expressed in frame C -- or nullptr (not for comparison).
  // @param[in] t_Co_C_expected_simple Expected net moment from the bushing on
  //   frame C about Co expressed in frame C -- or nullptr (not for comparison).
  // @note: In some situations, it is relatively easy to calculate expected
  //   results that can be passed to this method as a non nullptr and used as a
  //   secondary check on the Drake and MotionGenesis results.
  void CompareToMotionGenesisResult(
      const math::RollPitchYaw<double>& rpy,
      const Vector3<double>& p_AoCo_A,
      const Vector3<double>& w_AC_A,
      const Vector3<double>& v_ACo_A,
      const Vector3<double>* f_C_C_expected_simple = nullptr,
      const Vector3<double>* t_Co_C_expected_simple = nullptr) const {
    const math::RotationMatrix<double> R_AC(rpy);
    const math::RotationMatrix<double> R_CA = R_AC.inverse();

    // Set the context for subsequent harvest of Drake information.
    systems::Context<double>& context = *(context_.get());

    // Use the mobilizer to set the following in Drake:
    //   frame C's orientation in frame A,
    //   Co's position from Ao expressed in frame A,
    //   C's angular velocity in A expressed in A,
    //   Co's translational velocity in A expressed in A.
    mobilizer_->SetFromRotationMatrix(&context, R_AC);
    mobilizer_->set_position(&context, p_AoCo_A);
    mobilizer_->set_angular_velocity(&context, w_AC_A);
    mobilizer_->set_translational_velocity(&context, v_ACo_A);

    // ---------- Start of by-hand and/or MotionGenesis calculations ---------
    // TODO(Mitiguy) clarify which calculations have been done by-hand or by
    //  MotionGenesis before merging this PR.
    // Get the bushing's configuration and motion.
    const double q0 = rpy.roll_angle();    // rad
    const double q1 = rpy.pitch_angle();   // rad
    const double q2 = rpy.yaw_angle();     // rad
    const double wx = w_AC_A(0);           // rad/s
    const double wy = w_AC_A(1);           // rad/s
    const double wz = w_AC_A(2);           // rad/s

    // Form kinematical ODEs that relate q̇₀, q̇₁, q̇₂ to wx, wy, wz.
    const double c1 = std::cos(q1), s1 = std::sin(q1);
    const double c2 = std::cos(q2), s2 = std::sin(q2);
    const double oneOverc1 = 1.0/c1, tan1 = s1 * oneOverc1;
    const double q0Dt = c2 * oneOverc1 * wx + s2 * oneOverc1 * wy;
    const double q1Dt = -s2 * wx + c2 * wy;
    const double q2Dt = c2 * tan1 * wx + s2 * tan1 * wy + wz;

    // Frame B is oriented "half-way" between frames A and C.
    // One way to determine B's orientation relative to A is to first determine
    // angleAxis_AC (the angle-axis representation of C's orientation in A) and
    // then use `0.5 * angle_AC` along with axis_AC to form the R_AB rotation
    // matrix that relates frames B and C.  This way of calculating R_AB differs
    // from the calculation in the class LinearBushingRollPitchYaw in that the
    // calculation here is relatively straightforward, the other more efficient.
    const Eigen::AngleAxis<double> angleAxis_AC = R_AC.ToAngleAxis();
    const Vector3<double>& axis_AC = angleAxis_AC.axis();
    const double& angle_AC = angleAxis_AC.angle();
    const Eigen::AngleAxis<double> angleAxis_AB(0.5 * angle_AC, axis_AC);
    const math::RotationMatrix<double> R_AB(angleAxis_AB);
    const math::RotationMatrix<double> R_BA = R_AB.inverse();
    const math::RotationMatrix<double> R_CB = (R_BA * R_AC).inverse();

    // Form x, y, z, which are defined so p_AoCo_B = x*Bx + y*By + z*Bz.
    const Vector3<double> p_AoCo_B = R_BA * p_AoCo_A;
    const double x = p_AoCo_B(0);  // meters
    const double y = p_AoCo_B(1);  // meters
    const double z = p_AoCo_B(2);  // meters

    // Form [ẋ, ẏ, ż] -- which employs the following kinematical analysis:
    // v_ACo = DtA_p_AoCo                  (definition)
    //       = DtB_p_AoCo + w_AB x p_AoCo  (Golden rule for vector derivatives)
    // DtB_p_AoCo = v_ACo − wAB x p_AoCo   (rearrange previous line).
    const Vector3<double> w_AB_A = 0.5 * w_AC_A;
    const Vector3<double> DtB_p_AoCo_A = v_ACo_A - w_AB_A.cross(p_AoCo_A);
    const Vector3<double> DtB_p_AoCo_B = R_BA * DtB_p_AoCo_A;
    const double xDt = DtB_p_AoCo_B(0);  // m/s
    const double yDt = DtB_p_AoCo_B(1);  // m/s
    const double zDt = DtB_p_AoCo_B(2);  // m/s

    // Get the bushing's torque and force stiffness/damping constants.
    const Vector3<double>& k012 = K012();
    const Vector3<double>& b012 = B012();
    const Vector3<double>& kxyz = Kxyz();
    const Vector3<double>& bxyz = Bxyz();
    const double b0 = b012(0), b1 = b012(1), b2 = b012(2);  // N*m*s/rad
    const double bx = bxyz(0), by = bxyz(1), bz = bxyz(2);  // N*s/m
    const double k0 = k012(0), k1 = k012(1), k2 = k012(2);  // N*m/rad
    const double kx = kxyz(0), ky = kxyz(1), kz = kxyz(2);  // N/m

    // Form net force fx*Bx + fy*By + fz*Bz on frameC from the bushing.
    const double fx = -(kx * x + bx * xDt);
    const double fy = -(ky * y + by * yDt);
    const double fz = -(kz * z + bz * zDt);
    const Vector3<double> f_A_B_expected(-fx, -fy, -fz);
    const Vector3<double> f_C_B_expected(fx, fy, fz);

    // Re-express the net force on frame A in frame A.
    // Re-express the net force on frame C in frame C.
    const Vector3<double> f_A_A_expected = R_AB * f_A_B_expected;
    const Vector3<double> f_C_C_expected = R_CB * f_C_B_expected;

    // Form torques T0, T1, T2 associated with roll-pitch-yaw rotation sequence.
    const double T0 = -(k0 * q0 + b0 * q0Dt);
    const double T1 = -(k1 * q1 + b1 * q1Dt);
    const double T2 = -(k2 * q2 + b2 * q2Dt);

    // Calculate the torque T = Tx*Ax + Ty*Ay + Tz*Az on C which produces the
    // same power as the spring-damper "gimbal" torques T0, T1, T2.
    const double Tx = c2 * oneOverc1 * T0 - s2 * T1 + c2 * tan1 * T2;
    const double Ty = s2 * oneOverc1 * T0 + c2 * T1 + s2 * tan1 * T2;
    const double Tz = T2;

    // Calculate the moment on frame A about Ao, expressed in A.
    const Vector3<double> p_AoBo_B = 0.5 * p_AoCo_B;
    const Vector3<double> t_Ao_A_expected = Vector3<double>(-Tx, -Ty, -Tz) +
                                         R_AB * p_AoBo_B.cross(f_A_B_expected);

    // Calculate the moment on frame C about Co, expressed in A.
    const Vector3<double> p_CoBo_B = -p_AoBo_B;
    const Vector3<double> t_Co_C_expected = R_CA * Vector3<double>(Tx, Ty, Tz)
                                       + R_CB * p_CoBo_B.cross(f_C_B_expected);

    // Contributions to potential energy.
    using std::pow;
    const double potentialTranslation = 0.5 * kx * pow(x, 2)
                                      + 0.5 * ky * pow(y, 2)
                                      + 0.5 * kz * pow(z, 2);
    const double potentialRotation = 0.5 * k0 * pow(q0, 2)
                                   + 0.5 * k1 * pow(q1, 2)
                                   + 0.5 * k2 * pow(q2, 2);
    const double potentialEnergy_MG = potentialRotation + potentialTranslation;

    // Contributions to power.
    const double powerConservative_MG = -k0 * q0 * q0Dt - kx * x * xDt
                                       - k1 * q1 * q1Dt - ky * y * yDt
                                       - k2 * q2 * q2Dt - kz * z * zDt;
    const double powerDissipation_MG = -b0 * pow(q0Dt, 2) - bx * pow(xDt, 2)
                                      - b1 * pow(q1Dt, 2) - by * pow(yDt, 2)
                                      - b2 * pow(q2Dt, 2) - bz * pow(zDt, 2);
    const Vector3<double> rCrossF_B(fz*y - fy*z,  /* Bx measure of r.cross(F) */
                                    fx*z - fz*x,  /* By measure of r.cross(F) */
                                    fy*x - fx*y); /* Bz measure of r.cross(F) */
    const Vector3<double> w_AC_B = R_BA * w_AC_A;
    const double powerExtra_MG = w_AC_B.dot(rCrossF_B);
    double powerNonconservative_MG = powerDissipation_MG + powerExtra_MG;
    // ---------- End of by-hand and/or MotionGenesis calculations ---------

    // Verify F_A_A,, the bushing's spatial force calculation on frame A.
    double torque_epsilon = 32 * kEpsilon * t_Ao_A_expected.norm();
    double force_epsilon  = 32 * kEpsilon * f_A_A_expected.norm();
    const SpatialForce<double> F_A_A =
        bushing_->CalcBushingSpatialForceOnFrameA(context);
    EXPECT_TRUE(CompareMatrices(F_A_A.rotational(), t_Ao_A_expected,
                                torque_epsilon, MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(F_A_A.translational(), f_A_A_expected,
                                force_epsilon, MatrixCompareType::relative));

    // Verify the bushing's spatial force (torque and force) calculation at Cₒ.
    const SpatialForce<double> F_C_C =
        bushing_->CalcBushingSpatialForceOnFrameC(context);
    const Vector3<double>& t_Co_C = F_C_C.rotational();
    const Vector3<double>& f_C_C = F_C_C.translational();
    EXPECT_TRUE(CompareMatrices(t_Co_C, t_Co_C_expected,
                                torque_epsilon, MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(f_C_C, f_C_C_expected,
                                force_epsilon, MatrixCompareType::relative));

    // If available, also verify the previous results with by-hand calculations.
    if (t_Co_C_expected_simple != nullptr) {
      EXPECT_TRUE(CompareMatrices(t_Co_C, *t_Co_C_expected_simple,
                                  torque_epsilon, MatrixCompareType::relative));
    }
    if (f_C_C_expected_simple != nullptr) {
      EXPECT_TRUE(CompareMatrices(f_C_C, *f_C_C_expected_simple,
                                  force_epsilon, MatrixCompareType::relative));
    }

    // TODO(Mitiguy) Fix CalcPotentialEnergy(), CalcConservativePower(), etc.
    //  to avoid the need to form pc and vc.
    const MultibodyTree<double>& mbt = GetInternalTree(*mbtree_system_);
    const PositionKinematicsCache<double>& pc =
        mbt.EvalPositionKinematics(context);
    const VelocityKinematicsCache<double>& vc =
        mbt.EvalVelocityKinematics(context);

    // Verify the bushing's potential energy calculation.
    const ForceElement<double>* bushing_force_element = bushing_;
    const double potential_energy =
        bushing_force_element->CalcPotentialEnergy(context, pc);
    double scaled_epsilon = std::abs(potentialEnergy_MG) * 32 * kEpsilon;
    EXPECT_NEAR(potential_energy, potentialEnergy_MG, scaled_epsilon);

    // Verify the bushing's conservative power calculation.
    const double conservative_power =
        bushing_force_element->CalcConservativePower(context, pc, vc);
    scaled_epsilon = std::abs(powerConservative_MG) * 32 * kEpsilon;
    EXPECT_NEAR(conservative_power, powerConservative_MG, scaled_epsilon);

    // Verify the bushing's nonconservative power calculation.
    // Note: The LinearBushingRollPitchYaw class documentation describes the
    // calculation of power (conservative/nonconservative) and potential energy.
    // ----------------------------------------------------------------------
    const double non_conservative_power =
        bushing_force_element->CalcNonConservativePower(context, pc, vc);
    scaled_epsilon = std::abs(powerDissipation_MG) * 32 * kEpsilon;
    EXPECT_NEAR(non_conservative_power, powerNonconservative_MG,
                scaled_epsilon);
  }

  // Calculates "reasonable" torque and force stiffness/damping constants,
  // where "reasonable" means a critical damping ratio ζ = 0.1 and damped
  // natural periods of a few seconds.  This method returns output via its
  // arguments, which are defined in the LinearBushingRollPitchYaw constructor.
  void CalcReasonableStiffnessAndDampingConstants(
      Vector3<double>* torque_stiffness_constants,         /* [k₀, k₁, k₂] */
      Vector3<double>* torque_damping_constants,           /* [b₀, b₁, b₂] */
      Vector3<double>* force_stiffness_constants,          /* [kx, ky, kz] */
      Vector3<double>* force_damping_constants) const {    /* [bx, by, bz] */
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
    // Use τᴅᴀᴍᴩ = 2π/ωᴅᴀᴍᴩ, ωᴅᴀᴍᴩ = ωₙ√(1−ζ²) to find ωₙ = ωᴅᴀᴍᴩ / √(1−ζ²).
    // Since ζ = b /(2 √(m k)), the associated damping constant b = 2 ζ √(m k).
    // ------------ Calculate torque stiffness/damping constants --------------
    const double tau_rotate = 1;                                 // seconds
    const double wDamped_rotate = 2 * M_PI / tau_rotate;         // rad/s
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
    const double wDampedX = 2 * M_PI / tauX;                     // rad/s
    const double wDampedY = 2 * M_PI / tauY;                     // rad/s
    const double wDampedZ = 2 * M_PI / tauZ;                     // rad/s
    const double wnX = wDampedX / std::sqrt(1 - zeta * zeta);    // rad/s
    const double wnY = wDampedY / std::sqrt(1 - zeta * zeta);    // rad/s
    const double wnZ = wDampedZ / std::sqrt(1 - zeta * zeta);    // rad/s
    const double kx = mC_ * wnX * wnX;                           // N/m
    const double ky = mC_ * wnY * wnY;                           // N/m
    const double kz = mC_ * wnZ * wnZ;                           // N/m
    const double bx = 2 * zeta * std::sqrt(mC_ * kx);            // N*s/m
    const double by = 2 * zeta * std::sqrt(mC_ * ky);            // N*s/m
    const double bz = 2 * zeta * std::sqrt(mC_ * kz);            // N*s/m
    *force_stiffness_constants = Vector3<double>(kx, ky, kz);
    *force_damping_constants = Vector3<double>(bx, by, bz);
  }

 protected:
  // TODO(Mitiguy) Per issue #12739, consider using MultibodyPlant not MBT.
  std::unique_ptr<MultibodyTreeSystem<double>> mbtree_system_;
  std::unique_ptr<systems::Context<double>> context_;
  const QuaternionFloatingMobilizer<double>* mobilizer_{nullptr};

  const RigidBody<double>* bodyA_{nullptr};  // World body (inboard to bushing).
  const RigidBody<double>* bodyC_{nullptr};  // Body attached to bushing.
  const LinearBushingRollPitchYaw<double>* bushing_{nullptr};

  // Body C's mass, center of mass, and moments/products of inertia about Ccm.
  const double mC_ = 1;                       // C's mass in kilograms (kg).
  const double Ixx_ = 2, Iyy_ = 3, Izz_ = 4;  // C's moments  of inertia (m²).
  const double Ixy_ = 0, Ixz_ = 0, Iyz_ = 0;  // C's products of inertia (m²).
  const Vector3<double> p_CoCcm_{0, 0, 0};    // Position from Co to Ccm (m).

  const Vector3<double>& K012() const {
    return bushing_->torque_stiffness_constants();
  }
  const Vector3<double>& B012() const {
    return bushing_->torque_damping_constants();
  }
  const Vector3<double>& Kxyz() const {
    return bushing_->force_stiffness_constants();
  }
  const Vector3<double>& Bxyz() const {
    return bushing_->force_damping_constants();
  }
};

TEST_F(LinearBushingRollPitchYawTester, ConstructionAndAccessors) {
  // TODO(Mitiguy) add remaining accessors before merging this PR.
  EXPECT_EQ(bushing_->link0().index(), bodyA_->index());
  EXPECT_EQ(bushing_->link1().index(), bodyC_->index());
  EXPECT_EQ(bushing_->frameA().body().index(), bodyA_->index());
  EXPECT_EQ(bushing_->frameC().body().index(), bodyC_->index());

  Vector3<double> torque_stiffness_constants;
  Vector3<double> torque_damping_constants;
  Vector3<double> force_stiffness_constants;
  Vector3<double> force_damping_constants;
  CalcReasonableStiffnessAndDampingConstants(&torque_stiffness_constants,
                                             &torque_damping_constants,
                                             &force_stiffness_constants,
                                             &force_damping_constants);
  const double tolerance = 32 * kEpsilon;
  EXPECT_TRUE(CompareMatrices(torque_stiffness_constants,
                              bushing_->torque_stiffness_constants(),
                              tolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(torque_damping_constants,
                              bushing_->torque_damping_constants(),
                              tolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(force_stiffness_constants,
                              bushing_->force_stiffness_constants(),
                              tolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(force_damping_constants,
                              bushing_->force_damping_constants(),
                              tolerance, MatrixCompareType::relative));
}

// Verify results when body C has given orientation, but no motion in world.
TEST_F(LinearBushingRollPitchYawTester, VariousOrientationsAtRest) {
  const Vector3<double> p_zero(0, 0, 0);
  const Vector3<double> w_zero(0, 0, 0);
  const Vector3<double> v_zero(0, 0, 0);

  // Generic values for roll, pitch, and yaw angles.
  const double roll_angle = M_PI / 6.0;
  const double pitch_angle = M_PI / 3.0;
  const double yaw_angle = 2 * M_PI / 3.0;
  const double k0_roll = -K012()(0) * roll_angle;
  const double k1_pitch = -K012()(1) * pitch_angle;
  const double k2_yaw = -K012()(2) * yaw_angle;

  // Test identity orientation.
  math::RollPitchYaw<double> rpy(0, 0, 0);
  Vector3<double> f_C_C_expected = Vector3<double>(0, 0, 0);
  Vector3<double> t_Co_C_expected = Vector3<double>(0, 0, 0);
  CompareToMotionGenesisResult(rpy, p_zero, w_zero, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test for non-zero roll only.
  rpy.set(roll_angle, 0, 0);
  t_Co_C_expected = Vector3<double>(k0_roll, 0, 0);
  CompareToMotionGenesisResult(rpy, p_zero, w_zero, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test for non-zero pitch only.
  rpy.set(0, pitch_angle, 0);
  t_Co_C_expected = Vector3<double>(0, k1_pitch, 0);
  CompareToMotionGenesisResult(rpy, p_zero, w_zero, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test for non-zero yaw only.
  rpy.set(0, 0, yaw_angle);
  t_Co_C_expected = Vector3<double>(0, 0, k2_yaw);
  CompareToMotionGenesisResult(rpy, p_zero, w_zero, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test with roll ≠ 0, pitch ≠ 0, yaw = 0.
  rpy.set(roll_angle, pitch_angle, 0);
  CompareToMotionGenesisResult(rpy, p_zero, w_zero, v_zero);

  // Test with roll = 0, pitch ≠ 0, yaw ≠ 0.
  rpy.set(0, pitch_angle, yaw_angle);
  CompareToMotionGenesisResult(rpy, p_zero, w_zero, v_zero);

  // Test with roll ≠ 0, pitch = 0, yaw ≠ 0.
  rpy.set(roll_angle, 0, yaw_angle);
  CompareToMotionGenesisResult(rpy, p_zero, w_zero, v_zero);

  // Test with roll ≠ 0, pitch ≠ 0, yaw ≠ 0.
  rpy.set(roll_angle, pitch_angle, yaw_angle);
  CompareToMotionGenesisResult(rpy, p_zero, w_zero, v_zero);
}

// Verify results when body C has given position, but no motion in world A.
TEST_F(LinearBushingRollPitchYawTester, VariousPositionsAtRest) {
  const math::RollPitchYaw<double> rpy_zero(0, 0, 0);
  const Vector3<double> w_zero(0, 0, 0);
  const Vector3<double> v_zero(0, 0, 0);

  // Generic values for Co's position from Ao, expressed in B.
  // Note: The basis unit vectors associated with frames A, B, C are equal.
  const double x = 1.0;
  const double y = 2.0;
  const double z = 3.0;
  const double kx_x = -Kxyz()(0) * x;
  const double ky_y = -Kxyz()(1) * y;
  const double kz_z = -Kxyz()(2) * z;

  // Test zero position vector.
  Vector3<double> p_AoCo_A(0, 0, 0);
  Vector3<double> f_C_C_expected = Vector3<double>(0, 0, 0);
  Vector3<double> t_Co_C_expected = Vector3<double>(0, 0, 0);
  CompareToMotionGenesisResult(rpy_zero, p_AoCo_A, w_zero, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test simple translation (x-direction).
  p_AoCo_A = Vector3<double>(x, 0, 0);
  f_C_C_expected = Vector3<double>(kx_x, 0, 0);
  t_Co_C_expected = -0.5 * p_AoCo_A.cross(f_C_C_expected);
  CompareToMotionGenesisResult(rpy_zero, p_AoCo_A, w_zero, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test simple translation (y-direction).
  p_AoCo_A = Vector3<double>(0, y, 0);
  f_C_C_expected = Vector3<double>(0, ky_y, 0);
  t_Co_C_expected = -0.5 * p_AoCo_A.cross(f_C_C_expected);
  CompareToMotionGenesisResult(rpy_zero, p_AoCo_A, w_zero, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test simple translation (z-direction).
  p_AoCo_A = Vector3<double>(0, 0, z);
  f_C_C_expected = Vector3<double>(0, 0, kz_z);
  t_Co_C_expected = -0.5 * p_AoCo_A.cross(f_C_C_expected);
  CompareToMotionGenesisResult(rpy_zero, p_AoCo_A, w_zero, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test non-simple translation.
  p_AoCo_A = Vector3<double>(x, y, z);
  f_C_C_expected = Vector3<double>(kx_x, ky_y, kz_z);
  t_Co_C_expected = -0.5 * p_AoCo_A.cross(f_C_C_expected);
  CompareToMotionGenesisResult(rpy_zero, p_AoCo_A, w_zero, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);
}

// Verify results when body C has given pose, but no motion in world A.
TEST_F(LinearBushingRollPitchYawTester, VariousConfigurationsAtRest) {
  const Vector3<double> w_zero(0, 0, 0);
  const Vector3<double> v_zero(0, 0, 0);

  // Test with both zero and non-zero position vectors.
  const Vector3<double> p_zero(0, 0, 0);
  const Vector3<double> p_AoCo_A(1, 2, 3);

  // Test identity orientation.
  math::RollPitchYaw<double> rpy(0, 0, 0);
  CompareToMotionGenesisResult(rpy, p_zero, w_zero, v_zero);
  CompareToMotionGenesisResult(rpy, p_AoCo_A, w_zero, v_zero);

  // Test for non-zero roll only.
  rpy.set(M_PI / 6, 0, 0);
  CompareToMotionGenesisResult(rpy, p_zero, w_zero, v_zero);
  CompareToMotionGenesisResult(rpy, p_AoCo_A, w_zero, v_zero);

  // Test for non-zero pitch only.
  rpy.set(0, M_PI / 3, 0);
  CompareToMotionGenesisResult(rpy, p_zero, w_zero, v_zero);
  CompareToMotionGenesisResult(rpy, p_AoCo_A, w_zero, v_zero);

  // Test for non-zero yaw only.
  rpy.set(0, 0, 2 * M_PI / 3);
  CompareToMotionGenesisResult(rpy, p_zero, w_zero, v_zero);
  CompareToMotionGenesisResult(rpy, p_AoCo_A, w_zero, v_zero);

  // Test for non-zero roll, pitch, and yaw.
  rpy.set(M_PI / 6, M_PI / 3, 2 * M_PI / 3);
  CompareToMotionGenesisResult(rpy, p_zero, w_zero, v_zero);
  CompareToMotionGenesisResult(rpy, p_AoCo_A, w_zero, v_zero);
}

// Verify results when body C has angular velocity and identity pose.
TEST_F(LinearBushingRollPitchYawTester, IdentityPoseVariousRotationRates) {
  const math::RollPitchYaw<double> rpy_zero(0, 0, 0);
  const Vector3<double> p_zero(0, 0, 0);
  const Vector3<double> v_zero(0, 0, 0);

  // Generic values for roll, pitch, and yaw rates.
  const double roll_rate = 2.0;
  const double pitch_rate = 3.0;
  const double yaw_rate = 4.0;
  const double b0_roll = -B012()(0) * roll_rate;
  const double b1_pitch = -B012()(1) * pitch_rate;
  const double b2_yaw = -B012()(2) * yaw_rate;

  // Test zero angular velocity.
  Vector3<double> w_AC_A(0, 0, 0);
  Vector3<double> f_C_C_expected = Vector3<double>(0, 0, 0);
  Vector3<double> t_Co_C_expected = Vector3<double>(0, 0, 0);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_AC_A, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test simple angular velocity (x-direction).
  w_AC_A = Vector3<double>(roll_rate, 0, 0);
  t_Co_C_expected = Vector3<double>(b0_roll, 0, 0);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_AC_A, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test simple angular velocity (y-direction).
  w_AC_A = Vector3<double>(0, pitch_rate, 0);
  t_Co_C_expected = Vector3<double>(0, b1_pitch, 0);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_AC_A, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test simple angular velocity (z-direction).
  w_AC_A = Vector3<double>(0, 0, yaw_rate);
  t_Co_C_expected = Vector3<double>(0, 0, b2_yaw);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_AC_A, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test non-simple angular velocity.
  w_AC_A = Vector3<double>(2, 3, 4);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_AC_A, v_zero);
}

// Verify results when body C has translational velocity and identity pose.
TEST_F(LinearBushingRollPitchYawTester, IdentityPoseWithTranslationalRates) {
  const math::RollPitchYaw<double> rpy_zero(0, 0, 0);
  const Vector3<double> p_zero(0, 0, 0);
  const Vector3<double> w_zero(0, 0, 0);

  // Generic values for Co's position from Ao, expressed in B.
  // Note: The basis unit vectors associated with frames A, B, C are equal.
  const double xDt = 4.0;
  const double yDt = 5.0;
  const double zDt = 6.0;
  const double bx_xDt = -Bxyz()(0) * xDt;
  const double by_yDt = -Bxyz()(1) * yDt;
  const double bz_zDt = -Bxyz()(2) * zDt;

  // Test no translation.
  Vector3<double> v_ACo_A(0, 0, 0);
  Vector3<double> f_C_C_expected(0, 0, 0);
  Vector3<double> t_Co_C_expected(0, 0, 0);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_zero, v_ACo_A,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test simple translation (x-direction).
  v_ACo_A = Vector3<double>(xDt, 0, 0);
  f_C_C_expected = Vector3<double>(bx_xDt, 0, 0);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_zero, v_ACo_A,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test simple translation (y-direction).
  v_ACo_A = Vector3<double>(0, yDt, 0);
  f_C_C_expected = Vector3<double>(0, by_yDt, 0);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_zero, v_ACo_A,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test simple translation (z-direction).
  v_ACo_A = Vector3<double>(0, 0, zDt);
  f_C_C_expected = Vector3<double>(0, 0, bz_zDt);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_zero, v_ACo_A,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test non-simple translation.
  v_ACo_A = Vector3<double>(xDt, yDt, zDt);
  f_C_C_expected = Vector3<double>(bx_xDt, by_yDt, bz_zDt);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_zero, v_ACo_A,
                               &f_C_C_expected, &t_Co_C_expected);
}

// Verify results when body C has various poses and motion.
TEST_F(LinearBushingRollPitchYawTester, VariousPosesAndMotion) {
  const math::RollPitchYaw<double> rpy_zero(0, 0, 0);
  const math::RollPitchYaw<double> rpy(0.1, 0.2, 0.3);
  const Vector3<double> p_zero(0, 0, 0);
  const Vector3<double> p_AoCo_A(0.4, 0.5, 0.6);
  const Vector3<double> w_zero(0, 0, 0);
  const Vector3<double> w_AC_A(0.7, 0.8, 0.9);
  const Vector3<double> v_zero(0, 0, 0);
  const Vector3<double> v_ACo_A(1.1, 1.2, 1.3);
  CompareToMotionGenesisResult(rpy_zero, p_zero,   w_zero, v_zero);
  CompareToMotionGenesisResult(rpy_zero, p_zero,   w_zero, v_ACo_A);
  CompareToMotionGenesisResult(rpy_zero, p_zero,   w_AC_A, v_zero);
  CompareToMotionGenesisResult(rpy_zero, p_zero,   w_AC_A, v_ACo_A);
  CompareToMotionGenesisResult(rpy_zero, p_AoCo_A, w_zero, v_zero);
  CompareToMotionGenesisResult(rpy_zero, p_AoCo_A, w_zero, v_ACo_A);
  CompareToMotionGenesisResult(rpy_zero, p_AoCo_A, w_AC_A, v_zero);
  CompareToMotionGenesisResult(rpy_zero, p_AoCo_A, w_AC_A, v_ACo_A);
  CompareToMotionGenesisResult(rpy,      p_zero,   w_zero, v_zero);
  CompareToMotionGenesisResult(rpy,      p_zero,   w_zero, v_ACo_A);
  CompareToMotionGenesisResult(rpy,      p_zero,   w_AC_A, v_zero);
  CompareToMotionGenesisResult(rpy,      p_zero,   w_AC_A, v_ACo_A);
  CompareToMotionGenesisResult(rpy,      p_AoCo_A, w_zero, v_zero);
  CompareToMotionGenesisResult(rpy,      p_AoCo_A, w_zero, v_ACo_A);
  CompareToMotionGenesisResult(rpy,      p_AoCo_A, w_AC_A, v_zero);
  CompareToMotionGenesisResult(rpy,      p_AoCo_A, w_AC_A, v_ACo_A);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
