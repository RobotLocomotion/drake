#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

using math::RotationMatrixd;

// Friend class for accessing protected/private internals of class
// LinearBushingRollPitchYawTester.
class BushingTester {
 public:
  BushingTester() = delete;

  // Verify the efficient half-angle axis formula in LinearBushingRollPitchYaw.
  static void VerifyHalfAngleAxisAlgorithm(double angle,
                                           const Vector3<double>& axis) {
    RotationMatrixd R_AC(Eigen::AngleAxis<double>(angle, axis));
    RotationMatrixd R_AB = LinearBushingRollPitchYaw<double>::CalcR_AB(R_AC);

    RotationMatrixd R_AB_expected(Eigen::AngleAxis<double>(angle / 2, axis));
    DRAKE_EXPECT_NO_THROW(
      LinearBushingRollPitchYaw<double>::ThrowIfInvalidHalfAngleAxis(
              R_AC, R_AB_expected));

    // Ensure calculated R_AB is nearly equal to R_AB_expected.
    DRAKE_ASSERT(R_AB.IsNearlyEqualTo(R_AB_expected, 32 * kEpsilon));

    double bad_half_angle = angle / 2 + 128 * kEpsilon;
    RotationMatrixd R_AB_bad(Eigen::AngleAxis<double>(bad_half_angle, axis));
    DRAKE_EXPECT_THROWS_MESSAGE(LinearBushingRollPitchYaw<double>::
          ThrowIfInvalidHalfAngleAxis(R_AC, R_AB_bad),
          std::runtime_error,
          "Error: Calculation of R_AB from quaternion differs from the "
          "R_AB_expected formed via a half-angle axis calculation.");
  }
};


namespace internal {
namespace {

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
  //   frame C expressed in frame C -- or nullptr (no comparison).
  // @param[in] t_Co_C_expected_simple Expected net moment from the bushing on
  //   frame C about Co expressed in frame C -- or nullptr (no comparison).
  // @note: In some situations, it is relatively easy to calculate expected
  //   results that can be passed to this method as a non nullptr and used as a
  //   secondary check on the Drake results.
  void CompareToMotionGenesisResult(
      const math::RollPitchYaw<double>& rpy,
      const Vector3<double>& p_AoCo_A,
      const Vector3<double>& w_AC_A,
      const Vector3<double>& v_ACo_A,
      const Vector3<double>* f_C_C_expected_simple = nullptr,
      const Vector3<double>* t_Co_C_expected_simple = nullptr) const {
    const RotationMatrixd R_AC(rpy);
    const RotationMatrixd R_CA = R_AC.inverse();

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

    // ---------- Start of alternate calculations ---------
    // Get the roll-pitch-yaw angles relating orientation of frames A and C.
    const double q0 = rpy.roll_angle();    // rad
    const double q1 = rpy.pitch_angle();   // rad
    const double q2 = rpy.yaw_angle();     // rad

    // Form kinematical ODEs that relate [q̇₀ q̇₁ q̇₂] to w_AC_A.
    const Vector3<double> qDt =
        rpy.CalcRpyDtFromAngularVelocityInParent(w_AC_A);

    // Frame B is oriented "halfway" between frames A and C.
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
    const RotationMatrixd R_AB(angleAxis_AB);
    const RotationMatrixd R_BA = R_AB.inverse();
    const RotationMatrixd R_CB = (R_BA * R_AC).inverse();

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
    const Vector3<double>& d012 = D012();
    const Vector3<double>& kxyz = Kxyz();
    const Vector3<double>& dxyz = Dxyz();
    const double d0 = d012(0), d1 = d012(1), d2 = d012(2);  // N*m*s/rad
    const double dx = dxyz(0), dy = dxyz(1), dz = dxyz(2);  // N*s/m
    const double k0 = k012(0), k1 = k012(1), k2 = k012(2);  // N*m/rad
    const double kx = kxyz(0), ky = kxyz(1), kz = kxyz(2);  // N/m

    // Form net force 𝐟 = 𝐟ᴋ + 𝐟ᴅ on frameC from the bushing.
    const Vector3<double> fK(-kx * x, -ky * y, -kz * z);        // Stiffness
    const Vector3<double> fB(-dx * xDt, -dy * yDt, -dz * zDt);  // Damping
    const Vector3<double> f = fK + fB;
    const Vector3<double> f_A_B_expected = -f;
    const Vector3<double> f_C_B_expected = f;

    // Re-express the net force on frame A in frame A.
    // Re-express the net force on frame C in frame C.
    const Vector3<double> f_A_A_expected = R_AB * f_A_B_expected;
    const Vector3<double> f_C_C_expected = R_CB * f_C_B_expected;

    // Form "gimbal torques τ₀, τ₁, τ₂ associated with roll-pitch-yaw rotations.
    const Vector3<double> tau(-(k0 * q0 + d0 * qDt(0)),
                              -(k1 * q1 + d1 * qDt(1)),
                              -(k2 * q2 + d2 * qDt(2)));

    // Form `txyz = tx Ax + ty Ay + tz Az` which is the torque required when the
    // bushing forces on C have their resultant force f applied at Cp (not Co).
    // 𝐭 is the torque on C which produces the same power as the spring-damper
    // "gimbal" torques τ₀, τ₁, τ₂.
    const Matrix3<double> N =
        rpy.CalcMatrixRelatingRpyDtToAngularVelocityInParent();
    const Vector3<double> txyz = N.transpose() * tau;

    // Calculate the moment on frame A about Ao, expressed in A.
    const Vector3<double> p_AoBo_B = 0.5 * p_AoCo_B;
    const Vector3<double> t_Ao_A_expected = -txyz +
        R_AB * p_AoBo_B.cross(f_A_B_expected);

    // Calculate the moment on frame C about Co, expressed in A.
    const Vector3<double> p_CoBo_B = -p_AoBo_B;
    const Vector3<double> t_Co_C_expected = R_CA * txyz
        + R_CB * p_CoBo_B.cross(f_C_B_expected);

    // Verify F_A_A,, the bushing's spatial force calculation on frame A.
    double torque_epsilon = 32 * kEpsilon * t_Ao_A_expected.norm();
    double force_epsilon = 32 * kEpsilon * f_A_A_expected.norm();
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

    // Verify potential energy and power methods throw exceptions.
    const MultibodyTree<double>& mbt = GetInternalTree(*mbtree_system_);
    const PositionKinematicsCache<double>& pc =
        mbt.EvalPositionKinematics(context);
    const VelocityKinematicsCache<double>& vc =
        mbt.EvalVelocityKinematics(context);

    // Verify CalcPotentialEnergy() throws an exception (see issue #12982).
    const ForceElement<double> *bushing_force_element = bushing_;
    DRAKE_EXPECT_THROWS_MESSAGE(
        bushing_force_element->CalcPotentialEnergy(context, pc),
        std::runtime_error,
        "Error: LinearBushingRollPitchYaw::CalcPotentialEnergy().*");

    // Verify CalcConservativePower() throws an exception (see issue #12982).
    DRAKE_EXPECT_THROWS_MESSAGE(
        bushing_force_element->CalcConservativePower(context, pc, vc),
        std::runtime_error,
        "Error: LinearBushingRollPitchYaw::CalcConservativePower().*");

    // Verify CalcConservativePower() throws an exception (see issue #12982).
    DRAKE_EXPECT_THROWS_MESSAGE(
        bushing_force_element->CalcNonConservativePower(context, pc, vc),
        std::runtime_error,
        "Error: LinearBushingRollPitchYaw::CalcNonConservativePower().*");
  }

  // Calculates "reasonable" torque and force stiffness/damping constants,
  // where "reasonable" means a critical damping ratio ζ = 0.1 and damped
  // natural periods of a few seconds.  This method returns output via its
  // arguments, which are defined in the LinearBushingRollPitchYaw constructor.
  void CalcReasonableStiffnessAndDampingConstants(
      Vector3<double>* torque_stiffness_constants,         /* [k₀, k₁, k₂] */
      Vector3<double>* torque_damping_constants,           /* [d₀, d₁, d₂] */
      Vector3<double>* force_stiffness_constants,          /* [kx, ky, kz] */
      Vector3<double>* force_damping_constants) const {    /* [dx, dy, dz] */
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
    const double d0 = 2 * zeta * std::sqrt(Ixx_ * k0);           // N*m*s/rad
    const double d1 = 2 * zeta * std::sqrt(Iyy_ * k1);           // N*m*s/rad
    const double d2 = 2 * zeta * std::sqrt(Izz_ * k2);           // N*m*s/rad
    *torque_stiffness_constants = Vector3<double>(k0, k1, k2);
    *torque_damping_constants = Vector3<double>(d0, d1, d2);

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
    const Vector3<double> Kxyz(kx, ky, kz);
    const Vector3<double> Dxyz = 2 * zeta * (mC_ * Kxyz).cwiseSqrt();
    *force_stiffness_constants = Vector3<double>(kx, ky, kz);
    *force_damping_constants = Vector3<double>(Dxyz);
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
  const Vector3<double>& D012() const {
    return bushing_->torque_damping_constants();
  }
  const Vector3<double>& Kxyz() const {
    return bushing_->force_stiffness_constants();
  }
  const Vector3<double>& Dxyz() const {
    return bushing_->force_damping_constants();
  }
};

TEST_F(LinearBushingRollPitchYawTester, ConstructionAndAccessors) {
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
  const double d0_roll_rate = -D012()(0) * roll_rate;
  const double d1_pitch_rate = -D012()(1) * pitch_rate;
  const double d2_yaw_rate = -D012()(2) * yaw_rate;

  // Test zero angular velocity.
  Vector3<double> w_AC_A(0, 0, 0);
  Vector3<double> f_C_C_expected = Vector3<double>(0, 0, 0);
  Vector3<double> t_Co_C_expected = Vector3<double>(0, 0, 0);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_AC_A, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test simple angular velocity (x-direction).
  w_AC_A = Vector3<double>(roll_rate, 0, 0);
  t_Co_C_expected = Vector3<double>(d0_roll_rate, 0, 0);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_AC_A, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test simple angular velocity (y-direction).
  w_AC_A = Vector3<double>(0, pitch_rate, 0);
  t_Co_C_expected = Vector3<double>(0, d1_pitch_rate, 0);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_AC_A, v_zero,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test simple angular velocity (z-direction).
  w_AC_A = Vector3<double>(0, 0, yaw_rate);
  t_Co_C_expected = Vector3<double>(0, 0, d2_yaw_rate);
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
  const double dx_xDt = -Dxyz()(0) * xDt;
  const double dy_yDt = -Dxyz()(1) * yDt;
  const double dz_zDt = -Dxyz()(2) * zDt;

  // Test no translation.
  Vector3<double> v_ACo_A(0, 0, 0);
  Vector3<double> f_C_C_expected(0, 0, 0);
  Vector3<double> t_Co_C_expected(0, 0, 0);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_zero, v_ACo_A,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test simple translation (x-direction).
  v_ACo_A = Vector3<double>(xDt, 0, 0);
  f_C_C_expected = Vector3<double>(dx_xDt, 0, 0);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_zero, v_ACo_A,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test simple translation (y-direction).
  v_ACo_A = Vector3<double>(0, yDt, 0);
  f_C_C_expected = Vector3<double>(0, dy_yDt, 0);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_zero, v_ACo_A,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test simple translation (z-direction).
  v_ACo_A = Vector3<double>(0, 0, zDt);
  f_C_C_expected = Vector3<double>(0, 0, dz_zDt);
  CompareToMotionGenesisResult(rpy_zero, p_zero, w_zero, v_ACo_A,
                               &f_C_C_expected, &t_Co_C_expected);

  // Test non-simple translation.
  v_ACo_A = Vector3<double>(xDt, yDt, zDt);
  f_C_C_expected = Vector3<double>(dx_xDt, dy_yDt, dz_zDt);
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

// Test that an exception is thrown near gimbal lock singularity.
TEST_F(LinearBushingRollPitchYawTester, TestGimbalLock) {
  const math::RollPitchYaw<double> rpy_gimbal_lock(0, M_PI/2, 0);
  const Vector3<double> p_zero(0, 0, 0);
  const Vector3<double> w_zero(0, 0, 0);
  const Vector3<double> v_zero(0, 0, 0);

  // Use the mobilizer to set frame C's pose and motion in frame A.
  const RotationMatrixd R_AC(rpy_gimbal_lock);
  systems::Context<double>& context = *(context_.get());
  mobilizer_->SetFromRotationMatrix(&context, R_AC);
  mobilizer_->set_position(&context, p_zero);
  mobilizer_->set_angular_velocity(&context, w_zero);
  mobilizer_->set_translational_velocity(&context, v_zero);

  const char* expected_message =
      "LinearBushingRollPitchYaw::CalcBushingRollPitchYawAngleRates()"
      ".*gimbal-lock.*";

  // Check that CalcBushingSpatialForceOnFrameA() throws near gimbal lock.
  DRAKE_EXPECT_THROWS_MESSAGE(
      bushing_->CalcBushingSpatialForceOnFrameA(context),
      std::runtime_error, expected_message);

  // Check that CalcBushingSpatialForceOnFrameC() throws near gimbal lock.
  DRAKE_EXPECT_THROWS_MESSAGE(
      bushing_->CalcBushingSpatialForceOnFrameC(context),
      std::runtime_error, expected_message);
}

// Verify algorithm that calculates rotation matrix R_AB.
TEST_F(LinearBushingRollPitchYawTester, HalfAngleAxisAlgorithm) {
  // This test uses a generic unit vector for the "axis" part of AngleAxis.
  const Vector3<double> unit_vector = Vector3<double>(1, 2, 3).normalized();
  for (double angle = 0; angle <= 0.99 * M_PI;  angle += M_PI / 32) {;
    BushingTester::VerifyHalfAngleAxisAlgorithm(angle, unit_vector);
  }
}

GTEST_TEST(LinearBushingRollPitchYawTest, BushingParameters) {
  // Add a plant with a few rigid bodies.
  MultibodyPlant<double> plant(0.0);

  const double sphere_radius = 1.0;
  const double sphere_mass = 2.5;
  const Vector3<double> sphere_com(0, 0, 0);
  const UnitInertia<double> sphere_unit_inertia =
      UnitInertia<double>::SolidSphere(sphere_radius);

  const RigidBody<double>& sphere1 = plant.AddRigidBody(
      "sphere1",
      SpatialInertia<double>(sphere_mass, sphere_com, sphere_unit_inertia));

  const RigidBody<double>& sphere2 = plant.AddRigidBody(
      "sphere2",
      SpatialInertia<double>(sphere_mass, sphere_com, sphere_unit_inertia));

  const Vector3<double> torque_stiffness(100, 100, 100);
  const Vector3<double> torque_damping(5, 5, 5);
  const Vector3<double> force_stiffness(100, 100, 100);
  const Vector3<double> force_damping(5, 5, 5);

  const LinearBushingRollPitchYaw<double>& bushing =
      plant.AddForceElement<LinearBushingRollPitchYaw>(
          sphere1.body_frame(), sphere2.body_frame(), torque_stiffness,
          torque_damping, force_stiffness, force_damping);

  plant.Finalize();

  // Create a default context.
  auto context = plant.CreateDefaultContext();

  // Verify default parameters exist and are correct.
  const Vector3<double> default_torque_stiffness =
      bushing.GetTorqueStiffnessConstants(*context);
  const Vector3<double> default_torque_damping =
      bushing.GetTorqueDampingConstants(*context);
  const Vector3<double> default_force_stiffness =
      bushing.GetForceStiffnessConstants(*context);
  const Vector3<double> default_force_damping =
      bushing.GetForceDampingConstants(*context);

  EXPECT_TRUE(CompareMatrices(torque_stiffness, default_torque_stiffness));
  EXPECT_TRUE(CompareMatrices(torque_damping, default_torque_damping));
  EXPECT_TRUE(CompareMatrices(force_stiffness, default_force_stiffness));
  EXPECT_TRUE(CompareMatrices(force_damping, default_force_damping));

  // Change parameters.
  const Vector3<double> new_torque_stiffness(50, 50, 50);
  const Vector3<double> new_torque_damping(2, 2, 2);
  const Vector3<double> new_force_stiffness(50, 50, 50);
  const Vector3<double> new_force_damping(2, 2, 2);

  bushing.SetTorqueStiffnessConstants(context.get(), new_torque_stiffness);
  bushing.SetTorqueDampingConstants(context.get(), new_torque_damping);
  bushing.SetForceStiffnessConstants(context.get(), new_force_stiffness);
  bushing.SetForceDampingConstants(context.get(), new_force_damping);

  // Verify parameter changes propagate.
  const Vector3<double> new_default_torque_stiffness =
      bushing.GetTorqueStiffnessConstants(*context);
  const Vector3<double> new_default_torque_damping =
      bushing.GetTorqueDampingConstants(*context);
  const Vector3<double> new_default_force_stiffness =
      bushing.GetForceStiffnessConstants(*context);
  const Vector3<double> new_default_force_damping =
      bushing.GetForceDampingConstants(*context);

  EXPECT_TRUE(
      CompareMatrices(new_torque_stiffness, new_default_torque_stiffness));
  EXPECT_TRUE(CompareMatrices(new_torque_damping, new_default_torque_damping));
  EXPECT_TRUE(
      CompareMatrices(new_force_stiffness, new_default_force_stiffness));
  EXPECT_TRUE(CompareMatrices(new_force_damping, new_default_force_damping));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
