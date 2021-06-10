#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/linear_spring_damper.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using math::RigidTransform;
using math::RollPitchYaw;
using math::RotationMatrix;
using systems::Context;

// Fixture for energy and power tests. MBTree calculates kinetic energy,
// potential energy and power are just accumulated from force elements.
// We'll use gravity and a linear spring/damper to supply potential energy
// and power.
class EnergyAndPowerTester : public ::testing::Test {
 public:
  void SetUp() override {
    // Override default gravity so we can calculate locally.
    plant_.mutable_gravity_field().set_gravity_vector(gravity_W_);

    // We want a free body with a general inertia matrix.
    body_ = &plant_.AddRigidBody(
        "Lumpy",
        SpatialInertia<double>(body_mass_, p_BBcm_, G_BBo_));

    spring_damper_ = &plant_.AddForceElement<LinearSpringDamper>(
        plant_.world_body(), p_WP_, *body_, p_BQ_, free_length_, stiffness_,
        damping_);

    plant_.Finalize();
    context_ = plant_.CreateDefaultContext();

    // Set the pose and spatial velocity to something arbitrary and general.
    SetGeneralPose();
    SetGeneralVelocity();
  }

  // Put the body in a general pose (no zeroes).
  void SetGeneralPose() {
    const RigidTransform<double> X_WB(RollPitchYaw<double>(.5, -.2, 1.2),
                                      Eigen::Vector3d(10., 20., -9.));
    plant_.SetFreeBodyPose(context_.get(), *body_, X_WB);
  }

  // Give the body a general velocity (no zeroes).
  void SetGeneralVelocity() {
    Vector6<double> V_WB;
    V_WB << 7., 8., -9.,  // ω
        10., 11., 12.;    // v
    plant_.SetFreeBodySpatialVelocity(context_.get(), *body_,
                                      SpatialVelocity<double>(V_WB));
  }

  const RigidTransform<double>& get_X_WB() const {
    return body_->EvalPoseInWorld(*context_);
  }

  const SpatialVelocity<double>& get_V_WB() const {
    return body_->EvalSpatialVelocityInWorld(*context_);
  }

  const SpatialInertia<double>& get_M_B_W() const {
    const std::vector<SpatialInertia<double>>& M_Bi_W =
        plant_.EvalSpatialInertiaInWorldCache(*context_);
    return M_Bi_W[body_->node_index()];
  }

  // Return an appropriate tolerance to be used in checking the given quantity.
  // Units are the same as those of the quantity.
  double absolute_tolerance(double quantity) const {
    return kAccuracy * std::max(1., std::abs(quantity));
  }

 protected:
  // Require tests to achieve this fractional accuracy (except near zero).
  const double kAccuracy{10 * std::numeric_limits<double>::epsilon()};

  MultibodyPlant<double> plant_{0.};
  std::unique_ptr<Context<double>> context_;

  const RigidBody<double>* body_{nullptr};
  const LinearSpringDamper<double>* spring_damper_{nullptr};

  // Some general values to avoid zeroes that could mask bugs.
  const Vector3d gravity_W_{1., 2., 3.};  // Magnitude * direction.
  const double body_mass_{2.};
  const Vector3d p_BBcm_{.1, .3, -.2};  // Center of mass location.
  const UnitInertia<double> G_BBo_{1., 2., 2.5, .01, .02, .03};

  // Parameters of the spring connecting point P on World to point Q on body.
  const double free_length_{1.5};  // [m]
  const double stiffness_{100.};   // [N/m]
  const double damping_{3.};       // [Ns/m]
  const Vector3d p_WP_{0., 0., 0.};
  const Vector3d p_BQ_{0., 0., 0.};
};

// Position the body in some arbitrary pose with an arbitrary spatial velocity
// V (in World) and verify that the kinetic energy (KE) in World is ½VᵀMV where
// M is the body's spatial inertia expressed in World.
TEST_F(EnergyAndPowerTester, KineticEnergy) {
  // Note: to avoid an entirely tautological test, we're deliberately doing
  // this with plain Eigen objects rather than making use of the beautiful Drake
  // classes (like SpatialMomentum) used in the internal implementation.
  const Vector6<double> V_WB = get_V_WB().get_coeffs();
  const Matrix6<double> M_B_W = get_M_B_W().CopyToFullMatrix6();

  const double expected_KE_W = V_WB.dot(M_B_W * V_WB) / 2.;
  EXPECT_NEAR(plant_.EvalKineticEnergy(*context_), expected_KE_W,
              absolute_tolerance(expected_KE_W));
}

// Position the body in some arbitrary pose and verify that the system
// potential energy is the sum of the spring and gravity potential energies.
TEST_F(EnergyAndPowerTester, PotentialEnergy) {
  const RigidTransform<double>& X_WB = get_X_WB();

  // Get the body COM and spring attachment point in World.
  const Vector3d p_WBcm = X_WB * p_BBcm_;
  const Vector3d p_WQ = X_WB * p_BQ_;

  // Gravity PE is mass * gravity_magnitude * height.
  const double mgh = body_mass_ * p_WBcm.dot(-gravity_W_);

  // Stretch of the spring, and spring PE (= ½kx²).
  const double x = (p_WQ - p_WP_).norm() - free_length_;
  const double spring_PE = stiffness_ * x * x / 2.;
  const double energy = mgh + spring_PE;

  EXPECT_NEAR(plant_.EvalPotentialEnergy(*context_), energy,
              absolute_tolerance(energy));
}

// The only source of non-conservative power is the damping in the
// spring/damper. That should be equal to the damping force times the
// spring stretch velocity.
TEST_F(EnergyAndPowerTester, NonConservativePower) {
  const RigidTransform<double>& X_WB = get_X_WB();
  const Vector3d& p_WB = X_WB.translation();
  const RotationMatrix<double>& R_WB = X_WB.rotation();
  const SpatialVelocity<double>& V_WB = get_V_WB();
  const Vector3d p_BQ_W = R_WB * p_BQ_;
  const Vector3d p_WQ = p_WB + p_BQ_W;
  const Vector3d p_PQ_W = p_WQ - p_WP_;

  // Unit vector along the spring stretch direction, oriented from P to Q.
  const Vector3d u_PQ_W = p_PQ_W.normalized();
  const Vector3d v_WQ = V_WB.Shift(p_BQ_W).translational();
  // This is the relative velocity in World between P and Q because P is
  // fixed on World.
  const Vector3d v_PQ_W = v_WQ;

  // The stretch rate is positive when the spring is getting longer.
  const double stretch_rate = v_PQ_W.dot(u_PQ_W);
  // Damping force opposes the stretch_rate; at Q it is aligned with u_PQ_W.
  // (At P it would be aligned with -u_PQ_W.)
  const Vector3d damping_force_Q_W = -(damping_ * stretch_rate) * u_PQ_W;
  // The force at P produces no power since P doesn't move.
  const double damping_power = damping_force_Q_W.dot(v_WQ);

  EXPECT_NEAR(plant_.EvalNonConservativePower(*context_), damping_power,
              absolute_tolerance(damping_power));
}

// Conservative power is the rate at which potential energy is being
// converted to kinetic energy. That's the gravitational force times the
// body COM velocity, and the spring stiffness force times the stretch rate.
TEST_F(EnergyAndPowerTester, ConservativePower) {
  const RigidTransform<double>& X_WB = get_X_WB();
  const Vector3d& p_WB = X_WB.translation();
  const RotationMatrix<double>& R_WB = X_WB.rotation();
  const SpatialVelocity<double>& V_WB = get_V_WB();
  const Vector3d p_BBcm_W = R_WB * p_BBcm_;
  const Vector3d p_BQ_W = R_WB * p_BQ_;
  const Vector3d p_WQ = p_WB + p_BQ_W;
  const Vector3d p_PQ_W = p_WQ - p_WP_;

  // Gravity always acts at the body COM.
  const Vector3d gravity_force_Bcm_W = body_mass_ * gravity_W_;
  const Vector3d v_WBcm = V_WB.Shift(p_BBcm_W).translational();
  const double gravity_power = gravity_force_Bcm_W.dot(v_WBcm);

  // Displacement (stretch or compression) of the spring.
  const double x = p_PQ_W.norm() - free_length_;
  // Unit vector along the spring stretch direction, oriented from P to Q.
  const Vector3d u_PQ_W = p_PQ_W.normalized();
  // Stiffness force opposes the spring displacement; at Q it is aligned with
  // u_PQ_W. (At P it would be aligned with -u_PQ_W.)
  const Vector3d stiffness_force_Q_W = -(stiffness_ * x) * u_PQ_W;

  const Vector3d v_WQ = V_WB.Shift(p_BQ_W).translational();
  // The force at P produces no power since P doesn't move.
  const double stiffness_power = stiffness_force_Q_W.dot(v_WQ);
  const double power = stiffness_power + gravity_power;

  EXPECT_NEAR(plant_.EvalConservativePower(*context_), power,
              absolute_tolerance(power));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
