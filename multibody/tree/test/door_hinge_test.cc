#include "drake/multibody/tree/door_hinge.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

class DoorHingeTester {
 public:
  explicit DoorHingeTester(const DoorHinge<double>& door_hinge)
      : door_hinge_(door_hinge) {}

  double CalcHingeFrictionalTorque(double angular_velocity,
                                   const DoorHingeConfig& config) const {
    return door_hinge_.CalcHingeFrictionalTorque(angular_velocity, config);
  }

  double CalcHingeSpringTorque(double angle,
                               const DoorHingeConfig& config) const {
    return door_hinge_.CalcHingeSpringTorque(angle, config);
  }

  const DoorHinge<double>& door_hinge() const { return door_hinge_; }

  const DoorHingeConfig& door_hinge_config() const {
    return door_hinge_.config_;
  }

 private:
  const DoorHinge<double>& door_hinge_;
};

namespace {

constexpr double kPositionLowerLimit = -1.0;
constexpr double kPositionUpperLimit = 1.0;
constexpr double kDamping = 1.0;
constexpr double kAngle = 0.5;
constexpr double kAngularRate = 1.0;
constexpr double kIntegrationTimeStep = 1e-6;

class DoorHingeTest : public ::testing::Test {
 public:
  DoorHingeTest() {}

  void ConfigTest(const DoorHingeConfig& config) {
    // Create an empty model.
    auto model = std::make_unique<internal::MultibodyTree<double>>();

    auto& body1 = model->AddBody<RigidBody>(SpatialInertia<double>());
    revolute_joint_ = &model->AddJoint<RevoluteJoint>(
        "Joint1", model->world_body(), std::nullopt, body1, std::nullopt,
        Eigen::Vector3d::UnitZ(), kPositionLowerLimit, kPositionUpperLimit,
        kDamping);

    door_hinge_ = &model->AddForceElement<DoorHinge>(*revolute_joint_, config);

    door_hinge_tester_ = std::make_unique<DoorHingeTester>(*door_hinge_);

    // Transfer tree to system and get a Context.
    system_ = std::make_unique<internal::MultibodyTreeSystem<double>>(
        std::move(model));
    context_ = system_->CreateDefaultContext();
  }

  void SetHingeJointState(double angle, double angular_rate) {
    revolute_joint_->set_angle(context_.get(), angle);
    revolute_joint_->set_angular_rate(context_.get(), angular_rate);
  }

  const internal::MultibodyTree<double>& tree() const {
    return GetInternalTree(*system_);
  }

  const RevoluteJoint<double>& joint() {
    return dynamic_cast<const RevoluteJoint<double>&>(
        tree().get_joint(JointIndex(0)));
  }

  const DoorHingeTester& door_hinge_tester() { return *door_hinge_tester_; }

 protected:
  std::unique_ptr<internal::MultibodyTreeSystem<double>> system_;
  std::unique_ptr<systems::Context<double>> context_;

  const RevoluteJoint<double>* revolute_joint_{nullptr};
  const DoorHinge<double>* door_hinge_{nullptr};
  std::unique_ptr<DoorHingeTester> door_hinge_tester_;
};

DoorHingeConfig no_forces_config() {
  DoorHingeConfig config;
  config.spring_zero_angle_rad = 0;
  config.spring_constant = 0;
  config.dynamic_friction_torque = 0;
  config.static_friction_torque = 0;
  config.viscous_friction = 0;
  config.catch_width = 0;
  config.catch_torque = 0;
  return config;
}

// With the condition that there is only frictional torque, this function
// confirms that a) the potential energy and conservative power should be zero;
// b) on-conservative power equals to the corresponding torque times velocity.
void TestFrictionOnlyEnergyAndPower(const DoorHingeTester& dut,
                                    const systems::Context<double>& context,
                                    const internal::MultibodyTree<double>& tree,
                                    double angular_rate) {
  const double potential_energy_half_qc = dut.door_hinge().CalcPotentialEnergy(
      context, tree.EvalPositionKinematics(context));
  EXPECT_EQ(potential_energy_half_qc, 0.0);

  const double conserv_power = dut.door_hinge().CalcConservativePower(
      context, tree.EvalPositionKinematics(context),
      tree.EvalVelocityKinematics(context));
  EXPECT_EQ(conserv_power, 0.0);

  // Verify the non-conservative power
  const double non_conserv_power = dut.door_hinge().CalcNonConservativePower(
      context, tree.EvalPositionKinematics(context),
      tree.EvalVelocityKinematics(context));
  EXPECT_EQ(non_conserv_power, dut.CalcHingeFrictionalTorque(
                                   angular_rate, dut.door_hinge_config()) *
                                   angular_rate);
}

// With the condition that there is only spring related torque, this function
// confirms that a) the potential energy equals to the corresponding torque
// times velocity; b) on-conservative power should be zero.
void TestSpringTorqueOnlyPower(const DoorHingeTester& dut,
                               const systems::Context<double>& context,
                               const internal::MultibodyTree<double>& tree,
                               double angle, double angular_rate) {
  auto spring_power = [&dut](double q, double v) {
    return dut.CalcHingeSpringTorque(q, dut.door_hinge_config()) * v;
  };

  const double conserv_power = dut.door_hinge().CalcConservativePower(
      context, tree.EvalPositionKinematics(context),
      tree.EvalVelocityKinematics(context));
  EXPECT_EQ(conserv_power, spring_power(angle, angular_rate));

  const double non_conserv_power = dut.door_hinge().CalcNonConservativePower(
      context, tree.EvalPositionKinematics(context),
      tree.EvalVelocityKinematics(context));
  EXPECT_EQ(non_conserv_power, 0.0);
}

// This function integrates the conservative power (P) to get the corresponding
// energy (PE), i.e., PE = -∫Pdt. We assume the hinge joint moves from zero to
// the input `angle` with a constant `angular_rate`.
double IntegrateConservativePower(const DoorHingeTester& dut,
                                  double target_angle, double angular_rate) {
  auto conserv_power = [&dut](double q, double v) {
    return dut.CalcHingeSpringTorque(q, dut.door_hinge_config()) * v;
  };

  double angle = 0.0;
  double pe_integrated = 0.0;
  while (angle < target_angle) {
    pe_integrated -= conserv_power(angle, angular_rate) * kIntegrationTimeStep;
    angle += angular_rate * kIntegrationTimeStep;
  }
  return pe_integrated;
}

// Verify the torques and the energy should be zero when the config parameters
// are all zero.
TEST_F(DoorHingeTest, ZeroTest) {
  DoorHingeConfig config = no_forces_config();
  ConfigTest(config);
  DoorHingeTester dut = door_hinge_tester();

  // If no frictions, springs, etc. are applied, our torques should be 0.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1., config), 0);

  // Test energy should be zero at a default condition.
  const double potential_energy_1 = dut.door_hinge().CalcPotentialEnergy(
      *context_, tree().EvalPositionKinematics(*context_));
  EXPECT_EQ(potential_energy_1, 0.0);

  // Test energy should be zero at a non-default condition.
  SetHingeJointState(0.2, 0.1);
  const double potential_energy_2 = dut.door_hinge().CalcPotentialEnergy(
      *context_, tree().EvalPositionKinematics(*context_));
  EXPECT_EQ(potential_energy_2, 0.0);
}

// Test the case with only the torional spring torque, the corresponding energy
// and power are computed correctly at different states.
TEST_F(DoorHingeTest, SpringTest) {
  DoorHingeConfig config = no_forces_config();
  config.spring_constant = 1;
  ConfigTest(config);
  DoorHingeTester dut = door_hinge_tester();

  // Springs make spring torque (but not friction).
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1., config), 0);
  EXPECT_LT(dut.CalcHingeSpringTorque(1., config), 0);  // Pulls toward zero.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);

  // Test potential energy at non-zero angle.
  SetHingeJointState(kAngle, kAngularRate);
  const double integrated_potential_energy =
      IntegrateConservativePower(dut, kAngle, kAngularRate);
  const double potential_energy = dut.door_hinge().CalcPotentialEnergy(
      *context_, tree().EvalPositionKinematics(*context_));
  EXPECT_NEAR(potential_energy, integrated_potential_energy,
              kIntegrationTimeStep);

  // Test the powers are computed correctly.
  TestSpringTorqueOnlyPower(dut, *context_, tree(), kAngle, kAngularRate);
}

// Test the case with only the catch spring torque, the corresponding
// energy and power are computed correctly at different states.
TEST_F(DoorHingeTest, CatchTest) {
  DoorHingeConfig config = no_forces_config();
  config.catch_width = 2 * kAngle;
  config.catch_torque = 1.0;
  ConfigTest(config);
  DoorHingeTester dut = door_hinge_tester();

  // The catch makes spring torque (but not friction).
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1., config), 0);

  // Resists closing.
  EXPECT_GT(dut.CalcHingeSpringTorque(config.catch_width, config), 0);
  // Tipover point.
  EXPECT_EQ(dut.CalcHingeSpringTorque(config.catch_width / 2, config), 0);
  // Detent pulls in.
  EXPECT_LT(dut.CalcHingeSpringTorque(0., config), 0);

  // Verify that the potential energy with only the catch spring torque at zero
  // position and the catch_width position should be 0.
  const double potential_energy_q0 = dut.door_hinge().CalcPotentialEnergy(
      *context_, tree().EvalPositionKinematics(*context_));
  EXPECT_EQ(potential_energy_q0, 0.0);

  SetHingeJointState(config.catch_width, 0.0);
  const double potential_energy_qc = dut.door_hinge().CalcPotentialEnergy(
      *context_, tree().EvalPositionKinematics(*context_));
  EXPECT_EQ(potential_energy_qc, 0.0);

  // Test the energy from power integration.
  SetHingeJointState(kAngle, kAngularRate);
  const double integrated_potential_energy =
      IntegrateConservativePower(dut, kAngle, kAngularRate);
  const double potential_energy = dut.door_hinge().CalcPotentialEnergy(
      *context_, tree().EvalPositionKinematics(*context_));
  EXPECT_NEAR(potential_energy, integrated_potential_energy,
              kIntegrationTimeStep);

  // Verify the power terms are computed correctly.
  // Test the powers are computed correctly
  TestSpringTorqueOnlyPower(dut, *context_, tree(), kAngle, kAngularRate);
}

TEST_F(DoorHingeTest, StaticFrictionTest) {
  DoorHingeConfig config = no_forces_config();
  config.static_friction_torque = 1;
  ConfigTest(config);
  DoorHingeTester dut = door_hinge_tester();

  // Friction opposes tiny motion, but falls away with substantial motion.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_LT(dut.CalcHingeFrictionalTorque(0.001, config), -0.5);
  EXPECT_GT(dut.CalcHingeFrictionalTorque(-0.001, config), 0.5);
  EXPECT_NEAR(dut.CalcHingeFrictionalTorque(0.01, config), 0, 1e-7);

  // No spring torque.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1., config), 0);

  // Test the energy and power with a given state.
  SetHingeJointState(kAngle, kAngularRate);
  TestFrictionOnlyEnergyAndPower(dut, *context_, tree(), kAngularRate);
}

TEST_F(DoorHingeTest, DynamicFrictionTest) {
  DoorHingeConfig config = no_forces_config();
  config.dynamic_friction_torque = 1;
  ConfigTest(config);
  DoorHingeTester dut = door_hinge_tester();

  // Friction opposes any motion, even tiny motion.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_LT(dut.CalcHingeFrictionalTorque(0.001, config), -0.5);
  EXPECT_GT(dut.CalcHingeFrictionalTorque(-0.001, config), 0.5);
  EXPECT_NEAR(dut.CalcHingeFrictionalTorque(0.01, config), -1, 1e-7);

  // No spring torque.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1., config), 0);

  // Test the energy and power with a given state.
  SetHingeJointState(kAngle, kAngularRate);
  TestFrictionOnlyEnergyAndPower(dut, *context_, tree(), kAngularRate);
}

TEST_F(DoorHingeTest, ViscousFrictionTest) {
  DoorHingeConfig config = no_forces_config();
  config.viscous_friction = 1;
  ConfigTest(config);
  DoorHingeTester dut = door_hinge_tester();

  // Friction opposes motion proprotionally.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1., config), -1);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(-1., config), 1);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(-2., config), 2);

  // No spring torque.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1., config), 0);

  // Test the energy and power with a given state.
  SetHingeJointState(kAngle, kAngularRate);
  TestFrictionOnlyEnergyAndPower(dut, *context_, tree(), kAngularRate);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
