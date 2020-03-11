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

 private:
  const DoorHinge<double>& door_hinge_;
};

namespace {

constexpr double kPositionLowerLimit = -1.0;
constexpr double kPositionUpperLimit = 1.0;
constexpr double kDamping = 1.0;

class DoorHingeTest : public ::testing::Test {
 public:
  DoorHingeTest() {
    // Create an empty model.
    auto model = std::make_unique<internal::MultibodyTree<double>>();

    auto& body1 = model->AddBody<RigidBody>(SpatialInertia<double>());
    hinge_joint_ = &model->AddJoint<RevoluteJoint>(
        "Joint1", model->world_body(), std::nullopt, body1, std::nullopt,
        Eigen::Vector3d::UnitZ(), kPositionLowerLimit, kPositionUpperLimit,
        kDamping);

    // Transfer tree to system and get a Context.
    system_ = std::make_unique<internal::MultibodyTreeSystem<double>>(
        std::move(model));
    context_ = system_->CreateDefaultContext();
  }

  void SetHingeJointState(double angle, double angular_rate) {
    hinge_joint_->set_angle(context_.get(), angle);
    hinge_joint_->set_angular_rate(context_.get(), angular_rate);
  }

  const internal::MultibodyTree<double>& tree() const {
    return GetInternalTree(*system_);
  }

  const RevoluteJoint<double>& joint() {
    return dynamic_cast<const RevoluteJoint<double>&>(
        tree().get_joint(JointIndex(0)));
  }

  DoorHingeTester CreateDoorHingeTester(const DoorHingeConfig& config) {
    DoorHinge<double> door_hinge(joint(), config);
    return DoorHingeTester(door_hinge);
  }

 protected:
  std::unique_ptr<internal::MultibodyTreeSystem<double>> system_;
  std::unique_ptr<systems::Context<double>> context_;

  const RevoluteJoint<double>* hinge_joint_{nullptr};
};

// We assume that the MultibodyPlant boilerplate is adequately tested in our
// various regression tests; the tests here mostly just check that the math has
// the correct signs and rough magnitudes.
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

TEST_F(DoorHingeTest, ZeroTest) {
  DoorHingeConfig config = no_forces_config();
  DoorHingeTester dut = CreateDoorHingeTester(config);

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

TEST_F(DoorHingeTest, SpringTest) {
  DoorHingeConfig config = no_forces_config();
  config.spring_constant = 1;
  DoorHingeTester dut = CreateDoorHingeTester(config);

  // Springs make spring torque (but not friction).
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1., config), 0);
  EXPECT_LT(dut.CalcHingeSpringTorque(1., config), 0);  // Pulls toward zero.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);

  // Test potential energy with only a spring.
  const double potential_energy = dut.door_hinge().CalcPotentialEnergy(
      *context_, tree().EvalPositionKinematics(*context_));

  const double calculated_potential_energy =
      0.5 * config.spring_constant * (0 - config.spring_zero_angle_rad) *
      (0 - config.spring_zero_angle_rad);
  EXPECT_EQ(potential_energy, calculated_potential_energy);
}

TEST_F(DoorHingeTest, CatchTest) {
  DoorHingeConfig config = no_forces_config();
  config.catch_width = 1;
  config.catch_torque = 1;
  DoorHingeTester dut = CreateDoorHingeTester(config);

  // The catch makes spring torque (but not friction).
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1., config), 0);

  EXPECT_GT(dut.CalcHingeSpringTorque(1., config), 0);   // Resists closing.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0.5, config), 0);  // Tipover point.
  EXPECT_LT(dut.CalcHingeSpringTorque(0., config), 0);   // Detent pulls in.

  // The potential energy with only the catch torque at zero position and the
  // catch_width position should be zero.
  const double potential_energy_q0 = dut.door_hinge().CalcPotentialEnergy(
      *context_, tree().EvalPositionKinematics(*context_));
  EXPECT_EQ(potential_energy_q0, 0.0);

  SetHingeJointState(config.catch_width, 0.0);
  const double potential_energy_qc = dut.door_hinge().CalcPotentialEnergy(
      *context_, tree().EvalPositionKinematics(*context_));
  EXPECT_EQ(potential_energy_q0, potential_energy_qc);

  // Compute the maximum energy when the door opens at the half center
  // position.
  auto singlet = [](double t, double x) {
    return 1 - tanh(x / t) * tanh(x / t);
  };
  const double catch_center = config.catch_width / 2;
  const double computed_potential_energy =
      config.catch_torque * catch_center *
      (singlet(catch_center, 0.0) - singlet(catch_center, -catch_center));
  SetHingeJointState(catch_center, 0.0);
  const double potential_energy_half_qc = dut.door_hinge().CalcPotentialEnergy(
      *context_, tree().EvalPositionKinematics(*context_));
  EXPECT_EQ(potential_energy_half_qc, computed_potential_energy);
}

TEST_F(DoorHingeTest, StaticFrictionTest) {
  DoorHingeConfig config = no_forces_config();
  config.static_friction_torque = 1;
  DoorHingeTester dut = CreateDoorHingeTester(config);

  // Friction opposes tiny motion, but falls away with substantial motion.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_LT(dut.CalcHingeFrictionalTorque(0.001, config), -0.5);
  EXPECT_GT(dut.CalcHingeFrictionalTorque(-0.001, config), 0.5);
  EXPECT_NEAR(dut.CalcHingeFrictionalTorque(0.01, config), 0, 1e-7);

  // No spring torque.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1., config), 0);
}

TEST_F(DoorHingeTest, DynamicFrictionTest) {
  DoorHingeConfig config = no_forces_config();
  config.dynamic_friction_torque = 1;
  DoorHingeTester dut = CreateDoorHingeTester(config);

  // Friction opposes any motion, even tiny motion.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_LT(dut.CalcHingeFrictionalTorque(0.001, config), -0.5);
  EXPECT_GT(dut.CalcHingeFrictionalTorque(-0.001, config), 0.5);
  EXPECT_NEAR(dut.CalcHingeFrictionalTorque(0.01, config), -1, 1e-7);

  // No spring torque.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1., config), 0);
}

TEST_F(DoorHingeTest, ViscousFrictionTest) {
  DoorHingeConfig config = no_forces_config();
  config.viscous_friction = 1;
  DoorHingeTester dut = CreateDoorHingeTester(config);

  // Friction opposes motion proprotionally.
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(1., config), -1);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(-1., config), 1);
  EXPECT_EQ(dut.CalcHingeFrictionalTorque(-2., config), 2);

  // No spring torque.
  EXPECT_EQ(dut.CalcHingeSpringTorque(0., config), 0);
  EXPECT_EQ(dut.CalcHingeSpringTorque(1., config), 0);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
