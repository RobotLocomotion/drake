#include "drake/multibody/tree/door_hinge.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/revolute_joint.h"

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
    auto& body1 = model_.AddBody<RigidBody>(SpatialInertia<double>());
    model_.AddJoint<RevoluteJoint>(
        "Joint1", model_.world_body(), std::nullopt, body1, std::nullopt,
        Eigen::Vector3d::UnitZ(), kPositionLowerLimit, kPositionUpperLimit,
        kDamping);
  }

  const RevoluteJoint<double>& joint() {
    return dynamic_cast<const RevoluteJoint<double>&>(
        model_.get_joint(JointIndex(0)));
  }

  DoorHingeTester CreateDoorHingeTester(const DoorHingeConfig& config) {
    DoorHinge<double> door_hinge(joint(), config);
    return DoorHingeTester(door_hinge);
  }

 protected:
  internal::MultibodyTree<double> model_{};
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
