#include "drake/multibody/tree/door_hinge.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace {

namespace dut = drake::multibody::internal;

// We assume that the MultibodyPlant boilerplate is adequately tested in our
// various regression tests; the tests here mostly just check that my math has
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

GTEST_TEST(DoorHingeTest, ZeroTest) {
  DoorHingeConfig config = no_forces_config();
  // If no frictions, springs, etc. are applied, our torques should be 0.
  EXPECT_EQ(dut::hinge_frictional_torque(0., 0., config), 0);
  EXPECT_EQ(dut::hinge_frictional_torque(0., 1., config), 0);
  EXPECT_EQ(dut::hinge_frictional_torque(1., 0., config), 0);
  EXPECT_EQ(dut::hinge_spring_torque(0., 0., config), 0);
  EXPECT_EQ(dut::hinge_spring_torque(0., 1., config), 0);
  EXPECT_EQ(dut::hinge_spring_torque(1., 0., config), 0);
}

GTEST_TEST(DoorHingeTest, SpringTest) {
  DoorHingeConfig config = no_forces_config();
  config.spring_constant = 1;

  // Springs make spring torque (but not friction).
  EXPECT_EQ(dut::hinge_frictional_torque(0., 0., config), 0);
  EXPECT_EQ(dut::hinge_frictional_torque(0., 1., config), 0);
  EXPECT_EQ(dut::hinge_frictional_torque(1., 0., config), 0);
  EXPECT_EQ(dut::hinge_spring_torque(0., 0., config), 0);
  EXPECT_LT(dut::hinge_spring_torque(1., 0., config), 0);  // Pulls toward zero.
  EXPECT_EQ(dut::hinge_spring_torque(0., 1., config), 0);
}

GTEST_TEST(DoorHingeTest, CatchTest) {
  DoorHingeConfig config = no_forces_config();
  config.catch_width = 1;
  config.catch_torque = 1;

  // The catch makes spring torque (but not friction).
  EXPECT_EQ(dut::hinge_frictional_torque(0., 0., config), 0);
  EXPECT_EQ(dut::hinge_frictional_torque(0., 1., config), 0);
  EXPECT_EQ(dut::hinge_frictional_torque(1., 0., config), 0);

  EXPECT_GT(dut::hinge_spring_torque(1., 0., config), 0);   // Resists closing.
  EXPECT_EQ(dut::hinge_spring_torque(0.5, 0., config), 0);  // Tipover point.
  EXPECT_EQ(dut::hinge_spring_torque(0.5, 1., config), 0);  // 0 dynamic force.
  EXPECT_LT(dut::hinge_spring_torque(0., 0., config), 0);   // Detent pulls in.
}

GTEST_TEST(DoorHingeTest, StaticFrictionTest) {
  DoorHingeConfig config = no_forces_config();
  config.static_friction_torque = 1;

  // Friction opposes tiny motion, but falls away with substantial motion.
  EXPECT_EQ(dut::hinge_frictional_torque(0., 0., config), 0);
  EXPECT_LT(dut::hinge_frictional_torque(0., 0.001, config), -0.5);
  EXPECT_GT(dut::hinge_frictional_torque(0., -0.001, config), 0.5);
  EXPECT_NEAR(dut::hinge_frictional_torque(0., 0.01, config), 0, 1e-7);

  // No spring torque.
  EXPECT_EQ(dut::hinge_spring_torque(0., 0., config), 0);
  EXPECT_EQ(dut::hinge_spring_torque(0., 1., config), 0);
  EXPECT_EQ(dut::hinge_spring_torque(1., 0., config), 0);
}

GTEST_TEST(DoorHingeTest, DynamicFrictionTest) {
  DoorHingeConfig config = no_forces_config();
  config.dynamic_friction_torque = 1;

  // Friction opposes any motion, even tiny motion.
  EXPECT_EQ(dut::hinge_frictional_torque(0., 0., config), 0);
  EXPECT_LT(dut::hinge_frictional_torque(0., 0.001, config), -0.5);
  EXPECT_GT(dut::hinge_frictional_torque(0., -0.001, config), 0.5);
  EXPECT_NEAR(dut::hinge_frictional_torque(0., 0.01, config), -1, 1e-7);

  // No spring torque.
  EXPECT_EQ(dut::hinge_spring_torque(0., 0., config), 0);
  EXPECT_EQ(dut::hinge_spring_torque(0., 1., config), 0);
  EXPECT_EQ(dut::hinge_spring_torque(1., 0., config), 0);
}

GTEST_TEST(DoorHingeTest, ViscousFrictionTest) {
  DoorHingeConfig config = no_forces_config();
  config.viscous_friction = 1;

  // Friction opposes motion proprotionally.
  EXPECT_EQ(dut::hinge_frictional_torque(0., 0., config), 0);
  EXPECT_EQ(dut::hinge_frictional_torque(0., 1., config), -1);
  EXPECT_EQ(dut::hinge_frictional_torque(0., -1., config), 1);
  EXPECT_EQ(dut::hinge_frictional_torque(0., -2., config), 2);

  // No spring torque.
  EXPECT_EQ(dut::hinge_spring_torque(0., 0., config), 0);
  EXPECT_EQ(dut::hinge_spring_torque(0., 1., config), 0);
  EXPECT_EQ(dut::hinge_spring_torque(1., 0., config), 0);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
