#include "drake/multibody/plant/distance_constraint_params.h"

#include <limits>

#include <gtest/gtest.h>

using Eigen::Vector3d;

namespace drake {
namespace multibody {

GTEST_TEST(DistanceConstraintParams, BasicApi) {
  const Vector3d p_AP(1, 2, 3);
  const Vector3d p_BQ(4, 5, 6);
  const double distance = 1.2;
  const double stiffness = 1.3e7;
  const double damping = 0.5;
  const BodyIndex bodyA(1);
  const BodyIndex bodyB(2);
  const DistanceConstraintParams dut(bodyA, p_AP, bodyB, p_BQ, distance,
                                     stiffness, damping);
  EXPECT_EQ(dut.bodyA(), bodyA);
  EXPECT_EQ(dut.bodyB(), bodyB);
  EXPECT_EQ(dut.p_AP(), p_AP);
  EXPECT_EQ(dut.p_BQ(), p_BQ);
  EXPECT_EQ(dut.distance(), distance);
  EXPECT_EQ(dut.stiffness(), stiffness);
  EXPECT_EQ(dut.damping(), damping);
}

GTEST_TEST(DistanceConstraintParams, VerifyInvariants) {
  const Vector3d p_AP(1, 2, 3);
  const Vector3d p_BQ(4, 5, 6);
  const double distance = 1.2;
  const double stiffness = 1.3e7;
  const double damping = 0.5;
  const BodyIndex bodyA(1);
  const BodyIndex bodyB(2);

  // Same body indexes throw.
  EXPECT_THROW(DistanceConstraintParams(bodyA, p_AP, bodyA, p_BQ, distance,
                                        stiffness, damping),
               std::exception);

  // Invalid body A index throws.
  EXPECT_THROW(DistanceConstraintParams(BodyIndex(), p_AP, bodyB, p_BQ,
                                        distance, stiffness, damping),
               std::exception);

  // Invalid body B index throws.
  EXPECT_THROW(DistanceConstraintParams(bodyA, p_AP, BodyIndex(), p_BQ,
                                        distance, stiffness, damping),
               std::exception);

  // Negative distance throws.
  EXPECT_THROW(DistanceConstraintParams(bodyA, p_AP, bodyB, p_BQ, -1.0,
                                        stiffness, damping),
               std::exception);

  // Zero distance throws.
  EXPECT_THROW(DistanceConstraintParams(bodyA, p_AP, bodyB, p_BQ, 0.0,
                                        stiffness, damping),
               std::exception);

  // Valid set of parameters.
  EXPECT_NO_THROW(DistanceConstraintParams(bodyA, p_AP, bodyB, p_BQ, distance,
                                           stiffness, damping));

  // Infinite stiffness is valid.
  EXPECT_NO_THROW(DistanceConstraintParams(
      bodyA, p_AP, bodyB, p_BQ, distance,
      std::numeric_limits<double>::infinity(), damping));

  // Zero stiffness throws.
  EXPECT_THROW(DistanceConstraintParams(bodyA, p_AP, bodyB, p_BQ, distance, 0.0,
                                        damping),
               std::exception);

  // Negative stiffness throws.
  EXPECT_THROW(DistanceConstraintParams(bodyA, p_AP, bodyB, p_BQ, distance,
                                        -1.5, damping),
               std::exception);

  // Negative damping throws.
  EXPECT_THROW(DistanceConstraintParams(bodyA, p_AP, bodyB, p_BQ, distance,
                                        stiffness, -2.5),
               std::exception);

  // Zero damping is valid.
  EXPECT_NO_THROW(DistanceConstraintParams(bodyA, p_AP, bodyB, p_BQ, distance,
                                           stiffness, 0.0));
}

}  // namespace multibody
}  // namespace drake
