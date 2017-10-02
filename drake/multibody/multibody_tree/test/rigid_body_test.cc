#include "drake/multibody/multibody_tree/rigid_body.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace {

using Eigen::Vector3d;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Test rigid body constructor.
GTEST_TEST(RigidBody, RigidBodyConstructor) {
  // Construct a rigid body with a spatial inertia.
  const double mass = 2;
  const Vector3d p_BoBcm_B(0.4, 0.3, 0.2);
  const UnitInertia<double> U_BBo_B(6, 7, 8);
  const SpatialInertia<double> M_Bo_B(mass, p_BoBcm_B, U_BBo_B);
  const RigidBody<double> B(M_Bo_B);

  // Test that after construction, RigidBody class properly returns parts of its
  // default spatial inertia.
  EXPECT_EQ(B.get_default_mass(), mass);
  EXPECT_EQ(B.get_default_com(), p_BoBcm_B);
  const UnitInertia<double>& U_BBo_B_default = B.get_default_unit_inertia();
  EXPECT_EQ(U_BBo_B_default.get_moments(), U_BBo_B.get_moments());
  EXPECT_EQ(U_BBo_B_default.get_products(), U_BBo_B.get_products());

  // Test that RigidBody class properly calculates rotational inertia.
  const RotationalInertia<double> I_BBo_B_expected = mass * U_BBo_B;
  const RotationalInertia<double> I_BBo_B = B.get_default_rotational_inertia();
  EXPECT_TRUE(I_BBo_B.IsNearlyEqualTo(I_BBo_B_expected, 4.0*kEpsilon));
}

}  // namespace
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
