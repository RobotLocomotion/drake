#include "drake/multibody/plant/coulomb_friction.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace {

// Verifies the default constructor for a frictionless surface model, i.e. both
// static and dynamic friction coefficients must be zero.
GTEST_TEST(CoulombFriction, DefaultConstructor) {
  CoulombFriction<double> friction;
  EXPECT_EQ(friction.static_friction(), 0);
  EXPECT_EQ(friction.dynamic_friction(), 0);
}

// Verifies construction from a set of friction coefficients.
GTEST_TEST(CoulombFriction, ConstructionFromFrictionCoefficients) {
  const double kStaticFriction = 0.8;
  const double kDynamicFriction = 0.3;

  // Verify constructor throws for a negative static friction coefficient.
  DRAKE_EXPECT_THROWS_MESSAGE(
      CoulombFriction<double>(-kStaticFriction, kDynamicFriction),
      std::logic_error,
      "The given static friction is negative: .*");

  // Verify constructor throws for a negative dynamic friction coefficient.
  DRAKE_EXPECT_THROWS_MESSAGE(
      CoulombFriction<double>(kStaticFriction, -kDynamicFriction),
      std::logic_error,
      "The given dynamic friction is negative: .*");

  // Verify constructor throws when the dynamic friction coefficient is larger
  // than the static friction coefficient.
  DRAKE_EXPECT_THROWS_MESSAGE(
      CoulombFriction<double>(kDynamicFriction, kStaticFriction),
      std::logic_error,
      "The given dynamic friction \\(.*\\) is greater than the given static "
      "friction \\(.*\\); dynamic friction must be less than or equal to "
      "static friction.");

  CoulombFriction<double> friction(kStaticFriction, kDynamicFriction);
  EXPECT_EQ(friction.static_friction(), kStaticFriction);
  EXPECT_EQ(friction.dynamic_friction(), kDynamicFriction);
}

GTEST_TEST(CoulombFriction, EqualityOperator) {
  // They are equal.
  EXPECT_TRUE(
      CoulombFriction<double>(0.8, 0.5) == CoulombFriction<double>(0.8, 0.5));
  // Static friction coefficient differs.
  EXPECT_FALSE(
      CoulombFriction<double>(0.8, 0.5) == CoulombFriction<double>(1.2, 0.5));
  // Dynamic friction coefficient differs.
  EXPECT_FALSE(
      CoulombFriction<double>(0.8, 0.5) == CoulombFriction<double>(0.8, 0.3));
  // Both friction coefficients differ.
  EXPECT_FALSE(
      CoulombFriction<double>(0.8, 0.5) == CoulombFriction<double>(1.2, 0.6));
}

// Verify CoulombFriction::CalcContactFrictionFromSurfaceProperties().
GTEST_TEST(CoulombFriction, CalcContactFrictionFromSurfaceProperties) {
  auto combine_friction_coefficients = [](double mu1, double mu2) {
    return 2.0 * mu1 * mu2 / (mu1 + mu2);
  };

  // Friction coefficients for surface 1.
  const double mu_s1 = 0.8;
  const double mu_d1 = 0.3;
  CoulombFriction<double> friction1(mu_s1, mu_d1);
  // Friction coefficients for surface 2.
  const double mu_s2 = 0.7;
  const double mu_d2 = 0.5;
  CoulombFriction<double> friction2(mu_s2, mu_d2);

  // Verify correctness.
  CoulombFriction<double> friction1_with_friction2 =
      CalcContactFrictionFromSurfaceProperties(friction1, friction2);
  EXPECT_EQ(friction1_with_friction2.static_friction(),
            combine_friction_coefficients(mu_s1, mu_s2));
  EXPECT_EQ(friction1_with_friction2.dynamic_friction(),
            combine_friction_coefficients(mu_d1, mu_d2));

  // Verify the operation is commutative.
  CoulombFriction<double> friction2_with_friction1 =
      CalcContactFrictionFromSurfaceProperties(friction2, friction1);
  EXPECT_TRUE(
      friction1_with_friction2 == friction2_with_friction1);

  // Verify result when one of the surfaces is frictionless.
  CoulombFriction<double> friction1_with_frictionless =
      CalcContactFrictionFromSurfaceProperties(friction1,
                                               CoulombFriction<double>());
  EXPECT_TRUE(
      friction1_with_frictionless == CoulombFriction<double>());

  // Verify commutativity when one of the surfaces is frictionless.
  CoulombFriction<double> frictionless_with_friction1 =
      CalcContactFrictionFromSurfaceProperties(CoulombFriction<double>(),
                                               friction1);
  EXPECT_TRUE(
      friction1_with_frictionless == frictionless_with_friction1);
}

// Verify that the result of combining two identical surfaces returns the
// Coulomb friction coefficients of the interacting surfaces.
GTEST_TEST(CoulombFriction, SurfacesAreIdentical) {
  CoulombFriction<double> surface1(0.8, 0.3);
  CoulombFriction<double> surface2(surface1);  // A second identical surface.
  CoulombFriction<double> surface1_with_surface2 =
      CalcContactFrictionFromSurfaceProperties(surface1, surface2);
  EXPECT_NEAR(
      surface1.static_friction(), surface1_with_surface2.static_friction(),
      5 * std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(
      surface1.dynamic_friction(), surface1_with_surface2.dynamic_friction(),
      5 * std::numeric_limits<double>::epsilon());
}

// Verify the implementation handles the case of two frictionless surfaces
// correctly (handling of a 0/0 division).
GTEST_TEST(CoulombFriction, BothSurfacesAreFrictionless) {
  CoulombFriction<double> frictionless_surface1;
  CoulombFriction<double> frictionless_surface2;
  // Verify the surfaces are indeed frictionless.
  EXPECT_EQ(frictionless_surface1.dynamic_friction(), 0);
  EXPECT_EQ(frictionless_surface1.static_friction(), 0);
  EXPECT_TRUE(
      frictionless_surface1 == frictionless_surface2);

  CoulombFriction<double> frictionless_with_frictionless =
      CalcContactFrictionFromSurfaceProperties(frictionless_surface1,
                                               frictionless_surface2);
  // The result should be that of a frictionless surface pair.
  EXPECT_TRUE(
      frictionless_with_frictionless == CoulombFriction<double>());
}

}  // namespace
}  // namespace multibody
}  // namespace drake

