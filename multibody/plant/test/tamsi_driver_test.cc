#include "drake/multibody/plant/tamsi_driver.h"

#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/multibody/plant/test_utilities/rigid_body_on_compliant_ground.h"

constexpr double kEps = std::numeric_limits<double>::epsilon();

namespace drake {
namespace multibody {
namespace internal {

// This test verifies contact results in the equilibrium configuration.
TEST_P(RigidBodyOnCompliantGround, VerifyEquilibriumConfiguration) {
  const ContactTestConfig& config = GetParam();
  EXPECT_EQ(plant_->num_velocities(), 2);
  contact_solvers::internal::ContactSolverResults<double> results;
  tamsi_driver_->CalcContactSolverResults(*plant_context_, &results);

  EXPECT_EQ(results.v_next.size(), plant_->num_velocities());
  const int num_contacts = config.point_contact ? 1 : kNumberOfTriangles_;
  EXPECT_EQ(results.fn.size(), num_contacts);

  const double normal_force_expected = CalcBodyWeight();
  const double normal_force = results.fn.sum();
  EXPECT_NEAR(normal_force, normal_force_expected, kEps);
}

// Setup test cases using point and hydroelastic contact.
std::vector<ContactTestConfig> MakeTestCases() {
  return std::vector<ContactTestConfig>{
      {.description = "HydroelasticContact", .point_contact = false},
      {.description = "PointContact", .point_contact = true},
  };
}

INSTANTIATE_TEST_SUITE_P(TamsiDriverTests, RigidBodyOnCompliantGround,
                         testing::ValuesIn(MakeTestCases()),
                         testing::PrintToStringParamName());

}  // namespace internal
}  // namespace multibody
}  // namespace drake
