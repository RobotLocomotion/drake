#include "drake/multibody/rigid_body_plant/compliant_contact_parameters.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {

// Utility function to create a "baked" version of a parameter set. The new
// instance reports the same values as the input, but no longer depends
// on or changes with the default values.
CompliantContactParameters bake(const CompliantContactParameters& input) {
  CompliantContactParameters parameters;
  parameters.set_stiffness(input.stiffness());
  parameters.set_dissipation(input.dissipation());
  parameters.set_friction(input.static_friction(),
                           input.dynamic_friction());

  EXPECT_EQ(parameters.stiffness(), input.stiffness());
  EXPECT_EQ(parameters.dissipation(), input.dissipation());
  EXPECT_EQ(parameters.static_friction(), input.static_friction());
  EXPECT_EQ(parameters.dynamic_friction(), input.dynamic_friction());
  return parameters;
}

// NOTE: this does *not* test the parsing functionality. See
// drake/multibody/parsers/test/compliant_parameters_parse_test.cc for the
// testing of *that* functionality.

// Test the setting of *valid* values for the various parameters. Confirm they
// change.
GTEST_TEST(CompliantContactParameters, SetValidValues) {
  CompliantContactParameters params;
  CompliantContactParameters original = bake(params);

  params.set_stiffness(params.stiffness() + 1);
  EXPECT_EQ(params.stiffness(), original.stiffness() + 1);
  params.set_dissipation(params.dissipation() + 1);
  EXPECT_EQ(params.dissipation(), original.dissipation() + 1);
  params.set_friction(params.static_friction() + 1,
                      params.dynamic_friction() + 1);
  EXPECT_EQ(params.static_friction(), original.static_friction() + 1);
  EXPECT_EQ(params.dynamic_friction(), original.dynamic_friction() + 1);
}

// Confirms exceptions are thrown for bad parameter values.
GTEST_TEST(CompliantContactParameters, SetInvalidValues) {
  CompliantContactParameters params;
  EXPECT_THROW(params.set_stiffness(0), std::runtime_error);
  EXPECT_THROW(params.set_stiffness(-1), std::runtime_error);
  EXPECT_THROW(params.set_dissipation(-1), std::runtime_error);
  EXPECT_THROW(params.set_friction(-1), std::runtime_error);
  EXPECT_THROW(params.set_friction(-1, -2), std::runtime_error);
  EXPECT_THROW(params.set_friction(1, -2), std::runtime_error);
  EXPECT_THROW(params.set_friction(-1, 2), std::runtime_error);
  EXPECT_THROW(params.set_friction(1, 2), std::runtime_error);
}

// Tests the value evaluation of the parameter set:
//   1. Confirms that a newly constructed instance merely echoes the default
//      values.
//   2. Confirms that the default values can be successfully changed.
//   3. Confirms that unspecified values change with default values (whether
//      they were instantiated before or after the default values are set).
//   4. Confirms specified values don't change with default values.
GTEST_TEST(CompliantContactParameters, DefaultValues) {
  // Pick ridiculous values that *cannot* be actual defaults.
  double kStiffness = 1.111;
  double kDissipation = 17;
  double kFriction = 13;

  CompliantContactParameters param_change;
  param_change.set_dissipation(kDissipation);
  param_change.set_stiffness(kStiffness);
  param_change.set_friction(kFriction);

  // Parameter set created *before* changing default values -- however, no
  // values set, so it should echo the original defaults.
  CompliantContactParameters param_1;

  // The default values (represented by param_1) should not match the arbitrary
  // values set in param_change.
  EXPECT_NE(param_change.stiffness(), param_1.stiffness());
  EXPECT_NE(param_change.dissipation(), param_1.dissipation());
  EXPECT_NE(param_change.static_friction(), param_1.static_friction());
  EXPECT_NE(param_change.dynamic_friction(), param_1.dynamic_friction());

  // Change the defaults
  CompliantContactParameters::SetDefaultValues(param_change);

  // Parameter set created *after* changing default values -- no values set
  // means it should echo the defaults.
  CompliantContactParameters param_2;

  // Now all default values should echo the newly changed default values.
  EXPECT_EQ(param_2.stiffness(), kStiffness);
  EXPECT_EQ(param_2.dissipation(), kDissipation);
  EXPECT_EQ(param_2.static_friction(), kFriction);
  EXPECT_EQ(param_2.dynamic_friction(), kFriction);
  EXPECT_EQ(param_1.stiffness(), kStiffness);
  EXPECT_EQ(param_1.dissipation(), kDissipation);
  EXPECT_EQ(param_1.static_friction(), kFriction);
  EXPECT_EQ(param_1.dynamic_friction(), kFriction);
}

}  // namespace
}  // namespace systems
}  // namespace drake
