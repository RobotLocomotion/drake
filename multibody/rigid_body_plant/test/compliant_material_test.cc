#include "drake/multibody/rigid_body_plant/compliant_material.h"

#include <functional>

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {

// Utility function to create a "baked" version of a parameter set. The new
// instance reports the same values as the input, but no longer depends
// on or changes with the default values.
CompliantMaterial Bake(const CompliantMaterial& input) {
  CompliantMaterial material;
  material.set_youngs_modulus(input.youngs_modulus());
  material.set_dissipation(input.dissipation());
  material.set_friction(input.static_friction(),
                        input.dynamic_friction());

  EXPECT_EQ(material.youngs_modulus(), input.youngs_modulus());
  EXPECT_EQ(material.dissipation(), input.dissipation());
  EXPECT_EQ(material.static_friction(), input.static_friction());
  EXPECT_EQ(material.dynamic_friction(), input.dynamic_friction());
  return material;
}

// Produce a set of CompliantMaterial that are guaranteed *not* to
// be the hard-coded default values.
CompliantMaterial CalcNonDefaultMaterial() {
  // NOTE: This relies on the fact that the default constructor leaves all
  // properties as default.
  CompliantMaterial material;
  EXPECT_TRUE(material.youngs_modulus_is_default());
  EXPECT_TRUE(material.dissipation_is_default());
  EXPECT_TRUE(material.friction_is_default());

  material.set_youngs_modulus(material.youngs_modulus() + 1);
  material.set_dissipation(material.dissipation() + 1);
  material.set_friction(material.static_friction() + 1,
                        material.dynamic_friction() + 1);
  return material;
}

// NOTE: this does *not* test the parsing functionality. See
// drake/multibody/parsers/test/compliant_parameters_parse_test.cc for the
// testing of *that* functionality.

GTEST_TEST(CompliantMaterialTest, FullConstructor) {
  // Valid construction
  CompliantMaterial valid{10, 1, 0.3, 0.1};
  EXPECT_EQ(valid.youngs_modulus(), 10);
  EXPECT_EQ(valid.dissipation(), 1);
  EXPECT_EQ(valid.static_friction(), 0.3);
  EXPECT_EQ(valid.dynamic_friction(), 0.1);
  EXPECT_FALSE(valid.youngs_modulus_is_default());
  EXPECT_FALSE(valid.dissipation_is_default());
  EXPECT_FALSE(valid.friction_is_default());

  // Invalid construction
  // Case: Zero Young's modulus.
  EXPECT_THROW(CompliantMaterial(0, 1, 0.3, 0.1), std::runtime_error);
  // Case: Negative Young's modulus.
  EXPECT_THROW(CompliantMaterial(-1, 1, 0.3, 0.1), std::runtime_error);
  // Case: Negative dissipation
  EXPECT_THROW(CompliantMaterial(10, -1, 0.3, 0.1), std::runtime_error);
  // Case: Negative static friction
  EXPECT_THROW(CompliantMaterial(10, 1, -0.3, 0.1), std::runtime_error);
  // Case: Negative dynamic friction
  EXPECT_THROW(CompliantMaterial(10, 1, 0.3, -0.1), std::runtime_error);
  // Case: dynamic friction > static friction
  EXPECT_THROW(CompliantMaterial(10, 1, 0.3, 0.5), std::runtime_error);
}

// Test the setting of *valid* values for the various parameters. Confirm they
// change.
GTEST_TEST(CompliantMaterialTest, SetValidValues) {
  CompliantMaterial material;
  CompliantMaterial original = Bake(material);

  material.set_youngs_modulus(material.youngs_modulus() + 1);
  EXPECT_EQ(material.youngs_modulus(), original.youngs_modulus() + 1);
  material.set_dissipation(material.dissipation() + 1);
  EXPECT_EQ(material.dissipation(), original.dissipation() + 1);
  material.set_friction(material.static_friction() + 1,
                        material.dynamic_friction() + 1);
  EXPECT_EQ(material.static_friction(), original.static_friction() + 1);
  EXPECT_EQ(material.dynamic_friction(), original.dynamic_friction() + 1);
}

// Confirms exceptions are thrown for bad parameter values.
GTEST_TEST(CompliantMaterialTest, SetInvalidValues) {
  CompliantMaterial material;
  EXPECT_THROW(material.set_youngs_modulus(0), std::runtime_error);
  EXPECT_THROW(material.set_youngs_modulus(-1), std::runtime_error);
  EXPECT_THROW(material.set_dissipation(-1), std::runtime_error);
  EXPECT_THROW(material.set_friction(-1), std::runtime_error);
  EXPECT_THROW(material.set_friction(-1, -2), std::runtime_error);
  EXPECT_THROW(material.set_friction(1, -2), std::runtime_error);
  EXPECT_THROW(material.set_friction(-1, 2), std::runtime_error);
  EXPECT_THROW(material.set_friction(1, 2), std::runtime_error);
}

// Confirms that non-default values can be reset to default.
GTEST_TEST(CompliantMaterialTest, ResetToDefault) {
  // NOTE: This relies on the fact that the default constructor leaves all
  // properties as default.
  const double test_value = 10.3;

  CompliantMaterial material;
  material.set_youngs_modulus(test_value);
  material.set_dissipation(test_value);
  material.set_friction(test_value);
  EXPECT_FALSE(material.youngs_modulus_is_default());
  EXPECT_FALSE(material.dissipation_is_default());
  EXPECT_FALSE(material.friction_is_default());

  EXPECT_EQ(material.youngs_modulus(), test_value);
  EXPECT_EQ(material.dissipation(), test_value);
  EXPECT_EQ(material.static_friction(), test_value);
  EXPECT_EQ(material.dynamic_friction(), test_value);

  material.set_youngs_modulus_to_default();
  material.set_dissipation_to_default();
  material.set_friction_to_default();

  EXPECT_TRUE(material.youngs_modulus_is_default());
  EXPECT_TRUE(material.dissipation_is_default());
  EXPECT_TRUE(material.friction_is_default());
}

// For each property, confirm:
//  1. Default access provides default value,
//  2. Access with given default provides given default,
//  3. After setting value explicitly,
//    1. Default access provides explicit value.
//    2. Given default access provides explicit value.
// NOTE: dut is being passed *by value* so that I can mutate the copy in the
// course of the test without modifying the original.
void TestSingleValueAccess(
    CompliantMaterial dut, const CompliantMaterial& defaults,
    const CompliantMaterial& non_defaults,
    std::function<double(const CompliantMaterial&)> get,
    std::function<double(const CompliantMaterial&, double)> get_or,
    std::function<void(CompliantMaterial*, double)> set) {
  const double default_value = get(defaults);
  const double non_default_value = get(non_defaults);
  EXPECT_NE(default_value, non_default_value);

  EXPECT_EQ(get(dut), default_value);
  EXPECT_EQ(get_or(dut, non_default_value), non_default_value);

  set(&dut, default_value + non_default_value);

  EXPECT_NE(get(dut), default_value);
  EXPECT_NE(get_or(dut, non_default_value), non_default_value);
}

// Confirm that accessing property values respects the default hierarchy:
//  Explicit values > given defaults > hard-coded defaults
GTEST_TEST(CompliantMaterialTest, PropertyAccessors) {
  // NOTE: This alias is purely cosmetic; it allows the lambda functions below
  // to all fit on single lines.
  using Material = CompliantMaterial;
  const Material defaults;
  // Create a set of values that *cannot* be the hard-coded default values.
  const Material non_defaults = CalcNonDefaultMaterial();

  // Test parameters; everything starts out defaulted.
  Material dut;

  // NOTE: This slightly convoluted method for testing this functionality allows
  // the following:
  //    1. The implementation of CompliantMaterial can use a default
  //       value in the property value accessors.
  //    2. Guarantees that all properties are tested the same way.
  TestSingleValueAccess(
      dut, defaults, non_defaults,
      [](const Material& mat) { return mat.youngs_modulus(); },
      [](const Material& mat, double v) { return mat.youngs_modulus(v); },
      [](Material* mat, double v) { mat->set_youngs_modulus(v); });

  TestSingleValueAccess(
      dut, defaults, non_defaults,
      [](const Material& mat) { return mat.dissipation(); },
      [](const Material& mat, double v) { return mat.dissipation(v); },
      [](Material* mat, double v) { mat->set_dissipation(v); });

  TestSingleValueAccess(
      dut, defaults, non_defaults,
      [](const Material& mat) { return mat.static_friction(); },
      [](const Material& mat, double v) { return mat.static_friction(v); },
      [](Material* mat, double v) { mat->set_friction(v); });

  TestSingleValueAccess(
      dut, defaults, non_defaults,
      [](const Material& mat) { return mat.dynamic_friction(); },
      [](const Material& mat, double v) { return mat.dynamic_friction(v); },
      [](Material* mat, double v) { mat->set_friction(v); });
}

// Simply tests that the hard-coded default values haven't changed. This is
// brittle by design. If the hard-coded default values change, this will be the
// minimum gentle reminder to update the doxygen.
GTEST_TEST(CompliantMaterialTest, DefaultValues) {
  CompliantMaterial defaults;
  EXPECT_EQ(defaults.youngs_modulus(), 1e8) << "Have you updated the doxygen?";
  EXPECT_EQ(defaults.dissipation(), 0.32) << "Have you updated the doxygen?";
  EXPECT_EQ(defaults.static_friction(), 0.9) << "Have you updated the doxygen?";
  EXPECT_EQ(defaults.dynamic_friction(), 0.5)
            << "Have you updated the doxygen?";
}

}  // namespace
}  // namespace systems
}  // namespace drake
