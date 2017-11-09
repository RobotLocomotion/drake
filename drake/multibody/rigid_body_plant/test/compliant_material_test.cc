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
  material.set_stiffness(input.stiffness());
  material.set_dissipation(input.dissipation());
  material.set_friction(input.static_friction(),
                        input.dynamic_friction());

  EXPECT_EQ(material.stiffness(), input.stiffness());
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
  EXPECT_TRUE(material.stiffness_is_default());
  EXPECT_TRUE(material.dissipation_is_default());
  EXPECT_TRUE(material.friction_is_default());

  material.set_stiffness(material.stiffness() + 1);
  material.set_dissipation(material.dissipation() + 1);
  material.set_friction(material.static_friction() + 1,
                        material.dynamic_friction() + 1);
  return material;
}

// NOTE: this does *not* test the parsing functionality. See
// drake/multibody/parsers/test/compliant_parameters_parse_test.cc for the
// testing of *that* functionality.

// Test the setting of *valid* values for the various parameters. Confirm they
// change.
GTEST_TEST(CompliantMaterialTest, SetValidValues) {
  CompliantMaterial material;
  CompliantMaterial original = Bake(material);

  material.set_stiffness(material.stiffness() + 1);
  EXPECT_EQ(material.stiffness(), original.stiffness() + 1);
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
  EXPECT_THROW(material.set_stiffness(0), std::runtime_error);
  EXPECT_THROW(material.set_stiffness(-1), std::runtime_error);
  EXPECT_THROW(material.set_dissipation(-1), std::runtime_error);
  EXPECT_THROW(material.set_friction(-1), std::runtime_error);
  EXPECT_THROW(material.set_friction(-1, -2), std::runtime_error);
  EXPECT_THROW(material.set_friction(1, -2), std::runtime_error);
  EXPECT_THROW(material.set_friction(-1, 2), std::runtime_error);
  EXPECT_THROW(material.set_friction(1, 2), std::runtime_error);
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
      [](const Material& mat) { return mat.stiffness(); },
      [](const Material& mat, double v) { return mat.stiffness(v); },
      [](Material* mat, double v) { mat->set_stiffness(v); });

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

}  // namespace
}  // namespace systems
}  // namespace drake
