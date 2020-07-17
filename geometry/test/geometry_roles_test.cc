#include "drake/geometry/geometry_roles.h"

#include <set>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/hydroelastic_type.h"
#include "drake/geometry/proximity/tessellation_strategy.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace geometry {
namespace {

using geometry::internal::HydroelasticType;
using geometry::internal::TessellationStrategy;
using multibody::CoulombFriction;

/* Confirms that all of the canonical properties listed in the
 ProximityPropperties documentation return the expected value. This should be
 kept in sync with the documentation *and* the implemented static methods. */
GTEST_TEST(ProximityPropertiesTest, CanonicalProperties) {
  EXPECT_EQ(ProximityProperties::material_elastic_modulus(),
            PropertyName("material", "elastic_modulus"));
  EXPECT_EQ(ProximityProperties::material_coulomb_friction(),
            PropertyName("material", "coulomb_friction"));
  EXPECT_EQ(ProximityProperties::material_hunt_crossley_dissipation(),
            PropertyName("material", "hunt_crossley_dissipation"));
  EXPECT_EQ(ProximityProperties::material_point_contact_stiffness(),
            PropertyName("material", "point_contact_stiffness"));
  EXPECT_EQ(ProximityProperties::hydroelastic_resolution_hint(),
            PropertyName("hydroelastic", "resolution_hint"));
  EXPECT_EQ(ProximityProperties::hydroelastic_slab_thickness(),
            PropertyName("hydroelastic", "slab_thickness"));
  EXPECT_EQ(ProximityProperties::hydroelastic_compliance_type(),
            PropertyName("hydroelastic", "compliance_type"));
  EXPECT_EQ(ProximityProperties::hydrolastic_tessellation_strategy(),
            PropertyName("hydroelastic", "tessellation_strategy"));
}

/* Tests the named _canonical_ property for validation. All canonical properties
 should validate for *type*. Some also validate for values of the correct type.

 @param property        The name of the property to test.
 @param valid_values    One or more values that should pass validation.
 @param wrong_value     An example of a value of the wrong type.
 @param bad_values      A collection of values that would *not* pass validation.
                        If the property doesn't validate value, leave this
                        empty.
*/
template <typename ValidType, typename WrongType>
void TestCanonicalProperty(const PropertyName& property,
                           const std::vector<ValidType>& valid_values,
                           const WrongType& wrong_value,
                           const std::vector<ValidType>& bad_values = {}) {
  static_assert(!std::is_same<ValidType, WrongType>::value,
                "The test requires ValidType and WrongType to be different");
  DRAKE_DEMAND(valid_values.size() > 0);

  ProximityProperties props;

  EXPECT_FALSE(props.HasProperty(property));
  // The wrong type should throw.
  DRAKE_EXPECT_THROWS_MESSAGE(
      props.Add(property, wrong_value), std::logic_error,
      fmt::format("Failed to validate property {}; expected type.+", property));
  // Invalid values of the right type should throw.
  for (const auto& bad_value : bad_values) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        props.Add(property, bad_value), std::logic_error,
        fmt::format("Failed to validate property {}; value.+", property));
  }
  // Valid values should be content.
  for (const auto& valid_value : valid_values) {
    EXPECT_NO_THROW(props.Add(property, valid_value));
    EXPECT_TRUE(props.HasProperty(property));
    EXPECT_EQ(props.Get<ValidType>(property), valid_value);
    // Clean it up so that the next good value can be added without error.
    props.Remove(property);
  }
}

GTEST_TEST(ProximityPropertiesTest, MaterialElasticModulus) {
  SCOPED_TRACE("ProximityProperty::material_elastic_modulus");
  // Elastic modulus must be strictly positive; so we'll test zero and a
  // negative value as bad.
  TestCanonicalProperty<double>(ProximityProperties::material_elastic_modulus(),
                                {3.5}, "wrong_value", {0.0, -1.5});
}

GTEST_TEST(ProximityPropertiesTest, MaterialCoulombFriction) {
  SCOPED_TRACE("ProximityProperty::material_coulomb_friction");
  // Coulomb friction doesn't validate values (the CoulombFriction struct has
  // already done that.))
  const CoulombFriction<double> expected{0.2, 0.1};
  TestCanonicalProperty<CoulombFriction<double>>(
      ProximityProperties::material_coulomb_friction(), {expected},
      "wrong_value");
}

GTEST_TEST(ProximityPropertiesTest, MaterialHuntCrossleyDissipation) {
  SCOPED_TRACE("ProximityProperty::material_hunt_crossley_dissipation");
  // Dissippation can't be negative; so we'll test zero and a a positive value
  // as good values.
  TestCanonicalProperty<double>(
      ProximityProperties::material_hunt_crossley_dissipation(), {0, 1.75},
      "wrong_value", {-1.5});
}

GTEST_TEST(ProximityPropertiesTest, MaterialPointContactStiffness) {
  SCOPED_TRACE("ProximityProperty::material_point_contact_stiffness");
  // Point stiffness must be strictly positive; so we'll test zero and a
  // negative value as bad.
  TestCanonicalProperty<double>(
      ProximityProperties::material_point_contact_stiffness(), {3.5},
      "wrong_value", {0.0, -1.5});
}

GTEST_TEST(ProximityPropertiesTest, HydroelasticResolutionHint) {
  SCOPED_TRACE("ProximityProperty::hydroelastic_resolution_hint");
  // The resolution hint must be strictly positive; so we'll test zero and a
  // negative value as bad.
  TestCanonicalProperty<double>(
      ProximityProperties::hydroelastic_resolution_hint(), {3.5},
      "wrong_value", {0.0, -1.5});
}

GTEST_TEST(ProximityPropertiesTest, HydroelasticSlabThickness) {
  SCOPED_TRACE("ProximityProperty::hydroelastic_slab_thickness");
  // The slab thickness must be strictly positive; so we'll test zero and a
  // negative value as bad.
  TestCanonicalProperty<double>(
      ProximityProperties::hydroelastic_slab_thickness(), {3.5},
      "wrong_value", {0.0, -1.5});
}

GTEST_TEST(ProximityPropertiesTest, HydroelasticComplianceType) {
  SCOPED_TRACE("ProximityProperty::hydroelastic_compliance_type");
  // The compliance type is valid for soft and rigid, and not for undefined.
  TestCanonicalProperty<HydroelasticType>(
      ProximityProperties::hydroelastic_compliance_type(),
      {HydroelasticType::kRigid, HydroelasticType::kSoft}, "wrong_value",
      {HydroelasticType::kUndefined});
}

GTEST_TEST(ProximityPropertiesTest, HydroelasticTessellationStrategy) {
  SCOPED_TRACE("ProximityProperty::hydrolastic_tessellation_strategy");
  // The tessellation strategy is valid for all enumerations. We'll just pick
  // one.
  TestCanonicalProperty<TessellationStrategy>(
      ProximityProperties::hydrolastic_tessellation_strategy(),
      {TessellationStrategy::kSingleInteriorVertex}, "wrong_value");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
