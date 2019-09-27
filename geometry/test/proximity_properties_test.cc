#include "drake/geometry/proximity_properties.h"

#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_roles.h"

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(ProximityPropertiesTest, MakeUnitPressureField) {
  PressureField field = MakeUnitPressureField();

  for (double e : {0.0, 0.25, 0.6666, 1.0}) {
    EXPECT_EQ(field.p0(e), e);
    EXPECT_EQ(field.dp0_de(e), 1.0);
  }
}

GTEST_TEST(ProximityPropertiesTest, MakeLinearPressureField) {
  // Case: Various valid values of elasticity.
  for (double E : {0.25, 50.0, 1e7}) {
    PressureField field = MakeLinearPressureField(E);

    for (double e : {0.0, 0.25, 0.6666, 1.0}) {
      EXPECT_EQ(field.p0(e), E * e);
      EXPECT_EQ(field.dp0_de(e), E);
    }
  }

  // Error case: zero elasticity.
  {
    DRAKE_EXPECT_THROWS_MESSAGE(
        MakeLinearPressureField(0.0), std::logic_error,
        "A linear static pressure field requires a finite, positive value.*");
  }

  // Error case: negative elasticity.
  {
    DRAKE_EXPECT_THROWS_MESSAGE(
        MakeLinearPressureField(-1.3), std::logic_error,
        "A linear static pressure field requires a finite, positive value.*");
  }

  // Error case: infinite elasticity.
  {
    DRAKE_EXPECT_THROWS_MESSAGE(
        MakeLinearPressureField(std::numeric_limits<double>::infinity()),
        std::logic_error,
        "A linear static pressure field requires a finite, positive value.*");
  }
}

GTEST_TEST(ProximityPropertiesTest, AddRigidProperties) {
  for (double length : {1e-5, 1.25, 1e7}) {
    ProximityProperties props;
    AddRigidHydroelasticProperties(length, &props);
    EXPECT_TRUE(props.HasProperty(kHydroGroup, kRezHint));
    EXPECT_EQ(props.GetProperty<double>(kHydroGroup, kRezHint), length);
  }
}

// Tests the variant where the static pressure field is defined by the
// enumeration.
GTEST_TEST(ProximityPropertiesTest, AddSoftProperties_PressureModel) {
  for (PressureModel pressure_model :
       {PressureModel::kUnit, PressureModel::kLinear}) {
    for (double length : {1e-5, 1.25, 1e7}) {
      ProximityProperties props;
      const double E = 1.5e8;
      props.AddProperty(kMaterialGroup, kElastic, E);
      AddSoftHydroelasticProperties(length, pressure_model, &props);
      EXPECT_TRUE(props.HasProperty(kHydroGroup, kRezHint));
      EXPECT_EQ(props.GetProperty<double>(kHydroGroup, kRezHint), length);
      EXPECT_TRUE(props.HasProperty(kHydroGroup, kPressure));
      const auto& test_field =
          props.GetProperty<PressureField>(kHydroGroup, kPressure);
      PressureField expected_field = MakePressureField(pressure_model, E);
      for (double extent : {0.25, 0.666666}) {
        EXPECT_EQ(test_field.p0(extent), expected_field.p0(extent));
        EXPECT_EQ(test_field.dp0_de(extent), expected_field.dp0_de(extent));
      }
    }
  }
}

// Tests the variant where the static pressure is given explicitly. This doesn't
// vigorously test multiple values of the other fields. We assume that it's been
// tested already, and we just want to make sure the custom pressure field comes
// through.
GTEST_TEST(ProximityPropertiesTest, AddSoftProperties_StaticPressure) {
  PressureField expected_field = {[](double e) { return e * e; },
                                  [](double e) { return 2 * e; }};
  ProximityProperties props;
  // NOTE: elastic modulus explicitly is different from that implied in the
  // expected field (2); when a field is provided, it won't depend on elastic
  // modulus.
  props.AddProperty(kMaterialGroup, kElastic, 1e8);
  const double length = 0.25;
  AddSoftHydroelasticProperties(length, expected_field, &props);
  EXPECT_TRUE(props.HasProperty(kHydroGroup, kRezHint));
  EXPECT_EQ(props.GetProperty<double>(kHydroGroup, kRezHint), length);
  EXPECT_TRUE(props.HasProperty(kHydroGroup, kPressure));
  const auto& test_field =
      props.GetProperty<PressureField>(kHydroGroup, kPressure);
  for (double extent : {0.25, 0.666666}) {
    EXPECT_EQ(test_field.p0(extent), expected_field.p0(extent));
    EXPECT_EQ(test_field.dp0_de(extent), expected_field.dp0_de(extent));
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
