#include "drake/geometry/proximity_properties.h"

#include <gtest/gtest.h>

#include "drake/geometry/geometry_roles.h"

namespace drake {
namespace geometry {
namespace {

using internal::HydroelasticType;

GTEST_TEST(ProximityPropertiesTest, AddRigidProperties) {
  for (double length : {1e-5, 1.25, 1e7}) {
    ProximityProperties props;
    AddRigidHydroelasticProperties(length, &props);
    EXPECT_TRUE(
        props.HasProperty(internal::kHydroGroup, internal::kComplianceType));
    EXPECT_EQ(props.GetProperty<HydroelasticType>(internal::kHydroGroup,
                                                  internal::kComplianceType),
              HydroelasticType::kRigid);
    EXPECT_TRUE(props.HasProperty(internal::kHydroGroup, internal::kRezHint));
    EXPECT_EQ(
        props.GetProperty<double>(internal::kHydroGroup, internal::kRezHint),
        length);
  }

  ProximityProperties props;
  AddRigidHydroelasticProperties(&props);
  EXPECT_TRUE(
      props.HasProperty(internal::kHydroGroup, internal::kComplianceType));
  EXPECT_EQ(props.GetProperty<HydroelasticType>(internal::kHydroGroup,
                                                internal::kComplianceType),
            HydroelasticType::kRigid);
}

// Tests the variant where the static pressure is given explicitly. This doesn't
// vigorously test multiple values of the other fields. We assume that it's been
// tested already, and we just want to make sure the custom pressure field comes
// through.
GTEST_TEST(ProximityPropertiesTest, AddSoftProperties) {
  for (double length : {1e-5, 1.25, 1e7}) {
    ProximityProperties props;
    AddSoftHydroelasticProperties(length, &props);
    EXPECT_TRUE(
        props.HasProperty(internal::kHydroGroup, internal::kComplianceType));
    EXPECT_EQ(props.GetProperty<HydroelasticType>(internal::kHydroGroup,
                                                  internal::kComplianceType),
              HydroelasticType::kSoft);
    EXPECT_TRUE(props.HasProperty(internal::kHydroGroup, internal::kRezHint));
    EXPECT_EQ(
        props.GetProperty<double>(internal::kHydroGroup, internal::kRezHint),
        length);
  }

  ProximityProperties props;
  AddSoftHydroelasticProperties(&props);
  EXPECT_TRUE(
      props.HasProperty(internal::kHydroGroup, internal::kComplianceType));
  EXPECT_EQ(props.GetProperty<HydroelasticType>(internal::kHydroGroup,
                                                internal::kComplianceType),
            HydroelasticType::kSoft);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
