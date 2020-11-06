#include "drake/geometry/proximity_properties.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace geometry {
namespace {

using internal::HydroelasticType;
using internal::kComplianceType;
using internal::kElastic;
using internal::kFriction;
using internal::kHcDissipation;
using internal::kHydroGroup;
using internal::kMaterialGroup;
using internal::kPointStiffness;
using internal::kRezHint;
using internal::kSlabThickness;
using internal::PropName;
using CoulombFrictiond = multibody::CoulombFriction<double>;

GTEST_TEST(ProximityPropertiesTest, AddContactMaterial) {
  const double E = 1e8;
  const double d = 0.1;
  const double ps = 250.0;
  CoulombFrictiond mu{0.9, 0.5};

  // Case: Correct configuration.
  {
    ProximityProperties p;
    EXPECT_NO_THROW(AddContactMaterial(E, d, ps, mu, &p));
    EXPECT_EQ(p.Get<double>(PropName(kMaterialGroup, kElastic)), E);
    EXPECT_EQ(p.Get<double>(PropName(kMaterialGroup, kHcDissipation)), d);
    EXPECT_EQ(p.Get<double>(PropName(kMaterialGroup, kPointStiffness)), ps);
    const CoulombFrictiond& mu_stored =
        p.Get<CoulombFrictiond>(PropName(kMaterialGroup, kFriction));
    EXPECT_EQ(mu_stored.static_friction(), mu.static_friction());
    EXPECT_EQ(mu_stored.dynamic_friction(), mu.dynamic_friction());
  }

  // Error case: Already has elastic_modulus.
  {
    ProximityProperties p;
    p.Add(PropName(kMaterialGroup, kElastic), E);
    DRAKE_EXPECT_THROWS_MESSAGE(
        AddContactMaterial(E, d, ps, mu, &p), std::logic_error,
        ".+ Trying to add property .+ name already exists");
  }

  // Error case: Already has dissipation.
  {
    ProximityProperties p;
    p.Add(PropName(kMaterialGroup, kHcDissipation), d);
    DRAKE_EXPECT_THROWS_MESSAGE(
        AddContactMaterial(E, d, ps, mu, &p), std::logic_error,
        ".+ Trying to add property .+ name already exists");
  }

  // Error case: Already has stiffness.
  {
    ProximityProperties p;
    p.Add(PropName(kMaterialGroup, kPointStiffness), ps);
    DRAKE_EXPECT_THROWS_MESSAGE(
        AddContactMaterial(E, d, ps, mu, &p), std::logic_error,
        ".+ Trying to add property .+ name already exists");
  }

  // Error case: Already has friction.
  {
    ProximityProperties p;
    p.Add(PropName(kMaterialGroup, kFriction), mu);
    DRAKE_EXPECT_THROWS_MESSAGE(
        AddContactMaterial(E, d, ps, mu, &p), std::logic_error,
        ".+ Trying to add property .+ name already exists");
  }

  // Error case: 0 elasticity.
  {
    ProximityProperties p;
    DRAKE_EXPECT_THROWS_MESSAGE(AddContactMaterial(0, d, ps, mu, &p),
                                std::logic_error,
                                ".+elastic modulus must be positive.+");
  }

  // Error case: negative elasticity.
  {
    ProximityProperties p;
    DRAKE_EXPECT_THROWS_MESSAGE(AddContactMaterial(-1.3, d, ps, mu, &p),
                                std::logic_error,
                                ".+elastic modulus must be positive.+");
  }

  // Error case: negative dissipation.
  {
    ProximityProperties p;
    DRAKE_EXPECT_THROWS_MESSAGE(AddContactMaterial(E, -1.2, ps, mu, &p),
                                std::logic_error,
                                ".+dissipation can't be negative.+");
  }

  // Error case: negative stiffness.
  {
    ProximityProperties p;
    DRAKE_EXPECT_THROWS_MESSAGE(AddContactMaterial(E, d, -200, mu, &p),
                                std::logic_error,
                                ".+stiffness must be strictly positive.+");
  }

  // Error case: zero-valued stiffness.
  {
    ProximityProperties p;
    DRAKE_EXPECT_THROWS_MESSAGE(AddContactMaterial(E, d, 0, mu, &p),
                                std::logic_error,
                                ".+stiffness must be strictly positive.+");
  }
}

GTEST_TEST(ProximityPropertiesTest, AddRigidProperties) {
  for (double length : {1e-5, 1.25, 1e7}) {
    ProximityProperties props;
    AddRigidHydroelasticProperties(length, &props);
    EXPECT_TRUE(props.HasProperty(PropName(kHydroGroup, kComplianceType)));
    EXPECT_EQ(
        props.Get<HydroelasticType>(PropName(kHydroGroup, kComplianceType)),
        HydroelasticType::kRigid);
    EXPECT_TRUE(props.HasProperty(PropName(kHydroGroup, kRezHint)));
    EXPECT_EQ(props.Get<double>(PropName(kHydroGroup, kRezHint)), length);
  }

  ProximityProperties props;
  AddRigidHydroelasticProperties(&props);
  EXPECT_TRUE(props.HasProperty(PropName(kHydroGroup, kComplianceType)));
  EXPECT_EQ(props.Get<HydroelasticType>(PropName(kHydroGroup, kComplianceType)),
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
    EXPECT_TRUE(props.HasProperty(PropName(kHydroGroup, kComplianceType)));
    EXPECT_EQ(
        props.Get<HydroelasticType>(PropName(kHydroGroup, kComplianceType)),
        HydroelasticType::kSoft);
    EXPECT_TRUE(props.HasProperty(PropName(kHydroGroup, kRezHint)));
    EXPECT_EQ(props.Get<double>(PropName(kHydroGroup, kRezHint)), length);
  }

  ProximityProperties props;
  AddSoftHydroelasticProperties(&props);
  EXPECT_TRUE(props.HasProperty(PropName(kHydroGroup, kComplianceType)));
  EXPECT_EQ(props.Get<HydroelasticType>(PropName(kHydroGroup, kComplianceType)),
            HydroelasticType::kSoft);
}

// Tests the variant where the static pressure field is defined by the
// enumeration.
GTEST_TEST(ProximityPropertiesTest, AddHalfSpaceSoftProperties) {
  const double E = 1.5e8;
  for (double thickness : {1e-5, 1.25, 1e7}) {
    ProximityProperties props;
    props.Add(PropName(kMaterialGroup, kElastic), E);
    AddSoftHydroelasticPropertiesForHalfSpace(thickness, &props);
    EXPECT_TRUE(props.HasProperty(PropName(kHydroGroup, kSlabThickness)));
    EXPECT_EQ(props.Get<double>(
                  PropName(internal::kHydroGroup, internal::kSlabThickness)),
              thickness);
    EXPECT_EQ(
        props.Get<HydroelasticType>(PropName(kHydroGroup, kComplianceType)),
        HydroelasticType::kSoft);
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
