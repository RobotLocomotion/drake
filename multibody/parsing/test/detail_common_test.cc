#include "drake/multibody/parsing/detail_common.h"

#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/diagnostic_policy_test_base.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;
using geometry::GeometryProperties;
using geometry::ProximityProperties;
using geometry::internal::HydroelasticType;
using geometry::internal::kComplianceType;
using geometry::internal::kElastic;
using geometry::internal::kFriction;
using geometry::internal::kHcDissipation;
using geometry::internal::kHydroGroup;
using geometry::internal::kMargin;
using geometry::internal::kMaterialGroup;
using geometry::internal::kPointStiffness;
using geometry::internal::kRelaxationTime;
using geometry::internal::kRezHint;
using std::optional;
using testing::MatchesRegex;

GTEST_TEST(EndsWithCaseInsensitiveTest, BasicTests) {
  EXPECT_TRUE(EndsWithCaseInsensitive("something", "thing"));
  EXPECT_TRUE(EndsWithCaseInsensitive("something", "THING"));
  EXPECT_TRUE(EndsWithCaseInsensitive("something", "Thing"));
  EXPECT_TRUE(EndsWithCaseInsensitive("thing", "thing"));
  EXPECT_TRUE(EndsWithCaseInsensitive("thing", "THING"));
  EXPECT_TRUE(EndsWithCaseInsensitive("thing", "Thing"));
  EXPECT_FALSE(EndsWithCaseInsensitive("something", "some"));
  EXPECT_FALSE(EndsWithCaseInsensitive("thing", "something"));
}

class DataSourceTest : public ::testing::Test {
 protected:
  const std::string relative_path_{"relative.txt"};
  const DataSource relative_{DataSource::kFilename, &relative_path_};
  const std::string absolute_path_{"/a/b/c/absolute.txt"};
  const DataSource absolute_{DataSource::kFilename, &absolute_path_};
  const std::string stuff_{"stuff"};
  const DataSource contents_{DataSource::kContents, &stuff_};
};

TEST_F(DataSourceTest, GetAbsolutePath) {
  EXPECT_THAT(relative_.GetAbsolutePath(),
              ::testing::MatchesRegex("/.*/relative.txt"));
  EXPECT_EQ(absolute_.GetAbsolutePath(), absolute_path_);  // no change.
  EXPECT_EQ(contents_.GetAbsolutePath(), "");
}

TEST_F(DataSourceTest, GetRootDir) {
  EXPECT_THAT(relative_.GetRootDir(), ::testing::MatchesRegex("/.*[^/]"));
  EXPECT_EQ(absolute_.GetRootDir(), "/a/b/c");
  EXPECT_EQ(contents_.GetRootDir(), "");
}

TEST_F(DataSourceTest, GetStem) {
  EXPECT_EQ(relative_.GetStem(), "relative");
  EXPECT_EQ(absolute_.GetStem(), "absolute");
  EXPECT_EQ(contents_.GetStem(), DataSource::kContentsPseudoStem);
}

using ReadDoubleFunc = std::function<optional<double>(const char*)>;
const bool rigid{true};
const bool compliant{true};

// A read_double implementation that always returns nullopt.
optional<double> empty_read_double(const char*) {
  return {};
}

// Creates a read_double implementation that returns nullopt for every tag name
// expect for the given name (which returns the given value).
ReadDoubleFunc param_read_double(const std::string& tag, double value) {
  return [&tag, value](const char* name) -> optional<double> {
    return (tag == name) ? optional<double>(value) : std::nullopt;
  };
}

// Tests for a particular value in the given properties.
template <typename T>
::testing::AssertionResult ExpectScalar(const char* group, const char* property,
                                        T expected,
                                        const ProximityProperties& p) {
  ::testing::AssertionResult failure = ::testing::AssertionFailure();
  const bool has_value = p.HasProperty(group, property);
  if (!has_value) {
    return failure << fmt::format("Expected ({}, {}); not found", group,
                                  property);
  }
  const T value = p.template GetProperty<T>(group, property);
  if (value != expected) {
    return failure << fmt::format(
               "Wrong value for ({}, {}):\n"
               "  Expected: {}\n"
               "  Found: {}",
               group, property, expected, value);
  }
  return ::testing::AssertionSuccess();
}

class ParseProximityPropertiesTest : public test::DiagnosticPolicyTestBase {
 public:
  // This shadows the namespace-scoped free function under test in order to
  // bind the `diagnostic` argument.
  geometry::ProximityProperties ParseProximityProperties(
      const std::function<std::optional<double>(const char*)>& read_double,
      bool is_rigid, bool is_compliant) {
    return internal::ParseProximityProperties(diagnostic_policy_, read_double,
                                              is_rigid, is_compliant);
  }
};

// Confirms that an "empty" <drake:proximity_properties> tag produces an empty
// instance of ProximityProperties.
TEST_F(ParseProximityPropertiesTest, NoProperties) {
  ProximityProperties properties =
      ParseProximityProperties(empty_read_double, !rigid, !compliant);
  // It is empty if there is a single group: the default group with no
  // properties.
  ASSERT_EQ(properties.num_groups(), 1);
  ASSERT_TRUE(properties.HasGroup(GeometryProperties::default_group_name()));
  ASSERT_EQ(
      properties.GetPropertiesInGroup(GeometryProperties::default_group_name())
          .size(),
      0u);
}

// Confirms successful parsing of hydroelastic properties.
TEST_F(ParseProximityPropertiesTest, HydroelasticProperties) {
  const char* kTag = "drake:mesh_resolution_hint";
  const double kRezHintValue{0.25};

  auto expect_compliance =
      [](bool is_rigid, bool is_compliant,
         const ProximityProperties& p) -> ::testing::AssertionResult {
    DRAKE_DEMAND(!(is_rigid && is_compliant));
    ::testing::AssertionResult failure = ::testing::AssertionFailure();
    const bool has_compliance_type =
        p.HasProperty(kHydroGroup, kComplianceType);
    if (is_rigid || is_compliant) {
      if (!has_compliance_type) {
        return failure << "Expected compliance; found none";
      }
      auto compliance =
          p.GetProperty<HydroelasticType>(kHydroGroup, kComplianceType);
      if (is_rigid && compliance != HydroelasticType::kRigid) {
        return failure << fmt::format("Expected rigid compliance; found {}",
                                      compliance);
      } else if (is_compliant && compliance != HydroelasticType::kSoft) {
        return failure << fmt::format("Expected compliant; found {}",
                                      compliance);
      }
    } else {
      if (has_compliance_type) {
        return failure << "Expected no compliance; compliance found";
      }
    }
    return ::testing::AssertionSuccess();
  };

  // Case: Declared rigid without a resolution hint.
  {
    ProximityProperties properties =
        ParseProximityProperties(empty_read_double, rigid, !compliant);
    EXPECT_TRUE(expect_compliance(rigid, !compliant, properties));
    // Compliance is the only property.
    EXPECT_EQ(properties.GetPropertiesInGroup(kHydroGroup).size(), 1u);
  }

  // Case: Declared rigid with a resolution hint.
  {
    ProximityProperties properties = ParseProximityProperties(
        param_read_double(kTag, kRezHintValue), rigid, !compliant);
    EXPECT_TRUE(expect_compliance(rigid, !compliant, properties));
    // Should have compliance and resolution.
    EXPECT_EQ(properties.GetPropertiesInGroup(kHydroGroup).size(), 2u);
    EXPECT_TRUE(ExpectScalar(kHydroGroup, kRezHint, kRezHintValue, properties));
  }

  // Case: Declared compliant without a resolution hint.
  {
    ProximityProperties properties =
        ParseProximityProperties(empty_read_double, !rigid, compliant);
    EXPECT_TRUE(expect_compliance(!rigid, compliant, properties));
    // Compliance is the only property.
    EXPECT_EQ(properties.GetPropertiesInGroup(kHydroGroup).size(), 1u);
  }

  // Case: Declared compliant with a resolution hint.
  {
    ProximityProperties properties = ParseProximityProperties(
        param_read_double(kTag, kRezHintValue), !rigid, compliant);
    EXPECT_TRUE(expect_compliance(!rigid, compliant, properties));
    // Should have compliance and resolution.
    EXPECT_EQ(properties.GetPropertiesInGroup(kHydroGroup).size(), 2u);
    EXPECT_TRUE(ExpectScalar(kHydroGroup, kRezHint, kRezHintValue, properties));
  }

  // Case: Resolution without any hydroelastic declaration.
  {
    ProximityProperties properties = ParseProximityProperties(
        param_read_double(kTag, kRezHintValue), !rigid, !compliant);
    EXPECT_TRUE(expect_compliance(!rigid, !compliant, properties));
    // Resolution should be the only property.
    EXPECT_EQ(properties.GetPropertiesInGroup(kHydroGroup).size(), 1u);
    EXPECT_TRUE(ExpectScalar(kHydroGroup, kRezHint, kRezHintValue, properties));
  }
}

// Confirms successful parsing of hydroelastic modulus.
TEST_F(ParseProximityPropertiesTest, HydroelasticModulus) {
  const double kValue = 1.75;
  ProximityProperties properties = ParseProximityProperties(
      param_read_double("drake:hydroelastic_modulus", kValue), !rigid,
      !compliant);
  EXPECT_TRUE(ExpectScalar(kHydroGroup, kElastic, kValue, properties));
  // Hydroelastic modulus is the only property.
  EXPECT_EQ(properties.GetPropertiesInGroup(kHydroGroup).size(), 1u);
  EXPECT_EQ(properties.num_groups(), 2);  // Hydro and default groups.
}

// Confirms ignored parsing of hydroelastic modulus for explicit rigid geometry.
TEST_F(ParseProximityPropertiesTest, RigidHydroelasticModulusIgnored) {
  DiagnosticDetail warning;
  diagnostic_policy_.SetActionForWarnings([&](const DiagnosticDetail& detail) {
    warning = detail;
  });
  const double kValue = 1.75;
  ProximityProperties properties = ParseProximityProperties(
      param_read_double("drake:hydroelastic_modulus", kValue), rigid,
      !compliant);
  EXPECT_THAT(warning.message,
              ::testing::MatchesRegex(
                  ".*hydroelastic_modulus.*value.*1.75.*ignored.*"));
  EXPECT_FALSE(ExpectScalar(kHydroGroup, kElastic, kValue, properties));
  EXPECT_TRUE(ExpectScalar(kHydroGroup, kComplianceType,
                           geometry::internal::HydroelasticType::kRigid,
                           properties));
  // Compliance type is the only property.
  EXPECT_EQ(properties.GetPropertiesInGroup(kHydroGroup).size(), 1u);
  EXPECT_EQ(properties.num_groups(), 2);  // Hydro and default groups.
}

// Confirms successful parsing of dissipation.
TEST_F(ParseProximityPropertiesTest, Dissipation) {
  const double kValue = 1.25;
  ProximityProperties properties = ParseProximityProperties(
      param_read_double("drake:hunt_crossley_dissipation", kValue), !rigid,
      !compliant);
  EXPECT_TRUE(ExpectScalar(kMaterialGroup, kHcDissipation, kValue, properties));
  // Dissipation is the only property.
  EXPECT_EQ(properties.GetPropertiesInGroup(kMaterialGroup).size(), 1u);
  EXPECT_EQ(properties.num_groups(), 2);  // Material and default groups.
}

// Confirms successful parsing of margin.
TEST_F(ParseProximityPropertiesTest, Margin) {
  const double kValue = 1.25;
  ProximityProperties properties = ParseProximityProperties(
      param_read_double("drake:hydroelastic_margin", kValue), !rigid,
      !compliant);
  EXPECT_TRUE(ExpectScalar(kHydroGroup, kMargin, kValue, properties));
  // Margin is the only property.
  EXPECT_EQ(properties.GetPropertiesInGroup(kHydroGroup).size(), 1u);
  EXPECT_EQ(properties.num_groups(), 2);  // Material and default groups.
}

// Confirms successful rejection of negative margin.
TEST_F(ParseProximityPropertiesTest, BadMargin) {
  const double kValue = -25;
  ProximityProperties properties = ParseProximityProperties(
      param_read_double("drake:hydroelastic_margin", kValue), !rigid,
      !compliant);
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(".*margin.*negative.*"));
  EXPECT_FALSE(ExpectScalar(kHydroGroup, kMargin, kValue, properties));
  EXPECT_EQ(properties.num_groups(), 1);  // Default group only.
}

// Confirms successful parsing of relaxation time.
TEST_F(ParseProximityPropertiesTest, RelaxationTime) {
  const double kValue = 1.25;
  ProximityProperties properties = ParseProximityProperties(
      param_read_double("drake:relaxation_time", kValue), !rigid, !compliant);
  EXPECT_TRUE(
      ExpectScalar(kMaterialGroup, kRelaxationTime, kValue, properties));
  // Relaxation time is the only property.
  EXPECT_EQ(properties.GetPropertiesInGroup(kMaterialGroup).size(), 1u);
  EXPECT_EQ(properties.num_groups(), 2);  // Material and default groups.
}

// Confirms successful rejection of negative relaxation time.
TEST_F(ParseProximityPropertiesTest, BadRelaxationTime) {
  const double kValue = -10;
  ProximityProperties properties = ParseProximityProperties(
      param_read_double("drake:relaxation_time", kValue), !rigid, !compliant);
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(".*relaxation.*negative.*"));
  EXPECT_FALSE(
      ExpectScalar(kMaterialGroup, kRelaxationTime, kValue, properties));
  EXPECT_EQ(properties.num_groups(), 1);  // Default group only.
}

// Confirms successful parsing of stiffness.
TEST_F(ParseProximityPropertiesTest, Stiffness) {
  const double kValue = 300.0;
  ProximityProperties properties = ParseProximityProperties(
      param_read_double("drake:point_contact_stiffness", kValue), !rigid,
      !compliant);
  EXPECT_TRUE(
      ExpectScalar(kMaterialGroup, kPointStiffness, kValue, properties));
  // Stiffness is the only property.
  EXPECT_EQ(properties.GetPropertiesInGroup(kMaterialGroup).size(), 1u);
  EXPECT_EQ(properties.num_groups(), 2);  // Material and default groups.
}

// Confirms successful parsing of friction.
TEST_F(ParseProximityPropertiesTest, Friction) {
  // We're not testing the case where *no* coefficients are provided; that's
  // covered in the NoProperties test.
  auto friction_read_double = [](optional<double> mu_d,
                                 optional<double> mu_s) -> ReadDoubleFunc {
    return [mu_d, mu_s](const char* name) -> optional<double> {
      optional<double> result;
      if (mu_d.has_value() && std::string("drake:mu_dynamic") == name) {
        result = *mu_d;
      } else if (mu_s.has_value() && std::string("drake:mu_static") == name) {
        result = *mu_s;
      }
      return result;
    };
  };

  auto expect_friction =
      [](double mu_d, double mu_s,
         const ProximityProperties& p) -> ::testing::AssertionResult {
    ::testing::AssertionResult failure = ::testing::AssertionFailure();
    const bool has_value = p.HasProperty(kMaterialGroup, kFriction);
    if (!has_value) {
      return failure << "Expected (" << kMaterialGroup << ", " << kFriction
                     << "); not found";
    }
    const auto& friction =
        p.GetProperty<CoulombFriction<double>>(kMaterialGroup, kFriction);
    if (friction.dynamic_friction() != mu_d ||
        friction.static_friction() != mu_s) {
      return failure << "Wrong value for (" << kMaterialGroup << ", "
                     << kFriction << "):"
                     << "\n  Expected mu_d: " << mu_d << ", mu_s: " << mu_s
                     << "\n  Found mu_d " << friction.dynamic_friction()
                     << ", mu_s: " << friction.static_friction();
    }
    return ::testing::AssertionSuccess();
  };

  // Case: Only dynamic -- both coefficients match dynamic coefficient.
  {
    const double kValue = 1.25;
    ProximityProperties properties = ParseProximityProperties(
        friction_read_double(kValue, {}), !rigid, !compliant);
    EXPECT_TRUE(expect_friction(kValue, kValue, properties));
    // Friction is the only property.
    EXPECT_EQ(properties.GetPropertiesInGroup(kMaterialGroup).size(), 1u);
    EXPECT_EQ(properties.num_groups(), 2);  // Material and default groups.
  }

  // Case: Only static -- both coefficients match static coefficient.
  {
    const double kValue = 1.5;
    ProximityProperties properties = ParseProximityProperties(
        friction_read_double({}, kValue), !rigid, !compliant);
    EXPECT_TRUE(expect_friction(kValue, kValue, properties));
    // Friction is the only property.
    EXPECT_EQ(properties.GetPropertiesInGroup(kMaterialGroup).size(), 1u);
    EXPECT_EQ(properties.num_groups(), 2);  // Material and default groups.
  }

  // Case: Both defined -- resulting coefficients match.
  {
    const double kMuD = 1.5;
    const double kMuS = 2.25;
    ProximityProperties properties = ParseProximityProperties(
        friction_read_double(kMuD, kMuS), !rigid, !compliant);
    EXPECT_TRUE(expect_friction(kMuD, kMuS, properties));
    // Friction is the only property.
    EXPECT_EQ(properties.GetPropertiesInGroup(kMaterialGroup).size(), 1u);
    EXPECT_EQ(properties.num_groups(), 2);  // Material and default groups.
  }
}

class ParseSpatialInertiaTest : public test::DiagnosticPolicyTestBase {};

TEST_F(ParseSpatialInertiaTest, BadInertiaValues) {
  std::vector<SpatialInertia<double>> results;
  math::RigidTransformd X_BBi;

  // Absurd rotational inertia values.
  results.push_back(ParseSpatialInertia(
      diagnostic_policy_, X_BBi, 1,
      {.ixx = 1, .iyy = 4, .izz = 9, .ixy = 16, .ixz = 25, .iyz = 36}));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*rot.*inertia.*"));
  // Test some inertia values found in the wild.
  results.push_back(ParseSpatialInertia(diagnostic_policy_, X_BBi, 0.038,
                                        {.ixx = 4.30439933333e-05,
                                         .iyy = 5.1205e-06,
                                         .izz = 4.3043993333e-05,
                                         .ixy = 9.57068e-06,
                                         .ixz = 1.44451933333e-05,
                                         .iyz = 1.342825e-05}));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*rot.*inertia.*"));
  // Negative mass.
  results.push_back(ParseSpatialInertia(
      diagnostic_policy_, X_BBi, -1,
      {.ixx = 1, .iyy = 1, .izz = 1, .ixy = 0, .ixz = 0, .iyz = 0}));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*mass > 0.*"));
  // Test that attempt to parse links with zero mass and non-zero inertia fails.
  results.push_back(ParseSpatialInertia(
      diagnostic_policy_, X_BBi, 0,
      {.ixx = 1, .iyy = 1, .izz = 1, .ixy = 0, .ixz = 0, .iyz = 0}));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*mass > 0.*"));

  // Do some basic sanity checking on the plausible mass and inertia generated
  // when warnings are issued.
  int k = 0;
  for (const auto& result : results) {
    SCOPED_TRACE(fmt::format("test case result {}", k));
    EXPECT_TRUE(result.IsPhysicallyValid());
    ++k;
  }
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
