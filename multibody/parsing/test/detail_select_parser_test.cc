#include "drake/multibody/parsing/detail_select_parser.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/multibody/parsing/test/diagnostic_policy_test_base.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using testing::MatchesRegex;

class SelectParserTest : public test::DiagnosticPolicyTestBase {
 protected:
  MultibodyPlant<double> plant_{0.0};
  CollisionFilterGroupResolver resolver_{&plant_};
  ParsingWorkspace w_{{}, diagnostic_policy_, &plant_,
                      &resolver_, SelectParser};
};

// File names may use any mix of upper and lower case. A recognized extension
// selects a parser, and does not emit an unknown file type error.
TEST_F(SelectParserTest, CaseInsensitiveMatch) {
  EXPECT_NE(SelectParser(diagnostic_policy_, "foo.URDF"), nullptr);
  FlushDiagnostics();
  EXPECT_NE(SelectParser(diagnostic_policy_, "foo.Urdf"), nullptr);
  FlushDiagnostics();
  EXPECT_NE(SelectParser(diagnostic_policy_, "foo.urdf"), nullptr);
  FlushDiagnostics();
  EXPECT_NE(SelectParser(diagnostic_policy_, "foo.SDF"), nullptr);
  FlushDiagnostics();
  EXPECT_NE(SelectParser(diagnostic_policy_, "foo.Sdf"), nullptr);
  FlushDiagnostics();
  EXPECT_NE(SelectParser(diagnostic_policy_, "foo.sdf"), nullptr);
  FlushDiagnostics();
  EXPECT_NE(SelectParser(diagnostic_policy_, "foo.XML"), nullptr);
  FlushDiagnostics();
  EXPECT_NE(SelectParser(diagnostic_policy_, "foo.Xml"), nullptr);
  FlushDiagnostics();
  EXPECT_NE(SelectParser(diagnostic_policy_, "foo.xml"), nullptr);
  FlushDiagnostics();
}

// Unknown file type emits an error.
TEST_F(SelectParserTest, UnknownFileType) {
  ParserInterface* parser = SelectParser(diagnostic_policy_, "nope");
  EXPECT_THAT(TakeError(), MatchesRegex(".*not a recognized type.*"));

  // The resulting parser ignores input and does not crash.
  std::string empty;
  EXPECT_EQ(
      parser->AddModel({DataSource::kContents, &empty}, "", {}, w_),
      std::nullopt);
  EXPECT_EQ(
      parser->AddAllModels({DataSource::kContents, &empty}, "", w_).size(),
      0);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake

