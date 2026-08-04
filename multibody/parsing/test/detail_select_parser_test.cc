#include "drake/multibody/parsing/detail_select_parser.h"

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/diagnostic_policy_test_base.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using testing::MatchesRegex;

class SelectParserTest : public test::DiagnosticPolicyTestBase {
 protected:
  MultibodyPlant<double> plant_{0.0};
  CollisionFilterGroupResolver resolver_{&plant_};
  ParsingOptions options_;
  ParsingWorkspace w_{options_, {},         diagnostic_policy_, nullptr,
                      &plant_,  &resolver_, SelectParser};
};

// File names may use any mix of upper and lower case. A recognized extension
// selects a parser, and does not emit an unknown file type error.
TEST_F(SelectParserTest, CaseInsensitiveMatch) {
  SelectParser(diagnostic_policy_, "foo.URDF");
  FlushDiagnostics();
  SelectParser(diagnostic_policy_, "foo.Urdf");
  FlushDiagnostics();
  SelectParser(diagnostic_policy_, "foo.urdf");
  FlushDiagnostics();
  SelectParser(diagnostic_policy_, "foo.SDF");
  FlushDiagnostics();
  SelectParser(diagnostic_policy_, "foo.Sdf");
  FlushDiagnostics();
  SelectParser(diagnostic_policy_, "foo.sdf");
  FlushDiagnostics();
  SelectParser(diagnostic_policy_, "foo.XML");
  FlushDiagnostics();
  SelectParser(diagnostic_policy_, "foo.Xml");
  FlushDiagnostics();
  SelectParser(diagnostic_policy_, "foo.xml");
  FlushDiagnostics();
}

// Unknown file type emits an error.
TEST_F(SelectParserTest, UnknownFileType) {
  ParserInterface& parser = SelectParser(diagnostic_policy_, "nope");
  EXPECT_THAT(TakeError(), MatchesRegex(".*not a recognized type.*"));

  // The resulting parser ignores input, does not emit errors or warnings, and
  // does not crash.
  std::string empty;
  EXPECT_EQ(parser.AddModel({DataSource::kContents, &empty}, "", {}, w_),
            std::nullopt);
  EXPECT_EQ(parser.AddAllModels({DataSource::kContents, &empty}, "", w_).size(),
            0);
  FlushDiagnostics();
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
