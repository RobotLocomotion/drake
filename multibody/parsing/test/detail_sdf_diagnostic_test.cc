#include "drake/multibody/parsing/detail_sdf_diagnostic.h"

#include <string>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sdf/Root.hh>

#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;

class SDFormatDiagnosticTest : public test::DiagnosticPolicyTestBase {
 public:
  SDFormatDiagnosticTest() {
    data_ = R"""(<sdf version='1.6'>
<model name='mixed_emotions'>
  <link name='a'/>
</model>
</sdf>)""";
  }

 protected:
  std::string data_;
  sdf::ParserConfig parser_config_;
  sdf::Root root_;
};

class SDFormatDiagnosticContentsTest : public SDFormatDiagnosticTest {
 public:
  SDFormatDiagnosticContentsTest() {
    sdf::Errors errors =
        root_.LoadSdfString(data_source_.contents(), parser_config_);
    EXPECT_FALSE(sdf_diagnostic_.PropagateErrors(errors));
  }

 protected:
  DataSource data_source_{DataSource::kContents, &data_};
  SDFormatDiagnostic sdf_diagnostic_{&diagnostic_policy_, &data_source_,
                                     "stuff"};
};

class SDFormatDiagnosticFilenameTest : public SDFormatDiagnosticTest {
 public:
  SDFormatDiagnosticFilenameTest() {
    filename_ = temp_directory() + "/test_data.stuff";
    std::ofstream file(filename_);
    file << data_;
    file.close();
    sdf::Errors errors = root_.Load(filename_, parser_config_);
    sdf_diagnostic_.PropagateErrors(errors);
  }

 protected:
  std::string filename_;
  DataSource source_{DataSource::kFilename, &filename_};
  SDFormatDiagnostic sdf_diagnostic_{&diagnostic_policy_, &source_, "stuff"};
};

TEST_F(SDFormatDiagnosticContentsTest, Error) {
  sdf_diagnostic_.Error(root_.Element()->FindElement("model"),
                        std::move("badness"));
  EXPECT_EQ(TakeError(), "<literal-string>.stuff:2: error: badness");
}

TEST_F(SDFormatDiagnosticContentsTest, Warning) {
  sdf_diagnostic_.Warning(root_.Element()->FindElement("model"), "regret");
  EXPECT_EQ(TakeWarning(), "<literal-string>.stuff:2: warning: regret");
}

TEST_F(SDFormatDiagnosticFilenameTest, Error) {
  sdf_diagnostic_.Error(root_.Element()->FindElement("model"), "badness");
  EXPECT_THAT(TakeError(),
              testing::MatchesRegex("/.*/test_data.stuff:2: error: badness"));
}

TEST_F(SDFormatDiagnosticFilenameTest, Warning) {
  sdf_diagnostic_.Warning(root_.Element()->FindElement("model"), "regret");
  EXPECT_THAT(TakeWarning(),
              testing::MatchesRegex("/.*/test_data.stuff:2: warning: regret"));
}

TEST_F(SDFormatDiagnosticContentsTest, PolicyForNode) {
  // Policies for different nodes pass messages through to the
  // SDFormatDiagnostic that made them, with location information for their
  // respective nodes.
  auto root_policy = sdf_diagnostic_.MakePolicyForNode(*(root_.Element()));
  root_policy.Warning("root rot");
  EXPECT_EQ(TakeWarning(), "<literal-string>.stuff:1: warning: root rot");
  root_policy.Error("root gone");
  EXPECT_EQ(TakeError(), "<literal-string>.stuff:1: error: root gone");

  auto error_policy = sdf_diagnostic_.MakePolicyForNode(
      *(root_.Element()->FindElement("model")));
  error_policy.Error("bad");
  EXPECT_EQ(TakeError(), "<literal-string>.stuff:2: error: bad");

  auto warning_policy = sdf_diagnostic_.MakePolicyForNode(
      *(root_.Element()->FindElement("model")));
  warning_policy.Warning("meh");
  EXPECT_EQ(TakeWarning(), "<literal-string>.stuff:2: warning: meh");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
