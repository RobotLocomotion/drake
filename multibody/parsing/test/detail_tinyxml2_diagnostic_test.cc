#include "drake/multibody/parsing/detail_tinyxml2_diagnostic.h"

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;

class TinyXml2DiagnosticTest : public test::DiagnosticPolicyTestBase {
 public:
  TinyXml2DiagnosticTest() {
    data_ = R"""(
<?xml version="1.0"?>
<stuff>
  <error/>
  <warning unsupported_attribute='10'>
    <unsupported_element/>
  </warning>
</stuff>
)""";
  }

  const XMLElement& GetFirstChildNamed(const char* name) {
    DRAKE_DEMAND(root_ != nullptr);
    const XMLElement* element = root_->FirstChildElement(name);
    DRAKE_DEMAND(element != nullptr);
    return *element;
  }

 protected:
  std::string data_;

  XMLDocument xml_doc_;
  XMLElement* root_{};
};

class TinyXml2DiagnosticContentsTest : public TinyXml2DiagnosticTest {
 public:
  TinyXml2DiagnosticContentsTest() {
    xml_doc_.Parse(data_.c_str());
    root_ = xml_doc_.RootElement();
    DRAKE_DEMAND(root_ != nullptr);
  }

 protected:
  DataSource source_{DataSource::kContents, &data_};
  TinyXml2Diagnostic diagnostic_{&diagnostic_policy_, &source_, "stuff"};
};

class TinyXml2DiagnosticFilenameTest : public TinyXml2DiagnosticTest {
 public:
  TinyXml2DiagnosticFilenameTest() {
    filename_ = temp_directory() + "/test_data.stuff";
    std::ofstream file(filename_);
    file << data_;
    file.close();
    xml_doc_.LoadFile(filename_.c_str());
    root_ = xml_doc_.RootElement();
    DRAKE_DEMAND(root_ != nullptr);
  }

 protected:
  std::string filename_;
  DataSource source_{DataSource::kFilename, &filename_};
  TinyXml2Diagnostic diagnostic_{&diagnostic_policy_, &source_, "stuff"};
};

TEST_F(TinyXml2DiagnosticContentsTest, Error) {
  diagnostic_.Error(GetFirstChildNamed("error"), "badness");
  EXPECT_EQ(TakeError(), "<literal-string>.stuff:4: error: badness");
}

TEST_F(TinyXml2DiagnosticContentsTest, Warning) {
  diagnostic_.Warning(GetFirstChildNamed("warning"), "regret");
  EXPECT_EQ(TakeWarning(), "<literal-string>.stuff:5: warning: regret");
}

TEST_F(TinyXml2DiagnosticFilenameTest, Error) {
  diagnostic_.Error(GetFirstChildNamed("error"), "badness");
  EXPECT_THAT(TakeError(),
              testing::MatchesRegex("/.*/test_data.stuff:4: error: badness"));
}

TEST_F(TinyXml2DiagnosticFilenameTest, Warning) {
  diagnostic_.Warning(GetFirstChildNamed("warning"), "regret");
  EXPECT_THAT(TakeWarning(),
              testing::MatchesRegex("/.*/test_data.stuff:5: warning: regret"));
}

TEST_F(TinyXml2DiagnosticContentsTest, PolicyForNode) {
  // Policies for different nodes pass messages through to the
  // TinyXml2Diagnostic that made them, with location information for their
  // respective nodes.
  auto root_policy = diagnostic_.MakePolicyForNode(root_);
  root_policy.Warning("root rot");
  EXPECT_EQ(TakeWarning(), "<literal-string>.stuff:3: warning: root rot");
  root_policy.Error("root gone");
  EXPECT_EQ(TakeError(), "<literal-string>.stuff:3: error: root gone");

  auto error_policy =
      diagnostic_.MakePolicyForNode(&GetFirstChildNamed("error"));
  error_policy.Error("bad");
  EXPECT_EQ(TakeError(), "<literal-string>.stuff:4: error: bad");

  auto warning_policy =
      diagnostic_.MakePolicyForNode(&GetFirstChildNamed("warning"));
  warning_policy.Warning("meh");
  EXPECT_EQ(TakeWarning(), "<literal-string>.stuff:5: warning: meh");
}

TEST_F(TinyXml2DiagnosticContentsTest, Unsuppported) {
  diagnostic_.WarnUnsupportedElement(GetFirstChildNamed("warning"),
                                     "unsupported_element");
  EXPECT_EQ(TakeWarning(),
            "<literal-string>.stuff:6: warning: The tag 'unsupported_element'"
            " found as a child of 'warning' is currently unsupported and will"
            " be ignored.");

  diagnostic_.WarnUnsupportedAttribute(GetFirstChildNamed("warning"),
                                       "unsupported_attribute");
  EXPECT_EQ(TakeWarning(),
            "<literal-string>.stuff:5: warning: The attribute"
            " 'unsupported_attribute' found in a 'warning' tag is currently"
            " unsupported and will be ignored.");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
