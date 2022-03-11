#include "drake/multibody/parsing/detail_tinyxml2_diagnostic.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/temp_directory.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;

class TinyXml2DiagnosticTest : public ::testing::Test {
 public:
  TinyXml2DiagnosticTest() {
    policy_.SetActionForErrors(
        [this](const DiagnosticDetail& detail) {
          error_records_.push_back(detail);
        });
    policy_.SetActionForWarnings(
        [this](const DiagnosticDetail& detail) {
          warning_records_.push_back(detail);
        });

    data_ = R"""(
<?xml version="1.0"?>
<stuff>
  <error/>
  <warning/>
</stuff>
)""";
  }

 protected:
  std::string data_;
  DiagnosticPolicy policy_;

  XMLDocument xml_doc_;
  XMLElement* root_{};

  std::vector<DiagnosticDetail> error_records_;
  std::vector<DiagnosticDetail> warning_records_;
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
  TinyXml2Diagnostic diagnostic_{&policy_, &source_, "stuff"};
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
  TinyXml2Diagnostic diagnostic_{&policy_, &source_, "stuff"};
};

TEST_F(TinyXml2DiagnosticContentsTest, Error) {
  const XMLElement* error_node = root_->FirstChildElement("error");
  ASSERT_NE(error_node, nullptr);
  diagnostic_.Error(*error_node, "badness");
  ASSERT_EQ(error_records_.size(), 1);
  const std::string full_message = error_records_[0].FormatError();
  EXPECT_EQ(full_message, "<literal-string>.stuff:4: error: badness");
}

TEST_F(TinyXml2DiagnosticContentsTest, Warning) {
  const XMLElement* warning_node = root_->FirstChildElement("warning");
  ASSERT_NE(warning_node, nullptr);
  diagnostic_.Warning(*warning_node, "regret");
  ASSERT_EQ(warning_records_.size(), 1);
  const std::string full_message = warning_records_[0].FormatWarning();
  EXPECT_EQ(full_message, "<literal-string>.stuff:5: warning: regret");
}

TEST_F(TinyXml2DiagnosticFilenameTest, Error) {
  const XMLElement* error_node = root_->FirstChildElement("error");
  ASSERT_NE(error_node, nullptr);
  diagnostic_.Error(*error_node, "badness");
  ASSERT_EQ(error_records_.size(), 1);
  const std::string full_message = error_records_[0].FormatError();
  EXPECT_THAT(full_message,
              testing::MatchesRegex("/.*/test_data.stuff:4: error: badness"));
}

TEST_F(TinyXml2DiagnosticFilenameTest, Warning) {
  const XMLElement* warning_node = root_->FirstChildElement("warning");
  ASSERT_NE(warning_node, nullptr);
  diagnostic_.Warning(*warning_node, "regret");
  ASSERT_EQ(warning_records_.size(), 1);
  const std::string full_message = warning_records_[0].FormatWarning();
  EXPECT_THAT(full_message,
              testing::MatchesRegex("/.*/test_data.stuff:5: warning: regret"));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
