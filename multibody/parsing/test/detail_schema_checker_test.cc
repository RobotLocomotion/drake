#include "drake/multibody/parsing/detail_schema_checker.h"

#include <fstream>
#include <sstream>
#include <thread>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {
namespace internal {

namespace {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;

// Here are some fairly arbitrary and complicated schema and document files,
// taken from:
// https://authors.ietf.org/templates-and-schemas
//
const std::filesystem::path testdir("multibody/parsing/test");
const std::filesystem::path schema("rfc7991bis.rnc");
// This document will emit an error, fairly deep in the document file.
const std::filesystem::path doc_bad(
    "draft-rfcxml-general-template-annotated-00.xml");
// This document will validate cleanly.
const std::filesystem::path doc_good(
    "draft-rfcxml-general-template-bare-00.xml");

std::string slurp(const std::filesystem::path& path) {
  std::ifstream in;
  in.open(path, std::ifstream::in | std::ifstream::binary);
  DRAKE_DEMAND(in.good());
  std::stringstream stream;
  stream << in.rdbuf();
  in.close();
  return stream.str();
}

GTEST_TEST(SchemaCheckerTest, ThreadedTest) {
  std::vector<std::filesystem::path> docs{doc_bad, doc_good};
  std::vector<int> docs_results{false, true};
  std::vector<std::string> doc_strs;
  for (const auto& doc : docs) {
    doc_strs.push_back(slurp(testdir / doc));
  }

  auto thread_action = [&](int index) {
    SCOPED_TRACE(fmt::format("thread index {}", index));
    DiagnosticPolicy policy;
    int errors{0};
    int warnings{0};
    policy.SetActionForErrors(
        [&errors](const DiagnosticDetail& detail) {
          ++errors;
        });
    policy.SetActionForWarnings(
        [&warnings](const DiagnosticDetail& detail) { ++warnings; });

    // Cycle over the API entry points and documents.
    int entry_point = index % 2;
    int doc_index = index % docs.size();
    bool result{};
    switch (entry_point) {
      case 0:
        result = CheckDocumentFileAgainstRncSchemaFile(
            policy, testdir / schema, testdir / docs[doc_index]);
        break;
      case 1:
        result = CheckDocumentStringAgainstRncSchemaFile(
            policy, testdir / schema,
            doc_strs[doc_index], testdir / docs[doc_index]);
        break;
      default:
        DRAKE_UNREACHABLE();
    }
    EXPECT_EQ(result, !errors);
    EXPECT_EQ(result, docs_results[doc_index]);
    EXPECT_EQ(warnings, 0);
  };

  // Launch many threads as quickly as possible, each doing thread_action.
  std::vector<std::thread> threads;
  for (int i = 0; i < 10; ++i) {
    threads.emplace_back(thread_action, i);
  }

  // Wait for them all to finish.
  for (auto& item : threads) {
    item.join();
  }
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
