#include <gtest/gtest.h>

#include "drake/tools/workspace/pybind11/test/sample_header_documentation.h"

namespace {

// Mostly, this just checks for compilation failures.
GTEST_TEST(TestHeaderDocumentation, ExistenceTest) {
  EXPECT_EQ(
      sample_header_doc.drake.mkdoc_test.Class.ctor.doc_0args,
      "Custom constructor 1.");
}

}  // namespace
