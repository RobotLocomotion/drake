#include <cstring>

#include <gtest/gtest.h>

#include "drake/tools/workspace/pybind11/test/test_header_documentation.h"

namespace {

// Mostly, this just checks for compilation failures.
GTEST_TEST(TestHeaderDocumentation, ExistenceTest) {
  EXPECT_EQ(
      test_header_doc.drake.mkdoc_test.Class.ctor.doc_3,
      "Custom constructor 1.");
}

}  // namespace
