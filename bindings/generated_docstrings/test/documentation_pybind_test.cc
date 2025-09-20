#include <cstring>

#include <gtest/gtest.h>

#include "drake/bindings/generated_docstrings/math.h"

namespace {

// Mostly, this just checks for compilation failures.
GTEST_TEST(DocumentationPybindUsage, ExistenceTest) {
  const char* doc =
      pydrake_doc_math.drake.math.RigidTransform.ctor.doc_2args_R_p;
  EXPECT_GT(strlen(doc), 0);
}

}  // namespace
