#include "drake/bindings/pydrake/documentation_pybind.h"

#include <cstring>

#include <gtest/gtest.h>

namespace {

// Mostly, this just checks for compilation failures.
GTEST_TEST(DocumentationPybindUsage, ExistenceTest) {
  const char* doc = pydrake_doc.drake.math.RigidTransform.ctor.doc_2args_R_p;
  EXPECT_GT(strlen(doc), 0);
}

}  // namespace
