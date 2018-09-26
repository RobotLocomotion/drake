#include <cstring>

#include <gtest/gtest.h>

#include "drake/bindings/pydrake/documentation_pybind.h"

namespace {

// Mostly, this just checks for compilation failures.
GTEST_TEST(DocumentationPybindUsage, ExistenceTest) {
  EXPECT_GT(strlen(pydrake_doc.drake.math.RigidTransform.ctor.doc_4), 0);
}

}  // namespace
