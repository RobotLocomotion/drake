#include "drake/common/drake_nodiscard.h"

#include <gtest/gtest.h>

#include "drake/common/unused.h"

namespace drake {
namespace {

DRAKE_NODISCARD int foo() { return 0; }

GTEST_TEST(NodiscardTest, CompilationTest) {
  const int bar = foo();
  unused(bar);
}

}  // namespace
}  // namespace drake
