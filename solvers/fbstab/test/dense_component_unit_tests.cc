#include "drake/solvers/fbstab/test/dense_component_unit_tests.h"

#include <gtest/gtest.h>

namespace drake {
namespace solvers {
namespace fbstab {
namespace test {
namespace {

/**
 * @file Runs unit tests for the Dense MPC components. See
 * dense_component_unit_tests.h for documentation.
 */
GTEST_TEST(FBstabDense, DenseVariable) {
  DenseComponentUnitTests test;
  test.DenseVariableTests();
}

}  // namespace
}  // namespace test
}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
