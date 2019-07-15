/**
 * @file Runs unit tests for the Dense MPC components. See
 * dense_component_unit_tests.h for documentation.
 */
#include "drake/solvers/fbstab/components/test/dense_component_unit_tests.h"

#include <gtest/gtest.h>

namespace drake {
namespace solvers {
namespace fbstab {
namespace test {
namespace {

GTEST_TEST(FBstabDense, DenseVariable) {
  DenseComponentUnitTests test;
  test.DenseVariableTests();
}

GTEST_TEST(FBstabDense, TestInnerResidualCalculation) {
  DenseComponentUnitTests test;
  test.InnerResidualCalculation();
}

GTEST_TEST(FBstabDense, NaturalResidualCalculation) {
  DenseComponentUnitTests test;
  test.NaturalResidualCalculation();
}

GTEST_TEST(FBstabDense, DenseLinearSolver) {
  DenseComponentUnitTests test;
  test.LinearSolverResidual();
}

GTEST_TEST(FBstabDense, InfeasibilityDetection) {
  DenseComponentUnitTests test;
  test.InfeasibilityDetection();
}

GTEST_TEST(FBstabDense, UnboundednessDetection) {
  DenseComponentUnitTests test;
  test.UnboundednessDetection();
}

}  // namespace
}  // namespace test
}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
