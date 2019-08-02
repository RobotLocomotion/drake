#include "drake/solvers/fbstab/components/test/mpc_component_unit_tests.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace drake {
namespace solvers {
namespace fbstab {
namespace test {

/**
 * Runs unit tests for the MPC components. See
 * mpc_component_unit_tests.h for documentation.
 */
GTEST_TEST(MpcData, GEMVH) {
  MPCComponentUnitTests test;
  test.GEMVH();
}

GTEST_TEST(MpcData, GEMVA) {
  MPCComponentUnitTests test;
  test.GEMVA();
}

GTEST_TEST(MpcData, GEMVG) {
  MPCComponentUnitTests test;
  test.GEMVG();
}

GTEST_TEST(MpcData, GEMVGT) {
  MPCComponentUnitTests test;
  test.GEMVGT();
}

GTEST_TEST(MpcData, GEMVAT) {
  MPCComponentUnitTests test;
  test.GEMVAT();
}

GTEST_TEST(MpcData, AXPYF) {
  MPCComponentUnitTests test;
  test.AXPYF();
}

GTEST_TEST(MpcData, AXPYH) {
  MPCComponentUnitTests test;
  test.AXPYH();
}

GTEST_TEST(MpcData, AXPYB) {
  MPCComponentUnitTests test;
  test.AXPYB();
}

GTEST_TEST(MPCVariable, AXPY) {
  MPCComponentUnitTests test;
  test.Variable();
}

}  // namespace test
}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
