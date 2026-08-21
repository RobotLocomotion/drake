#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

// The Eigen::AutoDiffScalar reference implementation of atan2 doesn't handle
// atan2(0, 0) correctly, so we'll define a shim that does and test it instead.
AutoDiff3 atan2_for_testing(const AutoDiff3& y, const AutoDiff3& x) {
  if (x.value() == 0 && x.derivatives().isZero(0.0) && y.value() == 0 &&
      y.derivatives().isZero(0.0)) {
    return AutoDiff3(0.0, Eigen::Vector3d::Zero());
  }
  return atan2(y, x);
}

// We need to provide a like-named stub for the device under test, but without
// any adjustments to the computation.
AutoDiffDut atan2_for_testing(const AutoDiffDut& y, const AutoDiffDut& x) {
  return atan2(y, x);
}

TEST_F(StandardOperationsTest, Atan2AdsAds) {
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2_for_testing, x, y, 0.1);
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2_for_testing, x, y, -0.1);
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2_for_testing, y, x, 0.4);
  CHECK_BINARY_FUNCTION_ADS_ADS(atan2_for_testing, y, x, -0.4);
}

// Eigen doesn't provide mixed-scalar atan2() overloads, so we need to do it.
// This assumes the standard_operations_test.h is implemented using AutoDiff3.
AutoDiff3 atan2(const AutoDiff3& y, double x) {
  return atan2(y, AutoDiff3{x});
}
AutoDiff3 atan2(double y, const AutoDiff3& x) {
  return atan2(AutoDiff3{y}, x);
}

TEST_F(StandardOperationsTest, Atan2AdsDouble) {
  CHECK_BINARY_FUNCTION_ADS_SCALAR(atan2, x, y, 0.1);
  CHECK_BINARY_FUNCTION_ADS_SCALAR(atan2, x, y, -0.1);
  CHECK_BINARY_FUNCTION_ADS_SCALAR(atan2, y, x, 0.4);
  CHECK_BINARY_FUNCTION_ADS_SCALAR(atan2, y, x, -0.4);
}

TEST_F(StandardOperationsTest, Atan2DoubleAds) {
  CHECK_BINARY_FUNCTION_SCALAR_ADS(atan2, x, y, 0.1);
  CHECK_BINARY_FUNCTION_SCALAR_ADS(atan2, x, y, -0.1);
  CHECK_BINARY_FUNCTION_SCALAR_ADS(atan2, y, x, 0.4);
  CHECK_BINARY_FUNCTION_SCALAR_ADS(atan2, y, x, -0.4);
}

}  // namespace
}  // namespace test
}  // namespace drake
