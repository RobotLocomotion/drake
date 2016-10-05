#include "drake/automotive/idm_with_trajectory_agent-inl.h"

/// @file
/// Separate test program, so that we can use the -inl file.

#include "gtest/gtest.h"

#include "drake/common/drake_export.h"

namespace {
/// An expression of the minimal ScalarType (MST) concept for
/// IdmWithTrajectoryAgent.
struct MST {
  MST() {}
  explicit MST(const double&) {}

  MST operator*(double) const { return *this; }
  MST operator/(double) const { return *this; }
  MST operator-(double) const { return *this; }

  MST operator+(const MST&) const { return *this; }
  MST operator-(const MST&) const { return *this; }
  MST operator*(const MST&) const { return *this; }
  MST operator/(const MST&) const { return *this; }

  MST operator+=(const MST&) const { return *this; }
};
MST pow(const MST&, double) { return MST{}; }
MST operator+(double, const MST&) { return MST{}; }
MST operator-(double, const MST&) { return MST{}; }
MST operator*(double, const MST&) { return MST{}; }
}  // namespace

namespace drake {

// Override the is_numeric trait, since there are no rounding operations
// on the minimal ScalarType declared above.
template<> struct DRAKE_EXPORT is_numeric<MST> {
  static constexpr bool value = false;
};

namespace automotive {
template class DRAKE_EXPORT IdmWithTrajectoryAgent<MST>;
namespace {

GTEST_TEST(IdmWithTrajectoryAgentScalarTypeTest, CompileTest) {
  const IdmWithTrajectoryAgent<MST> dut;

  // If we compiled, declare victory.
  GTEST_SUCCEED();
}

}  // namespace
}  // namespace automotive
}  // namespace drake
