#include "drake/automotive/idm_with_trajectory_agent-inl.h"

/// @file
/// Separate test program, so that we can use the -inl file.

#include "gtest/gtest.h"

#include "drake/drakeAutomotive_export.h"

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
namespace automotive {
template class DRAKEAUTOMOTIVE_EXPORT IdmWithTrajectoryAgent<MST>;
namespace {

GTEST_TEST(IdmWithTrajectoryAgentScalarTypeTest, CompileTest) {
  const IdmWithTrajectoryAgent<MST> dut;

  // If we compiled, declare victory.
  GTEST_SUCCEED();
}

}  // namespace
}  // namespace automotive
}  // namespace drake
