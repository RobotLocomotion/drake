#include "drake/automotive/simple_car-inl.h"

/// @file
/// Separate test program, so that we can use the -inl file.

#include "gtest/gtest.h"

#include "drake/drakeAutomotive_export.h"

namespace {
/// An expression of the minimal ScalarType (MST) concept for SimpleCar.
struct MST {
  MST() {}
  explicit MST(const double&) {}
  MST& operator=(const MST&) { return *this; }

  MST operator-() const { return *this; }

  MST operator+(const MST&) const { return *this; }
  MST operator-(const MST&) const { return *this; }
  MST operator/(const MST&) const { return *this; }
  MST operator*(const MST&) const { return *this; }
  MST operator+=(const MST&) const { return *this; }

  bool operator<(const MST&) const { return true; }
};
MST sin(const MST&) { return MST{}; }
MST cos(const MST&) { return MST{}; }
MST tan(const MST&) { return MST{}; }
}  // namespace

namespace drake {
namespace automotive {
template class DRAKEAUTOMOTIVE_EXPORT SimpleCar<MST>;
namespace {

GTEST_TEST(SimpleCarScalarTypeTest, CompileTest) {
  const SimpleCar<MST> dut;

  // If we compiled, declare victory.
  GTEST_SUCCEED();
}

}  // namespace
}  // namespace automotive
}  // namespace drake
