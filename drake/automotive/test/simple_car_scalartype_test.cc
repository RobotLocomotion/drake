#include "drake/automotive/simple_car-inl.h"

/// @file
/// Separate test program, so that we can use the -inl file.

#include "gtest/gtest.h"

#include "drake/common/drake_assert.h"
#include "drake/common/number_traits.h"
#include "drake/drakeAutomotive_export.h"
#include "drake/systems/framework/leaf_system.h"

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

// Override the is_numeric trait, since there are no rounding operations
// on MST.
template<> struct DRAKEAUTOMOTIVE_EXPORT drake::is_numeric<MST> {
  static constexpr bool value = false;
};

namespace drake {
namespace automotive {

template class DRAKEAUTOMOTIVE_EXPORT SimpleCar<MST>;

namespace {

// Tests that we can compile a SimpleCar based on the MST.
GTEST_TEST(SimpleCarScalarTypeTest, CompileTest) {
  const SimpleCar<MST> dut;

  // If we compiled, declare victory.
  GTEST_SUCCEED();
}

}  // namespace
}  // namespace automotive
}  // namespace drake
