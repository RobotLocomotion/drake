#include "drake/examples/Cars/simple_car-inl.h"

/// @file
/// Separate test program, so that we can use the -inl file.

#include "gtest/gtest.h"

namespace {
/// An expression of the minimal ScalarType (MST) concept for SimpleCar.
struct MST {
  MST() {}
  explicit MST(const double&) {}
  MST& operator=(const MST&) { return *this; }

  MST operator*(double) const { return *this; }
  MST operator/(double) const { return *this; }

  MST operator+(const MST&) const { return *this; }
  MST operator-(const MST&) const { return *this; }
  MST operator+=(const MST&) const { return *this; }
  MST operator*(const MST&) const { return *this; }

  bool operator<(const MST&) const { return true; }
};
MST sin(const MST&) { return MST{}; }
MST cos(const MST&) { return MST{}; }
MST tan(const MST&) { return MST{}; }
}  // namespace

namespace drake {
namespace examples {
namespace simple_car {
namespace test {
namespace {

GTEST_TEST(SimpleCarScalarTypeTest, CompileTest) {
  const SimpleCar dut;

  const MST time_zero{};
  const SimpleCarState<MST> state_zeros{};
  const DrivingCommand<MST> input_zeros{};

  const SimpleCarState<MST> dynamics =
      dut.dynamics(time_zero, state_zeros, input_zeros);
  const SimpleCarState<MST> output =
      dut.output(time_zero, state_zeros, input_zeros);

  // If we compiled, declare victory.
  GTEST_SUCCEED();
}

}
}
}
}
}
