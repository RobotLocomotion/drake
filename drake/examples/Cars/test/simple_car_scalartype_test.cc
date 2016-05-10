#include "drake/examples/Cars/simple_car-inl.h"

/// @file
/// Separate test program, so that we can use the -inl file.

#include "gtest/gtest.h"

namespace {
/// An expression of the minimal ScalarType concept for SimpleCar.
struct MST {
  typedef MST T;

  MST() {}
  MST(const double&) {}
  MST& operator=(const MST&) { return *this; }

  T operator*(double) const { return *this; }
  T operator/(double) const { return *this; }

  T operator+(const T&) const { return *this; }
  T operator-(const T&) const { return *this; }
  T operator+=(const T&) const { return *this; }
  T operator*(const T&) const { return *this; }

  bool operator<(const T&) const { return true; }
};
MST remainder(const MST&, const MST&) { return MST{}; }
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
  EXPECT_EQ(0, 0);
}

}
}
}
}
}
