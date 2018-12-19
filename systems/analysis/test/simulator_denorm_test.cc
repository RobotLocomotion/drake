#include <limits>

#include <gtest/gtest.h>

#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace systems {
namespace {

// This test has to be segregated so that we can prevent valgrind
// from running on it. The odd floating point register instructions are
// not emulated in valgrind. From Valgrind manual:
//
//   Valgrind has the following limitations in its implementation of x86/AMD64
//   SSE2 FP arithmetic, relative to IEEE754 ... SSE2 has control bits which
//   make it treat denormalised numbers ... Valgrind detects, *ignores*, and
//   can warn about, attempts to enable either mode.
//   http://valgrind.org/docs/manual/manual-core.html#manual-core.limits
//
// The "ignored" part above would cause one of the tests below to fail.

// The purpose of this test is to verify that the GetPreviousNormalizedValue()
// method is not sensitive to the setting of floating point modes that affect
// treatment of denormalized numbers. We do not necessarily have control over
// those settings so want to make sure they don't matter.
GTEST_TEST(SimulatorTest, PreviousNormalizedValueTest) {
  // Do the tests with the "denormalized numbers treated as zero" (DAZ) and
  // "denormalized numbers flushed to zero" (FTZ) modes enabled. These modes
  // are activated when shared libraries are built using the -ffast-math option
  // in gcc.
  #ifdef __x86_64__
  const volatile double denorm_num = std::numeric_limits<double>::denorm_min();
  const unsigned MXCSR_DAZ = (1 << 6);
  const unsigned MXCSR_FTZ = (1 << 15);
  const unsigned mxcsr = __builtin_ia32_stmxcsr() | MXCSR_DAZ | MXCSR_FTZ;
  __builtin_ia32_ldmxcsr(mxcsr);

  // Verify that the flags were set as expected.
  EXPECT_EQ(denorm_num, 0.0);
  #endif

  const double min_double = std::numeric_limits<double>::min();
  EXPECT_EQ(internal::GetPreviousNormalizedValue(0.0), -min_double);
  EXPECT_EQ(internal::GetPreviousNormalizedValue(min_double/2), -min_double);
  EXPECT_EQ(internal::GetPreviousNormalizedValue(min_double), 0.0);
  EXPECT_EQ(internal::GetPreviousNormalizedValue(-min_double/2), -min_double);
  EXPECT_LE(internal::GetPreviousNormalizedValue(-min_double), -min_double);
  EXPECT_LE(internal::GetPreviousNormalizedValue(1.0), 1.0);
  EXPECT_NEAR(internal::GetPreviousNormalizedValue(1.0), 1.0,
              std::numeric_limits<double>::epsilon());

  // Since mxcsr has the flags set, XORing against those flags will set them
  // to zero.
  #ifdef __x86_64__
  __builtin_ia32_ldmxcsr(mxcsr ^ MXCSR_DAZ ^ MXCSR_FTZ);

  // Verify that the flags are set as expected.
  EXPECT_NE(denorm_num, 0.0);
  EXPECT_NE(std::numeric_limits<double>::min() / 2, 0.0);
  #endif

  // Do the tests again now that DAZ and FTZ modes are disabled.
  EXPECT_EQ(internal::GetPreviousNormalizedValue(0.0), -min_double);
  EXPECT_EQ(internal::GetPreviousNormalizedValue(min_double/2), -min_double);
  EXPECT_EQ(internal::GetPreviousNormalizedValue(min_double), 0.0);
  EXPECT_EQ(internal::GetPreviousNormalizedValue(-min_double/2), -min_double);
  EXPECT_LE(internal::GetPreviousNormalizedValue(-min_double), -min_double);
  EXPECT_LE(internal::GetPreviousNormalizedValue(1.0), 1.0);
  EXPECT_NEAR(internal::GetPreviousNormalizedValue(1.0), 1.0,
              std::numeric_limits<double>::epsilon());
}

}  // namespace
}  // namespace systems
}  // namespace drake
