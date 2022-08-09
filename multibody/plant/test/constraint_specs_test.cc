#include "drake/multibody/plant/constraint_specs.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace internal {
namespace {

template <typename T>
void CheckEquality(const CouplerConstraintSpecs<T>& spec1,
                   const CouplerConstraintSpecs<T>& spec2) {
  EXPECT_EQ(spec1.joint0_index, spec2.joint0_index);
  EXPECT_EQ(spec1.joint1_index, spec2.joint1_index);
  EXPECT_EQ(spec1.gear_ratio, spec2.gear_ratio);
  EXPECT_EQ(spec1.offset, spec2.offset);
}

GTEST_TEST(CouplerConstraintSpecs, ScalarConverison) {
  const JointIndex j0(3);
  const JointIndex j1(5);
  constexpr double kGearRatio = 1.2;
  constexpr double kOffSet = 0.3;

  CouplerConstraintSpecs<double> dut_double(j0, j1, kGearRatio, kOffSet);
  CouplerConstraintSpecs<AutoDiffXd> dut_ad(j0, j1, kGearRatio, kOffSet);

  CheckEquality(dut_double, CouplerConstraintSpecs<double>(dut_double));
  CheckEquality(dut_double, CouplerConstraintSpecs<double>(dut_ad));
  CheckEquality(dut_ad, CouplerConstraintSpecs<AutoDiffXd>(dut_double));
  CheckEquality(dut_ad, CouplerConstraintSpecs<AutoDiffXd>(dut_ad));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
