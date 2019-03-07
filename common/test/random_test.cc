#include "drake/common/random.h"

#include <gtest/gtest.h>

namespace drake {
namespace {

GTEST_TEST(RandomTest, CompareWith19337) {
  std::mt19937 oracle;

  RandomGenerator dut;
  EXPECT_EQ(dut.min(), oracle.min());
  EXPECT_EQ(dut.max(), oracle.max());

  RandomGenerator::result_type first = dut();
  auto oracle_first = oracle();
  EXPECT_EQ(first, oracle_first);

  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(dut(), oracle());
  }
}

GTEST_TEST(RandomTest, Seed) {
  RandomGenerator dut1;
  RandomGenerator dut2(RandomGenerator::default_seed);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(dut1(), dut2());
  }
}

}  // namespace
}  // namespace drake
