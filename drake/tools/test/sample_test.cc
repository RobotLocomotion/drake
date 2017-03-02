#include "drake/tools/test/gen/sample.h"

#include "gtest/gtest.h"

namespace drake {
namespace tools {
namespace test {
namespace {

// We don't really expect the Sample<T> implementation to fail, but we'd like
// to touch every method as an API sanity check, so that we are reminded in
// case we change the generated API without realizing it.
GTEST_TEST(SampleTest, SimpleCoverage) {
  EXPECT_EQ(SampleIndices::kNumCoordinates, 2);
  EXPECT_EQ(SampleIndices::kX, 0);
  EXPECT_EQ(SampleIndices::kTwoWord, 1);

  // The device under test.
  Sample<double> dut;

  // Size.
  EXPECT_EQ(dut.size(), SampleIndices::kNumCoordinates);

  // Accessors.
  dut.set_x(11.0);
  dut.set_two_word(22.0);
  EXPECT_EQ(dut.x(), 11.0);
  EXPECT_EQ(dut.two_word(), 22.0);

  // Clone.
  auto cloned = dut.Clone();
  ASSERT_NE(cloned.get(), nullptr);
  EXPECT_NE(dynamic_cast<Sample<double>*>(cloned.get()), nullptr);
  EXPECT_EQ(cloned->GetAtIndex(0), 11.0);
  EXPECT_EQ(cloned->GetAtIndex(1), 22.0);
}

}  // namespace
}  // namespace test
}  // namespace tools
}  // namespace drake
