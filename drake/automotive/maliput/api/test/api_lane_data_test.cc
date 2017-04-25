#include "drake/automotive/maliput/api/lane_data.h"

#include <gtest/gtest.h>

namespace drake {
namespace maliput {
namespace api {

GTEST_TEST(LanePositionTest, DefaultConstructor) {
  // Check that default constructor obeys its contract.
  LanePosition dut;

  EXPECT_EQ(dut.s(), 0.);
  EXPECT_EQ(dut.r(), 0.);
  EXPECT_EQ(dut.h(), 0.);
  EXPECT_EQ(dut.srh(), Vector3<double>::Zero());
}


GTEST_TEST(LanePositionTest, ParameterizedConstructor) {
  // Check the fully-parameterized constructor.
  LanePosition dut(23., 75., 0.567);

  EXPECT_EQ(dut.s(), 23.);
  EXPECT_EQ(dut.r(), 75.);
  EXPECT_EQ(dut.h(), 0.567);
  EXPECT_EQ(dut.srh(), Vector3<double>(23., 75., 0.567));
}


GTEST_TEST(LanePositionTest, ConstructionFromVector) {
  // Check the conversion-construction from a 3-vector.
  LanePosition dut = LanePosition::FromSrh(Vector3<double>(23., 75., 0.567));

  EXPECT_EQ(dut.s(), 23.);
  EXPECT_EQ(dut.r(), 75.);
  EXPECT_EQ(dut.h(), 0.567);
  EXPECT_EQ(dut.srh(), Vector3<double>(23., 75., 0.567));
}


GTEST_TEST(LanePositionTest, VectorSetter) {
  // Check the vector-based setter.
  LanePosition dut(23., 75., 0.567);
  dut.set_srh(Vector3<double>(9., 7., 8.));

  EXPECT_EQ(dut.s(), 9.);
  EXPECT_EQ(dut.r(), 7.);
  EXPECT_EQ(dut.h(), 8.);
  EXPECT_EQ(dut.srh(), Vector3<double>(9., 7., 8.));
}


GTEST_TEST(LanePositionTest, ComponentSetters) {
  // Check the individual component setters.
  LanePosition dut(0.1, 0.2, 0.3);

  dut.set_s(99.);
  EXPECT_EQ(dut.s(), 99.);
  EXPECT_EQ(dut.srh(), Vector3<double>(99., 0.2, 0.3));

  dut.set_r(2.3);
  EXPECT_EQ(dut.r(), 2.3);
  EXPECT_EQ(dut.srh(), Vector3<double>(99., 2.3, 0.3));

  dut.set_h(42.);
  EXPECT_EQ(dut.h(), 42.);
  EXPECT_EQ(dut.srh(), Vector3<double>(99., 2.3, 42.));
}


}  // namespace api
}  // namespace maliput
}  // namespace drake
