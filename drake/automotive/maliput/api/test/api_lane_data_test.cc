#include "drake/automotive/maliput/api/lane_data.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace maliput {
namespace api {

#define CHECK_ALL_LANE_POSITION_ACCESSORS(dut, _s, _r, _h)       \
  do {                                                           \
    EXPECT_EQ(dut.s(), _s);                                      \
    EXPECT_EQ(dut.r(), _r);                                      \
    EXPECT_EQ(dut.h(), _h);                                      \
    EXPECT_TRUE(CompareMatrices(dut.srh(), Vector3<double>(_s, _r, _h))); \
  } while (0)


GTEST_TEST(LanePositionTest, DefaultConstructor) {
  // Check that default constructor obeys its contract.
  LanePosition dut;
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, 0., 0., 0.);
}


GTEST_TEST(LanePositionTest, ParameterizedConstructor) {
  // Check the fully-parameterized constructor.
  LanePosition dut(23., 75., 0.567);
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, 23., 75., 0.567);
}


GTEST_TEST(LanePositionTest, ConstructionFromVector) {
  // Check the conversion-construction from a 3-vector.
  LanePosition dut = LanePosition::FromSrh(Vector3<double>(23., 75., 0.567));
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, 23., 75., 0.567);
}


GTEST_TEST(LanePositionTest, VectorSetter) {
  // Check the vector-based setter.
  LanePosition dut(23., 75., 0.567);
  dut.set_srh(Vector3<double>(9., 7., 8.));
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, 9., 7., 8.);
}


GTEST_TEST(LanePositionTest, ComponentSetters) {
  // Check the individual component setters.
  LanePosition dut(0.1, 0.2, 0.3);

  dut.set_s(99.);
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, 99., 0.2, 0.3);

  dut.set_r(2.3);
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, 99., 2.3, 0.3);

  dut.set_h(42.);
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, 99., 2.3, 42.);
}

#undef CHECK_ALL_LANE_POSITION_ACCESSORS


#define CHECK_ALL_GEO_POSITION_ACCESSORS(dut, _x, _y, _z)               \
  do {                                                                  \
    EXPECT_EQ(dut.x(), _x);                                             \
    EXPECT_EQ(dut.y(), _y);                                             \
    EXPECT_EQ(dut.z(), _z);                                             \
    EXPECT_TRUE(CompareMatrices(dut.xyz(), Vector3<double>(_x, _y, _z))); \
  } while (0)


GTEST_TEST(GeoPositionTest, DefaultConstructor) {
  // Check that default constructor obeys its contract.
  GeoPosition dut;
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, 0., 0., 0.);
}


GTEST_TEST(GeoPositionTest, ParameterizedConstructor) {
  // Check the fully-parameterized constructor.
  GeoPosition dut(23., 75., 0.567);
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, 23., 75., 0.567);
}


GTEST_TEST(GeoPositionTest, ConstructionFromVector) {
  // Check the conversion-construction from a 3-vector.
  GeoPosition dut = GeoPosition::FromXyz(Vector3<double>(23., 75., 0.567));
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, 23., 75., 0.567);
}


GTEST_TEST(GeoPositionTest, VectorSetter) {
  // Check the vector-based setter.
  GeoPosition dut(23., 75., 0.567);
  dut.set_xyz(Vector3<double>(9., 7., 8.));
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, 9., 7., 8.);
}


GTEST_TEST(GeoPositionTest, ComponentSetters) {
  // Check the individual component setters.
  GeoPosition dut(0.1, 0.2, 0.3);

  dut.set_x(99.);
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, 99., 0.2, 0.3);

  dut.set_y(2.3);
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, 99., 2.3, 0.3);

  dut.set_z(42.);
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, 99., 2.3, 42.);
}

#undef CHECK_ALL_GEO_POSITION_ACCESSORS

}  // namespace api
}  // namespace maliput
}  // namespace drake
