#include "drake/automotive/maliput/api/lane_data.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace maliput {
namespace api {
namespace {

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

// An arbitrary very small number (that passes the tests).
const double kRotationTolerance = 1e-15;

#define CHECK_ALL_ROTATION_ACCESSORS(dut, _w, _x, _y, _z, _ro, _pi, _ya, _ma) \
  do {                                                                  \
    EXPECT_TRUE(CompareMatrices(dut.quat().coeffs(),                    \
                                Vector4<double>(_x, _y, _z, _w),        \
                                kRotationTolerance));                   \
    EXPECT_TRUE(CompareMatrices(dut.rpy(),                              \
                                Vector3<double>(_ro, _pi, _ya),         \
                                kRotationTolerance));                   \
    EXPECT_NEAR(dut.roll(), _ro, kRotationTolerance);                   \
    EXPECT_NEAR(dut.pitch(), _pi, kRotationTolerance);                  \
    EXPECT_NEAR(dut.yaw(), _ya, kRotationTolerance);                    \
    EXPECT_TRUE(CompareMatrices(dut.matrix(), _ma,                      \
                                kRotationTolerance));                   \
  } while (0)


class RotationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // A quaternion that rotates x->y, y->z, z->x...
    twist_quat_ = Quaternion<double> (
        Eigen::AngleAxis<double>(M_PI * 2. / 3.,
                                 Vector3<double>(1.0, 1.0, 1.0).normalized()));

    nonnormalized_twist_quat_ = Quaternion<double>(7. * twist_quat_.coeffs());

    twist_roll_ = M_PI / 2.;
    twist_pitch_ = 0.;
    twist_yaw_ = M_PI / 2.;

    twist_matrix_ <<
        0., 0., 1.,
        1., 0., 0.,
        0., 1., 0.;
  }

  Quaternion<double> nonnormalized_twist_quat_;
  Quaternion<double> twist_quat_;
  double twist_roll_;
  double twist_pitch_;
  double twist_yaw_;
  Matrix3<double> twist_matrix_;
};


TEST_F(RotationTest, DefaultConstructor) {
  // Check that default constructor obeys its contract.
  Rotation dut;
  CHECK_ALL_ROTATION_ACCESSORS(dut, 1., 0., 0., 0., 0., 0., 0.,
                               Matrix3<double>::Identity());
}


TEST_F(RotationTest, ConstructionFromQuaternion) {
  // Check the conversion-construction from a Quaternion.
  Rotation dut = Rotation::FromQuat(nonnormalized_twist_quat_);
  CHECK_ALL_ROTATION_ACCESSORS(
      dut, twist_quat_.w(), twist_quat_.x(), twist_quat_.y(), twist_quat_.z(),
      twist_roll_, twist_pitch_, twist_yaw_, twist_matrix_);
}


TEST_F(RotationTest, ConstructionFromRpyVector) {
  // Check the conversion-construction from a 3-vector of roll, pitch, yaw.
  Rotation dut = Rotation::FromRpy(Vector3<double>(
      twist_roll_, twist_pitch_, twist_yaw_));
  CHECK_ALL_ROTATION_ACCESSORS(
      dut, twist_quat_.w(), twist_quat_.x(), twist_quat_.y(), twist_quat_.z(),
      twist_roll_, twist_pitch_, twist_yaw_, twist_matrix_);
}


TEST_F(RotationTest, ConstructionFromRpyComponents) {
  // Check the conversion-construction from individual roll, pitch, yaw.
  Rotation dut = Rotation::FromRpy(twist_roll_, twist_pitch_, twist_yaw_);
  CHECK_ALL_ROTATION_ACCESSORS(
      dut, twist_quat_.w(), twist_quat_.x(), twist_quat_.y(), twist_quat_.z(),
      twist_roll_, twist_pitch_, twist_yaw_, twist_matrix_);
}


TEST_F(RotationTest, QuaternionSetter) {
  // Check the vector-based setter.
  Rotation dut = Rotation::FromRpy(23., 75., 0.567);
  dut.set_quat(nonnormalized_twist_quat_);
  CHECK_ALL_ROTATION_ACCESSORS(
      dut, twist_quat_.w(), twist_quat_.x(), twist_quat_.y(), twist_quat_.z(),
      twist_roll_, twist_pitch_, twist_yaw_, twist_matrix_);
}

#undef CHECK_ALL_ROTATION_ACCESSORS

}  // namespace
}  // namespace api
}  // namespace maliput
}  // namespace drake
