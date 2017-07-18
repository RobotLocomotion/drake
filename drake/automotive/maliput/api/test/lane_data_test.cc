#include "drake/automotive/maliput/api/lane_data.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace maliput {
namespace api {
namespace {

static constexpr double kX0 = 23.;
static constexpr double kX1 = 75.;
static constexpr double kX2 = 0.567;

#define CHECK_ALL_LANE_POSITION_ACCESSORS(dut, _s, _r, _h)              \
  do {                                                                  \
    EXPECT_EQ(dut.s(), _s);                                             \
    EXPECT_EQ(dut.r(), _r);                                             \
    EXPECT_EQ(dut.h(), _h);                                             \
    EXPECT_EQ(dut.srh().x(), _s);                                       \
    EXPECT_EQ(dut.srh().y(), _r);                                       \
    EXPECT_EQ(dut.srh().z(), _h);                                       \
  } while (0)

GTEST_TEST(LanePositionTest, DefaultConstructor) {
  // Check that default constructor obeys its contract.
  const LanePosition dut;
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, 0., 0., 0.);
  EXPECT_NO_THROW(LanePositionT<double> dut_double);
  EXPECT_NO_THROW(LanePositionT<AutoDiffXd> dut_autodiff);
}


GTEST_TEST(LanePositionTest, ParameterizedConstructor) {
  // Check the fully-parameterized constructor.
  const LanePosition dut(kX0, kX1, kX2);
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, kX0, kX1, kX2);
  EXPECT_NO_THROW(LanePositionT<double> dut_double(kX0, kX1, kX2));
  EXPECT_NO_THROW(LanePositionT<AutoDiffXd> dut_autodiff(kX0, kX1, kX2));
}


GTEST_TEST(LanePositionTest, ConstructionFromVector) {
  // Check the conversion-construction from a 3-vector.
  const LanePosition dut =
      LanePosition::FromSrh(Vector3<double>(kX0, kX1, kX2));
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, kX0, kX1, kX2);
  EXPECT_NO_THROW(LanePositionT<double>::FromSrh({kX0, kX1, kX2}));
  EXPECT_NO_THROW(LanePositionT<AutoDiffXd>::FromSrh({kX0, kX1, kX2}));
}


GTEST_TEST(LanePositionTest, VectorSetter) {
  // Check the vector-based setter.
  LanePosition dut(kX0, kX1, kX2);
  const Vector3<double> srh(9., 7., 8.);
  dut.set_srh(srh);
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, srh.x(), srh.y(), srh.z());

  // Check the vector-based setter when using AutoDiffXd variables.
  LanePositionT<AutoDiffXd> dut_autodiff(kX0, kX1, kX2);
  const Vector3<AutoDiffXd> srh_autodiff(srh.x(), srh.y(), srh.z());
  dut_autodiff.set_srh(srh_autodiff);
  CHECK_ALL_LANE_POSITION_ACCESSORS(
      dut_autodiff, srh_autodiff.x(), srh_autodiff.y(), srh_autodiff.z());
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

  // Check the individual component setters when using AutoDiffXd variables.
  LanePositionT<AutoDiffXd> dut_autodiff(0.1, 0.2, 0.3);

  dut_autodiff.set_s(AutoDiffXd(99.));
  CHECK_ALL_LANE_POSITION_ACCESSORS(
      dut_autodiff, AutoDiffXd(99.), AutoDiffXd(0.2), AutoDiffXd(0.3));

  dut_autodiff.set_r(AutoDiffXd(2.3));
  CHECK_ALL_LANE_POSITION_ACCESSORS(
      dut_autodiff, AutoDiffXd(99.), AutoDiffXd(2.3), AutoDiffXd(0.3));

  dut_autodiff.set_h(AutoDiffXd(42.));
  CHECK_ALL_LANE_POSITION_ACCESSORS(
      dut_autodiff, AutoDiffXd(99.), AutoDiffXd(2.3), AutoDiffXd(42.));
}

#undef CHECK_ALL_LANE_POSITION_ACCESSORS


#define CHECK_ALL_GEO_POSITION_ACCESSORS(dut, _x, _y, _z)               \
  do {                                                                  \
    EXPECT_EQ(dut.x(), _x);                                             \
    EXPECT_EQ(dut.y(), _y);                                             \
    EXPECT_EQ(dut.z(), _z);                                             \
    EXPECT_EQ(dut.xyz().x(), _x);                                       \
    EXPECT_EQ(dut.xyz().y(), _y);                                       \
    EXPECT_EQ(dut.xyz().z(), _z);                                       \
  } while (0)


GTEST_TEST(GeoPositionTest, DefaultConstructor) {
  // Check that default constructor obeys its contract.
  const GeoPosition dut;
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, 0., 0., 0.);
  EXPECT_NO_THROW(GeoPositionT<double> dut_double);
  EXPECT_NO_THROW(GeoPositionT<AutoDiffXd> dut_autodiff);
}


GTEST_TEST(GeoPositionTest, ParameterizedConstructor) {
  // Check the fully-parameterized constructor.
  const GeoPosition dut(kX0, kX1, kX2);
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, kX0, kX1, kX2);
  EXPECT_NO_THROW(GeoPositionT<double> dut_double(kX0, kX1, kX2));
  EXPECT_NO_THROW(GeoPositionT<AutoDiffXd> dut_autodiff(kX0, kX1, kX2));
}


GTEST_TEST(GeoPositionTest, ConstructionFromVector) {
  // Check the conversion-construction from a 3-vector.
  const GeoPosition dut =
      GeoPosition::FromXyz(Vector3<double>(kX0, kX1, kX2));
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, kX0, kX1, kX2);
  EXPECT_NO_THROW(GeoPositionT<double>::FromXyz({kX0, kX1, kX2}));
  EXPECT_NO_THROW(GeoPositionT<AutoDiffXd>::FromXyz({kX0, kX1, kX2}));
}


GTEST_TEST(GeoPositionTest, VectorSetter) {
  // Check the vector-based setter.
  GeoPosition dut(kX0, kX1, kX2);
  const Vector3<double> xyz(9., 7., 8.);
  dut.set_xyz(xyz);
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, xyz.x(), xyz.y(), xyz.z());

  // Check the vector-based setter when using AutoDiffXd variables.
  GeoPositionT<AutoDiffXd> dut_autodiff(kX0, kX1, kX2);
  const Vector3<AutoDiffXd> xyz_autodiff(xyz.x(), xyz.y(), xyz.z());
  dut_autodiff.set_xyz(xyz_autodiff);
  CHECK_ALL_GEO_POSITION_ACCESSORS(
      dut_autodiff, xyz_autodiff.x(), xyz_autodiff.y(), xyz_autodiff.z());
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

  // Check the individual component setters when using AutoDiffXd variables.
  GeoPositionT<AutoDiffXd> dut_autodiff(0.1, 0.2, 0.3);

  dut_autodiff.set_x(AutoDiffXd(99.));
  CHECK_ALL_GEO_POSITION_ACCESSORS(
      dut_autodiff, AutoDiffXd(99.), AutoDiffXd(0.2), AutoDiffXd(0.3));

  dut_autodiff.set_y(AutoDiffXd(2.3));
  CHECK_ALL_GEO_POSITION_ACCESSORS(
      dut_autodiff, AutoDiffXd(99.), AutoDiffXd(2.3), AutoDiffXd(0.3));

  dut_autodiff.set_z(AutoDiffXd(42.));
  CHECK_ALL_GEO_POSITION_ACCESSORS(
      dut_autodiff, AutoDiffXd(99.), AutoDiffXd(2.3), AutoDiffXd(42.));
}

GTEST_TEST(GeoPositionTest, EqualityInequalityOperators) {
  // Checks that equality is true iff the constituent components are all equal,
  // and that inequality is true otherwise.
  GeoPosition gp1(0.1, 0.2, 0.3);
  GeoPosition gp2(0.1, 0.2, 0.3);

  EXPECT_TRUE(gp1 == gp2);
  EXPECT_FALSE(gp1 != gp2);

  GeoPosition gp_xerror(gp2.x() + 1e-6, gp2.y(), gp2.z());
  EXPECT_FALSE(gp1 == gp_xerror);
  EXPECT_TRUE(gp1 != gp_xerror);
  GeoPosition gp_yerror(gp2.x(), gp2.y() + 1e-6, gp2.z());
  EXPECT_FALSE(gp1 == gp_yerror);
  EXPECT_TRUE(gp1 != gp_xerror);
  GeoPosition gp_zerror(gp2.x(), gp2.y(), gp2.z() + 1e-6);
  EXPECT_FALSE(gp1 == gp_zerror);
  EXPECT_TRUE(gp1 != gp_xerror);
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
