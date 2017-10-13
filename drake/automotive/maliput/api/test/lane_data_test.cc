#include "drake/automotive/maliput/api/lane_data.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace maliput {
namespace api {
namespace {

static constexpr double kX0 = 23.;
static constexpr double kX1 = 75.;
static constexpr double kX2 = 0.567;

// TODO(jadecastro) Use CompareMatrices() to implement the
// LanePositionT<T>::srh() accessor checks once AutoDiff supported.
#define CHECK_ALL_LANE_POSITION_ACCESSORS(dut, _s, _r, _h)              \
  do {                                                                  \
    EXPECT_EQ(dut.s(), _s);                                             \
    EXPECT_EQ(dut.r(), _r);                                             \
    EXPECT_EQ(dut.h(), _h);                                             \
    EXPECT_EQ(dut.srh().rows(), 3);                                     \
    EXPECT_EQ(dut.srh().cols(), 1);                                     \
    EXPECT_EQ(dut.srh().x(), _s);                                       \
    EXPECT_EQ(dut.srh().y(), _r);                                       \
    EXPECT_EQ(dut.srh().z(), _h);                                       \
  } while (0)

// Class for defining identical tests in LanePositionTest across different
// scalar types, T.
template <typename T>
class LanePositionTest : public ::testing::Test {};

typedef ::testing::Types<double, AutoDiffXd> Implementations;
TYPED_TEST_CASE(LanePositionTest, Implementations);


TYPED_TEST(LanePositionTest, DefaultConstructor) {
  // Check that default constructor obeys its contract.
  using T = TypeParam;
  const LanePositionT<T> dut;
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, T(0.), T(0.), T(0.));
}


TYPED_TEST(LanePositionTest, ParameterizedConstructor) {
  // Check the fully-parameterized constructor.
  using T = TypeParam;
  const LanePositionT<T> dut(kX0, kX1, kX2);
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, T(kX0), T(kX1), T(kX2));
}


TYPED_TEST(LanePositionTest, ConstructionFromVector) {
  // Check the conversion-construction from a 3-vector.
  using T = TypeParam;
  const LanePositionT<T> dut =
      LanePositionT<T>::FromSrh(Vector3<T>(kX0, kX1, kX2));
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, T(kX0), T(kX1), T(kX2));
}


TYPED_TEST(LanePositionTest, VectorSetter) {
  // Check the vector-based setter.
  using T = TypeParam;
  LanePositionT<T> dut(kX0, kX1, kX2);
  const Vector3<T> srh(T(9.), T(7.), T(8.));
  dut.set_srh(srh);
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, T(srh.x()), T(srh.y()), T(srh.z()));
}


TYPED_TEST(LanePositionTest, ComponentSetters) {
  // Check the individual component setters.
  using T = TypeParam;
  LanePositionT<T> dut(T(0.1), T(0.2), T(0.3));

  dut.set_s(T(99.));
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, T(99.), T(0.2), T(0.3));

  dut.set_r(T(2.3));
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, T(99.), T(2.3), T(0.3));

  dut.set_h(T(42.));
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, T(99.), T(2.3), T(42.));
}

#undef CHECK_ALL_LANE_POSITION_ACCESSORS

// TODO(jadecastro) Use CompareMatrices() to implement the
// GeoPositionT<T>::xyz() accessor checks once AutoDiff supported.
#define CHECK_ALL_GEO_POSITION_ACCESSORS(dut, _x, _y, _z)               \
  do {                                                                  \
    EXPECT_EQ(dut.x(), _x);                                             \
    EXPECT_EQ(dut.y(), _y);                                             \
    EXPECT_EQ(dut.z(), _z);                                             \
    EXPECT_EQ(dut.xyz().rows(), 3);                                     \
    EXPECT_EQ(dut.xyz().cols(), 1);                                     \
    EXPECT_EQ(dut.xyz().x(), _x);                                       \
    EXPECT_EQ(dut.xyz().y(), _y);                                       \
    EXPECT_EQ(dut.xyz().z(), _z);                                       \
  } while (0)

// Class for defining identical tests in GeoPositionTest across different scalar
// types, T.
template <typename T>
class GeoPositionTest : public ::testing::Test {};

typedef ::testing::Types<double, AutoDiffXd> Implementations;
TYPED_TEST_CASE(GeoPositionTest, Implementations);

TYPED_TEST(GeoPositionTest, DefaultConstructor) {
  // Check that default constructor obeys its contract.
  using T = TypeParam;
  const GeoPositionT<T> dut;
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, T(0.), T(0.), T(0.));
}


TYPED_TEST(GeoPositionTest, ParameterizedConstructor) {
  // Check the fully-parameterized constructor.
  using T = TypeParam;
  const GeoPositionT<T> dut(kX0, kX1, kX2);
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, T(kX0), T(kX1), T(kX2));
}


TYPED_TEST(GeoPositionTest, ConstructionFromVector) {
  // Check the conversion-construction from a 3-vector.
  using T = TypeParam;
  const GeoPositionT<T> dut =
      GeoPositionT<T>::FromXyz(Vector3<T>(T(kX0), T(kX1), T(kX2)));
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, T(kX0), T(kX1), T(kX2));
}


TYPED_TEST(GeoPositionTest, VectorSetter) {
  // Check the vector-based setter.
  using T = TypeParam;
  GeoPositionT<T> dut(kX0, kX1, kX2);
  const Vector3<T> xyz(T(9.), T(7.), T(8.));
  dut.set_xyz(xyz);
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, T(xyz.x()), T(xyz.y()), T(xyz.z()));
}


TYPED_TEST(GeoPositionTest, ComponentSetters) {
  // Check the individual component setters.
  using T = TypeParam;
  GeoPositionT<T> dut(0.1, 0.2, 0.3);

  dut.set_x(T(99.));
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, T(99.), T(0.2), T(0.3));

  dut.set_y(T(2.3));
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, T(99.), T(2.3), T(0.3));

  dut.set_z(T(42.));
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, T(99.), T(2.3), T(42.));
}

#undef CHECK_ALL_GEO_POSITION_ACCESSORS

TYPED_TEST(GeoPositionTest, EqualityInequalityOperators) {
  // Checks that equality is true iff the constituent components are all equal,
  // and that inequality is true otherwise.
  using T = TypeParam;
  GeoPositionT<T> gp1(0.1, 0.2, 0.3);
  GeoPositionT<T> gp2(0.1, 0.2, 0.3);

  EXPECT_TRUE(gp1 == gp2);
  EXPECT_FALSE(gp1 != gp2);

  GeoPositionT<T> gp_xerror(gp2.x() + T(1e-6), gp2.y(), gp2.z());
  EXPECT_FALSE(gp1 == gp_xerror);
  EXPECT_TRUE(gp1 != gp_xerror);
  GeoPositionT<T> gp_yerror(gp2.x(), gp2.y() + T(1e-6), gp2.z());
  EXPECT_FALSE(gp1 == gp_yerror);
  EXPECT_TRUE(gp1 != gp_xerror);
  GeoPositionT<T> gp_zerror(gp2.x(), gp2.y(), gp2.z() + T(1e-6));
  EXPECT_FALSE(gp1 == gp_zerror);
  EXPECT_TRUE(gp1 != gp_xerror);

  GeoPositionT<T> gp_nan1(NAN, NAN, NAN);
  GeoPositionT<T> gp_nan2(NAN, NAN, NAN);
  EXPECT_TRUE(gp_nan1 != gp_nan2);
  EXPECT_FALSE(gp_nan1 == gp_nan2);
}

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

GTEST_TEST(RBoundsTest, DefaultConstructor) {
  const RBounds dut{};
  // Checks correct default value assignment.
  EXPECT_EQ(dut.min(), 0.);
  EXPECT_EQ(dut.max(), 0.);
}

GTEST_TEST(RBoundsTest, ParameterizedConstructor) {
  const double kMin{-5.};
  const double kMax{5.};
  RBounds dut(kMin, kMax);
  // Checks correct value assignment.
  EXPECT_EQ(dut.min(), kMin);
  EXPECT_EQ(dut.max(), kMax);
  // Checks constraints on the constructor.
  EXPECT_THROW(RBounds(kMax, kMax), std::runtime_error);
  EXPECT_THROW(RBounds(kMin, kMin), std::runtime_error);
}

GTEST_TEST(RBoundsTest, Setters) {
  const double kMin{-5.};
  const double kMax{5.};
  RBounds dut(kMin, kMax);
  // Set min and max correct values.
  dut.set_min(2. * kMin);
  EXPECT_EQ(dut.min(), 2. * kMin);
  dut.set_max(2. * kMax);
  EXPECT_EQ(dut.max(), 2. * kMax);
  // Checks constraints on the setters.
  EXPECT_THROW(dut.set_min(kMax), std::runtime_error);
  EXPECT_THROW(dut.set_max(kMin), std::runtime_error);
}

GTEST_TEST(HBoundsTest, DefaultConstructor) {
  const HBounds dut{};
  // Checks correct default value assignment.
  EXPECT_EQ(dut.min(), 0.);
  EXPECT_EQ(dut.max(), 0.);
}

GTEST_TEST(HBoundsTest, ParameterizedConstructor) {
  const double kMin{-5.};
  const double kMax{5.};
  HBounds dut(kMin, kMax);
  // Checks correct value assignment.
  EXPECT_EQ(dut.min(), kMin);
  EXPECT_EQ(dut.max(), kMax);
  // Checks constraints on the constructor.
  EXPECT_THROW(HBounds(kMax, kMax), std::runtime_error);
  EXPECT_THROW(HBounds(kMin, kMin), std::runtime_error);
}

GTEST_TEST(HBoundsTest, Setters) {
  const double kMin{-5.};
  const double kMax{5.};
  HBounds dut(kMin, kMax);
  // Set min and max correct values.
  dut.set_min(2. * kMin);
  EXPECT_EQ(dut.min(), 2. * kMin);
  dut.set_max(2. * kMax);
  EXPECT_EQ(dut.max(), 2. * kMax);
  // Checks constraints on the setters.
  EXPECT_THROW(dut.set_min(kMax), std::runtime_error);
  EXPECT_THROW(dut.set_max(kMin), std::runtime_error);
}

}  // namespace
}  // namespace api
}  // namespace maliput
}  // namespace drake
