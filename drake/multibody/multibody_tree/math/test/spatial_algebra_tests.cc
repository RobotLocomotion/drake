#include "drake/multibody/multibody_tree/math/spatial_velocity.h"

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"

#include "gtest/gtest.h"

#include <sstream>
#include <string>

namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::Vector3d;

// Tests default construction and proper size at compile time.
GTEST_TEST(SpatialVelocity, SizeAtCompileTime) {
  SpatialVelocity<double> V;
  EXPECT_EQ(V.size(), 6);
}

// Construction from two three dimensional vectors.
GTEST_TEST(SpatialVelocity, ConstructionFromTwo3DVectors) {
  // Linear velocity of frame B measured and expressed in frame A.
  Vector3d v_AB(1, 2, 0);

  // Angular velocity of frame B measured and expressed in frame A.
  Vector3d w_AB(0, 0, 3);

  // Spatial velocity of frame B with respect to A and expressed in A.
  SpatialVelocity<double> V_AB(w_AB, v_AB);

  // Verify compile-time size.
  EXPECT_EQ(V_AB.size(), 6);

  // Comparison to Eigen::NumTraits<double>::epsilon() precision.
  EXPECT_TRUE(V_AB.translational().isApprox(v_AB));
  EXPECT_TRUE(V_AB.rotational().isApprox(w_AB));
}

// Construction from a Eigen expressions.
GTEST_TEST(SpatialVelocity, ConstructionFromAnEigenExpression) {
  // A six-dimensional vector.
  Vector6<double> v;
  v << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  // A spatial velocity instantiated from an Eigen vector.
  SpatialVelocity<double> V1(v);

  // Verify the underlying Eigen vector matches the original vector.
  EXPECT_EQ(V1.rotational(), v.segment<3>(0));
  EXPECT_EQ(V1.translational(), v.segment<3>(3));

  // A spatial velocity instantiated from an Eigen block.
  SpatialVelocity<double> V2(v.segment<6>(0));
  EXPECT_EQ(V2.rotational(), v.segment<3>(0));
  EXPECT_EQ(V2.translational(), v.segment<3>(3));
}

class SpatialVelocityTest : public ::testing::Test {
 protected:
  // Linear velocity of a Frame Y, measured in Frame X, and expressed in a third
  // frame A.
  Vector3d v_XY_A_{1, 2, 0};

  // Angular velocity of a frame Y measured in X and expressed in A.
  Vector3d w_XY_A_{0, 0, 3};

  // Spatial velocity of a frame Y measured in X and expressed in A.
  SpatialVelocity<double> V_XY_A_{w_XY_A_, v_XY_A_};

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Tests array accessors.
TEST_F(SpatialVelocityTest, SpatialVelocityArrayAccessor) {
  // Mutable access.
  V_XY_A_[0] = 1.0;
  V_XY_A_[1] = 2.0;
  V_XY_A_[2] = 3.0;
  V_XY_A_[3] = 4.0;
  V_XY_A_[4] = 5.0;
  V_XY_A_[5] = 6.0;

  // Const access.
  const auto& V = V_XY_A_;
  EXPECT_EQ(V[0], 1.0);
  EXPECT_EQ(V[1], 2.0);
  EXPECT_EQ(V[2], 3.0);
  EXPECT_EQ(V[3], 4.0);
  EXPECT_EQ(V[4], 5.0);
  EXPECT_EQ(V[5], 6.0);
}

// Tests the (mutable) access with operator[](int).
TEST_F(SpatialVelocityTest, SpatialVelocityVectorComponentAccessors) {
  // They should be exactly equal, byte by byte.
  EXPECT_EQ(V_XY_A_.translational(), v_XY_A_);
  EXPECT_EQ(V_XY_A_.rotational(), w_XY_A_);
}

// Tests access to the raw data pointer.
TEST_F(SpatialVelocityTest, RawDataAccessors) {
  // Mutable access.
  double* mutable_data = V_XY_A_.mutable_data();
  for (int i = 0; i < 6; ++i) mutable_data[i] = i;

  // Const access.
  const double* const_data = V_XY_A_.data();
  EXPECT_EQ(V_XY_A_[0], const_data[0]);
  EXPECT_EQ(V_XY_A_[1], const_data[1]);
  EXPECT_EQ(V_XY_A_[2], const_data[2]);
  EXPECT_EQ(V_XY_A_[3], const_data[3]);
  EXPECT_EQ(V_XY_A_[4], const_data[4]);
  EXPECT_EQ(V_XY_A_[5], const_data[5]);
}

// Tests comparison to a given precision.
TEST_F(SpatialVelocityTest, IsApprox) {
  const double precision = 1.0e-10;
  SpatialVelocity<double> other(
      (1.0 + precision) * w_XY_A_, (1.0 + precision) * v_XY_A_);
  EXPECT_TRUE(V_XY_A_.IsApprox(other, (1.0 + 1.0e-7) * precision));
  EXPECT_FALSE(V_XY_A_.IsApprox(other, precision));
}

// Tests the shifting of a spatial velocity between two moving frames rigidly
// attached to each other.
TEST_F(SpatialVelocityTest, ShiftOperation) {
  // Consider a vector from the origin of a frame Y to the origin of a frame Z,
  // expressed in a third frame A.
  Vector3d p_YZ_A({2, -2, 0});

  // Consider now shifting the spatial velocity of a frame Y measured in frame
  // X to the spatial velocity of frame Z measured in frame X knowing that
  // frames Y and Z are rigidly attached to each other.
  SpatialVelocity<double> V_XZ_A = V_XY_A_.Shift(p_YZ_A);

  // Verify the result.
  SpatialVelocity<double> expected_V_XZ_A(w_XY_A_, Vector3d(7, 8, 0));
  EXPECT_TRUE(V_XZ_A.IsApprox(expected_V_XZ_A));
}

// Test the stream insertion operator to write into a stream.
TEST_F(SpatialVelocityTest, ShiftOperatorIntoStream) {
  std::stringstream stream;
  stream << V_XY_A_;
  std::string expected_string = "[0, 0, 3, 1, 2, 0]ᵀ";
  EXPECT_EQ(expected_string, stream.str());
}

// Test the multiplication of a spatial velocity by a scalar.
TEST_F(SpatialVelocityTest, MulitplicationByAScalar) {
  const double scalar = 3.0;
  SpatialVelocity<double> sxV = scalar * V_XY_A_;
  SpatialVelocity<double> Vxs = V_XY_A_ * scalar;

  // Verify the result using Eigen operations.
  EXPECT_EQ(sxV.rotational(), scalar * V_XY_A_.rotational());
  EXPECT_EQ(sxV.translational(), scalar * V_XY_A_.translational());

  // Verify the multiplication by a scalar is commutative.
  EXPECT_EQ(sxV.rotational(), Vxs.rotational());
  EXPECT_EQ(sxV.translational(), Vxs.translational());
}

}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
