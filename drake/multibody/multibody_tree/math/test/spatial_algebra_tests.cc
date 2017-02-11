#include "drake/multibody/multibody_tree/math/spatial_algebra.h"

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"

#include "gtest/gtest.h"

namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::Vector3d;

// Tests default construction and proper sizes at compile time.
GTEST_TEST(SpatialVector, SizeAtCompileTime) {
  SpatialVector<double> V;
  EXPECT_EQ(V.size(), 6);
  EXPECT_EQ(V.angular_size(), 3);
  EXPECT_EQ(V.linear_size(), 3);
}

// Construction from two three dimensional vectors.
GTEST_TEST(SpatialVector, ConstructionFromTwo3DVectors) {
  // Linear velocity of frame B measured and expressed in frame A.
  Vector3d v_AB(1, 2, 0);

  // Angular velocity of frame B measured and expressed in frame A.
  Vector3d w_AB(0, 0, 3);

  // Spatial velocity of frame B with respect to A and expressed in A.
  SpatialVector<double> V_AB(w_AB, v_AB);

  // Verify compile-time sizes.
  EXPECT_EQ(V_AB.size(), 6);
  EXPECT_EQ(V_AB.angular_size(), 3);
  EXPECT_EQ(V_AB.linear_size(), 3);

  // Comparison to Eigen::NumTraits<double>::epsilon() precision.
  EXPECT_TRUE(V_AB.linear().isApprox(v_AB));
  EXPECT_TRUE(V_AB.angular().isApprox(w_AB));
}

class SpatialVectorTest : public ::testing::Test {
 protected:
  // Linear velocity of a frame Y measured in X and expressed in A.
  Vector3d v_XY_A_{1, 2, 0};

  // Angular velocity of a frame Y measured in X and expressed in A.
  Vector3d w_XY_A_{0, 0, 3};

  // Spatial velocity of a frame Y measured in X and expressed in A.
  SpatialVector<double> V_XY_A_{w_XY_A_, v_XY_A_};

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Tests array accessors.
TEST_F(SpatialVectorTest, SpatialVectorArrayAccessor) {
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
TEST_F(SpatialVectorTest, SpatialVectorVectorComponentAccessors) {
  // They should be exactly equal, byte by byte.
  EXPECT_EQ(V_XY_A_.linear(), v_XY_A_);
  EXPECT_EQ(V_XY_A_.angular(), w_XY_A_);
}

// Tests access as an Eigen vector.
TEST_F(SpatialVectorTest, EigenAccess) {
  Vector6<double> V = V_XY_A_.get_coeffs();
  Vector6<double> V_expected;
  V_expected << w_XY_A_, v_XY_A_;
  EXPECT_EQ(V, V_expected);
}

// Tests access to the raw data pointer.
TEST_F(SpatialVectorTest, RawDataAccessors) {
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
TEST_F(SpatialVectorTest, IsApprox) {
  const double precision = 1.0e-10;
  SpatialVector<double> other(
      (1.0 + precision) * w_XY_A_, (1.0 + precision) * v_XY_A_);
  EXPECT_TRUE(V_XY_A_.IsApprox(other, 1.01 * precision));
  EXPECT_FALSE(V_XY_A_.IsApprox(other, 0.99 * precision));
}

// Tests the transformation of a spatial velocity between two frames using the
// spatial operator and its transpose.
TEST_F(SpatialVectorTest, ShiftOperator) {
  // A shift operator representing the rigid body transformation from frame Z
  // to frame X, expressed in a third frame A.
  ShiftOperator<double> phi_YZ_A({2, -2, 0});

  // Perform the actual shift operation.
  SpatialVector<double> V_XZ_A = phi_YZ_A.transpose() * V_XY_A_;

  // Verify the result.
  SpatialVector<double> expected_V_XZ_A(w_XY_A_, {7, 8, 0});
  EXPECT_TRUE(V_XZ_A.IsApprox(expected_V_XZ_A));
}

}
}  // math
}  // multibody
}  // drake