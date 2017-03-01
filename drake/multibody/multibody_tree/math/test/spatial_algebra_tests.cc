#include "drake/multibody/multibody_tree/math/spatial_velocity.h"
#include "drake/multibody/multibody_tree/math/spatial_force.h"

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/common/eigen_autodiff_types.h"

#include "gtest/gtest.h"

#include <sstream>
#include <string>

namespace drake {
namespace multibody {
namespace math {
namespace {

template <class SpatialQuantityUnderTest>
class SpatialQuantityTest : public ::testing::Test {
 public:
  // Useful typedefs when witting unit tests to access types.
  typedef SpatialQuantityUnderTest SpatialQuantityType;
  typedef typename internal::spatial_vector_traits<
      SpatialQuantityType>::ScalarType ScalarType;
 protected:
  // Linear velocity of a Frame Y, measured in Frame X, and expressed in a third
  // frame A.
  Vector3<ScalarType> v_XY_A_{1, 2, 0};

  // Angular velocity of a frame Y measured in X and expressed in A.
  Vector3<ScalarType> w_XY_A_{0, 0, 3};

  // Spatial velocity of a frame Y measured in X and expressed in A.
  SpatialQuantityType V_XY_A_{w_XY_A_, v_XY_A_};

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Create a list of SpatialVector types to be tested.
typedef ::testing::Types<
    SpatialVelocity<double>, SpatialForce<double>,
    SpatialVelocity<AutoDiffXd>, SpatialForce<AutoDiffXd>> SpatialQuantityTypes;
TYPED_TEST_CASE(SpatialQuantityTest, SpatialQuantityTypes);

// Tests default construction and proper size at compile time.
TYPED_TEST(SpatialQuantityTest, SizeAtCompileTime) {
  TypeParam V;
  EXPECT_EQ(V.size(), 6);
}

// Construction from a Eigen expressions.
TYPED_TEST(SpatialQuantityTest, ConstructionFromAnEigenExpression) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;

  // A six-dimensional vector.
  Vector6<T> v;
  v << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  // A spatial quantity instantiated from an Eigen vector.
  SpatialQuantity V1(v);

  // Verify the underlying Eigen vector matches the original vector.
  EXPECT_EQ(V1.rotational(), v.template segment<3>(0));
  EXPECT_EQ(V1.translational(), v.template segment<3>(3));

  // A spatial quantity instantiated from an Eigen block.
  SpatialQuantity V2(v.template segment<6>(0));
  EXPECT_EQ(V2.rotational(), v.template segment<3>(0));
  EXPECT_EQ(V2.translational(), v.template segment<3>(3));
}

// Construction from two three-dimensional vectors.
TYPED_TEST(SpatialQuantityTest, ConstructionFromTwo3DVectors) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;

  // Linear velocity of frame B measured and expressed in frame A.
  Vector3<T> v_AB(1, 2, 0);

  // Angular velocity of frame B measured and expressed in frame A.
  Vector3<T> w_AB(0, 0, 3);

  // Spatial velocity of frame B with respect to A and expressed in A.
  SpatialQuantity V_AB(w_AB, v_AB);

  // Verify compile-time size.
  EXPECT_EQ(V_AB.size(), 6);

  // Comparison to Eigen::NumTraits<double>::epsilon() precision.
  EXPECT_TRUE(V_AB.translational().isApprox(v_AB));
  EXPECT_TRUE(V_AB.rotational().isApprox(w_AB));
}

// Tests array accessors.
TYPED_TEST(SpatialQuantityTest, SpatialVelocityArrayAccessor) {
  typedef TypeParam SpatialQuantity;
  SpatialQuantity& V_XY_A = this->V_XY_A_;

  // Mutable access.
  V_XY_A[0] = 1.0;
  V_XY_A[1] = 2.0;
  V_XY_A[2] = 3.0;
  V_XY_A[3] = 4.0;
  V_XY_A[4] = 5.0;
  V_XY_A[5] = 6.0;

  // Const access.
  const auto& V = V_XY_A;
  EXPECT_EQ(V[0], 1.0);
  EXPECT_EQ(V[1], 2.0);
  EXPECT_EQ(V[2], 3.0);
  EXPECT_EQ(V[3], 4.0);
  EXPECT_EQ(V[4], 5.0);
  EXPECT_EQ(V[5], 6.0);
}

// Tests the (mutable) access with operator[](int).
TYPED_TEST(SpatialQuantityTest, SpatialVelocityVectorComponentAccessors) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  const SpatialQuantity& V_XY_A = this->V_XY_A_;
  const Vector3<T>& v_XY_A = this->v_XY_A_;
  const Vector3<T>& w_XY_A = this->w_XY_A_;

  // They should be exactly equal, byte by byte.
  EXPECT_EQ(V_XY_A.translational(), v_XY_A);
  EXPECT_EQ(V_XY_A.rotational(), w_XY_A);
}

// Tests access to the raw data pointer.
TYPED_TEST(SpatialQuantityTest, RawDataAccessors) {
  typedef TypeParam SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  SpatialQuantity& V_XY_A = this->V_XY_A_;

  // Mutable access.
  T* mutable_data = V_XY_A.mutable_data();
  for (int i = 0; i < 6; ++i) mutable_data[i] = i;

  // Const access.
  const T* const_data = V_XY_A.data();
  EXPECT_EQ(V_XY_A[0], const_data[0]);
  EXPECT_EQ(V_XY_A[1], const_data[1]);
  EXPECT_EQ(V_XY_A[2], const_data[2]);
  EXPECT_EQ(V_XY_A[3], const_data[3]);
  EXPECT_EQ(V_XY_A[4], const_data[4]);
  EXPECT_EQ(V_XY_A[5], const_data[5]);
}

// Tests comparison to a given precision.
TYPED_TEST(SpatialQuantityTest, IsApprox) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  const SpatialQuantity& V_XY_A = this->V_XY_A_;
  const Vector3<T>& v_XY_A = this->v_XY_A_;
  const Vector3<T>& w_XY_A = this->w_XY_A_;

  const double precision = 1.0e-10;
  SpatialQuantity other(
      (1.0 + precision) * w_XY_A, (1.0 + precision) * v_XY_A);
  EXPECT_TRUE(V_XY_A.IsApprox(other, (1.0 + 1.0e-7) * precision));
  EXPECT_FALSE(V_XY_A.IsApprox(other, precision));
}

// Test the stream insertion operator to write into a stream.
TYPED_TEST(SpatialQuantityTest, ShiftOperatorIntoStream) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  const SpatialQuantity& V_XY_A = this->V_XY_A_;

  std::stringstream stream;
  stream << V_XY_A;
  std::string expected_string = "[0, 0, 3, 1, 2, 0]ᵀ";
  EXPECT_EQ(expected_string, stream.str());
}

// Test the multiplication of a spatial velocity by a scalar.
TYPED_TEST(SpatialQuantityTest, MulitplicationByAScalar) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  const SpatialQuantity& V_XY_A = this->V_XY_A_;

  const T scalar = 3.0;
  SpatialQuantity sxV = scalar * V_XY_A;
  SpatialQuantity Vxs = V_XY_A * scalar;

  // Verify the result using Eigen operations.
  EXPECT_EQ(sxV.rotational(), scalar * V_XY_A.rotational());
  EXPECT_EQ(sxV.translational(), scalar * V_XY_A.translational());

  // Verify the multiplication by a scalar is commutative.
  EXPECT_EQ(sxV.rotational(), Vxs.rotational());
  EXPECT_EQ(sxV.translational(), Vxs.translational());
}

// SpatialVelocity specific unit tests.
template <typename T>
class SpatialVelocityTest : public ::testing::Test {
 public:
  // Useful typedefs when witting unit tests to access types.
  typedef T ScalarType;
 protected:
  // Linear velocity of a Frame Y, measured in Frame X, and expressed in a third
  // frame A.
  Vector3<ScalarType> v_XY_A_{1, 2, 0};

  // Angular velocity of a frame Y measured in X and expressed in A.
  Vector3<ScalarType> w_XY_A_{0, 0, 3};

  // Spatial velocity of a frame Y measured in X and expressed in A.
  SpatialVelocity<ScalarType> V_XY_A_{w_XY_A_, v_XY_A_};

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Create a list of scalar types for specific tests on different spatial
// quantities.
typedef ::testing::Types<double, AutoDiffXd> ScalarTypes;
TYPED_TEST_CASE(SpatialVelocityTest, ScalarTypes);

// Tests the shifting of a spatial velocity between two moving frames rigidly
// attached to each other.
TYPED_TEST(SpatialVelocityTest, ShiftOperation) {
  typedef typename TestFixture::ScalarType T;
  const SpatialVelocity<T>& V_XY_A = this->V_XY_A_;
  const Vector3<T>& w_XY_A = this->w_XY_A_;

  // Consider a vector from the origin of a frame Y to the origin of a frame Z,
  // expressed in a third frame A.
  Vector3<T> p_YZ_A(2., -2., 0.);

  // Consider now shifting the spatial velocity of a frame Y measured in frame
  // X to the spatial velocity of frame Z measured in frame X knowing that
  // frames Y and Z are rigidly attached to each other.
  SpatialVelocity<T> V_XZ_A = V_XY_A.Shift(p_YZ_A);

  // Verify the result.
  SpatialVelocity<T> expected_V_XZ_A(w_XY_A, Vector3<T>(7, 8, 0));
  EXPECT_TRUE(V_XZ_A.IsApprox(expected_V_XZ_A));
}

}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
