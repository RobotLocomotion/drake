#include "drake/multibody/multibody_tree/math/spatial_algebra.h"

#include <sstream>
#include <string>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace math {
namespace {

// Generic declaration of a traits class to figure out at compile time the
// scalar type a spatial quantity is instantiated with.
template <class SpatialQuantity> struct spatial_vector_traits {};

// traits specialization for SpatialVelocity.
template <typename T>
struct spatial_vector_traits<SpatialVelocity<T>> {
  typedef T ScalarType;
};

// traits specialization for SpatialForce.
template <typename T>
struct spatial_vector_traits<SpatialForce<T>> {
  typedef T ScalarType;
};

template <class SpatialQuantityUnderTest>
class SpatialQuantityTest : public ::testing::Test {
 public:
  // Useful typedefs when writing unit tests to access types.
  typedef SpatialQuantityUnderTest SpatialQuantityType;
  typedef typename spatial_vector_traits<
      SpatialQuantityType>::ScalarType ScalarType;
 protected:
  // A translational component ∈ ℝ³.
  Vector3<ScalarType> v_{1, 2, 0};

  // A rotational component ∈ ℝ³.
  Vector3<ScalarType> w_{0, 0, 3};

  // A spatial quantity related to the above rotational and translational
  // components.
  SpatialQuantityType V_{w_, v_};
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

  // A translational component ∈ ℝ³.
  Vector3<T> v_AB(1, 2, 0);

  // An rotational component ∈ ℝ³.
  Vector3<T> w_AB(0, 0, 3);

  // A spatial quantity related to the above rotational and translational
  // components.
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
  SpatialQuantity& V = this->V_;

  // Mutable access.
  V[0] = 1.0;
  V[1] = 2.0;
  V[2] = 3.0;
  V[3] = 4.0;
  V[4] = 5.0;
  V[5] = 6.0;

  // Const access.
  const auto& constV = V;
  EXPECT_EQ(constV[0], 1.0);
  EXPECT_EQ(constV[1], 2.0);
  EXPECT_EQ(constV[2], 3.0);
  EXPECT_EQ(constV[3], 4.0);
  EXPECT_EQ(constV[4], 5.0);
  EXPECT_EQ(constV[5], 6.0);
}

// Tests the (mutable) access with operator[](int).
TYPED_TEST(SpatialQuantityTest, SpatialVelocityVectorComponentAccessors) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  const SpatialQuantity& V = this->V_;
  const Vector3<T>& v = this->v_;
  const Vector3<T>& w = this->w_;

  // They should be exactly equal, byte by byte.
  EXPECT_EQ(V.translational(), v);
  EXPECT_EQ(V.rotational(), w);
}

// Tests access to the raw data pointer.
TYPED_TEST(SpatialQuantityTest, RawDataAccessors) {
  typedef TypeParam SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  SpatialQuantity& V = this->V_;

  // Mutable access.
  T* mutable_data = V.mutable_data();
  for (int i = 0; i < 6; ++i) mutable_data[i] = i;

  // Const access.
  const T* const_data = V.data();
  EXPECT_EQ(V[0], const_data[0]);
  EXPECT_EQ(V[1], const_data[1]);
  EXPECT_EQ(V[2], const_data[2]);
  EXPECT_EQ(V[3], const_data[3]);
  EXPECT_EQ(V[4], const_data[4]);
  EXPECT_EQ(V[5], const_data[5]);
}

// Tests comparison to a given precision.
TYPED_TEST(SpatialQuantityTest, IsApprox) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  const SpatialQuantity& V = this->V_;
  const Vector3<T>& v = this->v_;
  const Vector3<T>& w = this->w_;

  const double precision = 1.0e-10;
  SpatialQuantity other(
      (1.0 + precision) * w, (1.0 + precision) * v);
  EXPECT_TRUE(V.IsApprox(other, (1.0 + 1.0e-7) * precision));
  EXPECT_FALSE(V.IsApprox(other, precision));
}

// Test the stream insertion operator to write into a stream.
TYPED_TEST(SpatialQuantityTest, ShiftOperatorIntoStream) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  const SpatialQuantity& V = this->V_;

  std::stringstream stream;
  stream << V;
  std::string expected_string = "[0, 0, 3, 1, 2, 0]ᵀ";
  EXPECT_EQ(expected_string, stream.str());
}

// Test the multiplication of a spatial quantity by a scalar.
TYPED_TEST(SpatialQuantityTest, MulitplicationByAScalar) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  const SpatialQuantity& V = this->V_;

  const T scalar = 3.0;
  SpatialQuantity sxV = scalar * V;
  SpatialQuantity Vxs = V * scalar;

  // Verify the result using Eigen operations.
  EXPECT_EQ(sxV.rotational(), scalar * V.rotational());
  EXPECT_EQ(sxV.translational(), scalar * V.translational());

  // Verify the multiplication by a scalar is commutative.
  EXPECT_EQ(sxV.rotational(), Vxs.rotational());
  EXPECT_EQ(sxV.translational(), Vxs.translational());
}

// Create a list of scalar types for the unit tests that follow below.
typedef ::testing::Types<double, AutoDiffXd> ScalarTypes;

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
};
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

// Tests that we can take the dot product of a SpatialVelocity with a
// SpatialForce.
TYPED_TEST(SpatialVelocityTest, DotProductWithSpatialForce) {
  typedef typename TestFixture::ScalarType T;

  SpatialVelocity<T> V(Vector3<T>(1.0, 2.0, 3.0), /*rotational component*/
                       Vector3<T>(-2.0, -5.0, 8.0) /*translational component*/);
  SpatialForce<T> F(Vector3<T>(4.0, 1.0, -3.0), /*rotational component*/
                    Vector3<T>(11.0, 9.0, -2.0) /*translational component*/);
  T VdotF = V.dot(F);
  T VdotF_expected(-86);
  // Verify the result.
  EXPECT_EQ(VdotF, VdotF_expected);

  // Verify the dot product is commutative.
  T FdotV = F.dot(V);
  // Verify the result.
  EXPECT_EQ(FdotV, VdotF_expected);
}

// SpatialForce specific unit tests.
template <typename T>
class SpatialForceTest : public ::testing::Test {
 public:
  // Useful typedefs when witting unit tests to access types.
  typedef T ScalarType;
 protected:
  // Consider a force applied at the origin of Frame A, expressed in a Frame E.
  Vector3<ScalarType> f_Ao_E_{1, 2, 0};

  // Consider a torque applied about the origin of Frame A, expressed in a
  // Frame E.
  Vector3<ScalarType> tau_Ao_E_{0, 0, 3};

  // Consider a spatial force about the origin of Frame A, expressed in a
  // frame A.
  SpatialForce<ScalarType> F_Ao_E_{tau_Ao_E_, f_Ao_E_};
};
TYPED_TEST_CASE(SpatialForceTest, ScalarTypes);

// Tests the shifting of a spatial force between two moving frames rigidly
// attached to each other.
TYPED_TEST(SpatialForceTest, ShiftOperation) {
  typedef typename TestFixture::ScalarType T;
  const SpatialForce<T>& F_Ao_E = this->F_Ao_E_;
  const Vector3<T>& f_Ao_E = this->f_Ao_E_;

  // Consider a vector from the origin of a frame A to the origin of a frame B,
  // expressed in a third frame E.
  Vector3<T> p_AB_E(2., -2., 1.);

  // Consider now shifting the spatial force as applied about the origin of
  // frame A measured in frame E to the spatial force as applied about frame B
  // also measured in frame E knowing that frames A and B are rigidly attached
  // to each other.
  SpatialForce<T> F_Bo_E = F_Ao_E.Shift(p_AB_E);

  // Verify the result.
  SpatialForce<T> expected_F_Bo_E(Vector3<T>(2.0, -1.0, -3.0), f_Ao_E);
  EXPECT_TRUE(F_Bo_E.IsApprox(expected_F_Bo_E));
}

}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
