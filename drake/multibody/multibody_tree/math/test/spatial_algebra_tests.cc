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

using Eigen::AngleAxis;

// Generic declaration boilerplate of a traits class for spatial vectors.
// This is used by the (templated on SpatialQuantityUnderTest)
// SpatialQuantityTest unit test class below to infer on what scalar type the
// SpatialQuantityUnderTest is templated on.
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

// traits specialization for SpatialAcceleration.
template <typename T>
struct spatial_vector_traits<SpatialAcceleration<T>> {
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
    SpatialVelocity<double>,
    SpatialForce<double>,
    SpatialAcceleration<double>,
    SpatialVelocity<AutoDiffXd>,
    SpatialForce<AutoDiffXd>,
    SpatialAcceleration<AutoDiffXd>> SpatialQuantityTypes;
TYPED_TEST_CASE(SpatialQuantityTest, SpatialQuantityTypes);

// Tests default construction and proper size at compile time.
TYPED_TEST(SpatialQuantityTest, SizeAtCompileTime) {
  TypeParam V;
  EXPECT_EQ(V.size(), 6);
}

// Construction of a "zero" spatial vector.
TYPED_TEST(SpatialQuantityTest, ZeroFactory) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  SpatialQuantity V = SpatialQuantity::Zero();
  EXPECT_TRUE(V.rotational() == Vector3<T>::Zero());
  EXPECT_TRUE(V.translational() == Vector3<T>::Zero());
}

// Tests:
// - Construction from a Eigen expressions.
// - SetZero() method.
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

  // Unit tests SetZero().
  V1.SetZero();
  EXPECT_EQ(V1.rotational(), Vector3<T>::Zero());
  EXPECT_EQ(V1.translational(), Vector3<T>::Zero());
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

// Re-express in another frame. Given a spatial vector V_F expressed in a frame
// F, re-express this same spatial vector in another frame E as:
//   V_E = R_EF * V_F
// where R_EF is the rotation matrix from frame F into frame E.
TYPED_TEST(SpatialQuantityTest, ReExpressInAnotherFrame) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  const SpatialQuantity& V_AB_F = this->V_;

  // Some arbitrary rotation between frames E and F.
  const Matrix3<T> R_EF =
      (AngleAxis<T>(M_PI / 6, Vector3<T>::UnitX()) *
       AngleAxis<T>(M_PI / 6, Vector3<T>::UnitY()) *
       AngleAxis<T>(M_PI / 6, Vector3<T>::UnitZ())).matrix();

  SpatialQuantity V_AB_E = R_EF * V_AB_F;

  // Verify the result using Eigen operations.
  EXPECT_EQ(V_AB_E.rotational(), R_EF * V_AB_F.rotational());
  EXPECT_EQ(V_AB_E.translational(), R_EF * V_AB_F.translational());
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

// Unit tests for the composition of spatial velocities:
// - Shift(): shift of a spatial velocity between two moving frames rigidly
//            attached to each other.
// - ComposeWithMovingFrameVelocity(): compose the velocity V_XY of a frame Y
//   in X with that of a third frame Z moving in Y with velocity V_YZ.
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
  SpatialVelocity<T> V_XYz_A = V_XY_A.Shift(p_YZ_A);

  // Verify the result.
  SpatialVelocity<T> expected_V_XYz_A(w_XY_A, Vector3<T>(7, 8, 0));
  EXPECT_TRUE(V_XYz_A.IsApprox(expected_V_XYz_A));

  // ComposeWithMovingFrameVelocity() should yield the same result.
  EXPECT_TRUE(V_XY_A.ComposeWithMovingFrameVelocity(
      p_YZ_A, SpatialVelocity<T>::Zero()).IsApprox(V_XYz_A));

  // Unit test with the composition of the spatial velocity of a moving frame Z
  // in frame Y.
  const SpatialVelocity<T> V_YZ_A(Vector3<T>(1.0, 2.0, 3.0),
                                  Vector3<T>(4.0, 5.0, 6.0));
  const SpatialVelocity<T> V_XZ_A =
      V_XY_A.ComposeWithMovingFrameVelocity(p_YZ_A, V_YZ_A);

  // Verify the result: V_XZ = V_XYz + V_YZ = V_XY.Shift(p_YZ) + V_YZ
  SpatialVelocity<T> expected_V_XZ_A = expected_V_XYz_A + V_YZ_A;
  EXPECT_TRUE(V_XZ_A.IsApprox(expected_V_XZ_A));
}

// Tests operator+().
TYPED_TEST(SpatialVelocityTest, AdditionOperation) {
  typedef typename TestFixture::ScalarType T;
  const SpatialVelocity<T>& V_XY_A = this->V_XY_A_;
  const Vector3<T>& w_XY_A = this->w_XY_A_;
  const Vector3<T>& v_XY_A = this->v_XY_A_;

  SpatialVelocity<T> V = V_XY_A + V_XY_A;

  EXPECT_EQ(V.rotational(), w_XY_A + w_XY_A);
  EXPECT_EQ(V.translational(), v_XY_A + v_XY_A);
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

// SpatialAcceleration specific unit tests.
template <typename T>
class SpatialAccelerationTest : public ::testing::Test {
 public:
  // Useful typedefs when witting unit tests to access types.
  typedef T ScalarType;
};
TYPED_TEST_CASE(SpatialAccelerationTest, ScalarTypes);

// Unit test for the method SpatialAcceleration::Shift().
// Case 1:
// In this test, a frame P rotates with respect to a frame A with an angular
// velocity w_AP and has zero acceleration in frame A, ie. A_AP = 0. We can
// think of frames P and A having coincident origins.
// A third frame Q translated by a position p_PoQo moves rigidly with P.
// The angular velocity vector w_AP_E is orthogonal to the x-y plane while the
// offset vector p_PoQo_E is in the x-y plane.
// Therefore, the spatial acceleration of frame Q should correspond to that of
// a centrifugal linear component pointing inwards in the opposite direction of
// p_PoQo
TYPED_TEST(SpatialAccelerationTest, CentrifugalAcceleration) {
  typedef typename TestFixture::ScalarType T;

  // The spatial acceleration of frame P measured in A is zero.
  const SpatialAcceleration<T> A_AP = SpatialAcceleration<T>::Zero();

  // Position of Q's origin measured in P and expressed in E.
  const Vector3<T> p_PoQo_E = Vector3<T>::UnitX() + Vector3<T>::UnitY();

  // Angular velocity of frame P measured in frame A.
  const Vector3<T> w_AP_E = 3.0 * Vector3<T>::UnitZ();

  const SpatialAcceleration<T> A_AQ = A_AP.Shift(p_PoQo_E, w_AP_E);

  SpatialAcceleration<T> A_AQ_expected;
  A_AQ_expected.rotational() = Vector3<T>::Zero();
  // The centrifugal acceleration has magnitude w_AP^2 * ‖ p_PoQo ‖ and points
  // in the direction opposite to p_PoQo.
  A_AQ_expected.translational() =
      -w_AP_E.norm() * w_AP_E.norm() * p_PoQo_E.norm() * p_PoQo_E.normalized();

  EXPECT_TRUE(A_AQ.IsApprox(A_AQ_expected));

  // The result from ComposeWithMovingFrameAcceleration() should be the same
  // when velocity V_PQ and acceleration A_PQ are both zero:
  const SpatialAcceleration<T> A_AQ_moving =
      A_AP.ComposeWithMovingFrameAcceleration(
          p_PoQo_E, w_AP_E, /* Same arguments as in the Shift() operator */
          SpatialVelocity<T>::Zero() /* V_PQ */,
          SpatialAcceleration<T>::Zero() /* A_PQ */);
  EXPECT_TRUE(A_AQ.IsApprox(A_AQ_moving));
}

// Unit test for the method
// SpatialAcceleration::ComposeWithMovingFrameAcceleration().
// Case 1b:
// This unit test expands Case 1 by allowing point Q to move in frame P. This
// motion causes, in addition to the centrifugal acceleration of Case 1, a
// Coriolis acceleration due to the translational motion of point Q in the
// rotating frame P.
// Point Q has a spatial velocity V_PQ with a rotational component w_PQ along
// the z-axis and a translational component v_PQ in the x-y plane, pointing in
// the minus x direction in the P frame.
// The translational velocity v_PQ is responsible for a Coriolis acceleration as
// the result of the cross product w_AP with v_PQ, which then points radially
// outwards counteracting the centrifugal contribution.
TYPED_TEST(SpatialAccelerationTest, CoriolisAcceleration) {
  typedef typename TestFixture::ScalarType T;

  // The spatial acceleration of frame P measure in A is zero.
  const SpatialAcceleration<T> A_AP = SpatialAcceleration<T>::Zero();

  // Position of Q's origin measured in P and expressed in E.
  const Vector3<T> p_PoQo = Vector3<T>::UnitX() + Vector3<T>::UnitY();

  // Angular velocity of frame P measured in frame A.
  const Vector3<T> w_AP = 3.0 * Vector3<T>::UnitZ();

  // Spatial velocity of Q in P.
  const SpatialVelocity<T> V_PQ(
      1.5 * Vector3<T>::UnitZ() /* w_PQ */,
      2.0 * Vector3<T>::UnitX() /* v_PQ */);

  // Spatial acceleration of Q in P.
  const SpatialAcceleration<T> A_PQ = SpatialAcceleration<T>::Zero();

  // In this test, at this instantaneous moment, R_AP is the identity matrix and
  // therefore p_PoQo_A = p_PoQo_P. Similarly for V_PQ, A_PQ and w_AP.
  const SpatialAcceleration<T> A_AQ =
      A_AP.ComposeWithMovingFrameAcceleration(p_PoQo, w_AP, V_PQ, A_PQ);

  SpatialAcceleration<T> A_AQ_expected;
  A_AQ_expected.rotational() = Vector3<T>::Zero();
  A_AQ_expected.translational() =
      /* The centrifugal contribution has magnitude w_AP^2 * ‖ p_PoQo ‖ and
      points in the direction opposite to p_PoQo.*/
      -w_AP.norm() * w_AP.norm() * p_PoQo.norm() * p_PoQo.normalized() +
      /* Coriolis contribution. Since v_PQ points in the x direction and w_AP in
      the z direction, this contribution points in the positive y direction.*/
      2.0 * w_AP.norm() * V_PQ.translational().norm() * Vector3<T>::UnitY();

  EXPECT_TRUE(A_AQ.IsApprox(A_AQ_expected));
}

// Unit test for the method SpatialAcceleration::Shift().
// Case 2:
// This unit test is similar to the previous Case 1 test but with the offset
// vector p_PoQo aligned with w_AP. Therefore the centrifugal contribution is
// zero. In this case the spatial acceleration A_AP has zero translational
// component but non-zero rotational component alpha_AP.
TYPED_TEST(SpatialAccelerationTest, NoCentrifugalAcceleration) {
  typedef typename TestFixture::ScalarType T;

  // Angular acceleration of frame P in A, expressed in E.
  const Vector3<T> alpha_AP_E = 2.0 * Vector3<T>::UnitY();

  // The spatial acceleration of frame P measure in A.
  const SpatialAcceleration<T> A_AP(alpha_AP_E, Vector3<T>::Zero());

  // Angular velocity of frame P measured in frame A.
  const Vector3<T> w_AP_E = 3.0 * Vector3<T>::UnitZ();

  // Position of Q's origin measured in P and expressed in E. In this case
  // p_PoQo is aligned with w_AP.
  const Vector3<T> p_PoQo_E = Vector3<T>::UnitZ();

  const SpatialAcceleration<T> A_AQ = A_AP.Shift(p_PoQo_E, w_AP_E);

  SpatialAcceleration<T> A_AQ_expected;
  // The rotational component does not change.
  A_AQ_expected.rotational() = alpha_AP_E;

  // In this case the only contribution to the translational acceleration comes
  // from the angular acceleration of frame P in A.
  A_AQ_expected.translational() =
      alpha_AP_E.norm() * p_PoQo_E.norm() * Vector3<T>::UnitX();

  EXPECT_TRUE(A_AQ.IsApprox(A_AQ_expected));
}

// Unit test for the method SpatialAcceleration::Shift().
// Case 3:
// This test is a combination of Case 1 (with centrifugal acceleration), Case 2
// (with angular acceleration) and a translational acceleration a_AP of frame P
// in A.
TYPED_TEST(SpatialAccelerationTest, WithTranslationalAcceleration) {
  typedef typename TestFixture::ScalarType T;
  using std::sqrt;

  // Angular acceleration of frame P in A, expressed in E.
  const Vector3<T> alpha_AP_E = 2.0 * Vector3<T>::UnitY();

  // The spatial acceleration of frame P measure in A.
  const SpatialAcceleration<T> A_AP(alpha_AP_E, 1.5 * Vector3<T>::UnitY());

  // Position of Q's origin measured in P and expressed in E.
  const Vector3<T> p_PoQo_E = Vector3<T>::UnitX() + Vector3<T>::UnitY();

  // Angular velocity of frame P measured in frame A.
  const Vector3<T> w_AP_E = 3.0 * Vector3<T>::UnitZ();

  SpatialAcceleration<T> A_AQ_expected;
  // The rotational component does not change.
  A_AQ_expected.rotational() = alpha_AP_E;

  A_AQ_expected.translational() =
      /* Contribution due to the translational acceleration of frame P in A. */
      A_AP.translational() +
      /* Centrifugal contribution has magnitude w_AP^2 * ‖ p_PoQo ‖ and points
      in the direction opposite to p_PoQo. */
      -w_AP_E.norm() * w_AP_E.norm() * p_PoQo_E.norm() * p_PoQo_E.normalized() +
      /* Contribution due to the angular acceleration of frame P in A.
      The sqrt(2) factor comes from the angle between alpha_AP and p_PoQo. */
      -alpha_AP_E.norm() * p_PoQo_E.norm() * Vector3<T>::UnitZ() / sqrt(2);

  const SpatialAcceleration<T> A_AQ = A_AP.Shift(p_PoQo_E, w_AP_E);

  EXPECT_TRUE(A_AQ.IsApprox(A_AQ_expected));
}

// TODO(sherm1,mitiguy) Add independently-developed unit tests here by Mitiguy
// for the scary Shift() and Compose() methods, just to double check!

}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
