#include "drake/multibody/math/spatial_algebra.h"

#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/extract_double.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::AngleAxis;
using symbolic::Expression;
using symbolic::MakeVectorContinuousVariable;
using symbolic::Variable;

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

// traits specialization for SpatialMomentum.
template <typename T>
struct spatial_vector_traits<SpatialMomentum<T>> {
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

  // Two non-zero arbitrary spatial vector.
  const Vector3<ScalarType> r1_{1.0, 2.0, 3.0};
  const Vector3<ScalarType> t1_{4.0, 5.0, 6.0};
  const SpatialQuantityType V1_{r1_, t1_};
  const Vector3<ScalarType> r2_{-1.0, 1.5, -0.5};
  const Vector3<ScalarType> t2_{5.0, 7.0, 8.0};
  const SpatialQuantityType V2_{r2_, t2_};
};

// Create a list of SpatialVector types to be tested.
typedef ::testing::Types<
    SpatialVelocity<double>,
    SpatialForce<double>,
    SpatialAcceleration<double>,
    SpatialMomentum<double>,
    SpatialVelocity<AutoDiffXd>,
    SpatialForce<AutoDiffXd>,
    SpatialAcceleration<AutoDiffXd>,
    SpatialMomentum<AutoDiffXd>,
    SpatialVelocity<Expression>,
    SpatialForce<Expression>,
    SpatialAcceleration<Expression>,
    SpatialMomentum<Expression>> SpatialQuantityTypes;
TYPED_TEST_SUITE(SpatialQuantityTest, SpatialQuantityTypes);

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

  EXPECT_TRUE(V_AB.translational() == v_AB);
  EXPECT_TRUE(V_AB.rotational() == w_AB);
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

// Tests maximum absolute difference between two spatial vectors.
TYPED_TEST(SpatialQuantityTest, GetMaximumAbsoluteDifferences) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  const double w_delta = 2.2 * kEpsilon;
  const double v_delta = 4.4 * kEpsilon;
  const Vector3<T> wA(1.1, 2.2, -3.3), vA(9.9, 8.8, -7.7);
  const Vector3<T> wB = wA + Vector3<T>(w_delta, w_delta, -w_delta);
  const Vector3<T> vB = vA + Vector3<T>(v_delta, v_delta, -v_delta);
  const SpatialQuantity A(wA, vA), B(wB, vB);

  T w_difference, v_difference;
  std::tie(w_difference, v_difference) = A.GetMaximumAbsoluteDifferences(B);

  // Compare actual results with expected results.
  using std::abs;
  EXPECT_TRUE(w_difference < 4 * w_delta);
  EXPECT_TRUE(v_difference < 4 * v_delta);
  EXPECT_FALSE(w_difference < 0.25 * w_delta);
  EXPECT_FALSE(v_difference < 0.25 * v_delta);
}

// Tests the comparison between two spatial vectors (with absolute tolerances).
TYPED_TEST(SpatialQuantityTest, IsNearlyEqualWithinAbsoluteTolerance) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  const double w_delta = 2.2 * kEpsilon;
  const double v_delta = 4.4 * kEpsilon;
  const Vector3<T> wA(1.1, 2.2, -3.3), vA(9.9, 8.8, -7.7);
  const Vector3<T> wB = wA + Vector3<T>(w_delta, w_delta, -w_delta);
  const Vector3<T> vB = vA + Vector3<T>(v_delta, v_delta, -v_delta);
  const SpatialQuantity A(wA, vA), B(wB, vB);

  EXPECT_TRUE(A.IsNearlyEqualWithinAbsoluteTolerance(B, 4 * w_delta,
                                                     4 * v_delta));
  EXPECT_FALSE(A.IsNearlyEqualWithinAbsoluteTolerance(B, 0.25 * w_delta,
                                                      0.25 * v_delta));
}

// Tests comparison to a given precision.
TYPED_TEST(SpatialQuantityTest, IsApprox) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  const SpatialQuantity& V = this->V_;
  const Vector3<T>& v = this->v_;
  const Vector3<T>& w = this->w_;

  const double precision = 1.0e-10;
  const T max_v = v.template lpNorm<Eigen::Infinity>();
  const T max_w = w.template lpNorm<Eigen::Infinity>();
  using std::max;
  const double max_V = ExtractDoubleOrThrow(max(max_v, max_w));
  SpatialQuantity other(
      (1.0 + precision) * w, (1.0 + precision) * v);
  EXPECT_TRUE(V.IsApprox(other, (max_V + 1.0e-6) * precision));
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

TYPED_TEST(SpatialQuantityTest, MultiplicationAssignmentOperator) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  const T scalar = 3.0;
  SpatialQuantity V(this->V1_);
  V *= scalar;
  // Verify the result using Eigen operations.
  EXPECT_EQ(V.rotational(), Vector3<T>(this->r1_ * scalar));
  EXPECT_EQ(V.translational(), Vector3<T>(this->t1_ * scalar));
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

// Test the unary minus operator on a spatial vector.
TYPED_TEST(SpatialQuantityTest, UnaryMinusOperator) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;

  // Two non-zero rotational and translational components.
  const Vector3<T> w(1.0, 2.0, 3.0);
  const Vector3<T> v(4.0, 5.0, 6.0);

  // A spatial vector V from w and v.
  const SpatialQuantity V(w, v);

  // Negate V.
  const SpatialQuantity Vminus = -V;

  // Verify the result using Eigen operations.
  EXPECT_EQ(V.rotational(), -Vminus.rotational());
  EXPECT_EQ(V.translational(), -Vminus.translational());
}

TYPED_TEST(SpatialQuantityTest, AdditionAssignmentOperator) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  SpatialQuantity V(this->V1_);
  V += this->V2_;
  // Verify the result using Eigen operations.
  EXPECT_EQ(V.rotational(), Vector3<T>(this->r1_ + this->r2_));
  EXPECT_EQ(V.translational(), Vector3<T>(this->t1_ + this->t2_));
}

TYPED_TEST(SpatialQuantityTest, AdditionOperator) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  const SpatialQuantity V = this->V1_ + this->V2_;
  // Verify the result using Eigen operations.
  EXPECT_EQ(V.rotational(), Vector3<T>(this->r1_ + this->r2_));
  EXPECT_EQ(V.translational(), Vector3<T>(this->t1_ + this->t2_));
}

TYPED_TEST(SpatialQuantityTest, SubtractionAssignmentOperator) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  SpatialQuantity V(this->V1_);
  V -= this->V2_;
  // Verify the result using Eigen operations.
  EXPECT_EQ(V.rotational(), Vector3<T>(this->r1_ - this->r2_));
  EXPECT_EQ(V.translational(), Vector3<T>(this->t1_ - this->t2_));
}

TYPED_TEST(SpatialQuantityTest, SubtractionOperator) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  const SpatialQuantity V = this->V1_ - this->V2_;
  // Verify the result using Eigen operations.
  EXPECT_EQ(V.rotational(), Vector3<T>(this->r1_ - this->r2_));
  EXPECT_EQ(V.translational(), Vector3<T>(this->t1_ - this->t2_));
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
  const drake::math::RotationMatrix<T> R_EF(
       AngleAxis<T>(M_PI / 6, Vector3<T>::UnitX()) *
       AngleAxis<T>(M_PI / 6, Vector3<T>::UnitY()) *
       AngleAxis<T>(M_PI / 6, Vector3<T>::UnitZ()));

  SpatialQuantity V_AB_E = R_EF * V_AB_F;

  // Verify the result using Eigen operations.
  EXPECT_EQ(V_AB_E.rotational(), R_EF * V_AB_F.rotational());
  EXPECT_EQ(V_AB_E.translational(), R_EF * V_AB_F.translational());
}

// Create a list of scalar types for the unit tests that follow below.
typedef ::testing::Types<double, AutoDiffXd, Expression> ScalarTypes;

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
TYPED_TEST_SUITE(SpatialVelocityTest, ScalarTypes);

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

// Tests operator-().
TYPED_TEST(SpatialVelocityTest, SubtractionOperation) {
  typedef typename TestFixture::ScalarType T;

  // Arbitrary spatial velocity of a frame A shifted to a point Q.
  const Vector3<T> w_MAq_E(0.5, 2.5, -0.2);
  const Vector3<T> v_MAq_E(-0.8, 2.5, 1.3);
  const SpatialVelocity<T> V_MAq_E(w_MAq_E, v_MAq_E);

  // Arbitrary spatial velocity of a frame B shifted to a point Q.
  const Vector3<T> w_MBq_E(-0.2, 0.5, 2.5);
  const Vector3<T> v_MBq_E(1.0, 2.0, 3.0);
  const SpatialVelocity<T> V_MBq_E(w_MBq_E, v_MBq_E);

  SpatialVelocity<T> V_ABq_E = V_MBq_E - V_MAq_E;

  EXPECT_EQ(V_ABq_E.rotational(), w_MBq_E - w_MAq_E);
  EXPECT_EQ(V_ABq_E.translational(), v_MBq_E - v_MAq_E);
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

// Tests that we can take the dot product of a SpatialVelocity with a
// SpatialMomentum.
TYPED_TEST(SpatialVelocityTest, DotProductWithSpatialMomentum) {
  typedef typename TestFixture::ScalarType T;

  SpatialVelocity<T> V(Vector3<T>(1.0, 2.0, 3.0), /*rotational component*/
                       Vector3<T>(-2.0, -5.0, 8.0) /*translational component*/);
  SpatialMomentum<T> F(Vector3<T>(4.0, 1.0, -3.0), /*rotational component*/
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

// Unit tests specific to elements in F⁶.
template <class SpatialForceQuantityUnderTest>
class ElementsInF6Test : public ::testing::Test {
 public:
  // Useful typedefs when writing unit tests to access types.
  typedef SpatialForceQuantityUnderTest SpatialQuantityType;
  typedef typename spatial_vector_traits<
      SpatialQuantityType>::ScalarType ScalarType;
 protected:
  // Consider a force (or impulse) applied at the origin of Frame A, expressed
  // in a Frame E.
  Vector3<ScalarType> f_Ao_E_{1, 2, 0};

  // Consider a torque applied about the origin of Frame A, expressed in a
  // Frame E.
  Vector3<ScalarType> tau_Ao_E_{0, 0, 3};

  // Consider a spatial force (or spatial impulse) about the origin of Frame A,
  // expressed in a frame A.
  SpatialQuantityType F_Ao_E_{tau_Ao_E_, f_Ao_E_};
};

// Create a list of force types in F⁶ to be tested.
typedef ::testing::Types<
    SpatialForce<double>,
    SpatialMomentum<double>,
    SpatialForce<AutoDiffXd>,
    SpatialMomentum<AutoDiffXd>,
    SpatialForce<Expression>,
    SpatialMomentum<Expression>> ElementsInF6Types;
TYPED_TEST_SUITE(ElementsInF6Test, ElementsInF6Types);

// Tests the shifting of a spatial force between two moving frames rigidly
// attached to each other.
TYPED_TEST(ElementsInF6Test, ShiftOperation) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  const SpatialQuantity& F_Ao_E = this->F_Ao_E_;
  const Vector3<T>& f_Ao_E = this->f_Ao_E_;

  // Consider a vector from the origin of a frame A to the origin of a frame B,
  // expressed in a third frame E.
  Vector3<T> p_AB_E(2., -2., 1.);

  // Consider now shifting the spatial vector of F⁶ as applied about the origin
  // of frame A measured in frame E to the spatial vector of F⁶ as applied about
  // frame B also measured in frame E knowing that frames A and B are rigidly
  // attached to each other.
  SpatialQuantity F_Bo_E = F_Ao_E.Shift(p_AB_E);

  // Verify the result.
  SpatialQuantity expected_F_Bo_E(Vector3<T>(2.0, -1.0, -3.0), f_Ao_E);
  EXPECT_TRUE(F_Bo_E.IsApprox(expected_F_Bo_E));

  // We perform the shift operation on a Matrix storing a spatial force in each
  // column. First we'll use the not-in-place Shift() method.
  constexpr int num_forces = 3;
  const Matrix6X<T> Fmatrix_Ao_E =
      F_Ao_E.get_coeffs().rowwise().replicate(num_forces);
  Eigen::Matrix<T, 6, num_forces> Fmatrix_Bo_E;
  // TODO(amcastro-tri): Implement this version of Shift() for SpatialMomentum.
  SpatialForce<T>::Shift(Fmatrix_Ao_E, p_AB_E, &Fmatrix_Bo_E);
  for (int j = 0; j < num_forces; ++j) {
    SpatialQuantity Fj_Bo_E(Fmatrix_Bo_E.col(j));
    EXPECT_TRUE(Fj_Bo_E.IsApprox(expected_F_Bo_E));
  }

  // Now shift it back using the ShiftInPlace() method. We're expecting
  // near-perfect results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  SpatialForce<T>::ShiftInPlace(&Fmatrix_Bo_E, -p_AB_E);
  EXPECT_TRUE(CompareMatrices(Fmatrix_Bo_E, Fmatrix_Ao_E, kTolerance));
}

// Tests operator+().
TYPED_TEST(ElementsInF6Test, AdditionOperation) {
  typedef typename TestFixture::SpatialQuantityType SpatialQuantity;
  typedef typename TestFixture::ScalarType T;
  const SpatialQuantity& F_Ao_E = this->F_Ao_E_;
  const Vector3<T>& f_Ao_E = this->f_Ao_E_;
  const Vector3<T>& tau_Ao_E = this->tau_Ao_E_;

  SpatialQuantity V = F_Ao_E + F_Ao_E;

  EXPECT_EQ(V.rotational(), tau_Ao_E + tau_Ao_E);
  EXPECT_EQ(V.translational(), f_Ao_E + f_Ao_E);
}

// SpatialAcceleration specific unit tests.
template <typename T>
class SpatialAccelerationTest : public ::testing::Test {
 public:
  // Useful typedefs when witting unit tests to access types.
  typedef T ScalarType;
};
TYPED_TEST_SUITE(SpatialAccelerationTest, ScalarTypes);

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

template <class SymbolicSpatialQuantityType>
class SymbolicSpatialQuantityTest : public ::testing::Test {
 protected:
  // A translational component ∈ ℝ³.
  Vector3<Expression> v_{Variable("vx"), Variable("vy"), Variable("vz")};

  // A rotational component ∈ ℝ³.
  Vector3<Expression> w_{Variable("wx"), Variable("wy"), Variable("wz")};

  // A spatial quantity related to the above rotational and translational
  // components.
  SymbolicSpatialQuantityType V_{w_, v_};

  // A spatial quantity from a 6D vector of symbolic variables.
  SymbolicSpatialQuantityType Q_{MakeVectorContinuousVariable(6, "Q")};
};

// Create a list of SpatialVector with symbolic::Variable entries. These will
// get tested through fixture SymbolicSpatialQuantityTest.
typedef ::testing::Types<
    SpatialVelocity<Expression>,
    SpatialForce<Expression>,
    SpatialAcceleration<Expression>,
    SpatialMomentum<Expression>> SymbolicSpatialQuantityTypes;
TYPED_TEST_SUITE(SymbolicSpatialQuantityTest, SymbolicSpatialQuantityTypes);

TYPED_TEST(SymbolicSpatialQuantityTest, ShiftOperatorIntoStream) {
  std::stringstream V_stream;
  V_stream << this->V_;
  std::string V_expected_string = "[wx, wy, wz, vx, vy, vz]ᵀ";
  EXPECT_EQ(V_expected_string, V_stream.str());
  std::stringstream Q_stream;
  Q_stream << this->Q_;
  std::string Q_expected_string = "[Q(0), Q(1), Q(2), Q(3), Q(4), Q(5)]ᵀ";
  EXPECT_EQ(Q_expected_string, Q_stream.str());
}

// Tests the dot product between spatial momentum and spatial velocity
// quantities for a variety of scalar types.
template <typename T>
class MomentumDotVelocityTest : public ::testing::Test {
 public:
  // Useful typedefs when writing unit tests to access types.
  typedef T ScalarType;
 protected:
  SpatialMomentum<T> L_WBp_{Vector3<T>{1, 2, 3}, Vector3<T>{4, 5, 6}};
  SpatialVelocity<T> V_WBp_{Vector3<T>{7, 8, 9}, Vector3<T>{-1, -2, -3}};
  Vector3<T> p_PQ_{7, -3, 5};
};
TYPED_TEST_SUITE(MomentumDotVelocityTest, ScalarTypes);

// Verifies the result of the dot product of a spatial momentum L_WBp (of a body
// B in a frame W about a point P) with the spatial velocity V_WBp of frame Bp
// (body frame B shifted to point P) in frame W, is independent of point P.
TYPED_TEST(MomentumDotVelocityTest, InvariantUnderShiftOperation) {
  typedef typename TestFixture::ScalarType T;
  const SpatialMomentum<T>& L_WBp = this->L_WBp_;
  const SpatialVelocity<T>& V_WBp = this->V_WBp_;
  const Vector3<T>& p_PQ = this->p_PQ_;
  const T LdotV_P = L_WBp.dot(V_WBp);
  // Perform L_WBq.dot(V_WBq):
  const T HdotV_Q = L_WBp.Shift(p_PQ).dot(V_WBp.Shift(p_PQ));
  EXPECT_EQ(LdotV_P, HdotV_Q);
}

}  // namespace
}  // namespace math
}  // namespace multibody
}  // namespace drake
