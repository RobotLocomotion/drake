#include "drake/geometry/optimization/affine_ball.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/optimization/affine_subspace.h"
#include "drake/geometry/optimization/test_utilities.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using internal::CheckAddPointInSetConstraints;

GTEST_TEST(AffineBallTest, DefaultCtor) {
  const AffineBall dut;
  EXPECT_EQ(dut.B().rows(), 0);
  EXPECT_EQ(dut.B().cols(), 0);
  EXPECT_EQ(dut.center().size(), 0);
  EXPECT_EQ(dut.Volume(), 0.0);
  EXPECT_NO_THROW(dut.Clone());
  EXPECT_EQ(dut.ambient_dimension(), 0);
  EXPECT_TRUE(dut.IntersectsWith(dut));
  EXPECT_TRUE(dut.IsBounded());
  EXPECT_FALSE(dut.IsEmpty());
  EXPECT_TRUE(dut.PointInSet(Eigen::VectorXd::Zero(0)));
  ASSERT_TRUE(dut.MaybeGetPoint().has_value());
  EXPECT_TRUE(dut.PointInSet(dut.MaybeGetPoint().value()));
  ASSERT_TRUE(dut.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(dut.PointInSet(dut.MaybeGetFeasiblePoint().value()));
}

GTEST_TEST(AffineBallTest, UnitSphereTest) {
  // Test constructor.
  const Eigen::Matrix3d B = Eigen::Matrix3d::Identity();
  const Vector3d center = Vector3d::Zero();
  AffineBall ab(B, center);
  EXPECT_EQ(ab.ambient_dimension(), 3);
  EXPECT_TRUE(CompareMatrices(B, ab.B()));
  EXPECT_TRUE(CompareMatrices(center, ab.center()));

  // Test MaybeGetPoint.
  EXPECT_FALSE(ab.MaybeGetPoint().has_value());

  // Test IsEmpty (which is trivially false for Hyperellipsoid).
  EXPECT_FALSE(ab.IsEmpty());

  // Test MaybeGetFeasiblePoint.
  ASSERT_TRUE(ab.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(ab.PointInSet(ab.MaybeGetFeasiblePoint().value()));

  // Test PointInSet.
  const Vector3d in1_W{.99, 0, 0}, in2_W{.5, .5, .5}, out1_W{1.01, 0, 0},
      out2_W{1.0, 1.0, 1.0};

  EXPECT_TRUE(ab.PointInSet(in1_W));
  EXPECT_TRUE(ab.PointInSet(in2_W));
  EXPECT_FALSE(ab.PointInSet(out1_W));
  EXPECT_FALSE(ab.PointInSet(out2_W));

  EXPECT_TRUE(CheckAddPointInSetConstraints(ab, in1_W));
  EXPECT_TRUE(CheckAddPointInSetConstraints(ab, in2_W));
  EXPECT_FALSE(CheckAddPointInSetConstraints(ab, out1_W));
  EXPECT_FALSE(CheckAddPointInSetConstraints(ab, out2_W));
}

GTEST_TEST(AffineBallTest, Move) {
  const Eigen::Matrix3d B = Eigen::Matrix3d::Identity();
  const Vector3d center = Vector3d::Zero();
  AffineBall orig(B, center);

  // A move-constructed AffineBall takes over the original data.
  AffineBall dut(std::move(orig));
  EXPECT_EQ(dut.ambient_dimension(), 3);
  EXPECT_TRUE(CompareMatrices(dut.B(), B));
  EXPECT_TRUE(CompareMatrices(dut.center(), center));

  // The old AffineBall is in a valid but unspecified state.
  EXPECT_EQ(orig.B().cols(), orig.ambient_dimension());
  EXPECT_EQ(orig.center().size(), orig.ambient_dimension());
  EXPECT_NO_THROW(orig.Clone());
}

GTEST_TEST(AffineBallTest, UnitBall6DTest) {
  AffineBall ab = AffineBall::MakeUnitBall(6);
  EXPECT_EQ(ab.ambient_dimension(), 6);

  const double kScale = sqrt(1.0 / 6.0);
  Vector6d in1_W{Vector6d::Constant(-.99 * kScale)},
      in2_W{Vector6d::Constant(.99 * kScale)},
      out1_W{Vector6d::Constant(-1.01 * kScale)},
      out2_W{Vector6d::Constant(1.01 * kScale)};

  EXPECT_TRUE(ab.PointInSet(in1_W));
  EXPECT_TRUE(ab.PointInSet(in2_W));
  EXPECT_FALSE(ab.PointInSet(out1_W));
  EXPECT_FALSE(ab.PointInSet(out2_W));

  ASSERT_TRUE(ab.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(ab.PointInSet(ab.MaybeGetFeasiblePoint().value()));

  EXPECT_EQ(ab.Volume(), std::pow(M_PI, 3) / 6);
}

GTEST_TEST(AffineBallTest, CloneTest) {
  AffineBall ab(Eigen::Matrix<double, 6, 6>::Identity(), Vector6d::Zero());
  std::unique_ptr<ConvexSet> clone = ab.Clone();
  EXPECT_EQ(clone->ambient_dimension(), ab.ambient_dimension());
  AffineBall* pointer = dynamic_cast<AffineBall*>(clone.get());
  ASSERT_NE(pointer, nullptr);
  EXPECT_TRUE(CompareMatrices(ab.B(), pointer->B()));
  EXPECT_TRUE(CompareMatrices(ab.center(), pointer->center()));
}

GTEST_TEST(AffineBallTest, MakeUnitBallTest) {
  AffineBall ab = AffineBall::MakeUnitBall(4);
  EXPECT_TRUE(CompareMatrices(ab.B(), MatrixXd::Identity(4, 4)));
  EXPECT_TRUE(CompareMatrices(ab.center(), VectorXd::Zero(4)));
  EXPECT_EQ(ab.Volume(), 0.5 * std::pow(M_PI, 2));
}

GTEST_TEST(AffineBallTest, MakeHypersphereTest) {
  const double kRadius = 3.0;
  Vector4d center;
  center << 1.3, 1.4, 7.2, 9.1;
  AffineBall ab = AffineBall::MakeHypersphere(kRadius, center);
  EXPECT_TRUE(CompareMatrices(ab.B(), MatrixXd::Identity(4, 4) * kRadius));
  EXPECT_TRUE(CompareMatrices(ab.center(), center));
  const Vector4d xvec = Eigen::MatrixXd::Identity(4, 1);
  EXPECT_TRUE(ab.PointInSet(center + kRadius * xvec, 1e-16));
  EXPECT_FALSE(ab.PointInSet(center + 1.1 * kRadius * xvec, 1e-16));
}

GTEST_TEST(AffineBallTest, MakeAxisAlignedTest) {
  const double a = 2.3, b = 4.5, c = 6.1;
  const Vector3d center{3.4, -2.3, 7.4};
  AffineBall ab = AffineBall::MakeAxisAligned(Vector3d{a, b, c}, center);
  EXPECT_EQ(ab.ambient_dimension(), 3);

  EXPECT_TRUE(ab.PointInSet(center + Vector3d(0, 0, 0)));
  EXPECT_TRUE(ab.PointInSet(center + Vector3d(a, 0, 0)));
  EXPECT_TRUE(ab.PointInSet(center + Vector3d(-a, 0, 0)));
  EXPECT_TRUE(ab.PointInSet(center + Vector3d(0, b, 0)));
  EXPECT_TRUE(ab.PointInSet(center + Vector3d(0, -b, 0)));
  EXPECT_TRUE(ab.PointInSet(center + Vector3d(0, 0, c)));
  EXPECT_TRUE(ab.PointInSet(center + Vector3d(0, 0, -c)));

  EXPECT_FALSE(ab.PointInSet(center + Vector3d(a, b, 0)));
  EXPECT_FALSE(ab.PointInSet(center + Vector3d(a, -b, 0)));
  EXPECT_FALSE(ab.PointInSet(center + Vector3d(-a, b, 0)));
  EXPECT_FALSE(ab.PointInSet(center + Vector3d(-a, -b, 0)));
  EXPECT_FALSE(ab.PointInSet(center + Vector3d(a, 0, c)));
  EXPECT_FALSE(ab.PointInSet(center + Vector3d(a, 0, -c)));
  EXPECT_FALSE(ab.PointInSet(center + Vector3d(-a, 0, c)));
  EXPECT_FALSE(ab.PointInSet(center + Vector3d(-a, 0, -c)));
  EXPECT_FALSE(ab.PointInSet(center + Vector3d(0, b, c)));
  EXPECT_FALSE(ab.PointInSet(center + Vector3d(0, -b, c)));
  EXPECT_FALSE(ab.PointInSet(center + Vector3d(0, b, -c)));
  EXPECT_FALSE(ab.PointInSet(center + Vector3d(0, -b, -c)));

  EXPECT_EQ(ab.Volume(), a * b * c * 4 * M_PI / 3);
}

GTEST_TEST(AffineBallTest, LowerDimensionalEllipsoids) {
  const double kAffineHullTol = 1e-12;

  // Test the MakeAxisAligned constructor.
  const Vector3d center{3.4, -2.3, 7.4};
  const double a = 2.3, b = 4.5, c = 6.1;
  AffineBall ab0 = AffineBall::MakeAxisAligned(Vector3d{0, 0, 0}, center);
  AffineBall ab1 = AffineBall::MakeAxisAligned(Vector3d{0, 0, c}, center);
  AffineBall ab2 = AffineBall::MakeAxisAligned(Vector3d{0, b, c}, center);
  AffineBall ab3 = AffineBall::MakeAxisAligned(Vector3d{a, b, c}, center);

  AffineSubspace as0(ab0, kAffineHullTol);
  AffineSubspace as1(ab1, kAffineHullTol);
  AffineSubspace as2(ab2, kAffineHullTol);
  AffineSubspace as3(ab3, kAffineHullTol);

  EXPECT_EQ(as0.AffineDimension(), 0);
  EXPECT_EQ(as1.AffineDimension(), 1);
  EXPECT_EQ(as2.AffineDimension(), 2);
  EXPECT_EQ(as3.AffineDimension(), 3);

  // Test the MakeHypersphere constructor.
  AffineBall ab4 = AffineBall::MakeHypersphere(0, center);
  AffineSubspace as4(ab4, kAffineHullTol);
  EXPECT_EQ(as4.AffineDimension(), 0);

  // Test the standard constructor.
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 3);
  AffineBall ab5(B, center);
  AffineSubspace as5(ab5, kAffineHullTol);
  EXPECT_EQ(as5.AffineDimension(), 0);
  B(0, 0) = 1;
  AffineBall ab6(B, center);
  AffineSubspace as6(ab6, kAffineHullTol);
  EXPECT_EQ(as6.AffineDimension(), 1);
  B(1, 1) = 1;
  AffineBall ab7(B, center);
  AffineSubspace as7(ab7, kAffineHullTol);
  EXPECT_EQ(as7.AffineDimension(), 2);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
