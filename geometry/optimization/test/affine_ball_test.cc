#include "drake/geometry/optimization/affine_ball.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/optimization/affine_subspace.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/test_utilities.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using internal::CheckAddPointInSetConstraints;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::Solve;
using std::sqrt;

GTEST_TEST(AffineBallTest, DefaultCtor) {
  const AffineBall dut;
  EXPECT_EQ(dut.B().rows(), 0);
  EXPECT_EQ(dut.B().cols(), 0);
  EXPECT_EQ(dut.center().size(), 0);
  EXPECT_TRUE(dut.has_exact_volume());
  EXPECT_THROW(dut.CalcVolume(), std::exception);
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
  Vector6d in1_W{Vector6d::Constant(-0.99 * kScale)},
      in2_W{Vector6d::Constant(0.99 * kScale)},
      out1_W{Vector6d::Constant(-1.01 * kScale)},
      out2_W{Vector6d::Constant(1.01 * kScale)};

  EXPECT_TRUE(ab.PointInSet(in1_W));
  EXPECT_TRUE(ab.PointInSet(in2_W));
  EXPECT_FALSE(ab.PointInSet(out1_W));
  EXPECT_FALSE(ab.PointInSet(out2_W));

  ASSERT_TRUE(ab.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(ab.PointInSet(ab.MaybeGetFeasiblePoint().value()));

  EXPECT_EQ(ab.CalcVolume(), std::pow(M_PI, 3) / 6);
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
  EXPECT_EQ(ab.CalcVolume(), 0.5 * std::pow(M_PI, 2));

  EXPECT_NO_THROW(AffineBall::MakeUnitBall(0));
  EXPECT_THROW(AffineBall::MakeUnitBall(-1), std::exception);
}

GTEST_TEST(AffineBallTest, MakeHypersphereTest) {
  const double kRadius = 3.0;
  Vector4d center;
  center << 1.3, 1.4, 7.2, 9.1;
  AffineBall ab = AffineBall::MakeHypersphere(kRadius, center);
  EXPECT_TRUE(CompareMatrices(ab.B(), MatrixXd::Identity(4, 4) * kRadius));
  EXPECT_TRUE(CompareMatrices(ab.center(), center));

  EXPECT_NO_THROW(AffineBall::MakeHypersphere(kRadius, VectorXd::Zero(2)));
  EXPECT_NO_THROW(AffineBall::MakeHypersphere(kRadius, VectorXd::Zero(0)));
  EXPECT_NO_THROW(AffineBall::MakeHypersphere(0, VectorXd::Zero(2)));
  EXPECT_NO_THROW(AffineBall::MakeHypersphere(0, VectorXd::Zero(0)));
  EXPECT_THROW(AffineBall::MakeHypersphere(-1, VectorXd::Zero(2)),
               std::exception);
  EXPECT_THROW(AffineBall::MakeHypersphere(-1, VectorXd::Zero(0)),
               std::exception);
}

GTEST_TEST(AffineBallTest, MakeAxisAlignedTest) {
  const double a = 2.3, b = 4.5, c = 6.1;
  const Vector3d center{3.4, -2.3, 7.4};
  AffineBall ab = AffineBall::MakeAxisAligned(Vector3d{a, b, c}, center);
  EXPECT_EQ(ab.ambient_dimension(), 3);

  Eigen::MatrixXd B_expected(3, 3);
  // clang-format off
  B_expected << a, 0, 0,
                0, b, 0,
                0, 0, c;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(ab.B(), B_expected));
  EXPECT_TRUE(CompareMatrices(ab.center(), center));

  EXPECT_NO_THROW(
      AffineBall::MakeAxisAligned(VectorXd::Zero(0), VectorXd::Zero(0)));
  EXPECT_THROW(
      AffineBall::MakeAxisAligned(VectorXd::Zero(0), VectorXd::Zero(1)),
      std::exception);
  EXPECT_THROW(
      AffineBall::MakeAxisAligned(VectorXd::Zero(1), VectorXd::Zero(0)),
      std::exception);
  EXPECT_THROW(AffineBall::MakeAxisAligned(Vector2d(-1, 1), VectorXd::Zero(2)),
               std::exception);
}

GTEST_TEST(AffineBallTest, NotAxisAligned) {
  // Tests an example of an ellipsoid whose principal axes are
  // not aligned with the coordinate axes.
  const double one_over_sqrt_two = 1 / sqrt(2);
  // Construct B1 to rotate by 45 degrees, and double the length
  // of one axis.
  Eigen::Matrix2d B1;
  // clang-format off
  B1 << 2 * one_over_sqrt_two, -1 * one_over_sqrt_two,
        2 * one_over_sqrt_two,     one_over_sqrt_two;
  // clang-format on
  Eigen::Vector2d center(1, 1);
  AffineBall ab1(B1, center);

  EXPECT_FALSE(ab1.MaybeGetPoint().has_value());
  ASSERT_TRUE(ab1.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(ab1.PointInSet(ab1.MaybeGetFeasiblePoint().value()));

  const double kTol = 1e-12;
  EXPECT_NEAR(ab1.CalcVolume(), M_PI * 2, kTol);
  // This point would be in the set if it wasn't rotated properly.
  EXPECT_FALSE(ab1.PointInSet(Vector2d{3, 1}, kTol));
  // With the rotation, this point should be in the set.
  EXPECT_TRUE(ab1.PointInSet(
      center + Vector2d{2 * one_over_sqrt_two, 2 * one_over_sqrt_two}, kTol));

  // Negate the second column of B1: the resulting affine_ball
  // should be the same as the first one.
  Eigen::Matrix2d B1_negated = B1;
  B1_negated.col(1) *= -1;
  AffineBall ab1_negated(B1_negated, center);
  // The determinant of B1_negated is is the negation of the determinant of B1.
  EXPECT_NEAR(ab1_negated.B().determinant(), -ab1.B().determinant(), kTol);
  // However the volume is the same because we use the absolute value of the
  // determinant.
  EXPECT_NEAR(ab1_negated.CalcVolume(), ab1.CalcVolume(), kTol);

  // Same as B1, but one of the axes is dropped. This makes it just a line
  // segment from (1-2/sqrt(2), 1-2/sqrt(2)) to (1+2/sqrt(2), 1+2/sqrt(2)).
  Eigen::Matrix2d B2;
  // clang-format off
  B2 << 2 * one_over_sqrt_two, 0,
        2 * one_over_sqrt_two, 0;
  // clang-format on
  AffineBall ab2(B2, center);

  EXPECT_FALSE(ab2.MaybeGetPoint().has_value());
  ASSERT_TRUE(ab2.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(ab2.PointInSet(ab2.MaybeGetFeasiblePoint().value(), kTol));

  EXPECT_EQ(ab2.CalcVolume(), 0);

  EXPECT_TRUE(ab2.PointInSet(
      center + Vector2d{2 * one_over_sqrt_two, 2 * one_over_sqrt_two}, kTol));
  EXPECT_TRUE(ab2.PointInSet(
      center - Vector2d{2 * one_over_sqrt_two, 2 * one_over_sqrt_two}, kTol));
  EXPECT_FALSE(ab2.PointInSet(center + Vector2d{kTol * 2, 0}, kTol));
  EXPECT_FALSE(ab2.PointInSet(center + Vector2d{-kTol * 2, 0}, kTol));
  EXPECT_FALSE(ab2.PointInSet(center + Vector2d{0, kTol * 2}, kTol));
  EXPECT_FALSE(ab2.PointInSet(center + Vector2d{0, -kTol * 2}, kTol));
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

  EXPECT_EQ(ab0.CalcVolume(), 0);
  EXPECT_EQ(ab1.CalcVolume(), 0);
  EXPECT_EQ(ab2.CalcVolume(), 0);
  EXPECT_EQ(ab3.CalcVolume(), a * b * c * 4 * M_PI / 3);

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

GTEST_TEST(AffineBallTest, FromHyperellipsoid) {
  Hyperellipsoid E1(MatrixXd::Identity(3, 3), Vector3d::Zero());
  EXPECT_TRUE(E1.IsBounded());
  EXPECT_NO_THROW(AffineBall{E1});
  Hyperellipsoid E2(MatrixXd::Identity(2, 3), Vector3d::Zero());
  EXPECT_FALSE(E2.IsBounded());
  EXPECT_THROW(AffineBall{E2}, std::exception);
}

GTEST_TEST(AffineBallTest, MinimumVolumeCircumscribedEllipsoidPreconditions) {
  Eigen::MatrixXd zero_rows = Eigen::MatrixXd::Zero(0, 2);
  Eigen::MatrixXd zero_cols = Eigen::MatrixXd::Zero(2, 0);
  Eigen::MatrixXd nan_entry(1, 1);
  nan_entry << std::numeric_limits<double>::quiet_NaN();
  Eigen::MatrixXd inf_entry(1, 1);
  inf_entry << std::numeric_limits<double>::infinity();

  DRAKE_EXPECT_THROWS_MESSAGE(
      AffineBall::MinimumVolumeCircumscribedEllipsoid(zero_rows),
      ".*points.*rows.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AffineBall::MinimumVolumeCircumscribedEllipsoid(zero_cols),
      ".*points.*cols.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AffineBall::MinimumVolumeCircumscribedEllipsoid(nan_entry),
      ".*points.*hasNaN.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AffineBall::MinimumVolumeCircumscribedEllipsoid(inf_entry),
      ".*points.*allFinite.*");
}

GTEST_TEST(AffineBallTest, MinimumVolumeCircumscribedEllipsoidFullDimensional) {
  // Axis-aligned example
  Eigen::Matrix3Xd p_FA(3, 6);
  // clang-format off
  p_FA << 1, 0, 0, -1,  0,  0,
          0, 2, 0,  0, -2,  0,
          0, 0, 3,  0,  0, -3;
  // clang-format on
  AffineBall E_F = AffineBall::MinimumVolumeCircumscribedEllipsoid(p_FA);

  double tol, eps;
  if (solvers::MosekSolver::is_available() &&
      solvers::MosekSolver::is_enabled()) {
    tol = 1e-8;
    eps = 1e-6;
  } else if (solvers::GurobiSolver::is_available() &&
             solvers::GurobiSolver::is_enabled()) {
    tol = 1e-4;
    eps = 1e-3;
  } else {
    tol = 1e-5;
    eps = 1e-4;
  }

  Vector3d center_expected = Vector3d::Zero();
  EXPECT_TRUE(CompareMatrices(E_F.center(), center_expected, tol));

  for (int i = 0; i < p_FA.cols(); ++i) {
    EXPECT_TRUE(E_F.PointInSet(p_FA.col(i), tol));
    EXPECT_FALSE(E_F.PointInSet(p_FA.col(i) * (1. + eps), tol));
  }

  // Non-axis-aligned example
  const Vector3d translation(0.5, 0.87, 0.1);
  const RigidTransformd X_GF{RollPitchYawd(0.1, 0.2, 3), translation};
  Eigen::Matrix3Xd p_GA = X_GF * p_FA;

  AffineBall E_G = AffineBall::MinimumVolumeCircumscribedEllipsoid(p_GA);

  for (int i = 0; i < p_FA.cols(); ++i) {
    EXPECT_TRUE(E_F.PointInSet(p_FA.col(i), tol));
    Eigen::Vector3d point_outside =
        p_FA.col(i) + (p_FA.col(i) - translation) * (1. + eps);
    EXPECT_FALSE(E_F.PointInSet(point_outside, tol));
  }
}

GTEST_TEST(AffineBallTest,
           MinimumVolumeCircumscribedEllipsoidLowerDimensional) {
  // Axis-aligned example
  Eigen::Matrix3Xd p_FA(3, 4);
  // clang-format off
  p_FA << 1, 0, -1,  0,
          0, 2,  0, -2,
          0, 0,  0,  0;
  // clang-format on
  AffineBall E_F = AffineBall::MinimumVolumeCircumscribedEllipsoid(p_FA);

  double tol, eps;
  if (solvers::MosekSolver::is_available() &&
      solvers::MosekSolver::is_enabled()) {
    tol = 1e-8;
    eps = 1e-6;
  } else {
    tol = 1e-5;
    eps = 1e-4;
  }

  Vector3d center_expected = Vector3d::Zero();
  EXPECT_TRUE(CompareMatrices(E_F.center(), center_expected, tol));

  for (int i = 0; i < p_FA.cols(); ++i) {
    EXPECT_TRUE(E_F.PointInSet(p_FA.col(i), tol));
    EXPECT_FALSE(E_F.PointInSet(p_FA.col(i) * (1. + eps), tol));
  }
  EXPECT_FALSE(E_F.PointInSet(Vector3d{0, 0, eps}));
  EXPECT_FALSE(E_F.PointInSet(Vector3d{0, 0, -eps}));

  // Non-axis-aligned example
  MatrixXd points(5, 2);
  // clang-format off
  points << 0.1, -0.1,
            0.2, -0.2,
            0.3, -0.3,
            0.4, -0.4,
            0.5, -0.5;
  // clang-format on
  VectorXd center(5);
  center << 0.123, 0.435, 2.3, -0.2, 0.75;
  points.colwise() += center;

  AffineBall E = AffineBall::MinimumVolumeCircumscribedEllipsoid(points);
  EXPECT_TRUE(CompareMatrices(E.center(), center, tol));

  for (int i = 0; i < points.cols(); ++i) {
    EXPECT_TRUE(E.PointInSet(points.col(i), tol));
    Eigen::VectorXd point_outside =
        points.col(i) + (points.col(i) - center) * (1. + eps);
    EXPECT_FALSE(E.PointInSet(point_outside, tol));
  }
}

GTEST_TEST(AffineBallTest, MakeAffineBallFromLineSegment) {
  const Vector3d x_1 = Vector3d{0.0, 0.0, 0.0};
  const Vector3d x_2 = Vector3d{0.0, 0.0, 4.0};
  const double segment_length = (x_1 - x_2).norm();
  // Throws if called on a single point.
  DRAKE_EXPECT_THROWS_MESSAGE(
      AffineBall::MakeAffineBallFromLineSegment(x_1, x_1), ".*same point.*");
  AffineBall a_0 = AffineBall::MakeAffineBallFromLineSegment(x_1, x_2, 0);
  EXPECT_TRUE(CompareMatrices(a_0.center(), (x_1 + x_2) / 2.0));
  // This affine ball has affine dimension one.
  EXPECT_TRUE(a_0.PointInSet(x_1));
  EXPECT_TRUE(a_0.PointInSet(x_2));
  // It does include center, but not a bit outside in y direction.
  const double varepsilon = 1e-2;
  EXPECT_TRUE(a_0.PointInSet(a_0.center()));
  EXPECT_FALSE(a_0.PointInSet(a_0.center() + Vector3d{0.0, varepsilon, 0.0}));
  EXPECT_NEAR(a_0.CalcVolume(), 0.0, 1e-6);
  // Now give it some epsilon.
  const double epsilon = 0.1;
  AffineBall a_1 = AffineBall::MakeAffineBallFromLineSegment(x_1, x_2, epsilon);
  EXPECT_EQ(AffineSubspace(a_1.B(), a_1.center()).AffineDimension(), 3);
  // Check that the affine transformation matrix divided by hyperellipsoid
  // axis length vectors is a rotation matrix.
  const auto scale_back_vector =
      Eigen::Vector3d{2 / (x_1 - x_2).norm(), 1 / epsilon, 1 / epsilon};
  const auto rotation_matrix = a_1.B() * scale_back_vector.asDiagonal();
  EXPECT_TRUE(CompareMatrices(rotation_matrix * rotation_matrix.transpose(),
                              Eigen::Matrix3d::Identity(), 1e-6));
  // It must contain both center and a bit off-center in x-y-z directions.
  EXPECT_TRUE(a_1.PointInSet(a_1.center()));
  EXPECT_TRUE(a_1.PointInSet(a_1.center() + Vector3d{varepsilon, 0, 0}, 1e-9));
  EXPECT_TRUE(a_1.PointInSet(a_1.center() + Vector3d{0, varepsilon, 0}, 1e-9));
  EXPECT_TRUE(a_1.PointInSet(a_1.center() + Vector3d{0, 0, varepsilon}, 1e-9));
  // let's check the volume of the affine ball, should be 4/3*pi*r_1*r_2*r_3,
  // where r_1 = 2.5 (half the distance between x_1 and x_2), r_2 and r_3 are
  // epsilon.
  EXPECT_NEAR(a_1.CalcVolume(),
              4.0 / 3 * M_PI * segment_length / 2.0 * std::pow(epsilon, 2),
              1e-6);
}

GTEST_TEST(AffineBallTest, PointInNonnegativeScalingConstraints) {
  // Unit circle in the x-y plane, translated along the z-axis two units.
  Eigen::Matrix<double, 3, 3> B;
  // clang-format off
  B << 1, 0, 0,
       0, 1, 0,
       0, 0, 0;
  // clang-format on
  Vector3d center(0.0, 0.0, 2.0);
  const AffineBall ab(B, center);

  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  auto t = prog.NewContinuousVariables(1, "t");

  std::vector<Binding<Constraint>> constraints =
      ab.AddPointInNonnegativeScalingConstraints(&prog, x, t[0]);

  Vector3d x_test_value = Vector3d::Zero();
  double t_test_value = 0;

  auto x_constraint = prog.AddLinearEqualityConstraint(MatrixXd::Identity(3, 3),
                                                       x_test_value, x);
  auto t_constraint = prog.AddLinearEqualityConstraint(MatrixXd::Identity(1, 1),
                                                       t_test_value, t);

  // Test values for x, t, and whether the constraint is satisfied.
  const std::vector<std::tuple<Vector3d, double, bool>> test_x_t{
      {Vector3d(1.0, 0.0, 2.0), 1.0, true},
      {Vector3d(0.0, -1.0, 2.0), 1.0, true},
      {Vector3d(0.0, 2.0, 4.0), 2.0, true},
      {Vector3d(0.0, 0.0, 6.0), 3.0, true},
      {Vector3d(0.0, 0.0, 6.0), 3.0, true},
      {Vector3d(0.0, 0.0, 0.0), 0.0, true},
      {Vector3d(0.0, 0.0, 0.0), 1.0, false},
      {Vector3d(1.0, 0.0, 0.0), 0.0, false},
      {Vector3d(1.0, 0.0, 2.0), 2.0, false},
      {Vector3d(0.0, 2.0, 2.0), 2.0, false},
      {Vector3d(0.0, 2.0, 4.0), 1.0, false}};

  for (const auto& [x_val, t_val, expect_success] : test_x_t) {
    x_constraint.evaluator()->UpdateCoefficients(MatrixXd::Identity(3, 3),
                                                 x_val);
    t_constraint.evaluator()->UpdateCoefficients(MatrixXd::Identity(1, 1),
                                                 Vector1d(t_val));
    auto result = Solve(prog);
    EXPECT_EQ(result.is_success(), expect_success);
  }

  Eigen::Matrix<double, 3, 2> A;
  // clang-format off
  A << 1, 0,
       0, 1,
       2, 0;
  // clang-format on
  Vector3d b = Vector3d::Zero();
  Vector2d c(1, -1);
  double d = 0;

  MathematicalProgram prog2;
  auto x2 = prog2.NewContinuousVariables(2, "x");
  auto t2 = prog2.NewContinuousVariables(2, "t");

  std::vector<Binding<Constraint>> constraints2 =
      ab.AddPointInNonnegativeScalingConstraints(&prog2, A, b, c, d, x2, t2);

  Vector2d x2_test_value = Vector2d::Zero();
  Vector2d t2_test_value = Vector2d::Zero();

  auto x2_constraint = prog2.AddLinearEqualityConstraint(
      MatrixXd::Identity(2, 2), x2_test_value, x2);
  auto t2_constraint = prog2.AddLinearEqualityConstraint(
      MatrixXd::Identity(2, 2), t2_test_value, t2);

  // Test values for x, t, and whether the constraint is satisfied.
  const std::vector<std::tuple<Vector2d, Vector2d, bool>> test_x2_t2{
      {Vector2d(1, 0), Vector2d(1, 0), true},
      {Vector2d(1, 0), Vector2d(0, -1), true},
      {Vector2d(1, 0), Vector2d(2, 1), true},
      {Vector2d(2, 0), Vector2d(1, -1), true},
      {Vector2d(1, 0), Vector2d(1, -1), false},
      {Vector2d(1, 0), Vector2d(0, 1), false},
      {Vector2d(2, 0), Vector2d(1, -2), false}};

  for (const auto& [x2_val, t2_val, expect_success] : test_x2_t2) {
    x2_constraint.evaluator()->UpdateCoefficients(MatrixXd::Identity(2, 2),
                                                  x2_val);
    t2_constraint.evaluator()->UpdateCoefficients(MatrixXd::Identity(2, 2),
                                                  t2_val);
    auto result = Solve(prog2);
    EXPECT_EQ(result.is_success(), expect_success);
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
