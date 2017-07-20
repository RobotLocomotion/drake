#include "drake/common/symbolic_expression.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_formula.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace symbolic {
namespace {

using math::XRotation;
using math::YRotation;
using math::ZRotation;

class SymbolicExpressionTransformationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    R1_ = XRotation(.25 * M_PI) * YRotation(.5 * M_PI) * ZRotation(.33 * M_PI);
    R2_ = XRotation(M_PI / 2) * YRotation(-M_PI / 2) * ZRotation(M_PI / 2);

    affine_.setIdentity();
    affine_.rotate(R1_);
    affine_.translation() << 1.0, 2.0, 3.0;

    affine_compact_.setIdentity();
    affine_compact_.rotate(R1_ * R2_);
    affine_compact_.translation() << -2.0, 1.0, 3.0;

    isometry_.setIdentity();
    isometry_.rotate(-R2_);
    isometry_.translation() << 3.0, -1.0, 2.0;

    projective_.setIdentity();
    projective_.rotate(-R2_ * R1_);
    projective_.translation() << 2.0, 3.0, -1.0;
  }

  // Rotation Matrices.
  Eigen::Matrix3d R1_;
  Eigen::Matrix3d R2_;

  // Transformations.
  Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> affine_;
  Eigen::Transform<double, 3, Eigen::AffineCompact, Eigen::DontAlign>
      affine_compact_;
  Eigen::Transform<double, 3, Eigen::Isometry, Eigen::DontAlign> isometry_;
  Eigen::Transform<double, 3, Eigen::Projective, Eigen::DontAlign> projective_;
};

// Checks if `lhs.cast<Expresion>() * rhs` and `(lhs * rhs).cast<Expresion>()`
// produce the same output. Also checks the symmetric case. See the following
// commutative diagram.
//                  * rhs
//   lhs --------------------------> lhs * rhs
//    ||                                ||
//    || cast<Expression>()             || cast<Expression>()
//    \/                                ||
//   lhs.cast<Expression>()             ||
//    ||                                ||
//    || * rhs                          ||
//    \/                          ?     \/
//   lhs.cast<Expresion>() * rhs  ==  (lhs * rhs).cast<Expresion>()
//
template <typename LhsTransform, typename RhsTransform>
::testing::AssertionResult CheckMultiplication(const LhsTransform& lhs,
                                               const RhsTransform& rhs) {
  const auto expected = (lhs * rhs).template cast<Expression>();
  const auto lhs_cast = lhs.template cast<Expression>() * rhs;
  const auto rhs_cast = lhs * rhs.template cast<Expression>();

  // Types should be identical.
  ::testing::StaticAssertTypeEq<decltype(expected), decltype(lhs_cast)>();
  ::testing::StaticAssertTypeEq<decltype(expected), decltype(rhs_cast)>();

  // Their matrices should be identical.
  if (expected.matrix() != lhs_cast.matrix()) {
    return ::testing::AssertionFailure()
           << "lhs.cast<Expression>() * rhs != (lhs * rhs).cast<Expression>()";
  }
  if (expected.matrix() != rhs_cast.matrix()) {
    return ::testing::AssertionFailure()
           << "lhs * rhs.cast<Expression>() != (lhs * rhs).cast<Expression>()";
  }
  return ::testing::AssertionSuccess();
}

TEST_F(SymbolicExpressionTransformationTest, CheckMultiplication) {
  EXPECT_TRUE(CheckMultiplication(affine_, affine_));
  EXPECT_TRUE(CheckMultiplication(affine_, affine_compact_));
  EXPECT_TRUE(CheckMultiplication(affine_, isometry_));
  EXPECT_TRUE(CheckMultiplication(affine_, projective_));

  EXPECT_TRUE(CheckMultiplication(affine_compact_, affine_));
  EXPECT_TRUE(CheckMultiplication(affine_compact_, affine_compact_));
  EXPECT_TRUE(CheckMultiplication(affine_compact_, isometry_));
  EXPECT_TRUE(CheckMultiplication(affine_compact_, projective_));

  EXPECT_TRUE(CheckMultiplication(isometry_, affine_));
  EXPECT_TRUE(CheckMultiplication(isometry_, affine_compact_));
  EXPECT_TRUE(CheckMultiplication(isometry_, isometry_));
  EXPECT_TRUE(CheckMultiplication(isometry_, projective_));

  EXPECT_TRUE(CheckMultiplication(projective_, affine_));
  EXPECT_TRUE(CheckMultiplication(projective_, affine_compact_));
  EXPECT_TRUE(CheckMultiplication(projective_, isometry_));
  EXPECT_TRUE(CheckMultiplication(projective_, projective_));
}

TEST_F(SymbolicExpressionTransformationTest, ExpectedAnswers) {
  Isometry3<Expression> isometry_expr;
  const Variable x{"x"};
  const Variable y{"y"};
  const Variable z{"z"};
  isometry_expr.setIdentity();
  isometry_expr.translation().x() = x;
  isometry_expr.translation().y() = y;
  isometry_expr.translation().z() = z;

  Isometry3<double> isometry_double;
  isometry_double.setIdentity();
  isometry_double.translation().x() = 1;
  isometry_double.translation().y() = 2;
  isometry_double.translation().z() = 3;
  Eigen::Matrix3d R;
  // clang-format off
  R << 0, 0, -1,
       0, 1, 0,
       1, 0, 1;
  // clang-format on
  isometry_double.rotate(R);

  // Expected result from Transform<Expression> * Transform<double>.
  Eigen::Matrix<Expression, 4, 4> expected_expr_double;
  // clang-format off
  expected_expr_double << 0, 0, -1, (1 + x),
                          0, 1,  0, (2 + y),
                          1, 0,  1, (3 + z),
                          0, 0,  0, 1;
  // clang-format on
  EXPECT_EQ((isometry_expr * isometry_double).matrix(), expected_expr_double);

  // Expected result from Transform<double> * Transform<Expression>.
  Eigen::Matrix<Expression, 4, 4> expected_double_expr;
  // clang-format off
  expected_double_expr << 0, 0, -1, (1 - z),
                          0, 1,  0, (2 + y),
                          1, 0,  1, (3 + x + z),
                          0, 0,  0, 1;
  // clang-format on
  EXPECT_EQ((isometry_double * isometry_expr).matrix(), expected_double_expr);
}
}  // namespace
}  // namespace symbolic
}  // namespace drake
