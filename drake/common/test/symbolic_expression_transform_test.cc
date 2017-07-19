#include "drake/common/symbolic_expression.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace symbolic {
namespace {

class SymbolicExpressionTransformationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    R_ = AngleAxis<double>(0.25 * M_PI, Vector3<double>::UnitX()) *
         AngleAxis<double>(0.5 * M_PI, Vector3<double>::UnitY()) *
         AngleAxis<double>(0.33 * M_PI, Vector3<double>::UnitZ());

    affine_.rotate(R_);
    affine_.translation().x() = 1.0;
    affine_.translation().y() = 2.0;
    affine_.translation().z() = 3.0;

    affine_compact_.rotate(R_ * R_);
    affine_compact_.translation().x() = -2.0;
    affine_compact_.translation().y() = 1.0;
    affine_compact_.translation().z() = 3.0;

    isometry_.rotate(-R_);
    isometry_.translation().x() = 3.0;
    isometry_.translation().y() = -1.0;
    isometry_.translation().z() = 2.0;

    projective_.rotate(-R_ * R_);
    projective_.translation().x() = 2.0;
    projective_.translation().y() = 3.0;
    projective_.translation().z() = -1.0;
  }

  Eigen::Matrix3d R_;  // Rotation Matrix.

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

TEST_F(SymbolicExpressionTransformationTest, IsometryIsometry) {
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
}  // namespace
}  // namespace symbolic
}  // namespace drake
