#include "drake/common/symbolic_expression.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace symbolic {
namespace {

class SymbolicExpressionJacobianTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
};

TEST_F(SymbolicExpressionJacobianTest, Test1) {
  // Jacobian(2*x + 3*y + 4*z, [x, y, z])
  //  = [2, 3, 4]
  VectorX<Expression> f(1);
  f << 2 * x_ + 3 * y_ + 4 * z_;

  MatrixX<Expression> expected(1, 3);
  expected << 2, 3, 4;

  // std::vector of variables
  EXPECT_EQ(Jacobian(f, {x_, y_, z_}), expected);
  // Eigen::Vector of variables
  EXPECT_EQ(Jacobian(f, Vector3<Variable>{x_, y_, z_}), expected);
}

TEST_F(SymbolicExpressionJacobianTest, Test2) {
  // Jacobian([x * y * z, y^2, x + z], {x, y, z})
  //  = |(y * z)   (x * z)   (x * y)|
  //    |      0   (2 * y)         0|
  //    |      1         0         1|
  VectorX<Expression> f(3);
  f << x_ * y_ * z_, pow(y_, 2), x_ + z_;

  MatrixX<Expression> expected(3, 3);
  // clang-format off
  expected << y_ * z_, x_ * z_, x_ * y_,
                    0,  2 * y_,       0,
                    1,       0,       1;
  // clang-format on

  // std::vector of variables
  EXPECT_EQ(Jacobian(f, {x_, y_, z_}), expected);
  // Eigen::Vector of variables
  EXPECT_EQ(Jacobian(f, Vector3<Variable>{x_, y_, z_}), expected);
}

TEST_F(SymbolicExpressionJacobianTest, Test3) {
  // Jacobian([x^2*y, x*sin(y)], {x})
  // =  | 2*x*y  |
  //    | sin(y) |
  VectorX<Expression> f(2);
  f << pow(x_, 2) * y_, x_ * sin(y_);

  MatrixX<Expression> expected(2, 1);
  // clang-format off
  expected << 2 * x_ * y_,
                  sin(y_);
  // clang-format on

  // std::vector of variables
  EXPECT_EQ(Jacobian(f, {x_}), expected);
  // Eigen::Vector of variables
  EXPECT_EQ(Jacobian(f, Vector1<Variable>{x_}), expected);
}

TEST_F(SymbolicExpressionJacobianTest, Test4) {
  // Jacobian([x * cos(y), x * sin(y), x^2], {x, y})
  //  = |cos(y)   -x * sin(y)|
  //    |sin(y)    x * cos(y)|
  //    | 2 * x             0|
  VectorX<Expression> f(3);
  f << x_ * cos(y_), x_ * sin(y_), pow(x_, 2);

  MatrixX<Expression> expected(3, 2);
  // clang-format off
  expected << cos(y_), -x_ * sin(y_),
              sin(y_),  x_ * cos(y_),
               2 * x_,             0;
  // clang-format on

  // std::vector of variables
  EXPECT_EQ(Jacobian(f, {x_, y_}), expected);
  // Eigen::Vector of variables
  EXPECT_EQ(Jacobian(f, Vector2<Variable>{x_, y_}), expected);
}

TEST_F(SymbolicExpressionJacobianTest, Test5) {
  // Jacobian([x * y + sin(x)], {x, z})
  //  = |cos(y)   -x * sin(y)|
  //    |sin(y)    x * cos(y)|
  //    | 2 * x             0|
  VectorX<Expression> f(1);
  f << x_ * y_ + sin(x_);

  MatrixX<Expression> expected(1, 2);
  // clang-format off
  expected << y_ + cos(x_),
                         0;
  // clang-format on

  // std::vector of variables
  EXPECT_EQ(Jacobian(f, {x_, z_}), expected);
  // Eigen::Vector of variables
  EXPECT_EQ(Jacobian(f, Vector2<Variable>{x_, z_}), expected);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
