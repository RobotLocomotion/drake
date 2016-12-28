#include "drake/common/symbolic_variable.h"

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "gtest/gtest.h"

#include "drake/common/symbolic_expression.h"

namespace drake {
namespace symbolic {
namespace {

using std::move;
using std::unordered_map;
using std::unordered_set;
using std::vector;

static bool ExpEqual(const Expression& e1, const Expression& e2) {
  return e1.EqualTo(e2);
}

// Provides common variables that are used by the following tests.
class SymbolicVariableTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
  const Variable w_{"w"};

  Eigen::Matrix<double, 2, 2> double_mat_;
  Eigen::Matrix<symbolic::Variable, 2, 2> var_mat_;
  Eigen::Matrix<symbolic::Expression, 2, 2> exp_mat_;
  void SetUp() {
    // clang-format off
    double_mat_ << 1.0, 2.0,
                   3.0, 4.0;
    var_mat_ << x_, y_,
                z_, w_;
    exp_mat_ << (x_ + z_), (x_ + w_),
                (y_ + z_), (y_ + w_);
    // clang-format on
  }
};

TEST_F(SymbolicVariableTest, GetId) {
  const Variable x_prime{"x"};
  EXPECT_NE(x_.get_id(), x_prime.get_id());
}

TEST_F(SymbolicVariableTest, GetName) {
  const Variable x_prime{"x"};
  EXPECT_EQ(x_.get_name(), x_prime.get_name());
}

TEST_F(SymbolicVariableTest, MoveCopyPreserveId) {
  Variable x{"x"};
  const size_t x_id{x.get_id()};
  const size_t x_hash{x.get_hash()};
  const Variable x_copied{x};
  const Variable x_moved{move(x)};
  EXPECT_EQ(x_id, x_copied.get_id());
  EXPECT_EQ(x_hash, x_copied.get_hash());
  EXPECT_EQ(x_id, x_moved.get_id());
  EXPECT_EQ(x_hash, x_moved.get_hash());
}

TEST_F(SymbolicVariableTest, Lt) {
  EXPECT_FALSE(x_ < x_);
  EXPECT_TRUE(x_ < y_);
  EXPECT_TRUE(x_ < z_);
  EXPECT_TRUE(x_ < w_);

  EXPECT_FALSE(y_ < x_);
  EXPECT_FALSE(y_ < y_);
  EXPECT_TRUE(y_ < z_);
  EXPECT_TRUE(y_ < w_);

  EXPECT_FALSE(z_ < x_);
  EXPECT_FALSE(z_ < y_);
  EXPECT_FALSE(z_ < z_);
  EXPECT_TRUE(z_ < w_);

  EXPECT_FALSE(w_ < x_);
  EXPECT_FALSE(w_ < y_);
  EXPECT_FALSE(w_ < z_);
  EXPECT_FALSE(w_ < w_);
}

TEST_F(SymbolicVariableTest, Eq) {
  EXPECT_TRUE(x_ == x_);
  EXPECT_FALSE(x_ == y_);
  EXPECT_FALSE(x_ == z_);
  EXPECT_FALSE(x_ == w_);

  EXPECT_FALSE(y_ == x_);
  EXPECT_TRUE(y_ == y_);
  EXPECT_FALSE(y_ == z_);
  EXPECT_FALSE(y_ == w_);

  EXPECT_FALSE(z_ == x_);
  EXPECT_FALSE(z_ == y_);
  EXPECT_TRUE(z_ == z_);
  EXPECT_FALSE(z_ == w_);

  EXPECT_FALSE(w_ == x_);
  EXPECT_FALSE(w_ == y_);
  EXPECT_FALSE(w_ == z_);
  EXPECT_TRUE(w_ == w_);
}

TEST_F(SymbolicVariableTest, ToString) {
  EXPECT_EQ(x_.to_string(), "x");
  EXPECT_EQ(y_.to_string(), "y");
  EXPECT_EQ(z_.to_string(), "z");
  EXPECT_EQ(w_.to_string(), "w");
}

// This test checks whether symbolic::Variable is compatible with
// std::unordered_set.
TEST_F(SymbolicVariableTest, CompatibleWithUnorderedSet) {
  unordered_set<Variable, hash_value<Variable>> uset;
  uset.emplace(x_);
  uset.emplace(y_);
}

// This test checks whether symbolic::Variable is compatible with
// std::unordered_map.
TEST_F(SymbolicVariableTest, CompatibleWithUnorderedMap) {
  unordered_map<Variable, Variable, hash_value<Variable>> umap;
  umap.emplace(x_, y_);
}

// This test checks whether symbolic::Variable is compatible with
// std::vector.
TEST_F(SymbolicVariableTest, CompatibleWithVector) {
  vector<Variable> vec;
  vec.push_back(x_);
}

TEST_F(SymbolicVariableTest, OperatorOverloadingArithmetic) {
  // Variable op Variable.
  const Expression e1{x_ + y_};
  const Expression e2{x_ - y_};
  const Expression e3{x_ * y_};
  const Expression e4{x_ / y_};
  EXPECT_EQ(e1.to_string(), "(x + y)");
  EXPECT_EQ(e2.to_string(), "(x - y)");
  EXPECT_EQ(e3.to_string(), "(x * y)");
  EXPECT_EQ(e4.to_string(), "(x / y)");

  // Expression op Variable.
  const Expression e5{e1 + z_};  // (x + y) + z
  const Expression e6{e2 - z_};  // (x - y) - z
  const Expression e7{e3 * z_};  // (x * y) * z
  const Expression e8{e4 / z_};  // (x / y) / z
  EXPECT_EQ(e5.to_string(), "(x + y + z)");
  EXPECT_EQ(e6.to_string(), "(x - y - z)");
  EXPECT_EQ(e7.to_string(), "(x * y * z)");
  EXPECT_EQ(e8.to_string(), "((x / y) / z)");

  // Variable op Expression.
  const Expression e9{w_ + e1};   // w + x + y   -> x + z
  const Expression e10{w_ - e2};  // w - (x - y) -> w -x + y -> -x + y + w
  const Expression e11{w_ * e3};  // w * (x * y) -> x * y * w
  const Expression e12{w_ / e4};  // w / (x / y)
  EXPECT_EQ(e9.to_string(), "(x + y + w)");
  EXPECT_EQ(e10.to_string(), "( - x + y + w)");
  EXPECT_EQ(e11.to_string(), "(x * y * w)");
  EXPECT_EQ(e12.to_string(), "(w / (x / y))");

  // Variable op double.
  const Expression e13{x_ + 5.0};  // x + 5 -> 5 + x
  const Expression e14{x_ - 5.0};  // x - 5 -> -5 + x
  const Expression e15{x_ * 5.0};  // x * 5 -> 5 * x
  const Expression e16{x_ / 5.0};
  EXPECT_EQ(e13.to_string(), "(5 + x)");
  EXPECT_EQ(e14.to_string(), "(-5 + x)");
  EXPECT_EQ(e15.to_string(), "(5 * x)");
  EXPECT_EQ(e16.to_string(), "(x / 5)");

  // double op Variable.
  const Expression e17{5.0 + y_};
  const Expression e18{5.0 - y_};
  const Expression e19{5.0 * y_};
  const Expression e20{5.0 / y_};
  EXPECT_EQ(e17.to_string(), "(5 + y)");
  EXPECT_EQ(e18.to_string(), "(5 - y)");
  EXPECT_EQ(e19.to_string(), "(5 * y)");
  EXPECT_EQ(e20.to_string(), "(5 / y)");
}

TEST_F(SymbolicVariableTest, OperatorOverloadingArithmeticAssignment) {
  Expression e{x_ + y_};
  e += z_;  // x + y + z
  EXPECT_EQ(e.to_string(), "(x + y + z)");
  e -= x_;  // y + z
  EXPECT_EQ(e.to_string(), "(y + z)");
  e *= w_;  // w * (y + z)
  EXPECT_EQ(e.to_string(), "(w * (y + z))");
  e /= y_;  // (w * (y + z)) / y
  EXPECT_EQ(e.to_string(), "((w * (y + z)) / y)");
}

TEST_F(SymbolicVariableTest, OperatorOverloadingEigenTestSanityCheck) {
  // [1.0  2.0]
  // [3.0  4.0]
  EXPECT_EQ(double_mat_(0, 0), 1.0);
  EXPECT_EQ(double_mat_(0, 1), 2.0);
  EXPECT_EQ(double_mat_(1, 0), 3.0);
  EXPECT_EQ(double_mat_(1, 1), 4.0);

  // [x  y]
  // [z  w]
  EXPECT_EQ(var_mat_(0, 0), x_);
  EXPECT_EQ(var_mat_(0, 1), y_);
  EXPECT_EQ(var_mat_(1, 0), z_);
  EXPECT_EQ(var_mat_(1, 1), w_);

  // [x + z  x + w]
  // [y + z  y + w]
  EXPECT_PRED2(ExpEqual, exp_mat_(0, 0), x_ + z_);
  EXPECT_PRED2(ExpEqual, exp_mat_(0, 1), x_ + w_);
  EXPECT_PRED2(ExpEqual, exp_mat_(1, 0), y_ + z_);
  EXPECT_PRED2(ExpEqual, exp_mat_(1, 1), y_ + w_);
}

TEST_F(SymbolicVariableTest, OperatorOverloadingEigenVariableOpVariable) {
  const Eigen::Matrix<Expression, 2, 2> m1{var_mat_ + var_mat_};
  const Eigen::Matrix<Expression, 2, 2> m2{var_mat_ - var_mat_};
  const Eigen::Matrix<Expression, 2, 2> m3{var_mat_ * var_mat_};

  // [x  y] + [x  y] = [x + x    y + y]
  // [z  w]   [z  w]   [z + z    w + w]
  EXPECT_PRED2(ExpEqual, m1(0, 0), x_ + x_);
  EXPECT_PRED2(ExpEqual, m1(0, 1), y_ + y_);
  EXPECT_PRED2(ExpEqual, m1(1, 0), z_ + z_);
  EXPECT_PRED2(ExpEqual, m1(1, 1), w_ + w_);

  // [x  y] - [x  y] = [x - x    y - y] = [0 0]
  // [z  w]   [z  w]   [z - z    w - w]   [0 0]
  EXPECT_PRED2(ExpEqual, m2(0, 0), 0.0);
  EXPECT_PRED2(ExpEqual, m2(0, 1), 0.0);
  EXPECT_PRED2(ExpEqual, m2(1, 0), 0.0);
  EXPECT_PRED2(ExpEqual, m2(1, 1), 0.0);

  // [x  y] * [x  y] = [x * x + y * z    x * y + y * w]
  // [z  w]   [z  w]   [z * x + w * z    z * y + w * w]
  EXPECT_PRED2(ExpEqual, m3(0, 0), x_ * x_ + y_ * z_);
  EXPECT_PRED2(ExpEqual, m3(0, 1), x_ * y_ + y_ * w_);
  EXPECT_PRED2(ExpEqual, m3(1, 0), z_ * x_ + w_ * z_);
  EXPECT_PRED2(ExpEqual, m3(1, 1), z_ * y_ + w_ * w_);
}

TEST_F(SymbolicVariableTest, OperatorOverloadingEigenVariableOpExpression) {
  const Eigen::Matrix<Expression, 2, 2> m1{var_mat_ + exp_mat_};
  const Eigen::Matrix<Expression, 2, 2> m2{var_mat_ - exp_mat_};
  const Eigen::Matrix<Expression, 2, 2> m3{var_mat_ * exp_mat_};

  // [x  y] + [x + z  x + w] = [x + x + z    y + x + w]
  // [z  w]   [y + z  y + w]   [z + y + z    w + y + w]
  EXPECT_PRED2(ExpEqual, m1(0, 0), x_ + x_ + z_);
  EXPECT_PRED2(ExpEqual, m1(0, 1), y_ + x_ + w_);
  EXPECT_PRED2(ExpEqual, m1(1, 0), z_ + y_ + z_);
  EXPECT_PRED2(ExpEqual, m1(1, 1), w_ + y_ + w_);

  // [x  y] - [x + z  x + w] = [x - (x + z)    y - (x + w)]
  // [z  w]   [y + z  y + w]   [z - (y + z)    w - (y + w)]
  EXPECT_PRED2(ExpEqual, m2(0, 0), x_ - (x_ + z_));
  EXPECT_PRED2(ExpEqual, m2(0, 1), y_ - (x_ + w_));
  EXPECT_PRED2(ExpEqual, m2(1, 0), z_ - (y_ + z_));
  EXPECT_PRED2(ExpEqual, m2(1, 1), w_ - (y_ + w_));

  // [x  y] * [x + z  x + w]
  // [z  w]   [y + z  y + w]
  // = [x * (x + z) + y * (y + z)    x * (x + w) + y * (y + w)]
  //   [z * (x + z) + w * (y + z)    z * (x + w) + w * (y + w)]
  EXPECT_PRED2(ExpEqual, m3(0, 0), x_ * (x_ + z_) + y_ * (y_ + z_));
  EXPECT_PRED2(ExpEqual, m3(0, 1), x_ * (x_ + w_) + y_ * (y_ + w_));
  EXPECT_PRED2(ExpEqual, m3(1, 0), z_ * (x_ + z_) + w_ * (y_ + z_));
  EXPECT_PRED2(ExpEqual, m3(1, 1), z_ * (x_ + w_) + w_ * (y_ + w_));
}

TEST_F(SymbolicVariableTest, OperatorOverloadingEigenExpressionOpVariable) {
  const Eigen::Matrix<Expression, 2, 2> m1{exp_mat_ + var_mat_};
  const Eigen::Matrix<Expression, 2, 2> m2{exp_mat_ - var_mat_};
  const Eigen::Matrix<Expression, 2, 2> m3{exp_mat_ * var_mat_};

  // [x + z  x + w] + [x  y] = [x + z + x    x + w + y]
  // [y + z  y + w]   [z  w]   [y + z + z    y + w + w]
  EXPECT_PRED2(ExpEqual, m1(0, 0), x_ + z_ + x_);
  EXPECT_PRED2(ExpEqual, m1(0, 1), x_ + w_ + y_);
  EXPECT_PRED2(ExpEqual, m1(1, 0), y_ + z_ + z_);
  EXPECT_PRED2(ExpEqual, m1(1, 1), y_ + w_ + w_);

  // [x + z  x + w] - [x  y] = [x + z - x    x + w - y]
  // [y + z  y + w]   [z  w]   [y + z - z    y + w - w]
  EXPECT_PRED2(ExpEqual, m2(0, 0), x_ + z_ - x_);
  EXPECT_PRED2(ExpEqual, m2(0, 1), x_ + w_ - y_);
  EXPECT_PRED2(ExpEqual, m2(1, 0), y_ + z_ - z_);
  EXPECT_PRED2(ExpEqual, m2(1, 1), y_ + w_ - w_);

  // [x + z  x + w] * [x  y]
  // [y + z  y + w]   [z  w]
  //
  // = [(x + z) * x + (x + w) * z    (x + z) * y + (x + w) * w]
  //   [(y + z) * x + (y + w) * z    (y + z) * y + (y + w) * w]
  EXPECT_PRED2(ExpEqual, m3(0, 0), (x_ + z_) * x_ + (x_ + w_) * z_);
  EXPECT_PRED2(ExpEqual, m3(0, 1), (x_ + z_) * y_ + (x_ + w_) * w_);
  EXPECT_PRED2(ExpEqual, m3(1, 0), (y_ + z_) * x_ + (y_ + w_) * z_);
  EXPECT_PRED2(ExpEqual, m3(1, 1), (y_ + z_) * y_ + (y_ + w_) * w_);
}

TEST_F(SymbolicVariableTest, OperatorOverloadingEigenVariableOpDouble) {
  const Eigen::Matrix<Expression, 2, 2> m1{var_mat_ + double_mat_};
  const Eigen::Matrix<Expression, 2, 2> m2{var_mat_ - double_mat_};
  const Eigen::Matrix<Expression, 2, 2> m3{var_mat_ * double_mat_};

  // [x  y] + [1.0  2.0] = [x + 1.0    y + 2.0]
  // [z  w]   [3.0  4.0]   [z + 3.0    w + 4.0]
  EXPECT_PRED2(ExpEqual, m1(0, 0), x_ + 1.0);
  EXPECT_PRED2(ExpEqual, m1(0, 1), y_ + 2.0);
  EXPECT_PRED2(ExpEqual, m1(1, 0), z_ + 3.0);
  EXPECT_PRED2(ExpEqual, m1(1, 1), w_ + 4.0);

  // [x  y] - [1.0  2.0] = [x - 1.0    y - 2.0]
  // [z  w]   [3.0  4.0]   [z - 3.0    w - 4.0]
  EXPECT_PRED2(ExpEqual, m2(0, 0), x_ - 1.0);
  EXPECT_PRED2(ExpEqual, m2(0, 1), y_ - 2.0);
  EXPECT_PRED2(ExpEqual, m2(1, 0), z_ - 3.0);
  EXPECT_PRED2(ExpEqual, m2(1, 1), w_ - 4.0);

  // [x  y] * [1.0  2.0] = [x * 1.0 + y * 3.0    x * 2.0 + y * 4.0]
  // [z  w]   [3.0  4.0]   [z * 1.0 + w * 3.0    z * 2.0 + y * 4.0]
  EXPECT_PRED2(ExpEqual, m3(0, 0), x_ * 1.0 + y_ * 3.0);
  EXPECT_PRED2(ExpEqual, m3(0, 1), x_ * 2.0 + y_ * 4.0);
  EXPECT_PRED2(ExpEqual, m3(1, 0), z_ * 1.0 + w_ * 3.0);
  EXPECT_PRED2(ExpEqual, m3(1, 1), z_ * 2.0 + w_ * 4.0);
}

TEST_F(SymbolicVariableTest, OperatorOverloadingEigenDoubleOpVariable) {
  const Eigen::Matrix<Expression, 2, 2> m1{double_mat_ + var_mat_};
  const Eigen::Matrix<Expression, 2, 2> m2{double_mat_ - var_mat_};
  const Eigen::Matrix<Expression, 2, 2> m3{double_mat_ * var_mat_};

  // [1.0  2.0] + [x  y] = [1.0 + x    2.0 + y]
  // [3.0  4.0]   [z  w]   [3.0 + z    4.0 + w]
  EXPECT_PRED2(ExpEqual, m1(0, 0), 1.0 + x_);
  EXPECT_PRED2(ExpEqual, m1(0, 1), 2.0 + y_);
  EXPECT_PRED2(ExpEqual, m1(1, 0), 3.0 + z_);
  EXPECT_PRED2(ExpEqual, m1(1, 1), 4.0 + w_);

  // [1.0  2.0] - [x  y] = [1.0 - x    2.0 - y]
  // [3.0  4.0]   [z  w]   [3.0 - z    4.0 - w]
  EXPECT_PRED2(ExpEqual, m2(0, 0), 1.0 - x_);
  EXPECT_PRED2(ExpEqual, m2(0, 1), 2.0 - y_);
  EXPECT_PRED2(ExpEqual, m2(1, 0), 3.0 - z_);
  EXPECT_PRED2(ExpEqual, m2(1, 1), 4.0 - w_);

  // [1.0  2.0] * [x  y] = [1.0 * x + 2.0 * z    1.0 * y + 2.0 * w]
  // [3.0  4.0]   [z  w]   [3.0 * x + 4.0 * z    3.0 * y + 4.0 * w]
  EXPECT_PRED2(ExpEqual, m3(0, 0), 1.0 * x_ + 2.0 * z_);
  EXPECT_PRED2(ExpEqual, m3(0, 1), 1.0 * y_ + 2.0 * w_);
  EXPECT_PRED2(ExpEqual, m3(1, 0), 3.0 * x_ + 4.0 * z_);
  EXPECT_PRED2(ExpEqual, m3(1, 1), 3.0 * y_ + 4.0 * w_);
}

TEST_F(SymbolicVariableTest, OperatorOverloadingEigenDivideByVariable) {
  const Eigen::Matrix<Expression, 2, 2> m1{double_mat_ / x_};
  const Eigen::Matrix<Expression, 2, 2> m2{var_mat_ / x_};
  const Eigen::Matrix<Expression, 2, 2> m3{exp_mat_ / x_};

  // [1.0  2.0] / x = [1.0 / x    2.0 / x]
  // [3.0  4.0]       [3.0 / x    4.0 / x]
  EXPECT_PRED2(ExpEqual, m1(0, 0), 1.0 / x_);
  EXPECT_PRED2(ExpEqual, m1(0, 1), 2.0 / x_);
  EXPECT_PRED2(ExpEqual, m1(1, 0), 3.0 / x_);
  EXPECT_PRED2(ExpEqual, m1(1, 1), 4.0 / x_);

  // [x  y] / x = [x / x    y / x]
  // [z  w]       [z / x    w / x]
  EXPECT_PRED2(ExpEqual, m2(0, 0), x_ / x_);
  EXPECT_PRED2(ExpEqual, m2(0, 1), y_ / x_);
  EXPECT_PRED2(ExpEqual, m2(1, 0), z_ / x_);
  EXPECT_PRED2(ExpEqual, m2(1, 1), w_ / x_);

  // [x + z  x + w] / x = [(x + z) / x    (x + w) / x]
  // [y + z  y + w]       [(y + z) / x    (y + w) / x]
  EXPECT_PRED2(ExpEqual, m3(0, 0), (x_ + z_) / x_);
  EXPECT_PRED2(ExpEqual, m3(0, 1), (x_ + w_) / x_);
  EXPECT_PRED2(ExpEqual, m3(1, 0), (y_ + z_) / x_);
  EXPECT_PRED2(ExpEqual, m3(1, 1), (y_ + w_) / x_);
}
TEST_F(SymbolicVariableTest, OperatorOverloadingEigenDivideVariable) {
  const Eigen::Matrix<Expression, 2, 2> m1{var_mat_ / 3.0};
  const Eigen::Matrix<Expression, 2, 2> m2{var_mat_ / (x_ + y_)};

  // [x  y] / 3.0 = [x / 3.0    y / 3.0]
  // [z  w]         [z / 3.0    w / 3.0]
  EXPECT_PRED2(ExpEqual, m1(0, 0), x_ / 3.0);
  EXPECT_PRED2(ExpEqual, m1(0, 1), y_ / 3.0);
  EXPECT_PRED2(ExpEqual, m1(1, 0), z_ / 3.0);
  EXPECT_PRED2(ExpEqual, m1(1, 1), w_ / 3.0);

  // [x  y] / (x + y) = [x / (x + y)    y / (x + y)]
  // [z  w]             [z / (x + y)    w / (x + y)]
  EXPECT_PRED2(ExpEqual, m2(0, 0), x_ / (x_ + y_));
  EXPECT_PRED2(ExpEqual, m2(0, 1), y_ / (x_ + y_));
  EXPECT_PRED2(ExpEqual, m2(1, 0), z_ / (x_ + y_));
  EXPECT_PRED2(ExpEqual, m2(1, 1), w_ / (x_ + y_));
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
