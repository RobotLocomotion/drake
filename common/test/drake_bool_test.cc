#include "drake/common/drake_bool.h"

#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {

using std::move;

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;
using symbolic::test::ExprEqual;
using symbolic::test::FormulaEqual;

// ---------------
// Case T = double
// ---------------
class BoolTestDouble : public ::testing::Test {
 protected:
  void SetUp() override {
    // clang-format off
    m_ << 1, 2,
          3, 4;
    // clang-format on
  }

  MatrixX<double> m_{2, 2};
  MatrixX<double> zero_m_{0, 0};
};

TEST_F(BoolTestDouble, TypeCheck) {
  static_assert(std::is_same_v<boolean<double>, bool>,
                "boolean<double> should be bool");
  static_assert(std::is_same_v<scalar_predicate<double>::type, bool>,
                "scalar_predicate<double>::type should be bool");
  static_assert(scalar_predicate<double>::is_bool,
                "scalar_predicate<double>::is_bool should be true");
}

TEST_F(BoolTestDouble, All) {
  auto bools = Eigen::Matrix<bool, 3, 3>::Constant(true).eval();
  EXPECT_TRUE(drake::all(bools));
  bools(0, 0) = false;
  EXPECT_FALSE(drake::all(bools));

  // Vacuously true.
  EXPECT_TRUE(drake::all(Eigen::Matrix<bool, 0, 0>::Constant(false)));
}

TEST_F(BoolTestDouble, AllOf) {
  EXPECT_TRUE(all_of(m_, [](const double v) { return v >= 0.0; }));
  EXPECT_FALSE(all_of(m_, [](const double v) { return v >= 2.0; }));

  // Vacuously true.
  EXPECT_TRUE(all_of(zero_m_, [](const double v) { return v >= 0.0; }));
}

TEST_F(BoolTestDouble, Any) {
  auto bools = Eigen::Matrix<bool, 3, 3>::Constant(false).eval();
  EXPECT_FALSE(any(bools));
  bools(0, 0) = true;
  EXPECT_TRUE(any(bools));

  // Vacuously false.
  EXPECT_FALSE(any(Eigen::Matrix<bool, 0, 0>::Constant(true)));
}

TEST_F(BoolTestDouble, AnyOf) {
  EXPECT_TRUE(any_of(m_, [](const double v) { return v >= 4.0; }));
  EXPECT_FALSE(any_of(m_, [](const double v) { return v >= 5.0; }));

  // Vacuously false.
  EXPECT_FALSE(
      any_of(zero_m_, [](const double v) { return v >= 0.0; }));
}

TEST_F(BoolTestDouble, None) {
  auto bools = Eigen::Matrix<bool, 3, 3>::Constant(false).eval();
  EXPECT_TRUE(none(bools));
  bools(0, 0) = true;
  EXPECT_FALSE(none(bools));

  // Vacuously true.
  EXPECT_TRUE(none(Eigen::Matrix<bool, 0, 0>::Constant(true)));
}

TEST_F(BoolTestDouble, NoneOf) {
  EXPECT_TRUE(none_of(m_, [](const double v) { return v >= 5.0; }));
  EXPECT_FALSE(none_of(m_, [](const double v) { return v >= 4.0; }));

  // Vacuously true.
  EXPECT_TRUE(
      none_of(zero_m_, [](const double v) { return v >= 0.0; }));
}

// -------------------
// Case T = AutoDiffXd
// -------------------
class BoolTestAutoDiffXd : public ::testing::Test {
 protected:
  void SetUp() override {
    m_(0, 0) = AutoDiffXd{1., Eigen::Vector2d{1., 0.}};
    m_(0, 1) = AutoDiffXd{0., Eigen::Vector2d{1., 0.}};
    m_(1, 0) = AutoDiffXd{0., Eigen::Vector2d{0., 1.}};
    m_(1, 1) = AutoDiffXd{1., Eigen::Vector2d{0., 1.}};
  }

  MatrixX<AutoDiffXd> m_{2, 2};
  MatrixX<AutoDiffXd> zero_m_{0, 0};
};

TEST_F(BoolTestAutoDiffXd, TypeCheck) {
  static_assert(std::is_same_v<boolean<AutoDiffXd>, bool>,
                "boolean<AutoDiffXd> should be bool");
  static_assert(std::is_same_v<scalar_predicate<AutoDiffXd>::type, bool>,
                "scalar_predicate<AutoDiffXd>::type should be bool");
  static_assert(scalar_predicate<AutoDiffXd>::is_bool,
                "scalar_predicate<AutoDiffXd>::is_bool should be true");
}

TEST_F(BoolTestAutoDiffXd, AllOf) {
  EXPECT_TRUE(all_of(m_,
                     [](const AutoDiffXd& v) {
                       return v.derivatives()[0] == 1.0 ||
                              v.derivatives()[1] == 1.0;
                     }));
  EXPECT_FALSE(
      all_of(m_, [](const AutoDiffXd& v) { return v >= 1.0; }));

  // Vacuously true.
  EXPECT_TRUE(
      all_of(zero_m_, [](const AutoDiffXd& v) { return v >= 1.0; }));
}

TEST_F(BoolTestAutoDiffXd, AnyOf) {
  EXPECT_TRUE(any_of(m_,
                     [](const AutoDiffXd& v) {
                       return v.derivatives()[0] == 1.0 &&
                              v.derivatives()[1] == 0.0;
                     }));
  EXPECT_FALSE(any_of(m_,
                      [](const AutoDiffXd& v) {
                        return v.derivatives()[0] == 1.0 &&
                               v.derivatives()[1] == 1.0;
                      }));

  // Vacuously false.
  EXPECT_FALSE(
      any_of(zero_m_, [](const AutoDiffXd& v) { return v >= 1.0; }));
}

TEST_F(BoolTestAutoDiffXd, NoneOf) {
  EXPECT_TRUE(none_of(m_,
                      [](const AutoDiffXd& v) {
                        return v.derivatives()[0] == 1.0 &&
                               v.derivatives()[1] == 1.0;
                      }));
  EXPECT_FALSE(none_of(m_,
                       [](const AutoDiffXd& v) {
                         return v.derivatives()[0] == 1.0 &&
                                v.derivatives()[1] == 0.0;
                       }));

  // Vacuously true.
  EXPECT_TRUE(
      none_of(zero_m_, [](const AutoDiffXd& v) { return v >= 1.0; }));
}

// -----------------------------
// Case T = symbolic::Expression
// -----------------------------
class BoolTestSymbolic : public ::testing::Test {
 protected:
  void SetUp() override {
    // clang-format off
    m_ <<  x_, y_,
           z_, w_;
    // clang-format on
  }

  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
  const Variable w_{"w"};
  MatrixX<Expression> m_{2, 2};
  MatrixX<Expression> zero_m_{0, 0};
};

TEST_F(BoolTestSymbolic, TypeCheck) {
  static_assert(
      std::is_same_v<boolean<Expression>, Formula>,
      "boolean<Expression> should be Formula");
  static_assert(
      std::is_same_v<scalar_predicate<Expression>::type, Formula>,
      "scalar_predicate<Expression>::type should be Formula");
  static_assert(
      !scalar_predicate<Expression>::is_bool,
      "scalar_predicate<Expression>::is_bool should be false");
}

TEST_F(BoolTestSymbolic, All) {
  const MatrixX<Formula> formulae = m_.unaryExpr(&symbolic::isnan);
  EXPECT_PRED2(FormulaEqual, all(formulae),
               isnan(x_) && isnan(y_) && isnan(z_) && isnan(w_));

  // Vacuously true.
  EXPECT_TRUE(drake::all(MatrixX<Formula>::Constant(0, 0, Formula::False())));
}

TEST_F(BoolTestSymbolic, AllOf) {
  EXPECT_PRED2(
      FormulaEqual,
      all_of(m_, [](const Expression& v) { return !isnan(v); }),
      !isnan(x_) && !isnan(y_) && !isnan(z_) && !isnan(w_));

  // Vacuously true.
  EXPECT_TRUE(
      all_of(zero_m_, [](const Expression& v) { return v >= 0.0; }));
}

TEST_F(BoolTestSymbolic, Any) {
  const MatrixX<Formula> formulae = m_.unaryExpr(&symbolic::isnan);
  EXPECT_PRED2(FormulaEqual, any(formulae),
               isnan(x_) || isnan(y_) || isnan(z_) || isnan(w_));

  // Vacuously false.
  EXPECT_FALSE(any(MatrixX<Formula>::Constant(0, 0, Formula::True())));
}

TEST_F(BoolTestSymbolic, AnyOf) {
  EXPECT_PRED2(FormulaEqual,
               any_of(m_, [](const Expression& v) { return isnan(v); }),
               isnan(x_) || isnan(y_) || isnan(z_) || isnan(w_));

  // Vacuously false.
  EXPECT_FALSE(
      any_of(zero_m_, [](const Expression& v) { return v >= 0.0; }));
}

TEST_F(BoolTestSymbolic, None) {
  const MatrixX<Formula> formulae = m_.unaryExpr(&symbolic::isnan);
  EXPECT_PRED2(FormulaEqual, none(formulae),
               !isnan(x_) && !isnan(y_) && !isnan(z_) && !isnan(w_));

  // Vacuously true.
  EXPECT_TRUE(none(MatrixX<Formula>::Constant(0, 0, Formula::False())));
}

TEST_F(BoolTestSymbolic, NoneOf) {
  EXPECT_PRED2(
      FormulaEqual,
      none_of(m_, [](const Expression& v) { return isnan(v); }),
      !isnan(x_) && !isnan(y_) && !isnan(z_) && !isnan(w_));

  // Vacuously true.
  EXPECT_TRUE(
      none_of(zero_m_, [](const Expression& v) { return v >= 0.0; }));
}

}  // namespace drake
