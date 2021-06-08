#include <functional>

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using test::FormulaEqual;

class SymbolicExpressionArrayTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};

  const Expression zero_{0.0};
  const Expression one_{1.0};
  const Expression two_{2.0};
  const Expression neg_one_{-1.0};
  const Expression pi_{3.141592};
  const Expression neg_pi_{-3.141592};
  const Expression e_{2.718};

  Eigen::Matrix<Expression, 3, 2, Eigen::DontAlign> A_;
  Eigen::Matrix<Expression, 2, 3, Eigen::DontAlign> B_;
  Eigen::Matrix<Expression, 3, 2, Eigen::DontAlign> C_;

  Eigen::Array<Expression, 2, 2, Eigen::DontAlign> array_expr_1_;
  Eigen::Array<Expression, 2, 2, Eigen::DontAlign> array_expr_2_;
  Eigen::Array<Variable, 2, 2, Eigen::DontAlign> array_var_1_;
  Eigen::Array<Variable, 2, 2, Eigen::DontAlign> array_var_2_;
  Eigen::Array<double, 2, 2, Eigen::DontAlign> array_double_;

  void SetUp() override {
    // clang-format off
    A_ << x_, one_,       //  [x  1]
          y_, neg_one_,   //  [y -1]
          z_, pi_;        //  [z  3.141592]

    B_ << x_, y_,  z_,    //  [x     y        z]
          e_, pi_, two_;  //  [2.718 3.141592 2]

    C_ << z_, two_,       //  [z  2]
          x_, e_,         //  [x -2.718]
          y_, pi_;        //  [y  3.141592]

    array_expr_1_ << x_, y_,
                      z_, x_;
    array_expr_2_ << z_, x_,
                      y_, z_;
    array_var_1_ << var_x_, var_y_,
                     var_z_, var_x_;
    array_var_2_ << var_y_, var_z_,
                     var_x_, var_x_;
    array_double_ << 1.0, 2.0,
                      3.0, 4.0;
    // clang-format on
  }
};

// Given two Eigen arrays a1 and a2, it checks if a1 == a2 returns an array
// whose (i, j) element is a formula a1(i, j) == a2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<DerivedA>::XprKind,
                   Eigen::ArrayXpr> &&
        std::is_same_v<typename Eigen::internal::traits<DerivedB>::XprKind,
                       Eigen::ArrayXpr>,
    bool>
CheckArrayOperatorEq(const DerivedA& a1, const DerivedB& a2) {
  const auto arr = (a1 == a2);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a1(i, j) == a2(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given a scalar-type object @p v and an Eigen array @p a, it checks if v == a
// returns an array whose (i, j)-element is a formula v == a(i, j).
template <typename ScalarType, typename Derived>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<Derived>::XprKind,
                   Eigen::ArrayXpr> &&
        std::is_same_v<decltype(ScalarType() == typename Derived::Scalar()),
                       Formula>,
    bool>
CheckArrayOperatorEq(const ScalarType& v, const Derived& a) {
  const Eigen::Array<Formula, Derived::RowsAtCompileTime,
                     Derived::ColsAtCompileTime>
      arr = (v == a);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(v == a(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given an Eigen array @p a and a scalar-type object @p v, it checks if a == v
// returns an array whose (i, j)-element is a formula a(i, j) == v.
template <typename Derived, typename ScalarType>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<Derived>::XprKind,
                   Eigen::ArrayXpr> &&
        std::is_same_v<decltype(typename Derived::Scalar() == ScalarType()),
                       Formula>,
    bool>
CheckArrayOperatorEq(const Derived& a, const ScalarType& v) {
  const Eigen::Array<Formula, Derived::RowsAtCompileTime,
                     Derived::ColsAtCompileTime>
      arr = (a == v);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a(i, j) == v)) {
        return false;
      }
    }
  }
  return true;
}

// Given two Eigen matrices m1 and m2, it checks if m1.array() == m2.array()
// returns an array whose (i, j) element is a formula m1(i, j) == m2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<DerivedA>::XprKind,
                   Eigen::MatrixXpr> &&
        std::is_same_v<typename Eigen::internal::traits<DerivedB>::XprKind,
                       Eigen::MatrixXpr>,
    bool>
CheckArrayOperatorEq(const DerivedA& m1, const DerivedB& m2) {
  return CheckArrayOperatorEq(m1.array(), m2.array());
}

// Given two Eigen arrays a1 and a2, it checks if a1 <= a2 returns an array
// whose (i, j) element is a formula a1(i, j) <= a2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::ArrayBase<DerivedA>, DerivedA> &&
        std::is_base_of_v<Eigen::ArrayBase<DerivedB>, DerivedB>,
    bool>
CheckArrayOperatorLte(const DerivedA& a1, const DerivedB& a2) {
  const auto arr = (a1 <= a2);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a1(i, j) <= a2(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given a scalar-type object @p v and an Eigen array @p a, it checks if v <= a
// returns an array whose (i, j)-element is a formula v <= a(i, j).
template <typename ScalarType, typename Derived>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<Derived>::XprKind,
                   Eigen::ArrayXpr> &&
        std::is_same_v<decltype(ScalarType() <= typename Derived::Scalar()),
                       Formula>,
    bool>
CheckArrayOperatorLte(const ScalarType& v, const Derived& a) {
  const auto arr = (v <= a);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(v <= a(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given an Eigen array @p a and a scalar-type object @p v, it checks if a <= v
// returns an array whose (i, j)-element is a formula a(i, j) <= v.
template <typename Derived, typename ScalarType>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<Derived>::XprKind,
                   Eigen::ArrayXpr> &&
        std::is_same_v<decltype(typename Derived::Scalar() <= ScalarType()),
                       Formula>,
    bool>
CheckArrayOperatorLte(const Derived& a, const ScalarType& v) {
  const auto arr = (a <= v);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a(i, j) <= v)) {
        return false;
      }
    }
  }
  return true;
}

// Given two Eigen matrices m1 and m2, it checks if m1.array() <= m2.array()
// returns an array whose (i, j) element is a formula m1(i, j) <= m2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<DerivedA>::XprKind,
                   Eigen::MatrixXpr> &&
        std::is_same_v<typename Eigen::internal::traits<DerivedB>::XprKind,
                       Eigen::MatrixXpr>,
    bool>
CheckArrayOperatorLte(const DerivedA& m1, const DerivedB& m2) {
  return CheckArrayOperatorLte(m1.array(), m2.array());
}

// Given two Eigen arrays a1 and a2, it checks if a1 < a2 returns an array whose
// (i, j) element is a formula a1(i, j) < a2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::ArrayBase<DerivedA>, DerivedA> &&
        std::is_base_of_v<Eigen::ArrayBase<DerivedB>, DerivedB>,
    bool>
CheckArrayOperatorLt(const DerivedA& a1, const DerivedB& a2) {
  const auto arr = (a1 < a2);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a1(i, j) < a2(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given a scalar-type object @p v and an Eigen array @p a, it checks if v < a
// returns an array whose (i, j)-element is a formula v < a(i, j).
template <typename ScalarType, typename Derived>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<Derived>::XprKind,
                   Eigen::ArrayXpr> &&
        std::is_same_v<decltype(ScalarType() < typename Derived::Scalar()),
                       Formula>,
    bool>
CheckArrayOperatorLt(const ScalarType& v, const Derived& a) {
  const auto arr = (v < a);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(v < a(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given an Eigen array @p a and a scalar-type object @p v, it checks if a < v
// returns an array whose (i, j)-element is a formula a(i, j) < v.
template <typename Derived, typename ScalarType>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<Derived>::XprKind,
                   Eigen::ArrayXpr> &&
        std::is_same_v<decltype(typename Derived::Scalar() < ScalarType()),
                       Formula>,
    bool>
CheckArrayOperatorLt(const Derived& a, const ScalarType& v) {
  const auto arr = (a < v);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a(i, j) < v)) {
        return false;
      }
    }
  }
  return true;
}

// Given two Eigen matrices m1 and m2, it checks if m1.array() < m2.array()
// returns an array whose (i, j) element is a formula m1(i, j) < m2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<DerivedA>::XprKind,
                   Eigen::MatrixXpr> &&
        std::is_same_v<typename Eigen::internal::traits<DerivedB>::XprKind,
                       Eigen::MatrixXpr>,
    bool>
CheckArrayOperatorLt(const DerivedA& m1, const DerivedB& m2) {
  return CheckArrayOperatorLt(m1.array(), m2.array());
}

// Given two Eigen arrays a1 and a2, it checks if a1 >= a2 returns an array
// whose (i, j) element is a formula a1(i, j) >= a2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::ArrayBase<DerivedA>, DerivedA> &&
        std::is_base_of_v<Eigen::ArrayBase<DerivedB>, DerivedB>,
    bool>
CheckArrayOperatorGte(const DerivedA& a1, const DerivedB& a2) {
  const auto arr = (a1 >= a2);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a1(i, j) >= a2(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given a scalar-type object @p v and an Eigen array @p a, it checks if v >= a
// returns an array whose (i, j)-element is a formula a(i, j) <= v.
template <typename ScalarType, typename Derived>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<Derived>::XprKind,
                   Eigen::ArrayXpr> &&
        std::is_same_v<decltype(ScalarType() >= typename Derived::Scalar()),
                       Formula>,
    bool>
CheckArrayOperatorGte(const ScalarType& v, const Derived& a) {
  const auto arr = (v >= a);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      // Note that arr(i, j) should be `a(i, j) <= v` instead of `v >= a(i, j)`.
      if (!arr(i, j).EqualTo(a(i, j) <= v)) {
        return false;
      }
    }
  }
  return true;
}

// Given an Eigen array @p a and a scalar-type object @p v, it checks if a >= v
// returns an array whose (i, j)-element is a formula a(i, j) >= v.
template <typename Derived, typename ScalarType>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<Derived>::XprKind,
                   Eigen::ArrayXpr> &&
        std::is_same_v<decltype(typename Derived::Scalar() >= ScalarType()),
                       Formula>,
    bool>
CheckArrayOperatorGte(const Derived& a, const ScalarType& v) {
  const auto arr = (a >= v);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      // TODO(soonho): Add note here.
      if (!arr(i, j).EqualTo(a(i, j) >= v)) {
        return false;
      }
    }
  }
  return true;
}

// Given two Eigen matrices m1 and m2, it checks if m1.array() >= m2.array()
// returns an array whose (i, j) element is a formula m1(i, j) >= m2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<DerivedA>::XprKind,
                   Eigen::MatrixXpr> &&
        std::is_same_v<typename Eigen::internal::traits<DerivedB>::XprKind,
                       Eigen::MatrixXpr>,
    bool>
CheckArrayOperatorGte(const DerivedA& m1, const DerivedB& m2) {
  return CheckArrayOperatorGte(m1.array(), m2.array());
}

// Given two Eigen arrays a1 and a2, it checks if a1 > a2 returns an array whose
// (i, j) element is a formula a1(i, j) > a2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::ArrayBase<DerivedA>, DerivedA> &&
        std::is_base_of_v<Eigen::ArrayBase<DerivedB>, DerivedB>,
    bool>
CheckArrayOperatorGt(const DerivedA& a1, const DerivedB& a2) {
  const auto arr = (a1 > a2);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a1(i, j) > a2(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given a scalar-type object @p v and an Eigen array @p a, it checks if v > a
// returns an array whose (i, j)-element is a formula a(i, j) < v.
template <typename ScalarType, typename Derived>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<Derived>::XprKind,
                   Eigen::ArrayXpr> &&
        std::is_same_v<decltype(ScalarType() > typename Derived::Scalar()),
                       Formula>,
    bool>
CheckArrayOperatorGt(const ScalarType& v, const Derived& a) {
  const auto arr = (v > a);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      // Note that arr(i, j) should be `a(i, j) < v` instead of `v > a(i, j)`.
      if (!arr(i, j).EqualTo(a(i, j) < v)) {
        return false;
      }
    }
  }
  return true;
}

// Given an Eigen array @p a and a scalar-type object @p v, it checks if a > v
// returns an array whose (i, j)-element is a formula a(i, j) > v.
template <typename Derived, typename ScalarType>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<Derived>::XprKind,
                   Eigen::ArrayXpr> &&
        std::is_same_v<decltype(typename Derived::Scalar() > ScalarType()),
                       Formula>,
    bool>
CheckArrayOperatorGt(const Derived& a, const ScalarType& v) {
  const auto arr = (a > v);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a(i, j) > v)) {
        return false;
      }
    }
  }
  return true;
}

// Given two Eigen matrices m1 and m2, it checks if m1.array() > m2.array()
// returns an array whose (i, j) element is a formula m1(i, j) > m2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<DerivedA>::XprKind,
                   Eigen::MatrixXpr> &&
        std::is_same_v<typename Eigen::internal::traits<DerivedB>::XprKind,
                       Eigen::MatrixXpr>,
    bool>
CheckArrayOperatorGt(const DerivedA& m1, const DerivedB& m2) {
  return CheckArrayOperatorGt(m1.array(), m2.array());
}

// Given two Eigen arrays a1 and a2, it checks if a1 != a2 returns an array
// whose (i, j) element is a formula a1(i, j) != a2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::ArrayBase<DerivedA>, DerivedA> &&
        std::is_base_of_v<Eigen::ArrayBase<DerivedB>, DerivedB>,
    bool>
CheckArrayOperatorNeq(const DerivedA& a1, const DerivedB& a2) {
  const auto arr = (a1 != a2);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a1(i, j) != a2(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given a scalar-type object @p v and an Eigen array @p a, it checks if v != a
// returns an array whose (i, j)-element is a formula v != a(i, j).
template <typename ScalarType, typename Derived>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<Derived>::XprKind,
                   Eigen::ArrayXpr> &&
        std::is_same_v<decltype(ScalarType() != typename Derived::Scalar()),
                       Formula>,
    bool>
CheckArrayOperatorNeq(const ScalarType& v, const Derived& a) {
  const auto arr = (v != a);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(v != a(i, j))) {
        return false;
      }
    }
  }
  return true;
}

// Given an Eigen array @p a and a scalar-type object @p v, it checks if a != v
// returns an array whose (i, j)-element is a formula a(i, j) != v.
template <typename Derived, typename ScalarType>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<Derived>::XprKind,
                   Eigen::ArrayXpr> &&
        std::is_same_v<decltype(typename Derived::Scalar() != ScalarType()),
                       Formula>,
    bool>
CheckArrayOperatorNeq(const Derived& a, const ScalarType& v) {
  const auto arr = (a != v);
  for (int i = 0; i < arr.rows(); ++i) {
    for (int j = 0; j < arr.cols(); ++j) {
      if (!arr(i, j).EqualTo(a(i, j) != v)) {
        return false;
      }
    }
  }
  return true;
}

// Given two Eigen matrices m1 and m2, it checks if m1.array() != m2.array()
// returns an array whose (i, j) element is a formula m1(i, j) != m2(i, j).
template <typename DerivedA, typename DerivedB>
typename std::enable_if_t<
    std::is_same_v<typename Eigen::internal::traits<DerivedA>::XprKind,
                   Eigen::MatrixXpr> &&
        std::is_same_v<typename Eigen::internal::traits<DerivedB>::XprKind,
                       Eigen::MatrixXpr>,
    bool>
CheckArrayOperatorNeq(const DerivedA& m1, const DerivedB& m2) {
  return CheckArrayOperatorNeq(m1.array(), m2.array());
}

TEST_F(SymbolicExpressionArrayTest, ArrayExprEqArrayExpr) {
  const Eigen::Array<Formula, 3, 2> a1{A_.array() == A_.array()};
  const Eigen::Array<Formula, 2, 3> a2{B_.array() == B_.array()};
  const Eigen::Array<Formula, 3, 2> a3{C_.array() == C_.array()};
  auto is_true_lambda = [](const Formula& f) {return is_true(f);};
  EXPECT_TRUE(a1.unaryExpr(is_true_lambda).all());
  EXPECT_TRUE(a2.unaryExpr(is_true_lambda).all());
  EXPECT_TRUE(a3.unaryExpr(is_true_lambda).all());
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Expression>
// and Array<Expression>.
TEST_F(SymbolicExpressionArrayTest, ArrayExprRopArrayExpr) {
  EXPECT_TRUE(CheckArrayOperatorEq(A_, C_));
  EXPECT_TRUE(CheckArrayOperatorEq(B_ * A_, B_ * C_));
  EXPECT_TRUE(CheckArrayOperatorLte(A_, C_));
  EXPECT_TRUE(CheckArrayOperatorLte(B_ * A_, B_ * C_));
  EXPECT_TRUE(CheckArrayOperatorLt(A_, C_));
  EXPECT_TRUE(CheckArrayOperatorLt(B_ * A_, B_ * C_));
  EXPECT_TRUE(CheckArrayOperatorGte(A_, C_));
  EXPECT_TRUE(CheckArrayOperatorGte(B_ * A_, B_ * C_));
  EXPECT_TRUE(CheckArrayOperatorGt(A_, C_));
  EXPECT_TRUE(CheckArrayOperatorGt(B_ * A_, B_ * C_));
  EXPECT_TRUE(CheckArrayOperatorNeq(A_, C_));
  EXPECT_TRUE(CheckArrayOperatorNeq(B_ * A_, B_ * C_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Expression>
// and Array<Variable>
TEST_F(SymbolicExpressionArrayTest, ArrayExprRopArrayVar) {
  EXPECT_TRUE(CheckArrayOperatorEq(array_expr_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorEq(array_var_2_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_expr_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_var_2_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_expr_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_var_2_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_expr_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_var_2_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_expr_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_var_2_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_expr_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_var_2_, array_expr_1_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Expression>
// and Array<double>
TEST_F(SymbolicExpressionArrayTest, ArrayExprRopArrayDouble) {
  EXPECT_TRUE(CheckArrayOperatorEq(array_expr_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorEq(array_double_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_expr_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_double_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_expr_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_double_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_expr_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_double_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_expr_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_double_, array_expr_1_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_expr_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_double_, array_expr_1_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Variable>
// and Array<double>
TEST_F(SymbolicExpressionArrayTest, ArrayVarRopArrayDouble) {
  EXPECT_TRUE(CheckArrayOperatorEq(array_var_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorEq(array_double_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_var_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_double_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_var_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_double_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_var_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_double_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_var_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_double_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_var_1_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_double_, array_var_1_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Variable>
// and Array<Variable>
TEST_F(SymbolicExpressionArrayTest, ArrayVarRopArrayVar) {
  EXPECT_TRUE(CheckArrayOperatorEq(array_var_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorEq(array_var_2_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_var_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_var_2_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_var_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_var_2_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_var_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_var_2_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_var_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_var_2_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_var_1_, array_var_2_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_var_2_, array_var_1_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Expression>
// and Expression.
TEST_F(SymbolicExpressionArrayTest, ArrayExprRopExpr) {
  EXPECT_TRUE(CheckArrayOperatorEq(A_.array(), Expression{0.0}));
  EXPECT_TRUE(CheckArrayOperatorEq(x_ + y_, (B_ * C_).array()));
  EXPECT_TRUE(CheckArrayOperatorLte(A_.array(), Expression{0.0}));
  EXPECT_TRUE(CheckArrayOperatorLte(x_ + y_, (B_ * C_).array()));
  EXPECT_TRUE(CheckArrayOperatorLt(A_.array(), Expression{0.0}));
  EXPECT_TRUE(CheckArrayOperatorLt(x_ + y_, (B_ * C_).array()));
  EXPECT_TRUE(CheckArrayOperatorGte(A_.array(), Expression{0.0}));
  EXPECT_TRUE(CheckArrayOperatorGte(x_ + y_, (B_ * C_).array()));
  EXPECT_TRUE(CheckArrayOperatorGt(A_.array(), Expression{0.0}));
  EXPECT_TRUE(CheckArrayOperatorGt(x_ + y_, (B_ * C_).array()));
  EXPECT_TRUE(CheckArrayOperatorNeq(A_.array(), Expression{0.0}));
  EXPECT_TRUE(CheckArrayOperatorNeq(x_ + y_, (B_ * C_).array()));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Expression>
// and Variable.
TEST_F(SymbolicExpressionArrayTest, ArrayExprRopVar) {
  EXPECT_TRUE(CheckArrayOperatorEq(A_.array(), var_x_));
  EXPECT_TRUE(CheckArrayOperatorEq(var_x_, (B_ * C_).array()));
  EXPECT_TRUE(CheckArrayOperatorLte(A_.array(), var_x_));
  EXPECT_TRUE(CheckArrayOperatorLte(var_x_, (B_ * C_).array()));
  EXPECT_TRUE(CheckArrayOperatorLt(A_.array(), var_x_));
  EXPECT_TRUE(CheckArrayOperatorLt(var_x_, (B_ * C_).array()));
  EXPECT_TRUE(CheckArrayOperatorGte(A_.array(), var_x_));
  EXPECT_TRUE(CheckArrayOperatorGte(var_x_, (B_ * C_).array()));
  EXPECT_TRUE(CheckArrayOperatorGt(A_.array(), var_x_));
  EXPECT_TRUE(CheckArrayOperatorGt(var_x_, (B_ * C_).array()));
  EXPECT_TRUE(CheckArrayOperatorNeq(A_.array(), var_x_));
  EXPECT_TRUE(CheckArrayOperatorNeq(var_x_, (B_ * C_).array()));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Expression>
// and double.
TEST_F(SymbolicExpressionArrayTest, ArrayExprRopDouble) {
  EXPECT_TRUE(CheckArrayOperatorEq(A_.array(), 0.0));
  EXPECT_TRUE(CheckArrayOperatorEq(1.0, (B_ * C_).array()));
  EXPECT_TRUE(CheckArrayOperatorLte(A_.array(), 0.0));
  EXPECT_TRUE(CheckArrayOperatorLte(1.0, (B_ * C_).array()));
  EXPECT_TRUE(CheckArrayOperatorLt(A_.array(), 0.0));
  EXPECT_TRUE(CheckArrayOperatorLt(1.0, (B_ * C_).array()));
  EXPECT_TRUE(CheckArrayOperatorGte(A_.array(), 0.0));
  EXPECT_TRUE(CheckArrayOperatorGte(1.0, (B_ * C_).array()));
  EXPECT_TRUE(CheckArrayOperatorGt(A_.array(), 0.0));
  EXPECT_TRUE(CheckArrayOperatorGt(1.0, (B_ * C_).array()));
  EXPECT_TRUE(CheckArrayOperatorNeq(A_.array(), 0.0));
  EXPECT_TRUE(CheckArrayOperatorNeq(1.0, (B_ * C_).array()));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Variable>
// and Expression.
TEST_F(SymbolicExpressionArrayTest, ArraryVarRopExpr) {
  EXPECT_TRUE(CheckArrayOperatorEq(array_var_1_, x_ + y_));
  EXPECT_TRUE(CheckArrayOperatorEq(x_ + y_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_var_1_, x_ + y_));
  EXPECT_TRUE(CheckArrayOperatorLte(x_ + y_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_var_1_, x_ + y_));
  EXPECT_TRUE(CheckArrayOperatorLt(x_ + y_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_var_1_, x_ + y_));
  EXPECT_TRUE(CheckArrayOperatorGte(x_ + y_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_var_1_, x_ + y_));
  EXPECT_TRUE(CheckArrayOperatorGt(x_ + y_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_var_1_, x_ + y_));
  EXPECT_TRUE(CheckArrayOperatorNeq(x_ + y_, array_var_1_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Variable>
// and Variable.
TEST_F(SymbolicExpressionArrayTest, ArraryVarRopVar) {
  EXPECT_TRUE(CheckArrayOperatorEq(array_var_1_, var_x_));
  EXPECT_TRUE(CheckArrayOperatorEq(var_x_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_var_1_, var_x_));
  EXPECT_TRUE(CheckArrayOperatorLte(var_x_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_var_1_, var_x_));
  EXPECT_TRUE(CheckArrayOperatorLt(var_x_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_var_1_, var_x_));
  EXPECT_TRUE(CheckArrayOperatorGte(var_x_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_var_1_, var_x_));
  EXPECT_TRUE(CheckArrayOperatorGt(var_x_, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_var_1_, var_x_));
  EXPECT_TRUE(CheckArrayOperatorNeq(var_x_, array_var_1_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<Variable>
// and double.
TEST_F(SymbolicExpressionArrayTest, ArraryVarRopDouble) {
  EXPECT_TRUE(CheckArrayOperatorEq(array_var_1_, 3.0));
  EXPECT_TRUE(CheckArrayOperatorEq(3.0, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_var_1_, 3.0));
  EXPECT_TRUE(CheckArrayOperatorLte(3.0, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_var_1_, 3.0));
  EXPECT_TRUE(CheckArrayOperatorLt(3.0, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_var_1_, 3.0));
  EXPECT_TRUE(CheckArrayOperatorGte(3.0, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_var_1_, 3.0));
  EXPECT_TRUE(CheckArrayOperatorGt(3.0, array_var_1_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_var_1_, 3.0));
  EXPECT_TRUE(CheckArrayOperatorNeq(3.0, array_var_1_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<double>
// and Expression.
TEST_F(SymbolicExpressionArrayTest, ArraryDoubleRopExpr) {
  EXPECT_TRUE(CheckArrayOperatorEq(array_double_, x_ + y_));
  EXPECT_TRUE(CheckArrayOperatorEq(x_ + y_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_double_, x_ + y_));
  EXPECT_TRUE(CheckArrayOperatorLte(x_ + y_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_double_, x_ + y_));
  EXPECT_TRUE(CheckArrayOperatorLt(x_ + y_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_double_, x_ + y_));
  EXPECT_TRUE(CheckArrayOperatorGte(x_ + y_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_double_, x_ + y_));
  EXPECT_TRUE(CheckArrayOperatorGt(x_ + y_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_double_, x_ + y_));
  EXPECT_TRUE(CheckArrayOperatorNeq(x_ + y_, array_double_));
}

// Checks relational operators (==, !=, <=, <, >=, >) between Array<double>
// and Variable.
TEST_F(SymbolicExpressionArrayTest, ArraryDoubleRopVar) {
  EXPECT_TRUE(CheckArrayOperatorEq(array_double_, var_x_));
  EXPECT_TRUE(CheckArrayOperatorEq(var_x_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorLte(array_double_, var_x_));
  EXPECT_TRUE(CheckArrayOperatorLte(var_x_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorLt(array_double_, var_x_));
  EXPECT_TRUE(CheckArrayOperatorLt(var_x_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorGte(array_double_, var_x_));
  EXPECT_TRUE(CheckArrayOperatorGte(var_x_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorGt(array_double_, var_x_));
  EXPECT_TRUE(CheckArrayOperatorGt(var_x_, array_double_));
  EXPECT_TRUE(CheckArrayOperatorNeq(array_double_, var_x_));
  EXPECT_TRUE(CheckArrayOperatorNeq(var_x_, array_double_));
}

TEST_F(SymbolicExpressionArrayTest, ArrayOperatorReturnType) {
  Eigen::Array<Variable, 2, Eigen::Dynamic> m1(2, 2);
  Eigen::Array<Variable, Eigen::Dynamic, 2> m2(2, 2);
  EXPECT_TRUE(
      (std::is_same_v<decltype(m1 == m2), Eigen::Array<Formula, 2, 2>>));
  EXPECT_TRUE(
      (std::is_same_v<decltype(m1 != m2), Eigen::Array<Formula, 2, 2>>));
  EXPECT_TRUE(
      (std::is_same_v<decltype(m1 <= m2), Eigen::Array<Formula, 2, 2>>));
  EXPECT_TRUE(
      (std::is_same_v<decltype(m1 < m2), Eigen::Array<Formula, 2, 2>>));
  EXPECT_TRUE(
      (std::is_same_v<decltype(m1 >= m2), Eigen::Array<Formula, 2, 2>>));
  EXPECT_TRUE(
      (std::is_same_v<decltype(m1 > m2), Eigen::Array<Formula, 2, 2>>));
}

TEST_F(SymbolicExpressionArrayTest, ExpressionArraySegment) {
  Eigen::Array<Expression, 5, 1> v;
  v << x_, 1, y_, x_, 1;
  const auto s1 = v.segment(0, 2);  // [x, 1]
  const auto s2 = v.segment<2>(1);  // [1, y]
  const auto s3 = v.segment(3, 2);  // [x, 1]
  const auto a1 = (s1 == s2);       // [x = 1, 1 = y]
  const auto a2 = (s1 == s3);       // [True, True]

  EXPECT_PRED2(FormulaEqual, a1(0), x_ == 1);
  EXPECT_PRED2(FormulaEqual, a1(1), 1 == y_);
  EXPECT_TRUE(is_true(a2(0)));
  EXPECT_TRUE(is_true(a2(1)));
}

TEST_F(SymbolicExpressionArrayTest, ExpressionArrayBlock) {
  Eigen::Array<Expression, 3, 3> m;
  // clang-format off
  m << x_, y_, z_,
       y_, 1, 2,
       z_, 3, 4;
  // clang-format on

  // b1 = [x, y]
  //      [y, 1]
  const auto b1 = m.block<2, 2>(0, 0);
  // b2 = [1, 2]
  //      [3, 4]
  const auto b2 = m.block(1, 1, 2, 2);
  // a = [x = 1, y = 2]
  //     [y = 3, False]
  const auto a = (b1 == b2);

  EXPECT_PRED2(FormulaEqual, a(0, 0), x_ == 1);
  EXPECT_PRED2(FormulaEqual, a(0, 1), y_ == 2);
  EXPECT_PRED2(FormulaEqual, a(1, 0), y_ == 3);
  EXPECT_TRUE(is_false(a(1, 1)));
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
