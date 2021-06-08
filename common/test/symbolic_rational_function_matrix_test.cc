#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
using test::RationalFunctionEqual;

class SymbolicRationalFunctionMatrixTest : public ::testing::Test {
 public:
  void SetUp() override {
    M_rational_function_dynamic_ << RationalFunction(p1_, p2_),
        RationalFunction(p1_, p3_), RationalFunction(p2_, p3_),
        RationalFunction(p1_, p2_ + p3_);
    M_rational_function_static_ = M_rational_function_dynamic_;
    v_rational_function_dynamic_ << RationalFunction(p2_, p4_),
        RationalFunction(p3_, p2_ + p4_);
    v_rational_function_static_ = v_rational_function_dynamic_;

    M_poly_dynamic_ << p1_, p2_, p5_, p6_;
    M_poly_static_ = M_poly_dynamic_;
    v_poly_dynamic_ << p1_, p3_;
    v_poly_static_ = v_poly_dynamic_;

    M_double_dynamic_ << 1, 2, 3, 4;
    M_double_static_ = M_double_dynamic_;
    v_double_dynamic_ << 1, 2;
    v_double_static_ = v_double_dynamic_;
  }

 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Variables var_xyz_{{var_x_, var_y_, var_z_}};

  const Variable var_a_{"a"};
  const Variable var_b_{"b"};
  const Variable var_c_{"c"};
  const Variables var_abc_{{var_a_, var_b_, var_c_}};

  const Polynomial p1_{var_x_ * var_x_};
  const Polynomial p2_{var_x_ * var_y_ + 2 * var_y_ + 1};
  const Polynomial p3_{var_x_ * var_z_ + var_a_ * var_x_ * var_x_, var_xyz_};
  const Polynomial p4_{var_a_ * var_x_ * var_y_ + var_b_ * var_x_ * var_z_,
                       var_xyz_};
  const Polynomial p5_{var_a_ * var_x_ + var_b_ * 2 * var_z_ * var_y_,
                       var_xyz_};
  const Polynomial p6_{
      3 * var_a_ * var_x_ * var_x_ + var_b_ * 2 * var_z_ * var_y_, var_xyz_};

  MatrixX<RationalFunction> M_rational_function_dynamic_{2, 2};
  Matrix2<RationalFunction> M_rational_function_static_;
  VectorX<RationalFunction> v_rational_function_dynamic_{2};
  Vector2<RationalFunction> v_rational_function_static_;

  MatrixX<Polynomial> M_poly_dynamic_{2, 2};
  Matrix2<Polynomial> M_poly_static_;
  VectorX<Polynomial> v_poly_dynamic_{2};
  Vector2<Polynomial> v_poly_static_;

  Eigen::MatrixXd M_double_dynamic_{2, 2};
  Eigen::Matrix2d M_double_static_;
  Eigen::VectorXd v_double_dynamic_{2};
  Eigen::Vector2d v_double_static_;
};

template <typename Derived1, typename Derived2>
typename std::enable_if_t<
    std::is_same_v<typename Derived1::Scalar, RationalFunction> &&
    std::is_same_v<typename Derived2::Scalar, RationalFunction>>
CompareMatrixWithRationalFunction(const Derived1& m1, const Derived2& m2) {
  EXPECT_EQ(m1.rows(), m2.rows());
  EXPECT_EQ(m1.cols(), m2.cols());
  for (int i = 0; i < m1.rows(); ++i) {
    for (int j = 0; j < m1.cols(); ++j) {
      EXPECT_PRED2(test::PolyEqualAfterExpansion, m1(i, j).numerator(),
                   m2(i, j).numerator());
      EXPECT_PRED2(test::PolyEqualAfterExpansion, m1(i, j).denominator(),
                   m2(i, j).denominator());
    }
  }
}

template <typename Derived1, typename Derived2>
void CheckAddition(const Derived1& m1, const Derived2& m2) {
  DRAKE_DEMAND(m1.rows() == m2.rows());
  DRAKE_DEMAND(m1.cols() == m2.cols());
  MatrixX<RationalFunction> m1_add_m2_expected(m1.rows(), m1.cols());
  for (int i = 0; i < m1.rows(); ++i) {
    for (int j = 0; j < m1.cols(); ++j) {
      m1_add_m2_expected(i, j) = m1(i, j) + m2(i, j);
    }
  }
  static_assert(
      std::is_same_v<typename decltype(m1 + m2)::Scalar, RationalFunction>,
      "m1 + m2 should have scalar type RationalFunction.");
  const MatrixX<RationalFunction> m1_add_m2 = m1 + m2;
  CompareMatrixWithRationalFunction(m1_add_m2, m1_add_m2_expected);
  CompareMatrixWithRationalFunction(m2 + m1, m1_add_m2_expected);
}

template <typename Derived1, typename Derived2>
void CheckSubtraction(const Derived1& m1, const Derived2& m2) {
  DRAKE_DEMAND(m1.rows() == m2.rows());
  DRAKE_DEMAND(m1.cols() == m2.cols());
  MatrixX<RationalFunction> m1_minus_m2_expected(m1.rows(), m1.cols());
  for (int i = 0; i < m1.rows(); ++i) {
    for (int j = 0; j < m1.cols(); ++j) {
      m1_minus_m2_expected(i, j) = m1(i, j) - m2(i, j);
    }
  }
  static_assert(
      std::is_same_v<typename decltype(m1 - m2)::Scalar, RationalFunction>,
      "m1 - m2 should have scalar type RationalFunction.");
  const MatrixX<RationalFunction> m1_minus_m2 = m1 - m2;
  CompareMatrixWithRationalFunction(m1_minus_m2, m1_minus_m2_expected);
  CompareMatrixWithRationalFunction(m2 - m1, -m1_minus_m2_expected);
}

template <typename Derived1, typename Derived2>
void CheckProduct(const Derived1& m1, const Derived2& m2) {
  // If we change the type from Derived1 to Eigen::MatrixBase<Derived1>, the
  // compiler would fail, as the SFINAE fails for the overloaded operator* in
  // symbolic_ration_function.h.
  DRAKE_DEMAND(m1.cols() == m2.rows());
  MatrixX<RationalFunction> m1_times_m2_expected(m1.rows(), m2.cols());
  for (int i = 0; i < m1.rows(); ++i) {
    for (int j = 0; j < m2.cols(); ++j) {
      for (int k = 0; k < m1.cols(); ++k) {
        m1_times_m2_expected(i, j) += m1(i, k) * m2(k, j);
      }
    }
  }

  static_assert(
      std::is_same_v<typename decltype(m1 * m2)::Scalar, RationalFunction>,
      "m1 * m2 should have scalar type RationalFunction.");
  const MatrixX<RationalFunction> m1_times_m2 = m1 * m2;
  CompareMatrixWithRationalFunction(m1_times_m2, m1_times_m2_expected);
}

template <typename Derived1, typename Derived2>
typename std::enable_if_t<is_eigen_vector<Derived1>::value &&
                          is_eigen_vector<Derived2>::value>
CheckConjugateProdocut(const Derived1& v1, const Derived2& v2) {
  DRAKE_DEMAND(v1.rows() == v2.rows());
  RationalFunction v1_dot_v2_expected;
  for (int i = 0; i < v1.rows(); ++i) {
    v1_dot_v2_expected += v1(i) * v2(i);
  }
  static_assert(std::is_same_v<decltype(v1.dot(v2)), RationalFunction>,
                "v1.dot(v2) should be RationalFunction.");
  const RationalFunction v1_dot_v2 = v1.dot(v2);
  EXPECT_PRED2(test::PolyEqualAfterExpansion, v1_dot_v2.denominator(),
               v1_dot_v2_expected.denominator());
  EXPECT_PRED2(test::PolyEqualAfterExpansion, v1_dot_v2.numerator(),
               v1_dot_v2_expected.numerator());
}

template <typename Derived1, typename Derived2>
void CheckMatrixMatrixBinaryOperations(const Derived1& m1, const Derived2& m2) {
  CheckAddition(m1, m2);
  CheckSubtraction(m1, m2);
  CheckProduct(m1, m2);
  CheckProduct(m2, m1);
}

template <typename Derived1, typename Derived2>
typename std::enable_if_t<is_eigen_vector<Derived2>::value>
CheckMatrixVectorBinaryOperations(const Derived1& m1, const Derived2& m2) {
  CheckProduct(m1, m2);
}

template <typename Derived1, typename Derived2>
typename std::enable_if_t<is_eigen_vector<Derived1>::value &&
                          is_eigen_vector<Derived2>::value>
CheckVectorVectorBinaryOperations(const Derived1& m1, const Derived2& m2) {
  CheckAddition(m1, m2);
  CheckSubtraction(m1, m2);
  CheckConjugateProdocut(m1, m2);
}

TEST_F(SymbolicRationalFunctionMatrixTest, RationalFunctionOpRationalFunction) {
  Matrix2<RationalFunction> M2;
  M2 << RationalFunction(p2_, p3_ + 2 * p4_),
      RationalFunction(p1_ + p2_, 2 * p3_), RationalFunction(p1_, p4_ - p5_),
      RationalFunction(p2_, p3_ * p6_);

  CheckMatrixMatrixBinaryOperations(M_rational_function_static_, M2);
  CheckMatrixMatrixBinaryOperations(M_rational_function_dynamic_, M2);
  Vector2<RationalFunction> v2(RationalFunction(p1_, p4_ + 2 * p5_),
                               RationalFunction(p3_, p2_ - p1_));
  CheckVectorVectorBinaryOperations(v_rational_function_static_, v2);
  CheckVectorVectorBinaryOperations(v_rational_function_dynamic_, v2);

  CheckMatrixVectorBinaryOperations(M_rational_function_static_,
                                    v_rational_function_static_);
  CheckMatrixVectorBinaryOperations(M_rational_function_dynamic_,
                                    v_rational_function_static_);
  CheckMatrixVectorBinaryOperations(M_rational_function_static_,
                                    v_rational_function_dynamic_);
  CheckMatrixVectorBinaryOperations(M_rational_function_dynamic_,
                                    v_rational_function_dynamic_);
}

TEST_F(SymbolicRationalFunctionMatrixTest, RationalFunctionOpPolynomial) {
  CheckMatrixMatrixBinaryOperations(M_rational_function_static_,
                                    M_poly_static_);
  CheckMatrixMatrixBinaryOperations(M_rational_function_static_,
                                    M_poly_dynamic_);
  CheckMatrixMatrixBinaryOperations(M_rational_function_dynamic_,
                                    M_poly_static_);
  CheckMatrixMatrixBinaryOperations(M_rational_function_dynamic_,
                                    M_poly_dynamic_);

  CheckVectorVectorBinaryOperations(v_rational_function_static_,
                                    v_poly_static_);
  CheckVectorVectorBinaryOperations(v_rational_function_static_,
                                    v_poly_dynamic_);
  CheckVectorVectorBinaryOperations(v_rational_function_dynamic_,
                                    v_poly_static_);
  CheckVectorVectorBinaryOperations(v_rational_function_dynamic_,
                                    v_poly_dynamic_);

  CheckMatrixVectorBinaryOperations(M_rational_function_static_,
                                    v_poly_static_);
  CheckMatrixVectorBinaryOperations(M_rational_function_static_,
                                    v_poly_dynamic_);
  CheckMatrixVectorBinaryOperations(M_rational_function_dynamic_,
                                    v_poly_static_);
  CheckMatrixVectorBinaryOperations(M_rational_function_dynamic_,
                                    v_poly_dynamic_);
}

TEST_F(SymbolicRationalFunctionMatrixTest, RationalFunctionOpDouble) {
  CheckMatrixMatrixBinaryOperations(M_rational_function_static_,
                                    M_double_static_);
  CheckMatrixMatrixBinaryOperations(M_rational_function_static_,
                                    M_double_dynamic_);
  CheckMatrixMatrixBinaryOperations(M_rational_function_dynamic_,
                                    M_double_static_);
  CheckMatrixMatrixBinaryOperations(M_rational_function_dynamic_,
                                    M_double_dynamic_);

  CheckVectorVectorBinaryOperations(v_rational_function_static_,
                                    v_double_static_);
  CheckVectorVectorBinaryOperations(v_rational_function_static_,
                                    v_double_dynamic_);
  CheckVectorVectorBinaryOperations(v_rational_function_dynamic_,
                                    v_double_static_);
  CheckVectorVectorBinaryOperations(v_rational_function_dynamic_,
                                    v_double_dynamic_);

  CheckMatrixVectorBinaryOperations(M_rational_function_static_,
                                    v_double_static_);
  CheckMatrixVectorBinaryOperations(M_rational_function_static_,
                                    v_double_dynamic_);
  CheckMatrixVectorBinaryOperations(M_rational_function_dynamic_,
                                    v_double_static_);
  CheckMatrixVectorBinaryOperations(M_rational_function_dynamic_,
                                    v_double_dynamic_);
}
}  // namespace symbolic
}  // namespace drake
