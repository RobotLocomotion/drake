#include "drake/common/symbolic/polynomial.h"

#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/symbolic/monomial_util.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/common/unused.h"

namespace drake {
namespace symbolic {
namespace {
using std::pair;
using std::runtime_error;
using std::to_string;
using std::vector;

using test::ExprEqual;
using test::PolyEqual;

class SymbolicPolynomialTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Variables indeterminates_{var_x_, var_y_, var_z_};

  const Variable var_a_{"a"};
  const Variable var_b_{"b"};
  const Variable var_c_{"c"};

  const Variables var_xy_{var_x_, var_y_};
  const Variables var_xyz_{var_x_, var_y_, var_z_};
  const Variables var_abc_{var_a_, var_b_, var_c_};

  const drake::VectorX<symbolic::Monomial> monomials_{
      MonomialBasis(var_xyz_, 3)};

  const vector<double> doubles_{-9999.0, -5.0, -1.0, 0.0, 1.0, 5.0, 9999.0};

  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};
  const Expression a_{var_a_};
  const Expression b_{var_b_};
  const Expression c_{var_c_};
  const Expression xy_{var_x_ + var_y_};
  const Expression xyz_{var_x_ + var_y_ + var_z_};

  const vector<Expression> exprs_{
      0,
      -1,
      3,
      x_,
      5 * x_,
      -3 * x_,
      y_,
      x_* y_,
      2 * x_* x_,
      2 * x_* x_,
      6 * x_* y_,
      3 * x_* x_* y_ + 4 * pow(y_, 3) * z_ + 2,
      y_*(3 * x_ * x_ + 4 * y_ * y_ * z_) + 2,
      6 * pow(x_, 3) * pow(y_, 2),
      2 * pow(x_, 3) * 3 * pow(y_, 2),
      pow(x_, 3) - 4 * x_* y_* y_ + 2 * x_* x_* y_ - 8 * pow(y_, 3),
      pow(x_ + 2 * y_, 2) * (x_ - 2 * y_),
      (x_ + 2 * y_) * (x_ * x_ - 4 * y_ * y_),
      (x_ * x_ + 4 * x_ * y_ + 4 * y_ * y_) * (x_ - 2 * y_),
      pow(x_ + y_ + 1, 4),
      pow(x_ + y_ + 1, 3),
      1 + x_* x_ + 2 * (y_ - 0.5 * x_ * x_ - 0.5),
      Expression(5.0) / 2.0,     // constant / constant
      x_ / 3.0,                  // var / constant
      pow(x_, 2) / 2,            // pow / constant
      pow(x_* y_ / 3.0, 2) / 2,  // pow / constant
      (x_ + y_) / 2.0,           // sum / constant
      (x_* y_* z_ * 3) / 2.0,    // product / constant
      (x_* y_ / -5.0) / 2.0,     // div / constant
  };
};

// Tests that default constructor and EIGEN_INITIALIZE_MATRICES_BY_ZERO
// constructor both create the same value.
TEST_F(SymbolicPolynomialTest, DefaultConstructors) {
  const Polynomial p;
  EXPECT_TRUE(p.monomial_to_coefficient_map().empty());

  const Polynomial p_zero(0);
  EXPECT_TRUE(p_zero.monomial_to_coefficient_map().empty());
}

TEST_F(SymbolicPolynomialTest, ConstructFromMapType1) {
  Polynomial::MapType map;
  map.emplace(Monomial{var_x_}, -2.0 * a_);          // x ↦ -2a
  map.emplace(Monomial{{{var_y_, 3.0}}}, 4.0 * b_);  // y³ ↦ 4b
  const Polynomial p{map};                           // p = -2ax + 4by³
  EXPECT_EQ(p.monomial_to_coefficient_map(), map);
  EXPECT_EQ(p.ToExpression(), -2 * a_ * x_ + 4 * b_ * pow(y_, 3));
  EXPECT_EQ(p.decision_variables(), Variables({var_a_, var_b_}));
  EXPECT_EQ(p.indeterminates(), Variables({var_x_, var_y_}));
}

TEST_F(SymbolicPolynomialTest, ConstructFromMapType2) {
  Polynomial::MapType p_map;
  for (int i = 0; i < monomials_.size(); ++i) {
    p_map.emplace(monomials_[i], 1);
  }
  EXPECT_EQ(Polynomial{p_map}.monomial_to_coefficient_map(), p_map);
}

TEST_F(SymbolicPolynomialTest, ConstructFromMapType3) {
  Polynomial::MapType map;
  map.emplace(Monomial{var_x_}, -2.0 * a_);          // x ↦ -2a
  map.emplace(Monomial{{{var_a_, 2.0}}}, 4.0 * b_);  // a² ↦ 4b
  // We cannot construct a polynomial from the `map` because variable a is used
  // as a decision variable (x ↦ -2a) and an indeterminate (a² ↦ 4b) at the same
  // time.
  if (kDrakeAssertIsArmed) {
    EXPECT_THROW(Polynomial{map}, runtime_error);
  }
}

TEST_F(SymbolicPolynomialTest, ConstructFromMapType4) {
  // map has some coefficient being 0.
  Polynomial::MapType map;
  map.emplace(Monomial{var_x_}, 0);
  symbolic::Polynomial p{map};
  EXPECT_TRUE(p.monomial_to_coefficient_map().empty());

  map.emplace(Monomial{var_y_}, 2 * a_);
  p = symbolic::Polynomial{map};
  EXPECT_EQ(p.monomial_to_coefficient_map().size(), 1u);
  EXPECT_EQ(p.monomial_to_coefficient_map().at(Monomial{var_y_}), 2 * a_);
}

TEST_F(SymbolicPolynomialTest, ConstructFromMonomial) {
  for (int i = 0; i < monomials_.size(); ++i) {
    const Polynomial p{monomials_[i]};
    for (const std::pair<const Monomial, Expression>& map :
         p.monomial_to_coefficient_map()) {
      EXPECT_EQ(map.first, monomials_[i]);
      EXPECT_EQ(map.second, 1);
    }
  }
}

TEST_F(SymbolicPolynomialTest, ConstructFromExpression) {
  // Expression -------------------> Polynomial
  //     | .Expand()                     | .ToExpression()
  //    \/                              \/
  // Expanded Expression     ==      Expression
  for (const Expression& e : exprs_) {
    const Expression expanded_expr{e.Expand()};
    const Expression expr_from_polynomial{Polynomial{e}.ToExpression()};
    EXPECT_PRED2(ExprEqual, expanded_expr, expr_from_polynomial);
  }
}

TEST_F(SymbolicPolynomialTest, ConstructorFromExpressionAndIndeterminates1) {
  const Polynomial p1{1.0, var_xyz_};  // p₁ = 1.0,
  EXPECT_EQ(p1.monomial_to_coefficient_map(),
            Polynomial::MapType({{Monomial{}, Expression(1.0)}}));
  // p₂ = ax + by + cz + 10
  const Polynomial p2{a_ * x_ + b_ * y_ + c_ * z_ + 10, var_xyz_};
  EXPECT_EQ(p2.monomial_to_coefficient_map(),
            Polynomial::MapType({{Monomial{var_x_}, a_},
                                 {Monomial{var_y_}, b_},
                                 {Monomial{var_z_}, c_},
                                 {Monomial{}, 10}}));
  // p₃ = 3ab²*x²y -bc*z³
  const Polynomial p3{
      3 * a_ * pow(b_, 2) * pow(x_, 2) * y_ - b_ * c_ * pow(z_, 3), var_xyz_};
  EXPECT_EQ(p3.monomial_to_coefficient_map(),
            Polynomial::MapType(
                // x²y ↦ 3ab²
                {{Monomial{{{var_x_, 2}, {var_y_, 1}}}, 3 * a_ * pow(b_, 2)},
                 // z³ ↦ -bc
                 {Monomial{{{var_z_, 3}}}, -b_ * c_}}));

  // p₄ = 3ab²*x²y - bc*x³
  const Polynomial p4{
      3 * a_ * pow(b_, 2) * pow(x_, 2) * y_ - b_ * c_ * pow(x_, 3), var_xyz_};
  EXPECT_EQ(p4.monomial_to_coefficient_map(),
            Polynomial::MapType(
                {{Monomial{{{var_x_, 2}, {var_y_, 1}}}, 3 * a_ * pow(b_, 2)},
                 {Monomial{{{var_x_, 3}}}, -b_ * c_}}));
}

TEST_F(SymbolicPolynomialTest, ConstructorFromExpressionAndIndeterminates2) {
  const Expression e{x_ * x_ + y_ * y_};  // e = x² + y².
  // Show that providing a set of indeterminates {x, y, z} which is a super-set
  // of what appeared in `e`, {x, y}, doesn't change the constructed polynomial
  // .
  const Polynomial p1{e, {var_x_, var_y_}};
  const Polynomial p2{e, {var_x_, var_y_, var_z_}};
  EXPECT_EQ(p1, p2);
}

TEST_F(SymbolicPolynomialTest, ConstructorFromExpressionAndIndeterminates3) {
  // The expression has some coefficient being 0. Show that the constructed
  // polynomial removed the 0-coefficient term.
  const symbolic::Expression e1{0};
  const symbolic::Polynomial p1{e1, {var_x_}};
  EXPECT_TRUE(p1.monomial_to_coefficient_map().empty());
}

TEST_F(SymbolicPolynomialTest, ConstructFromVariable) {
  // Variable -------------------> Polynomial
  //     | Expression()                    | .ToExpression()
  //    \/                                 \/
  // Expanded Expression     ==      Expression
  for (const Variable& v : var_xyz_) {
    const Expression expr{v};
    const Expression expr_from_polynomial{Polynomial{v}.ToExpression()};
    EXPECT_PRED2(ExprEqual, expr, expr_from_polynomial);
  }
}

TEST_F(SymbolicPolynomialTest, IndeterminatesAndDecisionVariables) {
  // p = 3ab²*x²y -bc*z³
  const Polynomial p{
      3 * a_ * pow(b_, 2) * pow(x_, 2) * y_ - b_ * c_ * pow(z_, 3), var_xyz_};
  EXPECT_EQ(p.indeterminates(), var_xyz_);
  EXPECT_EQ(p.decision_variables(), var_abc_);
}

TEST_F(SymbolicPolynomialTest, DegreeAndTotalDegree) {
  // p = 3ab²*x²y -bc*z³
  const Polynomial p{
      3 * a_ * pow(b_, 2) * pow(x_, 2) * y_ - b_ * c_ * pow(z_, 3), var_xyz_};
  EXPECT_EQ(p.Degree(var_x_), 2);
  EXPECT_EQ(p.Degree(var_y_), 1);
  EXPECT_EQ(p.Degree(var_z_), 3);
  EXPECT_EQ(p.TotalDegree(), 3);
}

TEST_F(SymbolicPolynomialTest, AdditionPolynomialPolynomial) {
  // (Polynomial(e₁) + Polynomial(e₂)).ToExpression() = (e₁ + e₂).Expand()
  for (const Expression& e1 : exprs_) {
    for (const Expression& e2 : exprs_) {
      EXPECT_PRED2(ExprEqual, (Polynomial{e1} + Polynomial{e2}).ToExpression(),
                   (e1 + e2).Expand());
    }
  }
  // Test Polynomial& operator+=(Polynomial& c);
  for (const Expression& e1 : exprs_) {
    for (const Expression& e2 : exprs_) {
      Polynomial p{e1};
      p += Polynomial{e2};
      EXPECT_PRED2(ExprEqual, p.ToExpression(), (e1 + e2).Expand());
    }
  }
}

TEST_F(SymbolicPolynomialTest, AdditionPolynomialMonomial) {
  // (Polynomial(e) + m).ToExpression() = (e + m.ToExpression()).Expand()
  // (m + Polynomial(e)).ToExpression() = (m.ToExpression() + e).Expand()
  for (const Expression& e : exprs_) {
    const symbolic::Polynomial p{e};
    for (int i = 0; i < monomials_.size(); ++i) {
      const Monomial& m{monomials_[i]};
      const symbolic::Polynomial sum1 = p + m;
      const symbolic::Polynomial sum2 = m + p;
      EXPECT_PRED2(ExprEqual, sum1.ToExpression(),
                   (e + m.ToExpression()).Expand());
      EXPECT_PRED2(ExprEqual, sum2.ToExpression(),
                   (m.ToExpression() + e).Expand());
      EXPECT_EQ(sum1.indeterminates(), p.indeterminates() + m.GetVariables());
      EXPECT_EQ(sum2.indeterminates(), p.indeterminates() + m.GetVariables());
      EXPECT_EQ(sum1.decision_variables(), p.decision_variables());
      EXPECT_EQ(sum2.decision_variables(), p.decision_variables());
    }
  }
  // Test Polynomial& operator+=(const Monomial& m);
  for (const Expression& e : exprs_) {
    for (int i = 0; i < monomials_.size(); ++i) {
      const Monomial& m{monomials_[i]};
      Polynomial p{e};
      const symbolic::Variables e_indeterminates = p.indeterminates();
      const symbolic::Variables e_decision_variables = p.decision_variables();
      p += m;
      EXPECT_PRED2(ExprEqual, p.ToExpression(),
                   (e + m.ToExpression()).Expand());
      EXPECT_EQ(p.indeterminates(), e_indeterminates + m.GetVariables());
      EXPECT_EQ(p.decision_variables(), e_decision_variables);
    }
  }
}

TEST_F(SymbolicPolynomialTest, AdditionPolynomialDouble) {
  //   (Polynomial(e) + c).ToExpression() = (e + c).Expand()
  //   (c + Polynomial(e)).ToExpression() = (c + e).Expand()
  for (const Expression& e : exprs_) {
    for (const double c : doubles_) {
      EXPECT_PRED2(ExprEqual, (Polynomial(e) + c).ToExpression(),
                   (e + c).Expand());
      EXPECT_PRED2(ExprEqual, (c + Polynomial(e)).ToExpression(),
                   (c + e).Expand());
    }
  }
  // Test Polynomial& operator+=(double c).
  for (const Expression& e : exprs_) {
    for (const double c : doubles_) {
      Polynomial p{e};
      p += c;
      EXPECT_PRED2(ExprEqual, p.ToExpression(), (e + c).Expand());
    }
  }
}

TEST_F(SymbolicPolynomialTest, AdditionMonomialMonomial) {
  // (m1 + m2).ToExpression() = m1.ToExpression() + m2.ToExpression()
  for (int i = 0; i < monomials_.size(); ++i) {
    const Monomial& m_i{monomials_[i]};
    for (int j = 0; j < monomials_.size(); ++j) {
      const Monomial& m_j{monomials_[j]};
      EXPECT_PRED2(ExprEqual, (m_i + m_j).ToExpression(),
                   m_i.ToExpression() + m_j.ToExpression());
    }
  }
}

TEST_F(SymbolicPolynomialTest, AdditionMonomialDouble) {
  // (m + c).ToExpression() = m.ToExpression() + c
  // (c + m).ToExpression() = c + m.ToExpression()
  for (int i = 0; i < monomials_.size(); ++i) {
    const Monomial& m{monomials_[i]};
    for (const double c : doubles_) {
      EXPECT_PRED2(ExprEqual, (m + c).ToExpression(), m.ToExpression() + c);
      EXPECT_PRED2(ExprEqual, (c + m).ToExpression(), c + m.ToExpression());
    }
  }
}

TEST_F(SymbolicPolynomialTest, AdditionPolynomialExpression) {
  // (Polynomial(e₁) + e₂).Expand() = (e₁ + e₂).Expand() = (Polynomial(e₁) +
  // Polynomial(e₂).ToExpression()).Expand()
  for (const Expression& e1 : exprs_) {
    for (const Expression& e2 : exprs_) {
      const Polynomial p1{e1};
      const Polynomial p2{e2};
      EXPECT_PRED2(ExprEqual, (p1 + e2).Expand(), (e1 + e2).Expand());
      EXPECT_PRED2(ExprEqual, (e1 + p2).Expand(), (e1 + e2).Expand());
      // Tests that the return of Polynomial + Expression is the correct
      // Polynomial
      EXPECT_PRED2(PolyEqual, Polynomial{p1 + e2}, p1 + p2);
    }
  }
}

TEST_F(SymbolicPolynomialTest, SubtractionPolynomialPolynomial) {
  // (Polynomial(e₁) - Polynomial(e₂)).ToExpression() = (e₁ - e₂).Expand()
  for (const Expression& e1 : exprs_) {
    for (const Expression& e2 : exprs_) {
      EXPECT_PRED2(ExprEqual, (Polynomial{e1} - Polynomial{e2}).ToExpression(),
                   (e1 - e2).Expand());
    }
  }
  // Test Polynomial& operator-=(Polynomial& c);
  for (const Expression& e1 : exprs_) {
    for (const Expression& e2 : exprs_) {
      Polynomial p{e1};
      p -= Polynomial{e2};
      EXPECT_PRED2(ExprEqual, p.ToExpression(), (e1 - e2).Expand());
    }
  }
}

TEST_F(SymbolicPolynomialTest, SubtractionPolynomialMonomial) {
  // (Polynomial(e) - m).ToExpression() = (e - m.ToExpression()).Expand()
  // (m - Polynomial(e)).ToExpression() = (m.ToExpression() - e).Expand()
  for (const Expression& e : exprs_) {
    for (int i = 0; i < monomials_.size(); ++i) {
      const Monomial& m{monomials_[i]};
      EXPECT_PRED2(ExprEqual, (Polynomial(e) - m).ToExpression(),
                   (e - m.ToExpression()).Expand());
      EXPECT_PRED2(ExprEqual, (m - Polynomial(e)).ToExpression(),
                   (m.ToExpression() - e).Expand());
    }
  }
  // Test Polynomial& operator-=(const Monomial& m);
  for (const Expression& e : exprs_) {
    for (int i = 0; i < monomials_.size(); ++i) {
      const Monomial& m{monomials_[i]};
      Polynomial p{e};
      p -= m;
      EXPECT_PRED2(ExprEqual, p.ToExpression(),
                   (e - m.ToExpression()).Expand());
    }
  }
}

TEST_F(SymbolicPolynomialTest, SubtractionPolynomialDouble) {
  // (Polynomial(e) - c).ToExpression() = (e - c).Expand()
  // (c - Polynomial(e)).ToExpression() = (c - e).Expand()
  for (const Expression& e : exprs_) {
    for (const double c : doubles_) {
      EXPECT_PRED2(ExprEqual, (Polynomial(e) - c).ToExpression(),
                   (e - c).Expand());
      EXPECT_PRED2(ExprEqual, (c - Polynomial(e)).ToExpression(),
                   (c - e).Expand());
    }
  }
  // Test Polynomial& operator-=(double c).
  for (const Expression& e : exprs_) {
    for (const double c : doubles_) {
      Polynomial p{e};
      p -= c;
      EXPECT_PRED2(ExprEqual, p.ToExpression(), (e - c).Expand());
    }
  }
}

TEST_F(SymbolicPolynomialTest, SubtractionMonomialMonomial) {
  // (m1 - m2).ToExpression() = m1.ToExpression() - m2.ToExpression()
  for (int i = 0; i < monomials_.size(); ++i) {
    const Monomial& m_i{monomials_[i]};
    for (int j = 0; j < monomials_.size(); ++j) {
      const Monomial& m_j{monomials_[j]};
      EXPECT_PRED2(ExprEqual, (m_i - m_j).ToExpression(),
                   m_i.ToExpression() - m_j.ToExpression());
    }
  }
}

TEST_F(SymbolicPolynomialTest, SubtractionMonomialDouble) {
  // (m - c).ToExpression() = m.ToExpression() - c
  // (c - m).ToExpression() = c - m.ToExpression()
  for (int i = 0; i < monomials_.size(); ++i) {
    const Monomial& m{monomials_[i]};
    for (const double c : doubles_) {
      EXPECT_PRED2(ExprEqual, (m - c).ToExpression(), m.ToExpression() - c);
      EXPECT_PRED2(ExprEqual, (c - m).ToExpression(), c - m.ToExpression());
    }
  }
}

TEST_F(SymbolicPolynomialTest, SubtractionPolynomialExpression) {
  // (Polynomial(e₁) - e₂).Expand() = (e₁ - e₂).Expand() = (Polynomial(e₁) -
  // Polynomial(e₂).ToExpression()).Expand()
  for (const Expression& e1 : exprs_) {
    for (const Expression& e2 : exprs_) {
      const Polynomial p1{e1};
      const Polynomial p2{e2};
      EXPECT_PRED2(ExprEqual, (p1 - e2).Expand(), (e1 - e2).Expand());
      EXPECT_PRED2(ExprEqual, (e1 - p2).Expand(), (e1 - e2).Expand());
      // Tests that the return of Polynomial - Expression is the correct
      // Polynomial
      EXPECT_PRED2(PolyEqual, Polynomial{p1 - e2}, p1 - p2);
    }
  }
}

TEST_F(SymbolicPolynomialTest, UnaryMinus) {
  // (-Polynomial(e)).ToExpression() = -(e.Expand())
  for (const Expression& e : exprs_) {
    EXPECT_PRED2(ExprEqual, (-Polynomial(e)).ToExpression(), -(e.Expand()));
  }
}

TEST_F(SymbolicPolynomialTest, MultiplicationPolynomialPolynomial1) {
  // (Polynomial(e₁) * Polynomial(e₂)).ToExpression() = (e₁ * e₂).Expand()
  for (const Expression& e1 : exprs_) {
    for (const Expression& e2 : exprs_) {
      const symbolic::Polynomial p1{e1};
      const symbolic::Polynomial p2{e2};
      const symbolic::Polynomial product = p1 * p2;
      EXPECT_PRED2(ExprEqual, product.ToExpression(),
                   (e1.Expand() * e2.Expand()).Expand());
      // After multiplication, the product's indeterminates should be the union
      // of p1's indeterminates and p2's indeterminates. Same for the decision
      // variables.
      EXPECT_EQ(product.indeterminates(),
                p1.indeterminates() + p2.indeterminates());
      EXPECT_EQ(product.decision_variables(),
                p1.decision_variables() + p2.decision_variables());
    }
  }
  // Test Polynomial& operator*=(Polynomial& c);
  for (const Expression& e1 : exprs_) {
    for (const Expression& e2 : exprs_) {
      Polynomial p{e1};
      const symbolic::Variables e1_indeterminates = p.indeterminates();
      const symbolic::Variables e1_decision_variables = p.decision_variables();
      const symbolic::Polynomial p2{e2};
      p *= p2;
      EXPECT_PRED2(ExprEqual, p.ToExpression(),
                   (e1.Expand() * e2.Expand()).Expand());
      EXPECT_EQ(p.indeterminates(), e1_indeterminates + p2.indeterminates());
      EXPECT_EQ(p.decision_variables(),
                e1_decision_variables + p2.decision_variables());
    }
  }
}

TEST_F(SymbolicPolynomialTest, MultiplicationPolynomialMonomial) {
  // (Polynomial(e) * m).ToExpression() = (e * m.ToExpression()).Expand()
  // (m * Polynomial(e)).ToExpression() = (m.ToExpression() * e).Expand()
  for (const Expression& e : exprs_) {
    for (int i = 0; i < monomials_.size(); ++i) {
      const Monomial& m{monomials_[i]};
      const symbolic::Polynomial p{e};
      const symbolic::Polynomial product1 = p * m;
      const symbolic::Polynomial product2 = m * p;
      EXPECT_PRED2(ExprEqual, product1.ToExpression(),
                   (e * m.ToExpression()).Expand());
      EXPECT_PRED2(ExprEqual, product2.ToExpression(),
                   (m.ToExpression() * e).Expand());
      EXPECT_EQ(product1.indeterminates(),
                p.indeterminates() + m.GetVariables());
      EXPECT_EQ(product2.indeterminates(),
                p.indeterminates() + m.GetVariables());
      EXPECT_EQ(product1.decision_variables(), p.decision_variables());
      EXPECT_EQ(product2.decision_variables(), p.decision_variables());
    }
  }
  // Test Polynomial& operator*=(const Monomial& m);
  for (const Expression& e : exprs_) {
    for (int i = 0; i < monomials_.size(); ++i) {
      const Monomial& m{monomials_[i]};
      Polynomial p{e};
      p *= m;
      EXPECT_PRED2(ExprEqual, p.ToExpression(),
                   (e * m.ToExpression()).Expand());
    }
  }
}

TEST_F(SymbolicPolynomialTest, MultiplicationPolynomialDouble) {
  // (Polynomial(e) * c).ToExpression() = (e * c).Expand()
  // (c * Polynomial(e)).ToExpression() = (c * e).Expand()
  for (const Expression& e : exprs_) {
    for (const double c : doubles_) {
      EXPECT_PRED2(ExprEqual, (Polynomial(e) * c).ToExpression(),
                   (e * c).Expand());
      EXPECT_PRED2(ExprEqual, (c * Polynomial(e)).ToExpression(),
                   (c * e).Expand());
    }
  }
  // Test Polynomial& operator*=(double c).
  for (const Expression& e : exprs_) {
    for (const double c : doubles_) {
      Polynomial p{e};
      p *= c;
      EXPECT_PRED2(ExprEqual, p.ToExpression(), (e * c).Expand());
    }

    Polynomial p{e};
    p *= 0;
    EXPECT_TRUE(p.monomial_to_coefficient_map().empty());
  }
}

TEST_F(SymbolicPolynomialTest, MultiplicationMonomialDouble) {
  // (m * c).ToExpression() = (m.ToExpression() * c).Expand()
  // (c * m).ToExpression() = (c * m.ToExpression()).Expand()
  for (int i = 0; i < monomials_.size(); ++i) {
    const Monomial& m{monomials_[i]};
    for (const double c : doubles_) {
      EXPECT_PRED2(ExprEqual, (m * c).ToExpression(),
                   (m.ToExpression() * c).Expand());
      EXPECT_PRED2(ExprEqual, (c * m).ToExpression(),
                   (c * m.ToExpression()).Expand());
    }
  }
}

TEST_F(SymbolicPolynomialTest, MultiplicationPolynomialPolynomial2) {
  // Evaluates (1 + x) * (1 - x) to confirm that the cross term 0 * x is
  // erased from the product.
  const Polynomial p1(1 + x_);
  const Polynomial p2(1 - x_);
  Polynomial::MapType product_map_expected{};
  product_map_expected.emplace(Monomial(), 1);
  product_map_expected.emplace(Monomial(var_x_, 2), -1);
  EXPECT_EQ(product_map_expected, (p1 * p2).monomial_to_coefficient_map());
}

TEST_F(SymbolicPolynomialTest, MultiplicationPolynomialExpression) {
  // (Polynomial(e₁) * e₂).Expand() = (e₁ * e₂).Expand() = (Polynomial(e₁) *
  // Polynomial(e₂).ToExpression()).Expand()
  for (const Expression& e1 : exprs_) {
    for (const Expression& e2 : exprs_) {
      const Polynomial computed_left_product{Polynomial(e1) * e2};
      const Polynomial computed_right_product{e1 * Polynomial(e2)};
      const Polynomial expected{e1 * e2};
      const double tol = 1e-12;
      // Use PolynomialEqual rather than ExprEqual to be able to use floating
      // point rather than exact equality.
      EXPECT_TRUE(test::PolynomialEqual(computed_left_product, expected, tol));
      EXPECT_TRUE(test::PolynomialEqual(computed_right_product, expected, tol));
    }
  }
}

TEST_F(SymbolicPolynomialTest, BinaryOperationBetweenPolynomialAndVariable) {
  // p = 2a²x² + 3ax + 7.
  const Polynomial p{2 * pow(a_, 2) * pow(x_, 2) + 3 * a_ * x_ + 7, {var_x_}};
  const Monomial m_x_cube{var_x_, 3};
  const Monomial m_x_sq{var_x_, 2};
  const Monomial m_x{var_x_, 1};
  const Monomial m_one;

  // Checks addition.
  {
    const Polynomial result1{p + var_a_};
    const Polynomial result2{var_a_ + p};
    // result1 = 2a²x² + 3ax + (7 + a).
    EXPECT_TRUE(result1.EqualTo(result2));
    EXPECT_EQ(result1.monomial_to_coefficient_map().size(), 3);
    EXPECT_EQ(result1.indeterminates(), p.indeterminates());
    EXPECT_PRED2(ExprEqual, result1.monomial_to_coefficient_map().at(m_one),
                 7 + a_);

    const Polynomial result3{p + var_x_};
    const Polynomial result4{var_x_ + p};
    // result3 = 2a²x² + (3a + 1)x + 7.
    EXPECT_TRUE(result3.EqualTo(result4));
    EXPECT_EQ(result3.monomial_to_coefficient_map().size(), 3);
    EXPECT_EQ(result3.indeterminates(), p.indeterminates());
    EXPECT_PRED2(ExprEqual, result3.monomial_to_coefficient_map().at(m_x),
                 3 * a_ + 1);
  }

  // Checks subtraction.
  {
    const Polynomial result1{p - var_a_};
    // result1 = 2a²x² + 3ax + (7 - a).
    EXPECT_EQ(result1.indeterminates(), p.indeterminates());
    EXPECT_EQ(result1.monomial_to_coefficient_map().size(), 3);
    EXPECT_PRED2(ExprEqual, result1.monomial_to_coefficient_map().at(m_one),
                 7 - a_);

    const Polynomial result2{var_a_ - p};
    EXPECT_TRUE((-result2).EqualTo(result1));

    const Polynomial result3{p - var_x_};
    // result3 = 2a²x² + (3a - 1)x + 7.
    EXPECT_EQ(result3.indeterminates(), p.indeterminates());
    EXPECT_EQ(result3.monomial_to_coefficient_map().size(), 3);
    EXPECT_PRED2(ExprEqual, result3.monomial_to_coefficient_map().at(m_x),
                 3 * a_ - 1);

    const Polynomial result4{var_x_ - p};
    EXPECT_TRUE((-result4).EqualTo(result3));
  }

  // Checks multiplication.
  {
    const Polynomial result1{p * var_a_};
    // result1 = 2a³x² + 3a²x + 7a.
    EXPECT_EQ(result1.indeterminates(), p.indeterminates());
    EXPECT_EQ(result1.monomial_to_coefficient_map().size(), 3);
    EXPECT_PRED2(ExprEqual, result1.monomial_to_coefficient_map().at(m_x_sq),
                 2 * pow(a_, 3));
    EXPECT_PRED2(ExprEqual, result1.monomial_to_coefficient_map().at(m_x),
                 3 * pow(a_, 2));
    EXPECT_PRED2(ExprEqual, result1.monomial_to_coefficient_map().at(m_one),
                 7 * a_);

    const Polynomial result2{var_a_ * p};
    EXPECT_TRUE(result2.EqualTo(result1));

    const Polynomial result3{p * var_x_};
    // result3 = 2a²x³ + 3ax² + 7x.
    EXPECT_EQ(result3.indeterminates(), p.indeterminates());
    EXPECT_EQ(result3.monomial_to_coefficient_map().size(), 3);
    EXPECT_PRED2(ExprEqual, result3.monomial_to_coefficient_map().at(m_x_cube),
                 2 * pow(a_, 2));
    EXPECT_PRED2(ExprEqual, result3.monomial_to_coefficient_map().at(m_x_sq),
                 3 * a_);
    EXPECT_PRED2(ExprEqual, result3.monomial_to_coefficient_map().at(m_x), 7);

    const Polynomial result4{var_x_ * p};
    EXPECT_TRUE(result4.EqualTo(result3));

    // var_b_ doesn't show up in p, p * b will have b in the
    // decision_variables(). result5 = 2a²bx²+3abx+7b
    EXPECT_FALSE(p.decision_variables().include(var_b_));
    EXPECT_FALSE(p.indeterminates().include(var_b_));
    const symbolic::Polynomial result5 = var_b_ * p;
    EXPECT_EQ(result5.indeterminates(), symbolic::Variables({var_x_}));
    EXPECT_EQ(result5.decision_variables(),
              symbolic::Variables({var_a_, var_b_}));
    EXPECT_EQ(result5.monomial_to_coefficient_map().size(), 3);
    EXPECT_PRED2(ExprEqual, result5.monomial_to_coefficient_map().at(m_x_sq),
                 2 * pow(a_, 2) * b_);
    EXPECT_PRED2(ExprEqual, result5.monomial_to_coefficient_map().at(m_x),
                 3 * a_ * b_);
    EXPECT_PRED2(ExprEqual, result5.monomial_to_coefficient_map().at(m_one),
                 7 * b_);
  }
}

TEST_F(SymbolicPolynomialTest, Pow) {
  for (int n = 2; n <= 5; ++n) {
    for (const Expression& e : exprs_) {
      Polynomial p{pow(Polynomial{e}, n)};  // p = pow(e, n)
      EXPECT_PRED2(ExprEqual, p.ToExpression(), pow(e, n).Expand());
    }
  }
}

TEST_F(SymbolicPolynomialTest, DivideByConstant) {
  for (double v = -5.5; v <= 5.5; v += 1.0) {
    for (const Expression& e : exprs_) {
      EXPECT_PRED2(ExprEqual, (Polynomial(e) / v).ToExpression(),
                   Polynomial(e / v).ToExpression());
    }
  }
}

TEST_F(SymbolicPolynomialTest, ConstantDividedByPolynomial) {
  for (double v = -5.5; v <= 5.5; v += 1.0) {
    for (const Expression& e : exprs_) {
      if (e.EqualTo(0)) {
        continue;
      }
      EXPECT_PRED2(ExprEqual, (v / Polynomial(e)).Expand(), (v / e).Expand());
    }
  }
}

TEST_F(SymbolicPolynomialTest, DivPolynomialExpression) {
  // (Polynomial(e₁) / e₂).Expand() = (e₁ / e₂).Expand().
  for (const Expression& e1 : exprs_) {
    for (const Expression& e2 : exprs_) {
      if (e2.EqualTo(0)) {
        continue;
      }
      EXPECT_PRED2(ExprEqual, (Polynomial(e1) / e2).Expand(),
                   (e1 / e2).Expand());
      EXPECT_PRED2(ExprEqual, (e1 / Polynomial(e2)).Expand(),
                   (e1 / e2).Expand());
    }
  }
}

TEST_F(SymbolicPolynomialTest, DifferentiateJacobian) {
  // p = 2a²bx² + 3bc²x + 7ac.
  const Polynomial p{
      2 * pow(a_, 2) * b_ * pow(x_, 2) + 3 * b_ * pow(c_, 2) * x_ + 7 * a_ * c_,
      {var_a_, var_b_, var_c_}};

  // d/dx p = 4a²bx + 3bc²
  const Polynomial p_x{4 * pow(a_, 2) * b_ * x_ + 3 * b_ * pow(c_, 2),
                       {var_a_, var_b_, var_c_}};
  EXPECT_PRED2(PolyEqual, p.Differentiate(var_x_), p_x);

  // d/dy p = 0
  const Polynomial p_y{0, {var_a_, var_b_, var_c_}};
  EXPECT_PRED2(PolyEqual, p.Differentiate(var_y_), p_y);

  // d/da p = 4abx² + 7c
  const Polynomial p_a{4 * a_ * b_ * pow(x_, 2) + 7 * c_,
                       {var_a_, var_b_, var_c_}};
  EXPECT_PRED2(PolyEqual, p.Differentiate(var_a_), p_a);

  // d/db p = 2a²x² + 3c²x
  const Polynomial p_b{2 * pow(a_, 2) * pow(x_, 2) + 3 * pow(c_, 2) * x_,
                       {var_a_, var_b_, var_c_}};
  EXPECT_PRED2(PolyEqual, p.Differentiate(var_b_), p_b);

  // d/dc p = 6bcx + 7a
  const Polynomial p_c{6 * b_ * c_ * x_ + 7 * a_, {var_a_, var_b_, var_c_}};
  EXPECT_PRED2(PolyEqual, p.Differentiate(var_c_), p_c);

  // Checks p.Jacobian(x, y) using static-sized matrices.
  Eigen::Matrix<Variable, 2, 1> vars_xy;
  vars_xy << var_x_, var_y_;
  const auto J_xy = p.Jacobian(vars_xy);
  static_assert(decltype(J_xy)::RowsAtCompileTime == 1 &&
                    decltype(J_xy)::ColsAtCompileTime == 2,
                "The size of J_xy should be 1 x 2.");
  EXPECT_PRED2(PolyEqual, J_xy(0, 0), p_x);
  EXPECT_PRED2(PolyEqual, J_xy(0, 1), p_y);

  // Checks p.Jacobian(a, b, c) using dynamic-sized matrices.
  VectorX<Variable> vars_abc(3);
  vars_abc << var_a_, var_b_, var_c_;
  const MatrixX<Polynomial> J_abc{p.Jacobian(vars_abc)};
  EXPECT_PRED2(PolyEqual, J_abc(0, 0), p_a);
  EXPECT_PRED2(PolyEqual, J_abc(0, 1), p_b);
  EXPECT_PRED2(PolyEqual, J_abc(0, 2), p_c);
}

TEST_F(SymbolicPolynomialTest, Integrate) {
  // p = 2a²x²y + 3axy³.
  const Polynomial p{
      2 * pow(a_, 2) * pow(x_, 2) * y_ + 3 * a_ * x_ * pow(y_, 3),
      {var_x_, var_y_}};

  // ∫ p dx = 2/3 a²x³y + 3/2 ax²y³.
  const Polynomial int_p_dx{2 * pow(a_, 2) * pow(x_, 3) * y_ / 3 +
                                3 * a_ * pow(x_, 2) * pow(y_, 3) / 2,
                            {var_x_, var_y_}};
  EXPECT_PRED2(PolyEqual, p.Integrate(var_x_), int_p_dx);

  // ∫ p dx from 1 to 3 = 52/3 a²y + 12 ay³.
  const Polynomial def_int_p_dx{52 * pow(a_, 2) * y_ / 3 + 12 * a_ * pow(y_, 3),
                                {var_y_}};
  EXPECT_PRED2(PolyEqual, p.Integrate(var_x_, 1, 3), def_int_p_dx);
  // ∫ from [a,b] = -∫ from [b,a]
  EXPECT_PRED2(PolyEqual, p.Integrate(var_x_, 3, 1), -def_int_p_dx);

  // ∫ p dy = a²x²y² + 3/4 axy⁴.
  const Polynomial int_p_dy{
      pow(a_, 2) * pow(x_, 2) * pow(y_, 2) + 3 * a_ * x_ * pow(y_, 4) / 4,
      {var_x_, var_y_}};
  EXPECT_PRED2(PolyEqual, p.Integrate(var_y_), int_p_dy);

  // ∫ p dz = 2a²x²yz + 3axy³z.
  const Polynomial int_p_dz{
      2 * pow(a_, 2) * pow(x_, 2) * y_ * z_ + 3 * a_ * x_ * pow(y_, 3) * z_,
      {var_x_, var_y_, var_z_}};
  EXPECT_TRUE(p.Integrate(var_z_).indeterminates().include(var_z_));
  EXPECT_PRED2(PolyEqual, p.Integrate(var_z_), int_p_dz);

  // ∫ p dz from -1 to 1 = 4a²x²y + 6axy³.
  const Polynomial def_int_p_dz{
      4 * pow(a_, 2) * pow(x_, 2) * y_ + 6 * a_ * x_ * pow(y_, 3),
      {var_x_, var_y_, var_z_}};
  EXPECT_TRUE(p.Integrate(var_z_).indeterminates().include(var_z_));
  EXPECT_PRED2(PolyEqual, p.Integrate(var_z_, -1, 1), def_int_p_dz);

  EXPECT_THROW(unused(p.Integrate(var_a_)), std::exception);
  EXPECT_THROW(unused(p.Integrate(var_a_, -1, 1)), std::exception);
}

TEST_F(SymbolicPolynomialTest, ConstructNonPolynomialCoefficients) {
  // Given a pair of Expression and Polynomial::MapType, `(e, map)`, we check
  // `Polynomial(e, indeterminates)` has the expected map, `map`.
  vector<pair<Expression, Polynomial::MapType>> testcases;

  // sin(a)x = sin(a) * x
  testcases.emplace_back(sin(a_) * x_,
                         Polynomial::MapType{{{Monomial{x_}, sin(a_)}}});

  // cos(a)(x + 1)² = cos(a) * x² + 2cos(a) * x + cos(a) * 1
  testcases.emplace_back(
      cos(a_) * pow(x_ + 1, 2),
      Polynomial::MapType{{{Monomial{{{var_x_, 2}}}, cos(a_)},
                           {Monomial{x_}, 2 * cos(a_)},
                           {Monomial{}, cos(a_)}}});

  //   log(a)(x + 1)² / sqrt(b)
  // = log(a)/sqrt(b) * x² + 2log(a)/sqrt(b) * x + log(a)/sqrt(b) * 1
  testcases.emplace_back(
      log(a_) * pow(x_ + 1, 2) / sqrt(b_),
      Polynomial::MapType{{{Monomial{{{var_x_, 2}}}, log(a_) / sqrt(b_)},
                           {Monomial{x_}, 2 * log(a_) / sqrt(b_)},
                           {Monomial{}, log(a_) / sqrt(b_)}}});

  //   (tan(a)x + 1)²
  // = (tan(a))² * x² + 2tan(a) * x + 1
  testcases.emplace_back(
      pow(tan(a_) * x_ + 1, 2),
      Polynomial::MapType{{{Monomial{{{var_x_, 2}}}, pow(tan(a_), 2)},
                           {Monomial{x_}, 2 * tan(a_)},
                           {Monomial{}, 1}}});

  //   abs(b + 1)x + asin(a) + acos(a) - atan(c) * x
  // = (abs(b + 1) - atan(c)) * x + (asin(a) + acos(a))
  testcases.emplace_back(
      abs(b_ + 1) * x_ + asin(a_) + acos(a_) - atan(c_) * x_,
      Polynomial::MapType{{{Monomial{x_}, abs(b_ + 1) - atan(c_)},
                           {Monomial{}, asin(a_) + acos(a_)}}});

  //   atan(b)x * atan2(a, c)y
  // = (atan(b) * atan2(a, c)) * xy
  testcases.emplace_back(
      abs(b_) * x_ * atan2(a_, c_) * y_,
      Polynomial::MapType{{{Monomial{{{var_x_, 1}, {var_y_, 1}}},  // xy
                            abs(b_) * atan2(a_, c_)}}});

  //     (sinh(a)x + cosh(b)y + tanh(c)z) / (5 * min(a, b) * max(b, c))
  // =   (sinh(a) / (5 * min(a, b) * max(b, c))) * x
  //   + (cosh(b) / (5 * min(a, b) * max(b, c))) * y
  //   + (tanh(c) / (5 * min(a, b) * max(b, c))) * z
  testcases.emplace_back(
      (sinh(a_) * x_ + cosh(b_) * y_ + tanh(c_) * z_) /
          (5 * min(a_, b_) * max(b_, c_)),
      Polynomial::MapType{{{
                               Monomial{x_},
                               sinh(a_) / (5 * min(a_, b_) * max(b_, c_)),
                           },
                           {
                               Monomial{y_},
                               cosh(b_) / (5 * min(a_, b_) * max(b_, c_)),
                           },
                           {
                               Monomial{z_},
                               tanh(c_) / (5 * min(a_, b_) * max(b_, c_)),
                           }}});

  //     (ceil(a) * x + floor(b) * y)²
  // =   pow(ceil(a), 2) * x²
  // = + 2 * ceil(a) * floor(b) * xy
  // = + pow(floor(a), 2) * y²
  testcases.emplace_back(
      pow(ceil(a_) * x_ + floor(b_) * y_, 2),
      Polynomial::MapType{
          {{Monomial{{{var_x_, 2}}}, ceil(a_) * ceil(a_)},
           {Monomial{{{var_x_, 1}, {var_y_, 1}}}, 2 * ceil(a_) * floor(b_)},
           {Monomial{{{var_y_, 2}}}, floor(b_) * floor(b_)}}});

  //     (ceil(a) * x + floor(b) * y)²
  // =   pow(ceil(a), 2) * x²
  // = + 2 * ceil(a) * floor(b) * xy
  // = + pow(floor(a), 2) * y²
  testcases.emplace_back(
      pow(ceil(a_) * x_ + floor(b_) * y_, 2),
      Polynomial::MapType{
          {{Monomial{{{var_x_, 2}}}, ceil(a_) * ceil(a_)},
           {Monomial{{{var_x_, 1}, {var_y_, 1}}}, 2 * ceil(a_) * floor(b_)},
           {Monomial{{{var_y_, 2}}}, floor(b_) * floor(b_)}}});

  //   UF("unnamed1", {a})) * x * UF("unnamed2", {b}) * x
  // = UF("unnamed1", {a})) * UF("unnamed2", {b}) * x².
  const Expression uf1{uninterpreted_function("unnamed1", {var_a_})};
  const Expression uf2{uninterpreted_function("unnamed2", {var_b_})};
  testcases.emplace_back(
      uf1 * x_ * uf2 * x_,
      Polynomial::MapType{{{Monomial{{{var_x_, 2}}}, uf1 * uf2}}});

  //   (x + y)² = x² + 2xy + y²
  testcases.emplace_back(pow(x_ + y_, 2),
                         Polynomial::MapType{{
                             {Monomial{{{var_x_, 2}}}, 1},
                             {Monomial{{{var_x_, 1}, {var_y_, 1}}}, 2},
                             {Monomial{{{var_y_, 2}}}, 1},
                         }});

  // pow(pow(x, 2.5), 2) = x⁵
  testcases.emplace_back(pow(pow(x_, 2.5), 2),
                         Polynomial::MapType{{{Monomial{{{var_x_, 5}}}, 1}}});

  // pow(pow(x * y, 2.5), 2) = (xy)⁵
  testcases.emplace_back(
      pow(pow(x_ * y_, 2.5), 2),
      Polynomial::MapType{{{Monomial{{{var_x_, 5}, {var_y_, 5}}}, 1}}});

  for (const pair<Expression, Polynomial::MapType>& item : testcases) {
    const Expression& e{item.first};
    const Polynomial p{e, indeterminates_};
    const Polynomial::MapType& expected_map{item.second};
    EXPECT_EQ(p.monomial_to_coefficient_map(), expected_map);
  }
}

TEST_F(SymbolicPolynomialTest, NegativeTestConstruction1) {
  // sin(a) * x is a polynomial.
  const Expression e1{sin(a_) * x_};
  DRAKE_EXPECT_NO_THROW(Polynomial(e1, indeterminates_));

  // sin(x) * x is a not polynomial.
  const Expression e2{sin(x_) * x_};
  EXPECT_THROW(Polynomial(e2, indeterminates_), runtime_error);
}

TEST_F(SymbolicPolynomialTest, NegativeTestConstruction2) {
  // aˣ x is not a polynomial.
  const Expression e{pow(a_, x_)};
  EXPECT_THROW(Polynomial(e, indeterminates_), runtime_error);
}

TEST_F(SymbolicPolynomialTest, NegativeTestConstruction3) {
  // x⁻¹ is not a polynomial.
  const Expression e{pow(x_, -1)};
  EXPECT_THROW(Polynomial(e, indeterminates_), runtime_error);
}

TEST_F(SymbolicPolynomialTest, NegativeTestConstruction4) {
  // x^(2.5) is not a polynomial.
  const Expression e{pow(x_, 2.5)};
  EXPECT_THROW(Polynomial(e, indeterminates_), runtime_error);
}

TEST_F(SymbolicPolynomialTest, NegativeTestConstruction5) {
  // xˣ is not a polynomial.
  const Expression e{pow(x_, x_)};
  EXPECT_THROW(Polynomial(e, indeterminates_), runtime_error);
}

TEST_F(SymbolicPolynomialTest, NegativeTestConstruction6) {
  // 1 / a is polynomial.
  const Expression e1{1 / a_};
  DRAKE_EXPECT_NO_THROW(Polynomial(e1, indeterminates_));

  // However, 1 / x is not a polynomial.
  const Expression e2{1 / x_};
  EXPECT_THROW(Polynomial(e2, indeterminates_), runtime_error);
}

TEST_F(SymbolicPolynomialTest, NegativeTestConstruction7) {
  // sin(x + a) is not a polynomial.
  const Expression e{sin(x_ + a_)};
  EXPECT_THROW(Polynomial(e, indeterminates_), runtime_error);
}

TEST_F(SymbolicPolynomialTest, Evaluate) {
  // p = ax²y + bxy + cz
  const Polynomial p{a_ * x_ * x_ * y_ + b_ * x_ * y_ + c_ * z_, var_xyz_};

  const Environment env1{{
      {var_a_, 1.0},
      {var_b_, 2.0},
      {var_c_, 3.0},
      {var_x_, -1.0},
      {var_y_, -2.0},
      {var_z_, -3.0},
  }};
  const double expected1{1.0 * -1.0 * -1.0 * -2.0 + 2.0 * -1.0 * -2.0 +
                         3.0 * -3.0};
  EXPECT_EQ(p.Evaluate(env1), expected1);

  const Environment env2{{
      {var_a_, 4.0},
      {var_b_, 1.0},
      {var_c_, 2.0},
      {var_x_, -7.0},
      {var_y_, -5.0},
      {var_z_, -2.0},
  }};
  const double expected2{4.0 * -7.0 * -7.0 * -5.0 + 1.0 * -7.0 * -5.0 +
                         2.0 * -2.0};
  EXPECT_EQ(p.Evaluate(env2), expected2);

  const Environment partial_env{{
      {var_a_, 4.0},
      {var_c_, 2.0},
      {var_x_, -7.0},
      {var_z_, -2.0},
  }};
  EXPECT_THROW(unused(p.Evaluate(partial_env)), runtime_error);
}

TEST_F(SymbolicPolynomialTest, PartialEvaluate1) {
  // p1 = a*x² + b*x + c
  // p2 = p1[x ↦ 3.0] = 3²a + 3b + c.
  const Polynomial p1{a_ * x_ * x_ + b_ * x_ + c_, var_xyz_};
  const Polynomial p2{a_ * 3.0 * 3.0 + b_ * 3.0 + c_, var_xyz_};
  const Environment env{{{var_x_, 3.0}}};
  EXPECT_PRED2(PolyEqual, p1.EvaluatePartial(env), p2);
  EXPECT_PRED2(PolyEqual, p1.EvaluatePartial(var_x_, 3.0), p2);
}

TEST_F(SymbolicPolynomialTest, PartialEvaluate2) {
  // p1 = a*xy² - a*xy + c
  // p2 = p1[y ↦ 2.0] = (4a - 2a)*x + c = 2ax + c
  const Polynomial p1{a_ * x_ * y_ * y_ - a_ * x_ * y_ + c_, var_xyz_};
  const Polynomial p2{2 * a_ * x_ + c_, var_xyz_};
  const Environment env{{{var_y_, 2.0}}};
  EXPECT_PRED2(PolyEqual, p1.EvaluatePartial(env), p2);
  EXPECT_PRED2(PolyEqual, p1.EvaluatePartial(var_y_, 2.0), p2);
}

TEST_F(SymbolicPolynomialTest, PartialEvaluate3) {
  // p1 = a*x² + b*x + c
  // p2 = p1[a ↦ 2.0, x ↦ 3.0] = 2*3² + 3b + c
  //                           = 18 + 3b + c
  const Polynomial p1{a_ * x_ * x_ + b_ * x_ + c_, var_xyz_};
  const Polynomial p2{18 + 3 * b_ + c_, var_xyz_};
  const Environment env{{{var_a_, 2.0}, {var_x_, 3.0}}};
  EXPECT_PRED2(PolyEqual, p1.EvaluatePartial(env), p2);
}

TEST_F(SymbolicPolynomialTest, PartialEvaluate4) {
  // p = (a + c / b + c)*x² + b*x + c
  //
  // Partially evaluating p with [a ↦ 0, b ↦ 0, c ↦ 0] throws `runtime_error`
  // because of the divide-by-zero
  const Polynomial p{((a_ + c_) / (b_ + c_)) * x_ * x_ + b_ * x_ + c_,
                     var_xyz_};
  const Environment env{{{var_a_, 0.0}, {var_b_, 0.0}, {var_c_, 0.0}}};
  EXPECT_THROW(unused(p.EvaluatePartial(env)), runtime_error);
}

TEST_F(SymbolicPolynomialTest, EvaluateIndeterminates) {
  // Test EvaluateIndeterminates where polynomial.decision_variables() is empty.
  const Polynomial p(var_x_ * var_x_ + 5 * var_x_ * var_y_);
  // We intentionally test the case that `indeterminates`  being a strict
  // superset of p.indeterminates().
  Vector3<symbolic::Variable> indeterminates(var_x_, var_z_, var_y_);
  Eigen::Matrix<double, 3, 4> indeterminates_values;
  // clang-format off
  indeterminates_values << 1, 2, 3, 4,
                           -1, -2, -3, -4,
                           2, 3, 4, 5;
  // clang-format on
  Eigen::VectorXd polynomial_values =
      p.EvaluateIndeterminates(indeterminates, indeterminates_values);
  ASSERT_EQ(polynomial_values.rows(), indeterminates_values.cols());
  for (int i = 0; i < indeterminates_values.cols(); ++i) {
    symbolic::Environment env;
    env.insert(indeterminates, indeterminates_values.col(i));
    EXPECT_EQ(polynomial_values(i), p.Evaluate(env));
  }

  // The exception case, when the polynomial coefficients are not all constant;
  const Polynomial p_exception(var_x_ * var_x_ * var_a_, var_xyz_);
  DRAKE_EXPECT_THROWS_MESSAGE(
      p_exception.EvaluateIndeterminates(indeterminates, indeterminates_values),
      ".* the coefficient .* is not a constant");
}

TEST_F(SymbolicPolynomialTest, EvaluateWithAffineCoefficients) {
  const Vector2<symbolic::Variable> indeterminates(var_x_, var_y_);
  Eigen::Matrix<double, 2, 3> indeterminates_values;
  // clang-format off
  indeterminates_values << 1, 2, 3,
                           4, 5, 6;
  // clang-format on
  Eigen::MatrixXd A;
  VectorX<symbolic::Variable> decision_variables;
  Eigen::VectorXd b;
  {
    const Polynomial p((a_ + 1) * var_x_ * var_x_ + b_ * var_x_ * var_y_ + c_,
                       var_xy_);
    p.EvaluateWithAffineCoefficients(indeterminates, indeterminates_values, &A,
                                     &decision_variables, &b);
    EXPECT_EQ(A.rows(), indeterminates_values.cols());
    EXPECT_EQ(b.rows(), indeterminates_values.cols());
    EXPECT_EQ(decision_variables.rows(), 3);
    for (int i = 0; i < indeterminates.cols(); ++i) {
      symbolic::Environment env;
      env.insert(indeterminates, indeterminates_values.col(i));
      EXPECT_PRED2(ExprEqual, (p.ToExpression().EvaluatePartial(env)).Expand(),
                   (A.row(i).dot(decision_variables) + b(i)).Expand());
    }
  }

  // Test special case. When the coefficients of p(x) are all constants.
  {
    const symbolic::Polynomial p(var_x_ * var_x_ + 2 * var_y_);
    p.EvaluateWithAffineCoefficients(indeterminates, indeterminates_values, &A,
                                     &decision_variables, &b);
    EXPECT_EQ(A.cols(), 0);
    EXPECT_EQ(decision_variables.rows(), 0);
    for (int i = 0; i < indeterminates_values.cols(); ++i) {
      symbolic::Environment env;
      env.insert(indeterminates, indeterminates_values.col(i));
      EXPECT_EQ(b(i), p.Evaluate(env));
    }
  }

  // Test exception, when p(x)'s coefficient is not a polynomial expression of
  // decision variables.
  {
    const symbolic::Polynomial p(1 + sin(var_a_) * var_x_ + var_y_, var_xy_);
    DRAKE_EXPECT_THROWS_MESSAGE(
        p.EvaluateWithAffineCoefficients(indeterminates, indeterminates_values,
                                         &A, &decision_variables, &b),
        ".*is not a polynomial.\n");
  }
  // Test exception, when p(x)'s coefficient is not an affine expression of
  // decision variables.
  {
    const symbolic::Polynomial p(1 + (a_ * a_) * var_x_ + var_y_, var_xy_);
    DRAKE_EXPECT_THROWS_MESSAGE(
        p.EvaluateWithAffineCoefficients(indeterminates, indeterminates_values,
                                         &A, &decision_variables, &b),
        ".* is non-linear.*");
  }
}

TEST_F(SymbolicPolynomialTest, SubstituteAndExpandTest) {
  Polynomial::SubstituteAndExpandCacheData substitutions_cached_data;
  std::unordered_map<Variable, Polynomial> indeterminate_substitution;

  // A simple substitution.
  indeterminate_substitution.emplace(var_x_, Polynomial(2 * var_a_));
  // A substitution from one variable to a 2 indeterminate polynomial.
  indeterminate_substitution.emplace(var_y_,
                                     Polynomial(var_a_ * var_b_ + var_a_));
  // A substitution with powers of a variable
  indeterminate_substitution.emplace(
      var_z_, Polynomial(pow(var_a_, 3) + 2 * var_a_ + 1));

  // A polynomial with only linear monomials.
  const Polynomial poly0{x_ + y_};
  const Polynomial sub0 = poly0.SubstituteAndExpand(indeterminate_substitution,
                                                    &substitutions_cached_data);
  const Polynomial sub0_expected{3 * a_ + a_ * b_};
  EXPECT_TRUE(sub0.EqualTo(sub0_expected));
  // We expect {1: 1, x: 2a, y: ab+a }
  EXPECT_EQ(substitutions_cached_data.get_data()->size(), 3);
  EXPECT_TRUE(substitutions_cached_data.get_data()
                  ->at(Monomial())
                  .EqualTo(Polynomial(1)));

  const Polynomial poly1{2 * x_ * x_};
  const Polynomial sub1 = poly1.SubstituteAndExpand(indeterminate_substitution,
                                                    &substitutions_cached_data);
  const Polynomial sub1_expected{8 * a_ * a_};
  EXPECT_TRUE(sub1.EqualTo(sub1_expected));
  // Expect {1: 1, x: 2a, y: ab+a, x²: 4a²}.
  EXPECT_EQ(substitutions_cached_data.get_data()->size(), 4);
  EXPECT_TRUE(substitutions_cached_data.get_data()
                  ->at(Monomial(x_ * x_))
                  .EqualTo(Polynomial(4 * a_ * a_)));

  const Polynomial poly2{x_ * x_ * y_};
  const Polynomial sub2 = poly2.SubstituteAndExpand(indeterminate_substitution,
                                                    &substitutions_cached_data);
  const Polynomial sub2_expected{4 * pow(a_, 3) * (b_ + 1)};
  EXPECT_TRUE(sub2.EqualTo(sub2_expected));
  // Expect {1: 1, x: 2a, y: ab+a, x²: 4a²,  x²y: 8a³(b+1)}.
  EXPECT_EQ(substitutions_cached_data.get_data()->size(), 5);
  // Make sure this hasn't changed
  EXPECT_TRUE(substitutions_cached_data.get_data()
                  ->at(Monomial(x_ * x_))
                  .EqualTo(Polynomial(4 * a_ * a_)));
  EXPECT_TRUE(substitutions_cached_data.get_data()
                  ->at(Monomial(x_ * x_ * y_))
                  .EqualTo(Polynomial(4 * pow(a_, 3) * (b_ + 1))));

  const Polynomial poly3{pow(x_, 3) * z_};
  const Polynomial sub3 = poly3.SubstituteAndExpand(indeterminate_substitution,
                                                    &substitutions_cached_data);
  const Polynomial sub3_expected{pow(2 * a_, 3) *
                                 (pow(var_a_, 3) + 2 * var_a_ + 1)};
  EXPECT_TRUE(sub3.EqualTo(sub3_expected));
  // Expect {1: 1, x: 2a, y: ab+a, x²: 4a²,  x²y: 4a², xz: 2a + 4a² + 2a⁴, x³z:
  // 2a⁹+4a⁷+2a⁶}
  EXPECT_EQ(substitutions_cached_data.get_data()->size(), 8);
  EXPECT_TRUE(substitutions_cached_data.get_data()
                  ->at(Monomial(pow(x_, 3) * z_))
                  .EqualTo(sub3_expected));

  indeterminate_substitution.clear();
  const Polynomial::MapType a_poly_map{{Monomial(a_), pow(cos(z_) + 1, 2)}};
  const Polynomial a_poly{a_poly_map};
  indeterminate_substitution.emplace(var_x_, a_poly);

  const Polynomial poly4{x_};
  const Polynomial sub4 = poly4.SubstituteAndExpand(indeterminate_substitution);

  const Polynomial::MapType sub4_expected_map{
      {Monomial(a_), pow(cos(z_), 2) + 2 * cos(z_) + 1}};
  const Polynomial sub4_expected{sub4_expected_map};
  EXPECT_TRUE(sub4.EqualTo(sub4_expected));
}

TEST_F(SymbolicPolynomialTest, Hash) {
  const auto h = std::hash<Polynomial>{};
  Polynomial p1{x_ * x_};
  const Polynomial p2{x_ * x_};
  EXPECT_EQ(p1, p2);
  EXPECT_EQ(h(p1), h(p2));
  p1 += Polynomial{y_};
  EXPECT_NE(p1, p2);
  EXPECT_NE(h(p1), h(p2));
}

TEST_F(SymbolicPolynomialTest, CoefficientsAlmostEqual) {
  Polynomial p1{x_ * x_};
  // Two polynomials with the same number of terms.
  EXPECT_TRUE(p1.CoefficientsAlmostEqual(Polynomial{x_ * x_}, 1e-6));
  EXPECT_TRUE(
      p1.CoefficientsAlmostEqual(Polynomial{(1 + 1e-7) * x_ * x_}, 1e-6));
  EXPECT_FALSE(p1.CoefficientsAlmostEqual(Polynomial{2 * x_ * x_}, 1e-6));
  // Another polynomial with an additional small constant term.
  EXPECT_TRUE(p1.CoefficientsAlmostEqual(Polynomial{x_ * x_ + 1e-7}, 1e-6));
  EXPECT_FALSE(p1.CoefficientsAlmostEqual(Polynomial{x_ * x_ + 2e-6}, 1e-6));
  // Another polynomial with small difference on coefficients.
  EXPECT_TRUE(p1.CoefficientsAlmostEqual(
      Polynomial{(1. - 1e-7) * x_ * x_ + 1e-7}, 1e-6));
  EXPECT_FALSE(p1.CoefficientsAlmostEqual(
      Polynomial{(1. + 2e-6) * x_ * x_ + 1e-7}, 1e-6));

  // Another polynomial with decision variables in the coefficient.
  const symbolic::Polynomial p2(a_ * x_ * x_, {indeterminates_});
  EXPECT_TRUE(p2.CoefficientsAlmostEqual(
      Polynomial{(a_ + 1e-7) * x_ * x_, {indeterminates_}}, 1e-6));
  EXPECT_FALSE(p2.CoefficientsAlmostEqual(
      Polynomial{(a_ + 1e-7) * x_ * x_, {indeterminates_}}, 1e-8));
}

TEST_F(SymbolicPolynomialTest, RemoveTermsWithSmallCoefficients) {
  // Single term.
  Polynomial p1{1e-5 * x_ * x_};
  EXPECT_PRED2(PolyEqual, p1.RemoveTermsWithSmallCoefficients(1E-4),
               Polynomial(0));
  EXPECT_PRED2(PolyEqual, p1.RemoveTermsWithSmallCoefficients(1E-5),
               Polynomial(0));
  EXPECT_PRED2(PolyEqual, p1.RemoveTermsWithSmallCoefficients(1E-6), p1);

  // Multiple terms.
  Polynomial p2(2 * x_ * x_ + 3 * x_ * y_ + 1E-4 * x_ - 1E-4);
  EXPECT_PRED2(PolyEqual, p2.RemoveTermsWithSmallCoefficients(1E-4),
               Polynomial(2 * x_ * x_ + 3 * x_ * y_));

  // Coefficients are expressions.
  Polynomial::MapType p3_map{};
  p3_map.emplace(Monomial(var_x_, 2), 2 * sin(y_));
  p3_map.emplace(Monomial(var_x_, 1), 1E-4 * cos(y_));
  p3_map.emplace(Monomial(var_x_, 3), 1E-4 * y_);
  p3_map.emplace(Monomial(), 1E-6);
  Polynomial::MapType p3_expected_map{};
  p3_expected_map.emplace(Monomial(var_x_, 2), 2 * sin(y_));
  p3_expected_map.emplace(Monomial(var_x_, 1), 1E-4 * cos(y_));
  p3_expected_map.emplace(Monomial(var_x_, 3), 1E-4 * y_);
  EXPECT_PRED2(PolyEqual,
               Polynomial(p3_map).RemoveTermsWithSmallCoefficients(1E-3),
               Polynomial(p3_expected_map));
}

TEST_F(SymbolicPolynomialTest, EqualTo) {
  const Polynomial p1{var_x_};
  EXPECT_PRED2(PolyEqual, p1, Polynomial(var_x_));
  EXPECT_PRED2(PolyEqual, Polynomial(var_a_ * var_x_, var_xy_),
               Polynomial(var_x_ * var_a_, var_xy_));
  EXPECT_PRED2(test::PolyNotEqual, Polynomial(var_a_ * var_x_, var_xy_),
               Polynomial(var_b_ * var_x_, var_xy_));
}

TEST_F(SymbolicPolynomialTest, EqualToAfterExpansion) {
  const Polynomial p1(2 * var_a_ * var_x_ * var_x_ + var_y_, var_xy_);
  const Polynomial p2(var_x_ * var_x_ + 2 * var_y_, var_xy_);
  const Polynomial p3(2 * var_a_ * var_x_ + var_b_ * var_x_ * var_y_, var_xy_);
  // p2 * p3 * p1 and p1 * p2 * p3 are not structurally equal.
  EXPECT_PRED2(test::PolyNotEqual, p2 * p3 * p1, p1 * p2 * p3);
  // But they are equal after expansion.
  EXPECT_PRED2(test::PolyEqualAfterExpansion, p2 * p3 * p1, p1 * p2 * p3);

  // p1 * p2 is not equal to p2 * p3 after expansion.
  EXPECT_PRED2(test::PolyNotEqualAfterExpansion, p1 * p2, p2 * p3);
}

// Checks if internal::CompareMonomial implements the lexicographical order.
TEST_F(SymbolicPolynomialTest, InternalCompareMonomial) {
  // clang-format off
  const Monomial m1{{{var_x_, 1},                         }};
  const Monomial m2{{{var_x_, 1},              {var_z_, 2}}};
  const Monomial m3{{{var_x_, 1}, {var_y_, 1}             }};
  const Monomial m4{{{var_x_, 1}, {var_y_, 1}, {var_z_, 1}}};
  const Monomial m5{{{var_x_, 1}, {var_y_, 1}, {var_z_, 2}}};
  const Monomial m6{{{var_x_, 1}, {var_y_, 2}, {var_z_, 2}}};
  const Monomial m7{{{var_x_, 1}, {var_y_, 3}, {var_z_, 1}}};
  const Monomial m8{{{var_x_, 2},                         }};
  const Monomial m9{{{var_x_, 2},              {var_z_, 1}}};
  // clang-format on

  EXPECT_TRUE(internal::CompareMonomial()(m1, m2));
  EXPECT_TRUE(internal::CompareMonomial()(m2, m3));
  EXPECT_TRUE(internal::CompareMonomial()(m3, m4));
  EXPECT_TRUE(internal::CompareMonomial()(m4, m5));
  EXPECT_TRUE(internal::CompareMonomial()(m5, m6));
  EXPECT_TRUE(internal::CompareMonomial()(m6, m7));
  EXPECT_TRUE(internal::CompareMonomial()(m7, m8));
  EXPECT_TRUE(internal::CompareMonomial()(m8, m9));

  EXPECT_FALSE(internal::CompareMonomial()(m2, m1));
  EXPECT_FALSE(internal::CompareMonomial()(m3, m2));
  EXPECT_FALSE(internal::CompareMonomial()(m4, m3));
  EXPECT_FALSE(internal::CompareMonomial()(m5, m4));
  EXPECT_FALSE(internal::CompareMonomial()(m6, m5));
  EXPECT_FALSE(internal::CompareMonomial()(m7, m6));
  EXPECT_FALSE(internal::CompareMonomial()(m8, m7));
  EXPECT_FALSE(internal::CompareMonomial()(m9, m8));
}

TEST_F(SymbolicPolynomialTest, DeterministicTraversal) {
  // Using the following monomials, we construct two polynomials; one by summing
  // up from top to bottom and another by summing up from bottom to top. The two
  // polynomials should be the same mathematically. We check that the traversal
  // operations over the two polynomials give the same sequences as well. See
  // https://github.com/RobotLocomotion/drake/issues/11023#issuecomment-499948333
  // for details.

  const Monomial m1{{{var_x_, 1}}};
  const Monomial m2{{{var_x_, 1}, {var_y_, 1}}};
  const Monomial m3{{{var_x_, 1}, {var_y_, 1}, {var_z_, 1}}};
  const Monomial m4{{{var_x_, 1}, {var_y_, 1}, {var_z_, 2}}};

  const Polynomial p1{m1 + (m2 + (m3 + m4))};
  const Polynomial p2{m4 + (m3 + (m2 + m1))};

  const Polynomial::MapType& map1{p1.monomial_to_coefficient_map()};
  const Polynomial::MapType& map2{p2.monomial_to_coefficient_map()};

  EXPECT_EQ(map1.size(), map2.size());

  auto it1 = map1.begin();
  auto it2 = map2.begin();

  for (; it1 != map1.end(); ++it1, ++it2) {
    const Monomial& m_1{it1->first};
    const Monomial& m_2{it2->first};
    const Expression& e_1{it1->second};
    const Expression& e_2{it2->second};
    EXPECT_TRUE(m_1 == m_2);
    EXPECT_TRUE(e_1.EqualTo(e_2));
  }
}

TEST_F(SymbolicPolynomialTest, SetIndeterminates) {
  // ax² + bx + c
  const Expression e{a_ * x_ * x_ + b_ * x_ + c_};

  {
    // {x} -> {x, a}
    Polynomial p{e, {var_x_}};
    const Variables new_indeterminates{var_x_, var_a_};
    p.SetIndeterminates(new_indeterminates);
    EXPECT_PRED2(PolyEqual, p, Polynomial(e, new_indeterminates));
  }

  {
    // {x} -> {x, y}, note that y ∉ variables(e).
    Polynomial p{e, {var_x_}};
    const Variables new_indeterminates{var_x_, var_y_};
    p.SetIndeterminates(new_indeterminates);
    EXPECT_PRED2(PolyEqual, p, Polynomial(e, new_indeterminates));
  }

  {
    // {x, a} -> {x}
    Polynomial p{e, {var_x_, var_a_}};
    const Variables new_indeterminates{var_x_};
    p.SetIndeterminates(new_indeterminates);
    EXPECT_PRED2(PolyEqual, p, Polynomial(e, new_indeterminates));
  }

  {
    // {x, a} -> {a}
    Polynomial p{e, {var_x_, var_a_}};
    const Variables new_indeterminates{var_a_};
    p.SetIndeterminates(new_indeterminates);
    EXPECT_PRED2(PolyEqual, p, Polynomial(e, new_indeterminates));
  }

  {
    // {x, a, b, c} -> {x}
    Polynomial p{e, {var_x_, var_a_, var_b_, var_c_}};
    const Variables new_indeterminates{var_x_};
    p.SetIndeterminates(new_indeterminates);
    EXPECT_PRED2(PolyEqual, p, Polynomial(e, new_indeterminates));
  }
}

TEST_F(SymbolicPolynomialTest, IsEvenOdd) {
  symbolic::Polynomial p{0};
  EXPECT_TRUE(p.IsEven());
  EXPECT_TRUE(p.IsOdd());

  p = symbolic::Polynomial{1};
  EXPECT_TRUE(p.IsEven());
  EXPECT_FALSE(p.IsOdd());

  // p = (a+1)x⁰
  p = symbolic::Polynomial{{{Monomial(), a_ + 1}}};
  EXPECT_TRUE(p.IsEven());
  EXPECT_FALSE(p.IsOdd());

  // p = x
  p = symbolic::Polynomial{{{Monomial(var_x_), 1}}};
  EXPECT_FALSE(p.IsEven());
  EXPECT_TRUE(p.IsOdd());

  // p = x + 1
  p = symbolic::Polynomial{{{Monomial(), 1}, {Monomial(var_x_), 1}}};
  EXPECT_FALSE(p.IsEven());
  EXPECT_FALSE(p.IsOdd());

  // p = (a+2)*x
  p = symbolic::Polynomial{{{Monomial(), 0}, {Monomial(var_x_), a_ + 2}}};
  EXPECT_FALSE(p.IsEven());
  EXPECT_TRUE(p.IsOdd());

  // p = 1 + (a+1)*x*y
  p = symbolic::Polynomial{
      {{Monomial(), 1},
       {Monomial(var_x_), pow(a_, 2) - 1 - (a_ + 1) * (a_ - 1)},
       {Monomial({{var_x_, 1}, {var_y_, 1}}), a_ + 1}}};
  EXPECT_TRUE(p.IsEven());
  EXPECT_FALSE(p.IsOdd());

  // p = a * x  + 2 * x²y
  p = symbolic::Polynomial{
      {{Monomial(), pow(a_, 2) - 1 - (a_ + 1) * (a_ - 1)},
       {Monomial(var_x_), a_},
       {Monomial{{{var_x_, 2}, {var_y_, 1}}}, 2},
       {Monomial{{{var_x_, 2}}}, pow(a_ + 1, 2) - a_ * a_ - 2 * a_ - 1}}};
  EXPECT_FALSE(p.IsEven());
  EXPECT_TRUE(p.IsOdd());
}

TEST_F(SymbolicPolynomialTest, Roots) {
  symbolic::Polynomial p{0};
  DRAKE_EXPECT_THROWS_MESSAGE(p.Roots(), ".* is not a univariate polynomial.*");

  p = symbolic::Polynomial{xy_};
  DRAKE_EXPECT_THROWS_MESSAGE(p.Roots(), ".* is not a univariate polynomial.*");

  p = symbolic::Polynomial{xy_, {var_x_}};
  DRAKE_EXPECT_THROWS_MESSAGE(
      p.Roots(), ".* only supports polynomials with constant coefficients.*");

  p = symbolic::Polynomial{x_ - 1.23};
  Eigen::VectorXcd roots = p.Roots();
  EXPECT_TRUE(CompareMatrices(roots.real(), Vector1d{1.23}, 1e-14));
  EXPECT_TRUE(CompareMatrices(roots.imag(), Vector1d::Zero(), 1e-14));

  // Note: the order of the roots is not guaranteed. Sort them here by real,
  // then imaginary.
  auto sorted = [](const Eigen::VectorXcd& v) {
    Eigen::VectorXcd v_sorted = v;
    std::sort(v_sorted.data(), v_sorted.data() + v_sorted.size(),
              [](const std::complex<double>& a, const std::complex<double>& b) {
                if (a.real() == b.real()) {
                  return a.imag() < b.imag();
                }
                return a.real() < b.real();
              });
    return v_sorted;
  };

  // Include a repeated root.
  p = symbolic::Polynomial{(x_ - 1.23) * (x_ - 4.56) * (x_ - 4.56)};
  roots = sorted(p.Roots());
  EXPECT_TRUE(
      CompareMatrices(roots.real(), Eigen::Vector3d{1.23, 4.56, 4.56}, 1e-7));
  EXPECT_TRUE(CompareMatrices(roots.imag(), Eigen::Vector3d::Zero(), 1e-7));

  // Complex roots.  x^4 - 1 has roots {-1, -i, i, 1}.
  p = symbolic::Polynomial{pow(x_, 4) - 1};
  roots = sorted(p.Roots());
  EXPECT_TRUE(
      CompareMatrices(roots.real(), Eigen::Vector4d{-1, 0, 0, 1}, 1e-7));
  EXPECT_TRUE(
      CompareMatrices(roots.imag(), Eigen::Vector4d{0, -1, 1, 0}, 1e-7));

  // Leading coefficient is not 1.
  p = symbolic::Polynomial{(2.1 * x_ - 1.23) * (x_ - 4.56)};
  roots = sorted(p.Roots());
  EXPECT_TRUE(
      CompareMatrices(roots.real(), Eigen::Vector2d{1.23 / 2.1, 4.56}, 1e-7));
  EXPECT_TRUE(CompareMatrices(roots.imag(), Eigen::Vector2d::Zero(), 1e-7));
}

TEST_F(SymbolicPolynomialTest, Expand) {
  // p1 is already expanded.
  const symbolic::Polynomial p1{
      {{symbolic::Monomial(), a_}, {symbolic::Monomial(var_x_, 2), a_ + 1}}};
  const auto p1_expanded = p1.Expand();
  EXPECT_EQ(p1, p1_expanded);

  // p2 needs expansion.
  using std::pow;
  const symbolic::Polynomial p2{
      {{symbolic::Monomial(), (a_ + 2) * (b_ + 3)},
       {symbolic::Monomial(var_x_, 2), pow(a_ + 1, 2)}}};
  const symbolic::Polynomial p2_expanded = p2.Expand();
  EXPECT_EQ(p2_expanded, symbolic::Polynomial(p2.ToExpression().Expand(),
                                              symbolic::Variables({var_x_})));

  // p3 is 0 after expansion.
  const symbolic::Polynomial p3{
      {{symbolic::Monomial(), (a_ + 1) * (a_ - 1) - pow(a_, 2) + 1}}};
  EXPECT_TRUE(p3.Expand().monomial_to_coefficient_map().empty());
}

TEST_F(SymbolicPolynomialTest, CalcPolynomialWLowerTriangularPart) {
  // Q is a matrix of double.
  const Vector2<symbolic::Monomial> monomial_basis1(
      symbolic::Monomial({{var_x_, 2}, {var_y_, 1}}),
      symbolic::Monomial({{var_x_, 1}}));
  Eigen::Vector3d Q1_lower(1, 2, 3);
  const auto poly1 =
      CalcPolynomialWLowerTriangularPart(monomial_basis1, Q1_lower);
  EXPECT_PRED2(
      test::PolyEqual, poly1,
      symbolic::Polynomial(pow(var_x_, 4) * pow(var_y_, 2) +
                           4 * pow(var_x_, 3) * var_y_ + 3 * var_x_ * var_x_));

  // Q is a matrix of symbolic variable.
  const Vector3<symbolic::Monomial> monomial_basis2(
      symbolic::Monomial({{var_x_, 2}}),
      symbolic::Monomial({{var_x_, 1}, {var_y_, 1}}),
      symbolic::Monomial({{var_y_, 2}}));
  Vector6<symbolic::Variable> Q2_lower;
  for (int i = 0; i < 6; ++i) {
    Q2_lower(i) = symbolic::Variable("Q2_lower" + std::to_string(i));
  }
  const auto poly2 =
      CalcPolynomialWLowerTriangularPart(monomial_basis2, Q2_lower);
  const symbolic::Polynomial poly2_expected = symbolic::Polynomial(
      Q2_lower(0) * pow(var_x_, 4) + 2 * Q2_lower(1) * pow(var_x_, 3) * var_y_ +
          (2 * Q2_lower(2) + Q2_lower(3)) * pow(var_x_, 2) * pow(var_y_, 2) +
          2 * Q2_lower(4) * var_x_ * pow(var_y_, 3) +
          Q2_lower(5) * pow(var_y_, 4),
      var_xy_);
  EXPECT_PRED2(test::PolyEqualAfterExpansion, poly2, poly2_expected);

  // Q is a matrix of expression.
  Vector6<symbolic::Expression> Q3_lower;
  Q3_lower << var_a_, 1, var_a_ + var_b_, 2, 0, var_a_ * var_b_;
  const auto poly3 =
      CalcPolynomialWLowerTriangularPart(monomial_basis2, Q3_lower);
  const symbolic::Polynomial poly3_expected = symbolic::Polynomial(
      var_a_ * pow(var_x_, 4) + 2 * pow(var_x_, 3) * var_y_ +
          (2 * var_a_ + 2 * var_b_ + 2) * pow(var_x_, 2) * pow(var_y_, 2) +
          var_a_ * var_b_ * pow(var_y_, 4),
      var_xy_);
  EXPECT_PRED2(test::PolyEqualAfterExpansion, poly3, poly3_expected);
}

}  // namespace

}  // namespace symbolic
}  // namespace drake
