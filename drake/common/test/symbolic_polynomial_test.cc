#include "drake/common/symbolic_polynomial.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/monomial.h"
#include "drake/common/monomial_util.h"
#include "drake/common/test/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {
using std::vector;
using std::runtime_error;
using std::to_string;

using test::ExprEqual;

class SymbolicPolynomialTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
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

TEST_F(SymbolicPolynomialTest, DefaultConstructor) {
  const Polynomial p{};
  EXPECT_TRUE(p.monomial_to_coefficient_map().empty());
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
  EXPECT_THROW(Polynomial{map}, runtime_error);
}

TEST_F(SymbolicPolynomialTest, ConstructFromMonomial) {
  for (int i = 0; i < monomials_.size(); ++i) {
    const Polynomial p{monomials_[i]};
    for (const std::pair<Monomial, Expression>& map :
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

TEST_F(SymbolicPolynomialTest, ConstructorFromExpressionAndIndeterminates) {
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
    for (int i = 0; i < monomials_.size(); ++i) {
      const Monomial& m{monomials_[i]};
      EXPECT_PRED2(ExprEqual, (Polynomial(e) + m).ToExpression(),
                   (e + m.ToExpression()).Expand());
      EXPECT_PRED2(ExprEqual, (m + Polynomial(e)).ToExpression(),
                   (m.ToExpression() + e).Expand());
    }
  }
  // Test Polynomial& operator+=(const Monomial& m);
  for (const Expression& e : exprs_) {
    for (int i = 0; i < monomials_.size(); ++i) {
      const Monomial& m{monomials_[i]};
      Polynomial p{e};
      p += m;
      EXPECT_PRED2(ExprEqual, p.ToExpression(),
                   (e + m.ToExpression()).Expand());
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
      EXPECT_PRED2(ExprEqual, (Polynomial{e1} * Polynomial{e2}).ToExpression(),
                   (e1.Expand() * e2.Expand()).Expand());
    }
  }
  // Test Polynomial& operator*=(Polynomial& c);
  for (const Expression& e1 : exprs_) {
    for (const Expression& e2 : exprs_) {
      Polynomial p{e1};
      p *= Polynomial{e2};
      EXPECT_PRED2(ExprEqual, p.ToExpression(),
                   (e1.Expand() * e2.Expand()).Expand());
    }
  }
}

TEST_F(SymbolicPolynomialTest, MultiplicationPolynomialMonomial) {
  // (Polynomial(e) * m).ToExpression() = (e * m.ToExpression()).Expand()
  // (m * Polynomial(e)).ToExpression() = (m.ToExpression() * e).Expand()
  for (const Expression& e : exprs_) {
    for (int i = 0; i < monomials_.size(); ++i) {
      const Monomial& m{monomials_[i]};
      EXPECT_PRED2(ExprEqual, (Polynomial(e) * m).ToExpression(),
                   (e * m.ToExpression()).Expand());
      EXPECT_PRED2(ExprEqual, (m * Polynomial(e)).ToExpression(),
                   (m.ToExpression() * e).Expand());
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

TEST_F(SymbolicPolynomialTest, Pow) {
  for (int n = 2; n <= 5; ++n) {
    for (const Expression& e : exprs_) {
      Polynomial p{pow(Polynomial{e}, n)};  // p = pow(e, n)
      EXPECT_PRED2(ExprEqual, p.ToExpression(), pow(e, n).Expand());
    }
  }
}
}  // namespace
}  // namespace symbolic
}  // namespace drake
