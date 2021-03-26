#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/common/unused.h"

namespace drake {
namespace symbolic {
namespace {
using std::pair;
using std::runtime_error;
using std::vector;

using test::ExprEqual;
using test::GenericPolyEqual;

class SymbolicGenericPolynomialTest : public ::testing::Test {
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

  const VectorX<MonomialBasisElement> monomials_{
      ComputePolynomialBasisUpToDegree<Eigen::Dynamic, MonomialBasisElement>(
          var_xyz_, 3, internal::DegreeType::kAny)};
  const VectorX<ChebyshevBasisElement> chebyshev_basis_{
      ComputePolynomialBasisUpToDegree<Eigen::Dynamic, ChebyshevBasisElement>(
          var_xyz_, 3, internal::DegreeType::kAny)};

  const vector<double> doubles_{-9999.0, -5.0, -1.0, 0.0, 1.0, 5.0, 9999.0};

  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};
  const Expression a_{var_a_};
  const Expression b_{var_b_};
  const Expression c_{var_c_};

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
TEST_F(SymbolicGenericPolynomialTest, DefaultConstructors) {
  const GenericPolynomial<MonomialBasisElement> p1;
  EXPECT_TRUE(p1.basis_element_to_coefficient_map().empty());

  const GenericPolynomial<ChebyshevBasisElement> p2;
  EXPECT_TRUE(p2.basis_element_to_coefficient_map().empty());

  const GenericPolynomial<MonomialBasisElement> p3(nullptr);
  EXPECT_TRUE(p3.basis_element_to_coefficient_map().empty());

  const GenericPolynomial<ChebyshevBasisElement> p4(nullptr);
  EXPECT_TRUE(p4.basis_element_to_coefficient_map().empty());
}

TEST_F(SymbolicGenericPolynomialTest, ConstructFromMapType1) {
  GenericPolynomial<ChebyshevBasisElement>::MapType map1;
  map1.emplace(ChebyshevBasisElement{var_x_}, -2.0 * a_);  // T₁(x) → −2a
  map1.emplace(ChebyshevBasisElement{var_y_, 3}, 4.0 * b_);  // T₃(y) → 4b
  // p=−2aT₁(x)+4bT₃(y)
  const GenericPolynomial<ChebyshevBasisElement> p1(map1);
  EXPECT_EQ(p1.basis_element_to_coefficient_map(), map1);
  EXPECT_EQ(p1.decision_variables(), Variables({var_a_, var_b_}));
  EXPECT_EQ(p1.indeterminates(), Variables({var_x_, var_y_}));
}

TEST_F(SymbolicGenericPolynomialTest, ConstructFromMapTypeError) {
  GenericPolynomial<ChebyshevBasisElement>::MapType map;
  map.emplace(ChebyshevBasisElement{var_x_}, -2. * a_);
  map.emplace(ChebyshevBasisElement(var_a_, 2), 4 * b_);
  // We cannot construct a polynomial from `map` because variable a is used as
  // both a decision variable in -2a, and an indeterminate in T₂(a).
  if (kDrakeAssertIsArmed) {
    DRAKE_EXPECT_THROWS_MESSAGE(GenericPolynomial<ChebyshevBasisElement>{map},
                                std::runtime_error,
                                ".* does not satisfy the invariant .*\n.*");
  }
}

TEST_F(SymbolicGenericPolynomialTest, ConstructFromSingleElement) {
  const GenericPolynomial<ChebyshevBasisElement> p1(
      ChebyshevBasisElement({{var_x_, 1}, {var_y_, 2}}));
  EXPECT_EQ(p1.basis_element_to_coefficient_map().size(), 1);
  EXPECT_PRED2(ExprEqual,
               p1.basis_element_to_coefficient_map().at(
                   ChebyshevBasisElement({{var_x_, 1}, {var_y_, 2}})),
               Expression(1));
}

TEST_F(SymbolicGenericPolynomialTest, ConstructFromExpressionMonomialBasis) {
  // p1 = 2x²+3x
  const GenericPolynomial<MonomialBasisElement> p1(var_x_ * var_x_ * 2 +
                                                   3 * var_x_);
  EXPECT_EQ(p1.basis_element_to_coefficient_map().size(), 2);
  EXPECT_PRED2(
      ExprEqual,
      p1.basis_element_to_coefficient_map().at(MonomialBasisElement(var_x_, 2)),
      Expression(2));
  EXPECT_PRED2(
      ExprEqual,
      p1.basis_element_to_coefficient_map().at(MonomialBasisElement(var_x_)),
      Expression(3));

  // p2 = 4xy+5y²+3xz³+2
  const GenericPolynomial<MonomialBasisElement> p2(
      4 * var_x_ * var_y_ + 5 * pow(var_y_, 2) + 3 * var_x_ * pow(var_z_, 3) +
      2);
  EXPECT_EQ(p2.basis_element_to_coefficient_map().size(), 4);
  EXPECT_PRED2(ExprEqual,
               p2.basis_element_to_coefficient_map().at(
                   MonomialBasisElement({{var_x_, 1}, {var_y_, 1}})),
               Expression(4));
  EXPECT_PRED2(
      ExprEqual,
      p2.basis_element_to_coefficient_map().at(MonomialBasisElement(var_y_, 2)),
      Expression(5));
  EXPECT_PRED2(ExprEqual,
               p2.basis_element_to_coefficient_map().at(
                   MonomialBasisElement({{var_x_, 1}, {var_z_, 3}})),
               Expression(3));
  EXPECT_PRED2(ExprEqual,
               p2.basis_element_to_coefficient_map().at(MonomialBasisElement()),
               Expression(2));
}

TEST_F(SymbolicGenericPolynomialTest, ConstructFromExpressionChebyshevBasis) {
  // 1 = T0().
  const GenericPolynomial<ChebyshevBasisElement> p1(Expression(1.));
  EXPECT_EQ(p1.basis_element_to_coefficient_map().size(), 1);
  EXPECT_PRED2(
      ExprEqual,
      p1.basis_element_to_coefficient_map().at(ChebyshevBasisElement()),
      Expression(1.));

  // T₂(x)=2x²−1
  const GenericPolynomial<ChebyshevBasisElement> p2(2 * pow(var_x_, 2) - 1);
  EXPECT_EQ(p2.basis_element_to_coefficient_map().size(), 1);
  EXPECT_PRED2(ExprEqual,
               p2.basis_element_to_coefficient_map().at(
                   ChebyshevBasisElement(var_x_, 2)),
               Expression(1.));

  // T₂(x)T₃(y)=(2x²−1)(4y³−3y)
  const GenericPolynomial<ChebyshevBasisElement> p3(
      (2 * pow(var_x_, 2) - 1) * (4 * pow(var_y_, 3) - 3 * var_y_));
  EXPECT_EQ(p3.basis_element_to_coefficient_map().size(), 1);
  EXPECT_PRED2(ExprEqual,
               p3.basis_element_to_coefficient_map().at(
                   ChebyshevBasisElement({{var_x_, 2}, {var_y_, 3}})),
               Expression(1.));

  // 2T₂(x)T₃(y)+T₁(x)T₂(y)=2(2x²−1)(4y³−3y)+x(2y²−1)
  //                       = 16x²y³−8y³−12x²y+6y+2xy²−x
  const GenericPolynomial<ChebyshevBasisElement> p4(
      16 * pow(var_x_, 2) * pow(var_y_, 3) - 8 * pow(var_y_, 3) -
      12 * pow(var_x_, 2) * var_y_ + 6 * var_y_ + 2 * var_x_ * pow(var_y_, 2) -
      var_x_);
  EXPECT_EQ(p4.basis_element_to_coefficient_map().size(), 2);
  EXPECT_PRED2(ExprEqual,
               p4.basis_element_to_coefficient_map().at(
                   ChebyshevBasisElement({{var_x_, 2}, {var_y_, 3}})),
               Expression(2.));
  EXPECT_PRED2(ExprEqual,
               p4.basis_element_to_coefficient_map().at(
                   ChebyshevBasisElement({{var_x_, 1}, {var_y_, 2}})),
               Expression(1.));

  // 3T₀()+2T₁(x)+4T₂(x)+3T₃(x) = 3+2x+8x²−4+12x³−9x
  //                            = 12x³+8x²−7x−1
  const GenericPolynomial<ChebyshevBasisElement> p5(
      12 * pow(var_x_, 3) + 8 * pow(var_x_, 2) - 7 * var_x_ - 1);
  EXPECT_EQ(p5.basis_element_to_coefficient_map().size(), 4);
  EXPECT_PRED2(
      ExprEqual,
      p5.basis_element_to_coefficient_map().at(ChebyshevBasisElement()),
      Expression(3));
  EXPECT_PRED2(
      ExprEqual,
      p5.basis_element_to_coefficient_map().at(ChebyshevBasisElement(var_x_)),
      Expression(2));
  EXPECT_PRED2(ExprEqual,
               p5.basis_element_to_coefficient_map().at(
                   ChebyshevBasisElement(var_x_, 2)),
               Expression(4));
  EXPECT_PRED2(ExprEqual,
               p5.basis_element_to_coefficient_map().at(
                   ChebyshevBasisElement(var_x_, 3)),
               Expression(3));

  // 2T₃(x)+1=8x³−6x+1
  // Note that although the polynomial contains a term 6x, when represented by
  // Chebyshev basis, it doesn't contain T₁(x) (The coefficient for T₁(x) is 0).
  const GenericPolynomial<ChebyshevBasisElement> p6(8 * pow(var_x_, 3) -
                                                    6 * x_ + 1);
  EXPECT_EQ(p6.basis_element_to_coefficient_map().size(), 2);
  EXPECT_PRED2(ExprEqual,
               p6.basis_element_to_coefficient_map().at(
                   ChebyshevBasisElement(var_x_, 3)),
               Expression(2));
  EXPECT_PRED2(
      ExprEqual,
      p6.basis_element_to_coefficient_map().at(ChebyshevBasisElement()),
      Expression(1));
}

TEST_F(SymbolicGenericPolynomialTest,
       ConstructFromExpressionIndeterminatesMonomialBasis) {
  // Test constructing GenericPolynomial<MonomialBasisElement> from expression
  // and indeterminates

  // e=a²x²+(a+b)xy²+c, indeterminates = {x, y, z}.
  const GenericPolynomial<MonomialBasisElement> p1(
      pow(var_a_, 2) * pow(var_x_, 2) + (a_ + b_) * var_x_ * pow(var_y_, 2) +
          c_,
      indeterminates_);
  EXPECT_EQ(p1.basis_element_to_coefficient_map().size(), 3);
  EXPECT_EQ(p1.indeterminates(), indeterminates_);
  EXPECT_EQ(p1.decision_variables(), Variables({var_a_, var_b_, var_c_}));
  EXPECT_PRED2(
      ExprEqual,
      p1.basis_element_to_coefficient_map().at(MonomialBasisElement(var_x_, 2)),
      pow(var_a_, 2));
  EXPECT_PRED2(ExprEqual,
               p1.basis_element_to_coefficient_map().at(
                   MonomialBasisElement({{var_x_, 1}, {var_y_, 2}})),
               a_ + b_);
  EXPECT_PRED2(ExprEqual,
               p1.basis_element_to_coefficient_map().at(MonomialBasisElement()),
               c_);
}

TEST_F(SymbolicGenericPolynomialTest,
       ConstructFromExpressionIndeterminatesChebyshevBasis) {
  // Test constructing GenericPolynomial<ChebyshevBasisElement> from expression
  // and indeterminates.

  // e=aT₂(x)+(a+b)T₁(x)T₂(y)+cT₀(), indeterminates={x, y, z}.
  const GenericPolynomial<ChebyshevBasisElement> p1(
      a_ * (2 * pow(var_x_, 2) - 1) +
          (a_ + b_) * x_ * (2 * pow(var_y_, 2) - 1) + c_,
      indeterminates_);
  EXPECT_EQ(p1.basis_element_to_coefficient_map().size(), 3);
  EXPECT_EQ(p1.indeterminates(), indeterminates_);
  EXPECT_EQ(p1.decision_variables(), Variables({var_a_, var_b_, var_c_}));
  EXPECT_PRED2(ExprEqual,
               p1.basis_element_to_coefficient_map().at(
                   ChebyshevBasisElement(var_x_, 2)),
               a_);
  EXPECT_PRED2(ExprEqual,
               p1.basis_element_to_coefficient_map()
                   .at(ChebyshevBasisElement({{var_x_, 1}, {var_y_, 2}}))
                   .Expand(),
               a_ + b_);
  EXPECT_PRED2(
      ExprEqual,
      p1.basis_element_to_coefficient_map().at(ChebyshevBasisElement()), c_);
}

TEST_F(SymbolicGenericPolynomialTest,
       ConstructFromExpressionExpandMonomialBasis) {
  // Expression -------------------> Polynomial
  //     | .Expand()                     | .ToExpression()
  //    \/                              \/
  // Expanded Expression     ==      Expression
  for (const Expression& e : exprs_) {
    const Expression expanded_expr{e.Expand()};
    const Expression expr_from_polynomial{
        GenericPolynomial<MonomialBasisElement>{e}.ToExpression()};
    EXPECT_PRED2(ExprEqual, expanded_expr, expr_from_polynomial);
  }
}

TEST_F(SymbolicGenericPolynomialTest,
       ConstructFromExpressionExpandChebyshevBasis) {
  // Expression -------------------> Polynomial
  //     | .Expand()                     | .ToExpression()
  //    \/                              \/
  // Expanded Expression     ==      Expression
  for (const Expression& e : exprs_) {
    const Expression expanded_expr{e.Expand()};
    const Expression expr_from_polynomial{
        GenericPolynomial<ChebyshevBasisElement>{e}.ToExpression()};
    EXPECT_PRED2(ExprEqual, expanded_expr, expr_from_polynomial);
  }
}

TEST_F(SymbolicGenericPolynomialTest,
       ConstructorFromExpressionAndIndeterminates1) {
  const GenericPolynomial<MonomialBasisElement> p1{1.0, var_xyz_};  // p₁ = 1.0,
  EXPECT_EQ(p1.basis_element_to_coefficient_map(),
            GenericPolynomial<MonomialBasisElement>::MapType(
                {{MonomialBasisElement{}, Expression(1.0)}}));
  // p₂ = ax + by + cz + 10
  const GenericPolynomial<MonomialBasisElement> p2{
      a_ * x_ + b_ * y_ + c_ * z_ + 10, var_xyz_};
  EXPECT_EQ(p2.basis_element_to_coefficient_map(),
            GenericPolynomial<MonomialBasisElement>::MapType(
                {{MonomialBasisElement{var_x_}, a_},
                 {MonomialBasisElement{var_y_}, b_},
                 {MonomialBasisElement{var_z_}, c_},
                 {MonomialBasisElement{}, 10}}));
  // p₃ = 3ab²*x²y -bc*z³ + b²
  const GenericPolynomial<MonomialBasisElement> p3{
      3 * a_ * pow(b_, 2) * pow(x_, 2) * y_ - b_ * c_ * pow(z_, 3) + pow(b_, 2),
      var_xyz_};
  EXPECT_EQ(p3.basis_element_to_coefficient_map(),
            GenericPolynomial<MonomialBasisElement>::MapType(
                // x²y ↦ 3ab²
                {{MonomialBasisElement{{{var_x_, 2}, {var_y_, 1}}},
                  3 * a_ * pow(b_, 2)},
                 // z³ ↦ -bc
                 {MonomialBasisElement{{{var_z_, 3}}}, -b_ * c_},
                 // 1 ↦ b²
                 {MonomialBasisElement(), pow(b_, 2)}}));

  // p₄ = 3ab²*x²y - bc*x³
  const GenericPolynomial<MonomialBasisElement> p4{
      3 * a_ * pow(b_, 2) * pow(x_, 2) * y_ - b_ * c_ * pow(x_, 3), var_xyz_};
  EXPECT_EQ(p4.basis_element_to_coefficient_map(),
            GenericPolynomial<MonomialBasisElement>::MapType(
                {{MonomialBasisElement{{{var_x_, 2}, {var_y_, 1}}},
                  3 * a_ * pow(b_, 2)},
                 {MonomialBasisElement{{{var_x_, 3}}}, -b_ * c_}}));

  // p₅ = (ax)³
  const GenericPolynomial<MonomialBasisElement> p5{pow(a_ * x_, 3), var_xyz_};
  EXPECT_EQ(p5.indeterminates(), var_xyz_);
  EXPECT_EQ(p5.basis_element_to_coefficient_map().size(), 1);
  EXPECT_EQ(
      p5.basis_element_to_coefficient_map().at(MonomialBasisElement(var_x_, 3)),
      pow(a_, 3));
}

TEST_F(SymbolicGenericPolynomialTest,
       ConstructorFromExpressionAndIndeterminates2) {
  const Expression e{x_ * x_ + y_ * y_};  // e = x² + y².
  // Show that providing a set of indeterminates {x, y, z} which is a super-set
  // of what appeared in `e`, {x, y}, doesn't change the constructed polynomial.
  const GenericPolynomial<MonomialBasisElement> p1{e, {var_x_, var_y_}};
  const GenericPolynomial<MonomialBasisElement> p2{e, {var_x_, var_y_, var_z_}};
  EXPECT_EQ(p1.basis_element_to_coefficient_map(),
            p2.basis_element_to_coefficient_map());
}

TEST_F(SymbolicGenericPolynomialTest, DegreeAndTotalDegree) {
  const GenericPolynomial<ChebyshevBasisElement> p1(
      {{ChebyshevBasisElement({{var_x_, 1}, {var_y_, 2}}), 3 * var_a_ + var_b_},
       {ChebyshevBasisElement({{var_x_, 4}}), var_a_ * var_b_}});
  EXPECT_EQ(p1.TotalDegree(), 4);
  EXPECT_EQ(p1.Degree(var_x_), 4);
  EXPECT_EQ(p1.Degree(var_y_), 2);
  EXPECT_EQ(p1.Degree(var_z_), 0);
  EXPECT_EQ(p1.Degree(var_a_), 0);
}

// Could use the type_visit trick mentioned in the comment
// https://github.com/RobotLocomotion/drake/pull/14053#pullrequestreview-490104889
// to construct these templated tests.
template <typename BasisElement>
void CheckAdditionPolynomialPolynomial(const std::vector<Expression>& exprs,
                                       double tol) {
  // (Polynomial(e₁) + Polynomial(e₂)) = Polynomial(e₁ + e₂)
  for (const Expression& e1 : exprs) {
    for (const Expression& e2 : exprs) {
      EXPECT_PRED3(test::GenericPolyAlmostEqual<BasisElement>,
                   (GenericPolynomial<BasisElement>{e1} +
                    GenericPolynomial<BasisElement>{e2}),
                   GenericPolynomial<BasisElement>(e1 + e2), tol);
    }
  }
  // Test Polynomial& operator+=(Polynomial& c);
  for (const Expression& e1 : exprs) {
    for (const Expression& e2 : exprs) {
      GenericPolynomial<BasisElement> p{e1};
      p += GenericPolynomial<BasisElement>{e2};
      EXPECT_PRED3(test::GenericPolyAlmostEqual<BasisElement>, p,
                   GenericPolynomial<BasisElement>(e1 + e2, p.indeterminates()),
                   tol);
    }
  }
}

TEST_F(SymbolicGenericPolynomialTest,
       AdditionPolynomialPolynomialMonomialBasis) {
  CheckAdditionPolynomialPolynomial<MonomialBasisElement>(exprs_, 0.);
}

TEST_F(SymbolicGenericPolynomialTest,
       AdditionPolynomialPolynomialChebyshevBasis) {
  CheckAdditionPolynomialPolynomial<ChebyshevBasisElement>(exprs_, 1E-10);
}

template <typename BasisElement>
void CheckAdditionPolynomialBasisElement(
    const std::vector<Expression>& exprs,
    const VectorX<BasisElement>& basis_elements) {
  // (Polynomial(e) + m).ToExpression() = (e + m.ToExpression()).Expand()
  // (m + Polynomial(e)).ToExpression() = (m.ToExpression() + e).Expand()
  for (const Expression& e : exprs) {
    for (int i = 0; i < basis_elements.size(); ++i) {
      const BasisElement& m{basis_elements[i]};
      EXPECT_PRED2(ExprEqual,
                   (GenericPolynomial<BasisElement>(e) + m).ToExpression(),
                   (e + m.ToExpression()).Expand());
      EXPECT_PRED2(ExprEqual,
                   (m + GenericPolynomial<BasisElement>(e)).ToExpression(),
                   (m.ToExpression() + e).Expand());
    }
  }
  // Test Polynomial& operator+=(const Monomial& m);
  for (const Expression& e : exprs) {
    for (int i = 0; i < basis_elements.size(); ++i) {
      const BasisElement& m{basis_elements[i]};
      GenericPolynomial<BasisElement> p{e};
      p += m;
      EXPECT_PRED2(ExprEqual, p.ToExpression(),
                   (e + m.ToExpression()).Expand());
    }
  }
}

TEST_F(SymbolicGenericPolynomialTest, AdditionPolynomialMonomial) {
  CheckAdditionPolynomialBasisElement<MonomialBasisElement>(exprs_, monomials_);
}

TEST_F(SymbolicGenericPolynomialTest, AdditionPolynomialChebyshevBasisElement) {
  CheckAdditionPolynomialBasisElement<ChebyshevBasisElement>(exprs_,
                                                             chebyshev_basis_);
}

template <typename BasisElement>
void CheckAdditionPolynomialDouble(const std::vector<Expression>& exprs,
                                   const std::vector<double>& doubles) {
  //   (Polynomial(e) + c).ToExpression() = (e + c).Expand()
  //   (c + Polynomial(e)).ToExpression() = (c + e).Expand()
  for (const Expression& e : exprs) {
    for (const double c : doubles) {
      EXPECT_PRED2(ExprEqual,
                   (GenericPolynomial<BasisElement>(e) + c).ToExpression(),
                   (e + c).Expand());
      EXPECT_PRED2(ExprEqual,
                   (c + GenericPolynomial<BasisElement>(e)).ToExpression(),
                   (c + e).Expand());
    }
  }
  // Test Polynomial& operator+=(double c).
  for (const Expression& e : exprs) {
    for (const double c : doubles) {
      GenericPolynomial<BasisElement> p{e};
      p += c;
      EXPECT_PRED2(ExprEqual, p.ToExpression(), (e + c).Expand());
    }
  }
}

TEST_F(SymbolicGenericPolynomialTest, AdditionPolynomialDoubleMonomialBasis) {
  CheckAdditionPolynomialDouble<MonomialBasisElement>(exprs_, doubles_);
}

TEST_F(SymbolicGenericPolynomialTest, AdditionPolynomialDoubleChebyshevBasis) {
  CheckAdditionPolynomialDouble<ChebyshevBasisElement>(exprs_, doubles_);
}

template <typename BasisElement>
void CheckAdditionPolynomialVariable(const std::vector<Expression>& exprs,
                                     const Variables& vars) {
  //   (Polynomial(e) + v).ToExpression() = (e + v).Expand()
  //   (v + Polynomial(e)).ToExpression() = (v + e).Expand()
  for (const Expression& e : exprs) {
    for (const Variable& v : vars) {
      EXPECT_PRED2(ExprEqual,
                   (GenericPolynomial<BasisElement>(e) + v).ToExpression(),
                   (e + v).Expand());
      EXPECT_PRED2(ExprEqual,
                   (v + GenericPolynomial<BasisElement>(e)).ToExpression(),
                   (v + e).Expand());
    }
  }
  // Test Polynomial& operator+=(Variable v).
  for (const Expression& e : exprs) {
    for (const Variable& v : vars) {
      GenericPolynomial<BasisElement> p{e};
      p += v;
      EXPECT_PRED2(ExprEqual, p.ToExpression(), (e + v).Expand());
    }
  }
}

TEST_F(SymbolicGenericPolynomialTest, AdditionPolynomialVariableMonomialBasis) {
  CheckAdditionPolynomialVariable<MonomialBasisElement>(exprs_, var_xyz_);
}

TEST_F(SymbolicGenericPolynomialTest,
       AdditionPolynomialVariableChebyshevBasis) {
  CheckAdditionPolynomialVariable<ChebyshevBasisElement>(exprs_, var_xyz_);
}

template <typename BasisElement>
void CheckSubtractionPolynomialPolynomial(const std::vector<Expression>& exprs,
                                          double tol) {
  // (Polynomial(e₁) - Polynomial(e₂)) = Polynomial(e₁ - e₂)
  for (const Expression& e1 : exprs) {
    for (const Expression& e2 : exprs) {
      EXPECT_PRED3(test::GenericPolyAlmostEqual<BasisElement>,
                   GenericPolynomial<BasisElement>{e1} -
                       GenericPolynomial<BasisElement>{e2},
                   GenericPolynomial<BasisElement>(e1 - e2), tol);
    }
  }
  // Test Polynomial& operator-=(Polynomial& c);
  for (const Expression& e1 : exprs) {
    for (const Expression& e2 : exprs) {
      GenericPolynomial<BasisElement> p{e1};
      p -= GenericPolynomial<BasisElement>{e2};
      EXPECT_PRED3(test::GenericPolyAlmostEqual<BasisElement>, p,
                   GenericPolynomial<BasisElement>(e1 - e2), tol);
    }
  }
}

TEST_F(SymbolicGenericPolynomialTest,
       SubtractionPolynomialPolynomialMonomialBasis) {
  CheckSubtractionPolynomialPolynomial<MonomialBasisElement>(exprs_, 0.);
}

TEST_F(SymbolicGenericPolynomialTest,
       SubtractionPolynomialPolynomialChebyshevBasis) {
  CheckSubtractionPolynomialPolynomial<ChebyshevBasisElement>(exprs_, 1E-15);
}

template <typename BasisElement>
void CheckSubtractionPolynomialBasisElement(
    const std::vector<Expression>& exprs,
    const VectorX<BasisElement>& basis_elements) {
  // (Polynomial(e) - m).ToExpression() = (e - m.ToExpression()).Expand()
  // (m - Polynomial(e)).ToExpression() = (m.ToExpression() - e).Expand()
  for (const Expression& e : exprs) {
    for (int i = 0; i < basis_elements.size(); ++i) {
      const BasisElement& m{basis_elements[i]};
      EXPECT_PRED2(ExprEqual,
                   (GenericPolynomial<BasisElement>(e) - m).ToExpression(),
                   (e - m.ToExpression()).Expand());
      EXPECT_PRED2(ExprEqual,
                   (m - GenericPolynomial<BasisElement>(e)).ToExpression(),
                   (m.ToExpression() - e).Expand());
    }
  }
  // Test Polynomial& operator-=(const Monomial& m);
  for (const Expression& e : exprs) {
    for (int i = 0; i < basis_elements.size(); ++i) {
      const BasisElement& m{basis_elements[i]};
      GenericPolynomial<BasisElement> p{e};
      p -= m;
      EXPECT_PRED2(ExprEqual, p.ToExpression(),
                   (e - m.ToExpression()).Expand());
    }
  }
}

TEST_F(SymbolicGenericPolynomialTest, SubtractionPolynomialMonomial) {
  CheckSubtractionPolynomialBasisElement(exprs_, monomials_);
}

TEST_F(SymbolicGenericPolynomialTest, SubtractionPolynomialChebyshev) {
  CheckSubtractionPolynomialBasisElement(exprs_, chebyshev_basis_);
}

template <typename BasisElement>
void CheckSubtractionPolynomialDouble(const std::vector<Expression>& exprs,
                                      const std::vector<double>& doubles) {
  // (Polynomial(e) - c).ToExpression() = (e - c).Expand()
  // (c - Polynomial(e)).ToExpression() = (c - e).Expand()
  for (const Expression& e : exprs) {
    for (const double c : doubles) {
      EXPECT_PRED2(ExprEqual,
                   (GenericPolynomial<BasisElement>(e) - c).ToExpression(),
                   (e - c).Expand());
      EXPECT_PRED2(ExprEqual,
                   (c - GenericPolynomial<BasisElement>(e)).ToExpression(),
                   (c - e).Expand());
    }
  }
  // Test Polynomial& operator-=(double c).
  for (const Expression& e : exprs) {
    for (const double c : doubles) {
      GenericPolynomial<BasisElement> p{e};
      p -= c;
      EXPECT_PRED2(ExprEqual, p.ToExpression(), (e - c).Expand());
    }
  }
}

TEST_F(SymbolicGenericPolynomialTest,
       SubtractionPolynomialDoubleMonomialBasis) {
  CheckSubtractionPolynomialDouble<MonomialBasisElement>(exprs_, doubles_);
}

TEST_F(SymbolicGenericPolynomialTest,
       SubtractionPolynomialDoubleChebyshevBasis) {
  CheckSubtractionPolynomialDouble<ChebyshevBasisElement>(exprs_, doubles_);
}

template <typename BasisElement>
void CheckSubtractionBasisElementBasisElement(
    const VectorX<BasisElement>& basis_elements) {
  // (m1 - m2).ToExpression().Expand() = (m1.ToExpression() -
  // m2.ToExpression()).Expand()
  for (int i = 0; i < basis_elements.size(); ++i) {
    const BasisElement& m_i{basis_elements[i]};
    for (int j = 0; j < basis_elements.size(); ++j) {
      const BasisElement& m_j{basis_elements[j]};
      EXPECT_PRED2(ExprEqual, (m_i - m_j).ToExpression().Expand(),
                   (m_i.ToExpression() - m_j.ToExpression()).Expand());
    }
  }
}

TEST_F(SymbolicGenericPolynomialTest, SubtractionMonomialMonomial) {
  CheckSubtractionBasisElementBasisElement(monomials_);
}

TEST_F(SymbolicGenericPolynomialTest, SubtractionChebyshevChebyshev) {
  CheckSubtractionBasisElementBasisElement(chebyshev_basis_);
}

template <typename BasisElement>
void CheckSubtractionBasisElementDouble(
    const VectorX<BasisElement>& basis_elements,
    const std::vector<double>& doubles) {
  // (m - c).ToExpression() = m.ToExpression() - c
  // (c - m).ToExpression() = c - m.ToExpression()
  for (int i = 0; i < basis_elements.size(); ++i) {
    const BasisElement& m{basis_elements[i]};
    for (const double c : doubles) {
      EXPECT_PRED2(ExprEqual, (m - c).ToExpression().Expand(),
                   (m.ToExpression() - c).Expand());
      EXPECT_PRED2(ExprEqual, (c - m).ToExpression().Expand(),
                   (c - m.ToExpression()).Expand());
    }
  }
}

TEST_F(SymbolicGenericPolynomialTest, SubtractionMonomialDouble) {
  CheckSubtractionBasisElementDouble(monomials_, doubles_);
}

TEST_F(SymbolicGenericPolynomialTest, SubtractionChebyshevDouble) {
  CheckSubtractionBasisElementDouble(chebyshev_basis_, doubles_);
}

template <typename BasisElement>
void CheckUnaryMinus(const std::vector<Expression>& exprs) {
  // (-Polynomial(e)).ToExpression() = -(e.Expand())
  for (const Expression& e : exprs) {
    EXPECT_PRED2(ExprEqual,
                 (-GenericPolynomial<BasisElement>(e)).ToExpression(),
                 -(e.Expand()));
  }
}

TEST_F(SymbolicGenericPolynomialTest, UnaryMinusMonomialBasis) {
  CheckUnaryMinus<MonomialBasisElement>(exprs_);
}

TEST_F(SymbolicGenericPolynomialTest, UnaryMinusChebyshevBasis) {
  CheckUnaryMinus<ChebyshevBasisElement>(exprs_);
}

template <typename BasisElement>
void CheckMultiplicationPolynomialPolynomial(
    const std::vector<Expression>& exprs, double tol) {
  // (Polynomial(e₁) * Polynomial(e₂)) = Polynomial(e₁ * e₂)
  for (const Expression& e1 : exprs) {
    for (const Expression& e2 : exprs) {
      EXPECT_PRED3(test::GenericPolyAlmostEqual<BasisElement>,
                   (GenericPolynomial<BasisElement>{e1} *
                    GenericPolynomial<BasisElement>{e2}),
                   GenericPolynomial<BasisElement>(e1 * e2), tol);
    }
  }
  // Test Polynomial& operator*=(Polynomial& c);
  for (const Expression& e1 : exprs) {
    for (const Expression& e2 : exprs) {
      GenericPolynomial<BasisElement> p{e1};
      p *= GenericPolynomial<BasisElement>{e2};
      EXPECT_PRED3(test::GenericPolyAlmostEqual<BasisElement>, p,
                   GenericPolynomial<BasisElement>(e1 * e2), tol);
    }
  }
}

TEST_F(SymbolicGenericPolynomialTest,
       MultiplicationPolynomialPolynomialMonoialBasis1) {
  CheckMultiplicationPolynomialPolynomial<MonomialBasisElement>(exprs_, 1E-12);
}

TEST_F(SymbolicGenericPolynomialTest,
       MultiplicationPolynomialPolynomialChebyshev1) {
  CheckMultiplicationPolynomialPolynomial<ChebyshevBasisElement>(exprs_, 1E-12);
}

template <typename BasisElement>
void CheckMultiplicationPolynomialBasisElement(
    const std::vector<Expression>& exprs,
    const VectorX<BasisElement>& basis_elements, double tol) {
  // (Polynomial(e) * m) = Polynomial(e * m)
  // (m * Polynomial(e)) = Polynomial(m * e)
  for (const Expression& e : exprs) {
    for (int i = 0; i < basis_elements.size(); ++i) {
      const BasisElement& m{basis_elements[i]};
      EXPECT_PRED3(test::GenericPolyAlmostEqual<BasisElement>,
                   GenericPolynomial<BasisElement>(e) * m,
                   GenericPolynomial<BasisElement>(e * m.ToExpression()), tol);
      EXPECT_PRED3(test::GenericPolyAlmostEqual<BasisElement>,
                   GenericPolynomial<BasisElement>(m.ToExpression() * e),
                   m * GenericPolynomial<BasisElement>(e), tol);
    }
  }
  // Test Polynomial& operator*=(const BasisElement& m);
  for (const Expression& e : exprs) {
    for (int i = 0; i < basis_elements.size(); ++i) {
      const BasisElement& m{basis_elements[i]};
      GenericPolynomial<BasisElement> p{e};
      p *= m;
      EXPECT_PRED3(test::GenericPolyAlmostEqual<BasisElement>, p,
                   GenericPolynomial<BasisElement>(e * m.ToExpression()), tol);
    }
  }
}

TEST_F(SymbolicGenericPolynomialTest, MultiplicationPolynomialMonomial) {
  CheckMultiplicationPolynomialBasisElement<MonomialBasisElement>(
      exprs_, monomials_, 0.);
}

TEST_F(SymbolicGenericPolynomialTest, MultiplicationPolynomialChebyshev) {
  CheckMultiplicationPolynomialBasisElement<ChebyshevBasisElement>(
      exprs_, chebyshev_basis_, 1E-15);
}

template <typename BasisElement>
void CheckMultiplicationPolynomialDouble(const std::vector<Expression>& exprs,
                                         const std::vector<double>& doubles) {
  // (Polynomial(e) * c).ToExpression() = (e * c).Expand()
  // (c * Polynomial(e)).ToExpression() = (c * e).Expand()
  for (const Expression& e : exprs) {
    for (const double c : doubles) {
      EXPECT_PRED2(ExprEqual,
                   (GenericPolynomial<BasisElement>(e) * c).ToExpression(),
                   (e * c).Expand());
      EXPECT_PRED2(ExprEqual,
                   (c * GenericPolynomial<BasisElement>(e)).ToExpression(),
                   (c * e).Expand());
    }
  }
  // Test Polynomial& operator*=(double c).
  for (const Expression& e : exprs) {
    for (const double c : doubles) {
      GenericPolynomial<BasisElement> p{e};
      p *= c;
      EXPECT_PRED2(ExprEqual, p.ToExpression(), (e * c).Expand());
    }
  }
}

TEST_F(SymbolicGenericPolynomialTest,
       MultiplicationPolynomialDoubleMonomialBasis) {
  CheckMultiplicationPolynomialDouble<MonomialBasisElement>(exprs_, doubles_);
}

TEST_F(SymbolicGenericPolynomialTest,
       MultiplicationPolynomialDoubleChebyshevBasis) {
  CheckMultiplicationPolynomialDouble<ChebyshevBasisElement>(exprs_, doubles_);
}

template <typename BasisElement>
void CheckMultiplicationBasisElementDouble(
    const VectorX<BasisElement>& basis_elements,
    const std::vector<double>& doubles) {
  // (m * c).ToExpression() = (m.ToExpression() * c).Expand()
  // (c * m).ToExpression() = (c * m.ToExpression()).Expand()
  for (int i = 0; i < basis_elements.size(); ++i) {
    const BasisElement& m{basis_elements[i]};
    for (const double c : doubles) {
      EXPECT_PRED2(ExprEqual, (m * c).ToExpression(),
                   (m.ToExpression() * c).Expand());
      EXPECT_PRED2(ExprEqual, (c * m).ToExpression(),
                   (c * m.ToExpression()).Expand());
    }
  }
}

TEST_F(SymbolicGenericPolynomialTest, MultiplicationMonomialDouble) {
  CheckMultiplicationBasisElementDouble(monomials_, doubles_);
}

TEST_F(SymbolicGenericPolynomialTest, MultiplicationChebyshevDouble) {
  CheckMultiplicationBasisElementDouble(chebyshev_basis_, doubles_);
}

TEST_F(SymbolicGenericPolynomialTest,
       MultiplicationPolynomialPolynomialMonomialBasis2) {
  // Evaluates (1 + x) * (1 - x) to confirm that the cross term 0 * x is
  // erased from the product.
  const GenericPolynomial<MonomialBasisElement> p1(1 + x_);
  const GenericPolynomial<MonomialBasisElement> p2(1 - x_);
  GenericPolynomial<MonomialBasisElement>::MapType product_map_expected{};
  product_map_expected.emplace(MonomialBasisElement(), 1);
  product_map_expected.emplace(MonomialBasisElement(var_x_, 2), -1);
  EXPECT_EQ(product_map_expected, (p1 * p2).basis_element_to_coefficient_map());
}

TEST_F(SymbolicGenericPolynomialTest,
       MultiplicationPolynomialPolynomialChebyshevBasis2) {
  // Evaluates (1 + x) * (1 - x) to confirm that the cross term 0 * x is
  // erased from the product.
  const GenericPolynomial<ChebyshevBasisElement> p1(1 + x_);
  const GenericPolynomial<ChebyshevBasisElement> p2(1 - x_);
  GenericPolynomial<ChebyshevBasisElement>::MapType product_map_expected{};
  product_map_expected.emplace(ChebyshevBasisElement(), 0.5);
  product_map_expected.emplace(ChebyshevBasisElement(var_x_, 2), -0.5);
  EXPECT_EQ(product_map_expected, (p1 * p2).basis_element_to_coefficient_map());
}

TEST_F(SymbolicGenericPolynomialTest,
       BinaryOperationBetweenPolynomialAndVariableMonomialBasis) {
  // p = 2a²x² + 3ax + 7.
  const GenericPolynomial<MonomialBasisElement> p{
      2 * pow(a_, 2) * pow(x_, 2) + 3 * a_ * x_ + 7, {var_x_}};
  const MonomialBasisElement m_x_cube{var_x_, 3};
  const MonomialBasisElement m_x_sq{var_x_, 2};
  const MonomialBasisElement m_x{var_x_, 1};
  const MonomialBasisElement m_one;

  // Checks addition.
  {
    const GenericPolynomial<MonomialBasisElement> result1{p + var_a_};
    const GenericPolynomial<MonomialBasisElement> result2{var_a_ + p};
    // result1 = 2a²x² + 3ax + (7 + a).
    EXPECT_TRUE(result1.EqualTo(result2));
    EXPECT_EQ(result1.basis_element_to_coefficient_map().size(), 3);
    EXPECT_EQ(result1.indeterminates(), p.indeterminates());
    EXPECT_PRED2(ExprEqual,
                 result1.basis_element_to_coefficient_map().at(m_one), 7 + a_);

    const GenericPolynomial<MonomialBasisElement> result3{p + var_x_};
    const GenericPolynomial<MonomialBasisElement> result4{var_x_ + p};
    // result3 = 2a²x² + (3a + 1)x + 7.
    EXPECT_TRUE(result3.EqualTo(result4));
    EXPECT_EQ(result3.basis_element_to_coefficient_map().size(), 3);
    EXPECT_EQ(result3.indeterminates(), p.indeterminates());
    EXPECT_PRED2(ExprEqual, result3.basis_element_to_coefficient_map().at(m_x),
                 3 * a_ + 1);
  }

  // Checks subtraction.
  {
    const GenericPolynomial<MonomialBasisElement> result1{p - var_a_};
    // result1 = 2a²x² + 3ax + (7 - a).
    EXPECT_EQ(result1.indeterminates(), p.indeterminates());
    EXPECT_EQ(result1.basis_element_to_coefficient_map().size(), 3);
    EXPECT_PRED2(ExprEqual,
                 result1.basis_element_to_coefficient_map().at(m_one), 7 - a_);

    const GenericPolynomial<MonomialBasisElement> result2{var_a_ - p};
    EXPECT_TRUE((-result2).EqualTo(result1));

    const GenericPolynomial<MonomialBasisElement> result3{p - var_x_};
    // result3 = 2a²x² + (3a - 1)x + 7.
    EXPECT_EQ(result3.indeterminates(), p.indeterminates());
    EXPECT_EQ(result3.basis_element_to_coefficient_map().size(), 3);
    EXPECT_PRED2(ExprEqual, result3.basis_element_to_coefficient_map().at(m_x),
                 3 * a_ - 1);

    const GenericPolynomial<MonomialBasisElement> result4{var_x_ - p};
    EXPECT_TRUE((-result4).EqualTo(result3));
  }

  // Checks multiplication.
  {
    const GenericPolynomial<MonomialBasisElement> result1{p * var_a_};
    // result1 = 2a³x² + 3a²x + 7a.
    EXPECT_EQ(result1.indeterminates(), p.indeterminates());
    EXPECT_EQ(result1.basis_element_to_coefficient_map().size(), 3);
    EXPECT_PRED2(ExprEqual,
                 result1.basis_element_to_coefficient_map().at(m_x_sq),
                 2 * pow(a_, 3));
    EXPECT_PRED2(ExprEqual, result1.basis_element_to_coefficient_map().at(m_x),
                 3 * pow(a_, 2));
    EXPECT_PRED2(ExprEqual,
                 result1.basis_element_to_coefficient_map().at(m_one), 7 * a_);

    const GenericPolynomial<MonomialBasisElement> result2{var_a_ * p};
    EXPECT_TRUE(result2.EqualTo(result1));

    const GenericPolynomial<MonomialBasisElement> result3{p * var_x_};
    // result3 = 2a²x³ + 3ax² + 7x.
    EXPECT_EQ(result3.indeterminates(), p.indeterminates());
    EXPECT_EQ(result3.basis_element_to_coefficient_map().size(), 3);
    EXPECT_PRED2(ExprEqual,
                 result3.basis_element_to_coefficient_map().at(m_x_cube),
                 2 * pow(a_, 2));
    EXPECT_PRED2(ExprEqual,
                 result3.basis_element_to_coefficient_map().at(m_x_sq), 3 * a_);
    EXPECT_PRED2(ExprEqual, result3.basis_element_to_coefficient_map().at(m_x),
                 7);

    const GenericPolynomial<MonomialBasisElement> result4{var_x_ * p};
    EXPECT_TRUE(result4.EqualTo(result3));
  }
}

TEST_F(SymbolicGenericPolynomialTest,
       BinaryOperationBetweenPolynomialAndVariableChebyshevBasis) {
  // p = 2a²T₂(x)+3aT₁(x)+7T₀()
  const GenericPolynomial<ChebyshevBasisElement> p{
      {{ChebyshevBasisElement(var_x_, 2), 2 * pow(a_, 2)},
       {ChebyshevBasisElement(var_x_), 3 * a_},
       {ChebyshevBasisElement(), 7}}};

  const ChebyshevBasisElement t_x_cube{var_x_, 3};
  const ChebyshevBasisElement t_x_sq{var_x_, 2};
  const ChebyshevBasisElement t_x{var_x_, 1};
  const ChebyshevBasisElement t_zero;

  // Checks addition.
  {
    const GenericPolynomial<ChebyshevBasisElement> result1{p + var_a_};
    const GenericPolynomial<ChebyshevBasisElement> result2{var_a_ + p};
    // result1 = 2a²T₂(x)+3aT₁(x)+(7+a)T₀()
    EXPECT_TRUE(result1.EqualTo(result2));
    EXPECT_EQ(result1.basis_element_to_coefficient_map().size(), 3);
    EXPECT_EQ(result1.indeterminates(), p.indeterminates());
    EXPECT_PRED2(ExprEqual,
                 result1.basis_element_to_coefficient_map().at(t_zero), 7 + a_);

    const GenericPolynomial<ChebyshevBasisElement> result3{p + var_x_};
    const GenericPolynomial<ChebyshevBasisElement> result4{var_x_ + p};
    // result3 = 2a²T₂(x)+(3a + 1) T₁(x)+7T₀()
    EXPECT_TRUE(result3.EqualTo(result4));
    EXPECT_EQ(result3.basis_element_to_coefficient_map().size(), 3);
    EXPECT_EQ(result3.indeterminates(), p.indeterminates());
    EXPECT_PRED2(ExprEqual, result3.basis_element_to_coefficient_map().at(t_x),
                 3 * a_ + 1);
  }

  // Checks subtraction.
  {
    const GenericPolynomial<ChebyshevBasisElement> result1{p - var_a_};
    // result1 = 2a²T₂(x)+3aT₁(x)+(7-a)T₀()
    EXPECT_EQ(result1.indeterminates(), p.indeterminates());
    EXPECT_EQ(result1.basis_element_to_coefficient_map().size(), 3);
    EXPECT_PRED2(ExprEqual,
                 result1.basis_element_to_coefficient_map().at(t_zero), 7 - a_);

    const GenericPolynomial<ChebyshevBasisElement> result2{var_a_ - p};
    EXPECT_TRUE((-result2).EqualTo(result1));

    const GenericPolynomial<ChebyshevBasisElement> result3{p - var_x_};
    // result3 = 2a²T₂(x)+(3a-1)T₁(x)+7T₀()
    EXPECT_EQ(result3.indeterminates(), p.indeterminates());
    EXPECT_EQ(result3.basis_element_to_coefficient_map().size(), 3);
    EXPECT_PRED2(ExprEqual, result3.basis_element_to_coefficient_map().at(t_x),
                 3 * a_ - 1);

    const GenericPolynomial<ChebyshevBasisElement> result4{var_x_ - p};
    EXPECT_TRUE((-result4).EqualTo(result3));
  }

  // Checks multiplication.
  {
    const GenericPolynomial<ChebyshevBasisElement> result1{p * var_a_};
    // result1 = 2a³T₂(x)+3a²T₁(x)+7aT₀()
    EXPECT_EQ(result1.indeterminates(), p.indeterminates());
    EXPECT_EQ(result1.basis_element_to_coefficient_map().size(), 3);
    EXPECT_PRED2(ExprEqual,
                 result1.basis_element_to_coefficient_map().at(t_x_sq),
                 2 * pow(a_, 3));
    EXPECT_PRED2(ExprEqual, result1.basis_element_to_coefficient_map().at(t_x),
                 3 * pow(a_, 2));
    EXPECT_PRED2(ExprEqual,
                 result1.basis_element_to_coefficient_map().at(t_zero), 7 * a_);

    const GenericPolynomial<ChebyshevBasisElement> result2{var_a_ * p};
    EXPECT_TRUE(result2.EqualTo(result1));

    const GenericPolynomial<ChebyshevBasisElement> result3{p * var_x_};
    // result3 = a²T₃(x)+1.5aT₂(x)+(7+a²)T₁(x)+1.5aT₀()
    EXPECT_EQ(result3.indeterminates(), p.indeterminates());
    EXPECT_EQ(result3.basis_element_to_coefficient_map().size(), 4);
    EXPECT_PRED2(ExprEqual,
                 result3.basis_element_to_coefficient_map().at(t_x_cube),
                 pow(a_, 2));
    EXPECT_PRED2(ExprEqual,
                 result3.basis_element_to_coefficient_map().at(t_x_sq),
                 1.5 * a_);
    EXPECT_PRED2(ExprEqual, result3.basis_element_to_coefficient_map().at(t_x),
                 7 + pow(a_, 2));
    EXPECT_PRED2(ExprEqual,
                 result3.basis_element_to_coefficient_map().at(t_zero),
                 1.5 * a_);

    const GenericPolynomial<ChebyshevBasisElement> result4{var_x_ * p};
    EXPECT_TRUE(result4.EqualTo(result3));
  }
}

template <typename BasisElement>
void TestPow(const std::vector<symbolic::Expression>& exprs, double tol) {
  for (int n = 2; n <= 5; ++n) {
    for (const Expression& e : exprs) {
      const GenericPolynomial<BasisElement> p(e);
      EXPECT_PRED3(test::GenericPolyAlmostEqual<BasisElement>, pow(p, n),
                   GenericPolynomial<BasisElement>(pow(e, n)), tol);
    }
  }
}

TEST_F(SymbolicGenericPolynomialTest, Pow) {
  // Test negative power
  DRAKE_EXPECT_THROWS_MESSAGE(
      pow(GenericPolynomial<MonomialBasisElement>(2 * x_), -1),
      std::runtime_error, ".*the degree should be non-negative.*");
  // Test power of 0 degree.
  auto result = pow(GenericPolynomial<ChebyshevBasisElement>(2 * x_), 0);
  EXPECT_EQ(result.basis_element_to_coefficient_map().size(), 1);
  EXPECT_EQ(
      result.basis_element_to_coefficient_map().at(ChebyshevBasisElement()), 1);

  // Test power of 1 degree.
  result = pow(GenericPolynomial<ChebyshevBasisElement>(2 * x_), 1);
  EXPECT_PRED2(test::GenericPolyEqual<ChebyshevBasisElement>, result,
               GenericPolynomial<ChebyshevBasisElement>(2 * x_));

  // Test higher degrees.
  TestPow<MonomialBasisElement>(exprs_, 1E-14);
  TestPow<ChebyshevBasisElement>(exprs_, 1E-14);
}

template <typename BasisElement>
void CheckDivideByConstant(const std::vector<Expression>& exprs, double tol) {
  for (double v = -5.5; v <= 5.5; v += 1.0) {
    for (const Expression& e : exprs) {
      EXPECT_PRED3(test::GenericPolyAlmostEqual<BasisElement>,
                   GenericPolynomial<BasisElement>(e) / v,
                   GenericPolynomial<BasisElement>(e / v), tol);
    }
  }
}
TEST_F(SymbolicGenericPolynomialTest, DivideByConstant) {
  CheckDivideByConstant<MonomialBasisElement>(exprs_, 0.);
  CheckDivideByConstant<ChebyshevBasisElement>(exprs_, 1E-15);
}

TEST_F(SymbolicGenericPolynomialTest, DifferentiateForMonomialBasis) {
  // p = 2a²bx² + 3bc²x + 7ac.
  const GenericPolynomial<MonomialBasisElement> p{
      {{MonomialBasisElement(var_x_, 2), 2 * pow(a_, 2) * b_},
       {MonomialBasisElement(var_x_), 3 * b_ * pow(c_, 2)},
       {MonomialBasisElement(), 7 * a_ * c_}}};
  // d/dx p = 4a²bx + 3bc²
  const GenericPolynomial<MonomialBasisElement> p_x{
      {{MonomialBasisElement(var_x_), 4 * pow(a_, 2) * b_},
       {MonomialBasisElement(), 3 * b_ * pow(c_, 2)}}};
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, p.Differentiate(var_x_),
               p_x);

  // d/dy p = 0
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, p.Differentiate(var_y_),
               GenericPolynomial<MonomialBasisElement>());

  // d/da p = 4abx² + 7c
  const GenericPolynomial<MonomialBasisElement> p_a{
      {{MonomialBasisElement(var_x_, 2), 4 * a_ * b_},
       {MonomialBasisElement(), 7 * c_}}};
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, p.Differentiate(var_a_),
               p_a);

  // d/db p = 2a²x² + 3c²x
  const GenericPolynomial<MonomialBasisElement> p_b{
      {{MonomialBasisElement(var_x_, 2), 2 * pow(a_, 2)},
       {MonomialBasisElement(var_x_), 3 * pow(c_, 2)}}};
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, p.Differentiate(var_b_),
               p_b);

  // d/dc p = 6bcx + 7a
  const GenericPolynomial<MonomialBasisElement> p_c{
      {{MonomialBasisElement(var_x_), 6 * b_ * c_},
       {MonomialBasisElement(), 7 * a_}}};
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, p.Differentiate(var_c_),
               p_c);
}

TEST_F(SymbolicGenericPolynomialTest, DifferentiateForChebyshevBasis1) {
  // p = 2a²bT₁(x)T₃(y) + 3acT₂(y)
  const GenericPolynomial<ChebyshevBasisElement> p{
      {{ChebyshevBasisElement({{var_x_, 1}, {var_y_, 3}}), 2 * pow(a_, 2) * b_},
       {ChebyshevBasisElement(var_y_, 2), 3 * a_ * c_}}};

  // dp/dx = 2a²bT₃(y)
  const GenericPolynomial<ChebyshevBasisElement> p_x{
      {{ChebyshevBasisElement(var_y_, 3), 2 * pow(a_, 2) * b_}}};
  EXPECT_PRED2(GenericPolyEqual<ChebyshevBasisElement>, p.Differentiate(var_x_),
               p_x);

  // dp/dy = 6a²bT₁(x) + 12a²bT₁(x)T₂(y) + 12acT₁(y)
  const GenericPolynomial<ChebyshevBasisElement> p_y(
      {{ChebyshevBasisElement(var_x_), 6 * pow(a_, 2) * b_},
       {ChebyshevBasisElement({{var_x_, 1}, {var_y_, 2}}),
        12 * pow(a_, 2) * b_},
       {ChebyshevBasisElement(var_y_), 12 * a_ * c_}});
  EXPECT_PRED2(GenericPolyEqual<ChebyshevBasisElement>, p.Differentiate(var_y_),
               p_y);

  // dp/dz = 0
  EXPECT_PRED2(GenericPolyEqual<ChebyshevBasisElement>, p.Differentiate(var_z_),
               GenericPolynomial<ChebyshevBasisElement>(nullptr));

  // dp/da = 4abT₁(x)T₃(y)+3cT₂(y)
  const GenericPolynomial<ChebyshevBasisElement> p_a(
      {{ChebyshevBasisElement({{var_x_, 1}, {var_y_, 3}}), 4 * a_ * b_},
       {ChebyshevBasisElement(var_y_, 2), 3 * c_}});
  EXPECT_PRED2(GenericPolyEqual<ChebyshevBasisElement>, p.Differentiate(var_a_),
               p_a);
}

TEST_F(SymbolicGenericPolynomialTest, DifferentiateForChebyshevBasis2) {
  // p = aT₁(x)+aT₂(x)+a²T₃(x)
  // Notice that dT₁(x)/dx, dT₂(x)/dx and dT₃(x)/dx would all produce results
  // containing T₀(x) (or T₁(x) and T₂(x)).
  const GenericPolynomial<ChebyshevBasisElement> p(
      {{ChebyshevBasisElement(var_x_), a_},
       {ChebyshevBasisElement(var_x_, 2), a_},
       {ChebyshevBasisElement(var_x_, 3), pow(a_, 2)}});

  // dp/dx = (a+3a²)T₀(x) + 4aT₁(x) + 6a²T₂(x)
  const GenericPolynomial<ChebyshevBasisElement> p_x(
      {{ChebyshevBasisElement(nullptr), a_ + 3 * pow(a_, 2)},
       {ChebyshevBasisElement(var_x_), 4 * a_},
       {ChebyshevBasisElement(var_x_, 2), 6 * pow(a_, 2)}});
  EXPECT_PRED2(GenericPolyEqual<ChebyshevBasisElement>, p.Differentiate(var_x_),
               p_x);

  // dp/da =T₁(x)+T₂(x)+2aT₃(x)
  const GenericPolynomial<ChebyshevBasisElement> p_a(
      {{ChebyshevBasisElement(var_x_), 1},
       {ChebyshevBasisElement(var_x_, 2), 1},
       {ChebyshevBasisElement(var_x_, 3), 2 * a_}});
  EXPECT_PRED2(GenericPolyEqual<ChebyshevBasisElement>, p.Differentiate(var_a_),
               p_a);
}

TEST_F(SymbolicGenericPolynomialTest, Jacobian) {
  // p = ax²y + (3a+b)xz²
  const GenericPolynomial<MonomialBasisElement> p{
      {{MonomialBasisElement({{var_x_, 2}, {var_y_, 1}}), a_},
       {MonomialBasisElement({{var_x_, 1}, {var_z_, 2}}), 3 * a_ + b_}}};

  const Vector2<Variable> vars_xy(var_x_, var_y_);
  const auto J_xy = p.Jacobian(vars_xy);
  static_assert(decltype(J_xy)::RowsAtCompileTime == 1 &&
                    decltype(J_xy)::ColsAtCompileTime == 2,
                "The size of J_xy should be 1 x 2.");
  // dp/dx = 2axy + (3a+b)z²
  const GenericPolynomial<MonomialBasisElement> p_x(
      {{MonomialBasisElement({{var_x_, 1}, {var_y_, 1}}), 2 * a_},
       {MonomialBasisElement(var_z_, 2), 3 * a_ + b_}});
  // dp/dy = ax²
  const GenericPolynomial<MonomialBasisElement> p_y(
      {{MonomialBasisElement(var_x_, 2), a_}});
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, J_xy(0), p_x);
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, J_xy(1), p_y);

  // dp/da = x²y + 3xz²
  const GenericPolynomial<MonomialBasisElement> p_a(
      {{MonomialBasisElement({{var_x_, 2}, {var_y_, 1}}), 1},
       {MonomialBasisElement({{var_x_, 1}, {var_z_, 2}}), 3}});
  // dp/db = xz²
  const GenericPolynomial<MonomialBasisElement> p_b(
      MonomialBasisElement({{var_x_, 1}, {var_z_, 2}}));
  const Vector2<Variable> var_ab(var_a_, var_b_);
  const auto J_ab = p.Jacobian(var_ab);
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, J_ab(0), p_a);
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, J_ab(1), p_b);
}

TEST_F(SymbolicGenericPolynomialTest, ConstructNonPolynomialCoefficients) {
  // Given a pair of Expression and Polynomial::MapType, `(e, map)`, we check
  // `Polynomial(e, indeterminates)` has the expected map, `map`.
  vector<pair<Expression, GenericPolynomial<MonomialBasisElement>::MapType>>
      testcases;

  // sin(a)x = sin(a) * x
  testcases.emplace_back(sin(a_) * x_,
                         GenericPolynomial<MonomialBasisElement>::MapType{
                             {{MonomialBasisElement{x_}, sin(a_)}}});

  // cos(a)(x + 1)² = cos(a) * x² + 2cos(a) * x + cos(a) * 1
  testcases.emplace_back(cos(a_) * pow(x_ + 1, 2),
                         GenericPolynomial<MonomialBasisElement>::MapType{
                             {{MonomialBasisElement{{{var_x_, 2}}}, cos(a_)},
                              {MonomialBasisElement{x_}, 2 * cos(a_)},
                              {MonomialBasisElement{}, cos(a_)}}});

  //   log(a)(x + 1)² / sqrt(b)
  // = log(a)/sqrt(b) * x² + 2log(a)/sqrt(b) * x + log(a)/sqrt(b) * 1
  testcases.emplace_back(
      log(a_) * pow(x_ + 1, 2) / sqrt(b_),
      GenericPolynomial<MonomialBasisElement>::MapType{
          {{MonomialBasisElement{{{var_x_, 2}}}, log(a_) / sqrt(b_)},
           {MonomialBasisElement{x_}, 2 * log(a_) / sqrt(b_)},
           {MonomialBasisElement{}, log(a_) / sqrt(b_)}}});

  //   (tan(a)x + 1)²
  // = (tan(a))² * x² + 2tan(a) * x + 1
  testcases.emplace_back(
      pow(tan(a_) * x_ + 1, 2),
      GenericPolynomial<MonomialBasisElement>::MapType{
          {{MonomialBasisElement{{{var_x_, 2}}}, pow(tan(a_), 2)},
           {MonomialBasisElement{x_}, 2 * tan(a_)},
           {MonomialBasisElement{}, 1}}});

  //   abs(b + 1)x + asin(a) + acos(a) - atan(c) * x
  // = (abs(b + 1) - atan(c)) * x + (asin(a) + acos(a))
  testcases.emplace_back(
      abs(b_ + 1) * x_ + asin(a_) + acos(a_) - atan(c_) * x_,
      GenericPolynomial<MonomialBasisElement>::MapType{
          {{MonomialBasisElement{x_}, abs(b_ + 1) - atan(c_)},
           {MonomialBasisElement{}, asin(a_) + acos(a_)}}});

  //   atan(b)x * atan2(a, c)y
  // = (atan(b) * atan2(a, c)) * xy
  testcases.emplace_back(
      abs(b_) * x_ * atan2(a_, c_) * y_,
      GenericPolynomial<MonomialBasisElement>::MapType{
          {{MonomialBasisElement{{{var_x_, 1}, {var_y_, 1}}},  // xy
            abs(b_) * atan2(a_, c_)}}});

  //     (sinh(a)x + cosh(b)y + tanh(c)z) / (5 * min(a, b) * max(b, c))
  // =   (sinh(a) / (5 * min(a, b) * max(b, c))) * x
  //   + (cosh(b) / (5 * min(a, b) * max(b, c))) * y
  //   + (tanh(c) / (5 * min(a, b) * max(b, c))) * z
  testcases.emplace_back((sinh(a_) * x_ + cosh(b_) * y_ + tanh(c_) * z_) /
                             (5 * min(a_, b_) * max(b_, c_)),
                         GenericPolynomial<MonomialBasisElement>::MapType{
                             {{
                                  MonomialBasisElement{x_},
                                  sinh(a_) / (5 * min(a_, b_) * max(b_, c_)),
                              },
                              {
                                  MonomialBasisElement{y_},
                                  cosh(b_) / (5 * min(a_, b_) * max(b_, c_)),
                              },
                              {
                                  MonomialBasisElement{z_},
                                  tanh(c_) / (5 * min(a_, b_) * max(b_, c_)),
                              }}});

  //     (ceil(a) * x + floor(b) * y)²
  // =   pow(ceil(a), 2) * x²
  // = + 2 * ceil(a) * floor(b) * xy
  // = + pow(floor(a), 2) * y²
  testcases.emplace_back(
      pow(ceil(a_) * x_ + floor(b_) * y_, 2),
      GenericPolynomial<MonomialBasisElement>::MapType{
          {{MonomialBasisElement{{{var_x_, 2}}}, ceil(a_) * ceil(a_)},
           {MonomialBasisElement{{{var_x_, 1}, {var_y_, 1}}},
            2 * ceil(a_) * floor(b_)},
           {MonomialBasisElement{{{var_y_, 2}}}, floor(b_) * floor(b_)}}});

  //     (ceil(a) * x + floor(b) * y)²
  // =   pow(ceil(a), 2) * x²
  // = + 2 * ceil(a) * floor(b) * xy
  // = + pow(floor(a), 2) * y²
  testcases.emplace_back(
      pow(ceil(a_) * x_ + floor(b_) * y_, 2),
      GenericPolynomial<MonomialBasisElement>::MapType{
          {{MonomialBasisElement{{{var_x_, 2}}}, ceil(a_) * ceil(a_)},
           {MonomialBasisElement{{{var_x_, 1}, {var_y_, 1}}},
            2 * ceil(a_) * floor(b_)},
           {MonomialBasisElement{{{var_y_, 2}}}, floor(b_) * floor(b_)}}});

  //   UF("unnamed1", {a})) * x * UF("unnamed2", {b}) * x
  // = UF("unnamed1", {a})) * UF("unnamed2", {b}) * x².
  const Expression uf1{uninterpreted_function("unnamed1", {var_a_})};
  const Expression uf2{uninterpreted_function("unnamed2", {var_b_})};
  testcases.emplace_back(
      uf1 * x_ * uf2 * x_,
      GenericPolynomial<MonomialBasisElement>::MapType{
          {{MonomialBasisElement{{{var_x_, 2}}}, uf1 * uf2}}});

  //   (x + y)² = x² + 2xy + y²
  testcases.emplace_back(
      pow(x_ + y_, 2),
      GenericPolynomial<MonomialBasisElement>::MapType{{
          {MonomialBasisElement{{{var_x_, 2}}}, 1},
          {MonomialBasisElement{{{var_x_, 1}, {var_y_, 1}}}, 2},
          {MonomialBasisElement{{{var_y_, 2}}}, 1},
      }});

  // pow(pow(x, 2.5), 2) = x⁵
  testcases.emplace_back(pow(pow(x_, 2.5), 2),
                         GenericPolynomial<MonomialBasisElement>::MapType{
                             {{MonomialBasisElement{{{var_x_, 5}}}, 1}}});

  // pow(pow(x * y, 2.5), 2) = (xy)⁵
  testcases.emplace_back(
      pow(pow(x_ * y_, 2.5), 2),
      GenericPolynomial<MonomialBasisElement>::MapType{
          {{MonomialBasisElement{{{var_x_, 5}, {var_y_, 5}}}, 1}}});

  for (const auto& [e, expected_map] : testcases) {
    const GenericPolynomial<MonomialBasisElement> p{e, indeterminates_};
    EXPECT_EQ(p.basis_element_to_coefficient_map(), expected_map);
  }
}

TEST_F(SymbolicGenericPolynomialTest, NegativeTestConstruction1) {
  // sin(a) * x is a polynomial.
  const Expression e1{sin(a_) * x_};
  DRAKE_EXPECT_NO_THROW(
      GenericPolynomial<MonomialBasisElement>(e1, indeterminates_));

  // sin(x) * x is a not polynomial.
  const Expression e2{sin(x_) * x_};
  EXPECT_THROW(GenericPolynomial<MonomialBasisElement>(e2, indeterminates_),
               runtime_error);
}

TEST_F(SymbolicGenericPolynomialTest, NegativeTestConstruction2) {
  // aˣ x is not a polynomial.
  const Expression e{pow(a_, x_)};
  EXPECT_THROW(GenericPolynomial<MonomialBasisElement>(e, indeterminates_),
               runtime_error);
}

TEST_F(SymbolicGenericPolynomialTest, NegativeTestConstruction3) {
  // x⁻¹ is not a polynomial.
  const Expression e{pow(x_, -1)};
  EXPECT_THROW(GenericPolynomial<MonomialBasisElement>(e, indeterminates_),
               runtime_error);
}

TEST_F(SymbolicGenericPolynomialTest, NegativeTestConstruction4) {
  // x^(2.5) is not a polynomial.
  const Expression e{pow(x_, 2.5)};
  EXPECT_THROW(GenericPolynomial<MonomialBasisElement>(e, indeterminates_),
               runtime_error);
}

TEST_F(SymbolicGenericPolynomialTest, NegativeTestConstruction5) {
  // xˣ is not a polynomial.
  const Expression e{pow(x_, x_)};
  EXPECT_THROW(GenericPolynomial<MonomialBasisElement>(e, indeterminates_),
               runtime_error);
}

TEST_F(SymbolicGenericPolynomialTest, NegativeTestConstruction6) {
  // 1 / a is polynomial.
  const Expression e1{1 / a_};
  DRAKE_EXPECT_NO_THROW(
      GenericPolynomial<MonomialBasisElement>(e1, indeterminates_));

  // However, 1 / x is not a polynomial.
  const Expression e2{1 / x_};
  EXPECT_THROW(GenericPolynomial<MonomialBasisElement>(e2, indeterminates_),
               runtime_error);
}

TEST_F(SymbolicGenericPolynomialTest, NegativeTestConstruction7) {
  // sin(x + a) is not a polynomial.
  const Expression e{sin(x_ + a_)};
  EXPECT_THROW(GenericPolynomial<MonomialBasisElement>(e, indeterminates_),
               runtime_error);
}

TEST_F(SymbolicGenericPolynomialTest, NegativeTestConstruction) {
  std::vector<Expression> bad_expressions;
  // tan(x) is not a polynomial.
  bad_expressions.push_back(tan(x_));
  // abs(x + a) is not a polynomial.
  bad_expressions.push_back(abs(x_ + a_));
  // exp(x + a) is not a polynomial.
  bad_expressions.push_back(exp(x_ + a_));
  // sqrt(x  * x_ + 1) is not a polynomial.
  bad_expressions.push_back(sqrt(x_ * x_ + 1));
  // atan(x  * x_ + 1) is not a polynomial.
  bad_expressions.push_back(atan(x_ * x_ + 1));
  // atan2(2, x  * x_ + 1) is not a polynomial.
  bad_expressions.push_back(atan2(2, x_ * x_ + 1));
  // sinh(x  * x_ + 1) is not a polynomial.
  bad_expressions.push_back(sinh(x_ * x_ + 1));
  // cosh(x  * x_ + 1) is not a polynomial.
  bad_expressions.push_back(cosh(x_ * x_ + 1));
  // tanh(x  * x_ + 1) is not a polynomial.
  bad_expressions.push_back(tanh(x_ * x_ + 1));
  // min(x  * x_ + 1, x_* x_) is not a polynomial.
  bad_expressions.push_back(min(x_ * x_ + 1, x_ * x_));
  // max(x  * x_ + 1, x_* x_) is not a polynomial.
  bad_expressions.push_back(max(x_ * x_ + 1, x_ * x_));
  // ceil(x  * x_ + 1) is not a polynomial.
  bad_expressions.push_back(ceil(x_ * x_ + 1));
  // floor(x  * x_ + 1) is not a polynomial.
  bad_expressions.push_back(floor(x_ * x_ + 1));
  // IfThenElse(x_, x_, x  * x_ + 1) is not a polynomial.
  bad_expressions.push_back(if_then_else(x_ > 0, x_, x_ * x_ + 1));
  // (sin(x+a))³ is not a polynomial.
  bad_expressions.push_back(pow(sin(x_ + a_), 3));
  // (sin(x))³ is not a polynomial.
  bad_expressions.push_back(pow(sin(x_), 3));
  for (const auto& e : bad_expressions) {
    EXPECT_THROW(GenericPolynomial<MonomialBasisElement>(e, indeterminates_),
                 runtime_error);
  }
}

TEST_F(SymbolicGenericPolynomialTest, Evaluate) {
  // p = ax²y + bxy + cz
  const GenericPolynomial<MonomialBasisElement> p{
      {{MonomialBasisElement({{var_x_, 2}, {var_y_, 1}}), a_},
       {MonomialBasisElement({{var_x_, 1}, {var_y_, 1}}), b_},
       {MonomialBasisElement(var_z_), c_}}};

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
  double dummy{};
  EXPECT_THROW(dummy = p.Evaluate(partial_env), std::invalid_argument);
  unused(dummy);
}

TEST_F(SymbolicGenericPolynomialTest, PartialEvaluate1) {
  // p1 = a*x² + b*x + c
  // p2 = p1[x ↦ 3.0] = 3²a + 3b + c.
  const GenericPolynomial<MonomialBasisElement> p1{
      {{MonomialBasisElement(var_x_, 2), a_},
       {MonomialBasisElement(var_x_), b_},
       {MonomialBasisElement(), c_}}};
  const GenericPolynomial<MonomialBasisElement> p2{
      {{MonomialBasisElement(), a_ * 3.0 * 3.0 + b_ * 3.0 + c_}}};
  const Environment env{{{var_x_, 3.0}}};
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, p1.EvaluatePartial(env),
               p2);
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>,
               p1.EvaluatePartial(var_x_, 3.0), p2);
}

TEST_F(SymbolicGenericPolynomialTest, PartialEvaluate2) {
  // p1 = a*xy² - a*xy + c
  // p2 = p1[y ↦ 2.0] = (4a - 2a)*x + c = 2ax + c
  const GenericPolynomial<MonomialBasisElement> p1{
      {{MonomialBasisElement({{var_x_, 1}, {var_y_, 2}}), a_},
       {MonomialBasisElement({{var_x_, 1}, {var_y_, 1}}), -a_},
       {MonomialBasisElement(), c_}}};
  const GenericPolynomial<MonomialBasisElement> p2{
      {{MonomialBasisElement(var_x_), 2 * a_}, {MonomialBasisElement(), c_}}};
  const Environment env{{{var_y_, 2.0}}};
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, p1.EvaluatePartial(env),
               p2);
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>,
               p1.EvaluatePartial(var_y_, 2.0), p2);
}

TEST_F(SymbolicGenericPolynomialTest, PartialEvaluate3) {
  // p1 = a*x² + b*x + c
  // p2 = p1[a ↦ 2.0, x ↦ 3.0] = 2*3² + 3b + c
  //                           = 18 + 3b + c
  const GenericPolynomial<MonomialBasisElement> p1{
      {{MonomialBasisElement(var_x_, 2), a_},
       {MonomialBasisElement(var_x_), b_},
       {MonomialBasisElement(), c_}}};
  const GenericPolynomial<MonomialBasisElement> p2{
      {{MonomialBasisElement(), 18 + 3 * b_ + c_}}};
  const Environment env{{{var_a_, 2.0}, {var_x_, 3.0}}};
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, p1.EvaluatePartial(env),
               p2);
}

TEST_F(SymbolicGenericPolynomialTest, PartialEvaluate4) {
  // p = (a + c / b + c)*x² + b*x + c
  //
  // Partially evaluating p with [a ↦ 0, b ↦ 0, c ↦ 0] throws `runtime_error`
  // because of the divide-by-zero
  const GenericPolynomial<MonomialBasisElement> p{
      {{MonomialBasisElement(var_x_, 2), (a_ + c_) / (b_ + c_)},
       {MonomialBasisElement(var_x_), b_},
       {MonomialBasisElement(), c_}}};
  const Environment env{{{var_a_, 0.0}, {var_b_, 0.0}, {var_c_, 0.0}}};
  GenericPolynomial<MonomialBasisElement> dummy;
  EXPECT_THROW(dummy = p.EvaluatePartial(env), runtime_error);
}

TEST_F(SymbolicGenericPolynomialTest, EqualTo) {
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>,
               GenericPolynomial<MonomialBasisElement>(),
               GenericPolynomial<MonomialBasisElement>(nullptr));

  EXPECT_PRED2(
      GenericPolyEqual<MonomialBasisElement>,
      GenericPolynomial<MonomialBasisElement>(MonomialBasisElement(var_x_)),
      GenericPolynomial<MonomialBasisElement>(
          {{MonomialBasisElement(var_x_), 1}}));

  EXPECT_FALSE(
      GenericPolynomial<MonomialBasisElement>(MonomialBasisElement(var_x_))
          .EqualTo(GenericPolynomial<MonomialBasisElement>(
              MonomialBasisElement(var_y_))));
  EXPECT_FALSE(
      GenericPolynomial<MonomialBasisElement>(MonomialBasisElement(var_x_))
          .EqualTo(GenericPolynomial<MonomialBasisElement>(
              {{MonomialBasisElement(var_y_), a_}})));

  EXPECT_FALSE(GenericPolynomial<MonomialBasisElement>(
                   {{MonomialBasisElement(var_x_), 1},
                    {MonomialBasisElement(var_x_, 2), 2}})
                   .EqualTo(GenericPolynomial<MonomialBasisElement>(
                       {{MonomialBasisElement(var_x_), 1}})));

  EXPECT_FALSE(GenericPolynomial<MonomialBasisElement>(
                   {{MonomialBasisElement(var_x_), 2},
                    {MonomialBasisElement(var_x_, 2), 2}})
                   .EqualTo(GenericPolynomial<MonomialBasisElement>(
                       {{MonomialBasisElement(var_x_), 1},
                        {MonomialBasisElement(var_x_, 2), 2}})));
}

TEST_F(SymbolicGenericPolynomialTest, AddProduct1) {
  // p = ax² + bxy
  // p + cz² = ax² + cz² + bxy
  // The added term and p doesn't share basis element.
  GenericPolynomial<MonomialBasisElement> p(
      {{MonomialBasisElement(var_x_, 2), a_},
       {MonomialBasisElement({{var_x_, 1}, {var_y_, 1}}), b_}});
  const GenericPolynomial<MonomialBasisElement> sum =
      p.AddProduct(c_, MonomialBasisElement(var_z_, 2));
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, sum, p);
  const GenericPolynomial<MonomialBasisElement> sum_expected(
      {{MonomialBasisElement(var_x_, 2), a_},
       {MonomialBasisElement(var_z_, 2), c_},
       {MonomialBasisElement({{var_x_, 1}, {var_y_, 1}}), b_}});
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, sum, sum_expected);
  EXPECT_EQ(p.decision_variables(), Variables({var_a_, var_b_, var_c_}));
  EXPECT_EQ(p.indeterminates(), Variables({var_x_, var_y_, var_z_}));
}

TEST_F(SymbolicGenericPolynomialTest, AddProduct2) {
  // p = ay² + bxy
  // p + cx² = (a+c)x² + bxy
  // The added term and p shares basis element.
  GenericPolynomial<MonomialBasisElement> p(
      {{MonomialBasisElement(var_x_, 2), a_},
       {MonomialBasisElement({{var_x_, 1}, {var_y_, 1}}), b_}});
  const GenericPolynomial<MonomialBasisElement> sum =
      p.AddProduct(c_, MonomialBasisElement(var_x_, 2));
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, sum, p);
  const GenericPolynomial<MonomialBasisElement> sum_expected(
      {{MonomialBasisElement(var_x_, 2), a_ + c_},
       {MonomialBasisElement({{var_x_, 1}, {var_y_, 1}}), b_}});
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>, sum, sum_expected);
  EXPECT_EQ(p.decision_variables(), Variables({var_a_, var_b_, var_c_}));
  EXPECT_EQ(p.indeterminates(), Variables({var_x_, var_y_}));
}

template <typename BasisElement>
void CheckHash(const Expression& x, const Expression& y) {
  const auto h = std::hash<Polynomial>{};
  GenericPolynomial<BasisElement> p1{x * x};
  const GenericPolynomial<BasisElement> p2{x * x};
  EXPECT_EQ(p1, p2);
  EXPECT_EQ(h(p1), h(p2));
  p1 += GenericPolynomial<BasisElement>{y};
  EXPECT_NE(p1, p2);
  EXPECT_NE(h(p1), h(p2));
}
TEST_F(SymbolicGenericPolynomialTest, Hash) {
  CheckHash<MonomialBasisElement>(x_, y_);
  CheckHash<ChebyshevBasisElement>(x_, y_);
}

TEST_F(SymbolicGenericPolynomialTest, CoefficientsAlmostEqual) {
  GenericPolynomial<MonomialBasisElement> p1{x_ * x_};
  // Two polynomials with the same number of terms.
  EXPECT_TRUE(p1.CoefficientsAlmostEqual(
      GenericPolynomial<MonomialBasisElement>{x_ * x_}, 1e-6));
  EXPECT_TRUE(p1.CoefficientsAlmostEqual(
      GenericPolynomial<MonomialBasisElement>{(1 + 1e-7) * x_ * x_}, 1e-6));
  EXPECT_FALSE(p1.CoefficientsAlmostEqual(
      GenericPolynomial<MonomialBasisElement>{2 * x_ * x_}, 1e-6));
  // Another polynomial with an additional small constant term.
  EXPECT_TRUE(p1.CoefficientsAlmostEqual(
      GenericPolynomial<MonomialBasisElement>{x_ * x_ + 1e-7}, 1e-6));
  EXPECT_FALSE(p1.CoefficientsAlmostEqual(
      GenericPolynomial<MonomialBasisElement>{x_ * x_ + 2e-6}, 1e-6));
  EXPECT_TRUE(GenericPolynomial<MonomialBasisElement>{x_ * x_ + 1e-7}
                  .CoefficientsAlmostEqual(p1, 1e-6));
  EXPECT_FALSE(GenericPolynomial<MonomialBasisElement>{x_ * x_ + 2e-6}
                   .CoefficientsAlmostEqual(p1, 1e-6));
  // Another polynomial with small difference on coefficients.
  EXPECT_TRUE(p1.CoefficientsAlmostEqual(
      GenericPolynomial<MonomialBasisElement>{(1. - 1e-7) * x_ * x_ + 1e-7},
      1e-6));
  EXPECT_FALSE(p1.CoefficientsAlmostEqual(
      GenericPolynomial<MonomialBasisElement>{(1. + 2e-6) * x_ * x_ + 1e-7},
      1e-6));

  // Another polynomial with decision variables in the coefficient.
  const symbolic::GenericPolynomial<MonomialBasisElement> p2(a_ * x_ * x_,
                                                             {indeterminates_});
  EXPECT_TRUE(p2.CoefficientsAlmostEqual(
      GenericPolynomial<MonomialBasisElement>{(a_ + 1e-7) * x_ * x_,
                                              {indeterminates_}},
      1e-6));
  EXPECT_FALSE(p2.CoefficientsAlmostEqual(
      GenericPolynomial<MonomialBasisElement>{(a_ + 1e-7) * x_ * x_,
                                              {indeterminates_}},
      1e-8));
}

TEST_F(SymbolicGenericPolynomialTest, RemoveTermsWithSmallCoefficients) {
  // Single term.
  GenericPolynomial<MonomialBasisElement> p1{
      {{MonomialBasisElement(var_x_, 2), 1e-5}}};
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>,
               p1.RemoveTermsWithSmallCoefficients(1E-4),
               GenericPolynomial<MonomialBasisElement>(0));
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>,
               p1.RemoveTermsWithSmallCoefficients(1E-5),
               GenericPolynomial<MonomialBasisElement>(0));
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>,
               p1.RemoveTermsWithSmallCoefficients(1E-6), p1);

  // Multiple terms.
  const GenericPolynomial<MonomialBasisElement> p2{
      {{MonomialBasisElement(var_x_, 2), 2},
       {MonomialBasisElement({{var_x_, 1}, {var_y_, 1}}), 3},
       {MonomialBasisElement(var_x_), 1E-4},
       {MonomialBasisElement(), -1E-4}}};
  const GenericPolynomial<MonomialBasisElement> p2_cleaned{
      {{MonomialBasisElement(var_x_, 2), 2},
       {MonomialBasisElement({{var_x_, 1}, {var_y_, 1}}), 3}}};
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>,
               p2.RemoveTermsWithSmallCoefficients(1E-4), p2_cleaned);

  // Coefficients are expressions.
  GenericPolynomial<MonomialBasisElement>::MapType p3_map{};
  p3_map.emplace(MonomialBasisElement(var_x_, 2), 2 * sin(y_));
  p3_map.emplace(MonomialBasisElement(var_x_, 1), 1E-4 * cos(y_));
  p3_map.emplace(MonomialBasisElement(var_x_, 3), 1E-4 * y_);
  p3_map.emplace(MonomialBasisElement(), 1E-6);
  GenericPolynomial<MonomialBasisElement>::MapType p3_expected_map{};
  p3_expected_map.emplace(MonomialBasisElement(var_x_, 2), 2 * sin(y_));
  p3_expected_map.emplace(MonomialBasisElement(var_x_, 1), 1E-4 * cos(y_));
  p3_expected_map.emplace(MonomialBasisElement(var_x_, 3), 1E-4 * y_);
  EXPECT_PRED2(GenericPolyEqual<MonomialBasisElement>,
               GenericPolynomial<MonomialBasisElement>(p3_map)
                   .RemoveTermsWithSmallCoefficients(1E-3),
               GenericPolynomial<MonomialBasisElement>(p3_expected_map));
}

TEST_F(SymbolicGenericPolynomialTest, EqualToAfterExpansion) {
  // Test equal with / without expansion.
  const GenericPolynomial<MonomialBasisElement> p1(
      {{MonomialBasisElement(var_x_, 2), (a_ + b_) * (a_ + c_)}});
  const GenericPolynomial<MonomialBasisElement> p2(
      {{MonomialBasisElement(var_x_, 2),
        pow(var_a_, 2) + a_ * b_ + a_ * c_ + b_ * c_}});
  // No expansion
  EXPECT_FALSE(p1.EqualTo(p2));
  EXPECT_TRUE(p1.EqualToAfterExpansion(p2));
}

TEST_F(SymbolicGenericPolynomialTest, SetIndeterminates) {
  // ax² + bx + c
  const Expression e{a_ * x_ * x_ + b_ * x_ + c_};

  {
    // {x} -> {x, a}
    // Grow the indeterminates with a varible moves from decision variables
    // to indeterminates.
    GenericPolynomial<MonomialBasisElement> p{e, {var_x_}};
    const Variables new_indeterminates{var_x_, var_a_};
    p.SetIndeterminates(new_indeterminates);
    EXPECT_PRED2(
        GenericPolyEqual<MonomialBasisElement>, p,
        GenericPolynomial<MonomialBasisElement>(e, new_indeterminates));
  }

  {
    // {x} -> {x, y}, note that y ∉ variables(e).
    GenericPolynomial<MonomialBasisElement> p{e, {var_x_}};
    const Variables new_indeterminates{var_x_, var_y_};
    p.SetIndeterminates(new_indeterminates);
    EXPECT_PRED2(
        GenericPolyEqual<MonomialBasisElement>, p,
        GenericPolynomial<MonomialBasisElement>(e, new_indeterminates));
  }

  {
    // {x} -> {x, a, b, c}
    GenericPolynomial<MonomialBasisElement> p{e, {var_x_}};
    const Variables new_indeterminates{var_x_, var_a_, var_b_, var_c_};
    p.SetIndeterminates(new_indeterminates);
    EXPECT_PRED2(
        GenericPolyEqual<MonomialBasisElement>, p,
        GenericPolynomial<MonomialBasisElement>(e, new_indeterminates));
  }

  {
    // {x, a} -> {x}
    GenericPolynomial<MonomialBasisElement> p{e, {var_x_, var_a_}};
    const Variables new_indeterminates{var_x_};
    p.SetIndeterminates(new_indeterminates);
    EXPECT_PRED2(
        GenericPolyEqual<MonomialBasisElement>, p,
        GenericPolynomial<MonomialBasisElement>(e, new_indeterminates));
  }

  {
    // {x, a} -> {a}
    GenericPolynomial<MonomialBasisElement> p{e, {var_x_, var_a_}};
    const Variables new_indeterminates{var_a_};
    p.SetIndeterminates(new_indeterminates);
    EXPECT_PRED2(
        GenericPolyEqual<MonomialBasisElement>, p,
        GenericPolynomial<MonomialBasisElement>(e, new_indeterminates));
  }

  {
    // {x, a, b, c} -> {x}
    GenericPolynomial<MonomialBasisElement> p{e,
                                              {var_x_, var_a_, var_b_, var_c_}};
    const Variables new_indeterminates{var_x_};
    p.SetIndeterminates(new_indeterminates);
    EXPECT_PRED2(
        GenericPolyEqual<MonomialBasisElement>, p,
        GenericPolynomial<MonomialBasisElement>(e, new_indeterminates));
  }
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
