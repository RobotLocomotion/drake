#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/common/unused.h"

namespace drake {
namespace symbolic {
namespace {
using std::runtime_error;

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

}  // namespace
}  // namespace symbolic
}  // namespace drake
