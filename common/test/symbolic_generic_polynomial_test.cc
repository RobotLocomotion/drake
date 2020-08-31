#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

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
}  // namespace
}  // namespace symbolic
}  // namespace drake
