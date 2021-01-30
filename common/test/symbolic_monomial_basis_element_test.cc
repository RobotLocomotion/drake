#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/common/unused.h"

using std::pair;
using std::runtime_error;

namespace drake {
namespace symbolic {

using test::ExprEqual;
using test::VarLess;

class MonomialBasisElementTest : public ::testing::Test {
 protected:
  const symbolic::Variable var_w_{"w"};
  const symbolic::Variable var_z_{"z"};
  const symbolic::Variable var_y_{"y"};
  const symbolic::Variable var_x_{"x"};

  const Expression w_{var_w_};
  const Expression z_{var_z_};
  const Expression y_{var_y_};
  const Expression x_{var_x_};

  std::vector<MonomialBasisElement> monomials_;

  void SetUp() override {
    monomials_ = {
        MonomialBasisElement{},                            // 1.0
        MonomialBasisElement{var_x_, 1},                   // x^1
        MonomialBasisElement{var_y_, 1},                   // y^1
        MonomialBasisElement{var_z_, 1},                   // z^1
        MonomialBasisElement{var_x_, 2},                   // x^2
        MonomialBasisElement{var_y_, 3},                   // y^3
        MonomialBasisElement{var_z_, 4},                   // z^4
        MonomialBasisElement{{{var_x_, 1}, {var_y_, 2}}},  // xy^2
        MonomialBasisElement{{{var_y_, 2}, {var_z_, 5}}},  // y^2z^5
        MonomialBasisElement{
            {{var_x_, 1}, {var_y_, 2}, {var_z_, 3}}},  // xy^2z^3
        MonomialBasisElement{
            {{var_x_, 2}, {var_y_, 4}, {var_z_, 3}}},  // x^2y^4z^3
    };

    EXPECT_PRED2(VarLess, var_y_, var_x_);
    EXPECT_PRED2(VarLess, var_z_, var_y_);
    EXPECT_PRED2(VarLess, var_w_, var_z_);
  }

  // Helper function to extract Substitution (Variable -> Expression) from a
  // symbolic environment.
  Substitution ExtractSubst(const Environment& env) {
    Substitution subst;
    subst.reserve(env.size());
    for (const pair<const Variable, double>& p : env) {
      subst.emplace(p.first, p.second);
    }
    return subst;
  }

  // Checks if MonomialBasisElement::Evaluate corresponds to
  // Expression::Evaluate.
  //
  //                            ToExpression
  //                     MonomialBasisElement ----> Expression
  //                                ||                ||
  // MonomialBasisElement::Evaluate ||                || Expression::Evaluate
  //                                ||                ||
  //                                \/                \/
  //                          double (result2) == double (result1)
  //
  // In one direction (left-to-right), we convert a Monomial to an Expression
  // and evaluate it to a double value (result1). In another direction
  // (top-to-bottom), we directly evaluate a Monomial to double (result2). The
  // two result1 and result2 should be the same.
  bool CheckEvaluate(const MonomialBasisElement& m, const Environment& env) {
    const double result1{m.ToExpression().Evaluate(env)};
    const double result2{m.Evaluate(env)};
    return result1 == result2;
  }

  // Checks if MonomialBasisElement::EvaluatePartial corresponds to Expression's
  // EvaluatePartial.
  //
  //                            ToExpression
  //           MonomialBasisElement ---------> Expression
  //                      ||                      ||
  //      EvaluatePartial ||                      || EvaluatePartial
  //                      ||                      ||
  //                      \/                      \/
  //   double * MonomialBasisElement (e2) ==  Expression (e1)
  //
  // In one direction (left-to-right), first we convert a MonomialBasisElement
  // to an Expression using MonomialBasisElement::ToExpression and call
  // Expression::EvaluatePartial to have an Expression (e1). In another
  // direction (top-to-bottom), we call MonomialBasisElement::EvaluatePartial
  // which returns a pair of double (coefficient part) and Monomial. We obtain
  // e2 by multiplying the two. Then, we check if e1 and e2 are structurally
  // equal.
  bool CheckEvaluatePartial(const MonomialBasisElement& m,
                            const Environment& env) {
    const Expression e1{m.ToExpression().EvaluatePartial(env)};
    const pair<double, MonomialBasisElement> subst_result{
        m.EvaluatePartial(env)};
    const Expression e2{subst_result.first *
                        subst_result.second.ToExpression()};

    return e1.EqualTo(e2);
  }
};

TEST_F(MonomialBasisElementTest, Constructor) {
  const MonomialBasisElement m1{};
  EXPECT_TRUE(m1.var_to_degree_map().empty());
  EXPECT_EQ(m1.total_degree(), 0);

  const MonomialBasisElement m2{var_x_};
  EXPECT_EQ(m2.var_to_degree_map().size(), 1);
  EXPECT_EQ(m2.var_to_degree_map().at(var_x_), 1);
  EXPECT_EQ(m2.total_degree(), 1);

  const MonomialBasisElement m3{pow(var_x_, 2) * pow(var_y_, 3)};
  EXPECT_EQ(m3.var_to_degree_map().size(), 2);
  EXPECT_EQ(m3.var_to_degree_map().at(var_x_), 2);
  EXPECT_EQ(m3.var_to_degree_map().at(var_y_), 3);
  EXPECT_EQ(m3.total_degree(), 5);

  const MonomialBasisElement m4{{{var_x_, 3}, {var_y_, 2}}};
  EXPECT_EQ(m4.var_to_degree_map().size(), 2);
  EXPECT_EQ(m4.var_to_degree_map().at(var_x_), 3);
  EXPECT_EQ(m4.var_to_degree_map().at(var_y_), 2);
  EXPECT_EQ(m4.total_degree(), 5);

  const MonomialBasisElement m5{{{var_x_, 3}, {var_y_, 0}}};
  EXPECT_EQ(m5.var_to_degree_map().size(), 1);
  EXPECT_EQ(m5.var_to_degree_map().at(var_x_), 3);
  EXPECT_EQ(m5.total_degree(), 3);

  const MonomialBasisElement m6{Vector2<Variable>(var_x_, var_y_),
                                Eigen::Vector2i(1, 3)};
  EXPECT_EQ(m6.var_to_degree_map().size(), 2);
  EXPECT_EQ(m6.var_to_degree_map().at(var_x_), 1);
  EXPECT_EQ(m6.var_to_degree_map().at(var_y_), 3);
  EXPECT_EQ(m6.total_degree(), 4);

  const MonomialBasisElement m7{var_x_, 3};
  EXPECT_EQ(m7.var_to_degree_map().size(), 1);
  EXPECT_EQ(m7.var_to_degree_map().at(var_x_), 3);
  EXPECT_EQ(m7.total_degree(), 3);
}

// Tests that default constructor and EIGEN_INITIALIZE_MATRICES_BY_ZERO
// constructor both create the same value.
TEST_F(MonomialBasisElementTest, DefaultConstructors) {
  const MonomialBasisElement m_default;
  const MonomialBasisElement m_zero(0);
  EXPECT_EQ(m_default.total_degree(), 0);
  EXPECT_EQ(m_zero.total_degree(), 0);
}

TEST_F(MonomialBasisElementTest, ConstructFromVariable) {
  const MonomialBasisElement m1{var_x_};
  const std::map<Variable, int> powers{m1.var_to_degree_map()};

  // Checks that powers = {x ↦ 1}.
  ASSERT_EQ(powers.size(), 1u);
  EXPECT_EQ(powers.begin()->first, var_x_);
  EXPECT_EQ(powers.begin()->second, 1);
}

TEST_F(MonomialBasisElementTest, ConstructFromVariablesAndExponents) {
  // [] * [] => 1.
  const VectorX<Variable> vars(0);
  const VectorX<int> exponents(0);
  const MonomialBasisElement one{MonomialBasisElement{Expression::One()}};
  EXPECT_EQ(MonomialBasisElement(vars, exponents), one);

  const Vector3<Variable> vars_xyz{var_x_, var_y_, var_z_};
  // [x, y, z] * [0, 0, 0] => 1.
  const MonomialBasisElement m1{vars_xyz, Eigen::Vector3i{0, 0, 0}};
  EXPECT_EQ(m1, one);

  // [x, y, z] * [1, 1, 1] => xyz.
  const MonomialBasisElement m2{vars_xyz, Eigen::Vector3i{1, 1, 1}};
  const MonomialBasisElement m2_expected{x_ * y_ * z_};
  EXPECT_EQ(m2, m2_expected);

  // [x, y, z] * [2, 0, 3] => x²z³.
  const MonomialBasisElement m3{vars_xyz, Eigen::Vector3i{2, 0, 3}};
  const MonomialBasisElement m3_expected{pow(x_, 2) * pow(z_, 3)};
  EXPECT_EQ(m3, m3_expected);

  // [x, y, z] * [2, 0, -1] => Exception!
  DRAKE_EXPECT_THROWS_MESSAGE(
      MonomialBasisElement(vars_xyz, Eigen::Vector3i(2, 0, -1)),
      std::logic_error, "The exponent is negative.");
}

TEST_F(MonomialBasisElementTest, GetVariables) {
  const MonomialBasisElement m0{};
  EXPECT_EQ(m0.GetVariables(), Variables{});

  const MonomialBasisElement m1{var_z_, 4};
  EXPECT_EQ(m1.GetVariables(), Variables({var_z_}));

  const MonomialBasisElement m2{{{var_x_, 1}, {var_y_, 2}}};
  EXPECT_EQ(m2.GetVariables(), Variables({var_x_, var_y_}));

  const MonomialBasisElement m3{{{var_x_, 1}, {var_y_, 2}, {var_z_, 3}}};
  EXPECT_EQ(m3.GetVariables(), Variables({var_x_, var_y_, var_z_}));
}

// Checks we can have an Eigen matrix of Monomials without compilation
// errors. No assertions in the test.
TEST_F(MonomialBasisElementTest, EigenMatrixOfMonomials) {
  Eigen::Matrix<MonomialBasisElement, 2, 2> M;
  // M = | 1   x    |
  //     | y²  x²z³ |
  M << MonomialBasisElement{}, MonomialBasisElement{var_x_},
      MonomialBasisElement{{{var_y_, 2}}},
      MonomialBasisElement{{{var_x_, 2}, {var_z_, 3}}};

  // The following fails if we do not provide
  // `Eigen::NumTraits<drake::symbolic::MonomialBasisElement>`.
  std::ostringstream oss;
  oss << M;
}

TEST_F(MonomialBasisElementTest, MonomialOne) {
  // Compares monomials all equal to 1, but with different variables.
  MonomialBasisElement m1{};
  MonomialBasisElement m2({{var_x_, 0}});
  MonomialBasisElement m3({{var_x_, 0}, {var_y_, 0}});
  EXPECT_EQ(m1, m2);
  EXPECT_EQ(m1, m3);
  EXPECT_EQ(m2, m3);
}

TEST_F(MonomialBasisElementTest, MonomialWithZeroExponent) {
  // Compares monomials containing zero exponent, such as x^0 * y^2
  MonomialBasisElement m1({{var_y_, 2}});
  MonomialBasisElement m2({{var_x_, 0}, {var_y_, 2}});
  EXPECT_EQ(m1, m2);
  EXPECT_EQ(m2.var_to_degree_map().size(), 1);
  std::map<Variable, int> power_expected;
  power_expected.emplace(var_y_, 2);
  EXPECT_EQ(m2.var_to_degree_map(), power_expected);
}

// This test shows that we can have a std::unordered_map whose key is of
// MonomialBasisElement.
TEST_F(MonomialBasisElementTest, UnorderedMapOfMonomial) {
  std::unordered_map<MonomialBasisElement, double> monomial_to_coeff_map;
  MonomialBasisElement x_3{var_x_, 3};
  MonomialBasisElement y_5{var_y_, 5};
  // Add 2 * x^3
  monomial_to_coeff_map.emplace(x_3, 2);
  // Add -7 * y^5
  monomial_to_coeff_map.emplace(y_5, -7);

  const auto it1 = monomial_to_coeff_map.find(x_3);
  ASSERT_TRUE(it1 != monomial_to_coeff_map.end());
  EXPECT_EQ(it1->second, 2);

  const auto it2 = monomial_to_coeff_map.find(y_5);
  ASSERT_TRUE(it2 != monomial_to_coeff_map.end());
  EXPECT_EQ(it2->second, -7);
}

// Converts a constant to monomial.
TEST_F(MonomialBasisElementTest, ToMonomial0) {
  MonomialBasisElement expected;
  EXPECT_EQ(MonomialBasisElement(1), expected);
  EXPECT_EQ(MonomialBasisElement(pow(x_, 0)), expected);
  EXPECT_THROW(MonomialBasisElement(2), std::exception);
}

// Converts expression x to monomial.
TEST_F(MonomialBasisElementTest, ToMonomial1) {
  MonomialBasisElement expected(var_x_, 1);
  EXPECT_EQ(MonomialBasisElement(x_), expected);
}

// Converts expression x * y to monomial.
TEST_F(MonomialBasisElementTest, ToMonomial2) {
  std::map<Variable, int> powers;
  powers.emplace(var_x_, 1);
  powers.emplace(var_y_, 1);
  MonomialBasisElement expected(powers);
  EXPECT_EQ(MonomialBasisElement(x_ * y_), expected);
}

// Converts expression x^3 to monomial.
TEST_F(MonomialBasisElementTest, ToMonomial3) {
  MonomialBasisElement expected(var_x_, 3);
  EXPECT_EQ(MonomialBasisElement(pow(x_, 3)), expected);
  EXPECT_EQ(MonomialBasisElement(pow(x_, 2) * x_), expected);
}

// Converts expression x^3 * y to monomial.
TEST_F(MonomialBasisElementTest, ToMonomial4) {
  std::map<Variable, int> powers;
  powers.emplace(var_x_, 3);
  powers.emplace(var_y_, 1);
  MonomialBasisElement expected(powers);
  EXPECT_EQ(MonomialBasisElement(pow(x_, 3) * y_), expected);
  EXPECT_EQ(MonomialBasisElement(pow(x_, 2) * y_ * x_), expected);
}

// Converts expression x*(y+z) - x*y to monomial
TEST_F(MonomialBasisElementTest, ToMonomial5) {
  std::map<Variable, int> powers({{var_x_, 1}, {var_z_, 1}});
  MonomialBasisElement expected(powers);
  EXPECT_EQ(MonomialBasisElement(x_ * z_), expected);
  EXPECT_EQ(MonomialBasisElement(x_ * (y_ + z_) - x_ * y_), expected);
}

// `pow(x * y, 2) * pow(x^2 * y^2, 3)` is a monomial (x^8 * y^8).
TEST_F(MonomialBasisElementTest, ToMonomial6) {
  const MonomialBasisElement m1{pow(x_ * y_, 2) * pow(x_ * x_ * y_ * y_, 3)};
  const MonomialBasisElement m2{{{var_x_, 8}, {var_y_, 8}}};
  EXPECT_EQ(m1, m2);
}

// x^0 is a monomial (1).
TEST_F(MonomialBasisElementTest, ToMonomial7) {
  const MonomialBasisElement m1(var_x_, 0);
  const MonomialBasisElement m2{Expression{1.0}};
  const MonomialBasisElement m3{pow(x_, 0.0)};
  EXPECT_EQ(m1, m2);
  EXPECT_EQ(m1, m3);
  EXPECT_EQ(m2, m3);
}

// `2 * x` is not a monomial because of its coefficient `2`.
TEST_F(MonomialBasisElementTest, ToMonomialException1) {
  EXPECT_THROW(MonomialBasisElement{2 * x_}, runtime_error);
}

// `x + y` is not a monomial.
TEST_F(MonomialBasisElementTest, ToMonomialException2) {
  EXPECT_THROW(MonomialBasisElement{x_ + y_}, runtime_error);
}

// `x / 2.0` is not a monomial.
TEST_F(MonomialBasisElementTest, ToMonomialException3) {
  EXPECT_THROW(MonomialBasisElement{x_ / 2.0}, runtime_error);
}

// `x ^ -1` is not a monomial.
TEST_F(MonomialBasisElementTest, ToMonomialException4) {
  // Note: Parentheses are required around macro argument containing braced
  // initializer list.
  DRAKE_EXPECT_THROWS_MESSAGE((MonomialBasisElement{{{var_x_, -1}}}),
                              std::logic_error,
                              "The degree for x is negative.");
}

TEST_F(MonomialBasisElementTest, Multiplication) {
  // m₁ = xy²
  const MonomialBasisElement m1{{{var_x_, 1}, {var_y_, 2}}};
  // m₂ = y²z⁵
  const MonomialBasisElement m2{{{var_y_, 2}, {var_z_, 5}}};
  // m₃ = m₁ * m₂ = xy⁴z⁵
  const MonomialBasisElement m3{{{var_x_, 1}, {var_y_, 4}, {var_z_, 5}}};
  const std::map<MonomialBasisElement, double> result1 = m1 * m2;
  EXPECT_EQ(result1.size(), 1);
  EXPECT_EQ(result1.at(m3), 1);

  // m₄=z³
  const MonomialBasisElement m4(var_z_, 3);
  const auto result2 = m1 * m4;
  EXPECT_EQ(result2.size(), 1);
  EXPECT_EQ(
      result2.at(MonomialBasisElement({{var_x_, 1}, {var_y_, 2}, {var_z_, 3}})),
      1);

  const auto result3 = m2 * m4;
  EXPECT_EQ(result3.size(), 1);
  EXPECT_EQ(result3.at(MonomialBasisElement({{var_y_, 2}, {var_z_, 8}})), 1);
}

TEST_F(MonomialBasisElementTest, Pow) {
  // m₁ = xy²
  const MonomialBasisElement m1{{{var_x_, 1}, {var_y_, 2}}};
  // m₂ = y²z⁵
  const MonomialBasisElement m2{{{var_y_, 2}, {var_z_, 5}}};

  // pow(m₁, 0) = 1.0
  const std::map<MonomialBasisElement, double> result1 = pow(m1, 0);
  EXPECT_EQ(result1.size(), 1);
  EXPECT_EQ(result1.at(MonomialBasisElement()), 1);
  // pow(m₂, 0) = 1.0
  const std::map<MonomialBasisElement, double> result2 = pow(m2, 0);
  EXPECT_EQ(result2.size(), 1);
  EXPECT_EQ(result2.at(MonomialBasisElement()), 1);

  // pow(m₁, 1) = m₁
  const auto result3 = pow(m1, 1);
  EXPECT_EQ(result3.size(), 1);
  EXPECT_EQ(result3.at(m1), 1);
  // pow(m₂, 1) = m₂
  const auto result4 = pow(m2, 1);
  EXPECT_EQ(result4.size(), 1);
  EXPECT_EQ(result4.at(m2), 1);

  // pow(m₁, 3) = x³y⁶
  const auto result5 = pow(m1, 3);
  EXPECT_EQ(result5.size(), 1);
  EXPECT_EQ(result5.at(MonomialBasisElement({{var_x_, 3}, {var_y_, 6}})), 1);
  EXPECT_EQ(result5.begin()->first.total_degree(), 9);
  // pow(m₂, 4) = y⁸z²⁰
  const auto result6 = pow(m2, 4);
  EXPECT_EQ(result6.size(), 1);
  EXPECT_EQ(result6.at(MonomialBasisElement({{var_y_, 8}, {var_z_, 20}})), 1);
  EXPECT_EQ(result6.begin()->first.total_degree(), 28);

  // pow(m₁, -1) throws an exception.
  EXPECT_THROW(pow(m1, -1), runtime_error);
  // pow(m₂, -5) throws an exception.
  EXPECT_THROW(pow(m2, -5), runtime_error);
}

TEST_F(MonomialBasisElementTest, PowInPlace) {
  // m₁ = xy²
  MonomialBasisElement m1{{{var_x_, 1}, {var_y_, 2}}};
  const MonomialBasisElement m1_copy{m1};
  EXPECT_EQ(m1, m1_copy);

  // m₁.pow_in_place modifies m₁.
  m1.pow_in_place(2);
  EXPECT_NE(m1, m1_copy);
  EXPECT_EQ(m1, MonomialBasisElement({{var_x_, 2}, {var_y_, 4}}));
  EXPECT_EQ(m1.total_degree(), 6);

  // m₁ gets m₁⁰, which is 1.
  m1.pow_in_place(0);
  EXPECT_EQ(m1, MonomialBasisElement());
  EXPECT_EQ(m1.total_degree(), 0);
}

TEST_F(MonomialBasisElementTest, Evaluate) {
  const std::vector<Environment> environments{
      {{var_x_, 1.0}, {var_y_, 2.0}, {var_z_, 3.0}},     // + + +
      {{var_x_, -1.0}, {var_y_, 2.0}, {var_z_, 3.0}},    // - + +
      {{var_x_, 1.0}, {var_y_, -2.0}, {var_z_, 3.0}},    // + - +
      {{var_x_, 1.0}, {var_y_, 2.0}, {var_z_, -3.0}},    // + + -
      {{var_x_, -1.0}, {var_y_, -2.0}, {var_z_, 3.0}},   // - - +
      {{var_x_, 1.0}, {var_y_, -2.0}, {var_z_, -3.0}},   // + - -
      {{var_x_, -1.0}, {var_y_, 2.0}, {var_z_, -3.0}},   // - + -
      {{var_x_, -1.0}, {var_y_, -2.0}, {var_z_, -3.0}},  // - - -
  };
  for (const MonomialBasisElement& m : monomials_) {
    for (const Environment& env : environments) {
      EXPECT_TRUE(CheckEvaluate(m, env));
    }
  }
}

TEST_F(MonomialBasisElementTest, EvaluateException) {
  const MonomialBasisElement m{{{var_x_, 1}, {var_y_, 2}}};  // xy^2
  const Environment env{{{var_x_, 1.0}}};
  double dummy{};
  EXPECT_THROW(dummy = m.Evaluate(env), std::invalid_argument);
  unused(dummy);
}

TEST_F(MonomialBasisElementTest, EvaluatePartial) {
  const std::vector<Environment> environments{
      {{var_x_, 2.0}},
      {{var_y_, 3.0}},
      {{var_z_, 4.0}},
      {{var_x_, 2.0}, {var_y_, -2.0}},
      {{var_y_, -4.0}, {var_z_, 2.5}},
      {{var_x_, -2.3}, {var_z_, 2.6}},
      {{var_x_, -1.0}, {var_y_, 2.0}, {var_z_, 3.0}},
  };
  for (const MonomialBasisElement& m : monomials_) {
    for (const Environment& env : environments) {
      EXPECT_TRUE(CheckEvaluatePartial(m, env));
    }
  }
}

TEST_F(MonomialBasisElementTest, DegreeVariable) {
  EXPECT_EQ(MonomialBasisElement().degree(var_x_), 0);
  EXPECT_EQ(MonomialBasisElement().degree(var_y_), 0);

  EXPECT_EQ(MonomialBasisElement(var_x_).degree(var_x_), 1);
  EXPECT_EQ(MonomialBasisElement(var_x_).degree(var_y_), 0);

  EXPECT_EQ(MonomialBasisElement(var_x_, 4).degree(var_x_), 4);
  EXPECT_EQ(MonomialBasisElement(var_x_, 4).degree(var_y_), 0);

  EXPECT_EQ(MonomialBasisElement({{var_x_, 1}, {var_y_, 2}}).degree(var_x_), 1);
  EXPECT_EQ(MonomialBasisElement({{var_x_, 1}, {var_y_, 2}}).degree(var_y_), 2);
}

TEST_F(MonomialBasisElementTest, TotalDegree) {
  EXPECT_EQ(MonomialBasisElement().total_degree(), 0);
  EXPECT_EQ(MonomialBasisElement(var_x_).total_degree(), 1);
  EXPECT_EQ(MonomialBasisElement(var_x_, 1).total_degree(), 1);
  EXPECT_EQ(MonomialBasisElement(var_x_, 4).total_degree(), 4);
  EXPECT_EQ(MonomialBasisElement({{var_x_, 1}, {var_y_, 2}}).total_degree(), 3);
}

TEST_F(MonomialBasisElementTest, Differentiate) {
  // d1/dx = 0
  const std::map<MonomialBasisElement, double> result1 =
      MonomialBasisElement().Differentiate(var_x_);
  EXPECT_TRUE(result1.empty());

  // dxy / dz = 0
  const std::map<MonomialBasisElement, double> result2 =
      MonomialBasisElement{{{var_x_, 1}, {var_y_, 1}}}.Differentiate(var_z_);
  EXPECT_TRUE(result2.empty());

  // dx / dx = 1
  const auto result3 = MonomialBasisElement(var_x_).Differentiate(var_x_);
  EXPECT_EQ(result3.size(), 1);
  EXPECT_EQ(result3.at(MonomialBasisElement()), 1);

  // dxy / dy = x
  const auto result4 =
      MonomialBasisElement({{var_x_, 1}, {var_y_, 1}}).Differentiate(var_y_);
  EXPECT_EQ(result4.size(), 1);
  EXPECT_EQ(result4.at(MonomialBasisElement(var_x_)), 1);

  // dx²y²z / dx = 2xy²z
  const auto result5 =
      MonomialBasisElement({{var_x_, 2}, {var_y_, 2}, {var_z_, 1}})
          .Differentiate(var_x_);
  EXPECT_EQ(result5.size(), 1);
  EXPECT_EQ(
      result5.at(MonomialBasisElement({{var_x_, 1}, {var_y_, 2}, {var_z_, 1}})),
      2);
  // x³y⁴z² / dy = 4x³y³z²
  const auto result6 =
      MonomialBasisElement({{var_x_, 3}, {var_y_, 4}, {var_z_, 2}})
          .Differentiate(var_y_);
  EXPECT_EQ(result6.size(), 1);
  EXPECT_EQ(
      result6.at(MonomialBasisElement({{var_x_, 3}, {var_y_, 3}, {var_z_, 2}})),
      4);
}

TEST_F(MonomialBasisElementTest, Integrate) {
  // ∫1 dx = x
  const std::map<MonomialBasisElement, double> result1 =
      MonomialBasisElement().Integrate(var_x_);
  EXPECT_EQ(result1.size(), 1);
  EXPECT_EQ(result1.at(MonomialBasisElement(var_x_)), 1);

  // ∫ x dy = xy
  const std::map<MonomialBasisElement, double> result2 =
      MonomialBasisElement(var_x_).Integrate(var_y_);
  EXPECT_EQ(result2.size(), 1);
  EXPECT_EQ(result2.at(MonomialBasisElement({{var_x_, 1}, {var_y_, 1}})), 1);

  // ∫x dx = x²/2
  const auto result3 = MonomialBasisElement(var_x_).Integrate(var_x_);
  EXPECT_EQ(result3.size(), 1);
  EXPECT_EQ(result3.at(MonomialBasisElement(var_x_, 2)), 0.5);

  // ∫ xy² dy = 1/3 xy³
  const auto result4 =
      MonomialBasisElement({{var_x_, 1}, {var_y_, 2}}).Integrate(var_y_);
  EXPECT_EQ(result4.size(), 1);
  EXPECT_EQ(result4.at(MonomialBasisElement({{var_x_, 1}, {var_y_, 3}})),
            1. / 3);

  // ∫ x²y³z dy = 1/4 x²y⁴z
  const auto result5 =
      MonomialBasisElement({{var_x_, 2}, {var_y_, 3}, {var_z_, 1}})
          .Integrate(var_y_);
  EXPECT_EQ(result5.size(), 1);
  EXPECT_EQ(
      result5.at(MonomialBasisElement({{var_x_, 2}, {var_y_, 4}, {var_z_, 1}})),
      1. / 4);
}

TEST_F(MonomialBasisElementTest, ToChebyshevBasisUnivariate) {
  // Test ToChebyshevBasis for univariate monomials.
  // 1 = T0()
  const std::map<ChebyshevBasisElement, double> result1 =
      MonomialBasisElement().ToChebyshevBasis();
  EXPECT_EQ(result1.size(), 1);
  EXPECT_EQ(result1.at(ChebyshevBasisElement()), 1.);

  // x = T1(x)
  const std::map<ChebyshevBasisElement, double> result2 =
      MonomialBasisElement(var_x_).ToChebyshevBasis();
  EXPECT_EQ(result2.size(), 1);
  EXPECT_EQ(result2.at(ChebyshevBasisElement(var_x_)), 1.);

  // x² = 0.5T₂(x)+0.5T₀(x)
  const std::map<ChebyshevBasisElement, double> result3 =
      MonomialBasisElement(var_x_, 2).ToChebyshevBasis();
  EXPECT_EQ(result3.size(), 2);
  EXPECT_EQ(result3.at(ChebyshevBasisElement(var_x_, 2)), 0.5);
  EXPECT_EQ(result3.at(ChebyshevBasisElement()), 0.5);

  // x³ = 0.25T₃(x)+0.75T₁(x)
  const std::map<ChebyshevBasisElement, double> result4 =
      MonomialBasisElement(var_x_, 3).ToChebyshevBasis();
  EXPECT_EQ(result4.size(), 2);
  EXPECT_EQ(result4.at(ChebyshevBasisElement(var_x_, 3)), 0.25);
  EXPECT_EQ(result4.at(ChebyshevBasisElement(var_x_, 1)), 0.75);

  // x⁴ = 1/8T₄(x)+1/2T₂(x)+3/8T₀(x)
  const std::map<ChebyshevBasisElement, double> result5 =
      MonomialBasisElement(var_x_, 4).ToChebyshevBasis();
  EXPECT_EQ(result5.size(), 3);
  EXPECT_EQ(result5.at(ChebyshevBasisElement(var_x_, 4)), 1.0 / 8);
  EXPECT_EQ(result5.at(ChebyshevBasisElement(var_x_, 2)), 1.0 / 2);
  EXPECT_EQ(result5.at(ChebyshevBasisElement(var_x_, 0)), 3.0 / 8);

  // The coefficient of the Chebyshev polynomial of T20(x) for x^20 is 1/(2^19).
  const std::map<ChebyshevBasisElement, double> result6 =
      MonomialBasisElement(var_x_, 20).ToChebyshevBasis();
  EXPECT_EQ(result6.at(ChebyshevBasisElement(var_x_, 20)), std::pow(0.5, 19));
}

TEST_F(MonomialBasisElementTest, ToChebyshevBasisMultivariate) {
  // xy = T1(x)T1(y)
  const std::map<ChebyshevBasisElement, double> result1 =
      MonomialBasisElement({{var_x_, 1}, {var_y_, 1}}).ToChebyshevBasis();
  EXPECT_EQ(result1.size(), 1);
  EXPECT_EQ(result1.at(ChebyshevBasisElement({{var_x_, 1}, {var_y_, 1}})), 1.);

  // xyz = T1(x)T1(y)T1(z)
  const std::map<ChebyshevBasisElement, double> result2 =
      MonomialBasisElement({{var_x_, 1}, {var_y_, 1}, {var_z_, 1}})
          .ToChebyshevBasis();
  EXPECT_EQ(result2.size(), 1);
  EXPECT_EQ(result2.at(
                ChebyshevBasisElement({{var_x_, 1}, {var_y_, 1}, {var_z_, 1}})),
            1.);

  // x²y = 0.5T₂(x)T₁(y)+0.5T₁(y)
  const std::map<ChebyshevBasisElement, double> result3 =
      MonomialBasisElement({{var_x_, 2}, {var_y_, 1}}).ToChebyshevBasis();
  EXPECT_EQ(result3.size(), 2);
  EXPECT_EQ(result3.at(ChebyshevBasisElement({{var_x_, 2}, {var_y_, 1}})), 0.5);
  EXPECT_EQ(result3.at(ChebyshevBasisElement(var_y_)), 0.5);

  // x³y²z=1/8T₃(x)T₂(y)T₁(z)+3/8T₁(x)T₂(y)T₁(z)+1/8T₃(x)T₁(z)+3/8T₁(x)T₁(z)
  const std::map<ChebyshevBasisElement, double> result4 =
      MonomialBasisElement({{var_x_, 3}, {var_y_, 2}, {var_z_, 1}})
          .ToChebyshevBasis();
  EXPECT_EQ(result4.size(), 4);
  EXPECT_EQ(result4.at(
                ChebyshevBasisElement({{var_x_, 3}, {var_y_, 2}, {var_z_, 1}})),
            1.0 / 8);
  EXPECT_EQ(result4.at(
                ChebyshevBasisElement({{var_x_, 1}, {var_y_, 2}, {var_z_, 1}})),
            3.0 / 8);
  EXPECT_EQ(result4.at(ChebyshevBasisElement({{var_x_, 3}, {var_z_, 1}})),
            1.0 / 8);
  EXPECT_EQ(result4.at(ChebyshevBasisElement({{var_x_, 1}, {var_z_, 1}})),
            3.0 / 8);
}

TEST_F(MonomialBasisElementTest, MergeBasisElementInPlace) {
  // x²y³ * xyz² = x³y⁴z²
  MonomialBasisElement basis_element1({{var_x_, 2}, {var_y_, 3}});
  basis_element1.MergeBasisElementInPlace(
      MonomialBasisElement({{var_x_, 1}, {var_y_, 1}, {var_z_, 2}}));
  EXPECT_EQ(basis_element1.var_to_degree_map().size(), 3);
  EXPECT_EQ(basis_element1.var_to_degree_map().at(var_x_), 3);
  EXPECT_EQ(basis_element1.var_to_degree_map().at(var_y_), 4);
  EXPECT_EQ(basis_element1.var_to_degree_map().at(var_z_), 2);
  EXPECT_EQ(basis_element1.total_degree(), 9);
}
}  // namespace symbolic
}  // namespace drake
