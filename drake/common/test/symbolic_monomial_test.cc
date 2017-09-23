#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

using std::map;
using std::ostringstream;
using std::out_of_range;
using std::pair;
using std::runtime_error;
using std::unordered_map;
using std::vector;

namespace drake {
namespace symbolic {
namespace {

using test::ExprEqual;
using test::VarLess;

class MonomialTest : public ::testing::Test {
 protected:
  const Variable var_w_{"w"};
  const Variable var_z_{"z"};
  const Variable var_y_{"y"};
  const Variable var_x_{"x"};

  const Expression w_{var_w_};
  const Expression z_{var_z_};
  const Expression y_{var_y_};
  const Expression x_{var_x_};

  vector<Monomial> monomials_;

  void SetUp() override {
    monomials_ = {
        Monomial{},                                         // 1.0
        Monomial{var_x_, 1},                                // x^1
        Monomial{var_y_, 1},                                // y^1
        Monomial{var_z_, 1},                                // z^1
        Monomial{var_x_, 2},                                // x^2
        Monomial{var_y_, 3},                                // y^3
        Monomial{var_z_, 4},                                // z^4
        Monomial{{{var_x_, 1}, {var_y_, 2}}},               // xy^2
        Monomial{{{var_y_, 2}, {var_z_, 5}}},               // y^2z^5
        Monomial{{{var_x_, 1}, {var_y_, 2}, {var_z_, 3}}},  // xy^2z^3
        Monomial{{{var_x_, 2}, {var_y_, 4}, {var_z_, 3}}},  // x^2y^4z^3
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
    for (const pair<Variable, double>& p : env) {
      subst.emplace(p.first, p.second);
    }
    return subst;
  }

  // Checks if Monomial::Evaluate corresponds to Expression::Evaluate.
  //
  //                            ToExpression
  //                   Monomial ------------> Expression
  //                      ||                      ||
  //   Monomial::Evaluate ||                      || Expression::Evaluate
  //                      ||                      ||
  //                      \/                      \/
  //                double (result2)   ==   double (result1)
  //
  // In one direction (left-to-right), we convert a Monomial to an Expression
  // and evaluate it to a double value (result1). In another direction
  // (top-to-bottom), we directly evaluate a Monomial to double (result2). The
  // two result1 and result2 should be the same.
  bool CheckEvaluate(const Monomial& m, const Environment& env) {
    const double result1{m.ToExpression().Evaluate(env)};
    const double result2{m.Evaluate(env)};
    return result1 == result2;
  }

  // Checks if Monomial::Substitute corresponds to Expression::Substitute.
  //
  //                            ToExpression
  //                   Monomial ------------> Expression
  //                      ||                      ||
  // Monomial::Substitute ||                      || Expression::Substitute
  //                      ||                      ||
  //                      \/                      \/
  //          double * Monomial (e2)   ==   Expression (e1)
  //
  // In one direction (left-to-right), first we convert a Monomial to an
  // Expression using Monomial::ToExpression and call Expression::Substitution
  // to have an Expression (e1). In another direction (top-to-bottom), we call
  // Monomial::Substitution which returns a pair of double (coefficient part)
  // and Monomial. We obtain e2 by multiplying the two. Then, we check if e1 and
  // e2 are structurally equal.
  bool CheckSubstitute(const Monomial& m, const Environment& env) {
    const Substitution subst{ExtractSubst(env)};

    const Expression e1{m.ToExpression().Substitute(subst)};

    const pair<double, Monomial> subst_result{m.Substitute(env)};
    const Expression e2{subst_result.first *
                        subst_result.second.ToExpression()};

    return e1.EqualTo(e2);
  }
};

TEST_F(MonomialTest, ConstructFromVariable) {
  const Monomial m1{var_x_};
  const std::map<Variable, int> powers{m1.get_powers()};

  // Checks that powers = {x ↦ 1}.
  ASSERT_EQ(powers.size(), 1u);
  EXPECT_EQ(powers.begin()->first, var_x_);
  EXPECT_EQ(powers.begin()->second, 1);
}

TEST_F(MonomialTest, GetVariables) {
  const Monomial m0{};
  EXPECT_EQ(m0.GetVariables(), Variables{});

  const Monomial m1{var_z_, 4};
  EXPECT_EQ(m1.GetVariables(), Variables({var_z_}));

  const Monomial m2{{{var_x_, 1}, {var_y_, 2}}};
  EXPECT_EQ(m2.GetVariables(), Variables({var_x_, var_y_}));

  const Monomial m3{{{var_x_, 1}, {var_y_, 2}, {var_z_, 3}}};
  EXPECT_EQ(m3.GetVariables(), Variables({var_x_, var_y_, var_z_}));
}

// Checks we can have an Eigen matrix of Monomials without compilation
// errors. No assertions in the test.
TEST_F(MonomialTest, EigenMatrixOfMonomials) {
  Eigen::Matrix<Monomial, 2, 2> M;
  // M = | 1   x    |
  //     | y²  x²z³ |
  // clang-format off
  M << Monomial{}, Monomial{var_x_},
       Monomial{{{var_y_, 2}}}, Monomial{{{var_x_, 2}, {var_z_, 3}}};
  // clang-format on

  // The following fails if we do not provide
  // `Eigen::NumTraits<drake::symbolic::Monomial>`.
  ostringstream oss;
  oss << M;
}

TEST_F(MonomialTest, MonomialOne) {
  // Compares monomials all equal to 1, but with different variables.
  Monomial m1{};
  Monomial m2({{var_x_, 0}});
  Monomial m3({{var_x_, 0}, {var_y_, 0}});
  EXPECT_EQ(m1, m2);
  EXPECT_EQ(m1, m3);
  EXPECT_EQ(m2, m3);
}

TEST_F(MonomialTest, MonomialWithZeroExponent) {
  // Compares monomials containing zero exponent, such as x^0 * y^2
  Monomial m1({{var_y_, 2}});
  Monomial m2({{var_x_, 0}, {var_y_, 2}});
  EXPECT_EQ(m1, m2);
  EXPECT_EQ(m2.get_powers().size(), 1);
  std::map<Variable, int> power_expected;
  power_expected.emplace(var_y_, 2);
  EXPECT_EQ(m2.get_powers(), power_expected);
}

TEST_F(MonomialTest, MonomialBasis_x_0) {
  const drake::VectorX<Monomial> basis1{MonomialBasis({var_x_}, 0)};
  const auto basis2 = MonomialBasis<1, 0>({var_x_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 1);
  drake::Vector1<Monomial> expected;

  // MonomialBasis({x}, 0) = {x⁰} = {1}.
  expected << Monomial{};

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_2) {
  const drake::VectorX<Monomial> basis1{MonomialBasis({var_x_}, 2)};
  const auto basis2 = MonomialBasis<1, 2>({var_x_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 3);
  drake::Vector3<Monomial> expected;

  // MonomialBasis({x}, 2) = {x², x, 1}.
  // clang-format off
  expected << Monomial{var_x_, 2},
              Monomial{var_x_},
              Monomial{};
  // clang-format on
  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_y_0) {
  const drake::VectorX<Monomial> basis1{MonomialBasis({var_x_, var_y_}, 0)};
  const auto basis2 = MonomialBasis<2, 0>({var_x_, var_y_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 1);
  drake::Vector1<Monomial> expected;

  // MonomialBasis({x, y}, 0) = {1}.
  expected << Monomial{{{var_x_, 0}, {var_y_, 0}}};  // = {1}.

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_y_1) {
  const drake::VectorX<Monomial> basis1{MonomialBasis({var_x_, var_y_}, 1)};
  const auto basis2 = MonomialBasis<2, 1>({var_x_, var_y_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 3);
  drake::Vector3<Monomial> expected;

  // MonomialBasis({x, y}, 1) = {x, y, 1}.
  // clang-format off
  expected << Monomial{var_x_},
              Monomial{var_y_},
              Monomial{};
  // clang-format on

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_y_2) {
  const drake::VectorX<Monomial> basis1{MonomialBasis({var_x_, var_y_}, 2)};
  const auto basis2 = MonomialBasis<2, 2>({var_x_, var_y_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 6);
  drake::Vector6<Monomial> expected;

  // MonomialBasis({x, y}, 2) = {x², xy, y², x, y, 1}.
  // clang-format off
  expected << Monomial{var_x_, 2},
              Monomial{{{var_x_, 1}, {var_y_, 1}}},
              Monomial{var_y_, 2},
              Monomial{var_x_},
              Monomial{var_y_},
              Monomial{};
  // clang-format on

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_y_3) {
  const drake::VectorX<Monomial> basis1{MonomialBasis({var_x_, var_y_}, 3)};
  const auto basis2 = MonomialBasis<2, 3>({var_x_, var_y_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 10);
  Eigen::Matrix<Monomial, 10, 1> expected;

  // MonomialBasis({x, y}, 3) = {x³, x²y, xy², y³, x², xy, y², x, y, 1}.
  // clang-format off
  expected << Monomial{var_x_, 3},
              Monomial{{{var_x_, 2}, {var_y_, 1}}},
              Monomial{{{var_x_, 1}, {var_y_, 2}}},
              Monomial{var_y_, 3},
              Monomial{var_x_, 2},
              Monomial{{{var_x_, 1}, {var_y_, 1}}},
              Monomial{var_y_, 2},
              Monomial{var_x_},
              Monomial{var_y_},
              Monomial{};
  // clang-format on

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_y_z_2) {
  const drake::VectorX<Monomial> basis1{
      MonomialBasis({var_x_, var_y_, var_z_}, 2)};
  const auto basis2 = MonomialBasis<3, 2>({var_x_, var_y_, var_z_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 10);

  Eigen::Matrix<Monomial, 10, 1> expected;

  // MonomialBasis({x, y, z}, 2)
  // clang-format off
  expected << Monomial{var_x_, 2},
              Monomial{{{var_x_, 1}, {var_y_, 1}}},
              Monomial{var_y_, 2},
              Monomial{{{var_x_, 1}, {var_z_, 1}}},
              Monomial{{{var_y_, 1}, {var_z_, 1}}},
              Monomial{var_z_, 2},
              Monomial{var_x_},
              Monomial{var_y_},
              Monomial{var_z_},
              Monomial{};
  // clang-format on

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_y_z_3) {
  const drake::VectorX<Monomial> basis1{
      MonomialBasis({var_x_, var_y_, var_z_}, 3)};
  const auto basis2 = MonomialBasis<3, 3>({var_x_, var_y_, var_z_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 20);
  Eigen::Matrix<Monomial, 20, 1> expected;

  // MonomialBasis({x, y, z}, 3)
  // = {x³, x²y, xy², y³, x²z, xyz, y²z, xz², yz²,
  //    z³, x², xy, y², xz, yz, z², x, y, z, 1}
  // clang-format off
  expected << Monomial{var_x_, 3},
              Monomial{{{var_x_, 2}, {var_y_, 1}}},
              Monomial{{{var_x_, 1}, {var_y_, 2}}},
              Monomial{var_y_, 3},
              Monomial{{{var_x_, 2}, {var_z_, 1}}},
              Monomial{{{var_x_, 1}, {var_y_, 1}, {var_z_, 1}}},
              Monomial{{{var_y_, 2}, {var_z_, 1}}},
              Monomial{{{var_x_, 1}, {var_z_, 2}}},
              Monomial{{{var_y_, 1}, {var_z_, 2}}},
              Monomial{var_z_, 3},
              Monomial{var_x_, 2},
              Monomial{{{var_x_, 1}, {var_y_, 1}}},
              Monomial{var_y_, 2},
              Monomial{{{var_x_, 1}, {var_z_, 1}}},
              Monomial{{{var_y_, 1}, {var_z_, 1}}},
              Monomial{var_z_, 2},
              Monomial{var_x_},
              Monomial{var_y_},
              Monomial{var_z_},
              Monomial{};
  // clang-format on

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

TEST_F(MonomialTest, MonomialBasis_x_y_z_w_3) {
  const drake::VectorX<Monomial> basis1{
      MonomialBasis({var_x_, var_y_, var_z_, var_w_}, 3)};
  const auto basis2 = MonomialBasis<4, 3>({var_x_, var_y_, var_z_, var_w_});
  EXPECT_EQ(decltype(basis2)::RowsAtCompileTime, 35);
  Eigen::Matrix<Monomial, 35, 1> expected;

  // MonomialBasis({x, y, z, w}, 3)
  // clang-format off
  expected << Monomial{var_x_, 3},
              Monomial{{{var_x_, 2}, {var_y_, 1}}},
              Monomial{{{var_x_, 1}, {var_y_, 2}}},
              Monomial{var_y_, 3},
              Monomial{{{var_x_, 2}, {var_z_, 1}}},
              Monomial{{{var_x_, 1}, {var_y_, 1}, {var_z_, 1}}},
              Monomial{{{var_y_, 2}, {var_z_, 1}}},
              Monomial{{{var_x_, 1}, {var_z_, 2}}},
              Monomial{{{var_y_, 1}, {var_z_, 2}}},
              Monomial{var_z_, 3},
              Monomial{{{var_x_, 2}, {var_w_, 1}}},
              Monomial{{{var_y_, 1}, {var_x_, 1}, {var_w_, 1}}},
              Monomial{{{var_y_, 2}, {var_w_, 1}}},
              Monomial{{{var_z_, 1}, {var_x_, 1}, {var_w_, 1}}},
              Monomial{{{var_z_, 1}, {var_y_, 1}, {var_w_, 1}}},
              Monomial{{{var_z_, 2}, {var_w_, 1}}},
              Monomial{{{var_x_, 1}, {var_w_, 2}}},
              Monomial{{{var_y_, 1}, {var_w_, 2}}},
              Monomial{{{var_z_, 1}, {var_w_, 2}}},
              Monomial{var_w_, 3},
              Monomial{var_x_, 2},
              Monomial{{{var_x_, 1}, {var_y_, 1}}},
              Monomial{var_y_, 2},
              Monomial{{{var_x_, 1}, {var_z_, 1}}},
              Monomial{{{var_y_, 1}, {var_z_, 1}}},
              Monomial{var_z_, 2},
              Monomial{{{var_x_, 1}, {var_w_, 1}}},
              Monomial{{{var_y_, 1}, {var_w_, 1}}},
              Monomial{{{var_z_, 1}, {var_w_, 1}}},
              Monomial{var_w_, 2},
              Monomial{var_x_},
              Monomial{var_y_},
              Monomial{var_z_},
              Monomial{var_w_},
              Monomial{};
  // clang-format on

  EXPECT_EQ(basis1, expected);
  EXPECT_EQ(basis2, expected);
}

// This test shows that we can have a std::unordered_map whose key is of
// Monomial.
TEST_F(MonomialTest, UnorderedMapOfMonomial) {
  unordered_map<Monomial, double, hash_value<Monomial>> monomial_to_coeff_map;
  Monomial x_3{var_x_, 3};
  Monomial y_5{var_y_, 5};
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
TEST_F(MonomialTest, ToMonomial0) {
  Monomial expected;
  EXPECT_EQ(Monomial(1), expected);
  EXPECT_EQ(Monomial(pow(x_, 0)), expected);
  EXPECT_THROW(Monomial(2), std::exception);
}

// Converts expression x to monomial.
TEST_F(MonomialTest, ToMonomial1) {
  Monomial expected(var_x_, 1);
  EXPECT_EQ(Monomial(x_), expected);
}

// Converts expression x * y to monomial.
TEST_F(MonomialTest, ToMonomial2) {
  std::map<Variable, int> powers;
  powers.emplace(var_x_, 1);
  powers.emplace(var_y_, 1);
  Monomial expected(powers);
  EXPECT_EQ(Monomial(x_ * y_), expected);
}

// Converts expression x^3 to monomial.
TEST_F(MonomialTest, ToMonomial3) {
  Monomial expected(var_x_, 3);
  EXPECT_EQ(Monomial(pow(x_, 3)), expected);
  EXPECT_EQ(Monomial(pow(x_, 2) * x_), expected);
}

// Converts expression x^3 * y to monomial.
TEST_F(MonomialTest, ToMonomial4) {
  std::map<Variable, int> powers;
  powers.emplace(var_x_, 3);
  powers.emplace(var_y_, 1);
  Monomial expected(powers);
  EXPECT_EQ(Monomial(pow(x_, 3) * y_), expected);
  EXPECT_EQ(Monomial(pow(x_, 2) * y_ * x_), expected);
}

// Converts expression x*(y+z) - x*y to monomial
TEST_F(MonomialTest, ToMonomial5) {
  std::map<Variable, int> powers({{var_x_, 1}, {var_z_, 1}});
  Monomial expected(powers);
  EXPECT_EQ(Monomial(x_ * z_), expected);
  EXPECT_EQ(Monomial(x_ * (y_ + z_) - x_ * y_), expected);
}

// `pow(x * y, 2) * pow(x^2 * y^2, 3)` is a monomial (x^8 * y^8).
TEST_F(MonomialTest, ToMonomial6) {
  const Monomial m1{pow(x_ * y_, 2) * pow(x_ * x_ * y_ * y_, 3)};
  const Monomial m2{{{var_x_, 8}, {var_y_, 8}}};
  EXPECT_EQ(m1, m2);
}

// x^0 is a monomial (1).
TEST_F(MonomialTest, ToMonomial7) {
  const Monomial m1(var_x_, 0);
  const Monomial m2{Expression{1.0}};
  const Monomial m3{pow(x_, 0.0)};
  EXPECT_EQ(m1, m2);
  EXPECT_EQ(m1, m3);
  EXPECT_EQ(m2, m3);
}

// `2 * x` is not a monomial because of its coefficient `2`.
TEST_F(MonomialTest, ToMonomialException1) {
  EXPECT_THROW(Monomial{2 * x_}, runtime_error);
}

// `x + y` is not a monomial.
TEST_F(MonomialTest, ToMonomialException2) {
  EXPECT_THROW(Monomial{x_ + y_}, runtime_error);
}

// `x / 2.0` is not a monomial.
TEST_F(MonomialTest, ToMonomialException3) {
  EXPECT_THROW(Monomial{x_ / 2.0}, runtime_error);
}

// `x ^ -1` is not a monomial.
TEST_F(MonomialTest, ToMonomialException4) {
  // Note: Parentheses are required around macro argument containing braced
  // initializer list.
  EXPECT_THROW((Monomial{{{var_x_, -1}}}), runtime_error);
}

TEST_F(MonomialTest, Multiplication) {
  // m₁ = xy²
  const Monomial m1{{{var_x_, 1}, {var_y_, 2}}};
  // m₂ = y²z⁵
  const Monomial m2{{{var_y_, 2}, {var_z_, 5}}};
  // m₃ = m₁ * m₂ = xy⁴z⁵
  const Monomial m3{{{var_x_, 1}, {var_y_, 4}, {var_z_, 5}}};
  EXPECT_EQ(m1 * m2, m3);

  Monomial m{m1};  // m = m₁
  m *= m2;         // m = m₁ * m₂
  EXPECT_EQ(m, m1 * m2);
}

TEST_F(MonomialTest, Pow) {
  // m₁ = xy²
  const Monomial m1{{{var_x_, 1}, {var_y_, 2}}};
  // m₂ = y²z⁵
  const Monomial m2{{{var_y_, 2}, {var_z_, 5}}};

  // pow(m₁, 0) = 1.0
  EXPECT_EQ(pow(m1, 0), Monomial{});
  // pow(m₂, 0) = 1.0
  EXPECT_EQ(pow(m2, 0), Monomial{});

  // pow(m₁, 1) = m₁
  EXPECT_EQ(pow(m1, 1), m1);
  // pow(m₂, 1) = m₂
  EXPECT_EQ(pow(m2, 1), m2);

  // pow(m₁, 3) = x³y⁶
  EXPECT_EQ(pow(m1, 3), Monomial{pow(x_, 3) * pow(y_, 6)});
  // pow(m₂, 4) = y⁸z²⁰
  EXPECT_EQ(pow(m2, 4), Monomial{pow(y_, 8) * pow(z_, 20)});

  // pow(m₁, -1) throws an exception.
  EXPECT_THROW(pow(m1, -1), runtime_error);
  // pow(m₂, -5) throws an exception.
  EXPECT_THROW(pow(m2, -5), runtime_error);
}

TEST_F(MonomialTest, PowInPlace) {
  // m₁ = xy²
  Monomial m1{{{var_x_, 1}, {var_y_, 2}}};
  const Monomial m1_copy{m1};
  EXPECT_EQ(m1, m1_copy);

  // m₁.pow_in_place modifies m₁.
  m1.pow_in_place(2);
  EXPECT_NE(m1, m1_copy);
  EXPECT_EQ(m1, Monomial({{var_x_, 2}, {var_y_, 4}}));

  // m₁ gets m₁⁰, which is 1.
  m1.pow_in_place(0);
  EXPECT_EQ(m1, Monomial());
}

TEST_F(MonomialTest, Evaluate) {
  const vector<Environment> environments{
      {{var_x_, 1.0}, {var_y_, 2.0}, {var_z_, 3.0}},     // + + +
      {{var_x_, -1.0}, {var_y_, 2.0}, {var_z_, 3.0}},    // - + +
      {{var_x_, 1.0}, {var_y_, -2.0}, {var_z_, 3.0}},    // + - +
      {{var_x_, 1.0}, {var_y_, 2.0}, {var_z_, -3.0}},    // + + -
      {{var_x_, -1.0}, {var_y_, -2.0}, {var_z_, 3.0}},   // - - +
      {{var_x_, 1.0}, {var_y_, -2.0}, {var_z_, -3.0}},   // + - -
      {{var_x_, -1.0}, {var_y_, 2.0}, {var_z_, -3.0}},   // - + -
      {{var_x_, -1.0}, {var_y_, -2.0}, {var_z_, -3.0}},  // - - -
  };
  for (const Monomial m : monomials_) {
    for (const Environment env : environments) {
      EXPECT_TRUE(CheckEvaluate(m, env));
    }
  }
}

TEST_F(MonomialTest, EvaluateException) {
  const Monomial m{{{var_x_, 1}, {var_y_, 2}}};  // xy^2
  const Environment env{{{var_x_, 1.0}}};
  EXPECT_THROW(m.Evaluate(env), out_of_range);
}

TEST_F(MonomialTest, Substitute) {
  const vector<Environment> environments{
      {{var_x_, 2.0}},
      {{var_y_, 3.0}},
      {{var_z_, 4.0}},
      {{var_x_, 2.0}, {var_y_, -2.0}},
      {{var_y_, -4.0}, {var_z_, 2.5}},
      {{var_x_, -2.3}, {var_z_, 2.6}},
      {{var_x_, -1.0}, {var_y_, 2.0}, {var_z_, 3.0}},
  };
  for (const Monomial m : monomials_) {
    for (const Environment env : environments) {
      EXPECT_TRUE(CheckSubstitute(m, env));
    }
  }
}

TEST_F(MonomialTest, DegreeVariable) {
  EXPECT_EQ(Monomial().degree(var_x_), 0);
  EXPECT_EQ(Monomial().degree(var_y_), 0);

  EXPECT_EQ(Monomial(var_x_).degree(var_x_), 1);
  EXPECT_EQ(Monomial(var_x_).degree(var_y_), 0);

  EXPECT_EQ(Monomial(var_x_, 4).degree(var_x_), 4);
  EXPECT_EQ(Monomial(var_x_, 4).degree(var_y_), 0);

  EXPECT_EQ(Monomial({{var_x_, 1}, {var_y_, 2}}).degree(var_x_), 1);
  EXPECT_EQ(Monomial({{var_x_, 1}, {var_y_, 2}}).degree(var_y_), 2);
}

TEST_F(MonomialTest, TotalDegree) {
  EXPECT_EQ(Monomial().total_degree(), 0);
  EXPECT_EQ(Monomial(var_x_).total_degree(), 1);
  EXPECT_EQ(Monomial(var_x_, 1).total_degree(), 1);
  EXPECT_EQ(Monomial(var_x_, 4).total_degree(), 4);
  EXPECT_EQ(Monomial({{var_x_, 1}, {var_y_, 2}}).total_degree(), 3);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
