#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
class SymbolicChebyshevBasisElementTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
};

TEST_F(SymbolicChebyshevBasisElementTest, LessThan) {
  const ChebyshevBasisElement p1({{x_, 1}, {y_, 2}});
  const ChebyshevBasisElement p2({{y_, 3}});
  const ChebyshevBasisElement p3({{x_, 3}});
  const ChebyshevBasisElement p4({{z_, 1}});

  EXPECT_LT(p2, p1);
  EXPECT_LT(p1, p3);
  EXPECT_LT(p4, p1);
  EXPECT_LT(p2, p3);
  EXPECT_LT(p4, p3);
}

TEST_F(SymbolicChebyshevBasisElementTest, Constructor) {
  const ChebyshevBasisElement b1{};
  EXPECT_TRUE(b1.var_to_degree_map().empty());
  EXPECT_EQ(b1.total_degree(), 0);

  const ChebyshevBasisElement b2{x_};
  EXPECT_EQ(b2.var_to_degree_map().size(), 1);
  EXPECT_EQ(b2.var_to_degree_map().at(x_), 1);
  EXPECT_EQ(b2.total_degree(), 1);

  const ChebyshevBasisElement b3{x_, 2};
  EXPECT_EQ(b3.var_to_degree_map().size(), 1);
  EXPECT_EQ(b3.var_to_degree_map().at(x_), 2);
  EXPECT_EQ(b3.total_degree(), 2);

  const ChebyshevBasisElement b4{{{x_, 2}, {y_, 1}}};
  EXPECT_EQ(b4.var_to_degree_map().size(), 2);
  EXPECT_EQ(b4.var_to_degree_map().at(x_), 2);
  EXPECT_EQ(b4.var_to_degree_map().at(y_), 1);
  EXPECT_EQ(b4.total_degree(), 3);

  const ChebyshevBasisElement b5{Vector2<Variable>(x_, y_),
                                 Eigen::Vector2i(2, 1)};
  EXPECT_EQ(b5.var_to_degree_map().size(), 2);
  EXPECT_EQ(b5.var_to_degree_map().at(x_), 2);
  EXPECT_EQ(b5.var_to_degree_map().at(y_), 1);
  EXPECT_EQ(b5.total_degree(), 3);

  const ChebyshevBasisElement b6{Vector2<Variable>(x_, y_),
                                 Eigen::Vector2i(2, 0)};
  EXPECT_EQ(b6.var_to_degree_map().size(), 1);
  EXPECT_EQ(b6.var_to_degree_map().at(x_), 2);
  EXPECT_EQ(b6.total_degree(), 2);

  DRAKE_EXPECT_THROWS_MESSAGE(
      ChebyshevBasisElement(Vector2<Variable>(x_, x_), Eigen::Vector2i(2, 1)),
      std::invalid_argument, ".*x is repeated");

  DRAKE_EXPECT_THROWS_MESSAGE(
      ChebyshevBasisElement(Vector2<Variable>(x_, y_), Eigen::Vector2i(2, -1)),
      std::logic_error, "The exponent is negative.");
}

TEST_F(SymbolicChebyshevBasisElementTest, Evaluate) {
  Environment env;
  env.insert(x_, 2);
  env.insert(y_, 3);
  env.insert(z_, 4);

  EXPECT_EQ(ChebyshevBasisElement({{x_, 1}}).Evaluate(env), 2);
  EXPECT_EQ(ChebyshevBasisElement({{x_, 1}, {y_, 2}}).Evaluate(env), 34);
  EXPECT_EQ(ChebyshevBasisElement({{x_, 1}, {y_, 2}, {z_, 3}}).Evaluate(env),
            2 * 17 * 244);
}

TEST_F(SymbolicChebyshevBasisElementTest, multiply) {
  const auto result1 =
      ChebyshevBasisElement({{x_, 1}}) * ChebyshevBasisElement({{y_, 2}});
  EXPECT_EQ(result1.size(), 1);
  EXPECT_EQ(result1.at(ChebyshevBasisElement({{x_, 1}, {y_, 2}})), 1.);

  const auto result2 =
      ChebyshevBasisElement({{x_, 1}}) * ChebyshevBasisElement({{x_, 2}});
  EXPECT_EQ(result2.size(), 2);
  EXPECT_EQ(result2.at(ChebyshevBasisElement({{x_, 3}})), 0.5);
  EXPECT_EQ(result2.at(ChebyshevBasisElement({{x_, 1}})), 0.5);

  const auto result3 = ChebyshevBasisElement({{x_, 1}, {y_, 2}}) *
                       ChebyshevBasisElement({{x_, 2}});
  EXPECT_EQ(result3.size(), 2);
  EXPECT_EQ(result3.at(ChebyshevBasisElement({{x_, 3}, {y_, 2}})), 0.5);
  EXPECT_EQ(result3.at(ChebyshevBasisElement({{x_, 1}, {y_, 2}})), 0.5);

  const auto result4 =
      ChebyshevBasisElement({{y_, 2}}) * ChebyshevBasisElement({{x_, 1}});
  EXPECT_EQ(result4.size(), 1);
  EXPECT_EQ(result4.at(ChebyshevBasisElement({{x_, 1}, {y_, 2}})), 1.);

  const auto result5 = ChebyshevBasisElement({{x_, 1}, {y_, 3}}) *
                       ChebyshevBasisElement({{x_, 2}, {y_, 1}});
  EXPECT_EQ(result5.size(), 4);
  EXPECT_EQ(result5.at(ChebyshevBasisElement({{x_, 3}, {y_, 4}})), 0.25);
  EXPECT_EQ(result5.at(ChebyshevBasisElement({{x_, 3}, {y_, 2}})), 0.25);
  EXPECT_EQ(result5.at(ChebyshevBasisElement({{x_, 1}, {y_, 4}})), 0.25);
  EXPECT_EQ(result5.at(ChebyshevBasisElement({{x_, 1}, {y_, 2}})), 0.25);

  const auto result6 = ChebyshevBasisElement({{y_, 2}, {z_, 3}}) *
                       ChebyshevBasisElement({{x_, 3}, {y_, 1}});
  EXPECT_EQ(result6.size(), 2);
  EXPECT_EQ(result6.at(ChebyshevBasisElement({{x_, 3}, {y_, 3}, {z_, 3}})),
            0.5);
  EXPECT_EQ(result6.at(ChebyshevBasisElement({{x_, 3}, {y_, 1}, {z_, 3}})),
            0.5);

  const auto result7 = ChebyshevBasisElement({{x_, 1}, {y_, 2}, {z_, 3}}) *
                       ChebyshevBasisElement({{x_, 2}, {y_, 3}, {z_, 1}});
  EXPECT_EQ(result7.size(), 8);
  EXPECT_EQ(result7.at(ChebyshevBasisElement({{x_, 3}, {y_, 5}, {z_, 4}})),
            0.125);
  EXPECT_EQ(result7.at(ChebyshevBasisElement({{x_, 3}, {y_, 5}, {z_, 2}})),
            0.125);
  EXPECT_EQ(result7.at(ChebyshevBasisElement({{x_, 3}, {y_, 1}, {z_, 4}})),
            0.125);
  EXPECT_EQ(result7.at(ChebyshevBasisElement({{x_, 3}, {y_, 1}, {z_, 2}})),
            0.125);
  EXPECT_EQ(result7.at(ChebyshevBasisElement({{x_, 1}, {y_, 5}, {z_, 4}})),
            0.125);
  EXPECT_EQ(result7.at(ChebyshevBasisElement({{x_, 1}, {y_, 5}, {z_, 2}})),
            0.125);
  EXPECT_EQ(result7.at(ChebyshevBasisElement({{x_, 1}, {y_, 1}, {z_, 4}})),
            0.125);
  EXPECT_EQ(result7.at(ChebyshevBasisElement({{x_, 1}, {y_, 1}, {z_, 2}})),
            0.125);
}

TEST_F(SymbolicChebyshevBasisElementTest, Differentiate) {
  // d1/dx = 0
  const auto result1 = ChebyshevBasisElement().Differentiate(x_);
  EXPECT_TRUE(result1.empty());

  // dT2(x) / dy = 0
  const auto result2 = ChebyshevBasisElement({{x_, 2}}).Differentiate(y_);
  EXPECT_TRUE(result2.empty());

  // dT2(x) / dx = 4 * T1(x)
  const auto result3 = ChebyshevBasisElement({{x_, 2}}).Differentiate(x_);
  EXPECT_EQ(result3.size(), 1);
  EXPECT_EQ(result3.at(ChebyshevBasisElement({{x_, 1}})), 4);

  // d(T2(x) * T3(y)) / dx = 4 * T1(x)*T3(y)
  const auto result4 =
      ChebyshevBasisElement({{x_, 2}, {y_, 3}}).Differentiate(x_);
  EXPECT_EQ(result4.size(), 1);
  EXPECT_EQ(result4.at(ChebyshevBasisElement({{x_, 1}, {y_, 3}})), 4);

  // dT3(x) / dx = 6*T2(x) + 3
  const auto result5 = ChebyshevBasisElement({{x_, 3}}).Differentiate(x_);
  EXPECT_EQ(result5.size(), 2);
  EXPECT_EQ(result5.at(ChebyshevBasisElement({{x_, 2}})), 6);
  EXPECT_EQ(result5.at(ChebyshevBasisElement()), 3);

  // d(T3(x) * T4(y)) / dx = 6*T2(x)*T4(y) + 3*T4(y)
  const auto result6 =
      ChebyshevBasisElement({{x_, 3}, {y_, 4}}).Differentiate(x_);
  EXPECT_EQ(result6.size(), 2);
  EXPECT_EQ(result6.at(ChebyshevBasisElement({{x_, 2}, {y_, 4}})), 6);
  EXPECT_EQ(result6.at(ChebyshevBasisElement({{y_, 4}})), 3);

  // dT4(x) / dx = 8*T3(x) + 8*T1(x)
  const auto result7 = ChebyshevBasisElement({{x_, 4}}).Differentiate(x_);
  EXPECT_EQ(result7.size(), 2);
  EXPECT_EQ(result7.at(ChebyshevBasisElement({{x_, 3}})), 8);
  EXPECT_EQ(result7.at(ChebyshevBasisElement({{x_, 1}})), 8);

  // d(T4(x)*T3(y)) / dx = 8*T3(x)T3(y) + 8*T1(x)T3(y)
  const auto result8 =
      ChebyshevBasisElement({{x_, 4}, {y_, 3}}).Differentiate(x_);
  EXPECT_EQ(result8.size(), 2);
  EXPECT_EQ(result8.at(ChebyshevBasisElement({{x_, 3}, {y_, 3}})), 8);
  EXPECT_EQ(result8.at(ChebyshevBasisElement({{x_, 1}, {y_, 3}})), 8);
}

TEST_F(SymbolicChebyshevBasisElementTest, Integrate) {
  // ∫1dx = T1(x)
  const auto result1 = ChebyshevBasisElement().Integrate(x_);
  EXPECT_EQ(result1.size(), 1);
  EXPECT_EQ(result1.at(ChebyshevBasisElement({{x_, 1}})), 1);

  // ∫T2(x)dy = T2(x)T1(y)
  const auto result2 = ChebyshevBasisElement({{x_, 2}}).Integrate(y_);
  EXPECT_EQ(result2.size(), 1);
  EXPECT_EQ(result2.at(ChebyshevBasisElement({{x_, 2}, {y_, 1}})), 1);

  // ∫T2(x)dx = 1/6*T3(x) - 1/2*T1(x)
  const auto result3 = ChebyshevBasisElement({{x_, 2}}).Integrate(x_);
  EXPECT_EQ(result3.size(), 2);
  EXPECT_EQ(result3.at(ChebyshevBasisElement({{x_, 3}})), 1. / 6);
  EXPECT_EQ(result3.at(ChebyshevBasisElement({{x_, 1}})), -1. / 2);

  // ∫T3(x)T2(y)dx = 1/8*T4(x)T2(y) - 1/4*T2(x)T2(y)
  const auto result4 = ChebyshevBasisElement({{x_, 3}, {y_, 2}}).Integrate(x_);
  EXPECT_EQ(result4.size(), 2);
  EXPECT_EQ(result4.at(ChebyshevBasisElement({{x_, 4}, {y_, 2}})), 1. / 8);
  EXPECT_EQ(result4.at(ChebyshevBasisElement({{x_, 2}, {y_, 2}})), -1. / 4);
}

TEST_F(SymbolicChebyshevBasisElementTest, StringOutput) {
  std::ostringstream os1;
  os1 << ChebyshevBasisElement();
  EXPECT_EQ(fmt::format("{}", os1.str()), "T0()");

  std::ostringstream os2;
  os2 << ChebyshevBasisElement({{x_, 1}});
  EXPECT_EQ(fmt::format("{}", os2.str()), "T1(x)");

  std::ostringstream os3;
  os3 << ChebyshevBasisElement({{x_, 0}});
  EXPECT_EQ(fmt::format("{}", os3.str()), "T0()");

  std::ostringstream os4;
  os4 << ChebyshevBasisElement({{x_, 1}, {y_, 2}});
  EXPECT_EQ(fmt::format("{}", os4.str()), "T1(x)T2(y)");
}

TEST_F(SymbolicChebyshevBasisElementTest, ToExpression) {
  EXPECT_PRED2(test::ExprEqual, ChebyshevBasisElement().ToExpression(),
               Expression(1.));

  EXPECT_PRED2(test::ExprEqual, ChebyshevBasisElement({{x_, 0}}).ToExpression(),
               Expression(1.));

  EXPECT_PRED2(test::ExprEqual, ChebyshevBasisElement({{x_, 2}}).ToExpression(),
               ChebyshevPolynomial(x_, 2).ToPolynomial().ToExpression());

  EXPECT_PRED2(test::ExprEqual,
               ChebyshevBasisElement({{x_, 2}, {y_, 3}}).ToExpression(),
               ChebyshevPolynomial(x_, 2).ToPolynomial().ToExpression() *
                   ChebyshevPolynomial(y_, 3).ToPolynomial().ToExpression());
}

TEST_F(SymbolicChebyshevBasisElementTest, EigenMatrix) {
  // Checks we can have an Eigen matrix of ChebyshevBasisElements without
  // compilation errors. No assertions in the test.
  Eigen::Matrix<ChebyshevBasisElement, 2, 2> M;
  M << ChebyshevBasisElement(), ChebyshevBasisElement({{x_, 1}}),
      ChebyshevBasisElement({{x_, 1}, {y_, 2}}),
      ChebyshevBasisElement({{y_, 2}});

  // The following fails if we do not provide
  // `Eigen::NumTraits<drake::symbolic::DerivedA>`
  std::ostringstream oss;
  oss << M;
}

TEST_F(SymbolicChebyshevBasisElementTest, Hash) {
  // Check that ChebyshevBasisElement can be used as a key in
  // std::unordered_map.
  std::unordered_map<ChebyshevBasisElement, int> map;
  map.emplace(ChebyshevBasisElement({{x_, 1}}), 1);
  EXPECT_EQ(map.size(), 1);
  EXPECT_EQ(map.at(ChebyshevBasisElement({{x_, 1}})), 1);

  map.emplace(ChebyshevBasisElement({{x_, 1}, {y_, 2}}), 2);
  EXPECT_EQ(map.size(), 2);
  EXPECT_EQ(map.at(ChebyshevBasisElement({{x_, 1}, {y_, 2}})), 2);

  // Test the special case to map T0() to 3.
  map.emplace(ChebyshevBasisElement({{x_, 0}}), 3);
  EXPECT_EQ(map.size(), 3);
  // T0(y) = T0(x) = 1.
  EXPECT_EQ(map.at(ChebyshevBasisElement({{y_, 0}})), 3);
}

TEST_F(SymbolicChebyshevBasisElementTest, EvaluatePartial) {
  Environment env;
  env.insert(x_, 3);

  const ChebyshevBasisElement m1{{{x_, 2}, {y_, 4}, {z_, 3}}};
  double coeff;
  ChebyshevBasisElement new_basis_element;
  std::tie(coeff, new_basis_element) = m1.EvaluatePartial(env);
  EXPECT_EQ(coeff, 17);
  EXPECT_EQ(new_basis_element, ChebyshevBasisElement({{y_, 4}, {z_, 3}}));

  env.insert(z_, 2);
  std::tie(coeff, new_basis_element) = m1.EvaluatePartial(env);
  EXPECT_EQ(coeff, 17 * 26);
  EXPECT_EQ(new_basis_element, ChebyshevBasisElement(y_, 4));
}

TEST_F(SymbolicChebyshevBasisElementTest, MergeBasisElementInPlace) {
  // Merge T₁(x)T₃(y) and T₁(x)T₂(z) gets T₂(x)T₃(y)T₂(z)
  ChebyshevBasisElement basis_element1({{x_, 1}, {y_, 3}});
  basis_element1.MergeBasisElementInPlace(
      ChebyshevBasisElement({{x_, 1}, {z_, 2}}));
  EXPECT_EQ(basis_element1.var_to_degree_map().size(), 3);
  EXPECT_EQ(basis_element1.var_to_degree_map().at(x_), 2);
  EXPECT_EQ(basis_element1.var_to_degree_map().at(y_), 3);
  EXPECT_EQ(basis_element1.var_to_degree_map().at(z_), 2);
  EXPECT_EQ(basis_element1.total_degree(), 7);
}
}  // namespace symbolic
}  // namespace drake
