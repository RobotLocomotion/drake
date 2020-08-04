#include <gtest/gtest.h>

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
class SymbolicChebyshevBasisElementTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
};

TEST_F(SymbolicChebyshevBasisElementTest, less_than) {
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
}  // namespace symbolic
}  // namespace drake
