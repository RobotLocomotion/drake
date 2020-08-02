#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace symbolic {

// Create a concrete derived polynomial basis class.
class DerivedBasisA : public PolynomialBasis {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DerivedBasisA);

  explicit DerivedBasisA(const std::map<Variable, int>& var_to_degree_map)
      : PolynomialBasis(var_to_degree_map) {}

 private:
  double DoEvaluate(double variable_val, int degree) const override {
    return std::pow(variable_val, degree);
  }
};

class DerivedBasisB : public PolynomialBasis {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DerivedBasisB);

  explicit DerivedBasisB(const std::map<Variable, int>& var_to_degree_map)
      : PolynomialBasis(var_to_degree_map) {}

 private:
  double DoEvaluate(double variable_val, int degree) const override {
    return 1.;
  }
};

class SymbolicPolynomialBasisTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
};

TEST_F(SymbolicPolynomialBasisTest, Constructor) {
  const DerivedBasisA p1({{x_, 1}, {y_, 2}});
  EXPECT_EQ(p1.total_degree(), 3);
  EXPECT_EQ(p1.var_to_degree_map().size(), 2);
  EXPECT_EQ(p1.var_to_degree_map().at(x_), 1);
  EXPECT_EQ(p1.var_to_degree_map().at(y_), 2);

  const DerivedBasisA p2({{x_, 0}, {y_, 2}});
  EXPECT_EQ(p2.total_degree(), 2);
  EXPECT_EQ(p2.var_to_degree_map().size(), 1);
  EXPECT_EQ(p2.var_to_degree_map().at(y_), 2);

  const DerivedBasisA p3({{x_, 0}, {y_, 0}});
  EXPECT_EQ(p3.total_degree(), 0);
  EXPECT_EQ(p3.var_to_degree_map().size(), 0);

  const DerivedBasisA p4({});
  EXPECT_EQ(p4.total_degree(), 0);
  EXPECT_EQ(p4.var_to_degree_map().size(), 0);

  DRAKE_EXPECT_THROWS_MESSAGE(DerivedBasisA({{x_, -1}}), std::logic_error,
                              "The degree for x is negative.");
}

TEST_F(SymbolicPolynomialBasisTest, GetVariables) {
  const symbolic::DerivedBasisA p1({{x_, 1}, {y_, 2}});
  EXPECT_EQ(p1.GetVariables(), Variables({x_, y_}));

  const symbolic::DerivedBasisA p2({{x_, 0}, {y_, 2}});
  EXPECT_EQ(p2.GetVariables(), Variables({y_}));

  const symbolic::DerivedBasisA p3({{x_, 0}, {y_, 0}});
  EXPECT_EQ(p3.GetVariables(), Variables({}));

  const symbolic::DerivedBasisA p4({});
  EXPECT_EQ(p4.GetVariables(), Variables({}));
}

TEST_F(SymbolicPolynomialBasisTest, operator_equal_not_equal) {
  const DerivedBasisA p1({{x_, 1}, {y_, 2}});
  const DerivedBasisA p2({{x_, 1}, {y_, 2}});
  const DerivedBasisA p3({{x_, 0}, {y_, 2}});
  const DerivedBasisA p4({{y_, 2}});
  const DerivedBasisB p5({{y_, 2}});

  EXPECT_EQ(p1, p2);
  // x⁰y² == y²
  EXPECT_EQ(p3, p4);
  // The degree for x is different
  EXPECT_NE(p1, p3);
  // The derived basis types are different.
  EXPECT_NE(p4, p5);
}

TEST_F(SymbolicPolynomialBasisTest, Evaluate) {
  Environment env1;
  env1.insert(x_, 2);
  env1.insert(y_, 3);

  const DerivedBasisA p1({{x_, 0}, {y_, 2}});
  EXPECT_EQ(p1.Evaluate(env1), 9);

  const DerivedBasisA p2({{x_, 1}, {y_, 2}});
  EXPECT_EQ(p2.Evaluate(env1), 18);

  Environment env2;
  env2.insert(y_, 3);
  // p1=y²
  EXPECT_EQ(p1.Evaluate(env2), 9);
  // p2=xy², but env2 does not contain value for x.
  DRAKE_EXPECT_THROWS_MESSAGE(p2.Evaluate(env2), std::invalid_argument,
                              ".* x is not in env");
}
}  // namespace symbolic
}  // namespace drake
