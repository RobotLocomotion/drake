#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/common/unused.h"
#define DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER
#include "drake/common/symbolic_expression_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER

namespace drake {
namespace symbolic {

// Create a concrete derived polynomial basis class.
class DerivedBasisA : public PolynomialBasisElement {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DerivedBasisA);

  DerivedBasisA() : PolynomialBasisElement() {}

  explicit DerivedBasisA(const std::map<Variable, int>& var_to_degree_map)
      : PolynomialBasisElement(var_to_degree_map) {}

  bool operator<(const DerivedBasisA& other) const {
    return this->lexicographical_compare(other);
  }

  std::pair<double, DerivedBasisA> EvaluatePartial(
      const Environment& env) const {
    double coeff;
    std::map<Variable, int> new_var_to_degree_map;
    this->DoEvaluatePartial(env, &coeff, &new_var_to_degree_map);
    return std::make_pair(coeff, DerivedBasisA(new_var_to_degree_map));
  }

  void MergeBasisElementInPlace(const DerivedBasisA& other) {
    this->DoMergeBasisElementInPlace(other);
  }

 private:
  double DoEvaluate(double variable_val, int degree) const override {
    return std::pow(variable_val, degree);
  }

  Expression DoToExpression() const override {
    std::map<Expression, Expression> base_to_exponent_map;
    for (const auto& [var, degree] : var_to_degree_map()) {
      base_to_exponent_map.emplace(Expression{var}, degree);
    }
    return ExpressionMulFactory{1.0, base_to_exponent_map}.GetExpression();
  }
};

class DerivedBasisB : public PolynomialBasisElement {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DerivedBasisB);

  DerivedBasisB() : PolynomialBasisElement() {}

  explicit DerivedBasisB(const std::map<Variable, int>& var_to_degree_map)
      : PolynomialBasisElement(var_to_degree_map) {}

  bool operator<(const DerivedBasisB& other) const {
    return this->lexicographical_compare(other);
  }

  void MergeBasisElementInPlace(const DerivedBasisB& other) {
    this->DoMergeBasisElementInPlace(other);
  }

 private:
  double DoEvaluate(double variable_val, int degree) const override {
    return 1.;
  }

  Expression DoToExpression() const override { return Expression(1.); }
};

class SymbolicPolynomialBasisElementTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
};

TEST_F(SymbolicPolynomialBasisElementTest, Constructor) {
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

  const DerivedBasisA p4(std::map<Variable, int>({}));
  EXPECT_EQ(p4.total_degree(), 0);
  EXPECT_EQ(p4.var_to_degree_map().size(), 0);

  DRAKE_EXPECT_THROWS_MESSAGE(DerivedBasisA({{x_, -1}}), std::logic_error,
                              "The degree for x is negative.");
}

TEST_F(SymbolicPolynomialBasisElementTest, degree) {
  EXPECT_EQ(DerivedBasisA({{x_, 1}, {y_, 2}}).degree(x_), 1);
  EXPECT_EQ(DerivedBasisA({{x_, 1}, {y_, 2}}).degree(y_), 2);
  EXPECT_EQ(DerivedBasisA({{x_, 1}, {y_, 2}}).degree(z_), 0);
  EXPECT_EQ(DerivedBasisA(std::map<Variable, int>()).degree(x_), 0);
}

TEST_F(SymbolicPolynomialBasisElementTest, GetVariables) {
  const symbolic::DerivedBasisA p1({{x_, 1}, {y_, 2}});
  EXPECT_EQ(p1.GetVariables(), Variables({x_, y_}));

  const symbolic::DerivedBasisA p2({{x_, 0}, {y_, 2}});
  EXPECT_EQ(p2.GetVariables(), Variables({y_}));

  const symbolic::DerivedBasisA p3({{x_, 0}, {y_, 0}});
  EXPECT_EQ(p3.GetVariables(), Variables({}));

  const symbolic::DerivedBasisA p4(std::map<Variable, int>({}));
  EXPECT_EQ(p4.GetVariables(), Variables({}));
}

TEST_F(SymbolicPolynomialBasisElementTest, OperatorEqualNotEqual) {
  const DerivedBasisA p1({{x_, 1}, {y_, 2}});
  const DerivedBasisA p2({{x_, 1}, {y_, 2}});
  const DerivedBasisA p3({{x_, 0}, {y_, 2}});
  const DerivedBasisA p4({{y_, 2}});
  const DerivedBasisB p5({{y_, 2}});
  const DerivedBasisB p6({{x_, 2}});

  EXPECT_EQ(p1, p2);
  // x⁰y² == y²
  EXPECT_EQ(p3, p4);
  // The degree for x is different
  EXPECT_NE(p1, p3);
  // The derived basis types are different.
  EXPECT_NE(p4, p5);
  // The variable is different.
  EXPECT_NE(p5, p6);
}

TEST_F(SymbolicPolynomialBasisElementTest, Evaluate) {
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
  double dummy{};
  DRAKE_EXPECT_THROWS_MESSAGE(dummy = p2.Evaluate(env2), std::invalid_argument,
                              ".* x is not in env");
  unused(dummy);
}

TEST_F(SymbolicPolynomialBasisElementTest, LessThan) {
  const DerivedBasisA p1({{x_, 1}, {y_, 2}});
  const DerivedBasisA p2({{y_, 3}});
  const DerivedBasisA p3({{x_, 2}});

  EXPECT_LT(p2, p1);
  EXPECT_LT(p1, p3);
  EXPECT_LT(p2, p3);
}

TEST_F(SymbolicPolynomialBasisElementTest, EigenMatrix) {
  // Checks we can have an Eigen matrix of PolynomialBasisElements without
  // compilation errors. No assertions in the test.
  Eigen::Matrix<DerivedBasisA, 2, 2> M;
  M << DerivedBasisA(std::map<Variable, int>({})), DerivedBasisA({{x_, 1}}),
      DerivedBasisA({{x_, 1}, {y_, 2}}), DerivedBasisA({{y_, 2}});
}

TEST_F(SymbolicPolynomialBasisElementTest, EvaluatePartial) {
  Environment env;
  env.insert(x_, 2);
  const DerivedBasisA m1{{{x_, 3}, {y_, 4}}};
  double coeff;
  DerivedBasisA new_basis;
  std::tie(coeff, new_basis) = m1.EvaluatePartial(env);
  EXPECT_EQ(coeff, 8);
  EXPECT_EQ(new_basis, DerivedBasisA({{y_, 4}}));

  const DerivedBasisA m2{{{y_, 3}, {z_, 2}}};
  std::tie(coeff, new_basis) = m2.EvaluatePartial(env);
  EXPECT_EQ(coeff, 1);
  EXPECT_EQ(new_basis, m2);
}

TEST_F(SymbolicPolynomialBasisElementTest, BasisElementGradedReverseLexOrder) {
  EXPECT_PRED2(test::VarLess, x_, y_);
  EXPECT_PRED2(test::VarLess, y_, z_);
  BasisElementGradedReverseLexOrder<std::less<Variable>, DerivedBasisA> compare;
  // y^0 = x^0 = 1.
  EXPECT_FALSE(compare(DerivedBasisA({{y_, 0}}), DerivedBasisA({{x_, 0}})));
  EXPECT_FALSE(compare(DerivedBasisA({{x_, 0}}), DerivedBasisA({{y_, 0}})));

  // x < y < z
  EXPECT_TRUE(compare(DerivedBasisA({{x_, 1}}), DerivedBasisA({{y_, 1}})));
  EXPECT_TRUE(compare(DerivedBasisA({{x_, 1}}), DerivedBasisA({{z_, 1}})));
  EXPECT_TRUE(compare(DerivedBasisA({{y_, 1}}), DerivedBasisA({{z_, 1}})));

  // x < y < z < x² < xy < xz < y² < yz < z²
  std::vector<DerivedBasisA> derived_basis_all;
  derived_basis_all.emplace_back(std::map<Variable, int>{{x_, 0}});
  derived_basis_all.emplace_back(std::map<Variable, int>{{x_, 1}});
  derived_basis_all.emplace_back(std::map<Variable, int>{{y_, 1}});
  derived_basis_all.emplace_back(std::map<Variable, int>{{z_, 1}});
  derived_basis_all.emplace_back(std::map<Variable, int>{{x_, 2}});
  derived_basis_all.emplace_back(std::map<Variable, int>{{x_, 1}, {y_, 1}});
  derived_basis_all.emplace_back(std::map<Variable, int>{{x_, 1}, {z_, 1}});
  derived_basis_all.emplace_back(std::map<Variable, int>{{y_, 2}});
  derived_basis_all.emplace_back(std::map<Variable, int>{{y_, 1}, {z_, 1}});
  derived_basis_all.emplace_back(std::map<Variable, int>{{z_, 2}});
  for (int i = 0; i < static_cast<int>(derived_basis_all.size()); ++i) {
    for (int j = 0; j < static_cast<int>(derived_basis_all.size()); ++j) {
      if (i < j) {
        EXPECT_TRUE(compare(derived_basis_all[i], derived_basis_all[j]));
      } else {
        EXPECT_FALSE(compare(derived_basis_all[i], derived_basis_all[j]));
      }
    }
  }
}

TEST_F(SymbolicPolynomialBasisElementTest, MergeBasisElementInPlace) {
  // Merge [(x->1), (y->2)] and [(x->2), (y->1)] gets [(x->3), (y->3)]
  DerivedBasisA basis_element1({{x_, 1}, {y_, 2}});
  basis_element1.MergeBasisElementInPlace(DerivedBasisA({{x_, 2}, {y_, 1}}));
  EXPECT_EQ(basis_element1.var_to_degree_map().size(), 2);
  EXPECT_EQ(basis_element1.var_to_degree_map().at(x_), 3);
  EXPECT_EQ(basis_element1.var_to_degree_map().at(y_), 3);
  EXPECT_EQ(basis_element1.total_degree(), 6);

  // Merge [(x->2), (z->1)] and [(x->3), (y->4)] gets [(x->5), (y->4), (z->1)]
  DerivedBasisA basis_element2({{x_, 2}, {z_, 1}});
  basis_element2.MergeBasisElementInPlace(DerivedBasisA({{x_, 3}, {y_, 4}}));
  EXPECT_EQ(basis_element2.var_to_degree_map().size(), 3);
  EXPECT_EQ(basis_element2.var_to_degree_map().at(x_), 5);
  EXPECT_EQ(basis_element2.var_to_degree_map().at(y_), 4);
  EXPECT_EQ(basis_element2.var_to_degree_map().at(z_), 1);
  EXPECT_EQ(basis_element2.total_degree(), 10);

  // Merge [(z->3)] and [(x->1), (y->2)] gets [(x->1), (y->2), (z->3)]
  DerivedBasisA basis_element3({{z_, 3}});
  basis_element3.MergeBasisElementInPlace(DerivedBasisA({{x_, 1}, {y_, 2}}));
  EXPECT_EQ(basis_element3.var_to_degree_map().size(), 3);
  EXPECT_EQ(basis_element3.var_to_degree_map().at(x_), 1);
  EXPECT_EQ(basis_element3.var_to_degree_map().at(y_), 2);
  EXPECT_EQ(basis_element3.var_to_degree_map().at(z_), 3);
  EXPECT_EQ(basis_element3.total_degree(), 6);

  // Merge [(x->2)] and [(x->1), (y->3), (z->2)] gets [(x->3), (y->3), (z->2)]
  DerivedBasisA basis_element4({{x_, 2}});
  basis_element4.MergeBasisElementInPlace(
      DerivedBasisA({{x_, 1}, {y_, 3}, {z_, 2}}));
  EXPECT_EQ(basis_element4.var_to_degree_map().size(), 3);
  EXPECT_EQ(basis_element4.var_to_degree_map().at(x_), 3);
  EXPECT_EQ(basis_element4.var_to_degree_map().at(y_), 3);
  EXPECT_EQ(basis_element4.var_to_degree_map().at(z_), 2);
  EXPECT_EQ(basis_element4.total_degree(), 8);

  // Merge [] with [(x->1)] gets [(x->1)]
  DerivedBasisA basis_element5{};
  basis_element5.MergeBasisElementInPlace(DerivedBasisA({{x_, 1}}));
  EXPECT_EQ(basis_element5.var_to_degree_map().size(), 1);
  EXPECT_EQ(basis_element5.var_to_degree_map().at(x_), 1);
  EXPECT_EQ(basis_element5.total_degree(), 1);

  // Merge [(x->1)] with [] gets [(x->1)]
  DerivedBasisA basis_element6{{{x_, 1}}};
  basis_element6.MergeBasisElementInPlace(DerivedBasisA());
  EXPECT_EQ(basis_element6.var_to_degree_map().size(), 1);
  EXPECT_EQ(basis_element6.var_to_degree_map().at(x_), 1);
  EXPECT_EQ(basis_element6.total_degree(), 1);
}
}  // namespace symbolic
}  // namespace drake
