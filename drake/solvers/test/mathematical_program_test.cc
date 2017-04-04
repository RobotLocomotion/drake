#include "drake/solvers/mathematical_program.h"

#include <algorithm>
#include <typeinfo>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/monomial.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/test/symbolic_test_util.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

using Eigen::Dynamic;
using Eigen::Ref;
using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::solvers::detail::VecIn;
using drake::solvers::detail::VecOut;
using drake::symbolic::Expression;
using drake::symbolic::Formula;
using drake::symbolic::Variable;
using drake::symbolic::test::ExprEqual;

using std::numeric_limits;

namespace drake {
namespace solvers {
namespace test {

struct Movable {
  Movable() = default;
  Movable(Movable&&) = default;
  Movable(Movable const&) = delete;
  static size_t numInputs() { return 1; }
  static size_t numOutputs() { return 1; }
  template <typename ScalarType>
  void eval(VecIn<ScalarType> const&, VecOut<ScalarType>&) const {}
};

struct Copyable {
  Copyable() = default;
  Copyable(Copyable&&) = delete;
  Copyable(Copyable const&) = default;
  static size_t numInputs() { return 1; }
  static size_t numOutputs() { return 1; }
  template <typename ScalarType>
  void eval(VecIn<ScalarType> const&, VecOut<ScalarType>&) const {}
};

struct Unique {
  Unique() = default;
  Unique(Unique&&) = delete;
  Unique(Unique const&) = delete;
  static size_t numInputs() { return 1; }
  static size_t numOutputs() { return 1; }
  template <typename ScalarType>
  void eval(VecIn<ScalarType> const&, VecOut<ScalarType>&) const {}
};
// TODO(naveenoid) : tests need to be purged of Random initializations.

// Check the index, type and name etc of the newly added variables.
// This function only works if the only variables contained in @p prog are @p
// var.
template <typename Derived>
void CheckAddedVariable(const MathematicalProgram& prog,
                        const Eigen::MatrixBase<Derived>& var,
                        const std::string& var_name, bool is_symmetric,
                        MathematicalProgram::VarType type_expected) {
  // Checks the name of the newly added variables.
  std::stringstream msg_buff;
  msg_buff << var << std::endl;
  EXPECT_EQ(msg_buff.str(), var_name);
  // Checks num_vars() function.
  const int num_new_vars =
      is_symmetric ? var.rows() * (var.rows() + 1) / 2 : var.size();
  EXPECT_EQ(prog.num_vars(), num_new_vars);
  // Checks if the newly added variable is symmetric.
  EXPECT_EQ(math::IsSymmetric(var), is_symmetric);
  // Checks the indices of the newly added variables.
  if (is_symmetric) {
    int var_count = 0;
    for (int j = 0; j < var.cols(); ++j) {
      for (int i = j; i < var.rows(); ++i) {
        EXPECT_EQ(prog.FindDecisionVariableIndex(var(i, j)), var_count);
        ++var_count;
      }
    }
  } else {
    for (int i = 0; i < var.rows(); ++i) {
      for (int j = 0; j < var.cols(); ++j) {
        EXPECT_EQ(prog.FindDecisionVariableIndex(var(i, j)),
                  j * var.rows() + i);
      }
    }
  }

  // Checks the type of the newly added variables.
  const auto& variable_types = prog.DecisionVariableTypes();
  for (int i = 0; i < var.rows(); ++i) {
    for (int j = 0; j < var.cols(); ++j) {
      EXPECT_EQ(variable_types[prog.FindDecisionVariableIndex(var(i, j))],
                type_expected);
      EXPECT_EQ(prog.DecisionVariableType(var(i, j)), type_expected);
    }
  }
}

GTEST_TEST(testAddVariable, testAddContinuousVariables1) {
  // Adds a dynamic-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables(2, 3, "X");
  static_assert(std::is_same<decltype(X), MatrixXDecisionVariable>::value,
                "should be a dynamic sized matrix");
  EXPECT_EQ(X.rows(), 2);
  EXPECT_EQ(X.cols(), 3);
  CheckAddedVariable(prog, X, "X(0,0) X(0,1) X(0,2)\nX(1,0) X(1,1) X(1,2)\n",
                     false, MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddContinuousVariable2) {
  // Adds a static-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables<2, 3>("X");
  static_assert(std::is_same<decltype(X), MatrixDecisionVariable<2, 3>>::value,
                "should be a static sized matrix");
  CheckAddedVariable(prog, X, "X(0,0) X(0,1) X(0,2)\nX(1,0) X(1,1) X(1,2)\n",
                     false, MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddContinuousVariable3) {
  // Adds a dynamic-sized vector of continuous variables.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(4, "x");
  static_assert(std::is_same<decltype(x), VectorXDecisionVariable>::value,
                "Should be a VectorXDecisionVariable object.");
  EXPECT_EQ(x.rows(), 4);
  CheckAddedVariable(prog, x, "x(0)\nx(1)\nx(2)\nx(3)\n", false,
                     MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddContinuousVariable4) {
  // Adds a static-sized vector of continuous variables.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>("x");
  static_assert(std::is_same<decltype(x), VectorDecisionVariable<4>>::value,
                "Should be a VectorXDecisionVariable object.");
  CheckAddedVariable(prog, x, "x(0)\nx(1)\nx(2)\nx(3)\n", false,
                     MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddSymmetricVariable1) {
  // Adds a static-sized symmetric matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>("X");
  static_assert(std::is_same<decltype(X), MatrixDecisionVariable<3, 3>>::value,
                "should be a MatrixDecisionVariable<3> object");
  CheckAddedVariable(
      prog, X,
      "X(0,0) X(1,0) X(2,0)\nX(1,0) X(1,1) X(2,1)\nX(2,0) X(2,1) X(2,2)\n",
      true, MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddSymmetricVariable2) {
  // Adds a dynamic-sized symmetric matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables(3, "X");
  static_assert(std::is_same<decltype(X), MatrixXDecisionVariable>::value,
                "should be a MatrixXDecisionVariable object");
  EXPECT_EQ(X.rows(), 3);
  EXPECT_EQ(X.cols(), 3);
  CheckAddedVariable(
      prog, X,
      "X(0,0) X(1,0) X(2,0)\nX(1,0) X(1,1) X(2,1)\nX(2,0) X(2,1) X(2,2)\n",
      true, MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddBinaryVariable1) {
  // Adds a dynamic-sized matrix of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables(2, 3, "B");
  static_assert(std::is_same<decltype(X), MatrixXDecisionVariable>::value,
                "wrong type");
  EXPECT_EQ(X.rows(), 2);
  EXPECT_EQ(X.cols(), 3);
  CheckAddedVariable(prog, X, "B(0,0) B(0,1) B(0,2)\nB(1,0) B(1,1) B(1,2)\n",
                     false, MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(testAddVariable, testAddBinaryVariable3) {
  // Adds dynamic-sized vector of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables(4, "B");
  static_assert(std::is_same<decltype(X), VectorXDecisionVariable>::value,
                "wrong type");
  EXPECT_EQ(X.rows(), 4);
  CheckAddedVariable(prog, X, "B(0)\nB(1)\nB(2)\nB(3)\n", false,
                     MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(testAddVariable, testAddBinaryVariable4) {
  // Adds static-sized vector of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables<4>("B");
  static_assert(std::is_same<decltype(X), VectorDecisionVariable<4>>::value,
                "wrong type");
  CheckAddedVariable(prog, X, "B(0)\nB(1)\nB(2)\nB(3)\n", false,
                     MathematicalProgram::VarType::BINARY);
}

template <typename Derived1, typename Derived2>
typename std::enable_if<
    std::is_same<typename Derived1::Scalar, Variable>::value &&
    std::is_same<typename Derived2::Scalar, double>::value>::type
CheckGetSolution(const MathematicalProgram& prog,
                 const Eigen::MatrixBase<Derived1>& vars,
                 const Eigen::MatrixBase<Derived2>& val_expected) {
  auto val = prog.GetSolution(vars);
  static_assert(std::is_same<decltype(val),
                             Eigen::Matrix<double, Derived1::RowsAtCompileTime,
                                           Derived1::ColsAtCompileTime>>::value,
                "GetSolution does not return the right type of matrix");
  EXPECT_TRUE(CompareMatrices(val, val_expected));

  // Checks getting solution for a single variable.
  for (int i = 0; i < vars.rows(); ++i) {
    for (int j = 0; j < vars.cols(); ++j) {
      EXPECT_NEAR(prog.GetSolution(vars(i, j)), val(i, j), 1E-14);
    }
  }
}

GTEST_TEST(testGetSolution, testSetSolution1) {
  // Tests setting and getting solution for
  // 1. A static-sized  matrix of decision variables.
  // 2. A dynamic-sized matrix of decision variables.
  // 3. A static-sized  vector of decision variables.
  // 4. A dynamic-sized vector of decision variables.
  MathematicalProgram prog;
  auto X1 = prog.NewContinuousVariables<2, 3>("X");
  auto X2 = prog.NewContinuousVariables(2, 3, "X");
  auto x3 = prog.NewContinuousVariables<4>("x");
  auto x4 = prog.NewContinuousVariables(4, "x");

  Eigen::Matrix<double, 2, 3> X1_value{};
  X1_value << 0, 1, 2, 3, 4, 5;
  Eigen::Matrix<double, 2, 3> X2_value{};
  X2_value = -X1_value;
  Eigen::Vector4d x3_value(3, 4, 5, 6);
  Eigen::Vector4d x4_value = -x3_value;
  for (int i = 0; i < 3; ++i) {
    prog.SetDecisionVariableValues(X1.col(i), X1_value.col(i));
    prog.SetDecisionVariableValues(X2.col(i), X2_value.col(i));
  }
  prog.SetDecisionVariableValues(x3, x3_value);
  prog.SetDecisionVariableValues(x4, x4_value);

  CheckGetSolution(prog, X1, X1_value);
  CheckGetSolution(prog, X2, X2_value);
  CheckGetSolution(prog, x3, x3_value);
  CheckGetSolution(prog, x4, x4_value);

  // Check a variable that is not a decision variable of the mathematical
  // program.
  symbolic::Variable z1("z1");
  symbolic::Variable z2("z2");
  EXPECT_THROW(prog.GetSolution(z1), std::runtime_error);
  EXPECT_THROW(prog.GetSolution(VectorDecisionVariable<2>(z1, z2)),
               std::runtime_error);
  EXPECT_THROW(prog.GetSolution(VectorDecisionVariable<2>(z1, X1(0, 0))),
               std::runtime_error);
}

GTEST_TEST(testMathematicalProgram, testAddFunction) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>();

  Movable movable;
  prog.AddCost(std::move(movable), x);
  prog.AddCost(Movable(), x);

  Copyable copyable;
  prog.AddCost(copyable, x);

  Unique unique;
  prog.AddCost(std::cref(unique), x);
  prog.AddCost(std::make_shared<Unique>(), x);
  prog.AddCost(std::unique_ptr<Unique>(new Unique), x);
}

GTEST_TEST(testMathematicalProgram, BoundingBoxTest2) {
  // Test the scalar version of the bounding box constraint methods.

  MathematicalProgram prog;
  auto x1 = prog.NewContinuousVariables<2, 2>("x1");
  MatrixXDecisionVariable x2(2, 2);
  x2 = x1;
  VectorDecisionVariable<4> x3;
  x3 << x1.col(0), x1.col(1);
  VectorXDecisionVariable x4(4);
  x4 = x3;
  // Six different ways to construct an equivalent constraint.
  // 1. Imposes constraint on a static-sized matrix of decision variables.
  // 2. Imposes constraint on a list of vectors of decision variables.
  // 3. Imposes constraint on a dynamic-sized matrix of decision variables.
  // 4. Imposes constraint on a static-sized vector of decision variables.
  // 5. Imposes constraint on a dynamic-sized vector of decision variables.
  // 6. Imposes constraint using a vector of lower/upper bound, as compared
  //    to the previous three cases which use a scalar lower/upper bound.
  auto constraint1 = prog.AddBoundingBoxConstraint(0, 1, x1);
  auto constraint2 =
      prog.AddBoundingBoxConstraint(0, 1, {x1.col(0), x1.col(1)});
  auto constraint3 = prog.AddBoundingBoxConstraint(0, 1, x2);
  auto constraint4 = prog.AddBoundingBoxConstraint(0, 1, x3);
  auto constraint5 = prog.AddBoundingBoxConstraint(0, 1, x4);
  auto constraint6 = prog.AddBoundingBoxConstraint(Eigen::Vector4d::Zero(),
                                                   Eigen::Vector4d::Ones(), x3);

  // Checks that the bound variables are correct.
  for (const auto& binding : prog.bounding_box_constraints()) {
    EXPECT_EQ(binding.GetNumElements(), 4u);
    VectorDecisionVariable<4> x_expected;
    x_expected << x1(0, 0), x1(1, 0), x1(0, 1), x1(1, 1);
    for (int i = 0; i < 4; ++i) {
      EXPECT_EQ(binding.variables()(i), x_expected(i));
    }
  }
  EXPECT_TRUE(
      CompareMatrices(constraint1->lower_bound(), constraint2->lower_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint2->lower_bound(), constraint3->lower_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint3->lower_bound(), constraint4->lower_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint4->lower_bound(), constraint5->lower_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint5->lower_bound(), constraint6->lower_bound()));

  EXPECT_TRUE(
      CompareMatrices(constraint1->upper_bound(), constraint2->upper_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint2->upper_bound(), constraint3->upper_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint3->upper_bound(), constraint4->upper_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint4->upper_bound(), constraint5->upper_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint5->upper_bound(), constraint6->upper_bound()));
}

// A generic cost derived from Constraint class. This is meant for testing
// adding a cost to optimization program, and the cost is in the form of a
// derived class of Constraint.
class GenericTrivialCost1 : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GenericTrivialCost1)

  GenericTrivialCost1()
      : Constraint(1, 3, Vector1d(std::numeric_limits<double>::infinity()),
                   Vector1d(std::numeric_limits<double>::infinity())),
        private_val_(2) {}

 protected:
  void DoEval(const Ref<const Eigen::VectorXd>& x, VectorXd& y) const override {
    y.resize(1);
    y(0) = x(0) * x(1) + x(2) / x(0) * private_val_;
  }

  void DoEval(const Ref<const TaylorVecXd>& x, TaylorVecXd& y) const override {
    y.resize(1);
    y(0) = x(0) * x(1) + x(2) / x(0) * private_val_;
  }

 private:
  // Add a private data member to make sure no slicing on this class, derived
  // from Constraint.
  double private_val_{0};
};

// A generic cost. This class is meant for testing adding a cost to the
// optimization program, by calling `MathematicalProgram::MakeCost` to
// convert this class to a ConstraintImpl object.
class GenericTrivialCost2 {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GenericTrivialCost2)

  GenericTrivialCost2() = default;

  static size_t numInputs() { return 2; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
    y(0) = x(0) * x(0) - x(1) * x(1) + 2;
  }
};

// Verifies if the added cost evaluates the same as the original cost.
// This function is supposed to test these costs added as a derived class
// from Constraint.
void VerifyAddedCost1(const MathematicalProgram& prog,
                      const std::shared_ptr<Constraint>& cost,
                      const Eigen::Ref<const Eigen::VectorXd>& x_value,
                      int num_generic_costs_expected) {
  EXPECT_EQ(static_cast<int>(prog.generic_costs().size()),
            num_generic_costs_expected);
  Eigen::VectorXd y, y_expected;
  prog.generic_costs().back().constraint()->Eval(x_value, y);
  cost->Eval(x_value, y_expected);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
}

// Verifies if the added cost evaluates the same as the original cost.
// This function is supposed to test these costs added by converting
// a class to ConstraintImpl through MakeCost.
void VerifyAddedCost2(const MathematicalProgram& prog,
                      const GenericTrivialCost2& cost,
                      const std::shared_ptr<Constraint>& returned_cost,
                      const Eigen::Ref<const Eigen::Vector2d>& x_value,
                      int num_generic_costs_expected) {
  EXPECT_EQ(static_cast<int>(prog.generic_costs().size()),
            num_generic_costs_expected);
  Eigen::VectorXd y(1), y_expected(1), y_returned;
  prog.generic_costs().back().constraint()->Eval(x_value, y);
  cost.eval<double>(x_value, y_expected);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
  returned_cost->Eval(x_value, y_returned);
  EXPECT_TRUE(CompareMatrices(y, y_returned));
}

GTEST_TEST(testMathematicalProgram, AddCostTest) {
  // Test if the costs are added correctly.

  // There are ways to add a generic cost
  // 1. Add Binding<Constraint>
  // 2. Add shared_ptr<Constraint> on a VectorDecisionVariable object.
  // 3. Add shared_ptr<Constraint> on a VariableRefList.
  // 4. Add a ConstraintImpl object on a VectorDecisionVariable object.
  // 5. Add a ConstraintImpl object on a VariableRefList object.
  // 6. Add a unique_ptr of object that can be converted to a ConstraintImpl
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  auto y = prog.NewContinuousVariables<2>("y");
  // No cost yet.
  int num_generic_costs = 0;
  EXPECT_EQ(static_cast<int>(prog.generic_costs().size()), num_generic_costs);
  EXPECT_EQ(prog.linear_costs().size(), 0u);

  std::shared_ptr<Constraint> generic_trivial_cost1 =
      std::make_shared<GenericTrivialCost1>();

  // Adds Binding<Constraint>
  prog.AddCost(Binding<Constraint>(
      generic_trivial_cost1, VectorDecisionVariable<3>(x(0), x(1), y(1))));
  ++num_generic_costs;
  VerifyAddedCost1(prog, generic_trivial_cost1, Eigen::Vector3d(1, 3, 5),
                   num_generic_costs);

  // Adds a std::shared_ptr<Constraint> on a VectorDecisionVariable object.
  prog.AddCost(generic_trivial_cost1,
               VectorDecisionVariable<3>(x(0), x(1), y(1)));
  ++num_generic_costs;
  VerifyAddedCost1(prog, generic_trivial_cost1, Eigen::Vector3d(1, 2, 3),
                   num_generic_costs);

  // Adds a std::shared_ptr<Constraint> on a VariableRefList object.
  prog.AddCost(generic_trivial_cost1, {x, y.tail<1>()});
  ++num_generic_costs;
  VerifyAddedCost1(prog, generic_trivial_cost1, Eigen::Vector3d(2, 3, 4),
                   num_generic_costs);

  GenericTrivialCost2 generic_trivial_cost2;

  // Add an object that can be converted to a ConstraintImpl object on a
  // VectorDecisionVariable object.
  auto returned_cost3 = prog.AddCost(generic_trivial_cost2,
                                     VectorDecisionVariable<2>(x(0), y(1)));
  ++num_generic_costs;
  VerifyAddedCost2(prog, generic_trivial_cost2, returned_cost3,
                   Eigen::Vector2d(1, 2), num_generic_costs);

  // Add an object that can be converted to a ConstraintImpl object on a
  // VariableRefList object.
  auto returned_cost4 =
      prog.AddCost(generic_trivial_cost2, {x.head<1>(), y.tail<1>()});
  ++num_generic_costs;
  VerifyAddedCost2(prog, generic_trivial_cost2, returned_cost4,
                   Eigen::Vector2d(1, 2), num_generic_costs);
}

void CheckAddedSymbolicLinearCostUserFun(const MathematicalProgram& prog,
                                         const Expression& e,
                                         const Binding<Constraint>& binding,
                                         int num_linear_costs) {
  EXPECT_EQ(prog.linear_costs().size(), num_linear_costs);
  EXPECT_EQ(prog.linear_costs().back().constraint(), binding.constraint());
  EXPECT_TRUE(CheckStructuralEquality(prog.linear_costs().back().variables(),
                                      binding.variables()));
  EXPECT_EQ(binding.constraint()->num_constraints(), 1);
  auto cnstr = prog.linear_costs().back().constraint();
  auto vars = prog.linear_costs().back().variables();
  const Expression cx{(cnstr->A() * vars)(0)};
  double constant_term{0};
  if (is_addition(e)) {
    constant_term = get_constant_in_addition(e);
  } else if (is_constant(e)) {
    constant_term = get_constant_value(e);
  }
  EXPECT_TRUE((e - cx).EqualTo(constant_term));
}

void CheckAddedSymbolicLinearCost(MathematicalProgram* prog,
                                  const Expression& e) {
  int num_linear_costs = prog->linear_costs().size();
  auto binding1 = prog->AddLinearCost(e);
  CheckAddedSymbolicLinearCostUserFun(*prog, e, binding1, ++num_linear_costs);
  auto binding2 = prog->AddCost(e);
  CheckAddedSymbolicLinearCostUserFun(*prog, e, binding2, ++num_linear_costs);
}

GTEST_TEST(testMathematicalProgram, AddLinearCostSymbolic) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  // Add linear cost 2 * x(0) + 3 * x(1)
  CheckAddedSymbolicLinearCost(&prog, 2 * x(0) + 3 * x(1));
  // Add linear cost x(1)
  CheckAddedSymbolicLinearCost(&prog, +x(1));
  // Add linear cost x(0) + 2
  CheckAddedSymbolicLinearCost(&prog, x(0) + 2);
  // Add linear cost 2 * x(0) + 3 * x(1) + 2
  CheckAddedSymbolicLinearCost(&prog, 2 * x(0) + 3 * x(1) + 2);
  // Add linear cost 2 * x(1)
  CheckAddedSymbolicLinearCost(&prog, 2 * x(1));
  // Add linear (constant) cost 3
  CheckAddedSymbolicLinearCost(&prog, 3);
  // Add linear cost -x(0)
  CheckAddedSymbolicLinearCost(&prog, -x(0));
  // Add linear cost -(x(1) + 3 * x(0))
  CheckAddedSymbolicLinearCost(&prog, -(x(1) + 3 * x(0)));
  // Add linear cost x(1)*x(1) + x(0) - x(1)*x(1)
  CheckAddedSymbolicLinearCost(&prog, x(1) * x(1) + x(0) - x(1) * x(1));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic1) {
  // Add linear constraint: -10 <= 3 - 5*x0 + 10*x2 - 7*y1 <= 10
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  auto y = prog.NewContinuousVariables(3, "y");
  const Expression e{3 - 5 * x(0) + 10 * x(2) - 7 * y(1)};
  const double lb{-10};
  const double ub{+10};
  const auto binding = prog.AddLinearConstraint(e, lb, ub);

  // Check if the binding includes the correct linear constraint.
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.constraint();
  EXPECT_EQ(constraint_ptr->num_constraints(), 1u);
  const Expression Ax{(constraint_ptr->A() * var_vec)(0, 0)};
  const Expression lb_in_ctr{constraint_ptr->lower_bound()[0]};
  const Expression ub_in_ctr{constraint_ptr->upper_bound()[0]};
  EXPECT_TRUE((e - lb).EqualTo(Ax - lb_in_ctr));
  EXPECT_TRUE((e - ub).EqualTo(Ax - ub_in_ctr));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic2) {
  // Add linear constraint: -10 <= x0 <= 10
  // Note that this constraint is a bounding-box constraint which is a sub-class
  // of linear-constraint.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  const Expression e{x(0)};
  const auto binding = prog.AddLinearConstraint(e, -10, 10);

  // Check that the constraint in the binding is of BoundingBoxConstraint by
  // using dynamic_pointer_cast.
  const std::shared_ptr<BoundingBoxConstraint> constraint_ptr{
      std::dynamic_pointer_cast<BoundingBoxConstraint>(binding.constraint())};
  EXPECT_TRUE(constraint_ptr != nullptr);
  EXPECT_EQ(constraint_ptr->num_constraints(), 1u);

  // Check if the binding includes the correct linear constraint.
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const Expression Ax{(constraint_ptr->A() * var_vec)(0, 0)};
  const Expression lb_in_ctr{constraint_ptr->lower_bound()[0]};
  const Expression ub_in_ctr{constraint_ptr->upper_bound()[0]};
  EXPECT_TRUE((e - -10).EqualTo(Ax - lb_in_ctr));
  EXPECT_TRUE((e - 10).EqualTo(Ax - ub_in_ctr));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic3) {
  // Add linear constraints
  //     3 <=  3 - 5*x0 +      + 10*x2        - 7*y1        <= 9
  //   -10 <=                       x2                      <= 10
  //    -7 <= -5 + 2*x0 + 3*x2         + 3*y0 - 2*y1 + 6*y2 <= 12
  //     2 <=                     2*x2                      <= 3
  //     1 <=                                 -   y1        <= 3
  //
  // Note: the second, fourth and fifth rows are actually a bounding-box
  // constraints but we still process the five symbolic-constraints into a
  // single linear-constraint whose coefficient matrix is the following.
  //
  //         [-5 0 10 0 -7 0]
  //         [ 0 0  1 0  0 0]
  //         [ 2 3  0 3 -2 6]
  //         [ 0 0  2 0  0 0]
  //         [ 0 0  0 0 -1 0]
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  auto y = prog.NewContinuousVariables(3, "y");
  Matrix<Expression, 5, 1> M_e;
  Matrix<double, 5, 1> M_lb;
  Matrix<double, 5, 1> M_ub;

  // clang-format off
  M_e  <<  3 - 5 * x(0) + 10 * x(2) - 7 * y(1),
      +x(2),
      -5 + 2 * x(0) + 3 * x(2) + 3 * y(0) - 2 * y(1) + 6 * y(2),
      2 * x(2),
      -y(1);
  M_lb <<  3,
      -10,
      -7,
       2,
      1;
  M_ub << -7,
      10,
      12,
      3,
      3;
  // clang-format on

  // Check if the binding includes the correct linear constraint.
  const auto binding = prog.AddLinearConstraint(M_e, M_lb, M_ub);
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.constraint();
  EXPECT_EQ(constraint_ptr->num_constraints(), 5u);
  const auto Ax = constraint_ptr->A() * var_vec;
  const auto lb_in_ctr = constraint_ptr->lower_bound();
  const auto ub_in_ctr = constraint_ptr->upper_bound();

  for (int i = 0; i < M_e.size(); ++i) {
    EXPECT_PRED2(ExprEqual, M_e(i) - M_lb(i), Ax(i) - lb_in_ctr(i));
    EXPECT_PRED2(ExprEqual, M_e(i) - M_ub(i), Ax(i) - ub_in_ctr(i));
  }
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic4) {
  // Check the linear constraint 2  <= 2 * x <= 4.
  // Note: this is a bounding box constraint
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  const Expression e(2 * x(1));
  const auto& binding = prog.AddLinearConstraint(e, 2, 4);

  EXPECT_TRUE(prog.linear_constraints().empty());
  EXPECT_EQ(prog.bounding_box_constraints().size(), 1u);
  EXPECT_EQ(prog.bounding_box_constraints().back().constraint(),
            binding.constraint());
  EXPECT_EQ(prog.bounding_box_constraints().back().variables(),
            binding.variables());
  EXPECT_EQ(binding.variables(), VectorDecisionVariable<1>(x(1)));
  EXPECT_TRUE(
      CompareMatrices(binding.constraint()->lower_bound(), Vector1d(1)));
  EXPECT_TRUE(
      CompareMatrices(binding.constraint()->upper_bound(), Vector1d(2)));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic5) {
  // Check the linear constraint 2  <= -2 * x <= 4.
  // Note: this is a bounding box constraint
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  const Expression e(-2 * x(1));
  const auto& binding = prog.AddLinearConstraint(e, 2, 4);

  EXPECT_TRUE(prog.linear_constraints().empty());
  EXPECT_EQ(prog.bounding_box_constraints().size(), 1u);
  EXPECT_EQ(prog.bounding_box_constraints().back().constraint(),
            binding.constraint());
  EXPECT_EQ(prog.bounding_box_constraints().back().variables(),
            binding.variables());
  EXPECT_EQ(binding.variables(), VectorDecisionVariable<1>(x(1)));
  EXPECT_TRUE(
      CompareMatrices(binding.constraint()->lower_bound(), Vector1d(-2)));
  EXPECT_TRUE(
      CompareMatrices(binding.constraint()->upper_bound(), Vector1d(-1)));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic6) {
  // Checks the linear constraint 1 <= -x <= 3.
  // Note: this is a bounding box constraint.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  const Expression e(-x(0));
  const auto& binding = prog.AddLinearConstraint(e, 1, 3);
  EXPECT_TRUE(prog.linear_constraints().empty());
  EXPECT_EQ(prog.bounding_box_constraints().size(), 1);
  EXPECT_EQ(prog.bounding_box_constraints().back().constraint(),
            binding.constraint());
  EXPECT_EQ(prog.bounding_box_constraints().back().variables(),
            binding.variables());
  EXPECT_EQ(binding.variables(), VectorDecisionVariable<1>(x(0)));
  EXPECT_TRUE(
      CompareMatrices(binding.constraint()->lower_bound(), Vector1d(-3)));
  EXPECT_TRUE(
      CompareMatrices(binding.constraint()->upper_bound(), Vector1d(-1)));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic7) {
  // Checks the linear constraints
  // 1 <= -2 * x0 <= 3
  // 3 <= 4 * x0 + 2<= 5
  // 2 <= x1 + 2 * (x2 - 0.5*x1) + 3 <= 4;
  // 3 <= -4 * x1 + 3 <= inf
  // Note: these are all bounding box constraints.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  Vector4<Expression> e;
  // clang-format off
  e << -2 * x(0),
    4 * x(0) + 2,
    x(1) + 2 * (x(2) - 0.5 * x(1)) + 3,
    -4 * x(1) + 3;
  // clang-format on
  prog.AddLinearConstraint(
      e, Vector4d(1, 3, 2, 3),
      Vector4d(3, 5, 4, numeric_limits<double>::infinity()));

  EXPECT_EQ(prog.bounding_box_constraints().size(), 1);
  const auto& binding = prog.bounding_box_constraints().back();
  EXPECT_EQ(binding.variables(),
            VectorDecisionVariable<4>(x(0), x(0), x(2), x(1)));
  EXPECT_TRUE(CompareMatrices(
      binding.constraint()->lower_bound(),
      Vector4d(-1.5, 0.25, -0.5, -numeric_limits<double>::infinity())));
  EXPECT_TRUE(CompareMatrices(binding.constraint()->upper_bound(),
                              Vector4d(-0.5, 0.75, 0.5, 0)));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic8) {
  // Test the failure cases for adding linear constraint.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  // Non-polynomial.
  EXPECT_THROW(prog.AddLinearConstraint(sin(x(0)), 1, 2), std::runtime_error);

  // Non-linear.
  EXPECT_THROW(prog.AddLinearConstraint(x(0) * x(0), 1, 2), std::runtime_error);

  // Trivial (and infeasible) case 1 <= 0 <= 2
  EXPECT_THROW(prog.AddLinearConstraint(x(0) - x(0), 1, 2), std::runtime_error);
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic9) {
  // Test trivial constraint with no variables, such as 1 <= 2 <= 3
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  prog.AddLinearConstraint(Expression(2), 1, 3);
  EXPECT_EQ(prog.linear_constraints().size(), 1);
  auto binding = prog.linear_constraints().back();
  EXPECT_EQ(binding.constraint()->A().rows(), 1);
  EXPECT_EQ(binding.constraint()->A().cols(), 0);

  Vector2<Expression> expr;
  expr << 2, x(0);
  prog.AddLinearConstraint(expr, Vector2d(1, 2), Vector2d(3, 4));
  EXPECT_EQ(prog.linear_constraints().size(), 2);
  binding = prog.linear_constraints().back();
  EXPECT_TRUE(
      CompareMatrices(binding.constraint()->A(), Eigen::Vector2d(0, 1)));
  EXPECT_TRUE(CompareMatrices(binding.constraint()->lower_bound(),
                              Eigen::Vector2d(-1, 2)));
  EXPECT_TRUE(CompareMatrices(binding.constraint()->upper_bound(),
                              Eigen::Vector2d(1, 4)));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicFormula1) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  int num_bounding_box_constraint = 0;

  // x(0) <= 3
  std::vector<symbolic::Formula> f;
  f.push_back(x(0) <= 3);
  f.push_back(3 >= x(0));
  f.push_back(x(0) + 2 <= 5);
  f.push_back(4 + x(0) >= 1 + 2 * x(0));
  f.push_back(2 * x(0) + 1 <= 4 + x(0));
  f.push_back(3 * x(0) + x(1) <= 6 + x(0) + x(1));
  for (const auto &fi : f) {
    prog.AddLinearConstraint(fi);
    EXPECT_EQ(++num_bounding_box_constraint,
              prog.bounding_box_constraints().size());
    EXPECT_EQ(prog.linear_constraints().size(), 0);
    EXPECT_EQ(prog.linear_equality_constraints().size(), 0);
    auto binding = prog.bounding_box_constraints().back();
    EXPECT_EQ(binding.variables(), VectorDecisionVariable<1>(x(0)));
    EXPECT_TRUE(CompareMatrices(binding.constraint()->upper_bound(),
                                Vector1d(3)));
    EXPECT_TRUE(CompareMatrices(binding.constraint()->lower_bound(),
                                Vector1d(-numeric_limits<double>::infinity())));
  }
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicFormula2) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  int num_bounding_box_constraint = 0;
  // x0 >= 2
  std::vector<symbolic::Formula> f;
  f.push_back(x(0) >= 2);
  f.push_back(2 <= x(0));
  f.push_back(-x(0) <= -2);
  f.push_back(-2 >= -x(0));
  f.push_back(2 + 2 * x(0) >= x(0) + 4);
  f.push_back(3 + 3 * x(0) >= x(0) + 7);
  f.push_back(x(0) + 7 + 2 * x(1) <= 3 * x(0) + 3 + 2 * x(1));
  for (const auto& fi : f) {
    prog.AddLinearConstraint(fi);
    EXPECT_EQ(++num_bounding_box_constraint,
              prog.bounding_box_constraints().size());
    EXPECT_EQ(prog.linear_constraints().size(), 0);
    EXPECT_EQ(prog.linear_equality_constraints().size(), 0);
    auto binding = prog.bounding_box_constraints().back();
    EXPECT_EQ(binding.variables(), VectorDecisionVariable<1>(x(0)));
    EXPECT_TRUE(CompareMatrices(binding.constraint()->lower_bound(),
                                Vector1d(2)));
    EXPECT_TRUE(CompareMatrices(binding.constraint()->upper_bound(),
                                Vector1d(numeric_limits<double>::infinity())));
  }
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicFormula3) {
  // x(0) + x(2) == 1
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  int num_linear_equality_constraint = 0;

  std::vector<symbolic::Formula> f;
  f.push_back(x(0) + x(2) == 1);
  f.push_back(x(0) + 2 * x(2) == 1 + x(2));
  for (const auto& fi : f) {
    prog.AddLinearConstraint(fi);
    EXPECT_EQ(++num_linear_equality_constraint,
              prog.linear_equality_constraints().size());
    EXPECT_EQ(prog.linear_constraints().size(), 0);
    EXPECT_EQ(prog.bounding_box_constraints().size(), 0);
    auto binding = prog.linear_equality_constraints().back();
    EXPECT_TRUE(
        CompareMatrices(binding.constraint()->lower_bound(), Vector1d(1)));

    VectorX<Expression> expr = binding.constraint()->A() * binding.variables()
        - binding.constraint()->lower_bound();
    EXPECT_EQ(expr.size(), 1);
    EXPECT_PRED2(ExprEqual, expr(0), x(0) + x(2) - 1);
  }
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicFormula4) {
  // x(0) + 2 * x(2) <= 1
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  int num_linear_constraint = 0;

  std::vector<symbolic::Formula> f;
  f.push_back(x(0) + 2 * x(2) <= 1);
  f.push_back(x(0) + 3 * x(2) <= 1 + x(2));
  f.push_back(-1 <= -x(0) - 2 * x(2));
  f.push_back(-1 - x(2) <= -x(0) - 3 * x(2));
  f.push_back(2 * (x(0) + x(2)) - x(0) <= 1);
  f.push_back(1 >= x(0) + 2 * x(2));
  f.push_back(1 + x(2) >= x(0) + 3 * x(2));
  f.push_back(-x(0) - 2 * x(2) >= -1);
  f.push_back(-x(0) - 3 * x(2) >= -1 - x(2));
  f.push_back(1 >= 2 * (x(0) + x(2)) - x(0));
  for (const auto& fi : f) {
    prog.AddLinearConstraint(fi);
    EXPECT_EQ(++num_linear_constraint,
              prog.linear_constraints().size());
    EXPECT_EQ(prog.linear_equality_constraints().size(), 0);
    EXPECT_EQ(prog.bounding_box_constraints().size(), 0);
    auto binding = prog.linear_constraints().back();
    EXPECT_TRUE(CompareMatrices(binding.constraint()->upper_bound(),
                                Vector1d(numeric_limits<double>::infinity())));
    EXPECT_TRUE(
        CompareMatrices(binding.constraint()->lower_bound(), Vector1d(-1)));

    VectorX<Expression> expr = binding.constraint()->A() * binding.variables()
        - binding.constraint()->lower_bound();
    EXPECT_EQ(expr.size(), 1);
    EXPECT_PRED2(ExprEqual, expr(0), 1 - x(0) - 2 * x(2));
  }
}

// Checks AddLinearConstraint function which takes an Eigen::Array<Formula>.
GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicArrayFormula1) {
  // Add linear constraints
  //    3 - 5*x0 +      + 10*x2        - 7*y1         >= 3
  //                         x2                       >= -10
  //   -5 + 2*x0 + 3*x1         + 3*y0 - 2*y1 + 6*y2  <= -7
  //                       2*x2                       == 2
  //                                   -   y1         >= 1
  //
  // Note: the second, fourth and fifth rows are actually a bounding-box
  // constraints but we still process the five symbolic-constraints into a
  // single linear-constraint whose coefficient matrix is the following.
  //
  //         [-5 0 10 0 -7 0]
  //         [ 0 0  1 0  0 0]
  //         [ 2 3  0 3 -2 6]
  //         [ 0 0  2 0  0 0]
  //         [ 0 0  0 0 -1 0]
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  auto y = prog.NewContinuousVariables(3, "y");
  Matrix<Formula, 5, 1> M_f;
  // clang-format off
  M_f << (3 - 5 * x(0) + 10 * x(2) - 7 * y(1) >= 3),
         x(2) >= -10,
         -5 + 2 * x(0) + 3 * x(1) + 3 * y(0) - 2 * y(1) + 6 * y(2) <= -7,
         2 * x(2) == 2,
         -y(1) >= 1;
  // clang-format on

  // Check if the binding includes the correct linear constraint.
  const auto binding = prog.AddLinearConstraint(M_f.array());
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.constraint();
  EXPECT_EQ(constraint_ptr->num_constraints(), 5u);
  const auto Ax = constraint_ptr->A() * var_vec;
  const auto lb_in_ctr = constraint_ptr->lower_bound();
  const auto ub_in_ctr = constraint_ptr->upper_bound();
  for (int i{0}; i < M_f.size(); ++i) {
    if (!std::isinf(lb_in_ctr(i))) {
      EXPECT_PRED2(ExprEqual,
                   get_lhs_expression(M_f(i)) - get_rhs_expression(M_f(i)),
                   Ax(i) - lb_in_ctr(i));
    }
    if (!std::isinf(ub_in_ctr(i))) {
      EXPECT_PRED2(ExprEqual,
                   get_lhs_expression(M_f(i)) - get_rhs_expression(M_f(i)),
                   Ax(i) - ub_in_ctr(i));
    }
  }
}

// Checks AddLinearConstraint function which takes an Eigen::Array<Formula>.
// This test uses operator>= provided for Eigen::Array<Expression> and
// Eigen::Array<double>.
GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicArrayFormula2) {
  // Add linear constraints
  //
  //     M_f = |f1 f2|
  //           |f3 f4|
  //
  // where
  //   f1 =  3 - 5*x0        +10*x2        - 7*y1        >= 3
  //   f2 =                      x2                      >= -10
  //   f3 = -5 + 2*x0 + 3*x1        + 3*y0 - 2*y1 + 6*y2 >= -7
  //   f4 =                    2*x2                      >= 2
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  auto y = prog.NewContinuousVariables(3, "y");
  Matrix<Expression, 2, 2> M_e;
  Matrix<double, 2, 2> M_lb;
  // clang-format off
  M_e << 3 - 5 * x(0) + 10 * x(2) - 7 * y(1),                       x(2),
        -5 + 2 * x(0) +  3 * x(1) + 3 * y(0) - 2 * y(1) + 6 * y(2), 2 * x(2);
  M_lb << 3, -10,
         -7,   2;
  // clang-format on
  Eigen::Array<Formula, 2, 2> M_f{M_e.array() >= M_lb.array()};

  // Check if the binding includes the correct linear constraint.
  const auto binding = prog.AddLinearConstraint(M_f);
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.constraint();
  EXPECT_EQ(constraint_ptr->num_constraints(), M_e.rows() * M_e.cols());
  const auto Ax = constraint_ptr->A() * var_vec;
  const auto lb_in_ctr = constraint_ptr->lower_bound();
  const auto ub_in_ctr = constraint_ptr->upper_bound();
  int k{0};
  for (int i{0}; i < M_e.rows(); ++i) {
    for (int j{0}; j < M_e.cols(); ++j, ++k) {
      EXPECT_PRED2(ExprEqual, M_e(i, j) - M_lb(i, j), Ax(k) - lb_in_ctr(k));
    }
  }
}

// Checks AddLinearConstraint function which takes an Eigen::Array<Formula>.
GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicArrayFormula3) {
  // Add linear constraints
  //
  //   (A*x >= b) = |1 2| * |x0| >= |5|
  //                |3 4|   |x1|    |6|
  //
  //              = |  x0 + 2*x1 >= 5|
  //                |3*x0 + 4*x1 >= 6|
  //
  // where A = |1 2|, x = |x0|, b = |5|
  //           |3 4|      |x1|      |6|.
  //
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2, "x");
  Matrix<Expression, 2, 2> A;
  Eigen::Vector2d b;
  // clang-format off
  A << 1, 2,
       3, 4;
  b << 5,
       6;
  // clang-format on
  const auto binding = prog.AddLinearConstraint((A * x).array() >= b.array());
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.constraint();
  EXPECT_EQ(constraint_ptr->num_constraints(), b.size());
  const auto Ax = constraint_ptr->A() * var_vec;
  const auto lb_in_ctr = constraint_ptr->lower_bound();
  const auto ub_in_ctr = constraint_ptr->upper_bound();
  EXPECT_PRED2(ExprEqual, x(0) + 2 * x(1) - 5, Ax(0) - lb_in_ctr(0));
  EXPECT_PRED2(ExprEqual, 3 * x(0) + 4 * x(1) - 6, Ax(1) - lb_in_ctr(1));
}

// Checks AddLinearConstraint function which takes an Eigen::Array<Formula>.
GTEST_TEST(testMathematicalProgram,
           AddLinearConstraintSymbolicArrayFormulaException) {
  // Add linear constraints
  //    3 - 5*x0 + 10*x2 - 7*y1 >  3
  //                  x2        > -10
  // Note that this includes strict inequality (>) and results in an exception.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  auto y = prog.NewContinuousVariables(3, "y");
  Matrix<Formula, 2, 1> M_f;
  // clang-format off
  M_f << (3 - 5 * x(0) + 10 * x(2) - 7 * y(1) > 3),
                              x(2)            > -10;
  // clang-format on
  EXPECT_THROW(prog.AddLinearConstraint(M_f.array()), std::runtime_error);
}

namespace {
void CheckAddedLinearEqualityConstraintCommon(
    const Binding<LinearEqualityConstraint>& binding,
    const MathematicalProgram& prog, int num_linear_eq_cnstr) {
  // Checks if the number of linear equality constraints get incremented by 1.
  EXPECT_EQ(prog.linear_equality_constraints().size(), num_linear_eq_cnstr + 1);
  // Checks if the newly added linear equality constraint in prog is the same as
  // that returned from AddLinearEqualityConstraint.
  EXPECT_EQ(prog.linear_equality_constraints().back().constraint(),
            binding.constraint());
  // Checks if the bound variables of the newly added linear equality constraint
  // in prog is the same as that returned from AddLinearEqualityConstraint.
  EXPECT_EQ(prog.linear_equality_constraints().back().variables(),
            binding.variables());
}

template <typename DerivedV, typename DerivedB>
void CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
    MathematicalProgram* prog, const Eigen::MatrixBase<DerivedV>& v,
    const Eigen::MatrixBase<DerivedB>& b) {
  const int num_linear_eq_cnstr = prog->linear_equality_constraints().size();
  auto binding = prog->AddLinearEqualityConstraint(v, b);
  CheckAddedLinearEqualityConstraintCommon(binding, *prog, num_linear_eq_cnstr);
  // Checks if the number of rows in the newly added constraint is the same as
  // the input expression.
  int num_constraints_expected = v.size();

  EXPECT_EQ(binding.constraint()->num_constraints(), num_constraints_expected);
  // Check if the newly added linear equality constraint matches with the input
  // expression.
  VectorX<Expression> flat_V = binding.constraint()->A() * binding.variables() -
                               binding.constraint()->lower_bound();

  EXPECT_EQ(flat_V.size(), v.size());
  MatrixX<Expression> v_resize = flat_V;
  v_resize.resize(v.rows(), v.cols());
  for (int i = 0; i < v.rows(); ++i) {
    for (int j = 0; j < v.cols(); ++j) {
      EXPECT_EQ(v_resize(i, j).Expand(), (v(i, j) - b(i, j)).Expand());
    }
  }
}

template <typename DerivedV, typename DerivedB>
void CheckAddedSymmetricSymbolicLinearEqualityConstraint(
    MathematicalProgram* prog, const Eigen::MatrixBase<DerivedV>& v,
    const Eigen::MatrixBase<DerivedB>& b) {
  const int num_linear_eq_cnstr = prog->linear_equality_constraints().size();
  auto binding = prog->AddLinearEqualityConstraint(v, b, true);
  CheckAddedLinearEqualityConstraintCommon(binding, *prog, num_linear_eq_cnstr);
  // Checks if the number of rows in the newly added constraint is the same as
  // the input expression.
  int num_constraints_expected = v.rows() * (v.rows() + 1) / 2;
  EXPECT_EQ(binding.constraint()->num_constraints(), num_constraints_expected);
  // Check if the newly added linear equality constraint matches with the input
  // expression.
  VectorX<Expression> flat_V = binding.constraint()->A() * binding.variables() -
                               binding.constraint()->lower_bound();
  EXPECT_EQ(math::ToSymmetricMatrixFromLowerTriangularColumns(flat_V), v - b);
}

void CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
    MathematicalProgram* prog, const Expression& e, double b) {
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      prog, Vector1<Expression>(e), Vector1d(b));
}
}  // namespace

GTEST_TEST(testMathematicalProgram, AddSymbolicLinearEqualityConstraint1) {
  // Checks the single row linear equality constraint one by one:

  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");

  // Checks x(0) = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog, +x(0), 1);
  // Checks x(1) = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog, +x(1), 1);
  // Checks x(0) + x(1) = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog, x(0) + x(1), 1);
  // Checks x(0) + 2 * x(1) = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog, x(0) + 2 * x(1),
                                                         1);
  // Checks 3 * x(0) - 2 * x(1) = 2
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      &prog, 3 * x(0) - 2 * x(1), 2);
  // Checks 2 * x(0) = 2
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog, 2 * x(0), 2);
  // Checks x(0) + 3 * x(2) = 3
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog, x(0) + 3 * x(2),
                                                         3);
  // Checks 2 * x(1) - 3 * x(2) = 4
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      &prog, 2 * x(1) - 3 * x(2), 4);
  // Checks x(0) + 2 = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog, x(0) + 2, 1);
  // Checks x(1) - 2 = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog, x(1) - 2, 1);
  // Checks 3 * x(1) + 4 = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog, 3 * x(1) + 4,
                                                         1);
  // Checks x(0) + x(2) + 3 = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog, x(0) + x(2) + 3,
                                                         1);
  // Checks 2 * x(0) + x(2) - 3 = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      &prog, 2 * x(0) + x(2) - 3, 1);
  // Checks 3 * x(0) + x(1) + 4 * x(2) + 1 = 2
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      &prog, 3 * x(0) + x(1) + 4 * x(2) + 1, 2);
  // Checks -x(1) = 3
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog, -x(1), 3);
  // Checks -(x(0) + 2 * x(1)) = 2
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog,
                                                         -(x(0) + 2 * x(1)), 2);

  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      &prog, x(0) + 2 * (x(0) + x(2)) + 3 * (x(0) - x(1)), 3);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicLinearEqualityConstraint2) {
  // Checks adding multiple rows of linear equality constraints.

  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");

  // Checks x(1) = 2
  //        x(0) = 1
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      &prog, Vector2<Expression>(+x(1), +x(0)), Eigen::Vector2d(2, 1));

  // Checks 2 * x(1) = 3
  //        x(0) + x(2) = 4
  //        x(0) + 3 * x(1) + 7 = 1
  Vector3<Expression> v{};
  v << 2 * x(1), x(0) + x(2), x(0) + 3 * x(1) + 7;
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      &prog, v, Eigen::Vector3d(3, 4, 1));

  // Checks x(0) = 4
  //          1  = 1
  //       -x(1) = 2
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      &prog, Vector3<Expression>(+x(0), 1, -x(1)), Eigen::Vector3d(4, 1, 2));
}

GTEST_TEST(testMathematicalProgram, AddSymbolicLinearEqualityConstraint3) {
  // Checks adding a matrix of linear equality constraints. This matrix is not
  // symmetric.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables<2, 2>("X");

  // Checks A * X = [1, 2; 3, 4], both A * X and B are static sized.
  Eigen::Matrix2d A{};
  A << 1, 3, 4, 2;
  Eigen::Matrix2d B{};
  B << 1, 2, 3, 4;
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog, A * X, B);

  // Checks A * X = B, with A*X being dynamic sized, and B being static sized.
  MatrixX<Expression> A_times_X = A * X;
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog, A_times_X, B);

  // Checks A * X = B, with A*X being static sized, and B being dynamic sized.
  Eigen::MatrixXd B_dynamic = B;
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog, A * X,
                                                         B_dynamic);

  // Checks A * X = B, with both A*X and B being dynamic sized.
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(&prog, A_times_X,
                                                         B_dynamic);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicLinearEqualityConstraint4) {
  // Checks adding a symmetric matrix of linear equality constraints.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<2>("X");
  Eigen::Matrix2d A{};
  A << 1, 3, 4, 2;
  Eigen::Matrix2d B{};
  B << 1, 2, 2, 1;
  Eigen::MatrixXd B_dynamic = B;
  Matrix2<Expression> M = A.transpose() * X + X * A;
  MatrixX<Expression> M_dynamic = M;

  // Checks Aᵀ * X + X * A = B, both the left and right hand-side are static
  // sized.
  CheckAddedSymmetricSymbolicLinearEqualityConstraint(&prog, M, B);

  // Checks Aᵀ * X + X * A = B, the left hand-side being static sized, while the
  // right hand-side being dynamic sized.
  CheckAddedSymmetricSymbolicLinearEqualityConstraint(&prog, M, B_dynamic);

  // Checks Aᵀ * X + X * A = B, the left hand-side being dynamic sized, while
  // the
  // right hand-side being static sized.
  CheckAddedSymmetricSymbolicLinearEqualityConstraint(&prog, M_dynamic, B);

  // Checks Aᵀ * X + X * A = B, bot the left and right hand-side are dynamic
  // sized.
  CheckAddedSymmetricSymbolicLinearEqualityConstraint(&prog, M_dynamic,
                                                      B_dynamic);

  // Checks Aᵀ * X + X * A = E.
  CheckAddedSymmetricSymbolicLinearEqualityConstraint(
      &prog, M, Eigen::Matrix2d::Identity());
}

namespace {
bool AreTwoPolynomialsNear(
    const symbolic::MonomialToCoefficientMap &poly1,
    const symbolic::MonomialToCoefficientMap &poly2,
    double tol = std::numeric_limits<double>::epsilon()) {
  // TODO(hongkai.dai): rewrite this part when we have function to add and
  // subtract two polynomials.
  symbolic::MonomialToCoefficientMap poly_diff;
  poly_diff.reserve(poly1.size() + poly2.size());
  poly_diff.insert(poly1.begin(), poly1.end());
  for (const auto& p2 : poly2) {
    const auto it = poly_diff.find(p2.first);
    if (it == poly_diff.end()) {
      poly_diff.emplace_hint(it, p2.first, -p2.second);
    } else {
      it->second -= p2.second;
    }
  }
  return std::all_of(poly_diff.begin(), poly_diff.end(), [&tol](const auto& p) {
    return std::abs(symbolic::get_constant_value(p.second)) <= tol;
  });
}

void CheckParsedSymbolicLorentzConeConstraint(
    MathematicalProgram* prog, const Expression& linear_expr,
    const Expression& quadratic_expr) {
  const auto& binding1 =
      prog->AddLorentzConeConstraint(linear_expr, quadratic_expr);
  const auto& binding2 = prog->lorentz_cone_constraints().back();
  EXPECT_EQ(binding1.constraint(), binding2.constraint());
  EXPECT_EQ(binding1.variables(), binding2.variables());
  // Now check if the linear and quadratic constraints are parsed correctly.
  const VectorX<Expression> e_parsed =
      binding1.constraint()->A() * binding1.variables() +
      binding1.constraint()->b();
  EXPECT_PRED2(ExprEqual, (e_parsed(0) * e_parsed(0)).Expand(),
               (linear_expr * linear_expr).Expand());
  Expression quadratic_expr_parsed =
      e_parsed.tail(e_parsed.rows() - 1).squaredNorm();
  // Due to the small numerical error, quadratic_expr and quadratic_expr_parsed
  // do not match exactly.So we will compare each term in the two polynomials,
  // and regard them to be equal if the error in the coefficient is sufficiently
  // small.
  const auto& monomial_to_coeff_map_parsed =
      symbolic::DecomposePolynomialIntoMonomial(
          quadratic_expr_parsed, quadratic_expr_parsed.GetVariables());
  const auto& monomial_to_coeff_map = symbolic::DecomposePolynomialIntoMonomial(
      quadratic_expr, quadratic_expr.GetVariables());
  double tol = 100 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(AreTwoPolynomialsNear(monomial_to_coeff_map_parsed,
                                    monomial_to_coeff_map, tol));
}

void CheckParsedSymbolicLorentzConeConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const Eigen::Matrix<Expression, Eigen::Dynamic, 1>>& e) {
  const auto& binding1 = prog->AddLorentzConeConstraint(e);
  const auto& binding2 = prog->lorentz_cone_constraints().back();

  EXPECT_EQ(binding1.constraint(), binding2.constraint());
  EXPECT_EQ(binding1.constraint()->A() * binding1.variables() +
                binding1.constraint()->b(),
            e);
  EXPECT_EQ(binding2.constraint()->A() * binding2.variables() +
                binding2.constraint()->b(),
            e);

  CheckParsedSymbolicLorentzConeConstraint(prog, e(0),
                                           e.tail(e.rows() - 1).squaredNorm());
}

void CheckParsedSymbolicRotatedLorentzConeConstraint(
    MathematicalProgram* prog, const Eigen::Ref<const VectorX<Expression>>& e) {
  const auto& binding1 = prog->AddRotatedLorentzConeConstraint(e);
  const auto& binding2 = prog->rotated_lorentz_cone_constraints().back();

  EXPECT_EQ(binding1.constraint(), binding2.constraint());
  EXPECT_EQ(binding1.constraint()->A() * binding1.variables() +
                binding1.constraint()->b(),
            e);
  EXPECT_EQ(binding2.constraint()->A() * binding2.variables() +
                binding2.constraint()->b(),
            e);
}
}  // namespace

class SymbolicLorentzConeTest : public ::testing::Test {
 public:
  SymbolicLorentzConeTest() : prog_(), x_() {
    x_ = prog_.NewContinuousVariables<3>("x");
  }

 protected:
  MathematicalProgram prog_;
  VectorDecisionVariable<3> x_;
};

TEST_F(SymbolicLorentzConeTest, Test1) {
  // Add Lorentz cone constraint:
  // x is in Lorentz cone
  Matrix<Expression, 3, 1> e;
  e << 1 * x_(0), 1.0 * x_(1), 1.0 * x_(2);
  CheckParsedSymbolicLorentzConeConstraint(&prog_, e);
}

TEST_F(SymbolicLorentzConeTest, Test2) {
  // Add Lorentz cone constraint:
  // x + [1, 2, 0] is in Lorentz cone.
  Matrix<Expression, 3, 1> e;
  e << x_(0) + 1, x_(1) + 2, +x_(2);
  CheckParsedSymbolicLorentzConeConstraint(&prog_, e);
}

TEST_F(SymbolicLorentzConeTest, Test3) {
  // Add Lorentz cone constraint:
  // [2 * x(0) + 3 * x(2)]
  // [  - x(0) + 2 * x(2)]    is in Lorentz cone
  // [               x(2)]
  // [  -x(1)            ]
  Matrix<Expression, 4, 1> e;
  // clang-format on
  e << 2 * x_(0) + 3 * x_(2), -x_(0) + 2 * x_(2), +x_(2), -x_(1);
  // clang-format off;
  CheckParsedSymbolicLorentzConeConstraint(&prog_, e);
}

TEST_F(SymbolicLorentzConeTest, Test4) {
  // Add Lorentz cone constraint:
  // [ 2 * x(0) + 3 * x(1) +            5]
  // [ 4 * x(0)            + 4 * x(2) - 7]
  // [                                 10]
  // [                       2 * x(2)    ]
  Matrix<Expression, 4, 1> e;
  // clang-format off
  e << 2 * x_(0) + 3 * x_(1) + 5,
       4 * x_(0) + 4 * x_(2) - 7,
       10,
       2 * x_(2);
  // clang-format on
  CheckParsedSymbolicLorentzConeConstraint(&prog_, e);
}

TEST_F(SymbolicLorentzConeTest, Test5) {
  // Add Lorentz cone constraint:
  // [x(0); x(1); x(2); 0] is in the Lorentz cone.
  Vector4<Expression> e;
  e << x_(0), x_(1), x_(2), 0;
  CheckParsedSymbolicLorentzConeConstraint(&prog_, e);
}

TEST_F(SymbolicLorentzConeTest, Test6) {
  // Add Lorentz cone constraint:
  // [x(0); x(1) + x(2)] is in the Lorentz cone.
  Vector2<Expression> e;
  e << x_(0), x_(1) + x_(2);
  CheckParsedSymbolicLorentzConeConstraint(&prog_, e);
}

TEST_F(SymbolicLorentzConeTest, Test7) {
  CheckParsedSymbolicLorentzConeConstraint(
      &prog_, x_(0) + 2, pow(x_(0), 2) + 4 * x_(0) * x_(1) + 4 * pow(x_(1), 2));
}

TEST_F(SymbolicLorentzConeTest, Test8) {
  CheckParsedSymbolicLorentzConeConstraint(
      &prog_, x_(0) + 2,
      pow(x_(0), 2) - (x_(0) - x_(1)) * (x_(0) + x_(1)) + 2 * x_(1) + 3);
}

TEST_F(SymbolicLorentzConeTest, Test9) {
  CheckParsedSymbolicLorentzConeConstraint(&prog_, 2,
                                           pow(x_(0), 2) + pow(x_(1), 2));
}

TEST_F(SymbolicLorentzConeTest, TestLinearConstraint) {
  // Actually adding linear constraint, that the quadratic expression is
  // actually a constant.
  CheckParsedSymbolicLorentzConeConstraint(&prog_, x_(0) + 2, 1);
  CheckParsedSymbolicLorentzConeConstraint(&prog_, x_(0) + 2,
                                           x_(0) - 2 * (0.5 * x_(0) + 1) + 3);
}

TEST_F(SymbolicLorentzConeTest, TestError) {
  // Check the cases to add with invalid quadratic expression.

  // Check polynomial with order no smaller than 2
  EXPECT_THROW(prog_.AddLorentzConeConstraint(2 * x_(0) + 3, pow(x_(1), 3)),
               std::runtime_error);

  // The quadratic expression is actually affine.
  EXPECT_THROW(prog_.AddLorentzConeConstraint(2 * x_(0), 3 * x_(1) + 2),
               std::runtime_error);
  EXPECT_THROW(
      prog_.AddLorentzConeConstraint(
          2 * x_(0), x_(1) * x_(1) - (x_(1) - x_(0)) * (x_(1) + x_(0)) -
                         x_(0) * x_(0) + 2 * x_(1) + 3),
      std::runtime_error);

  // The Hessian matrix is not positive semidefinite.
  EXPECT_THROW(prog_.AddLorentzConeConstraint(2 * x_(0) + 3,
                                              x_(1) * x_(1) - x_(2) * x_(2)),
               std::runtime_error);
  EXPECT_THROW(
      prog_.AddLorentzConeConstraint(
          2 * x_(0) + 3, x_(1) * x_(1) + x_(2) * x_(2) + 3 * x_(1) * x_(2)),
      std::runtime_error);
  EXPECT_THROW(
      prog_.AddLorentzConeConstraint(
          2 * x_(0) + 3, x_(1) * x_(1) + x_(2) * x_(2) + 3 * x_(0) * x_(2)),
      std::runtime_error);

  // The quadratic expression is not always non-negative.
  EXPECT_THROW(prog_.AddLorentzConeConstraint(2 * x_(0) + 3,
                                             x_(1) * x_(1) + x_(2) * x_(2) - 1),
               std::runtime_error);
  EXPECT_THROW(prog_.AddLorentzConeConstraint(
                   2 * x_(0) + 3, pow(2 * x_(0) + 3 * x_(1) + 2, 2) - 1),
               std::runtime_error);

  // The quadratic expression is a negative constant.
  EXPECT_THROW(prog_.AddLorentzConeConstraint(
                   2 * x_(0) + 3, pow(x_(0), 2) - pow(x_(1), 2) -
                                      (x_(0) + x_(1)) * (x_(0) - x_(1)) - 1),
               std::runtime_error);

  // The first expression is not actually linear.
  EXPECT_THROW(prog_.AddLorentzConeConstraint(2 * x_(0) * x_(1), pow(x_(0), 2)),
               std::runtime_error);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicRotatedLorentzConeConstraint1) {
  // Add rotated Lorentz cone constraint:
  // x is in the rotated Lorentz cone constraint.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  Matrix<Expression, 3, 1> e;
  e << +x(0), +x(1), +x(2);
  CheckParsedSymbolicRotatedLorentzConeConstraint(&prog, e);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicRotatedLorentzConeConstraint2) {
  // Add rotated Lorentz cone constraint:
  // [x(0) + 2 * x(2)]
  // [x(0)           ] is in the rotated Lorentz cone
  // [           x(2)]
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  Matrix<Expression, 3, 1> e;
  e << x(0) + 2 * x(2), +x(0), +x(2);
  CheckParsedSymbolicRotatedLorentzConeConstraint(&prog, e);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicRotatedLorentzConeConstraint3) {
  // Add rotated Lorentz cone constraint:
  // [x(0) + 1]
  // [x(1) + 2] is in the rotated Lorentz cone
  // [x(2)    ]
  // [x(3) - 1]
  // [-x(1)   ]
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>("x");
  Matrix<Expression, 5, 1> e;
  e << x(0) + 1, x(1) + 2, +x(2), x(3) - 1, -x(1);
  CheckParsedSymbolicRotatedLorentzConeConstraint(&prog, e);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicRotatedLorentzConeConstraint4) {
  // Add rotated Lorentz cone constraint:
  // [2 * x(0) + 3 * x(2) + 3]
  // [    x(0) - 4 * x(2)    ] is in the rotated Lorentz cone
  // [           2 * x(2)    ]
  // [3 * x(0)            + 1]
  // [                      4]
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>("x");
  Matrix<Expression, 5, 1> e;
  e << 2 * x(0) + 3 * x(2) + 3, x(0) - 4 * x(2), 2 * x(2), 3 * x(0) + 1, 4;
  CheckParsedSymbolicRotatedLorentzConeConstraint(&prog, e);
}

namespace {
template <typename Derived>
typename std::enable_if<
    std::is_same<typename Derived::Scalar, symbolic::Expression>::value>::type
CheckAddedSymbolicPositiveSemidefiniteConstraint(
    MathematicalProgram* prog, const Eigen::MatrixBase<Derived>& V) {
  int num_psd_cnstr = prog->positive_semidefinite_constraints().size();
  int num_lin_eq_cnstr = prog->linear_equality_constraints().size();
  auto binding = prog->AddPositiveSemidefiniteConstraint(V);
  // Check if number of linear equality constraints and positive semidefinite
  // constraints are both incremented by 1.
  EXPECT_EQ(num_psd_cnstr + 1,
            prog->positive_semidefinite_constraints().size());
  EXPECT_EQ(num_lin_eq_cnstr + 1, prog->linear_equality_constraints().size());
  // Check if the returned binding is the correct one.
  EXPECT_EQ(
      binding.constraint().get(),
      prog->positive_semidefinite_constraints().back().constraint().get());
  // Check if the added linear constraint is correct. M is the newly added
  // variables representing the psd matrix.
  const Eigen::Map<const MatrixX<Variable>> M(&binding.variables()(0), V.rows(),
                                              V.cols());
  // The linear equality constraint is only imposed on the lower triangular
  // part of the psd matrix.
  const auto& new_lin_eq_cnstr = prog->linear_equality_constraints().back();
  auto V_minus_M = math::ToSymmetricMatrixFromLowerTriangularColumns(
      new_lin_eq_cnstr.constraint()->A() * new_lin_eq_cnstr.variables() -
      new_lin_eq_cnstr.constraint()->lower_bound());
  EXPECT_EQ(V_minus_M, V - M);
}
}  // namespace

GTEST_TEST(testMathematicalProgram, AddPositiveSemidefiniteConstraint) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<4>("X");

  auto psd_cnstr = prog.AddPositiveSemidefiniteConstraint(X);
  EXPECT_EQ(prog.positive_semidefinite_constraints().size(), 1);
  const auto& new_psd_cnstr = prog.positive_semidefinite_constraints().back();
  EXPECT_EQ(psd_cnstr.get(), new_psd_cnstr.constraint().get());
  Eigen::Map<Eigen::Matrix<Variable, 16, 1>> X_flat(&X(0, 0));
  EXPECT_TRUE(CheckStructuralEquality(X_flat, new_psd_cnstr.variables()));

  // Adds X is psd.
  CheckAddedSymbolicPositiveSemidefiniteConstraint(&prog,
                                                   Matrix4d::Identity() * X);

  // Adds 2 * X + Identity() is psd.
  CheckAddedSymbolicPositiveSemidefiniteConstraint(
      &prog, 2.0 * X + Matrix4d::Identity());

  // Adds a linear matrix expression Aᵀ * X + X * A is psd.
  Matrix4d A{};
  // clang-format off
  A << 1, 2, 3, 4,
       0, 1, 2, 3,
       0, 0, 2, 3,
       0, 0, 0, 1;
  // clang-format on
  CheckAddedSymbolicPositiveSemidefiniteConstraint(&prog,
                                                   A.transpose() * X + X * A);

  // Adds [X.topLeftCorner<2, 2>()  0                        ] is psd
  //      [ 0                     X.bottomRightCorner<2, 2>()]
  Eigen::Matrix<symbolic::Expression, 4, 4> Y{};
  // clang-format off
  Y << Matrix2d::Identity() * X.topLeftCorner<2, 2>(), Matrix2d::Zero(),
       Matrix2d::Zero(), Matrix2d::Identity() * X.bottomRightCorner<2, 2>();
  // clang-format on
  CheckAddedSymbolicPositiveSemidefiniteConstraint(&prog, Y);
}

void CheckAddedQuadraticCost(MathematicalProgram* prog,
                             const Eigen::MatrixXd& Q, const Eigen::VectorXd& b,
                             const VectorXDecisionVariable& x) {
  int num_quadratic_cost = prog->quadratic_costs().size();
  auto cnstr = prog->AddQuadraticCost(Q, b, x);

  EXPECT_EQ(++num_quadratic_cost, prog->quadratic_costs().size());
  // Check if the newly added quadratic constraint, and the returned
  // quadratic constraint, both match 0.5 * x' * Q * x + b' * x
  EXPECT_EQ(cnstr, prog->quadratic_costs().back().constraint());
  EXPECT_EQ(cnstr->Q(), Q);
  EXPECT_EQ(cnstr->b(), b);
}

GTEST_TEST(testMathematicalProgram, AddQuadraticCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  CheckAddedQuadraticCost(&prog, Matrix3d::Identity(), Vector3d::Zero(), x);

  CheckAddedQuadraticCost(&prog, Matrix3d::Identity(), Vector3d(1, 2, 3), x);
}

void CheckAddedSymbolicQuadraticCostUserFun(const MathematicalProgram& prog,
                                            const Expression& e,
                                            double constant,
                                            const Binding<Constraint>& binding,
                                            int num_quadratic_cost) {
  EXPECT_EQ(num_quadratic_cost, prog.quadratic_costs().size());
  EXPECT_EQ(binding.constraint(), prog.quadratic_costs().back().constraint());
  EXPECT_EQ(binding.variables(), prog.quadratic_costs().back().variables());

  auto cnstr = prog.quadratic_costs().back().constraint();
  // Check the added cost is 0.5 * x' * Q * x + b' * x
  const auto& x_bound = binding.variables();
  const Expression e_added =
      0.5 * x_bound.dot(cnstr->Q() * x_bound) + cnstr->b().dot(x_bound);
  EXPECT_PRED2(ExprEqual, e_added.Expand() + constant, e.Expand());
}

void CheckAddedSymbolicQuadraticCost(MathematicalProgram* prog,
                                     const Expression& e, double constant) {
  int num_quadratic_cost = prog->quadratic_costs().size();
  auto binding1 = prog->AddQuadraticCost(e);
  CheckAddedSymbolicQuadraticCostUserFun(*prog, e, constant, binding1,
                                         ++num_quadratic_cost);
  auto binding2 = prog->AddCost(e);
  CheckAddedSymbolicQuadraticCostUserFun(*prog, e, constant, binding2,
                                         ++num_quadratic_cost);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicQuadraticCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  // Identity diagonal term.
  Expression e1 = x.transpose() * x;
  CheckAddedSymbolicQuadraticCost(&prog, e1, 0);

  // Identity diagonal term.
  Expression e2 = x.transpose() * x + 1;
  CheckAddedSymbolicQuadraticCost(&prog, e2, 1);

  // Identity diagonal term.
  Expression e3 = x(0) * x(0) + x(1) * x(1) + 2;
  CheckAddedSymbolicQuadraticCost(&prog, e3, 2);

  // Non-identity diagonal term.
  Expression e4 = x(0) * x(0) + 2 * x(1) * x(1) + 3 * x(2) * x(2) + 3;
  CheckAddedSymbolicQuadraticCost(&prog, e4, 3);

  // Cross terms.
  Expression e5 = x(0) * x(0) + 2 * x(1) * x(1) + 4 * x(0) * x(1) + 2;
  CheckAddedSymbolicQuadraticCost(&prog, e5, 2);

  // Linear terms.
  Expression e6 = x(0) * x(0) + 2 * x(1) * x(1) + 4 * x(0);
  CheckAddedSymbolicQuadraticCost(&prog, e6, 0);

  // Cross terms and linear terms.
  Expression e7 = (x(0) + 2 * x(1) + 3) * (x(0) + x(1) + 4) + 3 * x(0) * x(0) +
                  6 * pow(x(1) + 1, 2);
  CheckAddedSymbolicQuadraticCost(&prog, e7, 18);

  // Cubic polynomial case.
  Expression e8 = pow(x(0), 3) + 1;
  EXPECT_THROW(prog.AddQuadraticCost(e8), std::runtime_error);
}

GTEST_TEST(testMathematicalProgram, TestL2NormCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  // |Ax - b|^2 = (x-xd)'Q(x-xd) => Q = A'*A and b = A*xd.
  Eigen::Matrix2d A;
  A << 1, 2, 3, 4;
  Eigen::Matrix2d Q = A.transpose() * A;
  Eigen::Vector2d x_desired;
  x_desired << 5, 6;
  Eigen::Vector2d b = A * x_desired;

  std::shared_ptr<QuadraticConstraint> obj1 =
      prog.AddQuadraticErrorCost(Q, x_desired, x);
  std::shared_ptr<QuadraticConstraint> obj2 = prog.AddL2NormCost(A, b, x);

  // Test the objective at a 6 arbitrary values (to guarantee correctness
  // of the six-parameter quadratic form.
  Eigen::Vector2d x0;
  Eigen::VectorXd y1, y2;
  x0 << 7, 8;

  for (int i = 0; i < 6; i++) {
    obj1->Eval(x0, y1);
    obj2->Eval(x0, y2);

    EXPECT_TRUE(CompareMatrices(y1, y2));
    EXPECT_TRUE(CompareMatrices(
        y2, (A * x0 - b).transpose() * (A * x0 - b) - b.transpose() * b));
    // Note: Currently have to subtract out the constant term (b'*b) due to
    // issue #3500.

    x0 += Eigen::Vector2d::Constant(2);
  }
}

GTEST_TEST(testMathematicalProgram, testAddCostThrowError) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  // Add a non-polynomial cost.
  EXPECT_THROW(prog.AddCost(sin(x(0))), std::runtime_error);

  // Add a third order polynomial cost.
  EXPECT_THROW(prog.AddCost(x(0) * x(0) * x(1)), std::runtime_error);
  EXPECT_THROW(prog.AddCost(pow(x(0), 3)), std::runtime_error);

  // Add a cost containing variable not included in the mathematical program.
  symbolic::Variable y("y");
  EXPECT_THROW(prog.AddCost(x(0) + y), std::runtime_error);
  EXPECT_THROW(prog.AddCost(x(0) * x(0) + y), std::runtime_error);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
