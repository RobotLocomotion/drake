#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

using Eigen::Dynamic;
using Eigen::Ref;
using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::solvers::detail::VecIn;
using drake::solvers::detail::VecOut;
using drake::symbolic::Expression;
using drake::symbolic::Variable;

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

/* A generic cost derived from Constraint class. This is meant for testing
 * adding a cost to optimization program, and the cost is in the form of a
 * derived class of Constraint.
 */
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

/* A generic cost. This class is meant for testing adding a cost to the
 * optimization program, by calling `MathematicalProgram::MakeCost` to
 * convert this class to a ConstraintImpl object.
 *
 */
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

void CheckAddedSymbolicLinearCost(MathematicalProgram* prog,
                                  const Expression& e) {
  int num_linear_costs = prog->linear_costs().size();
  const auto& binding = prog->AddLinearCost(e);
  EXPECT_EQ(prog->linear_costs().size(), num_linear_costs + 1);
  EXPECT_EQ(prog->linear_costs().back().constraint(), binding.constraint());
  EXPECT_EQ(binding.constraint()->num_constraints(), 1);
  const Expression cx{(binding.constraint()->A() * binding.variables())(0)};
  double constant_term{0};
  if (is_addition(e)) {
    constant_term = get_constant_in_addition(e);
  } else if (is_constant(e)) {
    constant_term = get_constant_value(e);
  }
  EXPECT_TRUE((e - cx).EqualTo(constant_term));
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
  // Add Linear (constant) cost 3
  CheckAddedSymbolicLinearCost(&prog, 3);
  // Add Linear cost -x(0)
  CheckAddedSymbolicLinearCost(&prog, -x(0));
  // Add Linear cost -(x(1) + 3 * x(0))
  CheckAddedSymbolicLinearCost(&prog, -(x(1) + 3 * x(0)));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic1) {
  // Add Linear Constraint: -10 <= 3 - 5*x0 + 10*x2 - 7*y1 <= 10
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
  // Add Linear Constraint: -10 <= x0 <= 10
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
  // Add Linear Constraints
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

  EXPECT_EQ(M_e - M_lb, Ax - lb_in_ctr);
  EXPECT_EQ(M_e - M_ub, Ax - ub_in_ctr);
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

  MatrixX<Expression> v_resize = flat_V;
  v_resize.resize(v.rows(), v.cols());
  EXPECT_EQ(v_resize, v - b);
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

GTEST_TEST(testMathematicalProgram, AddSymbolicLorentzConeConstraint1) {
  // Add Lorentz cone constraint:
  // x is in Lorentz cone
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  Matrix<Expression, 3, 1> e;
  e << 1 * x(0), 1.0 * x(1), 1.0 * x(2);
  CheckParsedSymbolicLorentzConeConstraint(&prog, e);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicLorentzConeConstraint2) {
  // Add Lorentz cone constraint:
  // x + [1, 2, 0] is in Lorentz cone.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  Matrix<Expression, 3, 1> e;
  e << x(0) + 1, x(1) + 2, +x(2);
  CheckParsedSymbolicLorentzConeConstraint(&prog, e);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicLorentzConeConstraint3) {
  // Add Lorentz cone constraint:
  // [2 * x(0) + 3 * x(2)]
  // [  - x(0) + 2 * x(2)]    is in Lorentz cone
  // [               x(2)]
  // [  -x(1)            ]
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  Matrix<Expression, 4, 1> e;
  // clang-format on
  e << 2 * x(0) + 3 * x(2), -x(0) + 2 * x(2), +x(2), -x(1);
  // clang-format off;
  CheckParsedSymbolicLorentzConeConstraint(&prog, e);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicLorentzConeConstraint4) {
  // Add Lorentz cone constraint:
  // [ 2 * x(0) + 3 * x(1) +            5]
  // [ 4 * x(0)            + 4 * x(2) - 7]
  // [                                 10]
  // [                       2 * x(2)    ]
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  Matrix<Expression, 4, 1> e;
  // clang-format off
  e << 2 * x(0) + 3 * x(1) + 5,
       4 * x(0) + 4 * x(2) - 7,
       10,
       2 * x(2);

  // clang-format on
  CheckParsedSymbolicLorentzConeConstraint(&prog, e);
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
  EXPECT_TRUE(X_flat == new_psd_cnstr.variables());

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
  CheckAddedSymbolicPositiveSemidefiniteConstraint(&prog, Y);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
