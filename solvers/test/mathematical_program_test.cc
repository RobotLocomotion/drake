#include "drake/solvers/mathematical_program.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/test/generic_trivial_constraints.h"
#include "drake/solvers/test/generic_trivial_costs.h"
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

using drake::Vector1d;
using drake::solvers::detail::VecIn;
using drake::solvers::detail::VecOut;
using drake::symbolic::Expression;
using drake::symbolic::Formula;
using drake::symbolic::Variable;
using drake::symbolic::test::ExprEqual;

using std::all_of;
using std::cref;
using std::enable_if;
using std::endl;
using std::is_permutation;
using std::is_same;
using std::make_shared;
using std::map;
using std::move;
using std::numeric_limits;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::set;
using std::shared_ptr;
using std::static_pointer_cast;
using std::string;
using std::unique_ptr;
using std::vector;

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
  void eval(VecIn<ScalarType> const&, VecOut<ScalarType>*) const {}
};

struct Copyable {
  Copyable() = default;
  Copyable(Copyable&&) = delete;
  Copyable(Copyable const&) = default;
  static size_t numInputs() { return 1; }
  static size_t numOutputs() { return 1; }
  template <typename ScalarType>
  void eval(VecIn<ScalarType> const&, VecOut<ScalarType>*) const {}
};

struct Unique {
  Unique() = default;
  Unique(Unique&&) = delete;
  Unique(Unique const&) = delete;
  static size_t numInputs() { return 1; }
  static size_t numOutputs() { return 1; }
  template <typename ScalarType>
  void eval(VecIn<ScalarType> const&, VecOut<ScalarType>*) const {}
};

// Check the index, type and name etc of the newly added variables.
// This function only works if the only variables contained in @p prog are @p
// var.
template <typename ExpectedType, typename T>
void CheckAddedVariable(const MathematicalProgram& prog, const T& var, int rows,
                        int cols, const string& var_name, bool is_symmetric,
                        MathematicalProgram::VarType type_expected) {
  static_assert(is_same<T, ExpectedType>::value, "Type not match");
  EXPECT_EQ(var.rows(), rows);
  EXPECT_EQ(var.cols(), cols);
  // Checks the name of the newly added variables.
  ostringstream msg_buff;
  msg_buff << var << endl;
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
  for (int i = 0; i < var.rows(); ++i) {
    for (int j = 0; j < var.cols(); ++j) {
      EXPECT_EQ(var(i, j).get_type(), type_expected);
    }
  }
}

template <typename Derived>
void CheckAddedIndeterminates(const MathematicalProgram& prog,
                              const Eigen::MatrixBase<Derived>& indeterminates,
                              const string& indeterminates_name) {
  // Checks the name of the newly added indeterminates.
  ostringstream msg_buff;
  msg_buff << indeterminates << endl;
  EXPECT_EQ(msg_buff.str(), indeterminates_name);
  // Checks num_indeterminates() function.
  const int num_new_indeterminates = indeterminates.size();
  EXPECT_EQ(prog.num_indeterminates(), num_new_indeterminates);
  // Checks the indices of the newly added indeterminates.
  for (int i = 0; i < indeterminates.rows(); ++i) {
    for (int j = 0; j < indeterminates.cols(); ++j) {
      EXPECT_EQ(prog.FindIndeterminateIndex(indeterminates(i, j)),
                j * indeterminates.rows() + i);
    }
  }

  // Checks if the indeterminate is of type
  // MathematicalProgram::VarType::CONTINUOUS variable (by default). This test
  // should always be true (by defaults), but keep it to make sure everything
  // works as it is supposed to be.
  for (int i = 0; i < indeterminates.rows(); ++i) {
    for (int j = 0; j < indeterminates.cols(); ++j) {
      EXPECT_EQ(indeterminates(i, j).get_type(),
                MathematicalProgram::VarType::CONTINUOUS);
    }
  }
}

GTEST_TEST(testMathematicalProgram, testConstructor) {
  MathematicalProgram prog;
  EXPECT_EQ(prog.initial_guess().rows(), 0);
  EXPECT_EQ(prog.num_vars(), 0);
  EXPECT_EQ(prog.GetSolution(prog.decision_variables()).rows(), 0);
}

GTEST_TEST(testAddVariable, testAddContinuousVariables1) {
  // Adds a dynamic-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables(2, 3, "X");
  CheckAddedVariable<MatrixXDecisionVariable>(
      prog, X, 2, 3, "X(0,0) X(0,1) X(0,2)\nX(1,0) X(1,1) X(1,2)\n", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddContinuousVariable2) {
  // Adds a static-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables<2, 3>("X");
  CheckAddedVariable<MatrixDecisionVariable<2, 3>>(
      prog, X, 2, 3, "X(0,0) X(0,1) X(0,2)\nX(1,0) X(1,1) X(1,2)\n", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddContinuousVariable3) {
  // Adds a dynamic-sized vector of continuous variables.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(4, "x");
  CheckAddedVariable<VectorXDecisionVariable>(
      prog, x, 4, 1, "x(0)\nx(1)\nx(2)\nx(3)\n", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddContinuousVariable4) {
  // Adds a static-sized vector of continuous variables.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>("y");
  CheckAddedVariable<VectorDecisionVariable<4>>(
      prog, x, 4, 1, "y(0)\ny(1)\ny(2)\ny(3)\n", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddContinuousVariable5) {
  // Adds a static-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables<2, 3>(2, 3, "Y");
  CheckAddedVariable<MatrixDecisionVariable<2, 3>>(
      prog, X, 2, 3, "Y(0,0) Y(0,1) Y(0,2)\nY(1,0) Y(1,1) Y(1,2)\n", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddContinuousVariables6) {
  // Adds a dynamic-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X =
      prog.NewContinuousVariables<Eigen::Dynamic, Eigen::Dynamic>(2, 3, "Y");
  CheckAddedVariable<MatrixXDecisionVariable>(
      prog, X, 2, 3, "Y(0,0) Y(0,1) Y(0,2)\nY(1,0) Y(1,1) Y(1,2)\n", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddContinuousVariables7) {
  // Adds a dynamic-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables<2, Eigen::Dynamic>(2, 3, "Y");
  CheckAddedVariable<MatrixDecisionVariable<2, Eigen::Dynamic>>(
      prog, X, 2, 3, "Y(0,0) Y(0,1) Y(0,2)\nY(1,0) Y(1,1) Y(1,2)\n", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddContinuousVariables8) {
  // Adds a dynamic-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables<Eigen::Dynamic, 3>(2, 3, "Y");
  CheckAddedVariable<MatrixDecisionVariable<Eigen::Dynamic, 3>>(
      prog, X, 2, 3, "Y(0,0) Y(0,1) Y(0,2)\nY(1,0) Y(1,1) Y(1,2)\n", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddContinuousVariables9) {
  // Adds continuous variables with default variable name.
  const std::string X_names = "X(0,0) X(0,1) X(0,2)\nX(1,0) X(1,1) X(1,2)\n";
  MathematicalProgram prog1;
  auto X1 = prog1.NewContinuousVariables(2, 3);
  CheckAddedVariable<MatrixXDecisionVariable>(
      prog1, X1, 2, 3, X_names, false,
      MathematicalProgram::VarType::CONTINUOUS);

  MathematicalProgram prog2;
  auto X2 = prog2.NewContinuousVariables<Eigen::Dynamic, 3>(2, 3);
  CheckAddedVariable<MatrixDecisionVariable<Eigen::Dynamic, 3>>(
      prog2, X2, 2, 3, X_names, false,
      MathematicalProgram::VarType::CONTINUOUS);

  MathematicalProgram prog3;
  auto X3 = prog3.NewContinuousVariables<2, 3>(2, 3);
  CheckAddedVariable<MatrixDecisionVariable<2, 3>>(
      prog3, X3, 2, 3, X_names, false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddSymmetricVariable1) {
  // Adds a static-sized symmetric matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>("X");
  CheckAddedVariable<MatrixDecisionVariable<3, 3>>(
      prog, X, 3, 3,
      "X(0,0) X(1,0) X(2,0)\nX(1,0) X(1,1) X(2,1)\nX(2,0) X(2,1) X(2,2)\n",
      true, MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddSymmetricVariable2) {
  // Adds a dynamic-sized symmetric matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables(3, "X");
  CheckAddedVariable<MatrixXDecisionVariable>(
      prog, X, 3, 3,
      "X(0,0) X(1,0) X(2,0)\nX(1,0) X(1,1) X(2,1)\nX(2,0) X(2,1) X(2,2)\n",
      true, MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddBinaryVariable1) {
  // Adds a dynamic-sized matrix of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables(2, 3, "B");
  CheckAddedVariable<MatrixXDecisionVariable>(
      prog, X, 2, 3, "B(0,0) B(0,1) B(0,2)\nB(1,0) B(1,1) B(1,2)\n", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(testAddVariable, testAddBinaryVariable2) {
  // Adds a dynamic-sized matrix of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables<Eigen::Dynamic, Eigen::Dynamic>(2, 3, "B");
  CheckAddedVariable<MatrixXDecisionVariable>(
      prog, X, 2, 3, "B(0,0) B(0,1) B(0,2)\nB(1,0) B(1,1) B(1,2)\n", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(testAddVariable, testAddBinaryVariable3) {
  // Adds dynamic-sized vector of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables(4, "B");
  CheckAddedVariable<VectorXDecisionVariable>(
      prog, X, 4, 1, "B(0)\nB(1)\nB(2)\nB(3)\n", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(testAddVariable, testAddBinaryVariable4) {
  // Adds static-sized vector of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables<4>("B");
  CheckAddedVariable<VectorDecisionVariable<4>>(
      prog, X, 4, 1, "B(0)\nB(1)\nB(2)\nB(3)\n", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(testAddVariable, testAddBinaryVariable5) {
  // Adds a static-sized matrix of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables<2, 3>("B");
  CheckAddedVariable<MatrixDecisionVariable<2, 3>>(
      prog, X, 2, 3, "B(0,0) B(0,1) B(0,2)\nB(1,0) B(1,1) B(1,2)\n", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(testAddVariable, testAddBinaryVariable6) {
  // Adds a static-sized matrix of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables<2, 3>(2, 3, "B");
  CheckAddedVariable<MatrixDecisionVariable<2, 3>>(
      prog, X, 2, 3, "B(0,0) B(0,1) B(0,2)\nB(1,0) B(1,1) B(1,2)\n", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(testAddVariable, testAddBinaryVariable7) {
  // Adds a dynamic-sized matrix of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables<2, Eigen::Dynamic>(2, 3, "B");
  CheckAddedVariable<MatrixDecisionVariable<2, Eigen::Dynamic>>(
      prog, X, 2, 3, "B(0,0) B(0,1) B(0,2)\nB(1,0) B(1,1) B(1,2)\n", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(testAddVariable, testAddBinaryVariable8) {
  // Adds a dynamic-sized matrix of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables<Eigen::Dynamic, 3>(2, 3, "B");
  CheckAddedVariable<MatrixDecisionVariable<Eigen::Dynamic, 3>>(
      prog, X, 2, 3, "B(0,0) B(0,1) B(0,2)\nB(1,0) B(1,1) B(1,2)\n", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(testAddDecisionVariables, AddDecisionVariables1) {
  // Call AddVariable on an empty program.
  MathematicalProgram prog;
  const Variable x0("x0", Variable::Type::CONTINUOUS);
  const Variable x1("x1", Variable::Type::CONTINUOUS);
  const Variable x2("x2", Variable::Type::BINARY);
  prog.AddDecisionVariables(VectorDecisionVariable<3>(x0, x1, x2));
  EXPECT_EQ(prog.num_vars(), 3);
  EXPECT_EQ(prog.FindDecisionVariableIndex(x0), 0);
  EXPECT_EQ(prog.FindDecisionVariableIndex(x1), 1);
  EXPECT_EQ(prog.FindDecisionVariableIndex(x2), 2);
  EXPECT_EQ(prog.initial_guess().rows(), 3);
  EXPECT_EQ(prog.decision_variables().rows(), 3);
  const VectorDecisionVariable<3> vars_expected(x0, x1, x2);
  SolverResult solver_result(SolverId("dummy"));
  solver_result.set_decision_variable_values(Vector3<double>(1, 2, 3));
  prog.SetSolverResult(solver_result);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(prog.GetSolution(vars_expected(i)), i + 1);
    EXPECT_TRUE(prog.decision_variables()(i).equal_to(vars_expected(i)));
  }
}

GTEST_TEST(testAddDecisionVariables, AddVariable2) {
  // Call AddDecisionVariables on a program that has some existing variables.
  MathematicalProgram prog;
  auto y = prog.NewContinuousVariables<3>("y");
  const Variable x0("x0", Variable::Type::CONTINUOUS);
  const Variable x1("x1", Variable::Type::CONTINUOUS);
  const Variable x2("x2", Variable::Type::BINARY);
  prog.AddDecisionVariables(VectorDecisionVariable<3>(x0, x1, x2));
  EXPECT_EQ(prog.num_vars(), 6);
  EXPECT_EQ(prog.FindDecisionVariableIndex(x0), 3);
  EXPECT_EQ(prog.FindDecisionVariableIndex(x1), 4);
  EXPECT_EQ(prog.FindDecisionVariableIndex(x2), 5);
  EXPECT_EQ(prog.initial_guess().rows(), 6);
  SolverResult solver_result(SolverId("dummy"));
  solver_result.set_decision_variable_values(Vector6<double>::Zero());
  prog.SetSolverResult(solver_result);
  VectorDecisionVariable<6> vars_expected;
  vars_expected << y, x0, x1, x2;
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(prog.GetSolution(vars_expected(i)), 0);
    EXPECT_TRUE(prog.decision_variables()(i).equal_to(vars_expected(i)));
  }
}

GTEST_TEST(testAddDecisionVariables, AddVariable3) {
  // Test the error inputs.
  MathematicalProgram prog;
  auto y = prog.NewContinuousVariables<3>("y");
  const Variable x0("x0", Variable::Type::CONTINUOUS);
  const Variable x1("x1", Variable::Type::CONTINUOUS);
  // Call AddDecisionVariables on a program that has some existing variables,
  // and the
  // new variables intersects with the existing variables.
  EXPECT_THROW(
      prog.AddDecisionVariables(VectorDecisionVariable<3>(x0, x1, y(0))),
      std::runtime_error);
  // The newly added variables have duplicated entries.
  EXPECT_THROW(prog.AddDecisionVariables(VectorDecisionVariable<3>(x0, x1, x0)),
               std::runtime_error);
  // The newly added variables contain a dummy variable.
  Variable dummy;
  EXPECT_TRUE(dummy.is_dummy());
  EXPECT_THROW(
      prog.AddDecisionVariables(VectorDecisionVariable<3>(x0, x1, dummy)),
      std::runtime_error);
  auto z = prog.NewIndeterminates<2>("z");
  // Call AddDecisionVariables on a program that has some indeterminates, and
  // the new
  // variables intersects with the indeterminates.
  EXPECT_THROW(prog.AddDecisionVariables(VectorDecisionVariable<2>(x0, z(0))),
               std::runtime_error);
}

GTEST_TEST(testAddIndeterminates, testAddIndeterminates1) {
  // Adds a dynamic-sized matrix of Indeterminates.
  MathematicalProgram prog;
  auto X = prog.NewIndeterminates(2, 3, "X");
  static_assert(is_same<decltype(X), MatrixXIndeterminate>::value,
                "should be a dynamic sized matrix");
  EXPECT_EQ(X.rows(), 2);
  EXPECT_EQ(X.cols(), 3);
  CheckAddedIndeterminates(prog, X,
                           "X(0,0) X(0,1) X(0,2)\nX(1,0) X(1,1) X(1,2)\n");
}

GTEST_TEST(testAddIndeterminates, testAddIndeterminates2) {
  // Adds a static-sized matrix of Indeterminates.
  MathematicalProgram prog;
  auto X = prog.NewIndeterminates<2, 3>("X");
  static_assert(is_same<decltype(X), MatrixIndeterminate<2, 3>>::value,
                "should be a static sized matrix");
  CheckAddedIndeterminates(prog, X,
                           "X(0,0) X(0,1) X(0,2)\nX(1,0) X(1,1) X(1,2)\n");
}

GTEST_TEST(testAddIndeterminates, testAddIndeterminates3) {
  // Adds a dynamic-sized vector of Indeterminates.
  MathematicalProgram prog;
  auto x = prog.NewIndeterminates(4, "x");
  static_assert(is_same<decltype(x), VectorXIndeterminate>::value,
                "Should be a VectorXDecisionVariable object.");
  EXPECT_EQ(x.rows(), 4);
  CheckAddedIndeterminates(prog, x, "x(0)\nx(1)\nx(2)\nx(3)\n");
}

GTEST_TEST(testAddIndeterminates, testAddIndeterminates4) {
  // Adds a static-sized vector of Indeterminate variables.
  MathematicalProgram prog;
  auto x = prog.NewIndeterminates<4>("x");
  static_assert(is_same<decltype(x), VectorIndeterminate<4>>::value,
                "Should be a VectorXDecisionVariable object.");
  CheckAddedIndeterminates(prog, x, "x(0)\nx(1)\nx(2)\nx(3)\n");
}

template <typename Derived1, typename Derived2>
typename enable_if<is_same<typename Derived1::Scalar, Variable>::value &&
                   is_same<typename Derived2::Scalar, double>::value>::type
CheckGetSolution(const MathematicalProgram& prog,
                 const Eigen::MatrixBase<Derived1>& vars,
                 const Eigen::MatrixBase<Derived2>& val_expected) {
  auto val = prog.GetSolution(vars);
  static_assert(
      is_same<decltype(val), Eigen::Matrix<double, Derived1::RowsAtCompileTime,
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

GTEST_TEST(testAddIndeterminates, AddIndeterminates1) {
  // Call AddIndeterminates on an empty program.
  MathematicalProgram prog;
  const Variable x0("x0", Variable::Type::CONTINUOUS);
  const Variable x1("x1", Variable::Type::CONTINUOUS);
  const Variable x2("x2", Variable::Type::CONTINUOUS);
  prog.AddIndeterminates(VectorIndeterminate<3>(x0, x1, x2));
  const VectorIndeterminate<3> indeterminates_expected(x0, x1, x2);
  EXPECT_EQ(prog.indeterminates().rows(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(prog.indeterminates()(i).equal_to(indeterminates_expected(i)));
    EXPECT_EQ(prog.FindIndeterminateIndex(indeterminates_expected(i)), i);
  }
}

GTEST_TEST(testAddIndeterminates, AddIndeterminates2) {
  // Call AddIndeterminates on a program with some indeterminates.
  MathematicalProgram prog;
  auto y = prog.NewIndeterminates<2>("y");
  const Variable x0("x0", Variable::Type::CONTINUOUS);
  const Variable x1("x1", Variable::Type::CONTINUOUS);
  const Variable x2("x2", Variable::Type::CONTINUOUS);
  prog.AddIndeterminates(VectorIndeterminate<3>(x0, x1, x2));
  VectorIndeterminate<5> indeterminates_expected;
  indeterminates_expected << y(0), y(1), x0, x1, x2;
  EXPECT_EQ(prog.indeterminates().rows(), 5);
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(prog.indeterminates()(i).equal_to(indeterminates_expected(i)));
    EXPECT_EQ(prog.FindIndeterminateIndex(indeterminates_expected(i)), i);
  }
}

GTEST_TEST(testAddIndeterminates, AddIndeterminates3) {
  // Call with erroneous inputs.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  auto y = prog.NewIndeterminates<2>("y");
  const Variable x0("x0", Variable::Type::CONTINUOUS);
  const Variable x1("x1", Variable::Type::BINARY);
  // Call AddIndeterminates with a input that intersects with old
  // indeterminates.
  EXPECT_THROW(prog.AddIndeterminates(VectorIndeterminate<2>(x0, y(0))),
               std::runtime_error);
  // Call AddIndeterminates with a input that intersects with old decision
  // variables.
  EXPECT_THROW(prog.AddIndeterminates(VectorIndeterminate<2>(x0, x(0))),
               std::runtime_error);
  // Call AddIndeterminates with a input of type BINARY.
  EXPECT_THROW(prog.AddIndeterminates(VectorIndeterminate<2>(x0, x1)),
               std::runtime_error);
  // Call AddIndeterminates with a dummy variable.
  Variable dummy;
  EXPECT_THROW(prog.AddIndeterminates(VectorIndeterminate<2>(x0, dummy)),
               std::runtime_error);
}
GTEST_TEST(testGetSolution, testGetSolution1) {
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

  Eigen::VectorXd x_val(prog.num_vars());
  auto SetDecisionVariableValue = [&prog, &x_val](
      const Eigen::Ref<const MatrixXDecisionVariable>& variable,
      const Eigen::Ref<const Eigen::MatrixXd>& val) {
    for (int i = 0; i < variable.rows(); ++i) {
      for (int j = 0; j < variable.cols(); ++j) {
        const int variable_index =
            prog.FindDecisionVariableIndex(variable(i, j));
        x_val(variable_index) = val(i, j);
      }
    }
  };
  SetDecisionVariableValue(X1, X1_value);
  SetDecisionVariableValue(X2, X2_value);
  SetDecisionVariableValue(x3, x3_value);
  SetDecisionVariableValue(x4, x4_value);

  SolverResult solver_result(SolverId("dummy"));
  solver_result.set_decision_variable_values(x_val);
  prog.SetSolverResult(solver_result);

  CheckGetSolution(prog, X1, X1_value);
  CheckGetSolution(prog, X2, X2_value);
  CheckGetSolution(prog, x3, x3_value);
  CheckGetSolution(prog, x4, x4_value);

  // Check a variable that is not a decision variable of the mathematical
  // program.
  Variable z1("z1");
  Variable z2("z2");
  EXPECT_THROW(prog.GetSolution(z1), runtime_error);
  EXPECT_THROW(prog.GetSolution(VectorDecisionVariable<2>(z1, z2)),
               runtime_error);
  EXPECT_THROW(prog.GetSolution(VectorDecisionVariable<2>(z1, X1(0, 0))),
               runtime_error);
}

GTEST_TEST(testGetSolution, testGetSolution2) {
  // GetSolution of a symbolic expression/polynomial
  MathematicalProgram prog;
  const auto x = prog.NewIndeterminates<2>();
  const auto a = prog.NewContinuousVariables<2>();

  const Eigen::Vector2d a_val(1, 2);
  SolverResult solver_result(SolverId("dummy"));
  solver_result.set_decision_variable_values(a_val);
  prog.SetSolverResult(solver_result);

  symbolic::Variable b("b");

  EXPECT_EQ(prog.SubstituteSolution(a(0) + a(1)), 3);

  const symbolic::Expression e = a.cast<symbolic::Expression>().dot(x);
  // `e` contains indeterminates
  const auto e1 = prog.SubstituteSolution(e);
  // All decision variables in the expression are also decision
  // variables in the optimization program.
  EXPECT_EQ(e1, a_val.dot(x));
  const symbolic::Polynomial p = symbolic::Polynomial(
      e, symbolic::Variables(x));
  const auto p1 = prog.SubstituteSolution(p);

  // All decision variables in the polynomial are also decision
  // variables in the optimization program.
  EXPECT_EQ(p1, symbolic::Polynomial(a_val.dot(x), symbolic::Variables(x)));

  // Not all decision variables in the expression/polynomial are decision
  // variables in the optimizatin program. b is a decision variable in the
  // expression/polynomial, but not one in the optimization program.
  EXPECT_THROW(prog.SubstituteSolution(e + b), std::runtime_error);
  EXPECT_THROW(prog.SubstituteSolution(
                   p + symbolic::Polynomial(b, symbolic::Variables())),
               std::runtime_error);
}

namespace {

// Overloads to permit `ExpectBadVar` call `AddItem` for both `Cost` and
// `Constraint`.
void AddItem(MathematicalProgram* prog, const Binding<Constraint>& binding) {
  prog->AddConstraint(binding);
}
void AddItem(MathematicalProgram* prog, const Binding<Cost>& binding) {
  prog->AddCost(binding);
}

// Expect that adding a given constraint with bad variables (those that have
// not been added to MathematicalProgram) will throw an exception.
template <typename C, typename... Args>
void ExpectBadVar(MathematicalProgram* prog, int num_var, Args&&... args) {
  using internal::CreateBinding;
  auto c = make_shared<C>(std::forward<Args>(args)...);
  VectorXDecisionVariable x(num_var);
  for (int i = 0; i < num_var; ++i) x(i) = Variable("bad" + std::to_string(i));
  // Use minimal call site (directly on adding Binding<C>).
  // TODO(eric.cousineau): Check if there is a way to parse the error text to
  // ensure that we are capturing the correct error.
  EXPECT_THROW(AddItem(prog, CreateBinding(c, x)), std::runtime_error);
}

}  // namespace

GTEST_TEST(testMathematicalProgram, testBadBindingVariable) {
  // Attempt to add a binding that does not have a valid decision variable.
  MathematicalProgram prog;

  const int num_var = 3;
  Eigen::Matrix3d A;
  A.setIdentity();
  Eigen::Vector3d f, lb, ub;
  f.setConstant(2);
  lb.setConstant(0);
  ub.setConstant(1);
  Eigen::Matrix3d twiceA = 2 * A;
  vector<Eigen::Ref<const MatrixXd>> F{A, twiceA};
  shared_ptr<EvaluatorBase> func = MakeFunctionEvaluator(Movable());

  // Test each constraint type.
  ExpectBadVar<LinearConstraint>(&prog, num_var, A, lb, ub);
  ExpectBadVar<LinearEqualityConstraint>(&prog, num_var, A, lb);
  ExpectBadVar<BoundingBoxConstraint>(&prog, num_var, lb, ub);
  ExpectBadVar<LorentzConeConstraint>(&prog, num_var, A, f);
  ExpectBadVar<RotatedLorentzConeConstraint>(&prog, num_var, A, f);
  ExpectBadVar<PositiveSemidefiniteConstraint>(&prog, num_var * num_var,
                                               num_var);
  ExpectBadVar<LinearMatrixInequalityConstraint>(&prog, F.size() - 1, F);
  ExpectBadVar<LinearComplementarityConstraint>(&prog, num_var, A, f);
  // Use this as a test for nonlinear constraints.
  ExpectBadVar<EvaluatorConstraint<>>(&prog, 1, func, lb.head(1), ub.head(1));

  // Test each cost type.
  ExpectBadVar<LinearCost>(&prog, num_var, f);
  ExpectBadVar<QuadraticCost>(&prog, num_var, A, f);
  ExpectBadVar<EvaluatorCost<>>(&prog, 1, func);
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
  prog.AddCost(cref(unique), x);
  prog.AddCost(make_shared<Unique>(), x);
  prog.AddCost(unique_ptr<Unique>(new Unique), x);
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
  auto constraint1 = prog.AddBoundingBoxConstraint(0, 1, x1).evaluator();
  auto constraint2 =
      prog.AddBoundingBoxConstraint(0, 1, {x1.col(0), x1.col(1)}).evaluator();
  auto constraint3 = prog.AddBoundingBoxConstraint(0, 1, x2).evaluator();
  auto constraint4 = prog.AddBoundingBoxConstraint(0, 1, x3).evaluator();
  auto constraint5 = prog.AddBoundingBoxConstraint(0, 1, x4).evaluator();
  auto constraint6 = prog.AddBoundingBoxConstraint(Eigen::Vector4d::Zero(),
                                                   Eigen::Vector4d::Ones(), x3)
                         .evaluator();

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

// Verifies if the added cost evaluates the same as the original cost.
// This function is supposed to test these costs added as a derived class
// from Constraint.
void VerifyAddedCost1(const MathematicalProgram& prog,
                      const shared_ptr<Cost>& cost,
                      const Eigen::Ref<const Eigen::VectorXd>& x_value,
                      int num_generic_costs_expected) {
  EXPECT_EQ(static_cast<int>(prog.generic_costs().size()),
            num_generic_costs_expected);
  Eigen::VectorXd y, y_expected;
  prog.generic_costs().back().evaluator()->Eval(x_value, &y);
  cost->Eval(x_value, &y_expected);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
}

// Verifies if the added cost evaluates the same as the original cost.
// This function is supposed to test these costs added by converting
// a class to ConstraintImpl through MakeCost.
void VerifyAddedCost2(const MathematicalProgram& prog,
                      const GenericTrivialCost2& cost,
                      const shared_ptr<Cost>& returned_cost,
                      const Eigen::Ref<const Eigen::Vector2d>& x_value,
                      int num_generic_costs_expected) {
  EXPECT_EQ(static_cast<int>(prog.generic_costs().size()),
            num_generic_costs_expected);
  Eigen::VectorXd y(1), y_expected(1), y_returned;
  prog.generic_costs().back().evaluator()->Eval(x_value, &y);
  cost.eval<double>(x_value, &y_expected);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
  returned_cost->Eval(x_value, &y_returned);
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

  shared_ptr<Cost> generic_trivial_cost1 = make_shared<GenericTrivialCost1>();

  // Adds Binding<Constraint>
  prog.AddCost(Binding<Cost>(generic_trivial_cost1,
                             VectorDecisionVariable<3>(x(0), x(1), y(1))));
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
  auto returned_cost3 =
      prog.AddCost(generic_trivial_cost2, VectorDecisionVariable<2>(x(0), y(1)))
          .evaluator();
  ++num_generic_costs;
  VerifyAddedCost2(prog, generic_trivial_cost2, returned_cost3,
                   Eigen::Vector2d(1, 2), num_generic_costs);

  // Add an object that can be converted to a ConstraintImpl object on a
  // VariableRefList object.
  auto returned_cost4 =
      prog.AddCost(generic_trivial_cost2, {x.head<1>(), y.tail<1>()})
          .evaluator();
  ++num_generic_costs;
  VerifyAddedCost2(prog, generic_trivial_cost2, returned_cost4,
                   Eigen::Vector2d(1, 2), num_generic_costs);
}

void CheckAddedSymbolicLinearCostUserFun(const MathematicalProgram& prog,
                                         const Expression& e,
                                         const Binding<Cost>& binding,
                                         int num_linear_costs) {
  EXPECT_EQ(prog.linear_costs().size(), num_linear_costs);
  EXPECT_EQ(prog.linear_costs().back().evaluator(), binding.evaluator());
  EXPECT_TRUE(CheckStructuralEquality(prog.linear_costs().back().variables(),
                                      binding.variables()));
  EXPECT_EQ(binding.evaluator()->num_outputs(), 1);
  auto cnstr = prog.linear_costs().back().evaluator();
  auto vars = prog.linear_costs().back().variables();
  const Expression cx{cnstr->a().dot(vars)};
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
  const auto constraint_ptr = binding.evaluator();
  EXPECT_EQ(constraint_ptr->num_constraints(), 1);
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

  // Check that the constraint in the binding is of BoundingBoxConstraint.
  ASSERT_TRUE(is_dynamic_castable<BoundingBoxConstraint>(binding.evaluator()));
  const shared_ptr<BoundingBoxConstraint> constraint_ptr{
      static_pointer_cast<BoundingBoxConstraint>(binding.evaluator())};
  EXPECT_EQ(constraint_ptr->num_constraints(), 1);

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
  const auto constraint_ptr = binding.evaluator();
  EXPECT_EQ(constraint_ptr->num_constraints(), 5);
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
  EXPECT_EQ(prog.bounding_box_constraints().back().evaluator(),
            binding.evaluator());
  EXPECT_EQ(prog.bounding_box_constraints().back().variables(),
            binding.variables());
  EXPECT_EQ(binding.variables(), VectorDecisionVariable<1>(x(1)));
  EXPECT_TRUE(
      CompareMatrices(binding.evaluator()->lower_bound(), Vector1d(1)));
  EXPECT_TRUE(
      CompareMatrices(binding.evaluator()->upper_bound(), Vector1d(2)));
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
  EXPECT_EQ(prog.bounding_box_constraints().back().evaluator(),
            binding.evaluator());
  EXPECT_EQ(prog.bounding_box_constraints().back().variables(),
            binding.variables());
  EXPECT_EQ(binding.variables(), VectorDecisionVariable<1>(x(1)));
  EXPECT_TRUE(
      CompareMatrices(binding.evaluator()->lower_bound(), Vector1d(-2)));
  EXPECT_TRUE(
      CompareMatrices(binding.evaluator()->upper_bound(), Vector1d(-1)));
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
  EXPECT_EQ(prog.bounding_box_constraints().back().evaluator(),
            binding.evaluator());
  EXPECT_EQ(prog.bounding_box_constraints().back().variables(),
            binding.variables());
  EXPECT_EQ(binding.variables(), VectorDecisionVariable<1>(x(0)));
  EXPECT_TRUE(
      CompareMatrices(binding.evaluator()->lower_bound(), Vector1d(-3)));
  EXPECT_TRUE(
      CompareMatrices(binding.evaluator()->upper_bound(), Vector1d(-1)));
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
      binding.evaluator()->lower_bound(),
      Vector4d(-1.5, 0.25, -0.5, -numeric_limits<double>::infinity())));
  EXPECT_TRUE(CompareMatrices(binding.evaluator()->upper_bound(),
                              Vector4d(-0.5, 0.75, 0.5, 0)));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic8) {
  // Test the failure cases for adding linear constraint.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  // Non-polynomial.
  EXPECT_THROW(prog.AddLinearConstraint(sin(x(0)), 1, 2), runtime_error);

  // Non-linear.
  EXPECT_THROW(prog.AddLinearConstraint(x(0) * x(0), 1, 2), runtime_error);

  // Trivial (and infeasible) case 1 <= 0 <= 2
  EXPECT_THROW(prog.AddLinearConstraint(x(0) - x(0), 1, 2), runtime_error);
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic9) {
  // Test trivial constraint with no variables, such as 1 <= 2 <= 3
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  prog.AddLinearConstraint(Expression(2), 1, 3);
  EXPECT_EQ(prog.linear_constraints().size(), 1);
  auto binding = prog.linear_constraints().back();
  EXPECT_EQ(binding.evaluator()->A().rows(), 1);
  EXPECT_EQ(binding.evaluator()->A().cols(), 0);

  Vector2<Expression> expr;
  expr << 2, x(0);
  prog.AddLinearConstraint(expr, Vector2d(1, 2), Vector2d(3, 4));
  EXPECT_EQ(prog.linear_constraints().size(), 2);
  binding = prog.linear_constraints().back();
  EXPECT_TRUE(
      CompareMatrices(binding.evaluator()->A(), Eigen::Vector2d(0, 1)));
  EXPECT_TRUE(CompareMatrices(binding.evaluator()->lower_bound(),
                              Eigen::Vector2d(-1, 2)));
  EXPECT_TRUE(CompareMatrices(binding.evaluator()->upper_bound(),
                              Eigen::Vector2d(1, 4)));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicFormula1) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  int num_bounding_box_constraint = 0;

  // x(0) <= 3
  vector<Formula> f;
  f.push_back(x(0) <= 3);
  f.push_back(3 >= x(0));
  f.push_back(x(0) + 2 <= 5);
  f.push_back(4 + x(0) >= 1 + 2 * x(0));
  f.push_back(2 * x(0) + 1 <= 4 + x(0));
  f.push_back(3 * x(0) + x(1) <= 6 + x(0) + x(1));
  for (const auto& fi : f) {
    prog.AddLinearConstraint(fi);
    EXPECT_EQ(++num_bounding_box_constraint,
              prog.bounding_box_constraints().size());
    EXPECT_EQ(prog.linear_constraints().size(), 0);
    EXPECT_EQ(prog.linear_equality_constraints().size(), 0);
    auto binding = prog.bounding_box_constraints().back();
    EXPECT_EQ(binding.variables(), VectorDecisionVariable<1>(x(0)));
    EXPECT_TRUE(
        CompareMatrices(binding.evaluator()->upper_bound(), Vector1d(3)));
    EXPECT_TRUE(CompareMatrices(binding.evaluator()->lower_bound(),
                                Vector1d(-numeric_limits<double>::infinity())));
  }
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicFormula2) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  int num_bounding_box_constraint = 0;
  // x0 >= 2
  vector<Formula> f;
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
    EXPECT_TRUE(
        CompareMatrices(binding.evaluator()->lower_bound(), Vector1d(2)));
    EXPECT_TRUE(CompareMatrices(binding.evaluator()->upper_bound(),
                                Vector1d(numeric_limits<double>::infinity())));
  }
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicFormula3) {
  // x(0) + x(2) == 1
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  int num_linear_equality_constraint = 0;

  vector<Formula> f;
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
        CompareMatrices(binding.evaluator()->lower_bound(), Vector1d(1)));

    VectorX<Expression> expr = binding.evaluator()->A() * binding.variables() -
                               binding.evaluator()->lower_bound();
    EXPECT_EQ(expr.size(), 1);
    EXPECT_PRED2(ExprEqual, expr(0), x(0) + x(2) - 1);
  }
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicFormula4) {
  // x(0) + 2 * x(2) <= 1
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  int num_linear_constraint = 0;

  vector<Formula> f;
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
    EXPECT_EQ(++num_linear_constraint, prog.linear_constraints().size());
    EXPECT_EQ(prog.linear_equality_constraints().size(), 0);
    EXPECT_EQ(prog.bounding_box_constraints().size(), 0);
    auto binding = prog.linear_constraints().back();
    EXPECT_TRUE(CompareMatrices(binding.evaluator()->lower_bound(),
                                Vector1d(-numeric_limits<double>::infinity())));
    EXPECT_TRUE(
        CompareMatrices(binding.evaluator()->upper_bound(), Vector1d(1)));

    const VectorX<Expression> expr =
        binding.evaluator()->upper_bound() -
        binding.evaluator()->A() * binding.variables();
    EXPECT_EQ(expr.size(), 1);
    EXPECT_PRED2(ExprEqual, expr(0), 1 - x(0) - 2 * x(2));
  }
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicFormula5) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<1>();
  const double inf = std::numeric_limits<double>::infinity();
  // This test checks all of the following formula is translated into the
  // bounding constraint, x ∈ [-∞, ∞].
  // clang-format off
  const vector<Formula> f{x(0) <= inf,
                      2 * x(0) <= inf,
                     -3 * x(0) <= inf,
                  2 * x(0) + 1 <= inf,
                 -7 * x(0) + 1 <= inf,
                          -inf <= x(0),
                          -inf <= 2 * x(0),
                          -inf <= -7 * x(0) + 9};
  // clang-format on
  for (const auto& f_i : f) {
    const auto binding = prog.AddLinearConstraint(f_i);
    const VectorX<Expression> expr =
        binding.evaluator()->A() * binding.variables();
    EXPECT_EQ(binding.evaluator()->lower_bound()(0), -inf);
    EXPECT_EQ(binding.evaluator()->upper_bound()(0), inf);
    EXPECT_PRED2(ExprEqual, expr(0), x(0));
  }
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicFormula6) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();
  const double inf = std::numeric_limits<double>::infinity();
  // This test checks that all of the following formula is translated into
  // linear constraints.
  // clang-format off
  const vector<Formula> f{x(0) + x(1) <= inf,
                      2 * x(0) + x(1) <= inf,
                 -3 * x(0) - 7 * x(1) <= inf};
  // clang-format on
  for (const auto& f_i : f) {
    const auto binding = prog.AddLinearConstraint(f_i);
    EXPECT_TRUE(is_dynamic_castable<LinearConstraint>(binding.evaluator()));
    const VectorX<Expression> expr =
        binding.evaluator()->A() * binding.variables();
    EXPECT_EQ(binding.evaluator()->lower_bound()(0), -inf);
    EXPECT_EQ(binding.evaluator()->upper_bound()(0), inf);
    EXPECT_PRED2(ExprEqual, expr(0), get_lhs_expression(f_i));
  }
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicFormula7) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();
  const double inf = std::numeric_limits<double>::infinity();
  // This test checks that all of the following formula is translated into
  // linear constraints.
  // clang-format off
  const vector<Formula> f{x(0) + x(1) >= -inf,
                      2 * x(0) + x(1) >= -inf,
                 -3 * x(0) - 7 * x(1) >= -inf};
  // clang-format on
  for (const auto& f_i : f) {
    const auto binding = prog.AddLinearConstraint(f_i);
    EXPECT_TRUE(is_dynamic_castable<LinearConstraint>(binding.evaluator()));
    const VectorX<Expression> expr =
        binding.evaluator()->A() * binding.variables();
    EXPECT_EQ(binding.evaluator()->lower_bound()(0), -inf);
    EXPECT_EQ(binding.evaluator()->upper_bound()(0), inf);
    EXPECT_PRED2(ExprEqual, expr(0), -get_lhs_expression(f_i));
  }
}

GTEST_TEST(testMathematicalProgram,
           AddLinearConstraintSymbolicFormulaException1) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(prog.AddLinearConstraint(x(0) + inf <= x(1)), runtime_error);
  EXPECT_THROW(prog.AddLinearConstraint(x(0) - inf <= x(1)), runtime_error);
  EXPECT_THROW(prog.AddLinearConstraint(x(0) <= x(1) + inf), runtime_error);
  EXPECT_THROW(prog.AddLinearConstraint(x(0) <= x(1) - inf), runtime_error);
}

GTEST_TEST(testMathematicalProgram,
           AddLinearConstraintSymbolicFormulaException2) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();
  // (x₀ + 1)² - x₀² -2x₀ -1 => 0 (by Expand()).
  const Expression zero_after_expansion0 =
      pow(x(0) + 1, 2) - pow(x(0), 2) - 2 * x(0) - 1;
  // (x₁ + 1)² - x₁² -2x₁ -1 => 0 (by Expand()).
  const Expression zero_after_expansion1 =
      pow(x(1) + 1, 2) - pow(x(1), 2) - 2 * x(1) - 1;

  const double inf = std::numeric_limits<double>::infinity();
  // +∞ <= -∞   -->  Exception.
  // +∞ <= +∞   -->  Exception.
  // -∞ <= +∞   -->  Exception.
  // -∞ <= -∞   -->  Exception.
  EXPECT_THROW(prog.AddLinearConstraint(zero_after_expansion0 + inf <=
                                        zero_after_expansion1 + -inf),
               runtime_error);
  EXPECT_THROW(prog.AddLinearConstraint(zero_after_expansion0 + inf <=
                                        zero_after_expansion1 + inf),
               runtime_error);
  EXPECT_THROW(prog.AddLinearConstraint(zero_after_expansion0 - inf <=
                                        zero_after_expansion1 + inf),
               runtime_error);
  EXPECT_THROW(prog.AddLinearConstraint(zero_after_expansion0 - inf <=
                                        zero_after_expansion1 - inf),
               runtime_error);
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicFormulaAnd1) {
  // Add linear constraints
  //
  //   (A*x == b) = |1 2| * |x0| == |5|
  //                |3 4|   |x1|    |6|
  //
  //              = |  x0 + 2*x1 == 5|
  //                |3*x0 + 4*x1 == 6|
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
  const auto binding = prog.AddLinearConstraint(A * x == b);
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.evaluator();
  // Checks that we have LinearEqualityConstraint instead of LinearConstraint.
  EXPECT_TRUE(is_dynamic_castable<LinearEqualityConstraint>(constraint_ptr));
  EXPECT_EQ(constraint_ptr->num_constraints(), b.size());
  const auto Ax = constraint_ptr->A() * var_vec;
  const auto lb_in_ctr = constraint_ptr->lower_bound();
  const auto ub_in_ctr = constraint_ptr->upper_bound();

  set<Expression> constraint_set;
  constraint_set.emplace(x(0) + 2 * x(1) - 5);
  constraint_set.emplace(3 * x(0) + 4 * x(1) - 6);
  EXPECT_EQ(constraint_set.count(Ax(0) - lb_in_ctr(0)), 1);
  EXPECT_EQ(constraint_set.count(Ax(0) - ub_in_ctr(0)), 1);
  EXPECT_EQ(constraint_set.count(Ax(1) - lb_in_ctr(1)), 1);
  EXPECT_EQ(constraint_set.count(Ax(1) - ub_in_ctr(1)), 1);
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolicFormulaAnd2) {
  // Add linear constraints f1 && f2 && f3 where
  //   f1 := (x0 + 2*x1 >= 3)
  //   f2 := (3*x0 + 4*x1 <= 5)
  //   f3 := (7*x0 + 2*x1 == 9).
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2, "x");
  const Expression e11{x(0) + 2 * x(1)};
  const Expression e12{3};
  const Formula f1{e11 >= e12};
  const Expression e21{3 * x(0) + 4 * x(1)};
  const Expression e22{5};
  const Formula f2{e21 <= e22};
  const Expression e31{7 * x(0) + 2 * x(1)};
  const Expression e32{9};
  const Formula f3{e31 == e32};

  const auto binding = prog.AddLinearConstraint(f1 && f2 && f3);
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.evaluator();
  // Checks that we do not have LinearEqualityConstraint.
  EXPECT_FALSE(is_dynamic_castable<LinearEqualityConstraint>(constraint_ptr));
  EXPECT_EQ(constraint_ptr->num_constraints(), 3);
  const auto Ax = constraint_ptr->A() * var_vec;
  const auto lb_in_ctr = constraint_ptr->lower_bound();
  const auto ub_in_ctr = constraint_ptr->upper_bound();

  set<Expression> constraint_set;
  constraint_set.emplace(e11 - e12);
  constraint_set.emplace(e21 - e22);
  constraint_set.emplace(e31 - e32);
  for (int i = 0; i < 3; ++i) {
    if (!std::isinf(lb_in_ctr(i))) {
      // Either `Ax - lb` or `-(Ax - lb)` should be in the constraint set.
      EXPECT_EQ(constraint_set.count(Ax(i) - lb_in_ctr(i)) +
                    constraint_set.count(-(Ax(i) - lb_in_ctr(i))),
                1);
    }
    if (!std::isinf(ub_in_ctr(i))) {
      // Either `Ax - ub` or `-(Ax - ub)` should be in the constraint set.
      EXPECT_EQ(constraint_set.count(Ax(i) - ub_in_ctr(i)) +
                    constraint_set.count(-(Ax(i) - ub_in_ctr(i))),
                1);
    }
  }
}

GTEST_TEST(testMathematicalProgram,
           AddLinearConstraintSymbolicFormulaAndException) {
  // Add linear constraints:
  //   (x0 + 2*x1 > 3)
  //   (3*x0 + 4*x1 < 5)
  //   (7*x0 + 2*x1 == 9)
  //
  // It includes relational formulas with strict inequalities (> and <). It will
  // throw std::runtime_error.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2, "x");
  const Expression e11{x(0) + 2 * x(1)};
  const Expression e12{3};
  const Formula f1{e11 > e12};
  const Expression e21{3 * x(0) + 4 * x(1)};
  const Expression e22{5};
  const Formula f2{e21 < e22};
  const Expression e31{7 * x(0) + 2 * x(1)};
  const Expression e32{9};
  const Formula f3{e31 == e32};
  EXPECT_THROW(prog.AddLinearConstraint(f1 && f2 && f3), runtime_error);
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
  const auto constraint_ptr = binding.evaluator();
  EXPECT_EQ(constraint_ptr->num_constraints(), 5);
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
  const auto constraint_ptr = binding.evaluator();
  EXPECT_EQ(constraint_ptr->num_constraints(), M_e.rows() * M_e.cols());
  const auto Ax = constraint_ptr->A() * var_vec;
  const auto lb_in_ctr = constraint_ptr->lower_bound();
  const auto ub_in_ctr = constraint_ptr->upper_bound();
  int k{0};
  for (int j{0}; j < M_e.cols(); ++j) {
    for (int i{0}; i < M_e.rows(); ++i) {
      EXPECT_PRED2(ExprEqual, M_e(i, j) - M_lb(i, j), Ax(k) - lb_in_ctr(k));
      ++k;
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
  const auto constraint_ptr = binding.evaluator();
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
  EXPECT_THROW(prog.AddLinearConstraint(M_f.array()), runtime_error);
}

namespace {
void CheckAddedLinearEqualityConstraintCommon(
    const Binding<LinearEqualityConstraint>& binding,
    const MathematicalProgram& prog, int num_linear_eq_cnstr) {
  // Checks if the number of linear equality constraints get incremented by 1.
  EXPECT_EQ(prog.linear_equality_constraints().size(), num_linear_eq_cnstr + 1);
  // Checks if the newly added linear equality constraint in prog is the same as
  // that returned from AddLinearEqualityConstraint.
  EXPECT_EQ(prog.linear_equality_constraints().back().evaluator(),
            binding.evaluator());
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

  EXPECT_EQ(binding.evaluator()->num_constraints(), num_constraints_expected);
  // Check if the newly added linear equality constraint matches with the input
  // expression.
  VectorX<Expression> flat_V = binding.evaluator()->A() * binding.variables() -
                               binding.evaluator()->lower_bound();

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
  EXPECT_EQ(binding.evaluator()->num_constraints(), num_constraints_expected);
  // Check if the newly added linear equality constraint matches with the input
  // expression.
  VectorX<Expression> flat_V = binding.evaluator()->A() * binding.variables() -
                               binding.evaluator()->lower_bound();
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

// Tests `AddLinearEqualityConstraint(const symbolic::Formula& f)` method with a
// case where `f` is a linear-equality formula (instead of a conjunction of
// them).
GTEST_TEST(testMathematicalProgram, AddSymbolicLinearEqualityConstraint5) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  // f = (3x₀ + 2x₁ + 3 == 5).
  const Formula f{3 * x(0) + 2 * x(1) + 3 == 5};
  const Binding<LinearEqualityConstraint> binding{
      prog.AddLinearEqualityConstraint(f)};
  EXPECT_EQ(prog.linear_equality_constraints().size(), 1u);

  const Expression expr_in_added_constraint{
      (binding.evaluator()->A() * binding.variables() -
       binding.evaluator()->lower_bound())(0)};
  // expr_in_added_constraint should be:
  //    lhs(f) - rhs(f)
  //  = (3x₀ + 2x₁ + 3) - 5
  //  = 3x₀ + 2x₁ - 2.
  EXPECT_PRED2(ExprEqual, expr_in_added_constraint, 3 * x(0) + 2 * x(1) - 2);
  EXPECT_PRED2(ExprEqual, expr_in_added_constraint,
               get_lhs_expression(f) - get_rhs_expression(f));
}

// Tests `AddLinearEqualityConstraint(const symbolic::Formula& f)` method with a
// case where `f` is a conjunction of linear-equality formulas .
GTEST_TEST(testMathematicalProgram, AddSymbolicLinearEqualityConstraint6) {
  // Test problem: Ax = b where
  //
  // A = |-3.0  0.0  2.0|  x = |x0|  b = | 9.0|
  //     | 0.0  7.0 -3.0|      |x1|      | 3.0|
  //     | 2.0  5.0  0.0|      |x2|      |-5.0|
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  Eigen::Matrix3d A;
  Vector3d b;
  // clang-format off
  A << -3.0, 0.0,  2.0,
        0.0, 7.0, -3.0,
        2.0, 5.0,  0.0;
  b << 9.0,
       3.0,
      -5.0;
  // clang-format on
  const Formula f{A * x == b};
  const Binding<LinearEqualityConstraint> binding{
      prog.AddLinearEqualityConstraint(f)};
  EXPECT_EQ(prog.linear_equality_constraints().size(), 1u);

  // Checks if AddLinearEqualityConstraint added the constraint correctly.
  const Eigen::Matrix<Expression, 3, 1> exprs_in_added_constraint{
      binding.evaluator()->A() * binding.variables() -
      binding.evaluator()->lower_bound()};
  const Eigen::Matrix<Expression, 3, 1> expected_exprs{A * x - b};

  // Since a conjunctive symbolic formula uses `std::set` as an internal
  // representation, we need to check if `exprs_in_added_constraint` is a
  // permutation of `expected_exprs`.
  EXPECT_TRUE(is_permutation(
      exprs_in_added_constraint.data(), exprs_in_added_constraint.data() + 3,
      expected_exprs.data(), expected_exprs.data() + 3, ExprEqual));
}

// Checks if `AddLinearEqualityConstraint(f)` throws std::runtime_error if `f`
// is a non-linear equality formula.
GTEST_TEST(testMathematicalProgram,
           AddSymbolicLinearEqualityConstraintException1) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  // f = (3x₀² + 2x₁ + 3 == 5).
  const Formula f{3 * x(0) * x(0) + 2 * x(1) + 3 == 5};
  EXPECT_THROW(prog.AddLinearEqualityConstraint(f), runtime_error);
}

// Checks if `AddLinearEqualityConstraint(f)` throws std::runtime_error if a
// conjunctive formula `f` includes a relational formula other than equality.
GTEST_TEST(testMathematicalProgram,
           AddSymbolicLinearEqualityConstraintException2) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  // f = (3x₀ + 2x₁ + 3 >= 5 && 7x₀ - 5x₁ == 0).
  const Formula f{3 * x(0) + 2 * x(1) + 3 >= 5 && 7 * x(0) - 5 * x(1) == 0};
  EXPECT_THROW(prog.AddLinearEqualityConstraint(f), runtime_error);
}

// Checks if `AddLinearEqualityConstraint(f)` throws std::runtime_error if `f`
// is neither a linear-equality formula nor a conjunctive formula.
GTEST_TEST(testMathematicalProgram,
           AddSymbolicLinearEqualityConstraintException3) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  // f = (3x₀ + 2x₁ + 3 == 5 || 7x₀ - 5x₁ == 0).
  const Formula f{3 * x(0) + 2 * x(1) + 3 == 5 || 7 * x(0) - 5 * x(1) == 0};
  EXPECT_THROW(prog.AddLinearEqualityConstraint(f), runtime_error);
}

namespace {
bool AreTwoPolynomialsNear(
    const symbolic::Polynomial& p1, const symbolic::Polynomial& p2,
    const double tol = numeric_limits<double>::epsilon()) {
  symbolic::Polynomial diff{p1 - p2};
  const auto& monomial_to_coeff_map = diff.monomial_to_coefficient_map();
  return all_of(monomial_to_coeff_map.begin(), monomial_to_coeff_map.end(),
                [tol](const auto& p) {
                  return std::abs(symbolic::get_constant_value(p.second)) <=
                         tol;
                });
}  // namespace

void CheckParsedSymbolicLorentzConeConstraint(
    MathematicalProgram* prog, const Expression& linear_expr,
    const Expression& quadratic_expr) {
  const auto& binding1 =
      prog->AddLorentzConeConstraint(linear_expr, quadratic_expr);
  const auto& binding2 = prog->lorentz_cone_constraints().back();
  EXPECT_EQ(binding1.evaluator(), binding2.evaluator());
  EXPECT_EQ(binding1.variables(), binding2.variables());
  // Now check if the linear and quadratic constraints are parsed correctly.
  const VectorX<Expression> e_parsed =
      binding1.evaluator()->A() * binding1.variables() +
      binding1.evaluator()->b();
  EXPECT_PRED2(ExprEqual, (e_parsed(0) * e_parsed(0)).Expand(),
               (linear_expr * linear_expr).Expand());
  Expression quadratic_expr_parsed =
      e_parsed.tail(e_parsed.rows() - 1).squaredNorm();
  // Due to the small numerical error, quadratic_expr and quadratic_expr_parsed
  // do not match exactly.So we will compare each term in the two polynomials,
  // and regard them to be equal if the error in the coefficient is sufficiently
  // small.
  const symbolic::Polynomial poly_parsed{quadratic_expr_parsed};
  const symbolic::Polynomial poly{quadratic_expr};
  const double tol{100 * numeric_limits<double>::epsilon()};
  EXPECT_TRUE(AreTwoPolynomialsNear(poly_parsed, poly, tol));
}

void CheckParsedSymbolicLorentzConeConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const Eigen::Matrix<Expression, Eigen::Dynamic, 1>>& e) {
  const auto& binding1 = prog->AddLorentzConeConstraint(e);
  const auto& binding2 = prog->lorentz_cone_constraints().back();

  EXPECT_EQ(binding1.evaluator(), binding2.evaluator());
  EXPECT_EQ(binding1.evaluator()->A() * binding1.variables() +
                binding1.evaluator()->b(),
            e);
  EXPECT_EQ(binding2.evaluator()->A() * binding2.variables() +
                binding2.evaluator()->b(),
            e);

  CheckParsedSymbolicLorentzConeConstraint(prog, e(0),
                                           e.tail(e.rows() - 1).squaredNorm());
}

void CheckParsedSymbolicRotatedLorentzConeConstraint(
    MathematicalProgram* prog, const Eigen::Ref<const VectorX<Expression>>& e) {
  const auto& binding1 = prog->AddRotatedLorentzConeConstraint(e);
  const auto& binding2 = prog->rotated_lorentz_cone_constraints().back();

  EXPECT_EQ(binding1.evaluator(), binding2.evaluator());
  EXPECT_EQ(binding1.evaluator()->A() * binding1.variables() +
                binding1.evaluator()->b(),
            e);
  EXPECT_EQ(binding2.evaluator()->A() * binding2.variables() +
                binding2.evaluator()->b(),
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
               runtime_error);

  // The quadratic expression is actually affine.
  EXPECT_THROW(prog_.AddLorentzConeConstraint(2 * x_(0), 3 * x_(1) + 2),
               runtime_error);
  EXPECT_THROW(prog_.AddLorentzConeConstraint(
                   2 * x_(0),
                   x_(1) * x_(1) - (x_(1) - x_(0)) * (x_(1) + x_(0)) -
                       x_(0) * x_(0) + 2 * x_(1) + 3),
               runtime_error);

  // The Hessian matrix is not positive semidefinite.
  EXPECT_THROW(prog_.AddLorentzConeConstraint(2 * x_(0) + 3,
                                              x_(1) * x_(1) - x_(2) * x_(2)),
               runtime_error);
  EXPECT_THROW(
      prog_.AddLorentzConeConstraint(
          2 * x_(0) + 3, x_(1) * x_(1) + x_(2) * x_(2) + 3 * x_(1) * x_(2)),
      runtime_error);
  EXPECT_THROW(
      prog_.AddLorentzConeConstraint(
          2 * x_(0) + 3, x_(1) * x_(1) + x_(2) * x_(2) + 3 * x_(0) * x_(2)),
      runtime_error);

  // The quadratic expression is not always non-negative.
  EXPECT_THROW(prog_.AddLorentzConeConstraint(
                   2 * x_(0) + 3, x_(1) * x_(1) + x_(2) * x_(2) - 1),
               runtime_error);
  EXPECT_THROW(prog_.AddLorentzConeConstraint(
                   2 * x_(0) + 3, pow(2 * x_(0) + 3 * x_(1) + 2, 2) - 1),
               runtime_error);

  // The quadratic expression is a negative constant.
  EXPECT_THROW(
      prog_.AddLorentzConeConstraint(2 * x_(0) + 3,
                                     pow(x_(0), 2) - pow(x_(1), 2) -
                                         (x_(0) + x_(1)) * (x_(0) - x_(1)) - 1),
      runtime_error);

  // The first expression is not actually linear.
  EXPECT_THROW(prog_.AddLorentzConeConstraint(2 * x_(0) * x_(1), pow(x_(0), 2)),
               runtime_error);
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

GTEST_TEST(testMathematicalProgram, AddSymbolicRotatedLorentzConeConstraint5) {
  // Add rotated Lorentz cone constraint, using quadratic expression.
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<4>("x");
  Expression linear_expression1 = x(0) + 1;
  Expression linear_expression2 = x(1) - x(2);
  Eigen::Matrix2d Q;
  Eigen::Vector2d b;
  const double c{5};
  Q << 1, 0.5, 0.5, 1;
  b << 0, 0.1;
  const Expression quadratic_expression =
      x.head<2>().cast<Expression>().dot(Q * x.head<2>() + b) + c;
  const auto binding = prog.AddRotatedLorentzConeConstraint(
      linear_expression1, linear_expression2, quadratic_expression);
  EXPECT_EQ(binding.evaluator(),
            prog.rotated_lorentz_cone_constraints().back().evaluator());
  const VectorX<Expression> z =
      binding.evaluator()->A() * binding.variables() +
      binding.evaluator()->b();
  const double tol{1E-10};
  EXPECT_TRUE(
      symbolic::test::PolynomialEqual(symbolic::Polynomial(linear_expression1),
                                      symbolic::Polynomial(z(0)), tol));
  EXPECT_TRUE(
      symbolic::test::PolynomialEqual(symbolic::Polynomial(linear_expression2),
                                      symbolic::Polynomial(z(1)), tol));
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      symbolic::Polynomial(quadratic_expression),
      symbolic::Polynomial(z.tail(z.rows() - 2).squaredNorm()), tol));
}

namespace {
template <typename Derived>
typename enable_if<is_same<typename Derived::Scalar, Expression>::value>::type
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
      binding.evaluator().get(),
      prog->positive_semidefinite_constraints().back().evaluator().get());
  // Check if the added linear constraint is correct. M is the newly added
  // variables representing the psd matrix.
  const Eigen::Map<const MatrixX<Variable>> M(&binding.variables()(0), V.rows(),
                                              V.cols());
  // The linear equality constraint is only imposed on the lower triangular
  // part of the psd matrix.
  const auto& new_lin_eq_cnstr = prog->linear_equality_constraints().back();
  auto V_minus_M = math::ToSymmetricMatrixFromLowerTriangularColumns(
      new_lin_eq_cnstr.evaluator()->A() * new_lin_eq_cnstr.variables() -
      new_lin_eq_cnstr.evaluator()->lower_bound());
  EXPECT_EQ(V_minus_M, V - M);
}
}  // namespace

GTEST_TEST(testMathematicalProgram, AddPositiveSemidefiniteConstraint) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<4>("X");

  auto psd_cnstr = prog.AddPositiveSemidefiniteConstraint(X).evaluator();
  EXPECT_EQ(prog.positive_semidefinite_constraints().size(), 1);
  const auto& new_psd_cnstr = prog.positive_semidefinite_constraints().back();
  EXPECT_EQ(psd_cnstr.get(), new_psd_cnstr.evaluator().get());
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

  // Test the MatrixX<Expression> version.
  MatrixX<Expression> M = A.transpose() * X + X * A;
  CheckAddedSymbolicPositiveSemidefiniteConstraint(&prog, M);

  // Adds [X.topLeftCorner<2, 2>()  0                        ] is psd
  //      [ 0                     X.bottomRightCorner<2, 2>()]
  Eigen::Matrix<Expression, 4, 4> Y{};
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
  auto cnstr = prog->AddQuadraticCost(Q, b, x).evaluator();

  EXPECT_EQ(++num_quadratic_cost, prog->quadratic_costs().size());
  // Check if the newly added quadratic constraint, and the returned
  // quadratic constraint, both match 0.5 * x' * Q * x + b' * x
  EXPECT_EQ(cnstr, prog->quadratic_costs().back().evaluator());
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
                                            const Binding<Cost>& binding,
                                            int num_quadratic_cost) {
  EXPECT_EQ(num_quadratic_cost, prog.quadratic_costs().size());
  EXPECT_EQ(binding.evaluator(), prog.quadratic_costs().back().evaluator());
  EXPECT_EQ(binding.variables(), prog.quadratic_costs().back().variables());

  auto cnstr = prog.quadratic_costs().back().evaluator();
  // Check the added cost is 0.5 * x' * Q * x + b' * x
  const auto& x_bound = binding.variables();
  const Expression e_added = 0.5 * x_bound.dot(cnstr->Q() * x_bound) +
                             cnstr->b().dot(x_bound) + cnstr->c();
  EXPECT_PRED2(ExprEqual, e_added.Expand(), e.Expand());
}

void CheckAddedSymbolicQuadraticCost(MathematicalProgram* prog,
                                     const Expression& e) {
  int num_quadratic_cost = prog->quadratic_costs().size();
  auto binding1 = prog->AddQuadraticCost(e);
  CheckAddedSymbolicQuadraticCostUserFun(*prog, e, binding1,
                                         ++num_quadratic_cost);
  auto binding2 = prog->AddCost(e);
  CheckAddedSymbolicQuadraticCostUserFun(*prog, e, binding2,
                                         ++num_quadratic_cost);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicQuadraticCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  // Identity diagonal term.
  Expression e1 = x.transpose() * x;
  CheckAddedSymbolicQuadraticCost(&prog, e1);

  // Identity diagonal term.
  Expression e2 = x.transpose() * x + 1;
  CheckAddedSymbolicQuadraticCost(&prog, e2);

  // Identity diagonal term.
  Expression e3 = x(0) * x(0) + x(1) * x(1) + 2;
  CheckAddedSymbolicQuadraticCost(&prog, e3);

  // Non-identity diagonal term.
  Expression e4 = x(0) * x(0) + 2 * x(1) * x(1) + 3 * x(2) * x(2) + 3;
  CheckAddedSymbolicQuadraticCost(&prog, e4);

  // Cross terms.
  Expression e5 = x(0) * x(0) + 2 * x(1) * x(1) + 4 * x(0) * x(1) + 2;
  CheckAddedSymbolicQuadraticCost(&prog, e5);

  // Linear terms.
  Expression e6 = x(0) * x(0) + 2 * x(1) * x(1) + 4 * x(0);
  CheckAddedSymbolicQuadraticCost(&prog, e6);

  // Cross terms and linear terms.
  Expression e7 = (x(0) + 2 * x(1) + 3) * (x(0) + x(1) + 4) + 3 * x(0) * x(0) +
                  6 * pow(x(1) + 1, 2);
  CheckAddedSymbolicQuadraticCost(&prog, e7);

  // Cubic polynomial case.
  Expression e8 = pow(x(0), 3) + 1;
  EXPECT_THROW(prog.AddQuadraticCost(e8), runtime_error);
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

  auto obj1 = prog.AddQuadraticErrorCost(Q, x_desired, x).evaluator();
  auto obj2 = prog.AddL2NormCost(A, b, x).evaluator();

  // Test the objective at a 6 arbitrary values (to guarantee correctness
  // of the six-parameter quadratic form.
  Eigen::Vector2d x0;
  Eigen::VectorXd y1, y2;
  x0 << 7, 8;

  for (int i = 0; i < 6; i++) {
    obj1->Eval(x0, &y1);
    obj2->Eval(x0, &y2);

    EXPECT_TRUE(CompareMatrices(y1, y2));
    EXPECT_TRUE(CompareMatrices(y2, (A * x0 - b).transpose() * (A * x0 - b)));

    x0 += Eigen::Vector2d::Constant(2);
  }
}

// Helper function for ArePolynomialIsomorphic.
//
// Transforms a monomial into an isomorphic one up to a given map (Variable::Id
// → Variable). Consider an example where monomial is "x³y⁴" and var_id_to_var
// is {x.get_id() ↦ z, y.get_id() ↦ w}. We have transform(x³y⁴, {x.get_id() ↦ z,
// y.get_id() ↦ w}) = z³w⁴.
//
// @pre `var_id_to_var` is 1-1.
// @pre The domain of `var_id_to_var` includes all variables in `monomial`.
// @pre `var_id_to_var` should be chain-free. Formally, for all variable v in
// the image of var_id_to_var, its ID, id(v) should not be in the domain of
// var_id_to_var. For example, {x.get_id() -> y, y.get_id() -> z} is not
// allowed.
symbolic::Monomial transform(const symbolic::Monomial& monomial,
                             const map<Variable::Id, Variable>& var_id_to_var) {
  // var_id_to_var should be chain-free.
  for (const pair<Variable::Id, Variable>& p : var_id_to_var) {
    const Variable& var{p.second};
    DRAKE_DEMAND(var_id_to_var.find(var.get_id()) == var_id_to_var.end());
  }
  map<Variable, int> new_powers;
  for (const pair<Variable, int> p : monomial.get_powers()) {
    const Variable& var_in_monomial{p.first};
    const int exponent{p.second};
    const auto it = var_id_to_var.find(var_in_monomial.get_id());

    // There should be a mapping for the ID in var_id_to_var.
    DRAKE_DEMAND(it != var_id_to_var.end());
    const Variable new_var{it->second};

    // var_id_to_var should be 1-1.
    DRAKE_DEMAND(new_powers.find(new_var) == new_powers.end());
    new_powers.emplace(new_var, exponent);
  }
  return symbolic::Monomial{new_powers};
}

// Helper function for ArePolynomialIsomorphic.
//
// Transforms a Polynomial into an isomorphic one up to a given map
// (Variable::Id → Variable). Consider an example where poly = x³y⁴ + 2x² and
// var_id_to_var is {x.get_id() ↦ z, y.get_id() ↦ w}. We have transform(poly,
// var_id_to_var) = z³w⁴ + 2z².
//
// @pre `var_id_to_var` is 1-1.
// @pre The domain of `var_id_to_var` includes all variables in `m`.
// @pre `var_id_to_var` should be chain-free.
symbolic::Polynomial transform(
    const symbolic::Polynomial& poly,
    const map<Variable::Id, Variable>& var_id_to_var) {
  symbolic::Polynomial::MapType new_map;
  for (const pair<symbolic::Monomial, symbolic::Expression>& p :
       poly.monomial_to_coefficient_map()) {
    new_map.emplace(transform(p.first, var_id_to_var), p.second);
  }
  return symbolic::Polynomial{new_map};
}

// Helper function for CheckAddedPolynomialCost.
//
// Checks if two Polynomial `p1` and `p2` are isomorphic with respect to a
// bijection `var_id_to_var`.
//
// @pre `var_id_to_var` is 1-1.
// @pre The domain of `var_id_to_var` includes all variables in `m`.
// @pre `var_id_to_var` should be chain-free.
bool ArePolynomialIsomorphic(const symbolic::Polynomial& p1,
                             const symbolic::Polynomial& p2,
                             const map<Variable::Id, Variable>& var_id_to_var) {
  return transform(p1, var_id_to_var).EqualTo(p2);
}

void CheckAddedPolynomialCost(MathematicalProgram* prog, const Expression& e) {
  int num_cost = prog->generic_costs().size();
  const auto binding = prog->AddPolynomialCost(e);
  EXPECT_EQ(prog->generic_costs().size(), ++num_cost);
  EXPECT_EQ(binding.evaluator(), prog->generic_costs().back().evaluator());
  // Now reconstruct the symbolic expression from `binding`.
  const auto polynomial = binding.evaluator()->polynomials()(0);

  // var_id_to_var : Variable::Id → Variable. It keeps the relation between a
  // variable in a Polynomial<double> and symbolic::Monomial.
  symbolic::Polynomial poly_expected;
  map<Variable::Id, Variable> var_id_to_var;
  for (const Polynomial<double>::Monomial& m : polynomial.GetMonomials()) {
    map<Variable, int> map_var_to_power;
    for (const Polynomial<double>::Term& term : m.terms) {
      auto it = var_id_to_var.find(term.var);
      if (it == var_id_to_var.end()) {
        Variable var{std::to_string(term.var)};
        var_id_to_var.emplace_hint(it, term.var, var);
        map_var_to_power.emplace(var, term.power);
      } else {
        map_var_to_power.emplace(it->second, term.power);
      }
    }
    poly_expected += symbolic::Monomial(map_var_to_power) * m.coefficient;
  }
  // Checks if the two polynomials, `poly` and `poly_expected` are isomorphic
  // with respect to `var_id_to_var`.
  const symbolic::Polynomial poly{e};
  EXPECT_TRUE(ArePolynomialIsomorphic(poly, poly_expected, var_id_to_var));
}

GTEST_TEST(testMathematicalProgram, testAddPolynomialCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  // Add a cubic cost
  CheckAddedPolynomialCost(&prog, pow(x(0), 3));

  // Add a cubic cost
  CheckAddedPolynomialCost(&prog, pow(x(0), 2) * x(1));

  // Add a 4th order cost
  CheckAddedPolynomialCost(
      &prog, x(0) * x(0) * x(1) * x(1) + pow(x(0), 3) * x(1) + 2 * x(1));
}

GTEST_TEST(testMathematicalProgram, testAddCostThrowError) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  // Add a non-polynomial cost.
  EXPECT_THROW(prog.AddCost(sin(x(0))), runtime_error);

  // Add a cost containing variable not included in the mathematical program.
  Variable y("y");
  EXPECT_THROW(prog.AddCost(x(0) + y), runtime_error);
  EXPECT_THROW(prog.AddCost(x(0) * x(0) + y), runtime_error);
}

GTEST_TEST(testMathematicalProgram, testAddGenericCost) {
  using GenericPtr = shared_ptr<Cost>;
  using Matrix1d = Vector1d;

  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>();

  GenericPtr linear_cost(new LinearCost(Matrix1d(1)));
  prog.AddCost(linear_cost, x);
  EXPECT_EQ(prog.linear_costs().size(), 1);

  GenericPtr quadratic_cost(new QuadraticCost(Matrix1d(1), Vector1d(1)));
  prog.AddCost(quadratic_cost, x);
  EXPECT_EQ(prog.quadratic_costs().size(), 1);
}

GTEST_TEST(testMathematicalProgram, testClone) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  auto y = prog.NewIndeterminates<2>("y");
  auto X = prog.NewSymmetricContinuousVariables<3>("X");

  // Add costs
  shared_ptr<Cost> generic_trivial_cost1 = make_shared<GenericTrivialCost1>();
  prog.AddCost(Binding<Cost>(generic_trivial_cost1,
                             VectorDecisionVariable<3>(x(0), x(1), x(2))));
  GenericTrivialCost2 generic_trivial_cost2;
  prog.AddCost(generic_trivial_cost2, VectorDecisionVariable<2>(x(2), x(1)));
  prog.AddLinearCost(x(0) + 2);
  prog.AddQuadraticCost(x(0) * x(0) + 2 * x(1) * x(1));
  prog.AddLinearCost(x(0) + 2 * x(2));
  prog.AddQuadraticCost(x(1) * x(1) + 1);

  // Add constraints
  shared_ptr<Constraint> generic_trivial_constraint1 =
      make_shared<GenericTrivialConstraint1>();
  prog.AddConstraint(
      Binding<Constraint>(generic_trivial_constraint1,
                          VectorDecisionVariable<3>(x(0), x(1), x(2))));
  prog.AddConstraint(
      Binding<Constraint>(generic_trivial_constraint1,
                          VectorDecisionVariable<3>(x(2), x(1), x(0))));
  prog.AddLinearConstraint(x(0) + x(1) <= 2);
  prog.AddLinearConstraint(x(1) + x(2) <= 1);
  prog.AddLinearEqualityConstraint(x(0) + x(2) == 0);
  prog.AddLinearEqualityConstraint(x(0) + x(1) + 3 * x(2) == 1);
  prog.AddBoundingBoxConstraint(-10, 10, x(0));
  prog.AddBoundingBoxConstraint(-4, 5, x(1));
  prog.AddLorentzConeConstraint(
      Vector3<symbolic::Expression>(+x(0), +x(1), x(2) - 0.5 * x(1)));
  prog.AddLorentzConeConstraint(
      Vector3<symbolic::Expression>(x(0) + x(1), +x(0), x(2) - x(1)));
  prog.AddRotatedLorentzConeConstraint(Vector4<symbolic::Expression>(
      +x(0), +x(1), 0.5 * (x(0) + x(1)), 0.5 * x(2)));
  prog.AddRotatedLorentzConeConstraint(
      Vector4<symbolic::Expression>(x(0) + x(1), x(1) + x(2), +x(0), +x(1)));
  prog.AddPositiveSemidefiniteConstraint(X);
  prog.AddPositiveSemidefiniteConstraint(X - Eigen::Matrix3d::Ones());
  prog.AddLinearMatrixInequalityConstraint(
      {Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Ones(),
       2 * Eigen::Matrix2d::Ones()},
      x.head<2>());
  prog.AddLinearComplementarityConstraint(Eigen::Matrix2d::Identity(),
                                          Eigen::Vector2d::Ones(), x.head<2>());
  prog.AddLinearComplementarityConstraint(2 * Eigen::Matrix2d::Identity(),
                                          Eigen::Vector2d::Ones(), x.tail<2>());

  // Set initial guess
  prog.SetInitialGuessForAllVariables(Eigen::VectorXd::Ones(prog.num_vars()));

  auto new_prog = prog.Clone();

  // Cloned program should have the same variables and indeterminates.
  EXPECT_EQ(prog.num_vars(), new_prog->num_vars());
  EXPECT_EQ(prog.num_indeterminates(), new_prog->num_indeterminates());
  for (int i = 0; i < prog.num_vars(); ++i) {
    EXPECT_TRUE(
        prog.decision_variable(i).equal_to(new_prog->decision_variable(i)));
    EXPECT_EQ(prog.FindDecisionVariableIndex(prog.decision_variable(i)),
              new_prog->FindDecisionVariableIndex(prog.decision_variable(i)));
    // Cloned program has all variable values set to NaN.
    EXPECT_TRUE(
        std::isnan(new_prog->GetSolution(new_prog->decision_variable(i))));
  }
  for (int i = 0; i < prog.num_indeterminates(); ++i) {
    EXPECT_TRUE(prog.indeterminate(i).equal_to(new_prog->indeterminate(i)));
    EXPECT_EQ(prog.FindIndeterminateIndex(prog.indeterminate((i))),
              new_prog->FindIndeterminateIndex(prog.indeterminate(i)));
  }

  // Cloned program should have the same costs.
  EXPECT_TRUE(
      IsVectorOfBindingEqual(prog.generic_costs(), new_prog->generic_costs()));
  EXPECT_TRUE(
      IsVectorOfBindingEqual(prog.linear_costs(), new_prog->linear_costs()));
  EXPECT_TRUE(IsVectorOfBindingEqual(prog.quadratic_costs(),
                                     new_prog->quadratic_costs()));

  // Cloned program should have the same constraints.
  EXPECT_TRUE(IsVectorOfBindingEqual(prog.generic_constraints(),
                                     new_prog->generic_constraints()));
  EXPECT_TRUE(IsVectorOfBindingEqual(prog.linear_constraints(),
                                     new_prog->linear_constraints()));
  EXPECT_TRUE(IsVectorOfBindingEqual(prog.linear_equality_constraints(),
                                     new_prog->linear_equality_constraints()));
  EXPECT_TRUE(IsVectorOfBindingEqual(prog.bounding_box_constraints(),
                                     new_prog->bounding_box_constraints()));
  EXPECT_TRUE(IsVectorOfBindingEqual(prog.lorentz_cone_constraints(),
                                     new_prog->lorentz_cone_constraints()));
  EXPECT_TRUE(
      IsVectorOfBindingEqual(prog.rotated_lorentz_cone_constraints(),
                             new_prog->rotated_lorentz_cone_constraints()));
  EXPECT_TRUE(
      IsVectorOfBindingEqual(prog.positive_semidefinite_constraints(),
                             new_prog->positive_semidefinite_constraints()));
  EXPECT_TRUE(
      IsVectorOfBindingEqual(prog.linear_matrix_inequality_constraints(),
                             new_prog->linear_matrix_inequality_constraints()));
  EXPECT_TRUE(
      IsVectorOfBindingEqual(prog.linear_matrix_inequality_constraints(),
                             new_prog->linear_matrix_inequality_constraints()));
  EXPECT_TRUE(
      IsVectorOfBindingEqual(prog.linear_complementarity_constraints(),
                             new_prog->linear_complementarity_constraints()));

  EXPECT_TRUE(CompareMatrices(new_prog->initial_guess(), prog.initial_guess()));
  EXPECT_EQ(new_prog->GetSolverId(), prog.GetSolverId());
}

GTEST_TEST(testMathematicalProgram, testEvalBinding) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  // Add linear constraint x(0) + 2x(1) = 2
  auto linear_constraint = prog.AddLinearEqualityConstraint(
      Eigen::RowVector2d(1, 2), Vector1d(2), x.head<2>());
  // Add constraint 0 ≤ x(1)² + 2x(2)² + x(1) + 2x(2) ≤ 1
  auto quadratic_constraint =
      prog.AddConstraint(std::make_shared<QuadraticConstraint>(
                             (Eigen::Matrix2d() << 2, 0, 0, 4).finished(),
                             Eigen::Vector2d(1, 2), 0, 1),
                         x.tail<2>());
  auto quadratic_cost = prog.AddQuadraticCost(x(1) * x(1) + x(2));

  const Eigen::Vector3d x_val(1, 2, 3);
  // The linear constraint should evaluate to 5.
  // The quadratic constraint should evaluate to 30.
  // The quadratic cost should evaluate to 7.
  EXPECT_TRUE(CompareMatrices(prog.EvalBinding(linear_constraint, x_val),
                              Vector1d(5), 1E-15, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(prog.EvalBinding(quadratic_constraint, x_val),
                              Vector1d(30), 1E-15,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(prog.EvalBinding(quadratic_cost, x_val),
                              Vector1d(7), 1E-15, MatrixCompareType::absolute));

  // Pass in an incorrect size input.
  EXPECT_THROW(prog.EvalBinding(linear_constraint, Eigen::Vector2d::Zero()),
               std::logic_error);

  // Pass in some variable not registered in the program.
  symbolic::Variable y("y");
  Binding<QuadraticCost> quadratic_cost_y(
      std::make_shared<QuadraticCost>(Vector1d(1), Vector1d(0)),
      VectorDecisionVariable<1>(y));
  EXPECT_THROW(prog.EvalBinding(quadratic_cost_y, x_val), std::runtime_error);
}

GTEST_TEST(testMathematicalProgram, testSetAndGetInitialGuess) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();

  // Set initial guess for a single variable.
  prog.SetInitialGuess(x(1), 2);
  EXPECT_EQ(prog.GetInitialGuess(x(1)), 2);

  // Set initial guess for a vector of variables.
  prog.SetInitialGuess(x.tail<2>(), Eigen::Vector2d(3, 4));
  EXPECT_TRUE(CompareMatrices(prog.GetInitialGuess(x.tail<2>()),
                              Eigen::Vector2d(3, 4)));

  // Now set initial guess for a variable not registered.
  symbolic::Variable y("y");
  EXPECT_THROW(prog.SetInitialGuess(y, 1), std::runtime_error);
  EXPECT_THROW(prog.GetInitialGuess(y), std::runtime_error);
}

GTEST_TEST(testMathematicalProgram, testNonlinearExpressionConstraints) {
  // min ∑ x , subject to x'x = 1.
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();

  prog.AddConstraint(x.transpose()*x == 1.);

  if (SnoptSolver().available()) {
    // Add equivalent constraints using all of the other entry points.
    // Note: restricted to SNOPT because IPOPT complains about the redundant
    // constraints.
    prog.AddConstraint(x.transpose()*x >= 1.);
    prog.AddConstraint(x.transpose()*x <= 1.);
    prog.AddConstraint((x.transpose()*x)(0), 1., 1.);
    prog.AddConstraint(x.transpose()*x, Vector1d{1.}, Vector1d{1.});
  }

  prog.AddCost(x(0) + x(1));
  prog.SetInitialGuess(x, Vector2d{-.5, -.5});
  SolutionResult result = prog.Solve();
  EXPECT_EQ(result, kSolutionFound);
  EXPECT_TRUE(CompareMatrices(prog.GetSolution(x),
                              Vector2d::Constant(-std::sqrt(2.)/2.), 1e-6));
}

GTEST_TEST(testMathematicalProgram, testSetSolverResult) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  // Pretend the problem has been solved.
  const SolverId dummy_solver_id("dummy");

  // Only set the solver ID in solver_result.
  SolverResult solver_result(dummy_solver_id);

  prog.SetSolverResult(solver_result);
  EXPECT_EQ(prog.GetSolverId(), dummy_solver_id);
  // The decision variables, optimal cost, and lower bound should all be NaN.
  EXPECT_TRUE(CompareMatrices(
      prog.GetSolution(x),
      Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN())));
  EXPECT_TRUE(std::isnan(prog.GetOptimalCost()));
  EXPECT_TRUE(std::isnan(prog.GetLowerBoundCost()));

  // Sets the variable values, the optimal cost, and lower bound cost.
  const Eigen::Vector2d x_val(1, 2);
  const double cost{1.0};
  const double lower_bound_cost{1.0};
  solver_result.set_decision_variable_values(x_val);
  solver_result.set_optimal_cost(cost);
  solver_result.set_optimal_cost_lower_bound(lower_bound_cost);
  prog.SetSolverResult(solver_result);
  EXPECT_TRUE(CompareMatrices(prog.GetSolution(x), x_val));
  EXPECT_EQ(prog.GetOptimalCost(), cost);
  EXPECT_EQ(prog.GetLowerBoundCost(), lower_bound_cost);

  // Now create a new solver_result.
  const SolverId dummy_solver_id2("dummy2");
  SolverResult solver_result2(dummy_solver_id2);
  prog.SetSolverResult(solver_result2);
  EXPECT_EQ(prog.GetSolverId(), dummy_solver_id2);
  // The decision variables, optimal cost, and lower bound should all be NaN.
  EXPECT_TRUE(CompareMatrices(
      prog.GetSolution(x),
      Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN())));
  EXPECT_TRUE(std::isnan(prog.GetOptimalCost()));
  EXPECT_TRUE(std::isnan(prog.GetLowerBoundCost()));
}

GTEST_TEST(testMathematicalProgram, testAddVisualizationCallback) {
  MathematicalProgram prog;

  auto x = prog.NewContinuousVariables<2>();
  bool was_called = false;
  auto my_callback = [&was_called](const Eigen::Ref<const Eigen::VectorXd>& v) {
    EXPECT_EQ(v.size(), 2);
    EXPECT_EQ(v(0), 1.);
    EXPECT_EQ(v(1), 2.);
    was_called = true;
  };

  Binding<VisualizationCallback> b =
      prog.AddVisualizationCallback(my_callback, x);
  EXPECT_EQ(prog.visualization_callbacks().size(), 1);

  const Vector2d test_x(1., 2.);

  // Call it via EvalVisualizationCallbacks.
  was_called = false;
  prog.EvalVisualizationCallbacks(test_x);
  EXPECT_TRUE(was_called);

  // Call it via EvalBinding.
  was_called = false;
  prog.EvalBinding(b, test_x);
  EXPECT_TRUE(was_called);

  // Call it directly via the double interface.
  VectorXd test_y(0);
  was_called = false;
  b.evaluator()->Eval(test_x, &test_y);
  EXPECT_TRUE(was_called);

  // Call it directly via the autodiff interface.
  const VectorX<AutoDiffXd> test_x_autodiff =
      math::initializeAutoDiff(VectorXd{test_x});
  VectorX<AutoDiffXd> test_y_autodiff(0);
  was_called = false;
  b.evaluator()->Eval(test_x_autodiff, &test_y_autodiff);
  EXPECT_TRUE(was_called);
}

GTEST_TEST(testMathematicalProgram, TestSolverOptions) {
  MathematicalProgram prog;
  const SolverId solver_id("solver_id");
  const SolverId wrong_solver_id("wrong_solver_id");

  prog.SetSolverOption(solver_id, "double_name", 1.0);
  EXPECT_EQ(prog.GetSolverOptionsDouble(solver_id).at("double_name"), 1.0);
  EXPECT_EQ(prog.GetSolverOptionsDouble(wrong_solver_id).size(), 0);

  prog.SetSolverOption(solver_id, "int_name", 2);
  EXPECT_EQ(prog.GetSolverOptionsInt(solver_id).at("int_name"), 2);
  EXPECT_EQ(prog.GetSolverOptionsInt(wrong_solver_id).size(), 0);

  prog.SetSolverOption(solver_id, "string_name", "3");
  EXPECT_EQ(prog.GetSolverOptionsStr(solver_id).at("string_name"), "3");
  EXPECT_EQ(prog.GetSolverOptionsStr(wrong_solver_id).size(), 0);
}

GTEST_TEST(testMathematicalProgram, TestGetSolution) {
  // Test GetSolution(var, result)
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  MathematicalProgramResult result;
  result.set_x_val(Eigen::Vector2d(1, 2));

  // GetSolution(var, result) retrieves solution from `result`, GetSolution(var)
  // retrieves the solution stored in prog. They should not be equal.
  EXPECT_NE(prog.GetSolution(x(0), result), prog.GetSolution(x(0)));
  EXPECT_NE(prog.GetSolution(x(1), result), prog.GetSolution(x(1)));

  EXPECT_EQ(prog.GetSolution(x(0), result), 1);
  EXPECT_EQ(prog.GetSolution(x(1), result), 2);
  EXPECT_TRUE(CompareMatrices(
      prog.GetSolution(Vector3<symbolic::Variable>(x(0), x(1), x(1)), result),
      Eigen::Vector3d(1, 2, 2)));

  // If result.get_x_val has wrong dimension, expect to throw an error.
  result.set_x_val(Eigen::Vector3d::Zero());
  EXPECT_THROW(prog.GetSolution(x(0), result), std::invalid_argument);
}

GTEST_TEST(testMathematicalProgram, NewSosPolynomial) {
  // Check if the newly created sos polynomial can be computed as m' * Q * m.
  MathematicalProgram prog;
  auto t = prog.NewIndeterminates<4>();
  const auto m = symbolic::MonomialBasis<4, 2>(symbolic::Variables(t));
  const auto pair = prog.NewSosPolynomial(m);
  const symbolic::Polynomial p = pair.first;
  const Binding<PositiveSemidefiniteConstraint> psd_binding = pair.second;
  const auto Q_flat = psd_binding.variables();
  MatrixX<symbolic::Polynomial> Q_poly(m.rows(), m.rows());
  const symbolic::Monomial monomial_one{};
  for (int i = 0; i < Q_poly.rows(); ++i) {
    for (int j = 0; j < Q_poly.cols(); ++j) {
      Q_poly(i, j) =
          symbolic::Polynomial({{monomial_one, Q_flat(j * Q_poly.rows() + i)}});
    }
  }
  const symbolic::Polynomial p_expected(m.dot(Q_poly * m));
  EXPECT_TRUE(p.EqualTo(p_expected));
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
