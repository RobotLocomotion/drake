#include "drake/solvers/mathematical_program.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/polynomial.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/common/text_logging.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/program_attribute.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/test/generic_trivial_constraints.h"
#include "drake/solvers/test/generic_trivial_costs.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::Vector1d;
using drake::solvers::internal::VecIn;
using drake::solvers::internal::VecOut;
using drake::symbolic::Expression;
using drake::symbolic::Formula;
using drake::symbolic::Variable;
using drake::symbolic::test::ExprEqual;
using drake::symbolic::test::PolyEqual;
using drake::symbolic::test::PolyNotEqual;
using drake::symbolic::test::TupleVarEqual;

using std::all_of;
using std::cref;
using std::endl;
using std::is_permutation;
using std::is_same_v;
using std::make_shared;
using std::map;
using std::numeric_limits;
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

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

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
  static_assert(is_same_v<T, ExpectedType>, "Type not match");
  EXPECT_EQ(var.rows(), rows);
  EXPECT_EQ(var.cols(), cols);
  // Checks the name of the newly added variables.
  EXPECT_EQ(fmt::to_string(fmt_eigen(var)), var_name);
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
  EXPECT_EQ(fmt::to_string(fmt_eigen(indeterminates)), indeterminates_name);
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

GTEST_TEST(TestMathematicalProgram, TestConstructor) {
  MathematicalProgram prog;
  EXPECT_EQ(prog.initial_guess().rows(), 0);
  EXPECT_EQ(prog.num_vars(), 0);
}

GTEST_TEST(TestAddVariable, TestAddContinuousVariables1) {
  // Adds a dynamic-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables(2, 3, "X");
  CheckAddedVariable<MatrixXDecisionVariable>(
      prog, X, 2, 3, "X(0,0) X(0,1) X(0,2)\nX(1,0) X(1,1) X(1,2)", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(TestAddVariable, TestAddContinuousVariable2) {
  // Adds a static-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables<2, 3>("X");
  CheckAddedVariable<MatrixDecisionVariable<2, 3>>(
      prog, X, 2, 3, "X(0,0) X(0,1) X(0,2)\nX(1,0) X(1,1) X(1,2)", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(TestAddVariable, TestAddContinuousVariable3) {
  // Adds a dynamic-sized vector of continuous variables.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(4, "x");
  CheckAddedVariable<VectorXDecisionVariable>(
      prog, x, 4, 1, "x(0)\nx(1)\nx(2)\nx(3)", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(TestAddVariable, TestAddContinuousVariable4) {
  // Adds a static-sized vector of continuous variables.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>("y");
  CheckAddedVariable<VectorDecisionVariable<4>>(
      prog, x, 4, 1, "y(0)\ny(1)\ny(2)\ny(3)", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(TestAddVariable, TestAddContinuousVariable5) {
  // Adds a static-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables<2, 3>(2, 3, "Y");
  CheckAddedVariable<MatrixDecisionVariable<2, 3>>(
      prog, X, 2, 3, "Y(0,0) Y(0,1) Y(0,2)\nY(1,0) Y(1,1) Y(1,2)", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(TestAddVariable, TestAddContinuousVariables6) {
  // Adds a dynamic-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X =
      prog.NewContinuousVariables<Eigen::Dynamic, Eigen::Dynamic>(2, 3, "Y");
  CheckAddedVariable<MatrixXDecisionVariable>(
      prog, X, 2, 3, "Y(0,0) Y(0,1) Y(0,2)\nY(1,0) Y(1,1) Y(1,2)", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(TestAddVariable, TestAddContinuousVariables7) {
  // Adds a dynamic-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables<2, Eigen::Dynamic>(2, 3, "Y");
  CheckAddedVariable<MatrixDecisionVariable<2, Eigen::Dynamic>>(
      prog, X, 2, 3, "Y(0,0) Y(0,1) Y(0,2)\nY(1,0) Y(1,1) Y(1,2)", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(TestAddVariable, TestAddContinuousVariables8) {
  // Adds a dynamic-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables<Eigen::Dynamic, 3>(2, 3, "Y");
  CheckAddedVariable<MatrixDecisionVariable<Eigen::Dynamic, 3>>(
      prog, X, 2, 3, "Y(0,0) Y(0,1) Y(0,2)\nY(1,0) Y(1,1) Y(1,2)", false,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(TestAddVariable, TestAddContinuousVariables9) {
  // Adds continuous variables with default variable name.
  const std::string X_names = "X(0,0) X(0,1) X(0,2)\nX(1,0) X(1,1) X(1,2)";
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

GTEST_TEST(TestAddVariable, TestAddSymmetricVariable1) {
  // Adds a static-sized symmetric matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>("X");
  CheckAddedVariable<MatrixDecisionVariable<3, 3>>(
      prog, X, 3, 3,
      "X(0,0) X(1,0) X(2,0)\nX(1,0) X(1,1) X(2,1)\nX(2,0) X(2,1) X(2,2)", true,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(TestAddVariable, TestAddSymmetricVariable2) {
  // Adds a dynamic-sized symmetric matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables(3, "X");
  CheckAddedVariable<MatrixXDecisionVariable>(
      prog, X, 3, 3,
      "X(0,0) X(1,0) X(2,0)\nX(1,0) X(1,1) X(2,1)\nX(2,0) X(2,1) X(2,2)", true,
      MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(TestAddVariable, TestAddBinaryVariable1) {
  // Adds a dynamic-sized matrix of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables(2, 3, "B");
  CheckAddedVariable<MatrixXDecisionVariable>(
      prog, X, 2, 3, "B(0,0) B(0,1) B(0,2)\nB(1,0) B(1,1) B(1,2)", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(TestAddVariable, TestAddBinaryVariable2) {
  // Adds a dynamic-sized matrix of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables<Eigen::Dynamic, Eigen::Dynamic>(2, 3, "B");
  CheckAddedVariable<MatrixXDecisionVariable>(
      prog, X, 2, 3, "B(0,0) B(0,1) B(0,2)\nB(1,0) B(1,1) B(1,2)", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(TestAddVariable, TestAddBinaryVariable3) {
  // Adds dynamic-sized vector of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables(4, "B");
  CheckAddedVariable<VectorXDecisionVariable>(
      prog, X, 4, 1, "B(0)\nB(1)\nB(2)\nB(3)", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(TestAddVariable, TestAddBinaryVariable4) {
  // Adds static-sized vector of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables<4>("B");
  CheckAddedVariable<VectorDecisionVariable<4>>(
      prog, X, 4, 1, "B(0)\nB(1)\nB(2)\nB(3)", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(TestAddVariable, TestAddBinaryVariable5) {
  // Adds a static-sized matrix of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables<2, 3>("B");
  CheckAddedVariable<MatrixDecisionVariable<2, 3>>(
      prog, X, 2, 3, "B(0,0) B(0,1) B(0,2)\nB(1,0) B(1,1) B(1,2)", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(TestAddVariable, TestAddBinaryVariable6) {
  // Adds a static-sized matrix of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables<2, 3>(2, 3, "B");
  CheckAddedVariable<MatrixDecisionVariable<2, 3>>(
      prog, X, 2, 3, "B(0,0) B(0,1) B(0,2)\nB(1,0) B(1,1) B(1,2)", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(TestAddVariable, TestAddBinaryVariable7) {
  // Adds a dynamic-sized matrix of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables<2, Eigen::Dynamic>(2, 3, "B");
  CheckAddedVariable<MatrixDecisionVariable<2, Eigen::Dynamic>>(
      prog, X, 2, 3, "B(0,0) B(0,1) B(0,2)\nB(1,0) B(1,1) B(1,2)", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(TestAddVariable, TestAddBinaryVariable8) {
  // Adds a dynamic-sized matrix of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables<Eigen::Dynamic, 3>(2, 3, "B");
  CheckAddedVariable<MatrixDecisionVariable<Eigen::Dynamic, 3>>(
      prog, X, 2, 3, "B(0,0) B(0,1) B(0,2)\nB(1,0) B(1,1) B(1,2)", false,
      MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(TestAddDecisionVariables, AddDecisionVariables1) {
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
  EXPECT_TRUE(
      prog.required_capabilities().contains(ProgramAttribute::kBinaryVariable));

  const auto decision_variable_index = prog.decision_variable_index();
  {
    const auto it = decision_variable_index.find(x0.get_id());
    ASSERT_TRUE(it != decision_variable_index.end());
    EXPECT_EQ(it->second, prog.FindDecisionVariableIndex(x0));
  }
  {
    const auto it = decision_variable_index.find(x1.get_id());
    ASSERT_TRUE(it != decision_variable_index.end());
    EXPECT_EQ(it->second, prog.FindDecisionVariableIndex(x1));
  }
  {
    const auto it = decision_variable_index.find(x2.get_id());
    ASSERT_TRUE(it != decision_variable_index.end());
    EXPECT_EQ(it->second, prog.FindDecisionVariableIndex(x2));
  }
}

GTEST_TEST(TestAddDecisionVariables, AddVariable2) {
  // Call AddDecisionVariables on a program that has some existing variables.
  MathematicalProgram prog;
  auto y = prog.NewContinuousVariables<3>("y");
  const Variable x0("x0", Variable::Type::CONTINUOUS);
  const Variable x1("x1", Variable::Type::CONTINUOUS);
  const Variable x2("x2", Variable::Type::BINARY);
  prog.AddDecisionVariables(VectorDecisionVariable<3>(x0, x1, x2));
  EXPECT_EQ(prog.num_vars(), 6);
  EXPECT_EQ(prog.num_indeterminates(), 0);
  EXPECT_EQ(prog.FindDecisionVariableIndex(x0), 3);
  EXPECT_EQ(prog.FindDecisionVariableIndex(x1), 4);
  EXPECT_EQ(prog.FindDecisionVariableIndex(x2), 5);
  EXPECT_EQ(prog.initial_guess().rows(), 6);
}

GTEST_TEST(TestAddDecisionVariables, AddVariable3) {
  // Call AddDecisionVariables on a program that has some existing variables.
  // and the new variable overlap with the old variables.
  MathematicalProgram prog;
  auto y = prog.NewContinuousVariables<3>("y");
  const Variable x0("x0", Variable::Type::CONTINUOUS);
  const Variable x1("x1", Variable::Type::CONTINUOUS);
  prog.AddDecisionVariables(VectorDecisionVariable<3>(x0, y(1), x1));
  EXPECT_EQ(prog.num_vars(), 5);
  EXPECT_EQ(prog.num_indeterminates(), 0);
  EXPECT_EQ(prog.FindDecisionVariableIndex(x0), 3);
  EXPECT_EQ(prog.FindDecisionVariableIndex(x1), 4);
  EXPECT_EQ(prog.initial_guess().rows(), 5);
}

GTEST_TEST(TestAddDecisionVariables, AddVariableError) {
  // Test the error inputs.
  MathematicalProgram prog;
  auto z = prog.NewIndeterminates<2>("z");
  // Call AddDecisionVariables on a program that has some indeterminates, and
  // the new variables intersects with the indeterminates.
  const Variable x0("x0", Variable::Type::CONTINUOUS);
  EXPECT_THROW(prog.AddDecisionVariables(VectorDecisionVariable<2>(x0, z(0))),
               std::runtime_error);

  // Call AddDecisionVariables with unsupported variable type.
  for (symbolic::Variable::Type unsupported_type :
       {symbolic::Variable::Type::BOOLEAN,
        symbolic::Variable::Type::RANDOM_UNIFORM,
        symbolic::Variable::Type::RANDOM_GAUSSIAN,
        symbolic::Variable::Type::RANDOM_EXPONENTIAL}) {
    const symbolic::Variable unsupported_var("b", unsupported_type);
    DRAKE_EXPECT_THROWS_MESSAGE(
        prog.AddDecisionVariables(VectorDecisionVariable<1>(unsupported_var)),
        "MathematicalProgram does not support .* variables.");
  }
}

GTEST_TEST(TestAddDecisionVariables, TestMatrixInput) {
  // AddDecisionVariables with a matrix of variables instead of a vector.
  Eigen::Matrix<symbolic::Variable, 2, 3> vars;
  for (int i = 0; i < vars.rows(); ++i) {
    for (int j = 0; j < vars.cols(); ++j) {
      vars(i, j) = symbolic::Variable(fmt::format("x({},{})", i, j));
    }
  }
  MathematicalProgram prog;
  prog.NewContinuousVariables<1>();
  const int num_existing_decision_vars = prog.num_vars();
  prog.AddDecisionVariables(vars);
  EXPECT_EQ(prog.num_vars(), 6 + num_existing_decision_vars);
  EXPECT_EQ(prog.GetInitialGuess(vars).rows(), 2);
  EXPECT_EQ(prog.GetInitialGuess(vars).cols(), 3);
  EXPECT_TRUE(prog.GetInitialGuess(vars).array().isNaN().all());
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      // Make sure that the variable has been registered in prog.
      EXPECT_NO_THROW(unused(prog.FindDecisionVariableIndex(vars(i, j))));
    }
  }
}

GTEST_TEST(NewIndeterminates, DynamicSizeMatrix) {
  // Adds a dynamic-sized matrix of Indeterminates.
  MathematicalProgram prog;
  auto X = prog.NewIndeterminates(2, 3, "X");
  static_assert(is_same_v<decltype(X), MatrixXIndeterminate>,
                "should be a dynamic sized matrix");
  EXPECT_EQ(X.rows(), 2);
  EXPECT_EQ(X.cols(), 3);
  CheckAddedIndeterminates(prog, X,
                           "X(0,0) X(0,1) X(0,2)\nX(1,0) X(1,1) X(1,2)");
}

GTEST_TEST(NewIndeterminates, StaticSizeMatrix) {
  // Adds a static-sized matrix of Indeterminates.
  MathematicalProgram prog;
  auto X = prog.NewIndeterminates<2, 3>("X");
  static_assert(is_same_v<decltype(X), MatrixIndeterminate<2, 3>>,
                "should be a static sized matrix");
  CheckAddedIndeterminates(prog, X,
                           "X(0,0) X(0,1) X(0,2)\nX(1,0) X(1,1) X(1,2)");
}

GTEST_TEST(NewIndeterminates, DynamicSizeVector) {
  // Adds a dynamic-sized vector of Indeterminates.
  MathematicalProgram prog;
  auto x = prog.NewIndeterminates(4, "x");
  static_assert(is_same_v<decltype(x), VectorXIndeterminate>,
                "Should be a VectorXDecisionVariable object.");
  EXPECT_EQ(x.rows(), 4);
  CheckAddedIndeterminates(prog, x, "x(0)\nx(1)\nx(2)\nx(3)");
}

GTEST_TEST(NewIndeterminates, StaticSizeVector) {
  // Adds a static-sized vector of Indeterminate variables.
  MathematicalProgram prog;
  auto x = prog.NewIndeterminates<4>("x");
  static_assert(is_same_v<decltype(x), VectorIndeterminate<4>>,
                "Should be a VectorXDecisionVariable object.");
  CheckAddedIndeterminates(prog, x, "x(0)\nx(1)\nx(2)\nx(3)");
}

GTEST_TEST(TestAddIndeterminate, AddIndeterminate1) {
  // Call AddIndeterminate on an empty program.
  MathematicalProgram prog;
  const Variable x("x", Variable::Type::CONTINUOUS);
  int var_index = prog.AddIndeterminate(x);
  EXPECT_EQ(prog.indeterminates().rows(), 1);
  EXPECT_EQ(var_index, 0);

  EXPECT_TRUE(prog.indeterminates()(0).equal_to(x));
  EXPECT_EQ(prog.FindIndeterminateIndex(x), 0);

  const auto it = prog.indeterminates_index().find(x.get_id());
  EXPECT_TRUE(it != prog.indeterminates_index().end());
  EXPECT_EQ(it->second, prog.FindIndeterminateIndex(x));
}

GTEST_TEST(TestAddIndeterminate, AddIndeterminate2) {
  // Call AddIndeterminate on a program with some indeterminates
  MathematicalProgram prog;
  auto y = prog.NewIndeterminates<2>("y");

  const Variable x("x", Variable::Type::CONTINUOUS);
  int var_index = prog.AddIndeterminate(x);

  EXPECT_EQ(prog.indeterminates().rows(), 3);
  EXPECT_EQ(var_index, 2);

  VectorIndeterminate<3> indeterminates_expected;
  indeterminates_expected << y(0), y(1), x;

  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(prog.indeterminates()(i).equal_to(indeterminates_expected(i)));
    EXPECT_EQ(prog.FindIndeterminateIndex(indeterminates_expected(i)), i);
  }
}

GTEST_TEST(TestAddIndeterminate, AddIndeterminate3) {
  // prog already contains some indeterminates, and we call AddIndeterminate on
  // an old indeterminate.
  MathematicalProgram prog;
  auto y = prog.NewIndeterminates<3>();

  auto var_index = prog.AddIndeterminate(y(1));
  EXPECT_EQ(var_index, prog.FindIndeterminateIndex(y(1)));
  EXPECT_EQ(prog.indeterminates().size(), 3);
  EXPECT_EQ(prog.indeterminates_index().size(), 3);
}

GTEST_TEST(TestAddIndeterminate, AddIndeterminateError) {
  // Call with erroneous inputs.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  auto y = prog.NewIndeterminates<2>("y");
  const Variable z("z", Variable::Type::BINARY);
  // Call AddIndeterminate with an input that intersects with old decision
  // variables.
  DRAKE_EXPECT_THROWS_MESSAGE(prog.AddIndeterminate(x(0)),
                              ".*is a decision variable.*");
  // Call AddIndeterminate with an input of type BINARY.
  DRAKE_EXPECT_THROWS_MESSAGE(prog.AddIndeterminate(z),
                              ".*should be of type CONTINUOUS.*");
}

GTEST_TEST(TestAddIndeterminates, AddIndeterminatesVec1) {
  // Call AddIndeterminates on an empty program.
  MathematicalProgram prog;
  const Variable x0("x0", Variable::Type::CONTINUOUS);
  const Variable x1("x1", Variable::Type::CONTINUOUS);
  const Variable x2("x2", Variable::Type::CONTINUOUS);
  prog.AddIndeterminates(VectorIndeterminate<3>(x0, x1, x2));
  const VectorIndeterminate<3> indeterminates_expected(x0, x1, x2);
  EXPECT_EQ(prog.indeterminates().rows(), 3);

  const auto indeterminates_index = prog.indeterminates_index();
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(prog.indeterminates()(i).equal_to(indeterminates_expected(i)));
    EXPECT_EQ(prog.FindIndeterminateIndex(indeterminates_expected(i)), i);

    const auto it =
        indeterminates_index.find(indeterminates_expected(i).get_id());
    ASSERT_TRUE(it != indeterminates_index.end());
    EXPECT_EQ(it->second,
              prog.FindIndeterminateIndex(indeterminates_expected(i)));
  }
}

GTEST_TEST(TestAddIndeterminates, AddIndeterminatesVec2) {
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

GTEST_TEST(TestAddIndeterminates, AddIndeterminatesVec3) {
  // The program already constains some indeterminates, call AddIndeterminates()
  // with some of the old indeterminates.
  MathematicalProgram prog;
  auto y = prog.NewIndeterminates<3>();
  const symbolic::Variable x0("x0", Variable::Type::CONTINUOUS);
  prog.AddIndeterminates(Vector2<symbolic::Variable>(y(1), x0));
  EXPECT_EQ(prog.indeterminates().size(), 4);
  EXPECT_EQ(prog.indeterminates_index().size(), 4);
  EXPECT_EQ(prog.FindIndeterminateIndex(x0), 3);
  EXPECT_EQ(prog.FindIndeterminateIndex(y(1)), 1);

  // AddIndeterminates with duplicated variables.
  const symbolic::Variable x1("x1", Variable::Type::CONTINUOUS);
  prog.AddIndeterminates(Vector3<symbolic::Variable>(x1, x0, x1));
  EXPECT_EQ(prog.indeterminates().size(), 5);
  EXPECT_EQ(prog.indeterminates_index().size(), 5);
  EXPECT_EQ(prog.FindIndeterminateIndex(x1), 4);
}

GTEST_TEST(TestAddIndeterminates, AddIndeterminatesVecError) {
  // Call with erroneous inputs.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  auto y = prog.NewIndeterminates<2>("y");
  const Variable x0("x0", Variable::Type::CONTINUOUS);
  const Variable x1("x1", Variable::Type::BINARY);
  // Call AddIndeterminates with an input that intersects with old decision
  // variables.
  DRAKE_EXPECT_THROWS_MESSAGE(
      prog.AddIndeterminates(VectorIndeterminate<2>(x(0), x0)),
      ".*is a decision variable.*");
  // Call AddIndeterminates with an input of type BINARY.
  DRAKE_EXPECT_THROWS_MESSAGE(
      prog.AddIndeterminates(VectorIndeterminate<2>(x1, x0)),
      ".*should be of type CONTINUOUS.*");
}

GTEST_TEST(TestAddIndeterminates, AddIndeterminatesVars1) {
  // Call AddIndeterminates on an empty program.
  MathematicalProgram prog;
  const Variable x0("x0", Variable::Type::CONTINUOUS);
  const Variable x1("x1", Variable::Type::CONTINUOUS);
  const Variable x2("x2", Variable::Type::CONTINUOUS);
  prog.AddIndeterminates(symbolic::Variables({x0, x1, x2}));
  const VectorIndeterminate<3> indeterminates_expected(x0, x1, x2);
  EXPECT_EQ(prog.indeterminates().rows(), 3);

  const auto indeterminates_index = prog.indeterminates_index();
  for (int i = 0; i < 3; ++i) {
    // the indeterminate is in the program, but in arbitrary place so we don't
    // test its index.
    const auto it =
        indeterminates_index.find(indeterminates_expected(i).get_id());
    ASSERT_TRUE(it != indeterminates_index.end());
    EXPECT_EQ(it->second,
              prog.FindIndeterminateIndex(indeterminates_expected(i)));
  }
}

GTEST_TEST(TestAddIndeterminates, AddIndeterminatesVars2) {
  // Call AddIndeterminates on a program with some indeterminates.
  MathematicalProgram prog;
  auto y = prog.NewIndeterminates<2>("y");
  const Variable x0("x0", Variable::Type::CONTINUOUS);
  const Variable x1("x1", Variable::Type::CONTINUOUS);
  const Variable x2("x2", Variable::Type::CONTINUOUS);
  prog.AddIndeterminates(symbolic::Variables({x0, x1, x2}));
  VectorIndeterminate<5> indeterminates_expected;
  indeterminates_expected << y(0), y(1), x0, x1, x2;
  EXPECT_EQ(prog.indeterminates().rows(), 5);

  const auto indeterminates_index = prog.indeterminates_index();
  for (int i = 0; i < 5; ++i) {
    if (i < 3) {
      // The variables already in the program should be in order.
      EXPECT_TRUE(
          prog.indeterminates()(i).equal_to(indeterminates_expected(i)));
      EXPECT_EQ(prog.FindIndeterminateIndex(indeterminates_expected(i)), i);
    } else {
      // The remaining variables were added using an unordered set so we can't
      // expect an order.
      const auto it =
          indeterminates_index.find(indeterminates_expected(i).get_id());
      ASSERT_TRUE(it != indeterminates_index.end());
      EXPECT_EQ(it->second,
                prog.FindIndeterminateIndex(indeterminates_expected(i)));
    }
  }
}

GTEST_TEST(TestAddIndeterminates, AddIndeterminatesVars3) {
  // Call AddIndeterminates that overlaps with the old indeterminates.
  MathematicalProgram prog;
  auto y = prog.NewIndeterminates<3>();
  const Variable x0("x0", Variable::Type::CONTINUOUS);
  const Variable x1("x1", Variable::Type::CONTINUOUS);
  const symbolic::Variables var_set{{x0, x1, y(1)}};
  prog.AddIndeterminates(var_set);
  EXPECT_EQ(prog.num_indeterminates(), 5);
  EXPECT_EQ(prog.indeterminates().size(), 5);
  EXPECT_EQ(prog.indeterminates_index().size(), 5);
  EXPECT_GT(prog.FindIndeterminateIndex(x0), 2);
  EXPECT_GT(prog.FindIndeterminateIndex(x1), 2);
}

GTEST_TEST(TestAddIndeterminates, AddIndeterminatesVarsError) {
  // Call with erroneous inputs.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  auto y = prog.NewIndeterminates<2>("y");
  const Variable x0("x0", Variable::Type::CONTINUOUS);
  const Variable x1("x1", Variable::Type::BINARY);
  // Call AddIndeterminates with an input that intersects with old decision
  // variables.
  DRAKE_EXPECT_THROWS_MESSAGE(
      prog.AddIndeterminates(symbolic::Variables({x0, x(0)})),
      ".*is a decision variable.*");
  // Call AddIndeterminates with an input of type BINARY.
  DRAKE_EXPECT_THROWS_MESSAGE(
      prog.AddIndeterminates(symbolic::Variables({x0, x1})),
      ".*should be of type CONTINUOUS.*");
}

GTEST_TEST(TestAddIndeterminates, MatrixInput) {
  Eigen::Matrix<symbolic::Variable, 2, 3, Eigen::RowMajor> vars;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      vars(i, j) = symbolic::Variable(fmt::format("x({},{})", i, j));
    }
  }
  MathematicalProgram prog;
  prog.NewIndeterminates<2>();
  const int num_existing_indeterminates = prog.num_indeterminates();
  prog.AddIndeterminates(vars);
  EXPECT_EQ(prog.num_indeterminates(), num_existing_indeterminates + 6);
  EXPECT_EQ(prog.num_vars(), 0);
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NO_THROW(unused(prog.FindIndeterminateIndex(vars(i, j))));
    }
  }
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
  DRAKE_EXPECT_THROWS_MESSAGE(AddItem(prog, CreateBinding(c, x)),
                              ".*is not a decision variable.*");
}

// Returns True if all the elements of the matrix m1 and m2 are EqualTo as
// Expressions.
bool MatrixExprAllEqual(const MatrixX<Expression>& m1,
                        const MatrixX<Expression>& m2) {
  return m1
      .binaryExpr(m2,
                  [](const Expression& u, const Expression& v) {
                    return u.EqualTo(v);
                  })
      .all();
}

}  // namespace

GTEST_TEST(TestMathematicalProgram, TestMakePolynomial) {
  MathematicalProgram prog;
  const auto x = prog.NewIndeterminates<2>("x");
  const auto a = prog.NewContinuousVariables<2>("a");

  // A decision variable that does not belong
  // to this mathematical program.
  const symbolic::Variable b{"b"};

  // e = a₀x₀ + (a₁ + b)x₀x₁ + (a₁b).
  const Expression e{a(0) * x(0) + (a(1) + b) * x(0) * x(1) + (a(1) * b)};
  const symbolic::Polynomial p{prog.MakePolynomial(e)};

  // We check the constructed polynomial has the following internal mapping.
  //   x₀ ↦ a₀
  //   x₀x₁ ↦ (a₁ + b)
  //   1 ↦ a₁b
  const auto& coeff_map = p.monomial_to_coefficient_map();
  EXPECT_EQ(coeff_map.size(), 3);
  const symbolic::Monomial x0{x(0)};
  const symbolic::Monomial x0x1{{{x(0), 1}, {x(1), 1}}};
  const symbolic::Monomial one;
  EXPECT_PRED2(ExprEqual, coeff_map.at(x0), a(0));
  EXPECT_PRED2(ExprEqual, coeff_map.at(x0x1), a(1) + b);
  EXPECT_PRED2(ExprEqual, coeff_map.at(one), a(1) * b);
}

GTEST_TEST(TestMathematicalProgram, TestBadBindingVariable) {
  // Attempt to add a binding that does not have a valid decision variable.
  MathematicalProgram prog;

  const int num_var = 3;
  Eigen::Matrix3d A;
  A.setIdentity();
  Eigen::Vector3d f, lb, ub;
  f.setConstant(2);
  lb.setConstant(0);
  ub.setConstant(1);
  vector<MatrixXd> F{A, 2 * A};
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
  ExpectBadVar<ExponentialConeConstraint>(
      &prog, num_var, Eigen::MatrixXd::Ones(3, num_var).sparseView(),
      Eigen::Vector3d::Zero());
  ExpectBadVar<LinearComplementarityConstraint>(&prog, num_var, A, f);
  // Use this as a test for nonlinear constraints.
  ExpectBadVar<EvaluatorConstraint<>>(&prog, 1, func, lb.head(1), ub.head(1));

  // Test each cost type.
  ExpectBadVar<LinearCost>(&prog, num_var, f);
  ExpectBadVar<QuadraticCost>(&prog, num_var, A, f);
  ExpectBadVar<EvaluatorCost<>>(&prog, 1, func);
}

GTEST_TEST(TestMathematicalProgram, TestAddFunction) {
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

GTEST_TEST(TestMathematicalProgram, BoundingBoxTest2) {
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

GTEST_TEST(TestMathematicalProgram, BoundingBoxTest3) {
  // The bounds and variables are matrices.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables(3, 2, "X");
  Eigen::MatrixXd X_lo(3, 2);
  X_lo << 1, 2, 3, 4, 5, 6;
  // Use a row-major matrix to make sure that our code works for different types
  // of matrix.
  Eigen::Matrix<double, 3, 2, Eigen::RowMajor> X_up =
      (X_lo.array() + 1).matrix();
  auto cnstr = prog.AddBoundingBoxConstraint(X_lo, X_up, X);
  EXPECT_EQ(cnstr.evaluator()->num_constraints(), 6);
  std::unordered_map<symbolic::Variable, std::pair<double, double>> X_bounds;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      X_bounds.emplace(X(i, j), std::make_pair(X_lo(i, j), X_up(i, j)));
    }
  }
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(cnstr.evaluator()->lower_bound()(i),
              X_bounds.at(cnstr.variables()(i)).first);
    EXPECT_EQ(cnstr.evaluator()->upper_bound()(i),
              X_bounds.at(cnstr.variables()(i)).second);
  }

  // Now add constraint on X.topRows<2>(). It doesn't occupy contiguous memory.
  auto cnstr2 = prog.AddBoundingBoxConstraint(
      X_lo.topRows<2>(), X_up.topRows<2>(), X.topRows<2>());
  EXPECT_EQ(cnstr2.variables().rows(), 4);
  X_bounds.clear();
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < X.cols(); j++) {
      X_bounds.emplace(X(i, j), std::make_pair(X_lo(i, j), X_up(i, j)));
    }
  }
  for (int i = 0; i < cnstr2.variables().rows(); ++i) {
    const auto it = X_bounds.find(cnstr2.variables()(i));
    EXPECT_NE(it, X_bounds.end());
    EXPECT_EQ(it->second.first, cnstr2.evaluator()->lower_bound()(i));
    EXPECT_EQ(it->second.second, cnstr2.evaluator()->upper_bound()(i));
  }
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

GTEST_TEST(TestMathematicalProgram, AddCostTest) {
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

  shared_ptr<Cost> generic_trivial_cost1 =
      make_shared<GenericTrivialCost1>(true);

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

class EmptyConstraint final : public Constraint {
 public:
  // This evaluator is thread safe in general. However, for the sake of testing
  // we allow the constructor argument which changes the value of
  // is_thread_safe.
  explicit EmptyConstraint(bool is_thread_safe = true)
      : Constraint(0, 2, Eigen::VectorXd(0), Eigen::VectorXd(0),
                   "empty_constraint") {
    set_is_thread_safe(is_thread_safe);
  }

  ~EmptyConstraint() {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>&, VectorXd*) const {}
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>&, AutoDiffVecXd*) const {}
  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const {}
};

GTEST_TEST(TestMathematicalProgram, AddEmptyConstraint) {
  // MathematicalProgram::AddFooConstraint with empty constraint.
  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto binding1 = prog.AddConstraint(std::make_shared<EmptyConstraint>(), x);
  EXPECT_TRUE(prog.GetAllConstraints().empty());
  EXPECT_EQ(binding1.evaluator()->get_description(), "empty_constraint");

  auto binding2 = prog.AddConstraint(
      std::make_shared<LinearConstraint>(
          Eigen::MatrixXd(0, 2), Eigen::VectorXd(0), Eigen::VectorXd(0)),
      x);
  EXPECT_TRUE(prog.GetAllConstraints().empty());
  EXPECT_EQ(binding2.evaluator()->num_constraints(), 0);

  auto binding3 =
      prog.AddConstraint(std::make_shared<LinearEqualityConstraint>(
                             Eigen::MatrixXd(0, 2), Eigen::VectorXd(0)),
                         x);
  EXPECT_TRUE(prog.GetAllConstraints().empty());
  EXPECT_EQ(binding3.evaluator()->num_constraints(), 0);

  auto binding4 =
      prog.AddConstraint(std::make_shared<BoundingBoxConstraint>(
                             Eigen::VectorXd(0), Eigen::VectorXd(0)),
                         VectorX<symbolic::Variable>(0));
  EXPECT_TRUE(prog.GetAllConstraints().empty());
  EXPECT_EQ(binding4.evaluator()->num_constraints(), 0);

  auto binding5 =
      prog.AddConstraint(std::make_shared<LinearComplementarityConstraint>(
                             Eigen::MatrixXd(0, 0), Eigen::VectorXd(0)),
                         VectorX<symbolic::Variable>(0));
  EXPECT_TRUE(prog.GetAllConstraints().empty());

  auto binding6 = prog.AddConstraint(internal::CreateBinding(
      std::make_shared<PositiveSemidefiniteConstraint>(0),
      VectorXDecisionVariable(0)));
  EXPECT_TRUE(prog.GetAllConstraints().empty());

  auto binding7 =
      prog.AddConstraint(std::make_shared<LinearMatrixInequalityConstraint>(
                             std::vector<Eigen::MatrixXd>(
                                 {Eigen::MatrixXd(0, 0), Eigen::MatrixXd(0, 0),
                                  Eigen::MatrixXd(0, 0)})),
                         x);
  EXPECT_TRUE(prog.GetAllConstraints().empty());
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

GTEST_TEST(TestMathematicalProgram, AddLinearCostSymbolic) {
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

GTEST_TEST(TestMathematicalProgram,
           AddLinearConstraintMatrixVectorVariablesTest) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(4, "x");
  Matrix4d A;
  // clang-format off
  A << 1, 2, 3, 4,
       5, 6, 7, 8,
       -9, -10, -11, -12,
       13, 14, 15, 16;
  // clang-format on
  const Vector4d lb{-10, -11, -12, -13};
  const Vector4d ub{3, 7, 11, 17};
  const auto binding = prog.AddLinearConstraint(A, lb, ub, x);

  // Check if the binding includes the correct linear constraint.
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.evaluator();
  EXPECT_EQ(constraint_ptr->num_constraints(), 4);

  const MatrixX<Expression> Ax{(constraint_ptr->GetDenseA() * var_vec)};
  const VectorX<Expression> lb_in_constraint{constraint_ptr->lower_bound()};
  const VectorX<Expression> ub_in_constraint{constraint_ptr->upper_bound()};

  EXPECT_TRUE(MatrixExprAllEqual(A * x - lb, Ax - lb_in_constraint));
  EXPECT_TRUE(MatrixExprAllEqual(A * x - ub, Ax - ub_in_constraint));
}

GTEST_TEST(TestMathematicalProgram,
           AddLinearConstraintMatrixVariableRefListTest) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(1, "x");
  auto y = prog.NewContinuousVariables(3, "y");
  VariableRefList vars{x, y};
  VectorXDecisionVariable vars_as_vec{ConcatenateVariableRefList(vars)};

  Matrix4d A;
  // clang-format off
  A << 1, 2, 3, 4,
       5, 6, 7, 8,
       -9, -10, -11, -12,
       13, 14, 15, 16;
  // clang-format on
  const Vector4d lb{-10, -11, -12, -13};
  const Vector4d ub{3, 7, 11, 17};
  const auto binding = prog.AddLinearConstraint(A, lb, ub, vars);

  // Check if the binding includes the correct linear constraint.
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.evaluator();
  EXPECT_EQ(constraint_ptr->num_constraints(), 4);

  const MatrixX<Expression> Ax{(constraint_ptr->GetDenseA() * var_vec)};
  const VectorX<Expression> lb_in_constraint{constraint_ptr->lower_bound()};
  const VectorX<Expression> ub_in_constraint{constraint_ptr->upper_bound()};

  EXPECT_TRUE(MatrixExprAllEqual(A * vars_as_vec - lb, Ax - lb_in_constraint));
  EXPECT_TRUE(MatrixExprAllEqual(A * vars_as_vec - ub, Ax - ub_in_constraint));
}

GTEST_TEST(TestMathematicalProgram,
           AddLinearConstraintSparseMatrixVectorVariablesTest) {
  MathematicalProgram prog;
  const int n = 4;
  auto x = prog.NewContinuousVariables(n, "x");
  std::vector<Eigen::Triplet<double>> A_triplet_list;
  A_triplet_list.reserve(3 * (n - 2) + 4);
  double count = 1;
  for (int i = 0; i < n; ++i) {
    for (int j = std::max(0, i - 1); j < std::min(n, i + 1); ++j) {
      A_triplet_list.emplace_back(i, j, count);
      ++count;
    }
  }

  Eigen::SparseMatrix<double> A(n, n);
  A.setFromTriplets(A_triplet_list.begin(), A_triplet_list.end());
  const Vector4d lb{-10, -11, -12, -13};
  const Vector4d ub{3, 7, 11, 17};
  const auto binding = prog.AddLinearConstraint(A, lb, ub, x);

  // Check if the binding called the sparse matrix constructor by checking that
  // a dense A has not been computed yet.
  EXPECT_FALSE(binding.evaluator()->is_dense_A_constructed());

  // Check if the binding includes the correct linear constraint.
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.evaluator();
  EXPECT_EQ(constraint_ptr->num_constraints(), 4);

  const MatrixX<Expression> Ax{(constraint_ptr->GetDenseA() * var_vec)};
  const VectorX<Expression> lb_in_constraint{constraint_ptr->lower_bound()};
  const VectorX<Expression> ub_in_constraint{constraint_ptr->upper_bound()};

  EXPECT_TRUE(MatrixExprAllEqual((A * x - lb), Ax - lb_in_constraint));
  EXPECT_TRUE(MatrixExprAllEqual((A * x - ub), Ax - ub_in_constraint));
}

GTEST_TEST(TestMathematicalProgram,
           AddLinearConstraintSparseMatrixVariableRefListTest) {
  MathematicalProgram prog;
  const int n = 4;
  auto x = prog.NewContinuousVariables(1, "x");
  auto y = prog.NewContinuousVariables(n - 1, "y");
  VariableRefList vars{x, y};
  VectorXDecisionVariable vars_as_vec{ConcatenateVariableRefList(vars)};

  std::vector<Eigen::Triplet<double>> A_triplet_list;
  A_triplet_list.reserve(3 * (n - 2) + 4);
  double count = 1;
  for (int i = 0; i < n; ++i) {
    for (int j = std::max(0, i - 1); j < std::min(n, i + 1); ++j) {
      A_triplet_list.emplace_back(i, j, count);
      ++count;
    }
  }

  Eigen::SparseMatrix<double> A(n, n);
  A.setFromTriplets(A_triplet_list.begin(), A_triplet_list.end());
  const Vector4d lb{-10, -11, -12, -13};
  const Vector4d ub{3, 7, 11, 17};
  const auto binding = prog.AddLinearConstraint(A, lb, ub, vars);

  // Check if the binding called the sparse matrix constructor by checking that
  // a dense A has not been computed yet.
  EXPECT_FALSE(binding.evaluator()->is_dense_A_constructed());

  // Check if the binding includes the correct linear constraint.
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.evaluator();
  EXPECT_EQ(constraint_ptr->num_constraints(), 4);

  const MatrixX<Expression> Ax{(constraint_ptr->GetDenseA() * var_vec)};
  const VectorX<Expression> lb_in_constraint{constraint_ptr->lower_bound()};
  const VectorX<Expression> ub_in_constraint{constraint_ptr->upper_bound()};

  EXPECT_TRUE(
      MatrixExprAllEqual((A * vars_as_vec - lb), Ax - lb_in_constraint));
  EXPECT_TRUE(
      MatrixExprAllEqual((A * vars_as_vec - ub), Ax - ub_in_constraint));
}

GTEST_TEST(TestMathematicalProgram,
           AddLinearConstraintRowVectorVectorVariablesTest) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(4, "x");
  const Eigen::RowVector4d a{1, 2, 3, 4};
  const double lb = -10;
  const double ub = 3;
  const auto binding = prog.AddLinearConstraint(a, lb, ub, x);

  // Check if the binding includes the correct linear constraint.
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.evaluator();
  EXPECT_EQ(constraint_ptr->num_constraints(), 1);

  const MatrixX<Expression> ax{(constraint_ptr->GetDenseA() * var_vec)};
  const VectorX<Expression> lb_in_constraint{constraint_ptr->lower_bound()};
  const VectorX<Expression> ub_in_constraint{constraint_ptr->upper_bound()};

  EXPECT_TRUE((a * x - lb).EqualTo((ax - lb_in_constraint)(0)));
  EXPECT_TRUE((a * x - ub).EqualTo((ax - ub_in_constraint)(0)));
}

GTEST_TEST(TestMathematicalProgram,
           AddLinearConstraintRowVectorVariableRefListTest) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(1, "x");
  auto y = prog.NewContinuousVariables(3, "y");
  VariableRefList vars{x, y};
  VectorXDecisionVariable vars_as_vec{ConcatenateVariableRefList(vars)};

  const Eigen::RowVector4d a{1, 2, 3, 4};
  const double lb = -10;
  const double ub = 3;
  const auto binding = prog.AddLinearConstraint(a, lb, ub, vars);

  // Check if the binding includes the correct linear constraint.
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.evaluator();
  EXPECT_EQ(constraint_ptr->num_constraints(), 1);

  const MatrixX<Expression> ax{(constraint_ptr->GetDenseA() * var_vec)};
  const VectorX<Expression> lb_in_constraint{constraint_ptr->lower_bound()};
  const VectorX<Expression> ub_in_constraint{constraint_ptr->upper_bound()};
  EXPECT_TRUE((a * vars_as_vec - lb).EqualTo((ax - lb_in_constraint)(0)));
  EXPECT_TRUE((a * vars_as_vec - ub).EqualTo((ax - ub_in_constraint)(0)));
}

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolic1) {
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
  const Expression Ax{(constraint_ptr->GetDenseA() * var_vec)(0, 0)};
  const Expression lb_in_constraint{constraint_ptr->lower_bound()[0]};
  const Expression ub_in_constraint{constraint_ptr->upper_bound()[0]};
  EXPECT_TRUE((e - lb).EqualTo(Ax - lb_in_constraint));
  EXPECT_TRUE((e - ub).EqualTo(Ax - ub_in_constraint));
}

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolic2) {
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
  const Expression Ax{(constraint_ptr->GetDenseA() * var_vec)(0, 0)};
  const Expression lb_in_constraint{constraint_ptr->lower_bound()[0]};
  const Expression ub_in_constraint{constraint_ptr->upper_bound()[0]};
  EXPECT_TRUE((e - -10).EqualTo(Ax - lb_in_constraint));
  EXPECT_TRUE((e - 10).EqualTo(Ax - ub_in_constraint));
}

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolic3) {
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
  M_ub << 9,
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
  const auto Ax = constraint_ptr->GetDenseA() * var_vec;
  const auto lb_in_constraint = constraint_ptr->lower_bound();
  const auto ub_in_constraint = constraint_ptr->upper_bound();

  for (int i = 0; i < M_e.size(); ++i) {
    EXPECT_PRED2(ExprEqual, M_e(i) - M_lb(i), Ax(i) - lb_in_constraint(i));
    EXPECT_PRED2(ExprEqual, M_e(i) - M_ub(i), Ax(i) - ub_in_constraint(i));
  }
}

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolic4) {
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
  EXPECT_TRUE(CompareMatrices(binding.evaluator()->lower_bound(), Vector1d(1)));
  EXPECT_TRUE(CompareMatrices(binding.evaluator()->upper_bound(), Vector1d(2)));
}

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolic5) {
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

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolic6) {
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

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolic7) {
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

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolic8) {
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

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolic9) {
  // Test trivial constraint with no variables, such as 1 <= 2 <= 3
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  prog.AddLinearConstraint(Expression(2), 1, 3);
  EXPECT_EQ(prog.linear_constraints().size(), 1);
  auto binding = prog.linear_constraints().back();
  EXPECT_EQ(binding.evaluator()->GetDenseA().rows(), 1);
  EXPECT_EQ(binding.evaluator()->GetDenseA().cols(), 0);

  Vector2<Expression> expr;
  expr << 2, x(0);
  prog.AddLinearConstraint(expr, Vector2d(1, 2), Vector2d(3, 4));
  EXPECT_EQ(prog.linear_constraints().size(), 2);
  binding = prog.linear_constraints().back();
  EXPECT_TRUE(
      CompareMatrices(binding.evaluator()->GetDenseA(), Eigen::Vector2d(0, 1)));
  EXPECT_TRUE(CompareMatrices(binding.evaluator()->lower_bound(),
                              Eigen::Vector2d(-1, 2)));
  EXPECT_TRUE(CompareMatrices(binding.evaluator()->upper_bound(),
                              Eigen::Vector2d(1, 4)));
}

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolicFormula1) {
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

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolicFormula2) {
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

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolicFormula3) {
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

    VectorX<Expression> expr =
        binding.evaluator()->GetDenseA() * binding.variables() -
        binding.evaluator()->lower_bound();
    EXPECT_EQ(expr.size(), 1);
    EXPECT_PRED2(ExprEqual, expr(0), x(0) + x(2) - 1);
  }
}

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolicFormula4) {
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
        binding.evaluator()->GetDenseA() * binding.variables();
    EXPECT_EQ(expr.size(), 1);
    EXPECT_PRED2(ExprEqual, expr(0), 1 - x(0) - 2 * x(2));
  }
}

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolicFormula5) {
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
        binding.evaluator()->GetDenseA() * binding.variables();
    EXPECT_EQ(binding.evaluator()->lower_bound()(0), -inf);
    EXPECT_EQ(binding.evaluator()->upper_bound()(0), inf);
    EXPECT_PRED2(ExprEqual, expr(0), x(0));
  }
}

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolicFormula6) {
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
        binding.evaluator()->GetDenseA() * binding.variables();
    EXPECT_EQ(binding.evaluator()->lower_bound()(0), -inf);
    EXPECT_EQ(binding.evaluator()->upper_bound()(0), inf);
    EXPECT_PRED2(ExprEqual, expr(0), get_lhs_expression(f_i));
  }
}

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolicFormula7) {
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
        binding.evaluator()->GetDenseA() * binding.variables();
    EXPECT_EQ(binding.evaluator()->lower_bound()(0), -inf);
    EXPECT_EQ(binding.evaluator()->upper_bound()(0), inf);
    EXPECT_PRED2(ExprEqual, expr(0), -get_lhs_expression(f_i));
  }
}

GTEST_TEST(TestMathematicalProgram,
           AddLinearConstraintSymbolicFormulaException1) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(prog.AddLinearConstraint(x(0) + inf <= x(1)), runtime_error);
  EXPECT_THROW(prog.AddLinearConstraint(x(0) - inf <= x(1)), runtime_error);
  EXPECT_THROW(prog.AddLinearConstraint(x(0) <= x(1) + inf), runtime_error);
  EXPECT_THROW(prog.AddLinearConstraint(x(0) <= x(1) - inf), runtime_error);
}

GTEST_TEST(TestMathematicalProgram,
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

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolicFormulaAnd1) {
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
  const auto Ax = constraint_ptr->GetDenseA() * var_vec;
  const auto lb_in_constraint = constraint_ptr->lower_bound();
  const auto ub_in_constraint = constraint_ptr->upper_bound();

  set<Expression> constraint_set;
  constraint_set.emplace(x(0) + 2 * x(1) - 5);
  constraint_set.emplace(3 * x(0) + 4 * x(1) - 6);
  EXPECT_TRUE(constraint_set.contains(Ax(0) - lb_in_constraint(0)));
  EXPECT_TRUE(constraint_set.contains(Ax(0) - ub_in_constraint(0)));
  EXPECT_TRUE(constraint_set.contains(Ax(1) - lb_in_constraint(1)));
  EXPECT_TRUE(constraint_set.contains(Ax(1) - ub_in_constraint(1)));
}

GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolicFormulaAnd2) {
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
  const auto Ax = constraint_ptr->GetDenseA() * var_vec;
  const auto lb_in_constraint = constraint_ptr->lower_bound();
  const auto ub_in_constraint = constraint_ptr->upper_bound();

  set<Expression> constraint_set;
  constraint_set.emplace(e11 - e12);
  constraint_set.emplace(e21 - e22);
  constraint_set.emplace(e31 - e32);
  for (int i = 0; i < 3; ++i) {
    if (!std::isinf(lb_in_constraint(i))) {
      // Either `Ax - lb` or `-(Ax - lb)` should be in the constraint set.
      EXPECT_NE(constraint_set.contains(Ax(i) - lb_in_constraint(i)),
                constraint_set.contains(-(Ax(i) - lb_in_constraint(i))));
    }
    if (!std::isinf(ub_in_constraint(i))) {
      // Either `Ax - ub` or `-(Ax - ub)` should be in the constraint set.
      EXPECT_NE(constraint_set.contains(Ax(i) - ub_in_constraint(i)),
                constraint_set.contains(-(Ax(i) - ub_in_constraint(i))));
    }
  }
}

GTEST_TEST(TestMathematicalProgram,
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
GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolicArrayFormula1) {
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
  const auto Ax = constraint_ptr->GetDenseA() * var_vec;
  const auto lb_in_constraint = constraint_ptr->lower_bound();
  const auto ub_in_constraint = constraint_ptr->upper_bound();
  for (int i{0}; i < M_f.size(); ++i) {
    if (!std::isinf(lb_in_constraint(i))) {
      EXPECT_PRED2(ExprEqual,
                   get_lhs_expression(M_f(i)) - get_rhs_expression(M_f(i)),
                   Ax(i) - lb_in_constraint(i));
    }
    if (!std::isinf(ub_in_constraint(i))) {
      EXPECT_PRED2(ExprEqual,
                   get_lhs_expression(M_f(i)) - get_rhs_expression(M_f(i)),
                   Ax(i) - ub_in_constraint(i));
    }
  }
}

// Checks AddLinearConstraint function which takes an Eigen::Array<Formula>.
// This test uses operator>= provided for Eigen::Array<Expression> and
// Eigen::Array<double>.
GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolicArrayFormula2) {
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
  const auto Ax = constraint_ptr->GetDenseA() * var_vec;
  const auto lb_in_constraint = constraint_ptr->lower_bound();
  const auto ub_in_constraint = constraint_ptr->upper_bound();
  int k{0};
  for (int j{0}; j < M_e.cols(); ++j) {
    for (int i{0}; i < M_e.rows(); ++i) {
      EXPECT_PRED2(ExprEqual, M_e(i, j) - M_lb(i, j),
                   Ax(k) - lb_in_constraint(k));
      ++k;
    }
  }
}

// Checks AddLinearConstraint function which takes an Eigen::Array<Formula>.
GTEST_TEST(TestMathematicalProgram, AddLinearConstraintSymbolicArrayFormula3) {
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
  const auto Ax = constraint_ptr->GetDenseA() * var_vec;
  const auto lb_in_constraint = constraint_ptr->lower_bound();
  const auto ub_in_constraint = constraint_ptr->upper_bound();
  EXPECT_PRED2(ExprEqual, x(0) + 2 * x(1) - 5, Ax(0) - lb_in_constraint(0));
  EXPECT_PRED2(ExprEqual, 3 * x(0) + 4 * x(1) - 6, Ax(1) - lb_in_constraint(1));
}

// Checks AddLinearConstraint function which takes an Eigen::Array<Formula>.
GTEST_TEST(TestMathematicalProgram,
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
  VectorX<Expression> flat_V =
      binding.evaluator()->GetDenseA() * binding.variables() -
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
  VectorX<Expression> flat_V =
      binding.evaluator()->GetDenseA() * binding.variables() -
      binding.evaluator()->lower_bound();
  EXPECT_EQ(math::ToSymmetricMatrixFromLowerTriangularColumns(flat_V), v - b);
}

void CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
    MathematicalProgram* prog, const Expression& e, double b) {
  CheckAddedNonSymmetricSymbolicLinearEqualityConstraint(
      prog, Vector1<Expression>(e), Vector1d(b));
}
}  // namespace

GTEST_TEST(TestMathematicalProgram,
           AddLinearEqualityConstraintMatrixVectorVariablesTest) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(4, "x");
  Matrix4d A;
  // clang-format off
  A << 1, 2, 3, 4,
       5, 6, 7, 8,
       -9, -10, -11, -12,
       13, 14, 15, 16;
  // clang-format on
  const Vector4d b{-10, -11, -12, -13};
  const auto binding = prog.AddLinearEqualityConstraint(A, b, x);
  CheckAddedLinearEqualityConstraintCommon(binding, prog, 0);
}

GTEST_TEST(TestMathematicalProgram,
           AddLinearEqualityConstraintMatrixVariableRefListTest) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(1, "x");
  auto y = prog.NewContinuousVariables(3, "y");
  VariableRefList vars{x, y};
  Matrix4d A;
  // clang-format off
  A << 1, 2, 3, 4,
       5, 6, 7, 8,
       -9, -10, -11, -12,
       13, 14, 15, 16;
  // clang-format on
  const Vector4d b{-10, -11, -12, -13};
  const auto binding = prog.AddLinearEqualityConstraint(A, b, vars);
  CheckAddedLinearEqualityConstraintCommon(binding, prog, 0);
}

GTEST_TEST(TestMathematicalProgram,
           AddLinearEqualityConstraintSparseMatrixVariableRefListTest) {
  MathematicalProgram prog;
  const int n = 4;
  auto x = prog.NewContinuousVariables(1, "x");
  auto y = prog.NewContinuousVariables(3, "y");
  VariableRefList vars{x, y};

  std::vector<Eigen::Triplet<double>> A_triplet_list;
  A_triplet_list.reserve(3 * (n - 2) + 4);
  double count = 1;
  for (int i = 0; i < n; ++i) {
    for (int j = std::max(0, i - 1); j < std::min(n, i + 1); ++j) {
      A_triplet_list.emplace_back(i, j, count);
      ++count;
    }
  }

  Eigen::SparseMatrix<double> A(n, n);
  A.setFromTriplets(A_triplet_list.begin(), A_triplet_list.end());
  const Vector4d b{-10, -11, -12, -13};

  const auto binding = prog.AddLinearEqualityConstraint(A, b, vars);

  // Check if the binding called the sparse matrix constructor by checking that
  // a dense A has not been computed yet.
  EXPECT_FALSE(binding.evaluator()->is_dense_A_constructed());

  CheckAddedLinearEqualityConstraintCommon(binding, prog, 0);
}

GTEST_TEST(TestMathematicalProgram,
           AddLinearEqualityConstraintSparseMatrixVectorVariablesTest) {
  MathematicalProgram prog;
  const int n = 4;
  auto x = prog.NewContinuousVariables(n, "x");
  std::vector<Eigen::Triplet<double>> A_triplet_list;
  A_triplet_list.reserve(3 * (n - 2) + 4);
  double count = 1;
  for (int i = 0; i < n; ++i) {
    for (int j = std::max(0, i - 1); j < std::min(n, i + 1); ++j) {
      A_triplet_list.emplace_back(i, j, count);
      ++count;
    }
  }

  Eigen::SparseMatrix<double> A(n, n);
  A.setFromTriplets(A_triplet_list.begin(), A_triplet_list.end());
  const Vector4d b{-10, -11, -12, -13};

  const auto binding = prog.AddLinearEqualityConstraint(A, b, x);

  // Check if the binding called the sparse matrix constructor by checking that
  // a dense A has not been computed yet.
  EXPECT_FALSE(binding.evaluator()->is_dense_A_constructed());

  CheckAddedLinearEqualityConstraintCommon(binding, prog, 0);
}

GTEST_TEST(TestMathematicalProgram,
           AddLinearEqualityConstraintRowVectorVectorVariablesTest) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(4, "x");
  const Eigen::RowVector4d a{1, 2, 3, 4};
  const double b = -10;
  const auto binding = prog.AddLinearEqualityConstraint(a, b, x);
  CheckAddedLinearEqualityConstraintCommon(binding, prog, 0);
}

GTEST_TEST(TestMathematicalProgram,
           AddLinearEqualityConstraintRowVectorVariableRefListTest) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(1, "x");
  auto y = prog.NewContinuousVariables(3, "y");
  VariableRefList vars{x, y};
  VectorXDecisionVariable vars_as_vec{ConcatenateVariableRefList(vars)};

  const Eigen::RowVector4d a{1, 2, 3, 4};
  const double b = -10;
  const auto binding = prog.AddLinearEqualityConstraint(a, b, vars);
  CheckAddedLinearEqualityConstraintCommon(binding, prog, 0);
}

GTEST_TEST(TestMathematicalProgram, AddSymbolicLinearEqualityConstraint1) {
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

GTEST_TEST(TestMathematicalProgram, AddSymbolicLinearEqualityConstraint2) {
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

GTEST_TEST(TestMathematicalProgram, AddSymbolicLinearEqualityConstraint3) {
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

GTEST_TEST(TestMathematicalProgram, AddSymbolicLinearEqualityConstraint4) {
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
GTEST_TEST(TestMathematicalProgram, AddSymbolicLinearEqualityConstraint5) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  // f = (3x₀ + 2x₁ + 3 == 5).
  const Formula f{3 * x(0) + 2 * x(1) + 3 == 5};
  const Binding<LinearEqualityConstraint> binding{
      prog.AddLinearEqualityConstraint(f)};
  EXPECT_EQ(prog.linear_equality_constraints().size(), 1u);

  const Expression expr_in_added_constraint{
      (binding.evaluator()->GetDenseA() * binding.variables() -
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
// case where `f` is a conjunction of linear-equality formulas, and tests
// `AddLinearEqualityConstraint(const Eigen::Ref<const
// Eigen::Array<symbolic::Formula, Eigen::Dynamic, Eigen::Dynamic>>& formulas)`
// method with a case where `f` is a vector of linear-equality formulas, a
// vector of conjunctions of linear-equality formulas, or a mix of the two.
GTEST_TEST(TestMathematicalProgram, AddSymbolicLinearEqualityConstraint6) {
  // Test problem: Ax = b where
  //
  // A = |-3.0  0.0  2.0|  x = |x0|  b = | 9.0|
  //     | 0.0  7.0 -3.0|      |x1|      | 3.0|
  //     | 2.0  5.0  0.0|      |x2|      |-5.0|
  MathematicalProgram prog1;
  VectorX<Variable> x = prog1.NewContinuousVariables<3>("x");
  prog1.AddDecisionVariables(x);
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
  Binding<LinearEqualityConstraint> binding1{
      prog1.AddLinearEqualityConstraint(f)};

  const Eigen::Matrix<Expression, 3, 1> expected_exprs{A * x - b};
  auto check_binding =
      [&expected_exprs](const MathematicalProgram& test_prog,
                        const Binding<LinearEqualityConstraint>& test_binding) {
        EXPECT_EQ(test_prog.linear_equality_constraints().size(), 1u);

        // Checks if AddLinearEqualityConstraint added the constraint correctly.
        auto exprs_in_added_constraint = Eigen::Matrix<Expression, 3, 1>{
            test_binding.evaluator()->GetDenseA() * test_binding.variables() -
            test_binding.evaluator()->lower_bound()};

        // Since a conjunctive symbolic formula uses `std::set` as an internal
        // representation, we need to check if `exprs_in_added_constraint` is a
        // permutation of `expected_exprs`.
        EXPECT_TRUE(is_permutation(exprs_in_added_constraint.data(),
                                   exprs_in_added_constraint.data() + 3,
                                   expected_exprs.data(),
                                   expected_exprs.data() + 3, ExprEqual));
      };

  check_binding(prog1, binding1);

  // Now we test the vector of formulas bindings.
  std::vector<Binding<LinearEqualityConstraint>> bindings;

  // Test 1: each formula is its own entry.
  Eigen::Vector3<Formula> f1;
  for (int i = 0; i < 3; ++i) {
    f1[i] = A.row(i).dot(x) == b[i];
  }

  // Test 2: combine the first two formulas in a conjunction of equalities,
  // while leaving the third as an equality.
  Eigen::Vector2<Formula> f2;
  f2[0] = f1[0] && f1[1];
  f2[1] = f1[2];

  // Test 3: combine all three formulas in a conjunction of equalities.
  Vector1<Formula> f3;
  f3[0] = f1[0] && f1[1] && f1[2];

  std::vector<VectorX<Formula>> formulas{f1, f2, f3};

  for (const auto& test_formula : formulas) {
    MathematicalProgram prog2;
    prog2.AddDecisionVariables(x);
    Binding<LinearEqualityConstraint> binding2{
        prog2.AddLinearEqualityConstraint(test_formula)};
    check_binding(prog2, binding2);
  }
}

// Checks if `AddLinearEqualityConstraint(f)` throws std::runtime_error if `f`
// is a non-linear equality formula.
GTEST_TEST(TestMathematicalProgram,
           AddSymbolicLinearEqualityConstraintException1) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  // f = (3x₀² + 2x₁ + 3 == 5).
  const Formula f{3 * x(0) * x(0) + 2 * x(1) + 3 == 5};
  EXPECT_THROW(prog.AddLinearEqualityConstraint(f), runtime_error);
}

// Checks if `AddLinearEqualityConstraint(f)` throws std::runtime_error if a
// conjunctive formula `f` includes a relational formula other than equality.
GTEST_TEST(TestMathematicalProgram,
           AddSymbolicLinearEqualityConstraintException2) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  // f = (3x₀ + 2x₁ + 3 >= 5 && 7x₀ - 5x₁ == 0).
  const Formula f{3 * x(0) + 2 * x(1) + 3 >= 5 && 7 * x(0) - 5 * x(1) == 0};
  EXPECT_THROW(prog.AddLinearEqualityConstraint(f), runtime_error);
}

// Checks if `AddLinearEqualityConstraint(f)` throws std::runtime_error if `f`
// is neither a linear-equality formula nor a conjunctive formula.
GTEST_TEST(TestMathematicalProgram,
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
    MathematicalProgram* prog, const Formula& f,
    LorentzConeConstraint::EvalType eval_type) {
  const auto& binding1 = prog->AddLorentzConeConstraint(f, eval_type);
  EXPECT_EQ(binding1.evaluator()->eval_type(), eval_type);
  const auto& binding2 = prog->lorentz_cone_constraints().back();
  EXPECT_EQ(binding1.evaluator(), binding2.evaluator());
  EXPECT_EQ(binding1.variables(), binding2.variables());
  const Eigen::MatrixXd A = binding1.evaluator()->A();
  const Eigen::VectorXd b = binding1.evaluator()->b();
  const VectorX<Expression> z = A * binding1.variables() + b;
  Expression greater, lesser;
  if (is_greater_than_or_equal_to(f)) {
    greater = get_lhs_expression(f);
    lesser = get_rhs_expression(f);
  } else {
    ASSERT_TRUE(is_less_than_or_equal_to(f));
    greater = get_rhs_expression(f);
    lesser = get_lhs_expression(f);
  }
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      symbolic::Polynomial(z(0)), symbolic::Polynomial(greater), 1E-10));
  ASSERT_TRUE(is_sqrt(lesser));
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      symbolic::Polynomial(z.tail(z.rows() - 1).squaredNorm()),
      symbolic::Polynomial(get_argument(lesser)), 1E-10));
}

void CheckParsedSymbolicLorentzConeConstraint(
    MathematicalProgram* prog, const Expression& linear_expr,
    const Expression& quadratic_expr,
    LorentzConeConstraint::EvalType eval_type) {
  const auto& binding1 = prog->AddLorentzConeConstraint(
      linear_expr, quadratic_expr, 0., eval_type);
  EXPECT_EQ(binding1.evaluator()->eval_type(), eval_type);
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
  for (const auto eval_type : {LorentzConeConstraint::EvalType::kConvex,
                               LorentzConeConstraint::EvalType::kConvexSmooth,
                               LorentzConeConstraint::EvalType::kNonconvex}) {
    const auto& binding1 = prog->AddLorentzConeConstraint(e, eval_type);
    EXPECT_EQ(binding1.evaluator()->eval_type(), eval_type);
    const auto& binding2 = prog->lorentz_cone_constraints().back();

    EXPECT_EQ(binding1.evaluator(), binding2.evaluator());
    EXPECT_EQ(binding1.evaluator()->A() * binding1.variables() +
                  binding1.evaluator()->b(),
              e);
    EXPECT_EQ(binding2.evaluator()->A() * binding2.variables() +
                  binding2.evaluator()->b(),
              e);

    CheckParsedSymbolicLorentzConeConstraint(
        prog, e(0), e.tail(e.rows() - 1).squaredNorm(), eval_type);
  }
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

TEST_F(SymbolicLorentzConeTest, Formula) {
  CheckParsedSymbolicLorentzConeConstraint(
      &prog_, 2.0 * x_(2) >= x_.head<2>().cast<Expression>().norm(),
      LorentzConeConstraint::EvalType::kConvex);

  CheckParsedSymbolicLorentzConeConstraint(
      &prog_, x_.head<2>().cast<Expression>().norm() <= 2.0 * x_(2),
      LorentzConeConstraint::EvalType::kConvex);

  DRAKE_EXPECT_THROWS_MESSAGE(
      prog_.AddLorentzConeConstraint(
          2.0 * x_(2) * x_(1) >= x_.head<2>().cast<Expression>().norm(),
          LorentzConeConstraint::EvalType::kConvex),
      ".*non-linear.*");
}

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
  e << 2 * x_(0) + 3 * x_(2), -x_(0) + 2 * x_(2), +x_(2), -x_(1);
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
      &prog_, x_(0) + 2, pow(x_(0), 2) + 4 * x_(0) * x_(1) + 4 * pow(x_(1), 2),
      LorentzConeConstraint::EvalType::kConvex);
  CheckParsedSymbolicLorentzConeConstraint(
      &prog_, x_(0) + 2, pow(x_(0), 2) + 4 * x_(0) * x_(1) + 4 * pow(x_(1), 2),
      LorentzConeConstraint::EvalType::kConvexSmooth);
}

TEST_F(SymbolicLorentzConeTest, Test8) {
  CheckParsedSymbolicLorentzConeConstraint(
      &prog_, x_(0) + 2,
      pow(x_(0), 2) - (x_(0) - x_(1)) * (x_(0) + x_(1)) + 2 * x_(1) + 3,
      LorentzConeConstraint::EvalType::kConvex);
  CheckParsedSymbolicLorentzConeConstraint(
      &prog_, x_(0) + 2,
      pow(x_(0), 2) - (x_(0) - x_(1)) * (x_(0) + x_(1)) + 2 * x_(1) + 3,
      LorentzConeConstraint::EvalType::kConvexSmooth);
}

TEST_F(SymbolicLorentzConeTest, Test9) {
  CheckParsedSymbolicLorentzConeConstraint(
      &prog_, 2, pow(x_(0), 2) + pow(x_(1), 2),
      LorentzConeConstraint::EvalType::kConvex);
  CheckParsedSymbolicLorentzConeConstraint(
      &prog_, 2, pow(x_(0), 2) + pow(x_(1), 2),
      LorentzConeConstraint::EvalType::kConvexSmooth);
}

TEST_F(SymbolicLorentzConeTest, TestLinearConstraint) {
  // Actually adding linear constraint, that the quadratic expression is
  // actually a constant.
  CheckParsedSymbolicLorentzConeConstraint(
      &prog_, x_(0) + 2, 1, LorentzConeConstraint::EvalType::kConvexSmooth);
  CheckParsedSymbolicLorentzConeConstraint(
      &prog_, x_(0) + 2, x_(0) - 2 * (0.5 * x_(0) + 1) + 3,
      LorentzConeConstraint::EvalType::kConvexSmooth);
}

TEST_F(SymbolicLorentzConeTest, TestError) {
  // Check the cases to add with invalid quadratic expression.

  // Check polynomial with order no smaller than 2
  EXPECT_THROW(prog_.AddLorentzConeConstraint(2 * x_(0) + 3, pow(x_(1), 3)),
               runtime_error);

  // The quadratic expression is actually affine.
  EXPECT_THROW(prog_.AddLorentzConeConstraint(2 * x_(0), 3 * x_(1) + 2),
               runtime_error);
  EXPECT_THROW(
      prog_.AddLorentzConeConstraint(
          2 * x_(0), x_(1) * x_(1) - (x_(1) - x_(0)) * (x_(1) + x_(0)) -
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
  EXPECT_THROW(prog_.AddLorentzConeConstraint(
                   2 * x_(0) + 3, pow(x_(0), 2) - pow(x_(1), 2) -
                                      (x_(0) + x_(1)) * (x_(0) - x_(1)) - 1),
               runtime_error);

  // The first expression is not actually linear.
  EXPECT_THROW(prog_.AddLorentzConeConstraint(2 * x_(0) * x_(1), pow(x_(0), 2)),
               runtime_error);
}

GTEST_TEST(TestMathematicalProgram, AddSymbolicRotatedLorentzConeConstraint1) {
  // Add rotated Lorentz cone constraint:
  // x is in the rotated Lorentz cone constraint.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  Matrix<Expression, 3, 1> e;
  e << +x(0), +x(1), +x(2);
  CheckParsedSymbolicRotatedLorentzConeConstraint(&prog, e);
}

GTEST_TEST(TestMathematicalProgram, AddSymbolicRotatedLorentzConeConstraint2) {
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

GTEST_TEST(TestMathematicalProgram, AddSymbolicRotatedLorentzConeConstraint3) {
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

GTEST_TEST(TestMathematicalProgram, AddSymbolicRotatedLorentzConeConstraint4) {
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

GTEST_TEST(TestMathematicalProgram, AddSymbolicRotatedLorentzConeConstraint5) {
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
      binding.evaluator()->A() * binding.variables() + binding.evaluator()->b();
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

GTEST_TEST(TestMathematicalProgram,
           TestAddQuadraticAsRotatedLorentzConeConstraint) {
  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  auto check = [&prog, &x](const Eigen::Matrix2d& Q, const Eigen::Vector2d& b,
                           double c) {
    auto dut = prog.AddQuadraticAsRotatedLorentzConeConstraint(Q, b, c, x);
    auto z = dut.evaluator()->A() * x + dut.evaluator()->b();
    double tol = 1E-10;
    // Make sure that the rotated Lorentz cone constraint is the same as the
    // quadratic constraint expression.
    EXPECT_TRUE(symbolic::test::PolynomialEqual(
        symbolic::Polynomial(z.tail(z.rows() - 2).squaredNorm() - z(0) * z(1)),
        symbolic::Polynomial(0.5 * x.cast<symbolic::Expression>().dot(Q * x) +
                             b.dot(x) + c),
        tol));
    EXPECT_EQ(prog.rotated_lorentz_cone_constraints().back().evaluator().get(),
              dut.evaluator().get());
  };
  Eigen::Matrix2d Q;
  Q << 2, 1, 3, 2;
  Eigen::Vector2d b(2, 3);
  double c = -0.5;
  check(Q, b, c);
  Q << 2, 2, 2, 6;
  check(Q, b, c);
}

namespace {
template <typename Derived>
typename std::enable_if_t<is_same_v<typename Derived::Scalar, Expression>>
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
  EXPECT_EQ(binding.evaluator().get(),
            prog->positive_semidefinite_constraints().back().evaluator().get());
  // Check if the added linear constraint is correct. M is the newly added
  // variables representing the psd matrix.
  const Eigen::Map<const MatrixX<Variable>> M(&binding.variables()(0), V.rows(),
                                              V.cols());
  // The linear equality constraint is only imposed on the lower triangular
  // part of the psd matrix.
  const auto& new_lin_eq_cnstr = prog->linear_equality_constraints().back();
  auto V_minus_M = math::ToSymmetricMatrixFromLowerTriangularColumns(
      new_lin_eq_cnstr.evaluator()->GetDenseA() * new_lin_eq_cnstr.variables() -
      new_lin_eq_cnstr.evaluator()->lower_bound());
  EXPECT_EQ(V_minus_M, V - M);
}
}  // namespace

GTEST_TEST(TestMathematicalProgram, AddPositiveSemidefiniteConstraint) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<4>("X");

  auto psd_cnstr = prog.AddPositiveSemidefiniteConstraint(X).evaluator();
  EXPECT_EQ(prog.positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(prog.GetAllConstraints().size(), 1);
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

GTEST_TEST(TestMathematicalProgram,
           AddPrincipalSubmatrixIsPsdConstraintVariable) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<4>("X");

  std::set<int> minor_indices{0, 1, 3};
  MatrixXDecisionVariable minor_manual(3, 3);
  // clang-format off
  minor_manual << X(0, 0), X(0, 1), X(0, 3),
                  X(1, 0), X(1, 1), X(1, 3),
                  X(3, 0), X(3, 1), X(3, 3);
  // clang-format on

  auto psd_cnstr =
      prog.AddPrincipalSubmatrixIsPsdConstraint(X, minor_indices).evaluator();
  EXPECT_EQ(prog.positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(prog.GetAllConstraints().size(), 1);
  const auto& new_psd_cnstr = prog.positive_semidefinite_constraints().back();
  EXPECT_EQ(psd_cnstr.get(), new_psd_cnstr.evaluator().get());
  Eigen::Map<Eigen::Matrix<Variable, 9, 1>> minor_flat(&minor_manual(0, 0));
  EXPECT_TRUE(CheckStructuralEquality(minor_flat, new_psd_cnstr.variables()));
}

GTEST_TEST(TestMathematicalProgram,
           AddPrincipalSubmatrixIsPsdConstraintExpression) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<4>("X");
  std::set<int> minor_indices{0, 1, 3};
  MatrixXDecisionVariable minor_manual(3, 3);
  // clang-format off
  minor_manual << X(0, 0), X(0, 1), X(0, 3),
                  X(1, 0), X(1, 1), X(1, 3),
                  X(3, 0), X(3, 1), X(3, 3);
  // clang-format on

  auto lmi_cnstr = prog.AddPrincipalSubmatrixIsPsdConstraint(
                           2 * Matrix4d::Identity() * X, minor_indices)
                       .evaluator();

  EXPECT_EQ(prog.linear_matrix_inequality_constraints().size(), 1);
  EXPECT_EQ(prog.GetAllConstraints().size(), 1);

  const auto& new_lmi_cnstr =
      prog.linear_matrix_inequality_constraints().back();
  EXPECT_EQ(lmi_cnstr.get(), new_lmi_cnstr.evaluator().get());

  MathematicalProgram prog_manual;
  prog_manual.AddDecisionVariables(minor_manual);
  auto lmi_cnstr_manual = prog_manual
                              .AddLinearMatrixInequalityConstraint(
                                  2 * Matrix3d::Identity() * minor_manual)
                              .evaluator();
  const auto& new_lmi_cnstr_manual =
      prog.linear_matrix_inequality_constraints().back();

  EXPECT_TRUE(CheckStructuralEquality(new_lmi_cnstr_manual.variables(),
                                      new_lmi_cnstr.variables()));
  EXPECT_EQ(prog.linear_matrix_inequality_constraints().size(),
            prog_manual.linear_matrix_inequality_constraints().size());
  EXPECT_EQ(prog.linear_equality_constraints().size(),
            prog_manual.linear_equality_constraints().size());
  EXPECT_TRUE(prog.linear_equality_constraints().empty());
  EXPECT_EQ(prog.GetAllConstraints().size(),
            prog_manual.GetAllConstraints().size());
}

GTEST_TEST(TestMathematicalProgram, AddLinearMatrixInequalityConstraint) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  Matrix3<symbolic::Expression> X;
  // clang-format off
  X << 1,                   2 + x(0) - 3 * x(1), 0,
       2 + x(0) - 3 * x(1), 2 * x(2),           -3,
       0,                   -3,                  1 + 2 * x(1);
  // clang-format on
  const auto lmi_binding = prog.AddLinearMatrixInequalityConstraint(X);
  EXPECT_TRUE(prog.required_capabilities().contains(
      ProgramAttribute::kPositiveSemidefiniteConstraint));
  Matrix3<symbolic::Expression> X_expected = lmi_binding.evaluator()->F()[0];
  for (int i = 0; i < lmi_binding.variables().rows(); ++i) {
    X_expected +=
        lmi_binding.evaluator()->F()[1 + i] * lmi_binding.variables()(i);
  }
  EXPECT_TRUE(MatrixExprAllEqual(X, X_expected));
}

GTEST_TEST(TestMathematicalProgram, TestExponentialConeConstraint) {
  MathematicalProgram prog;
  EXPECT_EQ(prog.required_capabilities().count(
                ProgramAttribute::kExponentialConeConstraint),
            0);
  auto x = prog.NewContinuousVariables<4>();
  const Vector3<symbolic::Expression> expr(2 * x(0) + x(1) + 2, 1,
                                           -2 * x(0) + 3);
  auto binding = prog.AddExponentialConeConstraint(expr);
  EXPECT_GT(prog.required_capabilities().count(
                ProgramAttribute::kExponentialConeConstraint),
            0);
  EXPECT_EQ(prog.GetAllConstraints().size(), 1);
  const VectorX<symbolic::Expression> expr_reconstructed =
      binding.evaluator()->A() * binding.variables() + binding.evaluator()->b();
  EXPECT_EQ(expr_reconstructed.rows(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(ExprEqual, expr(i), expr_reconstructed(i));
  }
}

void CheckAddedQuadraticCost(MathematicalProgram* prog,
                             const Eigen::MatrixXd& Q, const Eigen::VectorXd& b,
                             const VectorXDecisionVariable& x,
                             std::optional<bool> is_convex = std::nullopt) {
  int num_quadratic_cost = prog->quadratic_costs().size();
  auto cnstr = prog->AddQuadraticCost(Q, b, x, is_convex).evaluator();

  EXPECT_EQ(++num_quadratic_cost, prog->quadratic_costs().size());
  // Check if the newly added quadratic constraint, and the returned
  // quadratic constraint, both match 0.5 * x' * Q * x + b' * x
  EXPECT_EQ(cnstr, prog->quadratic_costs().back().evaluator());
  EXPECT_EQ(cnstr->Q(), Q);
  EXPECT_EQ(cnstr->b(), b);
}

GTEST_TEST(TestMathematicalProgram, AddQuadraticCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  CheckAddedQuadraticCost(&prog, Matrix3d::Identity(), Vector3d::Zero(), x);

  CheckAddedQuadraticCost(&prog, Matrix3d::Identity(), Vector3d(1, 2, 3), x);

  CheckAddedQuadraticCost(&prog, Matrix3d::Identity(), Vector3d(1, 2, 3), x,
                          true);

  CheckAddedQuadraticCost(&prog, -Matrix3d::Identity(), Vector3d(1, 2, 3), x,
                          false);

  // Now check AddQuadraticCost(Q, b, x) without passing is_convex flag.
  auto cost1 =
      prog.AddQuadraticCost(Matrix3d::Identity(), Vector3d(1, 2, 3), x);
  EXPECT_TRUE(cost1.evaluator()->is_convex());
  EXPECT_TRUE(prog.quadratic_costs().back().evaluator()->is_convex());
  // Intentionlly pass the wrong is_convex flag. MathematicalProgram will
  // use this wrong flag in the added cost.
  cost1 =
      prog.AddQuadraticCost(Matrix3d::Identity(), Vector3d(1, 2, 3), x, false);
  EXPECT_FALSE(cost1.evaluator()->is_convex());

  auto cost2 =
      prog.AddQuadraticCost(-Matrix3d::Identity(), Vector3d(1, 2, 3), x);
  EXPECT_FALSE(cost2.evaluator()->is_convex());
  EXPECT_FALSE(prog.quadratic_costs().back().evaluator()->is_convex());

  // Test with x being a VariableRefList.
  auto cost3 = prog.AddQuadraticCost(Matrix3d::Identity(), Vector3d(1, 2, 3),
                                     {x.tail<2>(), x.head<1>()});
  EXPECT_TRUE(cost3.evaluator()->is_convex());
  VectorX<symbolic::Expression> cost_eval_sym;
  cost3.evaluator()->Eval(cost3.variables(), &cost_eval_sym);
  EXPECT_PRED2(ExprEqual, cost_eval_sym(0).Expand(),
               (0.5 * (x(0) * x(0) + x(1) * x(1) + x(2) * x(2)) + x(1) +
                2 * x(2) + 3 * x(0))
                   .Expand());
  auto cost4 =
      prog.AddQuadraticCost(Matrix3d(Vector3d(-1, -2, -3).asDiagonal()),
                            Vector3d(1, 2, 3), {x.tail<2>(), x.head<1>()});
  EXPECT_FALSE(cost4.evaluator()->is_convex());
  cost4.evaluator()->Eval(cost4.variables(), &cost_eval_sym);
  EXPECT_PRED2(ExprEqual, cost_eval_sym(0).Expand(),
               (-0.5 * (x(1) * x(1) + 2 * x(2) * x(2) + 3 * x(0) * x(0)) +
                x(1) + 2 * x(2) + 3 * x(0))
                   .Expand());
  // Intentionlly pass the wrong is_convex flag. MathematicalProgram will
  // use this wrong flag in the added cost.
  cost4 = prog.AddQuadraticCost(Matrix3d(Vector3d(-1, -2, -3).asDiagonal()),
                                Vector3d(1, 2, 3), {x.tail<2>(), x.head<1>()},
                                true);
  EXPECT_TRUE(cost4.evaluator()->is_convex());

  // Now check AddQuadraticCost(Q, b, c, x) without passing is_convex flag.
  auto cost5 = prog.AddQuadraticCost(Eigen::Matrix3d::Identity(),
                                     Vector3d(1, 2, 3), 1.5, x);
  EXPECT_TRUE(cost5.evaluator()->is_convex());
  EXPECT_EQ(cost5.evaluator()->c(), 1.5);

  auto cost6 = prog.AddQuadraticCost(-Eigen::Matrix3d::Identity(),
                                     Vector3d(1, 2, 3), 2.5, x);
  EXPECT_FALSE(cost6.evaluator()->is_convex());
  EXPECT_EQ(cost6.evaluator()->c(), 2.5);
  // Intentionlly pass the wrong is_convex flag. MathematicalProgram will
  // use this wrong flag in the added cost.
  cost6 = prog.AddQuadraticCost(-Eigen::Matrix3d::Identity(), Vector3d(1, 2, 3),
                                2.5, x, true);
  EXPECT_TRUE(cost6.evaluator()->is_convex());
}

void CheckAddedSymbolicQuadraticCostUserFun(const MathematicalProgram& prog,
                                            const Expression& e,
                                            const Binding<Cost>& binding,
                                            int num_quadratic_cost) {
  EXPECT_EQ(num_quadratic_cost, prog.quadratic_costs().size());
  EXPECT_EQ(binding.evaluator(), prog.quadratic_costs().back().evaluator());
  EXPECT_EQ(binding.variables(), prog.quadratic_costs().back().variables());

  auto cnstr = prog.quadratic_costs().back().evaluator();
  // Check the added cost is 0.5 * x' * Q * x + b' * x + c
  const auto& x_bound = binding.variables();
  const Expression e_added = 0.5 * x_bound.dot(cnstr->Q() * x_bound) +
                             cnstr->b().dot(x_bound) + cnstr->c();
  EXPECT_PRED2(ExprEqual, e_added.Expand(), e.Expand());
}

void CheckAddedSymbolicQuadraticCost(MathematicalProgram* prog,
                                     const Expression& e,
                                     bool is_convex_expected) {
  int num_quadratic_cost = prog->quadratic_costs().size();
  auto binding1 = prog->AddQuadraticCost(e);
  EXPECT_EQ(binding1.evaluator()->is_convex(), is_convex_expected);
  CheckAddedSymbolicQuadraticCostUserFun(*prog, e, binding1,
                                         ++num_quadratic_cost);
  auto binding2 = prog->AddCost(e);
  CheckAddedSymbolicQuadraticCostUserFun(*prog, e, binding2,
                                         ++num_quadratic_cost);
}

GTEST_TEST(TestMathematicalProgram, AddSymbolicQuadraticCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  // Identity diagonal term.
  Expression e1 = x.transpose() * x;
  CheckAddedSymbolicQuadraticCost(&prog, e1, true);
  EXPECT_TRUE(prog.quadratic_costs().back().evaluator()->is_convex());

  // Identity diagonal term.
  Expression e2 = x.transpose() * x + 1;
  CheckAddedSymbolicQuadraticCost(&prog, e2, true);
  EXPECT_TRUE(prog.quadratic_costs().back().evaluator()->is_convex());
  // Confirm that const terms are respected.
  auto b = prog.AddQuadraticCost(e2);
  VectorXd y(1);
  b.evaluator()->Eval(Vector3d::Zero(), &y);
  EXPECT_EQ(y[0], 1);

  // Identity diagonal term.
  Expression e3 = x(0) * x(0) + x(1) * x(1) + 2;
  CheckAddedSymbolicQuadraticCost(&prog, e3, true);
  EXPECT_TRUE(prog.quadratic_costs().back().evaluator()->is_convex());

  // Non-identity diagonal term.
  Expression e4 = x(0) * x(0) + 2 * x(1) * x(1) + 3 * x(2) * x(2) + 3;
  CheckAddedSymbolicQuadraticCost(&prog, e4, true);
  EXPECT_TRUE(prog.quadratic_costs().back().evaluator()->is_convex());

  // Cross terms.
  Expression e5 = x(0) * x(0) + 2 * x(1) * x(1) + 4 * x(0) * x(1) + 2;
  CheckAddedSymbolicQuadraticCost(&prog, e5, false);
  EXPECT_FALSE(prog.quadratic_costs().back().evaluator()->is_convex());

  // Linear terms.
  Expression e6 = x(0) * x(0) + 2 * x(1) * x(1) + 4 * x(0);
  CheckAddedSymbolicQuadraticCost(&prog, e6, true);

  // Cross terms and linear terms.
  Expression e7 = (x(0) + 2 * x(1) + 3) * (x(0) + x(1) + 4) + 3 * x(0) * x(0) +
                  6 * pow(x(1) + 1, 2);
  CheckAddedSymbolicQuadraticCost(&prog, e7, true);

  // Cubic polynomial case.
  Expression e8 = pow(x(0), 3) + 1;
  EXPECT_THROW(prog.AddQuadraticCost(e8), runtime_error);

  // Call AddQuadraticCost with user-specified is_convex flag.
  const Expression e9 = x(0) * x(0) + 2 * x(1);
  auto cost9 = prog.AddQuadraticCost(e9, true);
  EXPECT_TRUE(cost9.evaluator()->is_convex());
  // We lie about the convexity of this cost, that we set is_convex=false. The
  // returned cost should also report is_convex=false.
  cost9 = prog.AddQuadraticCost(e9, false);
  EXPECT_FALSE(cost9.evaluator()->is_convex());

  const Expression e10 = -x(0) * x(0) + x(1) * x(1);
  auto cost10 = prog.AddQuadraticCost(e10, false);
  EXPECT_FALSE(cost10.evaluator()->is_convex());
}

GTEST_TEST(TestMathematicalProgram, Test2NormSquaredCost) {
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
  auto obj2 = prog.Add2NormSquaredCost(A, b, x).evaluator();

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

GTEST_TEST(TestMathematicalProgram, AddQuadraticErrorCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  const Eigen::Vector2d x_desired(1.24, 2.3);
  const double w1 = 3.4;
  auto obj1 = prog.AddQuadraticErrorCost(w1, x_desired, x);
  const double w2 = -1.23;
  auto obj2 = prog.AddQuadraticErrorCost(w2, x_desired, x);
  const Eigen::Vector2d x_test(7.6, 8.7);
  Eigen::VectorXd y(1);
  obj1.evaluator()->Eval(x_test, &y);
  const double tol = 1e-13;
  EXPECT_NEAR(y[0], w1 * (x_test - x_desired).squaredNorm(), tol);
  obj2.evaluator()->Eval(x_test, &y);
  EXPECT_NEAR(y[0], w2 * (x_test - x_desired).squaredNorm(), tol);
}

GTEST_TEST(TestMathematicalProgram, AddL2NormCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  Eigen::Matrix2d A;
  A << 1, 2, 3, 4;
  Eigen::Vector2d b(2, 3);

  auto obj1 =
      prog.AddCost(Binding<L2NormCost>(std::make_shared<L2NormCost>(A, b), x));
  EXPECT_TRUE(
      prog.required_capabilities().contains(ProgramAttribute::kL2NormCost));
  EXPECT_EQ(prog.l2norm_costs().size(), 1u);
  EXPECT_EQ(prog.GetAllCosts().size(), 1u);

  auto obj2 = prog.AddL2NormCost(A, b, x);
  EXPECT_EQ(prog.l2norm_costs().size(), 2u);

  symbolic::Expression e = (A * x + b).norm();
  auto obj3 = prog.AddL2NormCost(e, 1e-8, 1e-8);
  EXPECT_EQ(prog.l2norm_costs().size(), 3u);

  // Test that the AddCost method correctly recognizes the L2norm.
  auto obj4 = prog.AddCost(e);
  EXPECT_EQ(prog.l2norm_costs().size(), 4u);

  prog.RemoveCost(obj1);
  prog.RemoveCost(obj2);
  prog.RemoveCost(obj3);
  prog.RemoveCost(obj4);
  EXPECT_EQ(prog.l2norm_costs().size(), 0u);
  EXPECT_FALSE(
      prog.required_capabilities().contains(ProgramAttribute::kL2NormCost));

  prog.AddL2NormCost(A, b, {x.head<1>(), x.tail<1>()});
  EXPECT_EQ(prog.l2norm_costs().size(), 1u);
  EXPECT_TRUE(
      prog.required_capabilities().contains(ProgramAttribute::kL2NormCost));

  auto new_prog = prog.Clone();
  EXPECT_EQ(new_prog->l2norm_costs().size(), 1u);

  // AddL2NormCost(Expression) can throw.
  e = (A * x + b).squaredNorm();
  DRAKE_EXPECT_THROWS_MESSAGE(prog.AddL2NormCost(e), ".*is not an L2 norm.*");
}

GTEST_TEST(TestMathematicalProgram, AddQuadraticConstraint) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  auto constraint1 = prog.AddConstraint(Binding<QuadraticConstraint>(
      std::make_shared<QuadraticConstraint>(Eigen::Matrix2d::Identity(),
                                            Eigen::Vector2d::Ones(), 0, 1),
      x.head<2>()));
  EXPECT_GT(prog.required_capabilities().count(
                ProgramAttribute::kQuadraticConstraint),
            0);
  ASSERT_EQ(prog.quadratic_constraints().size(), 1);
  EXPECT_EQ(prog.quadratic_constraints()[0].evaluator().get(),
            constraint1.evaluator().get());
  EXPECT_EQ(prog.quadratic_constraints()[0].variables(), x.head<2>());

  auto constraint2 = prog.AddQuadraticConstraint(
      -Eigen::Matrix2d::Identity(), Eigen::Vector2d::Ones(), 0, 1, x.tail<2>(),
      QuadraticConstraint::HessianType::kNegativeSemidefinite);
  EXPECT_GT(prog.required_capabilities().count(
                ProgramAttribute::kQuadraticConstraint),
            0);
  EXPECT_EQ(prog.quadratic_constraints().size(), 2);

  auto constraint3 =
      prog.AddQuadraticConstraint(x(0) * x(0) - x(1) * x(1), 0, 1);
  EXPECT_GT(prog.required_capabilities().count(
                ProgramAttribute::kQuadraticConstraint),
            0);
  EXPECT_EQ(prog.quadratic_constraints().size(), 3);
  EXPECT_EQ(constraint3.evaluator()->hessian_type(),
            QuadraticConstraint::HessianType::kIndefinite);

  prog.RemoveConstraint(constraint2);
  EXPECT_GT(prog.required_capabilities().count(
                ProgramAttribute::kQuadraticConstraint),
            0);
  EXPECT_EQ(prog.quadratic_constraints().size(), 2);
  prog.RemoveConstraint(constraint1);
  prog.RemoveConstraint(constraint3);
  EXPECT_TRUE(prog.quadratic_constraints().empty());
  EXPECT_EQ(prog.required_capabilities().count(
                ProgramAttribute::kQuadraticConstraint),
            0);
}

GTEST_TEST(TestMathematicalProgram, AddL2NormCostUsingConicConstraint) {
  MathematicalProgram prog{};
  auto x = prog.NewContinuousVariables<2>();
  Eigen::Matrix2d A;
  A << 1, 2, 3, 4;
  const Eigen::Vector2d b(2, 3);
  const auto ret = prog.AddL2NormCostUsingConicConstraint(A, b, x);
  const symbolic::Variable& s = std::get<0>(ret);
  const Binding<LinearCost>& linear_cost = std::get<1>(ret);
  const Binding<LorentzConeConstraint>& lorentz_cone_constraint =
      std::get<2>(ret);
  EXPECT_GE(prog.FindDecisionVariableIndex(s), 0);
  EXPECT_EQ(linear_cost.variables().rows(), 1);
  EXPECT_EQ(linear_cost.variables(), Vector1<symbolic::Variable>(s));
  EXPECT_EQ(linear_cost.evaluator()->a(), Vector1d(1));
  EXPECT_EQ(linear_cost.evaluator()->b(), 0);
  EXPECT_EQ(prog.linear_costs().size(), 1u);
  EXPECT_EQ(prog.lorentz_cone_constraints().size(), 1u);
  EXPECT_EQ(lorentz_cone_constraint.variables().rows(), 3);
  EXPECT_EQ(lorentz_cone_constraint.variables(),
            Vector3<symbolic::Variable>(s, x(0), x(1)));
  Vector3<symbolic::Expression> lorentz_eval_expected;
  lorentz_eval_expected << s, A * x + b;
  EXPECT_EQ(lorentz_cone_constraint.evaluator()->A() *
                    lorentz_cone_constraint.variables() +
                lorentz_cone_constraint.evaluator()->b(),
            lorentz_eval_expected);
}

GTEST_TEST(TestMathematicalProgram, AddL1NormCostInEpigraphForm_Rectangular) {
  MathematicalProgram prog{};
  auto x = prog.NewContinuousVariables<2>("x");

  // Rectangular A: 3 rows, 2 columns.
  Eigen::Matrix<double, 3, 2> A;
  A << 1, 0, 0, 1, 1, 1;
  const Eigen::Vector3d b(1, -2, -1);  // So that A * [1, -2] + b = 0.

  const auto [s, linear_cost, linear_constraint] =
      prog.AddL1NormCostInEpigraphForm(A, b, x);

  // Check that s was added as new decision variables.
  EXPECT_EQ(s.rows(), 3);
  for (int i = 0; i < s.size(); ++i) {
    ASSERT_NO_THROW(void(prog.FindDecisionVariableIndex(s(i))));
  }

  // Check that the linear cost is Σᵢsᵢ.
  EXPECT_TRUE(linear_cost.evaluator());
  EXPECT_EQ(linear_cost.variables(), s);
  EXPECT_TRUE((linear_cost.evaluator()->a().array() == 1.0).all());
  EXPECT_EQ(linear_cost.evaluator()->b(), 0.0);
  EXPECT_EQ(prog.linear_costs().size(), 1);

  // Check that the constraint s >= Ax + b and s >= -(Ax + b) is encoded
  // correctly.
  EXPECT_TRUE(linear_constraint.evaluator() != nullptr);
  EXPECT_EQ(prog.linear_constraints().size(), 1);
  EXPECT_EQ(linear_constraint.evaluator()->GetDenseA().rows(),
            6);  // 2 * A.rows().
  EXPECT_EQ(linear_constraint.evaluator()->GetDenseA().cols(),
            5);  // s.size() + x.size().

  // Check constraint variable ordering: [s0, s1, s2, x0, x1].
  const auto& constraint_vars = linear_constraint.variables();
  EXPECT_EQ(constraint_vars.size(), 5);
  EXPECT_EQ(constraint_vars.segment(0, 3), s);
  EXPECT_EQ(constraint_vars.segment(3, 2), x);

  // s should satisfy:
  // s ≥  A * x + b
  // s ≥ -A * x - b.
  const Eigen::Vector2d x_value(-1.0, 2.0);
  const Eigen::Vector3d s_value = Eigen::Vector3d::Zero();

  // Form the full variable vector [s; x].
  Eigen::Matrix<double, 5, 1> vars;
  vars << s_value, x_value;

  const double kTol = 1e-15;
  EXPECT_TRUE(linear_constraint.evaluator()->CheckSatisfied(vars, kTol));

  // Verify it still works if s is larger.
  vars[0] += 1.0;
  EXPECT_TRUE(linear_constraint.evaluator()->CheckSatisfied(vars, kTol));
  vars[0] -= 1.0;

  // Check cost
  const double expected_cost = 0.0;
  Eigen::VectorXd actual_cost(1);
  linear_cost.evaluator()->Eval(s_value, &actual_cost);
  ASSERT_EQ(actual_cost.size(), 1);
  EXPECT_EQ(actual_cost[0], expected_cost);

  // Check failure case.
  vars[0] -= 2 * kTol;  // Decrease s[0].
  EXPECT_FALSE(linear_constraint.evaluator()->CheckSatisfied(vars, kTol));
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
  for (const pair<const Variable::Id, Variable>& p : var_id_to_var) {
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
  for (const pair<const symbolic::Monomial, symbolic::Expression>& p :
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

GTEST_TEST(TestMathematicalProgram, TestAddPolynomialCost) {
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

GTEST_TEST(TestMathematicalProgram, TestAddCostThrowError) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  // Add a cost containing variable not included in the mathematical program.
  Variable y("y");
  DRAKE_EXPECT_THROWS_MESSAGE(prog.AddCost(x(0) + y),
                              ".*is not a decision variable.*");
  DRAKE_EXPECT_THROWS_MESSAGE(prog.AddCost(x(0) * x(0) + y),
                              ".*is not a decision variable.*");
}

GTEST_TEST(TestMathematicalProgram, TestAddGenericCost) {
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

GTEST_TEST(TestMathematicalProgram, TestAddGenericCostExpression) {
  MathematicalProgram prog;
  Variable x = prog.NewContinuousVariables<1>()[0];
  Variable y = prog.NewContinuousVariables<1>()[0];

  using std::atan2;
  auto b = prog.AddCost(atan2(y, x));
  EXPECT_EQ(prog.generic_costs().size(), 1);
  Vector2d x_test(0.5, 0.2);
  Vector1d y_expected(atan2(0.2, 0.5));
  EXPECT_TRUE(CompareMatrices(prog.EvalBinding(b, x_test), y_expected));
}

// Confirm that even constant costs are supported.
GTEST_TEST(TestMathematicalProgram, TestAddCostConstant) {
  MathematicalProgram prog;

  auto b = prog.AddCost(Expression(0.5));
  EXPECT_EQ(prog.linear_costs().size(), 1);
}

GTEST_TEST(TestMathematicalProgram, TestClone) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  auto y = prog.NewIndeterminates<2>("y");
  auto X = prog.NewSymmetricContinuousVariables<3>("X");

  // Add costs
  shared_ptr<Cost> generic_trivial_cost1 =
      make_shared<GenericTrivialCost1>(true);
  prog.AddCost(Binding<Cost>(generic_trivial_cost1,
                             VectorDecisionVariable<3>(x(0), x(1), x(2))));
  GenericTrivialCost2 generic_trivial_cost2;
  prog.AddCost(generic_trivial_cost2, VectorDecisionVariable<2>(x(2), x(1)));
  prog.AddLinearCost(x(0) + 2);
  prog.AddQuadraticCost(x(0) * x(0) + 2 * x(1) * x(1), true);
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
      Vector3<symbolic::Expression>(+x(0), +x(1), x(2) - 0.5 * x(1)),
      LorentzConeConstraint::EvalType::kConvexSmooth);
  prog.AddLorentzConeConstraint(
      Vector3<symbolic::Expression>(x(0) + x(1), +x(0), x(2) - x(1)),
      LorentzConeConstraint::EvalType::kConvexSmooth);
  prog.AddRotatedLorentzConeConstraint(Vector4<symbolic::Expression>(
      +x(0), +x(1), 0.5 * (x(0) + x(1)), 0.5 * x(2)));
  prog.AddRotatedLorentzConeConstraint(
      Vector4<symbolic::Expression>(x(0) + x(1), x(1) + x(2), +x(0), +x(1)));
  prog.AddPositiveSemidefiniteConstraint(X);
  prog.AddLinearMatrixInequalityConstraint(X - Eigen::Matrix3d::Ones());
  int num_all_constraints = prog.GetAllConstraints().size();
  prog.AddLinearMatrixInequalityConstraint(
      {Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Ones(),
       2 * Eigen::Matrix2d::Ones()},
      x.head<2>());
  EXPECT_EQ(prog.GetAllConstraints().size(), num_all_constraints + 1);
  num_all_constraints++;
  prog.AddLinearComplementarityConstraint(Eigen::Matrix2d::Identity(),
                                          Eigen::Vector2d::Ones(), x.head<2>());
  EXPECT_EQ(prog.GetAllConstraints().size(), num_all_constraints + 1);
  num_all_constraints++;
  prog.AddLinearComplementarityConstraint(2 * Eigen::Matrix2d::Identity(),
                                          Eigen::Vector2d::Ones(), x.tail<2>());
  EXPECT_EQ(prog.GetAllConstraints().size(), num_all_constraints + 1);
  num_all_constraints++;
  prog.AddExponentialConeConstraint(Eigen::Matrix3d::Identity().sparseView(),
                                    Eigen::Vector3d::Ones(), x.head<3>());
  EXPECT_EQ(prog.GetAllConstraints().size(), num_all_constraints + 1);
  num_all_constraints++;

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
  EXPECT_TRUE(IsVectorOfBindingEqual(prog.exponential_cone_constraints(),
                                     new_prog->exponential_cone_constraints()));
  EXPECT_TRUE(
      IsVectorOfBindingEqual(prog.linear_complementarity_constraints(),
                             new_prog->linear_complementarity_constraints()));

  EXPECT_TRUE(CompareMatrices(new_prog->initial_guess(), prog.initial_guess()));
}

GTEST_TEST(TestMathematicalProgram, TestEvalBinding) {
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

  EXPECT_TRUE(
      CompareMatrices(prog.EvalBindings(prog.GetAllConstraints(), x_val),
                      Vector2d(30, 5), 1E-15, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(prog.EvalBindings(prog.GetAllCosts(), x_val),
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

GTEST_TEST(TestMathematicalProgram, TestGetBindingVariableValues) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();
  auto binding1 = prog.AddBoundingBoxConstraint(-1, 1, x(0));

  auto binding2 = prog.AddLinearEqualityConstraint(x(0) + 2 * x(2), 2);

  const Eigen::Vector3d x_val(-2, 1, 2);
  EXPECT_TRUE(CompareMatrices(prog.GetBindingVariableValues(binding1, x_val),
                              Vector1d(-2)));
  EXPECT_TRUE(CompareMatrices(prog.GetBindingVariableValues(binding2, x_val),
                              Vector2d(-2, 2)));
}

GTEST_TEST(TestMathematicalProgram, TestCheckSatisfied) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();
  const auto y = prog.NewContinuousVariables<2>();
  std::vector<Binding<Constraint>> bindings;
  bindings.emplace_back(prog.AddBoundingBoxConstraint(-.3, .4, x));
  bindings.emplace_back(prog.AddBoundingBoxConstraint(-2, 5, y));
  bindings.emplace_back(
      prog.AddLinearEqualityConstraint(y[0] == 3 * x[0] + 2 * x[1]));

  const double tol = std::numeric_limits<double>::epsilon();
  Vector3d x_guess = Vector3d::Constant(.39);
  Vector2d y_guess = Vector2d::Constant(4.99);
  y_guess[0] = 3 * x_guess[0] + 2 * x_guess[1];
  prog.SetInitialGuess(x, x_guess);
  prog.SetInitialGuess(y, y_guess);
  EXPECT_TRUE(prog.CheckSatisfied(bindings[0], prog.initial_guess(), 0));
  EXPECT_TRUE(prog.CheckSatisfied(bindings[1], prog.initial_guess(), 0));
  EXPECT_TRUE(prog.CheckSatisfied(bindings[2], prog.initial_guess(), tol));

  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(bindings[0], 0));
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(bindings[1], 0));
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(bindings[2], tol));

  EXPECT_TRUE(prog.CheckSatisfied(bindings, prog.initial_guess(), tol));
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(bindings, tol));

  x_guess[2] = .41;
  prog.SetInitialGuess(x, x_guess);
  EXPECT_FALSE(prog.CheckSatisfied(bindings[0], prog.initial_guess(), 0));
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(bindings[0], 0));
  EXPECT_FALSE(prog.CheckSatisfied(bindings, prog.initial_guess(), tol));
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(bindings, tol));
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(bindings[1], 0));

  x_guess[2] = .39;
  y_guess[0] = 3 * x_guess[0] + 2 * x_guess[1] + 0.2;
  prog.SetInitialGuess(x, x_guess);
  prog.SetInitialGuess(y, y_guess);
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(bindings[0], 0));
  EXPECT_FALSE(prog.CheckSatisfied(bindings[2], prog.initial_guess(), tol));
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(bindings[2], tol));
  EXPECT_FALSE(prog.CheckSatisfied(bindings, prog.initial_guess(), tol));
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(bindings, tol));
}

GTEST_TEST(TestMathematicalProgram, TestSetAndGetInitialGuess) {
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
  EXPECT_THROW(unused(prog.GetInitialGuess(y)), std::runtime_error);

  // Try the same things with an extrinsic guess.
  VectorXd guess = VectorXd::Constant(3, kNaN);
  prog.SetDecisionVariableValueInVector(x(2), 2, &guess);
  EXPECT_TRUE(std::isnan(guess[0]));
  EXPECT_EQ(guess[2], 2.0);
  prog.SetDecisionVariableValueInVector(x.head<2>(), Eigen::Vector2d(0.0, 1.0),
                                        &guess);
  EXPECT_EQ(guess[0], 0.0);
  EXPECT_EQ(guess[1], 1.0);
  EXPECT_EQ(guess[2], 2.0);
  EXPECT_THROW(prog.SetDecisionVariableValueInVector(y, 0.0, &guess),
               std::exception);
}

GTEST_TEST(TestMathematicalProgram, TestNonlinearExpressionConstraints) {
  // min ∑ x , subject to x'x = 1.
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();

  prog.AddConstraint(x.transpose() * x == 1.);

  if (SnoptSolver().available() && SnoptSolver().enabled()) {
    // Add equivalent constraints using all of the other entry points.
    // Note: restricted to SNOPT because IPOPT complains about the redundant
    // constraints.
    prog.AddConstraint(x.transpose() * x >= 1.);
    prog.AddConstraint(x.transpose() * x <= 1.);
    prog.AddConstraint((x.transpose() * x)(0), 1., 1.);
    prog.AddConstraint(x.transpose() * x, Vector1d{1.}, Vector1d{1.});
  }

  prog.AddCost(x(0) + x(1));
  const MathematicalProgramResult result =
      Solve(prog, Eigen::Vector2d(-0.5, -0.5));
  EXPECT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(result.get_x_val(),
                              Vector2d::Constant(-std::sqrt(2.) / 2.), 1e-6));
}

GTEST_TEST(TestMathematicalProgram, TestAddVisualizationCallback) {
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
      math::InitializeAutoDiff(VectorXd{test_x});
  VectorX<AutoDiffXd> test_y_autodiff(0);
  was_called = false;
  b.evaluator()->Eval(test_x_autodiff, &test_y_autodiff);
  EXPECT_TRUE(was_called);
}

GTEST_TEST(TestMathematicalProgram, TestSolverOptions) {
  MathematicalProgram prog;
  const SolverId solver_id("solver_id");

  // Set each type once, directly on the program.
  prog.SetSolverOption(solver_id, "double_name", 1.0);
  EXPECT_THAT(prog.solver_options().options.at("solver_id").at("double_name"),
              testing::VariantWith<double>(1.0));
  prog.SetSolverOption(solver_id, "int_name", 2);
  EXPECT_THAT(prog.solver_options().options.at("solver_id").at("int_name"),
              testing::VariantWith<int>(2));
  prog.SetSolverOption(solver_id, "string_name", "3");
  EXPECT_THAT(prog.solver_options().options.at("solver_id").at("string_name"),
              testing::VariantWith<std::string>("3"));

  // The only solver with options set is the "solver_id".
  EXPECT_EQ(prog.solver_options().options.size(), 1);

  // Set each type once on an options struct, then set that onto the program.
  // It erases all of the prior options.
  const SolverId dummy_id("dummy_id");
  SolverOptions dummy_options;
  dummy_options.SetOption(dummy_id, "double_name", 10.0);
  dummy_options.SetOption(dummy_id, "int_name", 20);
  dummy_options.SetOption(dummy_id, "string_name", "30.0");
  prog.SetSolverOptions(dummy_options);
  EXPECT_THAT(prog.solver_options().options.at("dummy_id").at("double_name"),
              testing::VariantWith<double>(10.0));
  EXPECT_THAT(prog.solver_options().options.at("dummy_id").at("int_name"),
              testing::VariantWith<int>(20));
  EXPECT_THAT(prog.solver_options().options.at("dummy_id").at("string_name"),
              testing::VariantWith<std::string>("30.0"));

  // The only solver with options set is the "dummy_id".
  EXPECT_EQ(prog.solver_options().options.size(), 1);
}

void CheckNewSosPolynomial(MathematicalProgram::NonnegativePolynomial type) {
  // Check if the newly created nonnegative polynomial can be computed as m' * Q
  // * m.
  MathematicalProgram prog;
  auto t = prog.NewIndeterminates<4>();
  const auto m = symbolic::MonomialBasis<4, 2>(symbolic::Variables(t));
  const auto pair = prog.NewSosPolynomial(m, type, "TestGram");
  const symbolic::Polynomial& p = pair.first;
  const MatrixXDecisionVariable& Q = pair.second;
  EXPECT_NE(Q(0, 0).get_name().find("TestGram"), std::string::npos);
  MatrixX<symbolic::Polynomial> Q_poly(m.rows(), m.rows());
  const symbolic::Monomial monomial_one{};
  for (int i = 0; i < Q_poly.rows(); ++i) {
    for (int j = 0; j < Q_poly.cols(); ++j) {
      Q_poly(i, j) =
          symbolic::Polynomial({{monomial_one, Q(j * Q_poly.rows() + i)}});
    }
  }
  const symbolic::Polynomial p_expected(m.dot(Q_poly * m));
  EXPECT_TRUE(p.EqualTo(p_expected));

  const auto p2 = prog.NewSosPolynomial(Q, m, type);
  EXPECT_TRUE(p2.EqualTo(p_expected));
}

GTEST_TEST(TestMathematicalProgram, NewSosPolynomial) {
  CheckNewSosPolynomial(MathematicalProgram::NonnegativePolynomial::kSos);
  CheckNewSosPolynomial(MathematicalProgram::NonnegativePolynomial::kSdsos);
  CheckNewSosPolynomial(MathematicalProgram::NonnegativePolynomial::kDsos);

  // Pass an uninitialized type to NewSosPolynomial method.
  EXPECT_THROW(CheckNewSosPolynomial(
                   static_cast<MathematicalProgram::NonnegativePolynomial>(0)),
               std::runtime_error);

  // Check NewSosPolynomial with degree = 0
  for (const auto type : {MathematicalProgram::NonnegativePolynomial::kSos,
                          MathematicalProgram::NonnegativePolynomial::kSdsos,
                          MathematicalProgram::NonnegativePolynomial::kDsos}) {
    solvers::MathematicalProgram prog;
    const auto x = prog.NewIndeterminates<2>();
    symbolic::Polynomial p;
    MatrixXDecisionVariable gram;
    std::tie(p, gram) = prog.NewSosPolynomial(symbolic::Variables(x), 0, type);
    EXPECT_EQ(prog.bounding_box_constraints().size(), 1u);
    EXPECT_TRUE(CompareMatrices(
        prog.bounding_box_constraints()[0].evaluator()->lower_bound(),
        Vector1d::Constant(0)));
    EXPECT_TRUE(CompareMatrices(
        prog.bounding_box_constraints()[0].evaluator()->upper_bound(),
        Vector1d::Constant(kInf)));
    EXPECT_EQ(prog.bounding_box_constraints()[0].variables(), gram);
    EXPECT_EQ(p.TotalDegree(), 0);
    EXPECT_EQ(p.monomial_to_coefficient_map().size(), 1u);
    EXPECT_EQ(gram.rows(), 1);
    EXPECT_EQ(gram.cols(), 1);
    EXPECT_EQ(p.monomial_to_coefficient_map().at(symbolic::Monomial()),
              gram(0, 0));
  }
}

void CheckNewEvenDegreeNonnegativePolynomial(
    MathematicalProgram::NonnegativePolynomial type) {
  // Check if the newly created nonnegative polynomial can be computed as m_e' *
  // Q_ee * m_e + m_o' * Q_oo * m_o * m.
  MathematicalProgram prog;
  auto t = prog.NewIndeterminates<2>();
  const symbolic::Variables t_vars(t);
  const int degree{4};
  const auto m_e = symbolic::EvenDegreeMonomialBasis(t_vars, degree / 2);
  const auto m_o = symbolic::OddDegreeMonomialBasis(t_vars, degree / 2);
  symbolic::Polynomial p;
  MatrixXDecisionVariable Q_oo, Q_ee;
  std::tie(p, Q_oo, Q_ee) =
      prog.NewEvenDegreeNonnegativePolynomial(t_vars, degree, type);
  symbolic::Polynomial p_expected{};
  for (int i = 0; i < Q_ee.rows(); ++i) {
    for (int j = 0; j < Q_ee.cols(); ++j) {
      p_expected += m_e(i) * Q_ee(i, j) * m_e(j);
    }
  }
  for (int i = 0; i < Q_oo.rows(); ++i) {
    for (int j = 0; j < Q_oo.cols(); ++j) {
      p_expected += m_o(i) * Q_oo(i, j) * m_o(j);
    }
  }
  EXPECT_PRED2(PolyEqual, p, p_expected);
  EXPECT_EQ(p.TotalDegree(), degree);
  if (type == MathematicalProgram::NonnegativePolynomial::kSos) {
    EXPECT_EQ(prog.positive_semidefinite_constraints().size(), 2);
  }
}

GTEST_TEST(TestMathematicalProgram, NewEvenDegreeNonnegativePolynomial) {
  CheckNewEvenDegreeNonnegativePolynomial(
      MathematicalProgram::NonnegativePolynomial::kSos);
  CheckNewEvenDegreeNonnegativePolynomial(
      MathematicalProgram::NonnegativePolynomial::kSdsos);
  CheckNewEvenDegreeNonnegativePolynomial(
      MathematicalProgram::NonnegativePolynomial::kDsos);
}

GTEST_TEST(TestMathematicalProgram, AddEqualityConstraintBetweenPolynomials) {
  MathematicalProgram prog;
  auto x = prog.NewIndeterminates<1>()(0);
  auto a = prog.NewContinuousVariables<4>();
  const symbolic::Polynomial p1(a(0) * x + a(1) + 2, {x});
  const symbolic::Polynomial p2((a(2) + 1) * x + 2 * a(3), {x});

  EXPECT_EQ(prog.linear_equality_constraints().size(), 0);
  const auto bindings = prog.AddEqualityConstraintBetweenPolynomials(p1, p2);
  EXPECT_EQ(prog.linear_equality_constraints().size(), 2);
  EXPECT_EQ(bindings.size(), 2u);
  for (int i = 0; i < 2; ++i) {
    EXPECT_EQ(bindings[i], prog.linear_equality_constraints()[i]);
  }

  // Test with different value of a, some satisfies the polynomial equality
  // constraints.
  auto is_satisfied = [&prog](const Eigen::Vector4d& a_val) {
    for (const auto& linear_eq_constraint :
         prog.linear_equality_constraints()) {
      const auto constraint_val = prog.EvalBinding(linear_eq_constraint, a_val);
      EXPECT_EQ(constraint_val.size(), 1);
      if (std::abs(constraint_val(0) -
                   linear_eq_constraint.evaluator()->lower_bound()(0)) >
          1E-12) {
        return false;
      }
    }
    return true;
  };

  EXPECT_TRUE(is_satisfied(Eigen::Vector4d(1, 2, 0, 2)));
  EXPECT_FALSE(is_satisfied(Eigen::Vector4d(1, 2, 0, 1)));
  EXPECT_FALSE(is_satisfied(Eigen::Vector4d(1, 2, 1, 2)));

  // Test with a polynomial whose coefficients are not affine function of
  // decision variables.
  EXPECT_THROW(prog.AddEqualityConstraintBetweenPolynomials(
                   p1, symbolic::Polynomial(a(0) * a(1) * x, {x})),
               std::runtime_error);
  // Test with a polynomial whose coefficients depend on variables that are not
  // decision variables of prog.
  symbolic::Variable b("b");
  DRAKE_EXPECT_THROWS_MESSAGE(prog.AddEqualityConstraintBetweenPolynomials(
                                  p1, symbolic::Polynomial(b * x, {x})),
                              ".*is not a decision variable.*");
  // If we add `b` to prog as decision variable, then the code throws no
  // exceptions.
  prog.AddDecisionVariables(Vector1<symbolic::Variable>(b));
  DRAKE_EXPECT_NO_THROW(prog.AddEqualityConstraintBetweenPolynomials(
      p1, symbolic::Polynomial(b * x, {x})));
}

GTEST_TEST(TestMathematicalProgram, TestVariableScaling) {
  MathematicalProgram prog;
  EXPECT_EQ(prog.GetVariableScaling().size(), 0);

  auto x = prog.NewContinuousVariables<4>();

  prog.SetVariableScaling(x(0), 1.0);
  prog.SetVariableScaling(x(1), 1.15);
  prog.SetVariableScaling(x(2), 1.15);
  prog.SetVariableScaling(x(3), 1.3);
  EXPECT_EQ(prog.GetVariableScaling().at(0), 1.0);
  EXPECT_EQ(prog.GetVariableScaling().at(1), 1.15);
  EXPECT_EQ(prog.GetVariableScaling().at(2), 1.15);
  EXPECT_EQ(prog.GetVariableScaling().at(3), 1.3);
  EXPECT_EQ(prog.GetVariableScaling().size(), 4);

  prog.SetVariableScaling(x(3), 3.0);
  EXPECT_EQ(prog.GetVariableScaling().at(0), 1.0);
  EXPECT_EQ(prog.GetVariableScaling().at(1), 1.15);
  EXPECT_EQ(prog.GetVariableScaling().at(2), 1.15);
  EXPECT_EQ(prog.GetVariableScaling().at(3), 3.0);
  EXPECT_EQ(prog.GetVariableScaling().size(), 4);
}

GTEST_TEST(TestMathematicalProgram, AddConstraintMatrix1) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  Eigen::Matrix<symbolic::Formula, 2, 2> formulas;
  // clang-format off
  formulas << (x(0) >= 0.0), (x(0) + x(1) <= 3.0),
              (x(1) == 2.0), (x(0) + x(1) >= -1.0);
  // clang-format on
  prog.AddConstraint(formulas);

  ASSERT_EQ(prog.GetAllConstraints().size(), 1);
  ASSERT_EQ(prog.GetAllLinearConstraints().size(), 1);
  const auto binding = prog.GetAllLinearConstraints()[0];

  Eigen::Matrix<double, 4, 2> A_expected;
  Eigen::Matrix<double, 4, 1> lower_bound_expected;
  Eigen::Matrix<double, 4, 1> upper_bound_expected;
  const double inf{numeric_limits<double>::infinity()};
  // clang-format off
  A_expected << 1, 0,           // x0 >= 0
                0, 1,           //      x1 == 2
                1, 1,           // x0 + x1 <= 3
                1, 1;           // x0 + x1 >= -1.0

  lower_bound_expected << 0,    // x0 >= 0
                          2,    //      x1 == 2
                       -inf,    // x0 + x1 <= 3
                         -1;    // x0 + x1 >= -1.0

  upper_bound_expected << inf,  // x0 >= 0
                            2,  //      x1 == 2
                            3,  // x0 + x1 <= 3
                          inf;  // x0 + x1 >= -1.0
  // clang-format on

  ASSERT_TRUE(binding.evaluator());
  EXPECT_EQ(binding.evaluator()->GetDenseA(), A_expected);
  EXPECT_EQ(binding.evaluator()->lower_bound(), lower_bound_expected);
  EXPECT_EQ(binding.evaluator()->upper_bound(), upper_bound_expected);
}

GTEST_TEST(TestMathematicalProgram, AddConstraintMatrix2) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  Eigen::Matrix<symbolic::Expression, 2, 2> exprs;
  // clang-format off
  exprs << x(0), x(0) + 2*x(1),
           x(1), x(0) + x(1);
  // clang-format on
  // This test relies on the pair (lb(i), ub(i)) being unique.
  Eigen::Matrix2d lb;
  lb << 0, -kInf, 2., -1.;
  Eigen::Matrix<double, 2, 2, Eigen::RowMajor> ub;
  ub(0, 0) = kInf;
  ub(0, 1) = 3;
  ub(1, 0) = 2;
  ub(1, 1) = kInf;
  prog.AddConstraint(exprs, lb, ub);

  ASSERT_EQ(prog.GetAllConstraints().size(), 1);
  ASSERT_EQ(prog.GetAllLinearConstraints().size(), 1);

  std::array<std::array<Eigen::RowVector2d, 2>, 2> coeff;
  coeff[0][0] << 1, 0;
  coeff[0][1] << 1, 2;
  coeff[1][0] << 0, 1;
  coeff[1][1] << 1, 1;

  auto check_binding = [&lb, &ub,
                        &coeff](const Binding<LinearConstraint>& binding) {
    EXPECT_EQ(binding.evaluator()->num_constraints(), 4);
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
        bool find_match = false;
        for (int k = 0; k < 4; ++k) {
          if (binding.evaluator()->lower_bound()(k) == lb(i, j) &&
              binding.evaluator()->upper_bound()(k) == ub(i, j)) {
            EXPECT_TRUE(CompareMatrices(binding.evaluator()->GetDenseA().row(k),
                                        coeff[i][j]));
            find_match = true;
          }
        }
        EXPECT_TRUE(find_match);
      }
    }
  };
  const auto binding1 = prog.GetAllLinearConstraints()[0];
  check_binding(binding1);

  const auto binding2 = prog.AddLinearConstraint(exprs, lb, ub);
  check_binding(binding2);
}

GTEST_TEST(TestMathematicalProgram, ReparsePolynomial) {
  MathematicalProgram prog;
  EXPECT_EQ(prog.GetVariableScaling().size(), 0);

  const auto a = prog.NewContinuousVariables<2>("a");
  const auto x = prog.NewIndeterminates<2>("x");

  // Variable not declared in this MathematicalProgram.
  const auto b = Variable{"b"};

  // a₀x₀ + a₁x₀ + x₁ + a₀ + b
  const Expression e = a(0) * x(0) + a(1) * x(0) + x(1) + a(0) + b;

  // (a₀ + a₁)x₀ + x₁ + (a₀ + b).
  const symbolic::Polynomial expected{prog.MakePolynomial(e)};

  {
    // (a₀x₀ + a₁x₀ + x₁ + a₀ + b , {x₀, x₁, a₀, a₁, b}).
    symbolic::Polynomial p{e};
    EXPECT_PRED2(PolyNotEqual, p, expected);
    prog.Reparse(&p);
    EXPECT_PRED2(PolyEqual, p, expected);
  }

  {
    // (a₀x₀ + a₁x₀ + x₁ + a₀ + b , {x₀, x₁}).
    symbolic::Polynomial p{e, {x(0), x(1)}};
    EXPECT_PRED2(PolyEqual, p, expected);  // Note that p == expected already.
    prog.Reparse(&p);
    EXPECT_PRED2(PolyEqual, p, expected);
  }

  {
    // (a₀x₀ + a₁x₀ + x₁ + a₀ + b , {x₀, a₀, b}).
    symbolic::Polynomial p{e, {x(0), a(0), b}};
    EXPECT_PRED2(PolyNotEqual, p, expected);
    prog.Reparse(&p);
    EXPECT_PRED2(PolyEqual, p, expected);
  }

  {
    // (a₀x₀ + a₁x₀ + x₁ + a₀ + b , {a₀, a₁}.
    symbolic::Polynomial p{e, {a(0), a(1)}};
    EXPECT_PRED2(PolyNotEqual, p, expected);
    prog.Reparse(&p);
    EXPECT_PRED2(PolyEqual, p, expected);
  }

  {
    // (a₀x₀ + a₁x₀ + x₁ + a₀ + b , {a₀, a₁, b}.
    symbolic::Polynomial p{e, {a(0), a(1), b}};
    EXPECT_PRED2(PolyNotEqual, p, expected);
    prog.Reparse(&p);
    EXPECT_PRED2(PolyEqual, p, expected);
  }
}

GTEST_TEST(TestMathematicalProgram, AddSosConstraint) {
  MathematicalProgram prog{};
  const auto x = prog.NewIndeterminates<1>()(0);
  const auto a = prog.NewContinuousVariables<1>()(0);

  // p1 has both a and x as indeterminates. So we need to reparse the polynomial
  // to have only x as the indeterminates.
  const symbolic::Polynomial p1(a + x * x);
  const Vector2<symbolic::Monomial> monomial_basis(symbolic::Monomial{},
                                                   symbolic::Monomial(x, 1));

  const Matrix2<symbolic::Variable> Q_psd = prog.AddSosConstraint(
      p1, monomial_basis, MathematicalProgram::NonnegativePolynomial::kSos,
      "Q");
  EXPECT_NE(Q_psd(0, 0).get_name().find("Q"), std::string::npos);
  EXPECT_EQ(prog.positive_semidefinite_constraints().size(), 1u);
  EXPECT_EQ(prog.lorentz_cone_constraints().size(), 0u);
  EXPECT_EQ(prog.rotated_lorentz_cone_constraints().size(), 0u);
  const int num_lin_con = prog.linear_constraints().size() +
                          prog.linear_equality_constraints().size();
  // Now call AddSosConstraint with type=kDsos.
  prog.AddSosConstraint(p1, monomial_basis,
                        MathematicalProgram::NonnegativePolynomial::kDsos);
  // With dsos, the mathematical program adds more linear constraints than it
  // did with sos.
  EXPECT_GT(prog.linear_constraints().size() +
                prog.linear_equality_constraints().size(),
            2 * num_lin_con);
  EXPECT_EQ(prog.positive_semidefinite_constraints().size(), 1u);

  // Now call AddSosConstraint with type=kSdsos.
  prog.AddSosConstraint(p1, monomial_basis,
                        MathematicalProgram::NonnegativePolynomial::kSdsos);
  EXPECT_GT(prog.lorentz_cone_constraints().size() +
                prog.rotated_lorentz_cone_constraints().size(),
            0u);
  EXPECT_EQ(prog.positive_semidefinite_constraints().size(), 1u);

  // p2 = (a+1)x² + 0 has some coefficient equal to 0. The constructed monomial
  // should remove that 0-term. Hence the returned monomial basis should only
  // contain [x].
  const symbolic::Polynomial p2{
      {{symbolic::Monomial(), 0}, {symbolic::Monomial(x, 2), a + 1}}};
  const auto [gram2, monomial_basis2] = prog.AddSosConstraint(p2);
  EXPECT_EQ(monomial_basis2.rows(), 1);
  EXPECT_EQ(monomial_basis2(0), symbolic::Monomial(x));
  EXPECT_EQ(gram2.rows(), 1);
}

template <typename C>
void RemoveCostTest(MathematicalProgram* prog,
                    const symbolic::Expression& cost1_expr,
                    const std::vector<Binding<C>>* program_costs,
                    ProgramAttribute affected_capability) {
  auto cost1 = prog->AddCost(cost1_expr);
  // cost1 and cost2 represent the same cost, but their evaluators point to
  // different objects.
  auto cost2 = prog->AddCost(cost1_expr);
  ASSERT_NE(cost1.evaluator().get(), cost2.evaluator().get());
  EXPECT_EQ(program_costs->size(), 2u);
  EXPECT_EQ(prog->RemoveCost(cost1), 1);
  EXPECT_EQ(program_costs->size(), 1u);
  EXPECT_EQ(program_costs->at(0).evaluator().get(), cost2.evaluator().get());
  EXPECT_TRUE(prog->required_capabilities().contains(affected_capability));
  // Now add another cost2 to program. If we remove cost2, now we get a program
  // with empty linear cost.
  prog->AddCost(cost2);
  EXPECT_EQ(program_costs->size(), 2u);
  EXPECT_EQ(prog->RemoveCost(cost2), 2);
  EXPECT_EQ(program_costs->size(), 0u);
  EXPECT_FALSE(prog->required_capabilities().contains(affected_capability));

  // Currently program_costs is empty.
  EXPECT_EQ(prog->RemoveCost(cost1), 0);
  EXPECT_FALSE(prog->required_capabilities().contains(affected_capability));

  prog->AddCost(cost1);
  // prog doesn't contain cost2, removing cost2 from prog ends up as a no-opt.
  EXPECT_EQ(prog->RemoveCost(cost2), 0);
  EXPECT_EQ(program_costs->size(), 1u);
  EXPECT_TRUE(prog->required_capabilities().contains(affected_capability));

  // cost3 and cost1 share the same evaluator, but the associated variables are
  // different.
  VectorX<symbolic::Variable> cost3_vars = cost1.variables();
  cost3_vars[0] = cost1.variables()[1];
  cost3_vars[1] = cost1.variables()[0];
  auto cost3 = prog->AddCost(cost1.evaluator(), cost3_vars);
  EXPECT_EQ(prog->RemoveCost(cost1), 1);
  EXPECT_EQ(program_costs->size(), 1u);
  EXPECT_TRUE(prog->required_capabilities().contains(affected_capability));
  EXPECT_EQ(program_costs->at(0).evaluator().get(), cost3.evaluator().get());
}

GTEST_TEST(TestMathematicalProgram, RemoveLinearCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  RemoveCostTest<LinearCost>(&prog, x[0] + 2 * x[1], &(prog.linear_costs()),
                             ProgramAttribute::kLinearCost);
}

GTEST_TEST(TestMathematicalProgram, RemoveQuadraticCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  RemoveCostTest(&prog, x[0] * x[0] + 2 * x[1] * x[1],
                 &(prog.quadratic_costs()), ProgramAttribute::kQuadraticCost);
}

GTEST_TEST(TestMathematicalProgram, RemoveGenericCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  RemoveCostTest(&prog, x[0] * x[0] * x[1], &(prog.generic_costs()),
                 ProgramAttribute::kGenericCost);
}

GTEST_TEST(TestMathematicalProgram, TestToString) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  auto y = prog.NewIndeterminates<1>("y");
  prog.AddLinearCost(2 * x[0] + 3 * x[1]);
  prog.AddLinearConstraint(x[0] + x[1] <= 2.0);
  prog.AddSosConstraint(x[0] * y[0] * y[0]);

  std::string s = prog.to_string();
  EXPECT_THAT(s, testing::HasSubstr("Decision variables"));
  EXPECT_THAT(s, testing::HasSubstr("Indeterminates"));
  EXPECT_THAT(s, testing::HasSubstr("Cost"));
  EXPECT_THAT(s, testing::HasSubstr("Constraint"));
  EXPECT_THAT(s, testing::HasSubstr("x"));
  EXPECT_THAT(s, testing::HasSubstr("y"));
  EXPECT_THAT(s, testing::HasSubstr("2"));
  EXPECT_THAT(s, testing::HasSubstr("3"));
}

GTEST_TEST(TestMathematicalProgram, RemoveDecisionVariable) {
  // A program where x(1) is not associated with any cost/constraint.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  prog.AddLinearCost(x(0) + x(2));
  prog.AddBoundingBoxConstraint(0, 2, x(0));
  prog.AddLinearConstraint(x(0) + 2 * x(2) <= 1);
  prog.SetInitialGuess(x, Eigen::Vector3d(0, 1, 2));
  prog.SetVariableScaling(x(0), 0.5);
  prog.SetVariableScaling(x(1), 3.0);
  prog.SetVariableScaling(x(2), 4.0);

  // Remove x(1) and check that all accessors remain in sync.
  const int x1_index = prog.FindDecisionVariableIndex(x(1));
  const int x1_index_removed = prog.RemoveDecisionVariable(x(1));
  EXPECT_EQ(x1_index, x1_index_removed);
  EXPECT_EQ(prog.num_vars(), 2);
  EXPECT_EQ(prog.FindDecisionVariableIndex(x(0)), 0);
  EXPECT_EQ(prog.FindDecisionVariableIndex(x(2)), 1);
  EXPECT_THAT(prog.decision_variables(),
              testing::Pointwise(testing::Truly(TupleVarEqual), {x(0), x(2)}));
  EXPECT_TRUE(CompareMatrices(prog.initial_guess(), Eigen::Vector2d(0, 2)));
  EXPECT_THAT(prog.GetVariableScaling(),
              testing::WhenSorted(testing::ElementsAre(
                  std::make_pair(0, 0.5), std::make_pair(1, 4.0))));
}

GTEST_TEST(TestMathematicalProgram, RemoveDecisionVariableError) {
  // Test RemoveDecisionVariable with erroneous input.
  // Remove an indeterminate.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  auto y = prog.NewIndeterminates<2>("y");
  DRAKE_EXPECT_THROWS_MESSAGE(
      prog.RemoveDecisionVariable(y(0)),
      ".*is not a decision variable of this MathematicalProgram.");

  // Remove a variable not in Mathematical program.
  const symbolic::Variable dummy("dummy");
  DRAKE_EXPECT_THROWS_MESSAGE(
      prog.RemoveDecisionVariable(dummy),
      ".*is not a decision variable of this MathematicalProgram.");

  // Remove a variable associated with a cost.
  prog.AddLinearCost(x(0));
  DRAKE_EXPECT_THROWS_MESSAGE(prog.RemoveDecisionVariable(x(0)),
                              ".* is associated with a LinearCost.*");

  // Remove a variable associated with a constraint.
  prog.AddLinearConstraint(x(0) + x(1) <= 1);
  DRAKE_EXPECT_THROWS_MESSAGE(prog.RemoveDecisionVariable(x(1)),
                              ".* is associated with a LinearConstraint[^]*");

  // Remove a variable associated with a visualization callback.
  prog.AddVisualizationCallback(
      [](const Eigen::VectorXd& vars) {
        drake::log()->info("{}", vars(0));
      },
      Vector1<symbolic::Variable>(x(2)));
  DRAKE_EXPECT_THROWS_MESSAGE(
      prog.RemoveDecisionVariable(x(2)),
      ".* is associated with a VisualizationCallback[^]*");
}

GTEST_TEST(TestMathematicalProgram, TestToLatex) {
  MathematicalProgram prog;
  std::string empty_prog = prog.ToLatex();
  EXPECT_EQ(empty_prog,
            "\\text{This MathematicalProgram has no decision variables.}");

  auto x = prog.NewContinuousVariables<2>("x");
  auto y = prog.NewIndeterminates<1>("y");
  prog.AddLinearCost(2 * x[0] + 3 * x[1]);
  prog.AddLinearConstraint(x[0] + x[1] <= 2.0);
  prog.AddSosConstraint(x[0] * y[0] * y[0]);

  std::string s = prog.ToLatex();
  EXPECT_THAT(s, testing::HasSubstr("\\min"));
  EXPECT_THAT(s, testing::HasSubstr("\\text{subject to}\\quad"));
  EXPECT_THAT(s, testing::HasSubstr("\\succeq 0."));
}

GTEST_TEST(TestMathematicalProgram, RemoveLinearConstraint) {
  // ProgramAttribute::kLinearConstraint depends on both
  // prog.linear_constraints() and prog.bounding_box_constraints().
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto lin_con1 = prog.AddLinearConstraint(x[0] + x[1] <= 1);
  auto lin_con2 = prog.AddLinearConstraint(x[0] + 2 * x[1] <= 1);
  EXPECT_EQ(prog.RemoveConstraint(lin_con1), 1);
  EXPECT_EQ(prog.linear_constraints().size(), 1u);
  EXPECT_TRUE(prog.required_capabilities().contains(
      ProgramAttribute::kLinearConstraint));
  // Now the program contains 2 lin_con2
  prog.AddConstraint(lin_con2);
  EXPECT_EQ(prog.RemoveConstraint(lin_con1), 0);
  EXPECT_EQ(prog.RemoveConstraint(lin_con2), 2);
  EXPECT_EQ(prog.linear_constraints().size(), 0u);
  EXPECT_FALSE(prog.required_capabilities().contains(
      ProgramAttribute::kLinearConstraint));

  auto bbcon = prog.AddBoundingBoxConstraint(1, 2, x);
  EXPECT_TRUE(prog.required_capabilities().contains(
      ProgramAttribute::kLinearConstraint));
  EXPECT_EQ(prog.RemoveConstraint(bbcon), 1);
  EXPECT_FALSE(prog.required_capabilities().contains(
      ProgramAttribute::kLinearConstraint));
}

GTEST_TEST(TestMathematicalProgram, RemoveConstraintPSD) {
  // ProgramAttribute::kPositiveSemidefiniteConstraint depends on both
  // prog.positive_semidefinite_constraints() and
  // prog.linear_matrix_inequality_constraints().
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  auto psd_con = prog.AddPositiveSemidefiniteConstraint(X);
  auto x = prog.NewContinuousVariables<2>();
  auto lmi_con = prog.AddLinearMatrixInequalityConstraint(
      {Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Ones(),
       2 * Eigen::Matrix3d::Ones()},
      x);
  EXPECT_EQ(prog.RemoveConstraint(psd_con), 1);
  EXPECT_EQ(prog.positive_semidefinite_constraints().size(), 0u);
  EXPECT_GT(prog.required_capabilities().count(
                ProgramAttribute::kPositiveSemidefiniteConstraint),
            0);
  EXPECT_EQ(prog.RemoveConstraint(lmi_con), 1);
  EXPECT_EQ(prog.linear_matrix_inequality_constraints().size(), 0u);
  EXPECT_EQ(prog.required_capabilities().count(
                ProgramAttribute::kPositiveSemidefiniteConstraint),
            0);
}

// Remove a constraint from @p prog. Before removing the constraint, @p
// prog_constraints has only one entry.
template <typename C>
void TestRemoveConstraint(MathematicalProgram* prog,
                          const Binding<C>& constraint,
                          const std::vector<Binding<C>>* prog_constraints,
                          ProgramAttribute removed_capability) {
  ASSERT_EQ(prog_constraints->size(), 1);
  ASSERT_TRUE(prog->required_capabilities().contains(removed_capability));
  EXPECT_EQ(prog->RemoveConstraint(constraint), 1);
  EXPECT_EQ(prog_constraints->size(), 0u);
  EXPECT_FALSE(prog->required_capabilities().contains(removed_capability));
}

GTEST_TEST(TestMathematicalProgram, RemoveConstraint) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  auto lin_eq_con = prog.AddLinearEqualityConstraint(x[0] + x[1] == 1);
  auto lorentz_con = prog.AddLorentzConeConstraint(
      x.cast<symbolic::Expression>(),
      LorentzConeConstraint::EvalType::kConvexSmooth);
  auto rotated_lorentz_con =
      prog.AddRotatedLorentzConeConstraint(x.cast<symbolic::Expression>());
  Eigen::SparseMatrix<double> A(3, 3);
  A.setIdentity();
  auto exponential_con =
      prog.AddExponentialConeConstraint(A, Eigen::Vector3d(1, 2, 3), x);
  auto generic_con = prog.AddConstraint(x(0) * x(0) * x(1) == 1);
  TestRemoveConstraint(&prog, lin_eq_con, &(prog.linear_equality_constraints()),
                       ProgramAttribute::kLinearEqualityConstraint);
  TestRemoveConstraint(&prog, lorentz_con, &(prog.lorentz_cone_constraints()),
                       ProgramAttribute::kLorentzConeConstraint);
  TestRemoveConstraint(&prog, rotated_lorentz_con,
                       &(prog.rotated_lorentz_cone_constraints()),
                       ProgramAttribute::kRotatedLorentzConeConstraint);
  TestRemoveConstraint(&prog, exponential_con,
                       &(prog.exponential_cone_constraints()),
                       ProgramAttribute::kExponentialConeConstraint);
  TestRemoveConstraint(&prog, generic_con, &(prog.generic_constraints()),
                       ProgramAttribute::kGenericConstraint);

  auto lcp_con = prog.AddLinearComplementarityConstraint(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Ones(), x);
  TestRemoveConstraint(&prog, lcp_con,
                       &(prog.linear_complementarity_constraints()),
                       ProgramAttribute::kLinearComplementarityConstraint);
}

GTEST_TEST(TestMathematicalProgram, RemoveVisualizationCallback) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  auto callback = prog.AddVisualizationCallback(
      [](const Eigen::VectorXd& vars) {
        drake::log()->info("{}", vars(0) + vars(1));
      },
      x);
  EXPECT_FALSE(prog.visualization_callbacks().empty());
  EXPECT_TRUE(
      prog.required_capabilities().contains(ProgramAttribute::kCallback));
  int count = prog.RemoveVisualizationCallback(callback);
  EXPECT_EQ(count, 1);
  EXPECT_TRUE(prog.visualization_callbacks().empty());
  EXPECT_FALSE(
      prog.required_capabilities().contains(ProgramAttribute::kCallback));
}

class ApproximatePSDConstraint : public ::testing::Test {
  // An arbitrary semidefinite program with 2 PSD constraints, 2 linear
  // constraints, and 1 equality constraint for testing the
  // Replace/TightenPsdConstraint methods.
 public:
  ApproximatePSDConstraint()
      : prog_(),
        X_{prog_.NewSymmetricContinuousVariables<3>()},
        Y_{prog_.NewSymmetricContinuousVariables<4>()},
        psd_constraint_X_{prog_.AddPositiveSemidefiniteConstraint(X_)},
        psd_constraint_Y_{prog_.AddPositiveSemidefiniteConstraint(Y_)} {
    // Add an arbitrary linear constraint on X.
    Eigen::MatrixXd A(2, 3);
    // clang-format off
    A << 1,  0, 1,
         0, -1, 1;
    // clang-format on
    Eigen::VectorXd lb(2);
    lb << -10, -7;
    Eigen::VectorXd ub(2);
    ub << 11, 9;

    prog_.AddLinearConstraint(A * X_ * Eigen::VectorXd::Ones(3) <= ub);
    prog_.AddLinearConstraint(A * X_ * Eigen::VectorXd::Ones(3) >= lb);
    prog_.AddLinearEqualityConstraint(Y_ == Eigen::MatrixXd::Identity(4, 4));
  }

 protected:
  MathematicalProgram prog_;
  MatrixXDecisionVariable X_;
  MatrixXDecisionVariable Y_;
  Binding<PositiveSemidefiniteConstraint> psd_constraint_X_;
  Binding<PositiveSemidefiniteConstraint> psd_constraint_Y_;
};

TEST_F(ApproximatePSDConstraint, TightenPsdConstraintToDd) {
  EXPECT_EQ(ssize(prog_.positive_semidefinite_constraints()), 2);
  EXPECT_EQ(ssize(prog_.linear_constraints()), 2);
  EXPECT_EQ(ssize(prog_.linear_equality_constraints()), 1);

  auto dd_constraint_X = prog_.TightenPsdConstraintToDd(psd_constraint_X_);

  EXPECT_EQ(ssize(prog_.positive_semidefinite_constraints()), 1);
  EXPECT_EQ(ssize(prog_.linear_constraints()), X_.rows() * X_.rows() + 2);
  EXPECT_EQ(ssize(prog_.linear_equality_constraints()), 1);

  auto dd_constraint_Y = prog_.TightenPsdConstraintToDd(psd_constraint_Y_);

  EXPECT_EQ(ssize(prog_.positive_semidefinite_constraints()), 0);
  EXPECT_EQ(ssize(prog_.linear_constraints()),
            Y_.rows() * Y_.rows() + X_.rows() * X_.rows() + 2);
  EXPECT_EQ(ssize(prog_.linear_equality_constraints()), 1);
}

GTEST_TEST(TightenPsdConstraintToDd, UnregisteredVariableError) {
  MathematicalProgram prog1;
  auto X1 = prog1.NewSymmetricContinuousVariables<3>();
  auto psd_constraint1 = prog1.AddPositiveSemidefiniteConstraint(X1);

  MathematicalProgram prog2;
  auto X2 = prog2.NewSymmetricContinuousVariables<3>();
  auto psd_constraint2 = prog2.AddPositiveSemidefiniteConstraint(X2);

  DRAKE_EXPECT_THROWS_MESSAGE(prog1.TightenPsdConstraintToDd(psd_constraint2),
                              ".*is not a decision variable.*");
}

GTEST_TEST(TightenPsdConstraintToDd, NoConstraintToReplace) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  // A constraint not in the program.
  auto constraint = internal::CreateBinding(
      std::make_shared<PositiveSemidefiniteConstraint>(X.rows()),
      Eigen::Map<VectorXDecisionVariable>(X.data(), X.size()));
  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  prog.TightenPsdConstraintToDd(constraint);
  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  // Still adds the DD constraint even though the constraint was not found in
  // the program.
  EXPECT_EQ(ssize(prog.linear_constraints()), X.rows() * X.rows());
}

TEST_F(ApproximatePSDConstraint, TightenPsdConstraintToSdd) {
  EXPECT_EQ(ssize(prog_.positive_semidefinite_constraints()), 2);
  EXPECT_EQ(ssize(prog_.linear_constraints()), 2);
  EXPECT_EQ(ssize(prog_.linear_equality_constraints()), 1);

  prog_.TightenPsdConstraintToSdd(psd_constraint_X_);
  EXPECT_EQ(ssize(prog_.positive_semidefinite_constraints()), 1);
  EXPECT_EQ(ssize(prog_.linear_constraints()), 2);
  // An sdd constraint on X adds an equality constraints on the upper diagonal
  // of X to represent slack variables
  EXPECT_EQ(ssize(prog_.linear_equality_constraints()), 2);
  // 3 choose 2 rotated lorentz cone constraints for constraint X to be sdd
  EXPECT_EQ(ssize(prog_.rotated_lorentz_cone_constraints()), 3);

  prog_.TightenPsdConstraintToSdd(psd_constraint_Y_);
  EXPECT_EQ(ssize(prog_.positive_semidefinite_constraints()), 0);
  EXPECT_EQ(ssize(prog_.linear_constraints()), 2);
  // An sdd constraint on Y adds an equality constraints on the upper diagonal
  // of Y to represent slack variables
  EXPECT_EQ(ssize(prog_.linear_equality_constraints()), 3);
  // 3 choose 2 rotated lorentz cone constraints for the constraint that X be
  // sdd and 4 choose 2 for the constraint that Y be sdd.
  EXPECT_EQ(ssize(prog_.rotated_lorentz_cone_constraints()), 9);
}

GTEST_TEST(TightenPsdConstraintToSdd, UnregisteredVariableError) {
  MathematicalProgram prog1;
  auto X1 = prog1.NewSymmetricContinuousVariables<3>();
  auto psd_constraint1 = prog1.AddPositiveSemidefiniteConstraint(X1);

  MathematicalProgram prog2;
  auto X2 = prog2.NewSymmetricContinuousVariables<3>();
  auto psd_constraint2 = prog2.AddPositiveSemidefiniteConstraint(X2);

  DRAKE_EXPECT_THROWS_MESSAGE(prog1.TightenPsdConstraintToDd(psd_constraint2),
                              ".*is not a decision variable.*");
}

GTEST_TEST(TightenPsdConstraintToSdd, NoConstraintToReplace) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  // A constraint not in the program.
  auto constraint = internal::CreateBinding(
      std::make_shared<PositiveSemidefiniteConstraint>(X.rows()),
      Eigen::Map<VectorXDecisionVariable>(X.data(), X.size()));
  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  prog.TightenPsdConstraintToSdd(constraint);
  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  // Still adds the SDD constraint even though the constraint was not found in
  // the program.
  EXPECT_EQ(ssize(prog.rotated_lorentz_cone_constraints()), 3);
}

TEST_F(ApproximatePSDConstraint, RelaxPsdConstraintToDdDualCone) {
  EXPECT_EQ(ssize(prog_.positive_semidefinite_constraints()), 2);
  EXPECT_EQ(ssize(prog_.linear_constraints()), 2);
  EXPECT_EQ(ssize(prog_.linear_equality_constraints()), 1);

  prog_.RelaxPsdConstraintToDdDualCone(psd_constraint_X_);

  EXPECT_EQ(ssize(prog_.positive_semidefinite_constraints()), 1);
  EXPECT_EQ(ssize(prog_.linear_constraints()), 3);
  EXPECT_EQ(ssize(prog_.linear_equality_constraints()), 1);

  prog_.RelaxPsdConstraintToDdDualCone(psd_constraint_Y_);

  EXPECT_EQ(ssize(prog_.positive_semidefinite_constraints()), 0);
  EXPECT_EQ(ssize(prog_.linear_constraints()), 4);
  EXPECT_EQ(ssize(prog_.linear_equality_constraints()), 1);
}

GTEST_TEST(RelaxPsdConstraintToDdDualCone, UnregisteredVariableError) {
  MathematicalProgram prog1;
  auto X1 = prog1.NewSymmetricContinuousVariables<3>();
  auto psd_constraint1 = prog1.AddPositiveSemidefiniteConstraint(X1);

  MathematicalProgram prog2;
  auto X2 = prog2.NewSymmetricContinuousVariables<3>();
  auto psd_constraint2 = prog2.AddPositiveSemidefiniteConstraint(X2);

  DRAKE_EXPECT_THROWS_MESSAGE(
      prog1.RelaxPsdConstraintToDdDualCone(psd_constraint2),
      ".*is not a decision variable.*");
}

GTEST_TEST(RelaxPsdConstraintToDdDualCone, NoConstraintToReplace) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  // A constraint not in the program.
  auto constraint = internal::CreateBinding(
      std::make_shared<PositiveSemidefiniteConstraint>(X.rows()),
      Eigen::Map<VectorXDecisionVariable>(X.data(), X.size()));
  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  prog.RelaxPsdConstraintToDdDualCone(constraint);
  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  // Still adds the DD constraint even though the constraint was not found in
  // the program.
  EXPECT_EQ(ssize(prog.linear_constraints()), 1);
}

TEST_F(ApproximatePSDConstraint, RelaxPsdConstraintToSddDualCone) {
  EXPECT_EQ(ssize(prog_.positive_semidefinite_constraints()), 2);
  EXPECT_EQ(ssize(prog_.linear_constraints()), 2);
  EXPECT_EQ(ssize(prog_.linear_equality_constraints()), 1);

  prog_.RelaxPsdConstraintToSddDualCone(psd_constraint_X_);

  EXPECT_EQ(ssize(prog_.positive_semidefinite_constraints()), 1);
  EXPECT_EQ(ssize(prog_.linear_constraints()), 2);
  EXPECT_EQ(ssize(prog_.linear_equality_constraints()), 1);
  // 3 choose 2 rotated lorentz cone constraints for constraint X to be sdd
  EXPECT_EQ(ssize(prog_.rotated_lorentz_cone_constraints()), 3);

  prog_.RelaxPsdConstraintToSddDualCone(psd_constraint_Y_);

  EXPECT_EQ(ssize(prog_.positive_semidefinite_constraints()), 0);
  EXPECT_EQ(ssize(prog_.linear_constraints()), 2);
  EXPECT_EQ(ssize(prog_.linear_equality_constraints()), 1);
  // 3 choose 2 rotated lorentz cone constraints for the constraint that X be
  // sdd and 4 choose 2 for the constraint that Y be sdd.
  EXPECT_EQ(ssize(prog_.rotated_lorentz_cone_constraints()), 9);
}

GTEST_TEST(RelaxPsdConstraintToSddDualCone, UnregisteredVariableError) {
  MathematicalProgram prog1;
  auto X1 = prog1.NewSymmetricContinuousVariables<3>();
  auto psd_constraint1 = prog1.AddPositiveSemidefiniteConstraint(X1);

  MathematicalProgram prog2;
  auto X2 = prog2.NewSymmetricContinuousVariables<3>();
  auto psd_constraint2 = prog2.AddPositiveSemidefiniteConstraint(X2);

  DRAKE_EXPECT_THROWS_MESSAGE(
      prog1.RelaxPsdConstraintToSddDualCone(psd_constraint2),
      ".*is not a decision variable.*");
}

GTEST_TEST(RelaxPsdConstraintToSddDualCone, NoConstraintToReplace) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  // A constraint not in the program.
  auto constraint = internal::CreateBinding(
      std::make_shared<PositiveSemidefiniteConstraint>(X.rows()),
      Eigen::Map<VectorXDecisionVariable>(X.data(), X.size()));
  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  prog.RelaxPsdConstraintToSddDualCone(constraint);
  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  // Still adds the SDD dual cone constraint even though the constraint was not
  // found in the program.
  EXPECT_EQ(ssize(prog.rotated_lorentz_cone_constraints()), 3);
}

GTEST_TEST(MathematicalProgramTest, AddLogDeterminantLowerBoundConstraint) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>("X");
  auto ret = prog.AddLogDeterminantLowerBoundConstraint(
      X.cast<symbolic::Expression>(), 1);
  const auto& constraint = std::get<0>(ret);
  const auto& t = std::get<1>(ret);
  EXPECT_TRUE(CompareMatrices(constraint.evaluator()->GetDenseA(),
                              Eigen::RowVector3d::Ones()));
  EXPECT_TRUE(
      CompareMatrices(constraint.evaluator()->lower_bound(), Vector1d(1)));
  EXPECT_TRUE(
      CompareMatrices(constraint.evaluator()->upper_bound(), Vector1d(kInf)));
  EXPECT_EQ(constraint.variables().size(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(constraint.variables()(i).equal_to(t(i)));
  }
  EXPECT_EQ(prog.exponential_cone_constraints().size(), 3);
  EXPECT_EQ(prog.linear_matrix_inequality_constraints().size(), 1);
}

class EmptyCost final : public Cost {
 public:
  // This cost is thread safe in general. However, for the sake of testing
  // we allow the constructor argument which changes the value of
  // is_thread_safe.
  explicit EmptyCost(bool is_thread_safe = true) : Cost(0) {
    set_is_thread_safe(is_thread_safe);
  }

  ~EmptyCost() {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>&, VectorXd*) const {}
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>&, AutoDiffVecXd*) const {}
  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const {}
};

GTEST_TEST(MathematicalProgramIsThreadSafe, MathematicalProgramIsThreadSafe) {
  MathematicalProgram prog;
  EXPECT_TRUE(prog.IsThreadSafe());

  auto x = prog.NewContinuousVariables<2>();
  auto linear_constraint = prog.AddLinearConstraint(x[0] <= 0);
  EXPECT_TRUE(prog.IsThreadSafe());

  // A constraint marked as non-thread safe.
  auto non_thread_safe_constraint_binding =
      prog.AddConstraint(x[0] * x[1] * x[1] == 1.);
  EXPECT_FALSE(prog.IsThreadSafe());

  prog.RemoveConstraint(linear_constraint);
  EXPECT_FALSE(prog.IsThreadSafe());

  prog.RemoveConstraint(non_thread_safe_constraint_binding);
  // The program has no costs and constraints, therefore it is thread safe.
  EXPECT_TRUE(prog.IsThreadSafe());

  auto linear_cost = prog.AddLinearCost(x[0]);
  EXPECT_TRUE(prog.IsThreadSafe());

  // A cost marked as non-thread safe.
  auto non_thread_safe_cost_binding = prog.AddCost(x[0] * x[1] * x[1]);
  EXPECT_FALSE(prog.IsThreadSafe());

  // The only cost and constraint is the thread-safe linear cost.
  prog.RemoveCost(non_thread_safe_cost_binding);
  EXPECT_TRUE(prog.IsThreadSafe());

  auto my_callback = [](const Eigen::Ref<const Eigen::VectorXd>& v) {
    EXPECT_EQ(v.size(), 2);
    EXPECT_EQ(v(0), 1.);
    EXPECT_EQ(v(1), 2.);
  };
  auto b = prog.AddVisualizationCallback(my_callback, x);
  // Programs with visualization call backs are not thread safe.
  EXPECT_FALSE(prog.IsThreadSafe());
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
