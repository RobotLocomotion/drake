#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/snopt_solver.h"

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

using std::numeric_limits;

namespace drake {
namespace solvers {
namespace {

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

GTEST_TEST(testMathematicalProgram, testAddFunction) {
  MathematicalProgram prog;
  prog.NewContinuousVariables<1>();

  Movable movable;
  prog.AddCost(std::move(movable));
  prog.AddCost(Movable());

  Copyable copyable;
  prog.AddCost(copyable);

  Unique unique;
  prog.AddCost(std::cref(unique));
  prog.AddCost(std::make_shared<Unique>());
  prog.AddCost(std::unique_ptr<Unique>(new Unique));
}

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void CheckSolverType(MathematicalProgram& prog,
                     std::string desired_solver_name) {
  std::string solver_name;
  int solver_result;
  prog.GetSolverResult(&solver_name, &solver_result);
  EXPECT_EQ(solver_name, desired_solver_name);
}

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void RunNonlinearProgram(MathematicalProgram& prog,
                         std::function<void(void)> test_func) {
  IpoptSolver ipopt_solver;
  NloptSolver nlopt_solver;
  SnoptSolver snopt_solver;

  std::pair<const char*, MathematicalProgramSolverInterface*> solvers[] = {
      std::make_pair("SNOPT", &snopt_solver),
      std::make_pair("NLopt", &nlopt_solver),
      std::make_pair("Ipopt", &ipopt_solver)};

  for (const auto& solver : solvers) {
    if (!solver.second->available()) {
      continue;
    }
    SolutionResult result = SolutionResult::kUnknownError;
    ASSERT_NO_THROW(result = solver.second->Solve(prog)) << "Using solver: "
                                                         << solver.first;
    EXPECT_EQ(result, SolutionResult::kSolutionFound) << "Using solver: "
                                                      << solver.first;
    EXPECT_NO_THROW(test_func()) << "Using solver: " << solver.first;
  }
}

GTEST_TEST(testMathematicalProgram, BoundingBoxTest) {
  // A simple test program to test if the bounding box constraints are added
  // correctly.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(4);

  // Deliberately add two constraints on overlapped decision variables.
  // For x(1), the lower bound of the second constraint are used; while
  // the upper bound of the first variable is used.
  VectorDecisionVariable<2> variable_vec(x(1), x(3));
  prog.AddBoundingBoxConstraint(Vector2d(-1, -2), Vector2d(-0.2, -1),
                                variable_vec);
  prog.AddBoundingBoxConstraint(Vector3d(-1, -0.5, -3), Vector3d(2, 1, -0.1),
                                {x.head<1>(), x.segment<2>(1)});

  Vector4d lb(-1, -0.5, -3, -2);
  Vector4d ub(2, -0.2, -0.1, -1);
  prog.SetInitialGuessForAllVariables(Vector4d::Zero());
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    for (int i = 0; i < 4; ++i) {
      EXPECT_GE(x_value(i), lb(i) - 1E-10);
      EXPECT_LE(x_value(i), ub(i) + 1E-10);
    }
  });
}

GTEST_TEST(testMathematicalProgram, BoundingBoxTest2) {
  // Test the scalar version of the bounding box constraint methods.

  MathematicalProgram prog;
  auto x1 = prog.NewContinuousVariables<2, 2>("x1");
  MatrixXDecisionVariable x2(2, 2);
  x2 = x1;
  // Four different ways to construct an equivalent constraint.
  // 1. Imposes constraint on a static-sized matrix of decision variables.
  // 2. Imposes constraint on a list of vectors of decision variables.
  // 3. Imposes constraint on a dynamic-sized matrix of decision variables.
  // 4. Imposes constraint using a vector of lower/upper bound, as compared
  //    to the previous three cases which use a scalar lower/upper bound.
  auto constraint1 = prog.AddBoundingBoxConstraint(0, 1, x1);
  auto constraint2 =
      prog.AddBoundingBoxConstraint(0, 1, {x1.col(0), x1.col(1)});
  auto constraint3 = prog.AddBoundingBoxConstraint(0, 1, x2);
  auto constraint4 = prog.AddBoundingBoxConstraint(Eigen::Vector4d::Zero(),
                                                   Eigen::Vector4d::Ones());

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
      CompareMatrices(constraint1->upper_bound(), constraint2->upper_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint2->upper_bound(), constraint3->upper_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint3->upper_bound(), constraint4->upper_bound()));
}

/* A generic cost derived from Constraint class. This is meant for testing
 * adding a cost to optimization program, and the cost is in the form of a
 * derived class of Constraint.
 */
class GenericTrivialCost1 : public Constraint {
 public:
  GenericTrivialCost1()
      : Constraint(1, 3, Vector1d(std::numeric_limits<double>::infinity()),
                   Vector1d(std::numeric_limits<double>::infinity())),
        private_val_(2) {}

 protected:
  void DoEval(const Ref<const Eigen::VectorXd> &x,
              VectorXd &y) const override {
    y.resize(1);
    y(0) = x(0) * x(1) + x(2) / x(0) * private_val_;
  }

  void DoEval(const Ref<const TaylorVecXd> &x,
              TaylorVecXd &y) const override {
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
  // 4. Add a ConstraintImpl object on a VariableRefList object.
  // 5. Add a unique_ptr of object that can be converted to a ConstraintImpl
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

GTEST_TEST(testMathematicalProgram, trivialLinearSystem) {
  MathematicalProgram prog;

  // First, add four variables set equal by four equations
  // to equal a random vector
  auto x = prog.NewContinuousVariables<4>();

  auto x2 = x(2);
  auto xhead = x.head(3);

  Vector4d b = Vector4d::Random();
  auto con = prog.AddLinearEqualityConstraint(Matrix4d::Identity(), b, x);

  prog.SetInitialGuessForAllVariables(Vector4d::Zero());
  prog.Solve();
  auto x_value = prog.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(b, x_value, 1e-10, MatrixCompareType::absolute));

  EXPECT_NEAR(b(2), prog.GetSolution(x2), 1e-10);

  const auto& xhead_value = prog.GetSolution(xhead);
  EXPECT_TRUE(CompareMatrices(b.head(3), xhead_value, 1e-10,
                              MatrixCompareType::absolute));

  EXPECT_NEAR(b(2), xhead_value(2), 1e-10);  // a segment of a segment.

  CheckSolverType(prog, "Linear System Solver");

  // Add two more variables with a very slightly more complicated
  // constraint and solve again. Should still be a linear system.
  auto y = prog.NewContinuousVariables<2>();
  prog.AddLinearEqualityConstraint(2 * Matrix2d::Identity(), b.topRows(2), y);
  prog.Solve();
  const auto& y_value = prog.GetSolution(y);
  EXPECT_TRUE(CompareMatrices(b.topRows(2) / 2, y_value, 1e-10,
                              MatrixCompareType::absolute));
  x_value = prog.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(b, x_value, 1e-10, MatrixCompareType::absolute));
  CheckSolverType(prog, "Linear System Solver");

  // Now modify the original constraint by its handle
  con->UpdateConstraint(3 * Matrix4d::Identity(), b);
  prog.Solve();
  EXPECT_TRUE(CompareMatrices(b.topRows(2) / 2, prog.GetSolution(y), 1e-10,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(b / 3, prog.GetSolution(x), 1e-10,
                              MatrixCompareType::absolute));
  CheckSolverType(prog, "Linear System Solver");

  prog.AddBoundingBoxConstraint(Vector2d::Constant(-1000),
                                Vector2d::Constant(1000.0), x.head<2>());

  // Now solve as a nonlinear program.
  RunNonlinearProgram(prog, [&]() {
    EXPECT_TRUE(CompareMatrices(b.topRows(2) / 2, prog.GetSolution(y), 1e-10,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(b / 3, prog.GetSolution(x), 1e-10,
                                MatrixCompareType::absolute));
  });
}

GTEST_TEST(testMathematicalProgram, trivialLinearEquality) {
  MathematicalProgram prog;

  auto vars = prog.NewContinuousVariables<2>();

  // Use a non-square matrix to catch row/column mistakes in the solvers.
  prog.AddLinearEqualityConstraint(Eigen::RowVector2d(0, 1),
                                   Vector1d::Constant(1));
  prog.SetInitialGuess(vars, Vector2d(2, 2));
  RunNonlinearProgram(prog, [&]() {
    const auto& vars_value = prog.GetSolution(vars);
    EXPECT_DOUBLE_EQ(vars_value(0), 2);
    EXPECT_DOUBLE_EQ(vars_value(1), 1);
  });
}

// Tests a quadratic optimization problem, with only quadratic cost
// 0.5 *x'*Q*x + b'*x
// The optimal solution is -inverse(Q)*b
GTEST_TEST(testMathematicalProgram, QuadraticCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>();

  Vector4d Qdiag(1.0, 2.0, 3.0, 4.0);
  Matrix4d Q = Qdiag.asDiagonal();
  Q(1, 2) = 0.1;
  Q(2, 3) = -0.02;

  Vector4d b(1.0, -0.5, 1.3, 2.5);
  prog.AddQuadraticCost(Q, b);

  Matrix4d Q_transpose = Q;
  Q_transpose.transposeInPlace();
  Matrix4d Q_symmetric = 0.5 * (Q + Q_transpose);
  Vector4d expected = -Q_symmetric.ldlt().solve(b);
  prog.SetInitialGuess(x, Vector4d::Zero());
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-6, MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(
        prog.EvalBindingAtSolution(prog.quadratic_costs().front()),
        0.5 * x_value.transpose() * Q_symmetric * x_value +
            b.transpose() * x_value,
        1E-14, MatrixCompareType::absolute));
  });
}

// This test comes from Section 2.2 of "Handbook of Test Problems in
// Local and Global Optimization."
class TestProblem1Cost {
 public:
  static size_t numInputs() { return 5; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
    y(0) = (-50.0 * x(0) * x(0)) + (42 * x(0)) - (50.0 * x(1) * x(1)) +
           (44 * x(1)) - (50.0 * x(2) * x(2)) + (45 * x(2)) -
           (50.0 * x(3) * x(3)) + (47 * x(3)) - (50.0 * x(4) * x(4)) +
           (47.5 * x(4));
  }
};

GTEST_TEST(testMathematicalProgram, testProblem1) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<5>();
  prog.AddCost(TestProblem1Cost());
  VectorXd constraint(5);
  constraint << 20, 12, 11, 7, 4;
  prog.AddLinearConstraint(
      constraint.transpose(),
      Vector1d::Constant(-numeric_limits<double>::infinity()),
      Vector1d::Constant(40));
  prog.AddBoundingBoxConstraint(MatrixXd::Constant(5, 1, 0),
                                MatrixXd::Constant(5, 1, 1));
  VectorXd expected(5);
  expected << 1, 1, 0, 1, 0;

  // IPOPT has difficulty with this problem depending on the initial
  // conditions, which is why the initial guess varies so little.
  std::srand(0);
  prog.SetInitialGuess(x, expected + .01 * VectorXd::Random(5));
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-9, MatrixCompareType::absolute));
  });
}

// This test is semantically equivalent with the above testProgram1 test, but it
// uses the symbolic version of AddLinearConstraint method in
// MathematicalProgram class.
GTEST_TEST(testMathematicalProgram, testProblem1Symbolic) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<5>();
  const auto constraint =
      20 * x(0) + 12 * x(1) + 11 * x(2) + 7 * x(3) + 4 * x(4);
  prog.AddCost(TestProblem1Cost());
  prog.AddLinearConstraint(constraint, -numeric_limits<double>::infinity(),
                           40.0);
  prog.AddBoundingBoxConstraint(MatrixXd::Constant(5, 1, 0),
                                MatrixXd::Constant(5, 1, 1));
  VectorXd expected(5);
  expected << 1, 1, 0, 1, 0;

  // IPOPT has difficulty with this problem depending on the initial
  // conditions, which is why the initial guess varies so little.
  std::srand(0);
  prog.SetInitialGuess(x, expected + .01 * VectorXd::Random(5));
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-9, MatrixCompareType::absolute));
  });
}

// This test is identical to testProblem1 above but the cost is
// framed as a QP instead.
GTEST_TEST(testMathematicalProgram, testProblem1AsQP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<5>();

  Eigen::MatrixXd Q = -100 * Eigen::Matrix<double, 5, 5>::Identity();
  Eigen::VectorXd c(5);
  c << 42, 44, 45, 47, 47.5;

  prog.AddQuadraticCost(Q, c);

  VectorXd constraint(5);
  constraint << 20, 12, 11, 7, 4;
  prog.AddLinearConstraint(constraint.transpose(),
                           -numeric_limits<double>::infinity(), 40);
  EXPECT_EQ(prog.linear_constraints().size(), 1u);
  EXPECT_EQ(prog.generic_constraints().size(), 0u);

  prog.AddBoundingBoxConstraint(MatrixXd::Constant(5, 1, 0),
                                MatrixXd::Constant(5, 1, 1));
  VectorXd expected(5);
  expected << 1, 1, 0, 1, 0;
  std::srand(0);
  prog.SetInitialGuess(x, expected + .01 * VectorXd::Random(5));
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-9, MatrixCompareType::absolute));
  });
}

// This test is semantically equivalent with the above testProgram1AsQp test,
// but it uses the symbolic version of AddLinearConstraint method in
// MathematicalProgram class.
GTEST_TEST(testMathematicalProgram, testProblem1AsQPSymbolic) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<5>();

  Eigen::MatrixXd Q{-100 * Eigen::Matrix<double, 5, 5>::Identity()};
  Eigen::VectorXd c(5);
  c << 42, 44, 45, 47, 47.5;

  prog.AddQuadraticCost(Q, c);

  const symbolic::Expression constraint{20 * x(0) + 12 * x(1) + 11 * x(2) +
                                        7 * x(3) + 4 * x(4)};
  prog.AddLinearConstraint(constraint, -numeric_limits<double>::infinity(), 40);
  EXPECT_EQ(prog.linear_constraints().size(), 1u);
  EXPECT_EQ(prog.generic_constraints().size(), 0u);

  prog.AddBoundingBoxConstraint(MatrixXd::Constant(5, 1, 0),
                                MatrixXd::Constant(5, 1, 1));
  VectorXd expected(5);
  expected << 1, 1, 0, 1, 0;
  std::srand(0);
  prog.SetInitialGuess(x, expected + .01 * VectorXd::Random(5));
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-9, MatrixCompareType::absolute));
  });
}

// This test comes from Section 2.3 of "Handbook of Test Problems in
// Local and Global Optimization."
class TestProblem2Cost {
 public:
  static size_t numInputs() { return 6; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
    y(0) = (-50.0 * x(0) * x(0)) + (-10.5 * x(0)) - (50.0 * x(1) * x(1)) +
           (-7.5 * x(1)) - (50.0 * x(2) * x(2)) + (-3.5 * x(2)) -
           (50.0 * x(3) * x(3)) + (-2.5 * x(3)) - (50.0 * x(4) * x(4)) +
           (-1.5 * x(4)) + (-10.0 * x(5));
  }
};

GTEST_TEST(testMathematicalProgram, testProblem2) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<6>();
  prog.AddCost(TestProblem2Cost());
  VectorXd constraint1(6), constraint2(6);
  constraint1 << 6, 3, 3, 2, 1, 0;
  prog.AddLinearConstraint(
      constraint1.transpose(),
      Vector1d::Constant(-numeric_limits<double>::infinity()),
      Vector1d::Constant(6.5));
  constraint2 << 10, 0, 10, 0, 0, 1;
  prog.AddLinearConstraint(
      constraint2.transpose(),
      Vector1d::Constant(-numeric_limits<double>::infinity()),
      Vector1d::Constant(20));
  Eigen::VectorXd lower(6);
  lower << 0, 0, 0, 0, 0, 0;
  Eigen::VectorXd upper(6);
  upper << 1, 1, 1, 1, 1, numeric_limits<double>::infinity();
  prog.AddBoundingBoxConstraint(lower, upper);
  VectorXd expected(6);
  expected << 0, 1, 0, 1, 1, 20;
  std::srand(0);
  prog.SetInitialGuess(x, expected + .01 * VectorXd::Random(6));
  // This test seems to be fairly sensitive to how much the randomness
  // causes the initial guess to deviate, so the tolerance is a bit
  // larger than others.  IPOPT is particularly sensitive here.
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-3, MatrixCompareType::absolute));
  });
}

// This test is semantically equivalent with the above testProgram2 test,
// but it uses the symbolic version of AddLinearConstraint method in
// MathematicalProgram class.
GTEST_TEST(testMathematicalProgram, testProblem2Symbolic) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<6>();
  prog.AddCost(TestProblem2Cost());
  const symbolic::Expression constraint1{6 * x(0) + 3 * x(1) + 3 * x(2) +
                                         2 * x(3) + x(4)};
  prog.AddLinearConstraint(constraint1, -numeric_limits<double>::infinity(),
                           6.5);
  const symbolic::Expression constraint2{10 * x(0) + 10 * x(2) + x(5)};
  prog.AddLinearConstraint(constraint2, -numeric_limits<double>::infinity(),
                           20);
  VectorXd lower(6);
  lower << 0, 0, 0, 0, 0, 0;
  VectorXd upper(6);
  upper << 1, 1, 1, 1, 1, numeric_limits<double>::infinity();
  prog.AddBoundingBoxConstraint(lower, upper);
  VectorXd expected(6);
  expected << 0, 1, 0, 1, 1, 20;
  std::srand(0);
  prog.SetInitialGuess(x, expected + .01 * VectorXd::Random(6));
  // This test seems to be fairly sensitive to how much the randomness
  // causes the initial guess to deviate, so the tolerance is a bit
  // larger than others.  IPOPT is particularly sensitive here.
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-3, MatrixCompareType::absolute));
  });
}

// This test is identical to testProblem2 above but the cost is
// framed as a QP instead.
GTEST_TEST(testMathematicalProgram, testProblem2AsQP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<6>();
  MatrixXd Q = -100.0 * MatrixXd::Identity(6, 6);
  Q(5, 5) = 0.0;
  VectorXd c(6);
  c << -10.5, -7.5, -3.5, -2.5, -1.5, -10.0;

  prog.AddQuadraticCost(Q, c);

  VectorXd constraint1(6), constraint2(6);
  constraint1 << 6, 3, 3, 2, 1, 0;
  prog.AddLinearConstraint(constraint1.transpose(),
                           -numeric_limits<double>::infinity(), 6.5);
  constraint2 << 10, 0, 10, 0, 0, 1;
  prog.AddLinearConstraint(constraint2.transpose(),
                           -numeric_limits<double>::infinity(), 20);

  Eigen::VectorXd lower(6);
  lower << 0, 0, 0, 0, 0, 0;
  Eigen::VectorXd upper(6);
  upper << 1, 1, 1, 1, 1, numeric_limits<double>::infinity();
  prog.AddBoundingBoxConstraint(lower, upper);

  VectorXd expected(6);
  expected << 0, 1, 0, 1, 1, 20;
  std::srand(0);
  prog.SetInitialGuess(x, expected + .01 * VectorXd::Random(6));

  // This test seems to be fairly sensitive to how much the randomness
  // causes the initial guess to deviate, so the tolerance is a bit
  // larger than others.  IPOPT is particularly sensitive here.
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-3, MatrixCompareType::absolute));
  });
}

// This test is semantically equivalent with the above testProgram2AsQP test,
// but it uses the symbolic version of AddLinearConstraint method in
// MathematicalProgram class.
GTEST_TEST(testMathematicalProgram, testProblem2AsQPSymbolic) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<6>();
  MatrixXd Q{-100.0 * MatrixXd::Identity(6, 6)};
  Q(5, 5) = 0.0;
  VectorXd c(6);
  c << -10.5, -7.5, -3.5, -2.5, -1.5, -10.0;

  prog.AddQuadraticCost(Q, c);

  const symbolic::Expression constraint1{6 * x(0) + 3 * x(1) + 3 * x(2) +
                                         2 * x(3) + x(4)};
  prog.AddLinearConstraint(constraint1, -numeric_limits<double>::infinity(),
                           6.5);
  const symbolic::Expression constraint2{10 * x(0) + 10 * x(2) + x(5)};
  prog.AddLinearConstraint(constraint2, -numeric_limits<double>::infinity(),
                           20);

  VectorXd lower(6);
  lower << 0, 0, 0, 0, 0, 0;
  VectorXd upper(6);
  upper << 1, 1, 1, 1, 1, numeric_limits<double>::infinity();
  prog.AddBoundingBoxConstraint(lower, upper);

  VectorXd expected(6);
  expected << 0, 1, 0, 1, 1, 20;
  std::srand(0);
  prog.SetInitialGuess(x, expected + .01 * VectorXd::Random(6));

  // This test seems to be fairly sensitive to how much the randomness
  // causes the initial guess to deviate, so the tolerance is a bit
  // larger than others.  IPOPT is particularly sensitive here.
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-3, MatrixCompareType::absolute));
  });
}

// This test comes from Section 3.4 of "Handbook of Test Problems in
// Local and Global Optimization."
class LowerBoundTestCost {
 public:
  static size_t numInputs() { return 6; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
    y(0) = -25 * (x(0) - 2) * (x(0) - 2) + (x(1) - 2) * (x(1) - 2) -
           (x(2) - 1) * (x(2) - 1) - (x(3) - 4) * (x(3) - 4) -
           (x(4) - 1) * (x(4) - 1) - (x(5) - 4) * (x(5) - 4);
  }
};

class LowerBoundTestConstraint : public Constraint {
 public:
  LowerBoundTestConstraint(int i1, int i2)
      : Constraint(1, Eigen::Dynamic, Vector1d::Constant(4),
                   Vector1d::Constant(numeric_limits<double>::infinity())),
        i1_(i1),
        i2_(i2) {}

 protected:
  // for just these two types, implementing this locally is almost cleaner...
  void DoEval(const Eigen::Ref<const Eigen::VectorXd> &x,
              Eigen::VectorXd &y) const override {
    EvalImpl(x, y);
  }
  void DoEval(const Eigen::Ref<const TaylorVecXd> &x,
              TaylorVecXd &y) const override {
    EvalImpl(x, y);
  }

 private:
  template <typename ScalarType>
  void EvalImpl(
      const Eigen::Ref<const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>>& x,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& y) const {
    y.resize(1);
    y(0) = (x(i1_) - 3) * (x(i1_) - 3) + x(i2_);
  }

  int i1_;
  int i2_;
};

GTEST_TEST(testMathematicalProgram, lowerBoundTest) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(6);
  prog.AddCost(LowerBoundTestCost());
  std::shared_ptr<Constraint> con1(new LowerBoundTestConstraint(2, 3));
  prog.AddConstraint(con1);
  std::shared_ptr<Constraint> con2(new LowerBoundTestConstraint(4, 5));
  prog.AddConstraint(con2);

  Eigen::VectorXd c1(6);
  c1 << 1, -3, 0, 0, 0, 0;
  prog.AddLinearConstraint(
      c1.transpose(), Vector1d::Constant(-numeric_limits<double>::infinity()),
      Vector1d::Constant(2));
  Eigen::VectorXd c2(6);
  c2 << -1, 1, 0, 0, 0, 0;
  prog.AddLinearConstraint(
      c2.transpose(), Vector1d::Constant(-numeric_limits<double>::infinity()),
      Vector1d::Constant(2));
  Eigen::VectorXd c3(6);
  c3 << 1, 1, 0, 0, 0, 0;
  prog.AddLinearConstraint(c3.transpose(), Vector1d::Constant(2),
                           Vector1d::Constant(6));
  Eigen::VectorXd lower(6);
  lower << 0, 0, 1, 0, 1, 0;
  Eigen::VectorXd upper(6);
  upper << numeric_limits<double>::infinity(),
      numeric_limits<double>::infinity(), 5, 6, 5, 10;
  prog.AddBoundingBoxConstraint(lower, upper);

  Eigen::VectorXd expected(6);
  expected << 5, 1, 5, 0, 5, 10;
  std::srand(0);
  Eigen::VectorXd delta = .05 * Eigen::VectorXd::Random(6);
  prog.SetInitialGuess(x, expected + delta);

  // This test seems to be fairly sensitive to how much the randomness
  // causes the initial guess to deviate, so the tolerance is a bit
  // larger than others.  IPOPT is particularly sensitive here.
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-3, MatrixCompareType::absolute));
  });

  // Try again with the offsets in the opposite direction.
  prog.SetInitialGuess(x, expected - delta);
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-3, MatrixCompareType::absolute));
  });
}

// This test is semantically equivalent with the above lowerBoundTest test,
// but it uses the symbolic version of AddLinearConstraint method in
// MathematicalProgram class.
GTEST_TEST(testMathematicalProgram, lowerBoundTestSymbolic) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(6);
  prog.AddCost(LowerBoundTestCost());
  std::shared_ptr<Constraint> con1(new LowerBoundTestConstraint(2, 3));
  prog.AddConstraint(con1);
  std::shared_ptr<Constraint> con2(new LowerBoundTestConstraint(4, 5));
  prog.AddConstraint(con2);

  prog.AddLinearConstraint(x(0) - 3 * x(1), -numeric_limits<double>::infinity(),
                           2);
  prog.AddLinearConstraint(-x(0) + x(1), -numeric_limits<double>::infinity(),
                           2);
  prog.AddLinearConstraint(x(0) + x(1), -numeric_limits<double>::infinity(), 6);

  VectorXd lower(6);
  lower << 0, 0, 1, 0, 1, 0;
  VectorXd upper(6);
  upper << numeric_limits<double>::infinity(),
      numeric_limits<double>::infinity(), 5, 6, 5, 10;
  prog.AddBoundingBoxConstraint(lower, upper);

  VectorXd expected(6);
  expected << 5, 1, 5, 0, 5, 10;
  std::srand(0);
  VectorXd delta{.05 * VectorXd::Random(6)};
  prog.SetInitialGuess(x, expected + delta);

  // This test seems to be fairly sensitive to how much the randomness
  // causes the initial guess to deviate, so the tolerance is a bit
  // larger than others.  IPOPT is particularly sensitive here.
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-3, MatrixCompareType::absolute));
  });

  // Try again with the offsets in the opposite direction.
  prog.SetInitialGuess(x, expected - delta);
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    EXPECT_TRUE(
        CompareMatrices(x_value, expected, 1e-3, MatrixCompareType::absolute));
  });
}

class SixHumpCamelCost {
 public:
  static size_t numInputs() { return 2; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
    y(0) =
        x(0) * x(0) * (4 - 2.1 * x(0) * x(0) + x(0) * x(0) * x(0) * x(0) / 3) +
        x(0) * x(1) + x(1) * x(1) * (-4 + 4 * x(1) * x(1));
  }
};

GTEST_TEST(testMathematicalProgram, sixHumpCamel) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2);
  auto cost = prog.AddCost(SixHumpCamelCost());

  prog.SetInitialGuess(x, Vector2d::Random());
  RunNonlinearProgram(prog, [&]() {
    // check (numerically) if it is a local minimum
    VectorXd ystar, y;
    const auto& x_value = prog.GetSolution(x);
    cost->Eval(x_value, ystar);
    for (int i = 0; i < 10; i++) {
      cost->Eval(x_value + .01 * Matrix<double, 2, 1>::Random(), y);
      if (y(0) < ystar(0)) throw std::runtime_error("not a local minima!");
    }
  });
}

class GloptipolyConstrainedExampleCost {
 public:
  static size_t numInputs() { return 3; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
    y(0) = -2 * x(0) + x(1) - x(2);
  }
};

class GloptipolyConstrainedExampleConstraint
    : public Constraint {  // want to also support deriving directly from
                           // constraint without going through drake::Function
 public:
  GloptipolyConstrainedExampleConstraint()
      : Constraint(
            1, 3, Vector1d::Constant(0),
            Vector1d::Constant(numeric_limits<double>::infinity())) {}

 protected:
  // for just these two types, implementing this locally is almost cleaner...
  void DoEval(const Eigen::Ref<const Eigen::VectorXd> &x,
              Eigen::VectorXd &y) const override {
    EvalImpl(x, y);
  }
  void DoEval(const Eigen::Ref<const TaylorVecXd> &x,
              TaylorVecXd &y) const override {
    EvalImpl(x, y);
  }

 private:
  template <typename ScalarType>
  void EvalImpl(const Ref<const Matrix<ScalarType, Dynamic, 1>>& x,
                // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                Matrix<ScalarType, Dynamic, 1>& y) const {
    y.resize(1);
    y(0) = 24 - 20 * x(0) + 9 * x(1) - 13 * x(2) + 4 * x(0) * x(0) -
           4 * x(0) * x(1) + 4 * x(0) * x(2) + 2 * x(1) * x(1) -
           2 * x(1) * x(2) + 2 * x(2) * x(2);
  }
};

// gloptiPolyConstrainedMinimization
// @brief from section 5.8.2 of the gloptipoly3 documentation
//
// Which is from section 3.5 in
//   Handbook of Test Problems in Local and Global Optimization
GTEST_TEST(testMathematicalProgram, gloptipolyConstrainedMinimization) {
  MathematicalProgram prog;

  // This test is run twice on different collections of continuous
  // variables to make sure that the solvers correctly handle mapping
  // variables to constraints/costs.
  auto x = prog.NewContinuousVariables(3);
  auto y = prog.NewContinuousVariables(3);
  prog.AddCost(GloptipolyConstrainedExampleCost(), x);
  prog.AddCost(GloptipolyConstrainedExampleCost(), y);
  std::shared_ptr<GloptipolyConstrainedExampleConstraint> qp_con(
      new GloptipolyConstrainedExampleConstraint());
  prog.AddConstraint(qp_con, x);
  prog.AddConstraint(qp_con, y);
  prog.AddLinearConstraint(Vector3d(1, 1, 1).transpose(),
                           -numeric_limits<double>::infinity(), 4, x);
  prog.AddLinearConstraint(Vector3d(1, 1, 1).transpose(),
                           -numeric_limits<double>::infinity(), 4, y);
  prog.AddLinearConstraint(Vector3d(0, 3, 1).transpose(),
                           -numeric_limits<double>::infinity(), 6, x);
  prog.AddLinearConstraint(Vector3d(0, 3, 1).transpose(),
                           -numeric_limits<double>::infinity(), 6, y);
  prog.AddBoundingBoxConstraint(
      Vector3d(0, 0, 0), Vector3d(2, numeric_limits<double>::infinity(), 3), x);
  prog.AddBoundingBoxConstraint(
      Vector3d(0, 0, 0), Vector3d(2, numeric_limits<double>::infinity(), 3), y);

  // IPOPT has difficulty with this problem depending on the initial
  // conditions, which is why the initial guess varies so little.
  Vector3d initial_guess = Vector3d(.5, 0, 3) + .01 * Vector3d::Random();
  prog.SetInitialGuess(x, initial_guess);
  prog.SetInitialGuess(y, initial_guess);
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    const auto& y_value = prog.GetSolution(y);
    EXPECT_TRUE(CompareMatrices(x_value, Vector3d(0.5, 0, 3), 1e-4,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(y_value, Vector3d(0.5, 0, 3), 1e-4,
                                MatrixCompareType::absolute));
  });
}

// This test is semantically equivalent with the above
// gloptipolyConstrainedMinimization test, but it uses the symbolic version of
// AddLinearConstraint method in MathematicalProgram class.
GTEST_TEST(testMathematicalProgram, gloptipolyConstrainedMinimizationSymbolic) {
  MathematicalProgram prog;

  // This test is run twice on different collections of continuous
  // variables to make sure that the solvers correctly handle mapping
  // variables to constraints/costs.
  auto x = prog.NewContinuousVariables(3);
  auto y = prog.NewContinuousVariables(3);
  prog.AddCost(GloptipolyConstrainedExampleCost(), x);
  prog.AddCost(GloptipolyConstrainedExampleCost(), y);
  std::shared_ptr<GloptipolyConstrainedExampleConstraint> qp_con(
      new GloptipolyConstrainedExampleConstraint());
  prog.AddConstraint(qp_con, x);
  prog.AddConstraint(qp_con, y);

  prog.AddLinearConstraint(x(0) + x(1) + x(2),
                           -numeric_limits<double>::infinity(), 4);
  prog.AddLinearConstraint(y(0) + y(1) + y(2),
                           -numeric_limits<double>::infinity(), 4);
  prog.AddLinearConstraint(3 * x(1) + x(2), -numeric_limits<double>::infinity(),
                           6);
  prog.AddLinearConstraint(3 * y(1) + y(2), -numeric_limits<double>::infinity(),
                           6);
  prog.AddBoundingBoxConstraint(
      Vector3d(0, 0, 0), Vector3d(2, numeric_limits<double>::infinity(), 3), x);
  prog.AddBoundingBoxConstraint(
      Vector3d(0, 0, 0), Vector3d(2, numeric_limits<double>::infinity(), 3), y);

  // IPOPT has difficulty with this problem depending on the initial
  // conditions, which is why the initial guess varies so little.
  Vector3d initial_guess{Vector3d(.5, 0, 3) + .01 * Vector3d::Random()};
  prog.SetInitialGuess(x, initial_guess);
  prog.SetInitialGuess(y, initial_guess);
  RunNonlinearProgram(prog, [&]() {
    const auto& x_value = prog.GetSolution(x);
    const auto& y_value = prog.GetSolution(y);
    EXPECT_TRUE(CompareMatrices(x_value, Vector3d(0.5, 0, 3), 1e-4,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(y_value, Vector3d(0.5, 0, 3), 1e-4,
                                MatrixCompareType::absolute));
  });
}

//
// Test that the Eval() method of LinearComplementarityConstraint correctly
// returns the slack.
GTEST_TEST(testMathematicalProgram, simpleLCPConstraintEval) {
  MathematicalProgram prog;
  Eigen::Matrix<double, 2, 2> M;

  // clang-format off
  M << 1, 0,
       0, 1;
  // clang-format on

  Eigen::Vector2d q(-1, -1);

  LinearComplementarityConstraint c(M, q);
  Eigen::VectorXd x;
  c.Eval(Eigen::Vector2d(1, 1), x);

  EXPECT_TRUE(
      CompareMatrices(x, Vector2d(0, 0), 1e-4, MatrixCompareType::absolute));
  c.Eval(Eigen::Vector2d(1, 2), x);

  EXPECT_TRUE(
      CompareMatrices(x, Vector2d(0, 1), 1e-4, MatrixCompareType::absolute));
}

// Simple linear complementarity problem example.
// @brief a hand-created LCP easily solved.
//
// Note: This test is meant to test that MathematicalProgram.Solve() works in
// this case; tests of the correctness of the Moby LCP solver itself live in
// testMobyLCP.
GTEST_TEST(testMathematicalProgram, simpleLCP) {
  MathematicalProgram prog;
  Eigen::Matrix<double, 2, 2> M;

  // clang-format off
  M << 1, 4,
       3, 1;
  // clang-format on

  Eigen::Vector2d q(-16, -15);

  auto x = prog.NewContinuousVariables<2>();

  prog.AddLinearComplementarityConstraint(M, q, x);
  EXPECT_NO_THROW(prog.Solve());
  const auto& x_value = prog.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(x_value, Vector2d(16, 0), 1e-4,
                              MatrixCompareType::absolute));
}

// Multiple LC constraints in a single optimization problem
// @brief Just two copies of the simpleLCP example, to make sure that the
// write-through of LCP results to the solution vector works correctly.
GTEST_TEST(testMathematicalProgram, multiLCP) {
  MathematicalProgram prog;
  Eigen::Matrix<double, 2, 2> M;

  // clang-format off
  M << 1, 4,
       3, 1;
  // clang-format on

  Eigen::Vector2d q(-16, -15);

  auto x = prog.NewContinuousVariables<2>();
  auto y = prog.NewContinuousVariables<2>();

  prog.AddLinearComplementarityConstraint(M, q, x);
  prog.AddLinearComplementarityConstraint(M, q, y);
  EXPECT_NO_THROW(prog.Solve());
  const auto& x_value = prog.GetSolution(x);
  const auto& y_value = prog.GetSolution(y);
  EXPECT_TRUE(CompareMatrices(x_value, Vector2d(16, 0), 1e-4,
                              MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(y_value, Vector2d(16, 0), 1e-4,
                              MatrixCompareType::absolute));
}

//
// Test that linear polynomial constraints get turned into linear constraints.
GTEST_TEST(testMathematicalProgram, linearPolynomialConstraint) {
  const Polynomiald x("x");
  MathematicalProgram problem;
  static const double kEpsilon = 1e-7;
  const auto x_var = problem.NewContinuousVariables(1);
  const std::vector<Polynomiald::VarType> var_mapping = {x.GetSimpleVariable()};
  std::shared_ptr<Constraint> resulting_constraint =
      problem.AddPolynomialConstraint(VectorXPoly::Constant(1, x), var_mapping,
                                      Vector1d::Constant(2),
                                      Vector1d::Constant(2));
  // Check that the resulting constraint is a LinearConstraint.
  EXPECT_NE(dynamic_cast<LinearConstraint*>(resulting_constraint.get()),
            nullptr);
  // Check that it gives the correct answer as well.
  problem.SetInitialGuessForAllVariables(drake::Vector1d(0));
  RunNonlinearProgram(problem, [&]() {
    EXPECT_NEAR(problem.GetSolution(x_var(0)), 2, kEpsilon);
  });
}

// The current windows CI build has no solver for generic constraints.  The
// DISABLED_ logic below ensures that we still at least get compile-time
// checking of the test and resulting template instantiations.
#if !defined(WIN32) && !defined(WIN64)
#define POLYNOMIAL_CONSTRAINT_TEST_NAME polynomialConstraint
#else
#define POLYNOMIAL_CONSTRAINT_TEST_NAME DISABLED_polynomialConstraint
#endif

// Simple test of polynomial constraints.
GTEST_TEST(testMathematicalProgram, POLYNOMIAL_CONSTRAINT_TEST_NAME) {
  static const double kInf = numeric_limits<double>::infinity();
  // Generic constraints in nlopt require a very generous epsilon.
  static const double kEpsilon = 1e-4;

  // Given a degenerate polynomial, get the trivial solution.
  {
    const Polynomiald x("x");
    MathematicalProgram problem;
    const auto x_var = problem.NewContinuousVariables(1);
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.GetSimpleVariable()};
    problem.AddPolynomialConstraint(VectorXPoly::Constant(1, x), var_mapping,
                                    Vector1d::Constant(2),
                                    Vector1d::Constant(2));
    problem.SetInitialGuessForAllVariables(drake::Vector1d::Zero());
    RunNonlinearProgram(problem, [&]() {
      EXPECT_NEAR(problem.GetSolution(x_var(0)), 2, kEpsilon);
      // TODO(ggould-tri) test this with a two-sided constraint, once
      // the nlopt wrapper supports those.
    });
  }

  // Given a small univariate polynomial, find a low point.
  {
    const Polynomiald x("x");
    const Polynomiald poly = (x - 1) * (x - 1);
    MathematicalProgram problem;
    const auto x_var = problem.NewContinuousVariables(1);
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.GetSimpleVariable()};
    problem.AddPolynomialConstraint(VectorXPoly::Constant(1, poly), var_mapping,
                                    Eigen::VectorXd::Zero(1),
                                    Eigen::VectorXd::Zero(1));
    problem.SetInitialGuessForAllVariables(drake::Vector1d::Zero());
    RunNonlinearProgram(problem, [&]() {
      EXPECT_NEAR(problem.GetSolution(x_var(0)), 1, 0.2);
      EXPECT_LE(poly.EvaluateUnivariate(problem.GetSolution(x_var(0))),
                kEpsilon);
    });
  }

  // Given a small multivariate polynomial, find a low point.
  {
    const Polynomiald x("x");
    const Polynomiald y("y");
    const Polynomiald poly = (x - 1) * (x - 1) + (y + 2) * (y + 2);
    MathematicalProgram problem;
    const auto xy_var = problem.NewContinuousVariables(2);
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.GetSimpleVariable(), y.GetSimpleVariable()};
    problem.AddPolynomialConstraint(VectorXPoly::Constant(1, poly), var_mapping,
                                    Eigen::VectorXd::Zero(1),
                                    Eigen::VectorXd::Zero(1));
    problem.SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
    RunNonlinearProgram(problem, [&]() {
      EXPECT_NEAR(problem.GetSolution(xy_var(0)), 1, 0.2);
      EXPECT_NEAR(problem.GetSolution(xy_var(1)), -2, 0.2);
      std::map<Polynomiald::VarType, double> eval_point = {
          {x.GetSimpleVariable(), problem.GetSolution(xy_var(0))},
          {y.GetSimpleVariable(), problem.GetSolution(xy_var(1))}};
      EXPECT_LE(poly.EvaluateMultivariate(eval_point), kEpsilon);
    });
  }

  // Given two polynomial constraints, satisfy both.
  {
    // (x^4 - x^2 + 0.2 has two minima, one at 0.5 and the other at -0.5;
    // constrain x < 0 and EXPECT that the solver finds the negative one.)
    const Polynomiald x("x");
    const Polynomiald poly = x * x * x * x - x * x + 0.2;
    MathematicalProgram problem;
    const auto x_var = problem.NewContinuousVariables(1);
    problem.SetInitialGuess(x_var, Vector1d::Constant(-0.1));
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.GetSimpleVariable()};
    VectorXPoly polynomials_vec(2, 1);
    polynomials_vec << poly, x;
    problem.AddPolynomialConstraint(polynomials_vec, var_mapping,
                                    Eigen::VectorXd::Constant(2, -kInf),
                                    Eigen::VectorXd::Zero(2));
    RunNonlinearProgram(problem, [&]() {
      EXPECT_NEAR(problem.GetSolution(x_var(0)), -0.7, 0.2);
      EXPECT_LE(poly.EvaluateUnivariate(problem.GetSolution(x_var(0))),
                kEpsilon);
    });
  }
}

//
// Test how an unconstrained QP is dispatched and solved:
//   - on the problem (x1 - 1)^2 + (x2 - 1)^2, with a min at
//     at (x1=1, x2=1).
//   - on the same problem plus the additional problem
//     (2*x2 - 5)^2 + (2*x3 - 2)^2, which, when combined
//     with the first problem, has min at (x1=1, x2=2, x3=1)
// The first case tests a single quadratic cost, and the
// second case tests multiple quadratic costs affecting
// different variable views. All fall under the
// umbrella of the Equality Constrained QP Solver.
GTEST_TEST(testMathematicalProgram, testUnconstrainedQPDispatch) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  MatrixXd Q(2, 2);
  // clang-format off
  Q << 1.0, 0.0,
       0.0, 1.0;
  // clang-format on
  VectorXd c(2);
  c << -1.0, -1.0;

  prog.AddQuadraticCost(Q, c);

  prog.SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
  prog.Solve();

  VectorXd expected_answer(2);
  expected_answer << 1.0, 1.0;
  auto x_value = prog.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, 1e-10,
                              MatrixCompareType::absolute));
  // There are no inequality constraints, and only quadratic costs,
  // so this should hold:
  CheckSolverType(prog, "Equality Constrained QP Solver");

  // Add one more variable and constrain a view into them.
  auto y = prog.NewContinuousVariables<1>("y");
  Q << 2.0, 0.0, 0.0, 2.0;
  c << -5.0, -2.0;
  VariableRefList vars;
  vars.push_back(x.segment<1>(1));
  vars.push_back(y);

  prog.AddQuadraticCost(Q, c, vars);
  prog.SetInitialGuessForAllVariables(Eigen::Vector3d::Zero());
  prog.Solve();
  expected_answer.resize(3);
  expected_answer << 1.0, 2.0, 1.0;
  VectorXd actual_answer(3);
  x_value = prog.GetSolution(x);
  const auto& y_value = prog.GetSolution(y);
  actual_answer << x_value, y_value;
  EXPECT_TRUE(CompareMatrices(expected_answer, actual_answer, 1e-10,
                              MatrixCompareType::absolute))
      << "\tExpected: " << expected_answer.transpose()
      << "\tActual: " << actual_answer.transpose();

  // Problem still has only quadratic costs, so solver should be the same.
  CheckSolverType(prog, "Equality Constrained QP Solver");
}

// Test how an equality-constrained QP is dispatched
//   - on the problem (x1 - 1)^2 + (x2 - 1)^2, with a min at
//     at (x1=1, x2=1), constrained with (x1 + x2 = 1).
//     The resulting constrained min is at (x1=0.5, x2=0.5).
//   - on the same problem with an additional variable x3,
//     with (2*x1 - x3 = 0). Resulting solution should be
//     (x1=0.5, x2=0.5, x3=1.0)
GTEST_TEST(testMathematicalProgram, testLinearlyConstrainedQPDispatch) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2);
  MatrixXd Q(2, 2);
  Q << 1, 0.0, 0.0, 1.0;
  VectorXd c(2);
  c << -1.0, -1.0;

  prog.AddQuadraticCost(Q, c);

  VectorXd constraint1(2);
  // x1 + x2 = 1
  constraint1 << 1, 1;
  prog.AddLinearEqualityConstraint(constraint1.transpose(), 1.0);

  prog.SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
  prog.Solve();

  VectorXd expected_answer(2);
  expected_answer << 0.5, 0.5;
  auto x_value = prog.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, 1e-10,
                              MatrixCompareType::absolute));

  // This problem is now an Equality Constrained QP and should
  // use this solver:
  CheckSolverType(prog, "Equality Constrained QP Solver");

  // Add one more variable and constrain it in a different way
  auto y = prog.NewContinuousVariables(1);
  Vector2d constraint2(2);
  constraint2 << 2., -1.;
  // 2*x1 - x3 = 0, so x3 should wind up as 1.0
  VariableRefList vars;
  vars.push_back(x.segment(0, 1));
  vars.push_back(y);

  prog.AddLinearEqualityConstraint(constraint2.transpose(), 0.0, vars);
  prog.SetInitialGuessForAllVariables(Eigen::Vector3d::Zero());
  prog.Solve();
  expected_answer.resize(3);
  expected_answer << 0.5, 0.5, 1.0;
  VectorXd actual_answer(3);
  x_value = prog.GetSolution(x);
  auto y_value = prog.GetSolution(y);
  actual_answer << x_value, y_value;
  EXPECT_TRUE(CompareMatrices(expected_answer, actual_answer, 1e-10,
                              MatrixCompareType::absolute))
      << "\tExpected: " << expected_answer.transpose()
      << "\tActual: " << actual_answer.transpose();
}

// Solve an SOCP with Lorentz cone and rotated Lorentz cone constraint as a
// nonlinear optimization problem.
// The objective is to find the smallest distance from a hyperplane
// A * x = b to the origin.
// We can solve the following SOCP with Lorentz cone constraint
// min  t
//  s.t t >= sqrt(x'*x)
//      A * x = b.
// Alternatively, we can solve the following SOCP with rotated Lorentz cone
// constraint
// min t
// s.t t >= x'*x
//     A * x = b.
//
// The optimal solution of this equality constrained QP can be found using
// Lagrangian method. The optimal solution x* and Lagrangiam multiplier z*
// satisfy
// A_hat * [x*; z*] = [b; 0]
// where A_hat = [A 0; 2*I A'].
void MinDistanceFromPlaneToOrigin(const MatrixXd& A, const VectorXd b) {
  DRAKE_ASSERT(A.rows() == b.rows());
  const int xDim = A.cols();
  MathematicalProgram prog_lorentz;
  auto t_lorentz = prog_lorentz.NewContinuousVariables(1, "t");
  auto x_lorentz = prog_lorentz.NewContinuousVariables(xDim, "x");
  prog_lorentz.AddLorentzConeConstraint({t_lorentz, x_lorentz});
  prog_lorentz.AddLinearEqualityConstraint(A, b, x_lorentz);
  prog_lorentz.AddLinearCost(drake::Vector1d(1.0), t_lorentz);

  // A_hat = [A 0; 2*I A']
  MatrixXd A_hat(A.rows() + A.cols(), A.rows() + A.cols());
  A_hat.topLeftCorner(A.rows(), A.cols()) = A;
  A_hat.topRightCorner(A.rows(), A.rows()) = MatrixXd::Zero(A.rows(), A.rows());
  A_hat.bottomLeftCorner(A.cols(), A.cols()) =
      2 * MatrixXd::Identity(A.cols(), A.cols());
  A_hat.bottomRightCorner(A.cols(), A.rows()) = A.transpose();
  VectorXd b_hat(A.rows() + A.cols());
  b_hat << b, VectorXd::Zero(A.cols());
  VectorXd xz_expected = A_hat.colPivHouseholderQr().solve(b_hat);
  VectorXd x_expected = xz_expected.head(xDim);

  double cost_expected_lorentz = x_expected.norm();
  // NLopt needs a really good starting point to solve SOCP, while SNOPT and
  // IPOPT seems OK with starting point not so close to optimal solution.
  prog_lorentz.SetInitialGuess(t_lorentz,
                               drake::Vector1d(cost_expected_lorentz + 0.1));
  VectorXd x_lorentz_guess = x_expected + 0.1 * VectorXd::Ones(xDim);
  prog_lorentz.SetInitialGuess(x_lorentz, x_lorentz_guess);
  RunNonlinearProgram(prog_lorentz, [&]() {
    const auto& x_lorentz_value = prog_lorentz.GetSolution(x_lorentz);
    EXPECT_TRUE(CompareMatrices(x_lorentz_value, x_expected, 1E-5,
                                MatrixCompareType::absolute));
    const auto& t_lorentz_value = prog_lorentz.GetSolution(t_lorentz);
    EXPECT_NEAR(cost_expected_lorentz, t_lorentz_value(0), 1E-3);
  });

  MathematicalProgram prog_rotated_lorentz;
  auto t_rotated_lorentz = prog_rotated_lorentz.NewContinuousVariables(1, "t");
  auto x_rotated_lorentz =
      prog_rotated_lorentz.NewContinuousVariables(xDim, "x");
  auto slack_rotated_lorentz =
      prog_rotated_lorentz.NewContinuousVariables<1>("slack");
  prog_rotated_lorentz.AddRotatedLorentzConeConstraint(
      {t_rotated_lorentz, slack_rotated_lorentz, x_rotated_lorentz});
  prog_rotated_lorentz.AddLinearEqualityConstraint(A, b, x_rotated_lorentz);
  prog_rotated_lorentz.AddBoundingBoxConstraint(1.0, 1.0,
                                                slack_rotated_lorentz(0));
  prog_rotated_lorentz.AddLinearCost(drake::Vector1d(1.0), t_rotated_lorentz);

  double cost_expected_rotated_lorentz = x_expected.squaredNorm();
  // NLopt needs a really good starting point to solve SOCP, while SNOPT and
  // IPOPT seems OK with starting point not so close to optimal solution.
  prog_rotated_lorentz.SetInitialGuess(
      t_rotated_lorentz, drake::Vector1d(cost_expected_rotated_lorentz + 0.1));
  prog_rotated_lorentz.SetInitialGuess(slack_rotated_lorentz,
                                       drake::Vector1d(1.0));
  VectorXd x_rotated_lorentz_guess = x_expected + 0.1 * VectorXd::Ones(xDim);
  prog_rotated_lorentz.SetInitialGuess(x_rotated_lorentz,
                                       x_rotated_lorentz_guess);
  RunNonlinearProgram(prog_rotated_lorentz, [&]() {
    const auto& x_rotated_lorentz_value =
        prog_rotated_lorentz.GetSolution(x_rotated_lorentz);
    EXPECT_TRUE(CompareMatrices(x_rotated_lorentz_value, x_expected, 1E-5,
                                MatrixCompareType::absolute));
    const auto& t_rotated_lorentz_value =
        prog_rotated_lorentz.GetSolution(t_rotated_lorentz);
    EXPECT_NEAR(cost_expected_rotated_lorentz, t_rotated_lorentz_value(0),
                1E-3);
  });

  // Now add a constraint x'*x <= 2*x_expected'*x_expected to the problem.
  // The optimal solution and the costs are still the same, but now we test
  // Lorentz cone (rotated Lorentz cone) constraints with generic nonlinear
  // constraints.
  std::shared_ptr<QuadraticConstraint> quadratic_constraint(
      new QuadraticConstraint(MatrixXd::Identity(xDim, xDim),
                              VectorXd::Zero(xDim), 0,
                              x_expected.squaredNorm()));

  prog_lorentz.AddConstraint(quadratic_constraint, x_lorentz);
  RunNonlinearProgram(prog_lorentz, [&]() {
    const auto& x_lorentz_value = prog_lorentz.GetSolution(x_lorentz);
    EXPECT_TRUE(CompareMatrices(x_lorentz_value, x_expected, 1E-5,
                                MatrixCompareType::absolute));
    const auto& t_lorentz_value = prog_lorentz.GetSolution(t_lorentz);
    EXPECT_NEAR(cost_expected_lorentz, t_lorentz_value(0), 1E-3);
  });

  prog_rotated_lorentz.AddConstraint(quadratic_constraint, x_rotated_lorentz);
  RunNonlinearProgram(prog_rotated_lorentz, [&]() {
    const auto& x_rotated_lorentz_value =
        prog_rotated_lorentz.GetSolution(x_rotated_lorentz);
    EXPECT_TRUE(CompareMatrices(x_rotated_lorentz_value, x_expected, 1E-5,
                                MatrixCompareType::absolute));
    const auto& t_rotated_lorentz_value =
        prog_rotated_lorentz.GetSolution(t_rotated_lorentz);
    EXPECT_NEAR(cost_expected_rotated_lorentz, t_rotated_lorentz_value(0),
                1E-3);
  });
}

GTEST_TEST(testMathematicalProgram, testSolveSOCPasNLP) {
  MatrixXd A = Matrix<double, 1, 2>::Ones();
  VectorXd b = drake::Vector1d(2);
  MinDistanceFromPlaneToOrigin(A, b);

  A = Matrix<double, 2, 3>::Zero();
  A << 0, 1, 2, -1, 2, 3;
  b = Vector2d(1.0, 3.0);
  MinDistanceFromPlaneToOrigin(A, b);
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic1) {
  // Add Linear Constraint: -10 <= 3 - 5*x0 + 10*x2 - 7*y1 <= 10
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  auto y = prog.NewContinuousVariables(3, "y");
  const symbolic::Expression e{3 - 5 * x(0) + 10 * x(2) - 7 * y(1)};
  const double lb{-10};
  const double ub{+10};
  const auto binding = prog.AddLinearConstraint(e, lb, ub);

  // Check if the binding includes the correct linear constraint.
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.constraint();
  EXPECT_EQ(constraint_ptr->num_constraints(), 1u);
  const symbolic::Expression Ax{(constraint_ptr->A() * var_vec)(0, 0)};
  const symbolic::Expression lb_in_ctr{constraint_ptr->lower_bound()[0]};
  const symbolic::Expression ub_in_ctr{constraint_ptr->upper_bound()[0]};
  EXPECT_TRUE((e - lb).EqualTo(Ax - lb_in_ctr));
  EXPECT_TRUE((e - ub).EqualTo(Ax - ub_in_ctr));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic2) {
  // Add Linear Constraint: -10 <= x0 <= 10
  // Note that this constraint is a bounding-box constraint which is a sub-class
  // of linear-constraint.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  const symbolic::Expression e{x(0)};
  const auto binding = prog.AddLinearConstraint(e, -10, 10);

  // Check that the constraint in the binding is of BoundingBoxConstraint by
  // using dynamic_pointer_cast.
  const std::shared_ptr<BoundingBoxConstraint> constraint_ptr{
      std::dynamic_pointer_cast<BoundingBoxConstraint>(binding.constraint())};
  EXPECT_TRUE(constraint_ptr != nullptr);
  EXPECT_EQ(constraint_ptr->num_constraints(), 1u);

  // Check if the binding includes the correct linear constraint.
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const symbolic::Expression Ax{(constraint_ptr->A() * var_vec)(0, 0)};
  const symbolic::Expression lb_in_ctr{constraint_ptr->lower_bound()[0]};
  const symbolic::Expression ub_in_ctr{constraint_ptr->upper_bound()[0]};
  EXPECT_TRUE((e - -10).EqualTo(Ax - lb_in_ctr));
  EXPECT_TRUE((e - 10).EqualTo(Ax - ub_in_ctr));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic3) {
  // Add Linear Constraints
  //     3 <=  3 - 5*x0 +      + 10*x2        - 7*y1        <= 9
  //   -10 <=                       x2                      <= 10
  //    -7 <= -5 + 2*x0 + 3*x2         + 3*y0 - 2*y1 + 6*y2 <= 12
  //
  // Note: the second constraint, -10 <= x2 <= 10 is actually a bounding-box
  // constraint but We still process the three symbolic-constraints into a
  // single linear-constraint whose coefficient matrix is the following.
  //
  //         [-5 0 10 0 -7 0]
  //         [ 0 0  1 0  0 0]
  //         [ 2 3  0 3 -2 6]
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  auto y = prog.NewContinuousVariables(3, "y");
  Matrix<symbolic::Expression, 3, 1> M_e;
  Vector3d M_lb;
  Vector3d M_ub;

  // clang-format off
  M_e  <<  3 - 5 * x(0) + 10 * x(2) - 7 * y(1),
          +x(2),
          -5 + 2 * x(0) + 3 * x(2) + 3 * y(0) - 2 * y(1) + 6 * y(2);
  M_lb <<  3,
         -10,
           7;
  M_ub << -7,
          10,
          12;
  // clang-format on

  // Check if the binding includes the correct linear constraint.
  const auto binding = prog.AddLinearConstraint(M_e, M_lb, M_ub);
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.constraint();
  EXPECT_EQ(constraint_ptr->num_constraints(), 3u);
  const auto Ax = constraint_ptr->A() * var_vec;
  const auto lb_in_ctr = constraint_ptr->lower_bound();
  const auto ub_in_ctr = constraint_ptr->upper_bound();

  EXPECT_EQ(M_e - M_lb, Ax - lb_in_ctr);
  EXPECT_EQ(M_e - M_ub, Ax - ub_in_ctr);
}

}  // namespace
}  // namespace solvers
}  // namespace drake
