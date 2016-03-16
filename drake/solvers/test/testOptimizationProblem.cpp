#include <typeinfo>
#include "drake/solvers/MathematicalProgram.h"
#include "drake/solvers/NloptSolver.h"
#include "drake/solvers/Optimization.h"
#include "drake/solvers/SnoptSolver.h"
#include "drake/util/eigen_matrix_compare.h"
#include "drake/util/testUtil.h"
#include "gtest/gtest.h"

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

using Drake::Constraint;
using Drake::TaylorVecXd;
using Drake::VecIn;
using Drake::Vector1d;
using Drake::VecOut;
using Drake::MathematicalProgramSolverInterface;
using Drake::NloptSolver;
using Drake::OptimizationProblem;
using Drake::BoundingBoxConstraint;
using Drake::SnoptSolver;
using Drake::LinearComplementarityConstraint;
using drake::util::MatrixCompareType;

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

TEST(testOptimizationProblem, testAddFunction) {
  OptimizationProblem prog;
  prog.addContinuousVariables(1);

  Movable movable;
  prog.addCost(std::move(movable));
  prog.addCost(Movable());

  Copyable copyable;
  prog.addCost(copyable);

  Unique unique;
  prog.addCost(std::cref(unique));
  prog.addCost(std::make_shared<Unique>());
  prog.addCost(std::unique_ptr<Unique>(new Unique));
}

void runNonlinearProgram(OptimizationProblem& prog,
                         std::function<void ()> test_func) {
  NloptSolver nlopt_solver;
  SnoptSolver snopt_solver;

  std::pair<const char*, MathematicalProgramSolverInterface*> solvers[] = {
    std::make_pair("SNOPT", &snopt_solver),
    std::make_pair("NLopt", &nlopt_solver)
  };

  for (const auto& solver : solvers) {
    if (!solver.second->available()) { continue; }
    ASSERT_NO_THROW(solver.second->solve(prog)) <<
        "Using solver: " << solver.first;
    EXPECT_NO_THROW(test_func()) << "Using solver: " << solver.first;
  }
}

TEST(testOptimizationProblem, trivialLeastSquares) {
  OptimizationProblem prog;

  auto const& x = prog.addContinuousVariables(4);

  auto x2 = x(2);
  auto xhead = x.head(3);

  Vector4d b = Vector4d::Random();
  auto con = prog.addLinearEqualityConstraint(Matrix4d::Identity(), b, {x});

  prog.solve();
  EXPECT_TRUE(
      CompareMatrices(b, x.value(), 1e-10, MatrixCompareType::absolute));

  valuecheck(b(2), x2.value()(0), 1e-10);
  EXPECT_TRUE(CompareMatrices(b.head(3), xhead.value(), 1e-10,
                              MatrixCompareType::absolute));

  valuecheck(b(2), xhead(2).value()(0), 1e-10);  // a segment of a segment

  auto const& y = prog.addContinuousVariables(2);
  prog.addLinearEqualityConstraint(2 * Matrix2d::Identity(), b.topRows(2), {y});
  prog.solve();
  EXPECT_TRUE(CompareMatrices(b.topRows(2) / 2, y.value(), 1e-10,
                              MatrixCompareType::absolute));

  EXPECT_TRUE(
      CompareMatrices(b, x.value(), 1e-10, MatrixCompareType::absolute));

  con->updateConstraint(3 * Matrix4d::Identity(), b);
  prog.solve();
  EXPECT_TRUE(CompareMatrices(b.topRows(2) / 2, y.value(), 1e-10,
                              MatrixCompareType::absolute));

  EXPECT_TRUE(
      CompareMatrices(b / 3, x.value(), 1e-10, MatrixCompareType::absolute));

  std::shared_ptr<BoundingBoxConstraint> bbcon(new BoundingBoxConstraint(
      MatrixXd::Constant(2, 1, -1000.0), MatrixXd::Constant(2, 1, 1000.0)));
  prog.addBoundingBoxConstraint(bbcon, {x.head(2)});

  // Now solve as a nonlinear program.
  runNonlinearProgram(prog, [&]() {
      EXPECT_TRUE(CompareMatrices(b.topRows(2) / 2, y.value(), 1e-10,
                                  MatrixCompareType::absolute));
      EXPECT_TRUE(
          CompareMatrices(b / 3, x.value(), 1e-10, MatrixCompareType::absolute));
    });
}

// This test comes from Section 2.2 of "Handbook of Test Problems in
// Local and Global Optimization"
class TestProblem1Objective {
 public:
  static size_t numInputs() { return 5; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    assert(x.rows() == numInputs());
    assert(y.rows() == numOutputs());
    y(0) =
        (-50.0 * x(0) * x(0)) + (42 * x(0)) - (50.0 * x(1) * x(1)) +
        (44 * x(1)) - (50.0 * x(2) * x(2)) + (45 * x(2)) -
        (50.0 * x(3) * x(3)) + (47 * x(3)) - (50.0 * x(4) * x(4)) +
        (47.5 * x(4));
  }
};

TEST(testOptimizationProblem, testProblem1) {
  OptimizationProblem prog;
  auto x = prog.addContinuousVariables(5);
  prog.addCost(TestProblem1Objective());
  VectorXd constraint(5);
  constraint << 20, 12, 11, 7, 4;
  prog.addLinearConstraint(
      constraint.transpose(),
      Drake::Vector1d::Constant(-std::numeric_limits<double>::infinity()),
      Drake::Vector1d::Constant(40));
  prog.addBoundingBoxConstraint(
      MatrixXd::Constant(5, 1, 0), MatrixXd::Constant(5, 1, 1));
  VectorXd expected(5);
  expected << 1, 1, 0, 1, 0;
  prog.setInitialGuess({x}, expected + .2 * VectorXd::Random(5));
  runNonlinearProgram(prog, [&]() {
      EXPECT_TRUE(CompareMatrices(x.value(), expected, 1e-10,
                                  MatrixCompareType::absolute));
    });
}

class SixHumpCamelObjective {
 public:
  static size_t numInputs() { return 2; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    assert(x.rows() == numInputs());
    assert(y.rows() == numOutputs());
    y(0) =
        x(0) * x(0) * (4 - 2.1 * x(0) * x(0) + x(0) * x(0) * x(0) * x(0) / 3) +
        x(0) * x(1) + x(1) * x(1) * (-4 + 4 * x(1) * x(1));
  }
};

TEST(testOptimizationProblem, sixHumpCamel) {
  OptimizationProblem prog;
  auto x = prog.addContinuousVariables(2);
  auto objective = prog.addCost(SixHumpCamelObjective());

  runNonlinearProgram(prog, [&]() {
      // check (numerically) if it is a local minimum
      VectorXd ystar, y;
      objective->eval(x.value(), ystar);
      for (int i = 0; i < 10; i++) {
        objective->eval(x.value() + .01 * Matrix<double, 2, 1>::Random(), y);
        if (y(0) < ystar(0)) throw std::runtime_error("not a local minima!");
      }
    });
}

class GloptipolyConstrainedExampleObjective {
 public:
  static size_t numInputs() { return 3; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    assert(x.rows() == numInputs());
    assert(y.rows() == numOutputs());
    y(0) = -2 * x(0) + x(1) - x(2);
  }
};

class GloptipolyConstrainedExampleConstraint
    : public Constraint {  // want to also support deriving directly from
                           // constraint without going through Drake::Function
 public:
  GloptipolyConstrainedExampleConstraint()
      : Constraint(1, Vector1d::Constant(0),
                   Vector1d::Constant(std::numeric_limits<double>::infinity())) {}

  // for just these two types, implementing this locally is almost cleaner...
  virtual void eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override {
    evalImpl(x, y);
  }
  virtual void eval(const Eigen::Ref<const TaylorVecXd>& x,
                    TaylorVecXd& y) const override {
    evalImpl(x, y);
  }

  template <typename ScalarType>
  void evalImpl(const Ref<const Matrix<ScalarType, Dynamic, 1>>& x,
                Matrix<ScalarType, Dynamic, 1>& y) const {
    y.resize(1);
    y(0) = 24 - 20 * x(0) + 9 * x(1) - 13 * x(2) + 4 * x(0) * x(0) -
           4 * x(0) * x(1) + 4 * x(0) * x(2) + 2 * x(1) * x(1) -
           2 * x(1) * x(2) + 2 * x(2) * x(2);
  }
};

/** gloptiPolyConstrainedMinimization
 * @brief from section 5.8.2 of the gloptipoly3 documentation
 *
 * Which is from section 3.5 in
 *   Handbook of Test Problems in Local and Global Optimization
 */
TEST(testOptimizationProblem, gloptipolyConstrainedMinimization) {
  OptimizationProblem prog;
  auto x = prog.addContinuousVariables(3);
  prog.addCost(GloptipolyConstrainedExampleObjective());
  std::shared_ptr<GloptipolyConstrainedExampleConstraint> qp_con(
      new GloptipolyConstrainedExampleConstraint());
  prog.addGenericConstraint(qp_con, {x});
  prog.addLinearConstraint(
      Vector3d(1, 1, 1).transpose(),
      Vector1d::Constant(-std::numeric_limits<double>::infinity()),
      Vector1d::Constant(4));
  prog.addLinearConstraint(
      Vector3d(0, 3, 1).transpose(),
      Vector1d::Constant(-std::numeric_limits<double>::infinity()),
      Vector1d::Constant(6));
  prog.addBoundingBoxConstraint(
      Vector3d(0, 0, 0),
      Vector3d(2, std::numeric_limits<double>::infinity(), 3));

  prog.setInitialGuess({x}, Vector3d(.5, 0, 3) + .1 * Vector3d::Random());
  runNonlinearProgram(prog, [&]() {
      EXPECT_TRUE(CompareMatrices(x.value(), Vector3d(0.5, 0, 3), 1e-4,
                                  MatrixCompareType::absolute));
    });
}

/**
 * Test that the eval() method of LinearComplementarityConstraint correctly
 * returns the slack.
 */
TEST(testOptimizationProblem, simpleLCPConstraintEval) {
  OptimizationProblem prog;
  Eigen::Matrix<double, 2, 2> M;

  // clang-format off
  M << 1, 0,
       0, 1;
  // clang-format on

  Eigen::Vector2d q(-1, -1);

  LinearComplementarityConstraint c(M, q);
  Eigen::VectorXd x;
  c.eval(Eigen::Vector2d(1, 1), x);

  EXPECT_TRUE(
      CompareMatrices(x, Vector2d(0, 0), 1e-4, MatrixCompareType::absolute));
  c.eval(Eigen::Vector2d(1, 2), x);

  EXPECT_TRUE(
      CompareMatrices(x, Vector2d(0, 1), 1e-4, MatrixCompareType::absolute));
}

/** Simple linear complementarity problem example.
 * @brief a hand-created LCP easily solved.
 *
 * Note: This test is meant to test that OptimizationProblem.solve() works in
 * this case; tests of the correctness of the Moby LCP solver itself live in
 * testMobyLCP.
 */
TEST(testOptimizationProblem, simpleLCP) {
  OptimizationProblem prog;
  Eigen::Matrix<double, 2, 2> M;

  // clang-format off
  M << 1, 4,
       3, 1;
  // clang-format on

  Eigen::Vector2d q(-16, -15);

  auto x = prog.addContinuousVariables(2);

  prog.addLinearComplementarityConstraint(M, q, {x});
  EXPECT_NO_THROW(prog.solve());
  EXPECT_TRUE(CompareMatrices(x.value(), Vector2d(16, 0), 1e-4,
                              MatrixCompareType::absolute));
}

/** Multiple LC constraints in a single optimization problem
 * @brief Just two copies of the simpleLCP example, to make sure that the
 * write-through of LCP results to the solution vector works correctly.
 */
TEST(testOptimizationProblem, multiLCP) {
  OptimizationProblem prog;
  Eigen::Matrix<double, 2, 2> M;

  // clang-format off
  M << 1, 4,
       3, 1;
  // clang-format on

  Eigen::Vector2d q(-16, -15);

  auto x = prog.addContinuousVariables(2);
  auto y = prog.addContinuousVariables(2);

  prog.addLinearComplementarityConstraint(M, q, {x});
  prog.addLinearComplementarityConstraint(M, q, {y});
  EXPECT_NO_THROW(prog.solve());

  EXPECT_TRUE(CompareMatrices(x.value(), Vector2d(16, 0), 1e-4,
                              MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(y.value(), Vector2d(16, 0), 1e-4,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace solvers
}  // namespace drake
