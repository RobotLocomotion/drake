#include <typeinfo>

#include "drake/common/drake_assert.h"
#include "drake/solvers/IpoptSolver.h"
#include "drake/solvers/MathematicalProgram.h"
#include "drake/solvers/NloptSolver.h"
#include "drake/solvers/Optimization.h"
#include "drake/solvers/SnoptSolver.h"
#include "drake/util/Polynomial.h"
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

using Drake::TaylorVecXd;
using Drake::VecIn;
using Drake::Vector1d;
using Drake::VecOut;
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
// TODO(naveenoid) : tests need to be purged of Random initializations.

GTEST_TEST(testOptimizationProblem, testAddFunction) {
  OptimizationProblem prog;
  prog.AddContinuousVariables(1);

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

void RunNonlinearProgram(OptimizationProblem& prog,
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

GTEST_TEST(testOptimizationProblem, trivialLeastSquares) {
  OptimizationProblem prog;

  auto const& x = prog.AddContinuousVariables(4);

  auto x2 = x(2);
  auto xhead = x.head(3);

  Vector4d b = Vector4d::Random();
  auto con = prog.AddLinearEqualityConstraint(Matrix4d::Identity(), b, {x});

  prog.Solve();
  EXPECT_TRUE(
      CompareMatrices(b, x.value(), 1e-10, MatrixCompareType::absolute));

  valuecheck(b(2), x2.value()(0), 1e-10);
  EXPECT_TRUE(CompareMatrices(b.head(3), xhead.value(), 1e-10,
                              MatrixCompareType::absolute));

  valuecheck(b(2), xhead(2).value()(0), 1e-10);  // a segment of a segment.

  auto const& y = prog.AddContinuousVariables(2);
  prog.AddLinearEqualityConstraint(2 * Matrix2d::Identity(), b.topRows(2), {y});
  prog.Solve();
  EXPECT_TRUE(CompareMatrices(b.topRows(2) / 2, y.value(), 1e-10,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(b, x.value(), 1e-10, MatrixCompareType::absolute));

  con->updateConstraint(3 * Matrix4d::Identity(), b);
  prog.Solve();
  EXPECT_TRUE(CompareMatrices(b.topRows(2) / 2, y.value(), 1e-10,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(b / 3, x.value(), 1e-10, MatrixCompareType::absolute));

  std::shared_ptr<BoundingBoxConstraint> bbcon(new BoundingBoxConstraint(
      MatrixXd::Constant(2, 1, -1000.0), MatrixXd::Constant(2, 1, 1000.0)));
  prog.AddBoundingBoxConstraint(bbcon, {x.head(2)});

  // Now solve as a nonlinear program.
  RunNonlinearProgram(prog, [&]() {
    EXPECT_TRUE(CompareMatrices(b.topRows(2) / 2, y.value(), 1e-10,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(
        CompareMatrices(b / 3, x.value(), 1e-10, MatrixCompareType::absolute));
  });
}

GTEST_TEST(testOptimizationProblem, trivialLinearEquality) {
  OptimizationProblem prog;

  auto vars = prog.AddContinuousVariables(2);

  // Use a non-square matrix to catch row/column mistakes in the solvers.
  prog.AddLinearEqualityConstraint(Vector2d(0, 1).transpose(),
                                   Vector1d::Constant(1));
  prog.SetInitialGuess(vars, Vector2d(2, 2));
  RunNonlinearProgram(prog, [&]() {
    EXPECT_DOUBLE_EQ(vars.value()(0), 2);
    EXPECT_DOUBLE_EQ(vars.value()(1), 1);
  });
}

// This test comes from Section 2.2 of "Handbook of Test Problems in
// Local and Global Optimization."
class TestProblem1Cost {
 public:
  static size_t numInputs() { return 5; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(x.rows() == numInputs());
    DRAKE_ASSERT(y.rows() == numOutputs());
    y(0) = (-50.0 * x(0) * x(0)) + (42 * x(0)) - (50.0 * x(1) * x(1)) +
           (44 * x(1)) - (50.0 * x(2) * x(2)) + (45 * x(2)) -
           (50.0 * x(3) * x(3)) + (47 * x(3)) - (50.0 * x(4) * x(4)) +
           (47.5 * x(4));
  }
};

GTEST_TEST(testOptimizationProblem, testProblem1) {
  OptimizationProblem prog;
  auto x = prog.AddContinuousVariables(5);
  prog.AddCost(TestProblem1Cost());
  VectorXd constraint(5);
  constraint << 20, 12, 11, 7, 4;
  prog.AddLinearConstraint(
      constraint.transpose(),
      Vector1d::Constant(-std::numeric_limits<double>::infinity()),
      Vector1d::Constant(40));
  prog.AddBoundingBoxConstraint(MatrixXd::Constant(5, 1, 0),
                                MatrixXd::Constant(5, 1, 1));
  VectorXd expected(5);
  expected << 1, 1, 0, 1, 0;

  // IPOPT has difficulty with this problem depending on the initial
  // conditions, which is why the initial guess varies so little.
  std::srand(0);
  prog.SetInitialGuess({x}, expected + .01 * VectorXd::Random(5));
  RunNonlinearProgram(prog, [&]() {
    EXPECT_TRUE(CompareMatrices(x.value(), expected, 1e-9,
                                MatrixCompareType::absolute));
  });
}

// This test is identical to testProblem1 above but the cost is
// framed as a QP instead.
GTEST_TEST(testOptimizationProblem, testProblem1AsQP) {
  OptimizationProblem prog;
  auto x = prog.AddContinuousVariables(5);

  Eigen::MatrixXd Q = -100 * Eigen::Matrix<double, 5, 5>::Identity();
  Eigen::VectorXd c(5);
  c << 42, 44, 45, 47, 47.5;

  prog.AddQuadraticProgramCost(Q, c);

  VectorXd constraint(5);
  constraint << 20, 12, 11, 7, 4;
  prog.AddLinearConstraint(
      constraint.transpose(),
      Drake::Vector1d::Constant(-std::numeric_limits<double>::infinity()),
      Drake::Vector1d::Constant(40));
  prog.AddBoundingBoxConstraint(MatrixXd::Constant(5, 1, 0),
                                MatrixXd::Constant(5, 1, 1));
  VectorXd expected(5);
  expected << 1, 1, 0, 1, 0;
  std::srand(0);
  prog.SetInitialGuess({x}, expected + .01 * VectorXd::Random(5));
  RunNonlinearProgram(prog, [&]() {
    EXPECT_TRUE(CompareMatrices(x.value(), expected, 1e-9,
                                MatrixCompareType::absolute));
  });
}

// This test comes from Section 2.3 of "Handbook of Test Problems in
// Local and Global Optimization."
class TestProblem2Cost {
 public:
  static size_t numInputs() { return 6; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(x.rows() == numInputs());
    DRAKE_ASSERT(y.rows() == numOutputs());
    y(0) = (-50.0 * x(0) * x(0)) + (-10.5 * x(0)) - (50.0 * x(1) * x(1)) +
           (-7.5 * x(1)) - (50.0 * x(2) * x(2)) + (-3.5 * x(2)) -
           (50.0 * x(3) * x(3)) + (-2.5 * x(3)) - (50.0 * x(4) * x(4)) +
           (-1.5 * x(4)) + (-10.0 * x(5));
  }
};

GTEST_TEST(testOptimizationProblem, testProblem2) {
  OptimizationProblem prog;
  auto x = prog.AddContinuousVariables(6);
  prog.AddCost(TestProblem2Cost());
  VectorXd constraint1(6), constraint2(6);
  constraint1 << 6, 3, 3, 2, 1, 0;
  prog.AddLinearConstraint(
      constraint1.transpose(),
      Vector1d::Constant(-std::numeric_limits<double>::infinity()),
      Vector1d::Constant(6.5));
  constraint2 << 10, 0, 10, 0, 0, 1;
  prog.AddLinearConstraint(
      constraint2.transpose(),
      Vector1d::Constant(-std::numeric_limits<double>::infinity()),
      Vector1d::Constant(20));
  Eigen::VectorXd lower(6);
  lower << 0, 0, 0, 0, 0, 0;
  Eigen::VectorXd upper(6);
  upper << 1, 1, 1, 1, 1, std::numeric_limits<double>::infinity();
  prog.AddBoundingBoxConstraint(lower, upper);
  VectorXd expected(6);
  expected << 0, 1, 0, 1, 1, 20;
  std::srand(0);
  prog.SetInitialGuess({x}, expected + .01 * VectorXd::Random(6));
  // This test seems to be fairly sensitive to how much the randomness
  // causes the initial guess to deviate, so the tolerance is a bit
  // larger than others.  IPOPT is particularly sensitive here.
  RunNonlinearProgram(prog, [&]() {
    EXPECT_TRUE(CompareMatrices(x.value(), expected, 1e-3,
                                MatrixCompareType::absolute));
  });
}


// This test is identical to testProblem2 above but the cost is
// framed as a QP instead.
GTEST_TEST(testOptimizationProblem, testProblem2AsQP) {
  OptimizationProblem prog;
  auto x = prog.AddContinuousVariables(6);
  MatrixXd Q = -100.0 * MatrixXd::Identity(6, 6);
  Q(5, 5) = 0.0;
  VectorXd c(6);
  c << -10.5, -7.5, -3.5, -2.5, -1.5, -10.0;

  prog.AddQuadraticProgramCost(Q, c);

  VectorXd constraint1(6), constraint2(6);
  constraint1 << 6, 3, 3, 2, 1, 0;
  prog.AddLinearConstraint(
      constraint1.transpose(),
      Drake::Vector1d::Constant(-std::numeric_limits<double>::infinity()),
      Drake::Vector1d::Constant(6.5));
  constraint2 << 10, 0, 10, 0, 0, 1;
  prog.AddLinearConstraint(
      constraint2.transpose(),
      Drake::Vector1d::Constant(-std::numeric_limits<double>::infinity()),
      Drake::Vector1d::Constant(20));

  Eigen::VectorXd lower(6);
  lower << 0, 0, 0, 0, 0, 0;
  Eigen::VectorXd upper(6);
  upper << 1, 1, 1, 1, 1, std::numeric_limits<double>::infinity();
  prog.AddBoundingBoxConstraint(lower, upper);

  VectorXd expected(6);
  expected << 0, 1, 0, 1, 1, 20;
  std::srand(0);
  prog.SetInitialGuess({x}, expected + .01 * VectorXd::Random(6));

  // This test seems to be fairly sensitive to how much the randomness
  // causes the initial guess to deviate, so the tolerance is a bit
  // larger than others.  IPOPT is particularly sensitive here.
  RunNonlinearProgram(prog, [&]() {
    EXPECT_TRUE(CompareMatrices(x.value(), expected, 1e-3,
                                MatrixCompareType::absolute));
  });
}

// This test comes from Section 3.4 of "Handbook of Test Problems in
// Local and Global Optimization."
class LowerBoundTestCost {
 public:
  static size_t numInputs() { return 6; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(x.rows() == numInputs());
    DRAKE_ASSERT(y.rows() == numOutputs());
    y(0) = -25 * (x(0) - 2) * (x(0) - 2) + (x(1) - 2) * (x(1) - 2) -
           (x(2) - 1) * (x(2) - 1) - (x(3) - 4) * (x(3) - 4) -
           (x(4) - 1) * (x(4) - 1) - (x(5) - 4) * (x(5) - 4);
  }
};

class LowerBoundTestConstraint : public Constraint {
 public:
  LowerBoundTestConstraint(int i1, int i2)
      : Constraint(1, Vector1d::Constant(4),
                   Vector1d::Constant(std::numeric_limits<double>::infinity())),
        i1_(i1),
        i2_(i2) {}

  // for just these two types, implementing this locally is almost cleaner...
  void eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override {
    evalImpl(x, y);
  }
  void eval(const Eigen::Ref<const TaylorVecXd>& x,
            TaylorVecXd& y) const override {
    evalImpl(x, y);
  }

 private:
  template <typename ScalarType>
  void evalImpl(
      const Eigen::Ref<const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>>& x,
      Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& y) const {
    y.resize(1);
    y(0) = (x(i1_) - 3) * (x(i1_) - 3) + x(i2_);
  }

  int i1_;
  int i2_;
};

GTEST_TEST(testOptimizationProblem, lowerBoundTest) {
  OptimizationProblem prog;
  auto x = prog.AddContinuousVariables(6);
  prog.AddCost(LowerBoundTestCost());
  std::shared_ptr<Constraint> con1(new LowerBoundTestConstraint(2, 3));
  prog.AddGenericConstraint(con1);
  std::shared_ptr<Constraint> con2(new LowerBoundTestConstraint(4, 5));
  prog.AddGenericConstraint(con2);

  Eigen::VectorXd c1(6);
  c1 << 1, -3, 0, 0, 0, 0;
  prog.AddLinearConstraint(
      c1.transpose(),
      Vector1d::Constant(-std::numeric_limits<double>::infinity()),
      Vector1d::Constant(2));
  Eigen::VectorXd c2(6);
  c2 << -1, 1, 0, 0, 0, 0;
  prog.AddLinearConstraint(
      c2.transpose(),
      Vector1d::Constant(-std::numeric_limits<double>::infinity()),
      Vector1d::Constant(2));
  Eigen::VectorXd c3(6);
  c3 << 1, 1, 0, 0, 0, 0;
  prog.AddLinearConstraint(c3.transpose(), Vector1d::Constant(2),
                           Vector1d::Constant(6));
  Eigen::VectorXd lower(6);
  lower << 0, 0, 1, 0, 1, 0;
  Eigen::VectorXd upper(6);
  upper << std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(), 5, 6, 5, 10;
  prog.AddBoundingBoxConstraint(lower, upper);

  Eigen::VectorXd expected(6);
  expected << 5, 1, 5, 0, 5, 10;
  std::srand(0);
  Eigen::VectorXd delta = .05 * Eigen::VectorXd::Random(6);
  prog.SetInitialGuess({x}, expected + delta);

  // This test seems to be fairly sensitive to how much the randomness
  // causes the initial guess to deviate, so the tolerance is a bit
  // larger than others.  IPOPT is particularly sensitive here.
  RunNonlinearProgram(prog, [&]() {
    EXPECT_TRUE(CompareMatrices(x.value(), expected, 1e-3,
                                MatrixCompareType::absolute));
  });

  // Try again with the offsets in the opposite direction.
  prog.SetInitialGuess({x}, expected - delta);
  RunNonlinearProgram(prog, [&]() {
    EXPECT_TRUE(CompareMatrices(x.value(), expected, 1e-3,
                                MatrixCompareType::absolute));
  });
}

class SixHumpCamelCost {
 public:
  static size_t numInputs() { return 2; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(x.rows() == numInputs());
    DRAKE_ASSERT(y.rows() == numOutputs());
    y(0) =
        x(0) * x(0) * (4 - 2.1 * x(0) * x(0) + x(0) * x(0) * x(0) * x(0) / 3) +
        x(0) * x(1) + x(1) * x(1) * (-4 + 4 * x(1) * x(1));
  }
};

GTEST_TEST(testOptimizationProblem, sixHumpCamel) {
  OptimizationProblem prog;
  auto x = prog.AddContinuousVariables(2);
  auto cost = prog.AddCost(SixHumpCamelCost());

  RunNonlinearProgram(prog, [&]() {
    // check (numerically) if it is a local minimum
    VectorXd ystar, y;
    cost->eval(x.value(), ystar);
    for (int i = 0; i < 10; i++) {
      cost->eval(x.value() + .01 * Matrix<double, 2, 1>::Random(), y);
      if (y(0) < ystar(0)) throw std::runtime_error("not a local minima!");
    }
  });
}

class GloptipolyConstrainedExampleCost {
 public:
  static size_t numInputs() { return 3; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(x.rows() == numInputs());
    DRAKE_ASSERT(y.rows() == numOutputs());
    y(0) = -2 * x(0) + x(1) - x(2);
  }
};

class GloptipolyConstrainedExampleConstraint
    : public Constraint {  // want to also support deriving directly from
                           // constraint without going through Drake::Function
 public:
  GloptipolyConstrainedExampleConstraint()
      : Constraint(
            1, Vector1d::Constant(0),
            Vector1d::Constant(std::numeric_limits<double>::infinity())) {}

  // for just these two types, implementing this locally is almost cleaner...
  void eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override {
    evalImpl(x, y);
  }
  void eval(const Eigen::Ref<const TaylorVecXd>& x,
            TaylorVecXd& y) const override {
    evalImpl(x, y);
  }

 private:
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
GTEST_TEST(testOptimizationProblem, gloptipolyConstrainedMinimization) {
  OptimizationProblem prog;

  // This test is run twice on different collections of continuous
  // variables to make sure that the solvers correctly handle mapping
  // variables to constraints/costs.
  auto x = prog.AddContinuousVariables(3);
  auto y = prog.AddContinuousVariables(3);
  prog.AddCost(GloptipolyConstrainedExampleCost(), {x});
  prog.AddCost(GloptipolyConstrainedExampleCost(), {y});
  std::shared_ptr<GloptipolyConstrainedExampleConstraint> qp_con(
      new GloptipolyConstrainedExampleConstraint());
  prog.AddGenericConstraint(qp_con, {x});
  prog.AddGenericConstraint(qp_con, {y});
  prog.AddLinearConstraint(
      Vector3d(1, 1, 1).transpose(),
      Vector1d::Constant(-std::numeric_limits<double>::infinity()),
      Vector1d::Constant(4), {x});
  prog.AddLinearConstraint(
      Vector3d(1, 1, 1).transpose(),
      Vector1d::Constant(-std::numeric_limits<double>::infinity()),
      Vector1d::Constant(4), {y});
  prog.AddLinearConstraint(
      Vector3d(0, 3, 1).transpose(),
      Vector1d::Constant(-std::numeric_limits<double>::infinity()),
      Vector1d::Constant(6), {x});
  prog.AddLinearConstraint(
      Vector3d(0, 3, 1).transpose(),
      Vector1d::Constant(-std::numeric_limits<double>::infinity()),
      Vector1d::Constant(6), {y});
  prog.AddBoundingBoxConstraint(
      Vector3d(0, 0, 0),
      Vector3d(2, std::numeric_limits<double>::infinity(), 3), {x});
  prog.AddBoundingBoxConstraint(
      Vector3d(0, 0, 0),
      Vector3d(2, std::numeric_limits<double>::infinity(), 3), {y});

  // IPOPT has difficulty with this problem depending on the initial
  // conditions, which is why the initial guess varies so little.
  Vector3d initial_guess = Vector3d(.5, 0, 3) + .01 * Vector3d::Random();
  prog.SetInitialGuess({x}, initial_guess);
  prog.SetInitialGuess({y}, initial_guess);
  RunNonlinearProgram(prog, [&]() {
    EXPECT_TRUE(CompareMatrices(x.value(), Vector3d(0.5, 0, 3), 1e-4,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(y.value(), Vector3d(0.5, 0, 3), 1e-4,
                                MatrixCompareType::absolute));
  });
}

/**
 * Test that the eval() method of LinearComplementarityConstraint correctly
 * returns the slack.
 */
GTEST_TEST(testOptimizationProblem, simpleLCPConstraintEval) {
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
 * Note: This test is meant to test that OptimizationProblem.Solve() works in
 * this case; tests of the correctness of the Moby LCP solver itself live in
 * testMobyLCP.
 */
GTEST_TEST(testOptimizationProblem, simpleLCP) {
  OptimizationProblem prog;
  Eigen::Matrix<double, 2, 2> M;

  // clang-format off
  M << 1, 4,
       3, 1;
  // clang-format on

  Eigen::Vector2d q(-16, -15);

  auto x = prog.AddContinuousVariables(2);

  prog.AddLinearComplementarityConstraint(M, q, {x});
  EXPECT_NO_THROW(prog.Solve());
  EXPECT_TRUE(CompareMatrices(x.value(), Vector2d(16, 0), 1e-4,
                              MatrixCompareType::absolute));
}

/** Multiple LC constraints in a single optimization problem
 * @brief Just two copies of the simpleLCP example, to make sure that the
 * write-through of LCP results to the solution vector works correctly.
 */
GTEST_TEST(testOptimizationProblem, multiLCP) {
  OptimizationProblem prog;
  Eigen::Matrix<double, 2, 2> M;

  // clang-format off
  M << 1, 4,
       3, 1;
  // clang-format on

  Eigen::Vector2d q(-16, -15);

  auto x = prog.AddContinuousVariables(2);
  auto y = prog.AddContinuousVariables(2);

  prog.AddLinearComplementarityConstraint(M, q, {x});
  prog.AddLinearComplementarityConstraint(M, q, {y});
  EXPECT_NO_THROW(prog.Solve());

  EXPECT_TRUE(CompareMatrices(x.value(), Vector2d(16, 0), 1e-4,
                              MatrixCompareType::absolute));

  EXPECT_TRUE(CompareMatrices(y.value(), Vector2d(16, 0), 1e-4,
                              MatrixCompareType::absolute));
}

// The current windows CI build has no solver for generic constraints.  The
// DISABLED_ logic below ensures that we still at least get compile-time
// checking of the test and resulting template instantiations.
#if !defined(WIN32) && !defined(WIN64)
#define POLYNOMIAL_CONSTRAINT_TEST_NAME polynomialConstraint
#else
#define POLYNOMIAL_CONSTRAINT_TEST_NAME DISABLED_polynomialConstraint
#endif

/** Simple test of polynomial constraints. */
GTEST_TEST(testOptimizationProblem, POLYNOMIAL_CONSTRAINT_TEST_NAME) {
  static const double kInf = std::numeric_limits<double>::infinity();
  // Generic constraints in nlopt require a very generous epsilon.
  static const double kEpsilon = 1e-4;

  // Given a degenerate polynomial, get the trivial solution.
  {
    const Polynomiald x("x");
    OptimizationProblem problem;
    const auto x_var = problem.AddContinuousVariables(1);
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.getSimpleVariable()};
    problem.AddPolynomialConstraint(VectorXPoly::Constant(1, x), var_mapping,
                                    Vector1d::Constant(2),
                                    Vector1d::Constant(2));
    RunNonlinearProgram(problem, [&]() {
      EXPECT_NEAR(x_var.value()[0], 2, kEpsilon);
      // TODO(ggould-tri) test this with a two-sided constraint, once
      // the nlopt wrapper supports those.
    });
  }

  // Given a small univariate polynomial, find a low point.
  {
    const Polynomiald x("x");
    const Polynomiald poly = (x - 1) * (x - 1);
    OptimizationProblem problem;
    const auto x_var = problem.AddContinuousVariables(1);
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.getSimpleVariable()};
    problem.AddPolynomialConstraint(VectorXPoly::Constant(1, poly), var_mapping,
                                    Eigen::VectorXd::Zero(1),
                                    Eigen::VectorXd::Zero(1));
    RunNonlinearProgram(problem, [&]() {
      EXPECT_NEAR(x_var.value()[0], 1, 0.2);
      EXPECT_LE(poly.evaluateUnivariate(x_var.value()[0]), kEpsilon);
    });
  }

  // Given a small multivariate polynomial, find a low point.
  {
    const Polynomiald x("x");
    const Polynomiald y("y");
    const Polynomiald poly = (x - 1) * (x - 1) + (y + 2) * (y + 2);
    OptimizationProblem problem;
    const auto xy_var = problem.AddContinuousVariables(2);
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.getSimpleVariable(), y.getSimpleVariable()};
    problem.AddPolynomialConstraint(VectorXPoly::Constant(1, poly), var_mapping,
                                    Eigen::VectorXd::Zero(1),
                                    Eigen::VectorXd::Zero(1));
    RunNonlinearProgram(problem, [&]() {
      EXPECT_NEAR(xy_var.value()[0], 1, 0.2);
      EXPECT_NEAR(xy_var.value()[1], -2, 0.2);
      std::map<Polynomiald::VarType, double> eval_point = {
          {x.getSimpleVariable(), xy_var.value()[0]},
          {y.getSimpleVariable(), xy_var.value()[1]}};
      EXPECT_LE(poly.evaluateMultivariate(eval_point), kEpsilon);
    });
  }

  // Given two polynomial constraints, satisfy both.
  {
    // (x^4 - x^2 + 0.2 has two minima, one at 0.5 and the other at -0.5;
    // constrain x < 0 and EXPECT that the solver finds the negative one.)
    const Polynomiald x("x");
    const Polynomiald poly = x * x * x * x - x * x + 0.2;
    OptimizationProblem problem;
    const auto x_var = problem.AddContinuousVariables(1);
    problem.SetInitialGuess({x_var}, Vector1d::Constant(-0.1));
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.getSimpleVariable()};
    VectorXPoly polynomials_vec(2, 1);
    polynomials_vec << poly, x;
    problem.AddPolynomialConstraint(polynomials_vec, var_mapping,
                                    Eigen::VectorXd::Constant(2, -kInf),
                                    Eigen::VectorXd::Zero(2));
    RunNonlinearProgram(problem, [&]() {
      EXPECT_NEAR(x_var.value()[0], -0.7, 0.2);
      EXPECT_LE(poly.evaluateUnivariate(x_var.value()[0]), kEpsilon);
    });
  }
}

}  // namespace
}  // namespace solvers
}  // namespace drake
