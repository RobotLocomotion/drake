
#include <typeinfo>
#include "drake/solvers/Optimization.h"
#include "drake/util/testUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

struct Movable {
  Movable() = default;
  Movable(Movable&&) = default;
  Movable(Movable const&) = delete;
  static size_t numInputs() { return 1; }
  static size_t numOutputs() { return 1; }
  template <typename ScalarType>
  void Eval(VecIn<ScalarType> const&, VecOut<ScalarType>&) const {}
};

struct Copyable {
  Copyable() = default;
  Copyable(Copyable&&) = delete;
  Copyable(Copyable const&) = default;
  static size_t numInputs() { return 1; }
  static size_t numOutputs() { return 1; }
  template <typename ScalarType>
  void Eval(VecIn<ScalarType> const&, VecOut<ScalarType>&) const {}
};

struct Unique {
  Unique() = default;
  Unique(Unique&&) = delete;
  Unique(Unique const&) = delete;
  static size_t numInputs() { return 1; }
  static size_t numOutputs() { return 1; }
  template <typename ScalarType>
  void Eval(VecIn<ScalarType> const&, VecOut<ScalarType>&) const {}
};

void testAddFunction() {
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

void trivialLeastSquares() {
  OptimizationProblem prog;

  auto const& x = prog.AddContinuousVariables(4);

  auto x2 = x(2);
  auto xhead = x.head(3);

  Vector4d b = Vector4d::Random();
  auto con = prog.AddLinearEqualityConstraint(Matrix4d::Identity(), b, {x});
  prog.Solve();
  valuecheckMatrix(b, x.value(), 1e-10);
  valuecheck(b(2), x2.value()(0), 1e-10);
  valuecheckMatrix(b.head(3), xhead.value(), 1e-10);
  valuecheck(b(2), xhead(2).value()(0), 1e-10);  // a segment of a segment

  auto const& y = prog.AddContinuousVariables(2);
  prog.AddLinearEqualityConstraint(2 * Matrix2d::Identity(), b.topRows(2), {y});
  prog.Solve();
  valuecheckMatrix(b.topRows(2) / 2, y.value(), 1e-10);
  valuecheckMatrix(b, x.value(), 1e-10);

  con->UpdateConstraint(3 * Matrix4d::Identity(), b);
  prog.Solve();
  valuecheckMatrix(b.topRows(2) / 2, y.value(), 1e-10);
  valuecheckMatrix(b / 3, x.value(), 1e-10);

  std::shared_ptr<BoundingBoxConstraint> bbcon(new BoundingBoxConstraint(
      MatrixXd::Constant(2, 1, -1000.0), MatrixXd::Constant(2, 1, 1000.0)));
  prog.AddBoundingBoxConstraint(bbcon, {x.head(2)});
  prog.Solve();  // now it will solve as a nonlinear program
  valuecheckMatrix(b.topRows(2) / 2, y.value(), 1e-10);
  valuecheckMatrix(b / 3, x.value(), 1e-10);
}

class SixHumpCamelObjective {
 public:
  static size_t numInputs() { return 2; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  void Eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    assert(x.rows() == numInputs());
    assert(y.rows() == numOutputs());
    y(0) =
        x(0) * x(0) * (4 - 2.1 * x(0) * x(0) + x(0) * x(0) * x(0) * x(0) / 3) +
        x(0) * x(1) + x(1) * x(1) * (-4 + 4 * x(1) * x(1));
  }
};

void sixHumpCamel() {
  OptimizationProblem prog;
  auto x = prog.AddContinuousVariables(2);
  auto objective = prog.AddCost(SixHumpCamelObjective());
  prog.Solve();
  prog.PrintSolution();

  // check (numerically) if it is a local minimum
  VectorXd ystar, y;
  objective->Eval(x.value(), ystar);
  for (int i = 0; i < 10; i++) {
    objective->Eval(x.value() + .01 * Eigen::Matrix<double, 2, 1>::Random(), y);
    if (y(0) < ystar(0)) throw std::runtime_error("not a local minimum!");
  }
}

class GloptipolyConstrainedExampleObjective {
 public:
  static size_t numInputs() { return 3; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  void Eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
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
                   Vector1d::Constant(numeric_limits<double>::infinity())) {}

  // for just these two types, implementing this locally is almost cleaner...
  virtual void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override {
    EvalImpl(x, y);
  }
  virtual void Eval(const Eigen::Ref<const TaylorVecXd>& x,
                    TaylorVecXd& y) const override {
    EvalImpl(x, y);
  }

  template <typename ScalarType>
  void EvalImpl(const Ref<const Matrix<ScalarType, Dynamic, 1>>& x,
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
void gloptipolyConstrainedMinimization() {
  OptimizationProblem prog;
  auto x = prog.AddContinuousVariables(3);
  prog.AddCost(GloptipolyConstrainedExampleObjective());
  std::shared_ptr<GloptipolyConstrainedExampleConstraint> qp_con(
      new GloptipolyConstrainedExampleConstraint());
  prog.AddGenericConstraint(qp_con, {x});
  prog.AddLinearConstraint(
      Vector3d(1, 1, 1).transpose(),
      Vector1d::Constant(-numeric_limits<double>::infinity()),
      Vector1d::Constant(4));
  prog.AddLinearConstraint(
      Vector3d(0, 3, 1).transpose(),
      Vector1d::Constant(-numeric_limits<double>::infinity()),
      Vector1d::Constant(6));
  prog.AddBoundingBoxConstraint(
      Vector3d(0, 0, 0), Vector3d(2, numeric_limits<double>::infinity(), 3));

  prog.set_initial_guess({x}, Vector3d(.5, 0, 3) + .1 * Vector3d::Random());
  prog.Solve();
  prog.PrintSolution();

  valuecheckMatrix(x.value(), Vector3d(.5, 0, 3), 1e-4);
}

/**
 * Test that the eval() method of LinearComplementarityConstraint correctly
 * returns the slack.
 */
void simpleLCPConstraintEval() {
  OptimizationProblem prog;
  Eigen::Matrix<double, 2, 2> M;
  M << 1, 0,
      0, 1;

  Eigen::Vector2d q(-1, -1);

  LinearComplementarityConstraint c(M, q);
  Eigen::VectorXd x;
  c.Eval(Eigen::Vector2d(1, 1), x);
  valuecheckMatrix(x, Vector2d(0, 0), 1e-4);
  c.Eval(Eigen::Vector2d(1, 2), x);
  valuecheckMatrix(x, Vector2d(0, 1), 1e-4);
}

/** Simple linear complementarity problem example.
 * @brief a hand-created LCP easily solved.
 *
 * Note: This test is meant to test that OptimizationProblem.Solve() works in
 * this case; tests of the correctness of the Moby LCP solver itself live in
 * testMobyLCP.
 */
void simpleLCP() {
  OptimizationProblem prog;
  Eigen::Matrix<double, 2, 2> M;
  M << 1, 4,
      3, 1;

  Eigen::Vector2d q(-16, -15);

  auto x = prog.AddContinuousVariables(2);

  prog.AddLinearComplementarityConstraint(M, q, {x});
  prog.Solve();
  prog.PrintSolution();
  valuecheckMatrix(x.value(), Vector2d(16, 0), 1e-4);
}

/** Multiple LC constraints in a single optimization problem
 * @brief Just two copies of the simpleLCP example, to make sure that the
 * write-through of LCP results to the solution vector works correctly.
 */
void multiLCP() {
  OptimizationProblem prog;
  Eigen::Matrix<double, 2, 2> M;
  M << 1, 4,
      3, 1;

  Eigen::Vector2d q(-16, -15);

  auto x = prog.AddContinuousVariables(2);
  auto y = prog.AddContinuousVariables(2);

  prog.AddLinearComplementarityConstraint(M, q, {x});
  prog.AddLinearComplementarityConstraint(M, q, {y});
  prog.Solve();
  prog.PrintSolution();
  valuecheckMatrix(x.value(), Vector2d(16, 0), 1e-4);
  valuecheckMatrix(y.value(), Vector2d(16, 0), 1e-4);
}

int main(int argc, char* argv[]) {
  // SNOPT tests
  try {
    testAddFunction();
    trivialLeastSquares();
    sixHumpCamel();
    gloptipolyConstrainedMinimization();
  } catch (const std::exception& e) {
    // If the exception is SNOPT unavailble, skip the remaining snopt tests
    // and proceed; if not, reraise it to fail.
    if (std::string(e.what()) != "SNOPT unavailable") {
      throw;
    }
  }
  simpleLCPConstraintEval();
  simpleLCP();
  multiLCP();
  return 0;
}
