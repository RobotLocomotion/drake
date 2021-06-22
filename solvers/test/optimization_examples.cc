#include "drake/solvers/test/optimization_examples.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/clp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/scs_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::RowVector2d;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::symbolic::Expression;
using std::numeric_limits;
namespace drake {
namespace solvers {
namespace test {

const double kInf = std::numeric_limits<double>::infinity();

std::set<CostForm> linear_cost_form() {
  return std::set<CostForm>{CostForm::kNonSymbolic, CostForm::kSymbolic};
}

std::set<ConstraintForm> linear_constraint_form() {
  return std::set<ConstraintForm>{ConstraintForm::kNonSymbolic,
                                  ConstraintForm::kSymbolic,
                                  ConstraintForm::kFormula};
}

std::set<CostForm> quadratic_cost_form() {
  return std::set<CostForm>{CostForm::kNonSymbolic, CostForm::kSymbolic};
}

double EvaluateSolutionCost(const MathematicalProgram& prog,
                            const MathematicalProgramResult& result) {
  double cost{0};
  for (auto const& binding : prog.GetAllCosts()) {
    cost += prog.EvalBinding(binding, result.get_x_val())(0);
  }
  return cost;
}

/*
 * Expect that the optimal cost stored by the solver in the MathematicalProgram
 * be nearly the same as the cost reevaluated at the solution
 */
void ExpectSolutionCostAccurate(const MathematicalProgram& prog,
                                const MathematicalProgramResult& result,
                                double tol) {
  EXPECT_NEAR(EvaluateSolutionCost(prog, result), result.get_optimal_cost(),
              tol);
}

OptimizationProgram::OptimizationProgram(CostForm cost_form,
                                         ConstraintForm constraint_form)
    : cost_form_(cost_form),
      constraint_form_(constraint_form),
      prog_(std::make_unique<MathematicalProgram>()),
      initial_guess_{} {}

void OptimizationProgram::RunProblem(SolverInterface* solver) {
  if (solver->available()) {
    EXPECT_TRUE(solver->AreProgramAttributesSatisfied(*prog_));
    EXPECT_EQ(solver->ExplainUnsatisfiedProgramAttributes(*prog_), "");
    const MathematicalProgramResult result =
        RunSolver(*prog_, *solver, initial_guess());
    CheckSolution(result);
  }
}

double OptimizationProgram::GetSolverSolutionDefaultCompareTolerance(
    SolverId solver_id) const {
  if (solver_id == ClpSolver::id()) {
    return 1E-8;
  }
  if (solver_id == MosekSolver::id()) {
    return 1E-10;
  }
  if (solver_id == GurobiSolver::id()) {
    return 1E-10;
  }
  if (solver_id == SnoptSolver::id()) {
    return 1E-8;
  }
  if (solver_id == IpoptSolver::id()) {
    return 1E-6;
  }
  if (solver_id == NloptSolver::id()) {
    return 1E-6;
  }
  if (solver_id == OsqpSolver::id()) {
    return 1E-10;
  }
  if (solver_id == ScsSolver::id()) {
    return 3E-5;  // Scs is not very accurate.
  }
  throw std::runtime_error("Unsupported solver type.");
}

LinearSystemExample1::LinearSystemExample1()
    : prog_(std::make_unique<MathematicalProgram>()),
      x_{},
      initial_guess_{},
      b_{},
      con_{} {
  x_ = prog_->NewContinuousVariables<4>();
  b_ = Vector4d::Random();
  con_ = prog_->AddLinearEqualityConstraint(Matrix4d::Identity(), b_, x_)
             .evaluator();
  initial_guess_.setZero();
}

void LinearSystemExample1::CheckSolution(
    const MathematicalProgramResult& result) const {
  auto x_sol = result.GetSolution(x_);
  EXPECT_TRUE(CompareMatrices(x_sol, b_, tol(), MatrixCompareType::absolute));
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(b_(i), x_sol(i), tol());
    EXPECT_TRUE(CompareMatrices(x_sol.head(i), b_.head(i), tol(),
                                MatrixCompareType::absolute));
  }
  EXPECT_NEAR(0.0, result.get_optimal_cost(), tol());
}

LinearSystemExample2::LinearSystemExample2() : LinearSystemExample1(), y_{} {
  y_ = prog()->NewContinuousVariables<2>();
  prog()->AddLinearEqualityConstraint(2 * Matrix2d::Identity(),
                                      b().topRows<2>(), y_);
}

void LinearSystemExample2::CheckSolution(
    const MathematicalProgramResult& result) const {
  LinearSystemExample1::CheckSolution(result);
  EXPECT_TRUE(CompareMatrices(result.GetSolution(y_), b().topRows<2>() / 2,
                              tol(), MatrixCompareType::absolute));
  EXPECT_NEAR(0.0, result.get_optimal_cost(), tol());
}

LinearSystemExample3::LinearSystemExample3() : LinearSystemExample2() {
  con()->UpdateCoefficients(3 * Matrix4d::Identity(), b());
}

void LinearSystemExample3::CheckSolution(
    const MathematicalProgramResult& result) const {
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x()), b() / 3, tol(),
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(result.GetSolution(y()), b().topRows<2>() / 2,
                              tol(), MatrixCompareType::absolute));
  EXPECT_NEAR(0.0, result.get_optimal_cost(), tol());
}

LinearMatrixEqualityExample::LinearMatrixEqualityExample()
    : prog_(std::make_unique<MathematicalProgram>()), X_{}, A_{} {
  X_ = prog_->NewSymmetricContinuousVariables<3>("X");
  // clang-format off
  A_ << -1, -2,  3,
         0, -2,  4,
         0,  0, -4;
  // clang-format on
  prog_->AddLinearEqualityConstraint(A_.transpose() * X_ + X_ * A_,
                                     -Eigen::Matrix3d::Identity(), true);
}

void LinearMatrixEqualityExample::CheckSolution(
    const MathematicalProgramResult& result) const {
  auto X_value = result.GetSolution(X_);
  EXPECT_TRUE(CompareMatrices(A_.transpose() * X_value + X_value * A_,
                              -Eigen::Matrix3d::Identity(), 1E-8,
                              MatrixCompareType::absolute));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
  es.compute(X_value);
  EXPECT_TRUE((es.eigenvalues().array() >= 0).all());
  EXPECT_NEAR(0.0, result.get_optimal_cost(), 1E-8);
}

NonConvexQPproblem1::NonConvexQPproblem1(CostForm cost_form,
                                         ConstraintForm constraint_form)
    : prog_(std::make_unique<MathematicalProgram>()), x_{}, x_expected_{} {
  x_ = prog_->NewContinuousVariables<5>("x");
  prog_->AddBoundingBoxConstraint(0, 1, x_);
  switch (cost_form) {
    case CostForm::kGeneric: {
      prog_->AddCost(TestProblem1Cost(), x_);
      break;
    }
    case CostForm::kNonSymbolic: {
      AddQuadraticCost();
      break;
    }
    default:
      throw std::runtime_error("unsupported cost form");
  }
  switch (constraint_form) {
    case ConstraintForm::kSymbolic: {
      AddSymbolicConstraint();
      break;
    }
    case ConstraintForm::kNonSymbolic: {
      AddConstraint();
      break;
    }
    default:
      throw std::runtime_error("unsupported constraint form");
  }

  x_expected_ << 1, 1, 0, 1, 0;
}

Eigen::Matrix<double, 5, 1> NonConvexQPproblem1::initial_guess() const {
  return (Eigen::Matrix<double, 5, 1>() << 1.01, 1.02, -0.02, 1.03, -0.05)
      .finished();
}

void NonConvexQPproblem1::CheckSolution(
    const MathematicalProgramResult& result) const {
  const auto& x_value = result.GetSolution(x_);
  EXPECT_TRUE(
      CompareMatrices(x_value, x_expected_, 1E-9, MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog_, result, 1E-5);
}

void NonConvexQPproblem1::AddConstraint() {
  Eigen::Matrix<double, 1, 5> a;
  a << 20, 12, 11, 7, 4;
  prog_->AddLinearConstraint(a, -kInf, 40, x_);
}

void NonConvexQPproblem1::AddSymbolicConstraint() {
  const auto constraint =
      20 * x_(0) + 12 * x_(1) + 11 * x_(2) + 7 * x_(3) + 4 * x_(4);
  prog_->AddLinearConstraint(constraint, -kInf, 40);
}

void NonConvexQPproblem1::AddQuadraticCost() {
  Eigen::Matrix<double, 5, 5> Q =
      -100 * Eigen::Matrix<double, 5, 5>::Identity();
  Eigen::Matrix<double, 5, 1> c;
  c << 42, 44, 45, 47, 47.5;
  double r = -100;
  prog_->AddQuadraticCost(Q, c, r, x_);
}

NonConvexQPproblem2::NonConvexQPproblem2(CostForm cost_form,
                                         ConstraintForm constraint_form)
    : prog_(std::make_unique<MathematicalProgram>()), x_{}, x_expected_{} {
  x_ = prog_->NewContinuousVariables<6>("x");

  prog_->AddBoundingBoxConstraint(0, 1, x_.head<5>());
  prog_->AddBoundingBoxConstraint(0, kInf, x_(5));

  switch (cost_form) {
    case CostForm::kGeneric: {
      prog_->AddCost(TestProblem2Cost(), x_);
      break;
    }
    case CostForm::kNonSymbolic: {
      AddQuadraticCost();
      break;
    }
    default:
      throw std::runtime_error("Unsupported cost form");
  }

  switch (constraint_form) {
    case ConstraintForm::kNonSymbolic: {
      AddNonSymbolicConstraint();
      break;
    }
    case ConstraintForm::kSymbolic: {
      AddSymbolicConstraint();
      break;
    }
    default:
      throw std::runtime_error("Unsupported constraint form");
  }

  x_expected_ << 0, 1, 0, 1, 1, 20;
}

Vector6<double> NonConvexQPproblem2::initial_guess() const {
  return x_expected_ +
         (Vector6<double>() << 0.01, -0.02, 0.03, -0.04, 0.05, -0.06)
             .finished();
}

void NonConvexQPproblem2::CheckSolution(
    const MathematicalProgramResult& result) const {
  const auto& x_value = result.GetSolution(x_);
  EXPECT_TRUE(
      CompareMatrices(x_value, x_expected_, 1E-3, MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog_, result, 1E-4);
}

void NonConvexQPproblem2::AddQuadraticCost() {
  Eigen::Matrix<double, 6, 6> Q =
      -100.0 * Eigen::Matrix<double, 6, 6>::Identity();
  Q(5, 5) = 0.0;
  Vector6d c{};
  c << -10.5, -7.5, -3.5, -2.5, -1.5, -10.0;

  prog_->AddQuadraticCost(Q, c, x_);
}

void NonConvexQPproblem2::AddNonSymbolicConstraint() {
  Eigen::Matrix<double, 1, 6> a1{};
  Eigen::Matrix<double, 1, 6> a2{};
  a1 << 6, 3, 3, 2, 1, 0;
  a2 << 10, 0, 10, 0, 0, 1;
  prog_->AddLinearConstraint(a1, -kInf, 6.5, x_);
  prog_->AddLinearConstraint(a2, -kInf, 20, x_);
}

void NonConvexQPproblem2::AddSymbolicConstraint() {
  const symbolic::Expression constraint1{6 * x_(0) + 3 * x_(1) + 3 * x_(2) +
                                         2 * x_(3) + x_(4)};
  const symbolic::Expression constraint2{10 * x_(0) + 10 * x_(2) + x_(5)};
  prog_->AddLinearConstraint(constraint1, -kInf, 6.5);
  prog_->AddLinearConstraint(constraint2, -kInf, 20);
}

LowerBoundedProblem::LowerBoundedProblem(ConstraintForm constraint_form)
    : prog_(std::make_unique<MathematicalProgram>()), x_{}, x_expected_{} {
  x_ = prog_->NewContinuousVariables<6>("x");

  Vector6d lb{};
  Vector6d ub{};
  lb << 0, 0, 1, 0, 1, 0;
  ub << kInf, kInf, 5, 6, 5, 10;
  prog_->AddBoundingBoxConstraint(lb, ub, x_);

  prog_->AddCost(LowerBoundTestCost(), x_);
  std::shared_ptr<Constraint> con1(new LowerBoundTestConstraint(2, 3));
  prog_->AddConstraint(con1, x_);
  std::shared_ptr<Constraint> con2(new LowerBoundTestConstraint(4, 5));
  prog_->AddConstraint(con2, x_);

  switch (constraint_form) {
    case ConstraintForm::kNonSymbolic: {
      AddNonSymbolicConstraint();
      break;
    }
    case ConstraintForm::kSymbolic: {
      AddSymbolicConstraint();
      break;
    }
    default:
      throw std::runtime_error("Not a supported constraint form");
  }

  x_expected_ << 5, 1, 5, 0, 5, 10;
}

void LowerBoundedProblem::CheckSolution(
    const MathematicalProgramResult& result) const {
  const auto& x_value = result.GetSolution(x_);
  EXPECT_TRUE(
      CompareMatrices(x_value, x_expected_, 1E-3, MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog_, result, 1E-2);
}

Vector6<double> LowerBoundedProblem::initial_guess1() const {
  std::srand(0);
  Vector6d delta =
      0.05 * Vector6d::Random();
  return x_expected_ + delta;
}

Vector6<double> LowerBoundedProblem::initial_guess2() const {
  std::srand(0);
  Vector6d delta =
      0.05 * Vector6d::Random();
  return x_expected_ - delta;
}

void LowerBoundedProblem::AddSymbolicConstraint() {
  prog_->AddLinearConstraint(x_(0) - 3 * x_(1), -kInf, 2);
  prog_->AddLinearConstraint(-x_(0) + x_(1), -kInf, 2);
  prog_->AddLinearConstraint(x_(0) + x_(1), -kInf, 6);
}

void LowerBoundedProblem::AddNonSymbolicConstraint() {
  prog_->AddLinearConstraint(RowVector2d(1, -3), -kInf, 2, x_.head<2>());
  prog_->AddLinearConstraint(RowVector2d(-1, 1), -kInf, 2, x_.head<2>());
  prog_->AddLinearConstraint(RowVector2d(1, 1), -kInf, 6, x_.head<2>());
}

GloptiPolyConstrainedMinimizationProblem::
    GloptiPolyConstrainedMinimizationProblem(CostForm cost_form,
                                             ConstraintForm constraint_form)
    : prog_(std::make_unique<MathematicalProgram>()),
      x_{},
      y_{},
      expected_(0.5, 0, 3) {
  x_ = prog_->NewContinuousVariables<3>("x");
  y_ = prog_->NewContinuousVariables<3>("y");

  prog_->AddBoundingBoxConstraint(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(2, std::numeric_limits<double>::infinity(), 3), x_);
  prog_->AddBoundingBoxConstraint(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(2, std::numeric_limits<double>::infinity(), 3), y_);

  switch (cost_form) {
    case CostForm::kGeneric: {
      AddGenericCost();
      break;
    }
    case CostForm::kNonSymbolic: {
      AddNonSymbolicCost();
      break;
    }
    case CostForm::kSymbolic: {
      AddSymbolicCost();
      break;
    }
    default:
      throw std::runtime_error("Not a supported cost form");
  }

  // TODO(hongkai.dai): write this in symbolic form also.
  std::shared_ptr<GloptipolyConstrainedExampleConstraint> qp_con(
      new GloptipolyConstrainedExampleConstraint());
  prog_->AddConstraint(qp_con, x_);
  prog_->AddConstraint(qp_con, y_);

  switch (constraint_form) {
    case ConstraintForm::kNonSymbolic: {
      AddNonSymbolicConstraint();
      break;
    }
    case ConstraintForm::kSymbolic: {
      AddSymbolicConstraint();
      break;
    }
    default:
      throw std::runtime_error("Not a supported constraint form");
  }
}

Vector6<double> GloptiPolyConstrainedMinimizationProblem::initial_guess()
    const {
  Eigen::Vector3d init = expected_ + Eigen::Vector3d(0.02, 0.01, -0.01);
  return (Vector6<double>() << init, init).finished();
}

void GloptiPolyConstrainedMinimizationProblem::CheckSolution(
    const MathematicalProgramResult& result) const {
  const auto& x_value = result.GetSolution(x_);
  const auto& y_value = result.GetSolution(y_);
  EXPECT_TRUE(
      CompareMatrices(x_value, expected_, 1E-4, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(y_value, expected_, 1E-4, MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog_, result, 1E-4);
}

void GloptiPolyConstrainedMinimizationProblem::AddGenericCost() {
  prog_->AddCost(GloptipolyConstrainedExampleCost(), x_);
  prog_->AddCost(GloptipolyConstrainedExampleCost(), y_);
}

void GloptiPolyConstrainedMinimizationProblem::AddSymbolicCost() {
  prog_->AddLinearCost(-2 * x_(0) + x_(1) - x_(2));
  prog_->AddLinearCost(-2 * y_(0) + y_(1) - y_(2));
}

void GloptiPolyConstrainedMinimizationProblem::AddNonSymbolicCost() {
  prog_->AddLinearCost(Eigen::Vector3d(-2, 1, -1), x_);
  prog_->AddLinearCost(Eigen::Vector3d(-2, 1, -1), y_);
}

void GloptiPolyConstrainedMinimizationProblem::AddNonSymbolicConstraint() {
  Eigen::Matrix<double, 2, 3> A{};
  // clang-format off
  A << 1, 1, 1,
      0, 3, 1;
  // clang-format on
  prog_->AddLinearConstraint(
      A, Eigen::Vector2d::Constant(-std::numeric_limits<double>::infinity()),
      Eigen::Vector2d(4, 6), x_);
  prog_->AddLinearConstraint(
      A, Eigen::Vector2d::Constant(-std::numeric_limits<double>::infinity()),
      Eigen::Vector2d(4, 6), y_);
}

void GloptiPolyConstrainedMinimizationProblem::AddSymbolicConstraint() {
  Eigen::Matrix<symbolic::Expression, 2, 1> e1{};
  Eigen::Matrix<symbolic::Expression, 2, 1> e2{};
  // clang-format off
  e1 << x_(0) + x_(1) + x_(2),
      3 * x_(1) + x_(2);
  e2 << y_(0) + y_(1) + y_(2),
      3 * y_(1) + y_(2);
  // clang-format on
  prog_->AddLinearConstraint(
      e1, Eigen::Vector2d::Constant(-std::numeric_limits<double>::infinity()),
      Eigen::Vector2d(4, 6));
  prog_->AddLinearConstraint(
      e2, Eigen::Vector2d::Constant(-std::numeric_limits<double>::infinity()),
      Eigen::Vector2d(4, 6));
}

MinDistanceFromPlaneToOrigin::MinDistanceFromPlaneToOrigin(
    const Eigen::MatrixXd& A, const Eigen::VectorXd& b, CostForm cost_form,
    ConstraintForm constraint_form)
    : A_(A),
      b_(b),
      prog_lorentz_(std::make_unique<MathematicalProgram>()),
      prog_rotated_lorentz_(std::make_unique<MathematicalProgram>()),
      t_lorentz_{},
      x_lorentz_(A.cols()),
      t_rotated_lorentz_{},
      x_rotated_lorentz_(A.cols()) {
  const int kXdim = A.cols();
  t_lorentz_ = prog_lorentz_->NewContinuousVariables<1>("t");
  x_lorentz_ = prog_lorentz_->NewContinuousVariables(kXdim, "x");
  t_rotated_lorentz_ = prog_rotated_lorentz_->NewContinuousVariables<1>("t");
  x_rotated_lorentz_ =
      prog_rotated_lorentz_->NewContinuousVariables(kXdim, "x");

  switch (cost_form) {
    case CostForm::kNonSymbolic: {
      prog_lorentz_->AddLinearCost(Vector1d(1), t_lorentz_);
      prog_rotated_lorentz_->AddLinearCost(Vector1d(1), t_rotated_lorentz_);
      break;
    }
    case CostForm::kSymbolic: {
      prog_lorentz_->AddLinearCost(+t_lorentz_(0));
      prog_rotated_lorentz_->AddLinearCost(+t_rotated_lorentz_(0));
      break;
    }
    default:
      throw std::runtime_error("Not a supported cost form");
  }

  switch (constraint_form) {
    case ConstraintForm::kNonSymbolic: {
      AddNonSymbolicConstraint();
      break;
    }
    case ConstraintForm::kSymbolic: {
      AddSymbolicConstraint();
      break;
    }
    default:
      throw std::runtime_error("Not a supported constraint form");
  }

  // compute expected value
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
  x_expected_ = xz_expected.head(kXdim);
}

void MinDistanceFromPlaneToOrigin::AddNonSymbolicConstraint() {
  prog_lorentz_->AddLorentzConeConstraint({t_lorentz_, x_lorentz_});
  prog_lorentz_->AddLinearEqualityConstraint(A_, b_, x_lorentz_);
  // A2 * [t;x] + b = [1;t;x]
  Eigen::MatrixXd A2(2 + A_.cols(), 1 + A_.cols());
  A2 << Eigen::RowVectorXd::Zero(1 + A_.cols()),
      Eigen::MatrixXd::Identity(1 + A_.cols(), 1 + A_.cols());
  Eigen::VectorXd b2(2 + A_.cols());
  b2 << 1, VectorXd::Zero(1 + A_.cols());
  prog_rotated_lorentz_->AddRotatedLorentzConeConstraint(
      A2, b2, {t_rotated_lorentz_, x_rotated_lorentz_});
  prog_rotated_lorentz_->AddLinearEqualityConstraint(A_, b_,
                                                     x_rotated_lorentz_);
}

void MinDistanceFromPlaneToOrigin::AddSymbolicConstraint() {
  VectorX<Expression> tx(1 + A_.cols());
  tx(0) = +t_lorentz_(0);
  for (int i = 0; i < A_.cols(); ++i) {
    tx(i + 1) = +x_lorentz_(i);
  }
  prog_lorentz_->AddLorentzConeConstraint(tx);
  // TODO(hongkai.dai): change this to symbolic form.
  prog_lorentz_->AddLinearEqualityConstraint(A_ * x_lorentz_, b_);

  VectorX<Expression> tx2(2 + A_.cols());
  tx2(0) = 1;
  tx2(1) = +t_rotated_lorentz_(0);
  for (int i = 0; i < A_.cols(); ++i) {
    tx2(i + 2) = +x_rotated_lorentz_(i);
  }
  prog_rotated_lorentz_->AddRotatedLorentzConeConstraint(tx2);
  prog_rotated_lorentz_->AddLinearEqualityConstraint(A_ * x_rotated_lorentz_,
                                                     b_);
}

Eigen::VectorXd MinDistanceFromPlaneToOrigin::prog_lorentz_initial_guess()
    const {
  Eigen::VectorXd initial_guess(1 + A_.cols());
  initial_guess(0) = x_expected_.norm() + 0.1;
  initial_guess.tail(A_.cols()) = x_expected_ + 0.1 * VectorXd::Ones(A_.cols());
  return initial_guess;
}

Eigen::VectorXd
MinDistanceFromPlaneToOrigin::prog_rotated_lorentz_initial_guess() const {
  Eigen::VectorXd initial_guess(1 + A_.cols());
  initial_guess(0) = x_expected_.squaredNorm() + 0.1;
  initial_guess.tail(A_.cols()) = x_expected_ + 0.1 * VectorXd::Ones(A_.cols());
  return initial_guess;
}

void MinDistanceFromPlaneToOrigin::CheckSolution(
    const MathematicalProgramResult& result, bool rotated_cone) const {
  if (rotated_cone) {
    auto x_rotated_lorentz_value = result.GetSolution(x_rotated_lorentz_);
    auto t_rotated_lorentz_value = result.GetSolution(t_rotated_lorentz_);
    EXPECT_TRUE(CompareMatrices(x_rotated_lorentz_value, x_expected_, 1E-3,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(t_rotated_lorentz_value,
                                Vector1d(x_expected_.squaredNorm()), 1E-3,
                                MatrixCompareType::absolute));
    ExpectSolutionCostAccurate(*prog_rotated_lorentz_, result, 1E-3);
  } else {
    auto x_lorentz_value = result.GetSolution(x_lorentz_);
    auto t_lorentz_value = result.GetSolution(t_lorentz_);
    EXPECT_TRUE(CompareMatrices(x_lorentz_value, x_expected_, 1E-3,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(t_lorentz_value, Vector1d(x_expected_.norm()),
                                1E-3, MatrixCompareType::absolute));
    ExpectSolutionCostAccurate(*prog_lorentz_, result, 1E-3);
  }
}

ConvexCubicProgramExample::ConvexCubicProgramExample() {
  x_ = NewContinuousVariables<1>("x");
  AddCost(pow(x_(0), 3) - 12 * x_(0));
  AddBoundingBoxConstraint(0, std::numeric_limits<double>::infinity(), x_(0));
}

void ConvexCubicProgramExample::CheckSolution(
    const MathematicalProgramResult& result) const {
  const auto x_val = result.GetSolution(x_(0));
  EXPECT_NEAR(x_val, 2, 1E-6);
}

UnitLengthProgramExample::UnitLengthProgramExample()
    : MathematicalProgram(), x_(NewContinuousVariables<4>()) {
  // Impose the unit length constraint xᵀx = 1, by writing it as a quadratic
  // constraint 0.5xᵀQx + bᵀx = 1, where Q = 2I, and b = 0.
  auto unit_length_constraint = std::make_shared<QuadraticConstraint>(
      2 * Eigen::Matrix4d::Identity(), Eigen::Vector4d::Zero(), 1, 1);
  AddConstraint(unit_length_constraint, x_);
}

void UnitLengthProgramExample::CheckSolution(
    const MathematicalProgramResult& result, double tolerance) const {
  const auto x_val = result.GetSolution(x_);
  EXPECT_NEAR(x_val.squaredNorm(), 1, tolerance);
}

DistanceToTetrahedronExample::DistanceToTetrahedronExample(
    double distance_expected) {
  x_ = NewContinuousVariables<18>();

  // Distance to the tetrahedron is fixed.
  AddBoundingBoxConstraint(distance_expected, distance_expected, x_(17));
  // clang-format off
  A_tetrahedron_ << 1,  1,  1,
                   -1,  0,  0,
                    0, -1,  0,
                    0,  0, -1;
  // clang-format on
  b_tetrahedron_ << 1, 0, 0, 0;
  auto distance_constraint =
      std::make_shared<DistanceToTetrahedronNonlinearConstraint>(
          A_tetrahedron_, b_tetrahedron_);
  AddConstraint(distance_constraint, x_);
}

DistanceToTetrahedronExample::DistanceToTetrahedronNonlinearConstraint::
    DistanceToTetrahedronNonlinearConstraint(
        const Eigen::Matrix<double, 4, 3>& A_tetrahedron,
        const Eigen::Vector4d& b_tetrahedron)
    : Constraint(15, 18), A_tetrahedron_(A_tetrahedron) {
  const double inf = std::numeric_limits<double>::infinity();
  Eigen::Matrix<double, 15, 1> lower_bound, upper_bound;
  lower_bound << 1, 1, -inf, 0, 0, 0, 0, 0, 0, 0, 0, -inf, -inf, -inf, -inf;
  upper_bound << 1, 1, 0, 0, 0, 0, 0, inf, inf, inf, inf, b_tetrahedron;
  UpdateLowerBound(lower_bound);
  UpdateUpperBound(upper_bound);
}

EckhardtProblem::EckhardtProblem(bool set_sparsity_pattern)
    : prog_{new MathematicalProgram()}, x_{prog_->NewContinuousVariables<3>()} {
  prog_->AddLinearCost(-x_(0));
  auto constraint = std::make_shared<EckhardtConstraint>(set_sparsity_pattern);
  prog_->AddConstraint(constraint, x_);
  prog_->AddBoundingBoxConstraint(Eigen::Vector3d::Zero(),
                                  Eigen::Vector3d(100, 100, 10), x_);
}

void EckhardtProblem::CheckSolution(const MathematicalProgramResult& result,
                                    double tol) const {
  ASSERT_TRUE(result.is_success());
  const auto x_val = result.GetSolution(x_);
  Eigen::Vector3d x_expected(std::log(std::log(10)), std::log(10), 10.0);
  EXPECT_TRUE(CompareMatrices(x_val, x_expected, tol));
  EXPECT_NEAR(result.get_optimal_cost(), -x_expected(0), tol);
}

EckhardtProblem::EckhardtConstraint::EckhardtConstraint(
    bool set_sparsity_pattern)
    : Constraint(2, 3, Eigen::Vector2d::Zero(),
                 Eigen::Vector2d::Constant(kInf)) {
  if (set_sparsity_pattern) {
    SetGradientSparsityPattern({{0, 0}, {0, 1}, {1, 1}, {1, 2}});
  }
}

void TestEckhardtDualSolution(
    const SolverInterface& solver,
    const Eigen::Ref<const Eigen::VectorXd>& x_initial, double tol) {
  if (solver.available()) {
    EckhardtProblem problem{true};
    MathematicalProgramResult result;
    solver.Solve(problem.prog(), x_initial, {}, &result);
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(CompareMatrices(
        result.GetDualSolution(problem.prog().generic_constraints()[0]),
        Eigen::Vector2d(1. / std::log(10.), 1. / (10 * std::log(10.))), tol));
    EXPECT_TRUE(CompareMatrices(
        result.GetDualSolution(problem.prog().bounding_box_constraints()[0]),
        Eigen::Vector3d(0, 0, -1. / (10 * std::log(10.))), tol));
  }
}

HeatExchangerDesignProblem::HeatExchangerDesignConstraint1::
    HeatExchangerDesignConstraint1()
    : Constraint(1, 6, Vector1d(0), Vector1d(kInf)) {
  SetGradientSparsityPattern({{0, 0}, {0, 3}, {0, 5}});
}

HeatExchangerDesignProblem::HeatExchangerDesignConstraint2::
    HeatExchangerDesignConstraint2()
    : Constraint(2, 7, Eigen::Vector2d::Zero(),
                 Eigen::Vector2d::Constant(kInf)) {
  SetGradientSparsityPattern(
      {{0, 0}, {0, 5}, {0, 3}, {0, 2}, {1, 1}, {1, 6}, {1, 3}});
}

HeatExchangerDesignProblem::HeatExchangerDesignProblem()
    : prog_{new MathematicalProgram()}, x_{prog_->NewContinuousVariables<8>()} {
  prog_->AddLinearConstraint(1 - 0.0025 * (x_(3) + x_(5)) >= 0);
  prog_->AddLinearConstraint(1 - 0.0025 * (x_(4) + x_(6) - x_(3)) >= 0);
  prog_->AddLinearConstraint(1 - 0.01 * (x_(7) - x_(4)) >= 0);
  prog_->AddConstraint(std::make_shared<HeatExchangerDesignConstraint1>(),
                       x_.head<6>());
  prog_->AddConstraint(std::make_shared<HeatExchangerDesignConstraint2>(),
                       x_.tail<7>());
  Eigen::Matrix<double, 8, 1> x_lower, x_upper;
  x_lower << 100, 1000, 1000, 10, 10, 10, 10, 10;
  x_upper << 10000, 10000, 10000, 1000, 1000, 1000, 1000, 1000;
  prog_->AddBoundingBoxConstraint(x_lower, x_upper, x_);
  prog_->AddLinearCost(x_(0) + x_(1) + x_(2));
}

void HeatExchangerDesignProblem::CheckSolution(
    const MathematicalProgramResult& result, double tol) const {
  ASSERT_TRUE(result.is_success());
  const auto x_val = result.GetSolution(x_);
  Eigen::Matrix<double, 8, 1> x_expected;
  x_expected << 579.3167, 1359.943, 5110.071, 182.0174, 295.5985, 217.9799,
      286.4162, 395.5979;
  EXPECT_TRUE(CompareMatrices(x_val, x_expected, tol));
  EXPECT_NEAR(result.get_optimal_cost(), 7049.330923, tol);
}

EmptyGradientProblem::EmptyGradientProblem()
    : prog_{new MathematicalProgram()}, x_{prog_->NewContinuousVariables<2>()} {
  prog_->AddCost(std::make_shared<EmptyGradientProblem::EmptyGradientCost>(),
                 x_);
  prog_->AddConstraint(
      std::make_shared<EmptyGradientProblem::EmptyGradientConstraint>(), x_);
}

void EmptyGradientProblem::CheckSolution(
    const MathematicalProgramResult& result) const {
  EXPECT_TRUE(result.is_success());
}

EmptyGradientProblem::EmptyGradientConstraint::EmptyGradientConstraint()
    : Constraint(1, 2, Vector1d(-kInf), Vector1d(0)) {}
}  // namespace test
}  // namespace solvers
}  // namespace drake
