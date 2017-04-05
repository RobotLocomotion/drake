#include "drake/solvers/test/optimization_examples.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

using Eigen::Vector4d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Matrix2d;
using Eigen::RowVector2d;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::RowVectorXd;

using std::numeric_limits;
using drake::symbolic::Expression;
namespace drake {
namespace solvers {
namespace test {
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

OptimizationProgram::OptimizationProgram(CostForm cost_form,
                                         ConstraintForm cnstr_form)
    : prog_(std::make_unique<MathematicalProgram>()) {}

void OptimizationProgram::RunProblem(
    MathematicalProgramSolverInterface* solver) {
  if (solver->available()) {
    RunSolver(prog_.get(), *solver);
    CheckSolution(solver->solver_type());
  }
}

double OptimizationProgram::GetSolverSolutionDefaultCompareTolerance(
    SolverType solver_type) const {
  switch (solver_type) {
    case SolverType::kMosek : {
      return 1E-10;
    }
    case SolverType::kGurobi : {
      return 1E-10;
    }
    case SolverType::kSnopt : {
      return 1E-8;
    }
    case SolverType::kIpopt : {
      return 1E-6;
    }
    case SolverType::kNlopt : {
      return 1E-6;
    }
    default : {
      throw std::runtime_error("Unsupported solver type.");
    }
  }
}

LinearSystemExample1::LinearSystemExample1()
    : prog_(std::make_unique<MathematicalProgram>()), x_{}, b_{}, con_{} {
  x_ = prog_->NewContinuousVariables<4>();
  b_ = Vector4d::Random();
  con_ = prog_->AddLinearEqualityConstraint(Matrix4d::Identity(), b_, x_);
  prog_->SetInitialGuessForAllVariables(Vector4d::Zero());
}

bool LinearSystemExample1::CheckSolution() const {
  auto x_sol = prog_->GetSolution(x_);
  if (!CompareMatrices(x_sol, b_, tol(), MatrixCompareType::absolute)) {
    return false;
  }
  for (int i = 0; i < 4; ++i) {
    if (std::abs(x_sol(i) - b_(i)) > tol()) {
      return false;
    }
    if (!CompareMatrices(x_sol.head(i), b_.head(i), tol(),
                         MatrixCompareType::absolute)) {
      return false;
    }
  }
  return true;
}

LinearSystemExample2::LinearSystemExample2() : LinearSystemExample1(), y_{} {
  y_ = prog()->NewContinuousVariables<2>();
  prog()->AddLinearEqualityConstraint(2 * Matrix2d::Identity(),
                                      b().topRows<2>(), y_);
}

bool LinearSystemExample2::CheckSolution() const {
  if (!LinearSystemExample1::CheckSolution()) {
    return false;
  }
  if (!CompareMatrices(prog()->GetSolution(y_), b().topRows<2>() / 2, tol(),
                       MatrixCompareType::absolute)) {
    return false;
  }
  return true;
}

LinearSystemExample3::LinearSystemExample3() : LinearSystemExample2() {
  con()->UpdateConstraint(3 * Matrix4d::Identity(), b());
}

bool LinearSystemExample3::CheckSolution() const {
  if (!CompareMatrices(prog()->GetSolution(x()), b() / 3, tol(),
                       MatrixCompareType::absolute)) {
    return false;
  }
  if (!CompareMatrices(prog()->GetSolution(y()), b().topRows<2>() / 2, tol(),
                       MatrixCompareType::absolute)) {
    return false;
  }
  return true;
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

bool LinearMatrixEqualityExample::CheckSolution() const {
  auto X_value = prog_->GetSolution(X_);
  if (!CompareMatrices(A_.transpose() * X_value + X_value * A_,
                       -Eigen::Matrix3d::Identity(), 1E-8,
                       MatrixCompareType::absolute)) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
  es.compute(X_value);
  if (!(es.eigenvalues().array() >= 0).all()) {
    return false;
  }
  return true;
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
  prog_->SetInitialGuess(
      x_, x_expected_ + 0.01 * Eigen::Matrix<double, 5, 1>::Random());
}

bool NonConvexQPproblem1::CheckSolution() const {
  const auto& x_value = prog_->GetSolution(x_);
  return CompareMatrices(x_value, x_expected_, 1E-9,
                         MatrixCompareType::absolute);
}

void NonConvexQPproblem1::AddConstraint() {
  Eigen::Matrix<double, 1, 5> a;
  a << 20, 12, 11, 7, 4;
  prog_->AddLinearConstraint(a, -numeric_limits<double>::infinity(), 40, x_);
}

void NonConvexQPproblem1::AddSymbolicConstraint() {
  const auto constraint =
      20 * x_(0) + 12 * x_(1) + 11 * x_(2) + 7 * x_(3) + 4 * x_(4);
  prog_->AddLinearConstraint(constraint, -numeric_limits<double>::infinity(),
                             40);
}

void NonConvexQPproblem1::AddQuadraticCost() {
  Eigen::Matrix<double, 5, 5> Q =
      -100 * Eigen::Matrix<double, 5, 5>::Identity();
  Eigen::Matrix<double, 5, 1> c;
  c << 42, 44, 45, 47, 47.5;
  prog_->AddQuadraticCost(Q, c, x_);
}

NonConvexQPproblem2::NonConvexQPproblem2(CostForm cost_form,
                                         ConstraintForm cnstr_form)
    : prog_(std::make_unique<MathematicalProgram>()), x_{}, x_expected_{} {
  x_ = prog_->NewContinuousVariables<6>("x");

  prog_->AddBoundingBoxConstraint(0, 1, x_.head<5>());
  prog_->AddBoundingBoxConstraint(0, numeric_limits<double>::infinity(), x_(5));

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

  switch (cnstr_form) {
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
  prog_->SetInitialGuess(
      x_, x_expected_ + 0.01 * Eigen::Matrix<double, 6, 1>::Random());
}

bool NonConvexQPproblem2::CheckSolution() const {
  const auto& x_value = prog_->GetSolution(x_);
  return CompareMatrices(x_value, x_expected_, 1E-3,
                         MatrixCompareType::absolute);
}

void NonConvexQPproblem2::AddQuadraticCost() {
  Eigen::Matrix<double, 6, 6> Q =
      -100.0 * Eigen::Matrix<double, 6, 6>::Identity();
  Q(5, 5) = 0.0;
  Eigen::Matrix<double, 6, 1> c{};
  c << -10.5, -7.5, -3.5, -2.5, -1.5, -10.0;

  prog_->AddQuadraticCost(Q, c, x_);
}

void NonConvexQPproblem2::AddNonSymbolicConstraint() {
  Eigen::Matrix<double, 1, 6> a1{};
  Eigen::Matrix<double, 1, 6> a2{};
  a1 << 6, 3, 3, 2, 1, 0;
  a2 << 10, 0, 10, 0, 0, 1;
  prog_->AddLinearConstraint(a1, -numeric_limits<double>::infinity(), 6.5, x_);
  prog_->AddLinearConstraint(a2, -numeric_limits<double>::infinity(), 20, x_);
}

void NonConvexQPproblem2::AddSymbolicConstraint() {
  const symbolic::Expression constraint1{6 * x_(0) + 3 * x_(1) + 3 * x_(2) +
                                         2 * x_(3) + x_(4)};
  const symbolic::Expression constraint2{10 * x_(0) + 10 * x_(2) + x_(5)};
  prog_->AddLinearConstraint(constraint1, -numeric_limits<double>::infinity(),
                             6.5);
  prog_->AddLinearConstraint(constraint2, -numeric_limits<double>::infinity(),
                             20);
}

LowerBoundedProblem::LowerBoundedProblem(ConstraintForm cnstr_form)
    : prog_(std::make_unique<MathematicalProgram>()), x_{}, x_expected_{} {
  x_ = prog_->NewContinuousVariables<6>("x");

  Eigen::Matrix<double, 6, 1> lb{};
  Eigen::Matrix<double, 6, 1> ub{};
  lb << 0, 0, 1, 0, 1, 0;
  ub << numeric_limits<double>::infinity(), numeric_limits<double>::infinity(),
      5, 6, 5, 10;
  prog_->AddBoundingBoxConstraint(lb, ub, x_);

  prog_->AddCost(LowerBoundTestCost(), x_);
  std::shared_ptr<Constraint> con1(new LowerBoundTestConstraint(2, 3));
  prog_->AddConstraint(con1, x_);
  std::shared_ptr<Constraint> con2(new LowerBoundTestConstraint(4, 5));
  prog_->AddConstraint(con2, x_);

  switch (cnstr_form) {
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

bool LowerBoundedProblem::CheckSolution() const {
  const auto& x_value = prog_->GetSolution(x_);
  return CompareMatrices(x_value, x_expected_, 1E-3,
                         MatrixCompareType::absolute);
}

void LowerBoundedProblem::SetInitialGuess1() {
  std::srand(0);
  Eigen::Matrix<double, 6, 1> delta =
      0.05 * Eigen::Matrix<double, 6, 1>::Random();
  prog_->SetInitialGuess(x_, x_expected_ + delta);
}

void LowerBoundedProblem::SetInitialGuess2() {
  std::srand(0);
  Eigen::Matrix<double, 6, 1> delta =
      0.05 * Eigen::Matrix<double, 6, 1>::Random();
  prog_->SetInitialGuess(x_, x_expected_ - delta);
}

void LowerBoundedProblem::AddSymbolicConstraint() {
  prog_->AddLinearConstraint(x_(0) - 3 * x_(1),
                             -numeric_limits<double>::infinity(), 2);
  prog_->AddLinearConstraint(-x_(0) + x_(1),
                             -numeric_limits<double>::infinity(), 2);
  prog_->AddLinearConstraint(x_(0) + x_(1), -numeric_limits<double>::infinity(),
                             6);
}

void LowerBoundedProblem::AddNonSymbolicConstraint() {
  prog_->AddLinearConstraint(
      RowVector2d(1, -3), -numeric_limits<double>::infinity(), 2, x_.head<2>());
  prog_->AddLinearConstraint(
      RowVector2d(-1, 1), -numeric_limits<double>::infinity(), 2, x_.head<2>());
  prog_->AddLinearConstraint(
      RowVector2d(1, 1), -numeric_limits<double>::infinity(), 6, x_.head<2>());
}

GloptiPolyConstrainedMinimizationProblem::
    GloptiPolyConstrainedMinimizationProblem(CostForm cost_form,
                                             ConstraintForm cnstr_form)
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

  switch (cnstr_form) {
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

  Eigen::Vector3d initial_guess = expected_ + 0.01 * Eigen::Vector3d::Random();
  prog_->SetInitialGuess(x_, initial_guess);
  prog_->SetInitialGuess(y_, initial_guess);
}

bool GloptiPolyConstrainedMinimizationProblem::CheckSolution() const {
  const auto& x_value = prog_->GetSolution(x_);
  const auto& y_value = prog_->GetSolution(y_);
  return (CompareMatrices(x_value, expected_, 1E-4,
                          MatrixCompareType::absolute)) &&
         (CompareMatrices(y_value, expected_, 1E-4,
                          MatrixCompareType::absolute));
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
    ConstraintForm cnstr_form)
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

  switch (cnstr_form) {
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

void MinDistanceFromPlaneToOrigin::SetInitialGuess() {
  prog_lorentz_->SetInitialGuess(t_lorentz_,
                                 Vector1d(x_expected_.norm() + 0.1));
  prog_lorentz_->SetInitialGuess(x_lorentz_,
                                 x_expected_ + 0.1 * VectorXd::Ones(A_.cols()));
  prog_rotated_lorentz_->SetInitialGuess(
      t_rotated_lorentz_, Vector1d(x_expected_.squaredNorm() + 0.1));
  prog_rotated_lorentz_->SetInitialGuess(
      x_rotated_lorentz_, x_expected_ + 0.1 * VectorXd::Ones(A_.cols()));
}

bool MinDistanceFromPlaneToOrigin::CheckSolution(bool rotated_cone) const {
  if (rotated_cone) {
    auto x_rotated_lorentz_value =
        prog_rotated_lorentz_->GetSolution(x_rotated_lorentz_);
    auto t_rotated_lorentz_value =
        prog_rotated_lorentz_->GetSolution(t_rotated_lorentz_);
    return CompareMatrices(x_rotated_lorentz_value, x_expected_, 1E-3,
                           MatrixCompareType::absolute) &&
           CompareMatrices(t_rotated_lorentz_value,
                           Vector1d(x_expected_.squaredNorm()), 1E-3,
                           MatrixCompareType::absolute);
  } else {
    auto x_lorentz_value = prog_lorentz_->GetSolution(x_lorentz_);
    auto t_lorentz_value = prog_lorentz_->GetSolution(t_lorentz_);
    return CompareMatrices(x_lorentz_value, x_expected_, 1E-3,
                           MatrixCompareType::absolute) &&
           CompareMatrices(t_lorentz_value, Vector1d(x_expected_.norm()), 1E-3,
                           MatrixCompareType::absolute);
  }
}

ConvexCubicProgramExample::ConvexCubicProgramExample() {
  x_ = NewContinuousVariables<1>("x");
  AddCost(pow(x_(0), 3) - 12 * x_(0));
  AddBoundingBoxConstraint(0, std::numeric_limits<double>::infinity(), x_(0));
}

bool ConvexCubicProgramExample::CheckSolution() const {
  const auto x_val = GetSolution((x_(0)));
  return std::abs(x_val - 2) < 1E-6;
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
