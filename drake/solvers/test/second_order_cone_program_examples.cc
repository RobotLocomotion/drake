#include "drake/solvers/test/second_order_cone_program_examples.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

namespace drake {
namespace solvers {
namespace test {
std::vector<EllipsoidsSeparationProblem> GetEllipsoidsSeparationProblems() {
  return {EllipsoidsSeparationProblem::kProblem0,
          EllipsoidsSeparationProblem::kProblem1,
          EllipsoidsSeparationProblem::kProblem2,
          EllipsoidsSeparationProblem::kProblem3};
}

TestEllipsoidsSeparation::TestEllipsoidsSeparation() {
  switch (GetParam()) {
    case EllipsoidsSeparationProblem::kProblem0:
      x1_ = Eigen::Vector3d::Zero();
      x2_ = Eigen::Vector3d::Zero();
      x2_(0) = 2.0;
      R1_ = 0.5 * Eigen::Matrix3d::Identity();
      R2_ = Eigen::Matrix3d::Identity();
      break;
    case EllipsoidsSeparationProblem::kProblem1:
      x1_ = Eigen::Vector3d::Zero();
      x2_ = Eigen::Vector3d::Zero();
      x2_(0) = 1.0;
      R1_ = Eigen::Matrix3d::Identity();
      R2_ = Eigen::Matrix3d::Identity();
      break;
    case EllipsoidsSeparationProblem::kProblem2:
      x1_ = Eigen::Vector2d(1.0, 0.2);
      x2_ = Eigen::Vector2d(0.5, 0.4);
      R1_.resize(2, 2);
      R1_ << 0.1, 0.6, 0.2, 1.3;
      R2_.resize(2, 2);
      R2_ << -0.4, 1.5, 1.7, 0.3;
      break;
    case EllipsoidsSeparationProblem::kProblem3:
      x1_ = Eigen::Vector3d(1.0, 0.2, 0.8);
      x2_ = Eigen::Vector3d(3.0, -1.5, 1.9);
      R1_.resize(3, 3);
      R1_ << 0.2, 0.4, 0.2, -0.2, -0.1, 0.3, 0.2, 0.1, 0.1;
      R2_.resize(3, 2);
      R2_ << 0.1, 0.2, -0.1, 0.01, -0.2, 0.1;
      break;
  }
  const int kXdim = x1_.rows();
  t_ = prog_.NewContinuousVariables<2>("t");
  a_ = prog_.NewContinuousVariables(kXdim, "a");

  // Add Lorentz cone constraints
  // t1 >= |R1'*a|
  // t2 >= |R2'*a|
  // Introduce matrices
  // A_lorentz1 = [1 0;0 R1']
  // A_lorentz2 = [1 0;0 R2']
  // b_lorentz1 = 0;
  // b_lorentz2 = 0;
  // And both A_lorentz1*[t;a]+b_lorentz1, A_lorentz2*[t;a]+b_lorentz2 are
  // in the Lorentz cone.
  Eigen::MatrixXd A_lorentz1(1 + R1_.cols(), 1 + R1_.rows());
  Eigen::MatrixXd A_lorentz2(1 + R2_.cols(), 1 + R2_.rows());
  A_lorentz1.setZero();
  A_lorentz2.setZero();
  // clang-format off
  A_lorentz1 << 1, Eigen::RowVectorXd::Zero(R1_.rows()),
      Eigen::VectorXd::Zero(R1_.cols()), R1_.transpose();
  A_lorentz2 << 1, Eigen::RowVectorXd::Zero(R2_.rows()),
      Eigen::VectorXd::Zero(R2_.cols()), R2_.transpose();
  // clang-format on
  Eigen::VectorXd b_lorentz1 = Eigen::VectorXd::Zero(1 + R1_.cols());
  Eigen::VectorXd b_lorentz2 = Eigen::VectorXd::Zero(1 + R2_.cols());
  auto lorentz_cone1 = prog_
                           .AddLorentzConeConstraint(A_lorentz1, b_lorentz1,
                                                     {t_.segment<1>(0), a_})
                           .constraint();
  auto lorentz_cone2 = prog_
                           .AddLorentzConeConstraint(A_lorentz2, b_lorentz2,
                                                     {t_.segment<1>(1), a_})
                           .constraint();
  // a'*(x2 - x1) = 1
  prog_.AddLinearEqualityConstraint((x2_ - x1_).transpose(), 1.0, a_);

  // Add cost
  auto cost = prog_.AddLinearCost(Eigen::Vector2d(1.0, 1.0), t_).constraint();
}

void TestEllipsoidsSeparation::SolveAndCheckSolution(
    const MathematicalProgramSolverInterface& solver) {
  RunSolver(&prog_, solver);

  // Check the solution.
  // First check if each constraint is satisfied.
  const auto& a_value = prog_.GetSolution(a_);
  const auto& R1a_value = R1_.transpose() * a_value;
  const auto& R2a_value = R2_.transpose() * a_value;
  EXPECT_NEAR(prog_.GetSolution((t_(0))), R1a_value.norm(), 1.1e-6);
  EXPECT_NEAR(prog_.GetSolution((t_(1))), R2a_value.norm(), 1.1e-6);
  EXPECT_NEAR((x2_ - x1_).dot(a_value), 1.0, 1e-8);

  // Now check if the solution is meaningful, that it really finds a separating
  // hyperplane.
  // The separating hyperplane exists if and only if p* <= 1
  const double p_star = prog_.GetSolution(t_(0)) + prog_.GetSolution(t_(1));
  const bool is_separated = p_star <= 1.0;
  const double t1 = prog_.GetSolution(t_(0));
  const double t2 = prog_.GetSolution(t_(1));
  if (is_separated) {
    // Then the hyperplane a' * x = 0.5 * (a'*x1 + t1 + a'*x2 - t2)
    const double b1 = a_value.dot(x1_) + t1;
    const double b2 = a_value.dot(x2_) - t2;
    const double b = 0.5 * (b1 + b2);
    // Verify that b - a'*x1 >= |R1' * a|
    //             a'*x2 - b >= |R2' * a|
    EXPECT_GE(b - a_value.dot(x1_), (R1_.transpose() * a_value).norm());
    EXPECT_GE(a_value.dot(x2_) - b, (R2_.transpose() * a_value).norm());
  } else {
    // Now solve another SOCP to find a point y in the intersecting region
    // y = x1 + R1*u1
    // y = x2 + R2*u2
    // 1 >= |u1|
    // 1 >= |u2|
    MathematicalProgram prog_intersect;
    const int kXdim = R1_.rows();
    auto u1 = prog_intersect.NewContinuousVariables(R1_.cols(), "u1");
    auto u2 = prog_intersect.NewContinuousVariables(R2_.cols(), "u2");
    auto y = prog_intersect.NewContinuousVariables(kXdim, "y");

    // Add the constraint that both
    // [1; u1] and [1; u2] are in the Lorentz cone.
    VectorX<symbolic::Expression> e1(1 + u1.rows());
    VectorX<symbolic::Expression> e2(1 + u2.rows());
    e1(0) = 1;
    e2(0) = 1;
    for (int i = 0; i < u1.rows(); ++i) {
      e1(i + 1) = +u1(i);
    }
    for (int i = 0; i < u2.rows(); ++i) {
      e2(i + 1) = +u2(i);
    }
    prog_intersect.AddLorentzConeConstraint(e1);
    prog_intersect.AddLorentzConeConstraint(e2);

    // Add constraint y = x1 + R1*u1
    //                y = x2 + R2*u2
    Eigen::MatrixXd A1(y.rows(), y.rows() + R1_.cols());
    A1.block(0, 0, y.rows(), y.rows()) =
        Eigen::MatrixXd::Identity(y.rows(), y.rows());
    A1.block(0, y.rows(), y.rows(), R1_.cols()) = -R1_;
    Eigen::MatrixXd A2(y.rows(), y.rows() + R2_.cols());
    A2.block(0, 0, y.rows(), y.rows()) =
        Eigen::MatrixXd::Identity(y.rows(), y.rows());
    A2.block(0, y.rows(), y.rows(), R2_.cols()) = -R2_;
    prog_intersect.AddLinearEqualityConstraint(A1, x1_, {y, u1});
    prog_intersect.AddLinearEqualityConstraint(A2, x2_, {y, u2});

    RunSolver(&prog_intersect, solver);

    // Check if the constraints are satisfied
    const auto& u1_value = prog_intersect.GetSolution(u1);
    const auto& u2_value = prog_intersect.GetSolution(u2);
    EXPECT_LE(u1_value.norm(), 1);
    EXPECT_LE(u2_value.norm(), 1);
    const auto& y_value = prog_intersect.GetSolution(y);
    EXPECT_TRUE(CompareMatrices(y_value, x1_ + R1_ * u1_value, 1e-8,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(y_value, x2_ + R2_ * u2_value, 1e-8,
                                MatrixCompareType::absolute));
  }
}

std::vector<QPasSOCPProblem> GetQPasSOCPProblems() {
  return {QPasSOCPProblem::kProblem0, QPasSOCPProblem::kProblem1};
}

TestQPasSOCP::TestQPasSOCP() {
  switch (GetParam()) {
    case QPasSOCPProblem::kProblem0 :
      // Un-constrained QP
      Q_ = Eigen::Matrix2d::Identity();
      c_ = Eigen::Vector2d::Ones();
      A_ = Eigen::RowVector2d(0, 0);
      b_lb_ = Vector1<double>(-std::numeric_limits<double>::infinity());
      b_ub_ = Vector1<double>(std::numeric_limits<double>::infinity());
      break;
    case QPasSOCPProblem::kProblem1 :
      // Constrained QP
      Q_ = Eigen::Matrix3d::Zero();
      Q_(0, 0) = 1.0;
      Q_(1, 1) = 1.3;
      Q_(2, 2) = 2.0;
      Q_(1, 2) = 0.01;
      Q_(0, 1) = -0.2;
      c_ = Eigen::Vector3d(-1.0, -2.0, 1.2);

      A_.resize(2, 3);
      A_ << 1, 0, 2, 0, 1, 3;
      b_lb_ = Eigen::Vector2d(-1, -2);
      b_ub_ = Eigen::Vector2d(2, 4);
      break;
  }

  const int kXdim = Q_.rows();
  const Eigen::MatrixXd Q_symmetric = 0.5 * (Q_ + Q_.transpose());

  x_socp_ = prog_socp_.NewContinuousVariables(kXdim, "x");
  y_ = prog_socp_.NewContinuousVariables<1>("y")(0);
  Eigen::LLT<Eigen::MatrixXd, Eigen::Upper> lltOfQ(Q_symmetric);
  Eigen::MatrixXd Q_sqrt = lltOfQ.matrixU();
  VectorX<symbolic::Expression> e(2 + kXdim);
  e << y_, 2, Q_sqrt * x_socp_;
  prog_socp_.AddRotatedLorentzConeConstraint(e);

  prog_socp_.AddLinearConstraint(A_, b_lb_, b_ub_, x_socp_);

  auto cost_socp1 = std::make_shared<LinearCost>(c_.transpose());
  prog_socp_.AddCost(cost_socp1, x_socp_);
  prog_socp_.AddLinearCost(+y_);

  x_qp_ = prog_qp_.NewContinuousVariables(kXdim, "x");
  prog_qp_.AddQuadraticCost(Q_, c_, x_qp_);
  prog_qp_.AddLinearConstraint(A_, b_lb_, b_ub_, x_qp_);
}

void TestQPasSOCP::SolveAndCheckSolution(const MathematicalProgramSolverInterface &solver) {
  RunSolver(&prog_socp_, solver);
  const auto& x_socp_value = prog_socp_.GetSolution(x_socp_);
  double objective_value_socp =
      c_.dot(x_socp_value) + prog_socp_.GetSolution(y_);

  // Check the solution
  const int kXdim = Q_.rows();
  const Eigen::MatrixXd Q_symmetric = 0.5 * (Q_ + Q_.transpose());
  Eigen::LLT<Eigen::MatrixXd, Eigen::Upper> lltOfQ(Q_symmetric);
  Eigen::MatrixXd Q_sqrt = lltOfQ.matrixU();
  EXPECT_NEAR(2 * prog_socp_.GetSolution(y_),
              (Q_sqrt * x_socp_value).squaredNorm(), 1E-6);
  EXPECT_GE(prog_socp_.GetSolution(y_), 0);

  RunSolver(&prog_qp_, solver);
  const auto& x_qp_value = prog_qp_.GetSolution(x_qp_);
  const Eigen::RowVectorXd x_qp_transpose = x_qp_value.transpose();
  Eigen::VectorXd Q_x_qp = Q_ * x_qp_value;
  double objective_value_qp = c_.dot(x_qp_value);
  for (int i = 0; i < kXdim; ++i) {
    objective_value_qp += 0.5 * x_qp_value(i) * Q_x_qp(i);
  }

  // TODO(hongkai.dai@tri.global): tighten the tolerance. socp does not really
  // converge to true optimal yet.
  EXPECT_TRUE(CompareMatrices(x_qp_value, x_socp_value, 2e-4,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(objective_value_qp, objective_value_socp, 1E-6);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
