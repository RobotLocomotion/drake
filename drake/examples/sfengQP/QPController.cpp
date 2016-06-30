#include "drake/solvers/Optimization.h"
#include "drake/solvers/SnoptSolver.h"

#include "sfUtils.h"
#include "HumanoidState.h"
#include "QPController.h"

using namespace drake::solvers;

// some version of this should go in Optimization.h
static VectorXd variableList2VectorXd(VariableList const& vlist) {
  size_t dim = 0;
  for (auto var : vlist) {
    dim += var.size();
  }
  VectorXd X(dim);
  dim = 0;
  for (auto var : vlist) {
    X.segment(dim, var.size()) = var.value();
    dim += var.size();
  }
  return X;
}

int QPController::control(const HumanoidState& rs, const QPInput& input,
                          QPOutput& output) {
  if (!input.isSane()) {
    return -1;
  }

  int nContacts = 2;
  int nQdd = rs.robot->number_of_velocities();
  int nWrench = 6 * nContacts;
  int nTrq = nQdd - 6;
  int nVar = nQdd + nWrench;

  OptimizationProblem prog;
  const DecisionVariableView qdd = prog.AddContinuousVariables(nQdd, "qdd");
  const DecisionVariableView lambda =
      prog.AddContinuousVariables(nWrench, "lambda");

  int lambda_start = lambda.index();
  int qdd_start = qdd.index();

  // tau = M_l * qdd + h_l - J^T_l * lambda,
  // tau = TAU * _X + tau0
  MatrixXd Tau(MatrixXd::Zero(nTrq, nVar));
  Tau.block(0, qdd_start, nTrq, nQdd) = rs.M.bottomRows(nTrq);
  for (int i = 0; i < nContacts; i++) {
    Tau.block(0, lambda_start + i * 6, nTrq, 6) =
        -rs.foot[i]->J.block(0, 6, 6, nTrq).transpose();
  }
  VectorXd tau0 = rs.h.tail(nTrq);

  ////////////////////////////////////////////////////////////////////
  // equality constraints:
  // equations of motion part, 6 rows
  MatrixXd DynEq = MatrixXd::Zero(6, nVar);
  DynEq.block(0, qdd_start, 6, nQdd) = rs.M.topRows(6);

  for (int i = 0; i < nContacts; i++) {
    DynEq.block(0, lambda_start + i * 6, 6, 6) =
        -rs.foot[i]->J.block<6, 6>(0, 0).transpose();
  }
  prog.AddLinearEqualityConstraint(DynEq, -rs.h.head(6));

  // contact constraints, 6 rows per contact
  for (int i = 0; i < nContacts; i++) {
    prog.AddLinearEqualityConstraint(
        rs.foot[i]->J, -(rs.foot[i]->Jdv - input.footdd_d[i]), {qdd});
  }

  ////////////////////////////////////////////////////////////////////
  // set up inequality constraints
  // these are for contact wrench, 11 rows per contact.
  // NOTE: these constraints are specified for the contact wrench in the body
  // frame, but lambda are in world frame. So need to transform it by R^-1
  // 2 for Fx, Fy, Mz within friction cone, 6
  // 2 for CoP x within foot, 2
  // 2 for CoP y within foot, 2
  // 1 for Fz >= 0,
  int rowIdx = 0;
  MatrixXd CI = MatrixXd::Zero(11 * nContacts, nWrench);
  VectorXd ci_u = VectorXd::Constant(11 * nContacts,
                                     std::numeric_limits<double>::infinity());
  VectorXd ci_l = VectorXd::Constant(11 * nContacts,
                                     -std::numeric_limits<double>::infinity());

  // Fz >= 0;
  for (int i = 0; i < nContacts; i++) {
    CI(rowIdx, i * 6 + 5) = 1;
    ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // Fx <= mu * Fz, Fx - mu * Fz <= 0
  for (int i = 0; i < nContacts; i++) {
    CI(rowIdx, i * 6 + 3) = 1;
    CI(rowIdx, i * 6 + 5) = -param.mu;
    ci_u(rowIdx) = 0;
    rowIdx++;
  }
  // Fx >= -mu * Fz, Fx + mu * Fz >= 0
  for (int i = 0; i < nContacts; i++) {
    CI(rowIdx, i * 6 + 3) = 1;
    CI(rowIdx, i * 6 + 5) = param.mu;
    ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // Fy <= mu * Fz, Fy - mu * Fz <= 0
  for (int i = 0; i < nContacts; i++) {
    CI(rowIdx, i * 6 + 4) = 1;
    CI(rowIdx, i * 6 + 5) = -param.mu;
    ci_u(rowIdx) = 0;
    rowIdx++;
  }
  // Fy >= -mu * Fz, Fy + mu * Fz >= 0
  for (int i = 0; i < nContacts; i++) {
    CI(rowIdx, i * 6 + 4) = 1;
    CI(rowIdx, i * 6 + 5) = param.mu;
    ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // Mz <= mu * Fz, Mz - mu * Fz <= 0
  for (int i = 0; i < nContacts; i++) {
    CI(rowIdx, i * 6 + 2) = 1;
    CI(rowIdx, i * 6 + 5) = -param.mu_Mz;
    ci_u(rowIdx) = 0;
    rowIdx++;
  }
  // Mz >= -mu * Fz, Mz + mu * Fz >= 0
  for (int i = 0; i < nContacts; i++) {
    CI(rowIdx, i * 6 + 2) = 1;
    CI(rowIdx, i * 6 + 5) = param.mu_Mz;
    ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // cop_x <= x_max, -My / Fz <= x_max, -My - x_max * Fz <= 0
  for (int i = 0; i < nContacts; i++) {
    CI(rowIdx, i * 6 + 1) = -1;
    CI(rowIdx, i * 6 + 5) = -param.x_max;
    ci_u(rowIdx) = 0;
    rowIdx++;
  }
  // cop_x >= x_min, -My / Fz >= x_min, -My - x_min * Fz >= 0
  for (int i = 0; i < nContacts; i++) {
    CI(rowIdx, i * 6 + 1) = -1;
    CI(rowIdx, i * 6 + 5) = -param.x_min;
    ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // cop_y <= y_max, Mx / Fz <= y_max, Mx - y_max * Fz <= 0
  for (int i = 0; i < nContacts; i++) {
    CI(rowIdx, i * 6 + 0) = 1;
    CI(rowIdx, i * 6 + 5) = -param.y_max;
    ci_u(rowIdx) = 0;
    rowIdx++;
  }
  // cop_y >= y_min, Mx / Fz >= y_min, Mx - y_min * Fz >= 0
  for (int i = 0; i < nContacts; i++) {
    CI(rowIdx, i * 6 + 0) = 1;
    CI(rowIdx, i * 6 + 5) = -param.y_min;
    ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // Since all of the above are constraints on wrench expressed in the body
  // frame, we need to rotate the lambda into body frame.
  MatrixXd world_to_foot(MatrixXd::Zero(nWrench, nWrench));
  for (int i = 0; i < nContacts; i++) {
    world_to_foot.block<3, 3>(i * 6, i * 6) =
        rs.foot[i]->pose.linear().transpose();
    world_to_foot.block<3, 3>(i * 6 + 3, i * 6 + 3) =
        world_to_foot.block<3, 3>(i * 6, i * 6);
  }
  CI = CI * world_to_foot;
  prog.AddLinearConstraint(CI, ci_l, ci_u, {lambda});

  // torque limits: min <= tau <= max, nTrq rows
  // min <= M_l * qdd + h_l - J^T_l * lambda <= max
  // min - h_l <= M_l * qdd - J^T_l * lambda <= max - h_l
  // tau = rs.robot->B.bottomRows(nTrq) * u,
  // u = rs.robot->B.bottomRows(nTrq).transpose() * tau
  // since B should be orthonormal.
  // tau is joint space indexed, and u is actuator space indexed.
  // constraints are specified with u index.
  CI = rs.robot->B.bottomRows(nTrq).transpose() * Tau;
  ci_u = ci_l = -rs.robot->B.bottomRows(nTrq).transpose() * tau0;
  for (size_t i = 0; i < rs.robot->actuators.size(); i++) {
    ci_l[i] += rs.robot->actuators[i].effort_limit_min;
    ci_u[i] += rs.robot->actuators[i].effort_limit_max;
  }
  prog.AddLinearConstraint(CI, ci_l, ci_u, {qdd, lambda});

  ////////////////////////////////////////////////////////////////////
  // cost function:
  // CoM term (task space acceleration costs)
  // w * (J * qdd + Jdv - comdd_d)^T * (J * qdd + Jdv - comdd_d)
  prog.AddQuadraticCost(
      input.w_com * rs.J_com.transpose() * rs.J_com,
      input.w_com * (rs.Jdv_com - input.comdd_d).transpose() * rs.J_com, {qdd});

  prog.AddQuadraticCost(
      input.w_pelv * rs.pelv.J.transpose() * rs.pelv.J,
      input.w_pelv * (rs.pelv.Jdv - input.pelvdd_d).transpose() * rs.pelv.J,
      {qdd});

  // regularize qdd to qdd_d
  prog.AddQuadraticCost(input.w_qdd * MatrixXd::Identity(nQdd, nQdd),
                        input.w_qdd * (-input.qdd_d), {qdd});

  // regularize lambda to lambda_d
  VectorXd lambda_d(VectorXd::Zero(nWrench));
  for (int i = 0; i < 2; i++) lambda_d.segment<6>(6 * i) = input.wrench_d[i];
  prog.AddQuadraticCost(
      input.w_wrench_reg * MatrixXd::Identity(nWrench, nWrench),
      input.w_wrench_reg * (-lambda_d), {lambda});

  ////////////////////////////////////////////////////////////////////
  // solve
  prog.SetInitialGuess(qdd, VectorXd::Zero(nQdd));
  VectorXd lambda0 = VectorXd::Zero(nWrench);
  prog.SetInitialGuess(lambda, lambda0);
  SolutionResult result;
  SnoptSolver snopt;
  result = snopt.Solve(prog);
  if (result != drake::solvers::SolutionResult::kSolutionFound) {
    return -1;
  }

  ////////////////////////////////////////////////////////////////////
  // example of inspecting each cost / eq, ineq term
  auto costs = prog.generic_costs();
  auto eqs = prog.linear_equality_constraints();
  auto ineqs = prog.linear_constraints();

  // would be nice to have a variable list -> flat VectorXd method
  for (auto cost_b : costs) {
    VectorXd val;
    std::shared_ptr<Constraint> cost = cost_b.constraint();
    cost->eval(variableList2VectorXd(cost_b.variable_list()), val);
    std::cout << "cost term 0.5 x^T * H * x + h0 * x: " << val.transpose()
              << std::endl;
  }

  for (auto eq_b : eqs) {
    std::shared_ptr<LinearEqualityConstraint> eq = eq_b.constraint();
    VectorXd X = variableList2VectorXd(eq_b.variable_list());
    assert((eq->A() * X - eq->lower_bound()).isZero());
  }

  for (auto ineq_b : ineqs) {
    std::shared_ptr<LinearConstraint> ineq = ineq_b.constraint();
    VectorXd X = variableList2VectorXd(ineq_b.variable_list());
    X = ineq->A() * X;
    for (size_t i = 0; i < X.size(); i++) {
      assert(X[i] >= ineq->lower_bound()[i] && X[i] <= ineq->upper_bound()[i]);
    }
  }

  ////////////////////////////////////////////////////////////////////
  // parse result
  output.qdd = qdd.value();
  output.comdd = rs.J_com * output.qdd + rs.Jdv_com;
  output.pelvdd = rs.pelv.J * output.qdd + rs.pelv.Jdv;
  output.torsodd = rs.torso.J * output.qdd + rs.torso.Jdv;

  for (int i = 0; i < nContacts; i++) {
    output.footdd[i] = (rs.foot[i]->J * output.qdd + rs.foot[i]->Jdv);
    output.foot_wrench_w[i] = lambda.value().segment<6>(i * 6);
  }

  output.trq = rs.M.bottomRows(nTrq) * output.qdd + rs.h.tail(nTrq);
  for (int i = 0; i < nContacts; i++) {
    output.trq -= rs.foot[i]->J.block(0, 6, 6, nTrq).transpose() *
                  output.foot_wrench_w[i];
  }

  for (int i = 0; i < nContacts; i++) {
    Isometry3d T(Isometry3d::Identity());
    T.translation() =
        rs.foot[i]->pose.translation() - rs.foot_sensor[i]->pose.translation();

    output.foot_wrench_in_sensor_frame[i] =
        transformSpatialForce(T, output.foot_wrench_w[i]);

    output.foot_wrench_in_sensor_frame[i].head<3>() =
        rs.foot_sensor[i]->pose.linear().transpose() *
        output.foot_wrench_in_sensor_frame[i].head<3>();
    output.foot_wrench_in_sensor_frame[i].tail<3>() =
        rs.foot_sensor[i]->pose.linear().transpose() *
        output.foot_wrench_in_sensor_frame[i].tail<3>();
  }

  // sanity checks,
  // check dynamics, foot not moving, should check
  VectorXd residual = rs.M * output.qdd + rs.h;
  for (int i = 0; i < nContacts; i++)
    residual -= rs.foot[i]->J.transpose() * output.foot_wrench_w[i];
  residual.tail(nTrq) -= output.trq;
  assert(residual.isZero());

  for (int i = 0; i < nContacts; i++) {
    assert(output.footdd[i].isZero());
  }

  if (!output.isSane()) {
    return -1;
  }

  return 0;
}

double QPOutput::computeCost(const HumanoidState& rs,
                             const QPInput& input) const {
  VectorXd c, tot;
  c = 0.5 * qdd.transpose() * input.w_com * rs.J_com.transpose() * rs.J_com *
          qdd +
      input.w_com * (rs.Jdv_com - input.comdd_d).transpose() * rs.J_com * qdd;
  tot = c;
  std::cout << "com cost: " << c << std::endl;

  c = 0.5 * qdd.transpose() * input.w_pelv * rs.pelv.J.transpose() * rs.pelv.J *
          qdd +
      input.w_pelv * (rs.pelv.Jdv - input.pelvdd_d).transpose() * rs.pelv.J *
          qdd;
  tot += c;
  std::cout << "pelv cost: " << c << std::endl;

  c = 0.5 * qdd.transpose() * input.w_qdd * qdd +
      input.w_qdd * (-input.qdd_d).transpose() * qdd;
  tot += c;
  std::cout << "qdd cost: " << c << std::endl;

  for (int i = 0; i < 2; i++) {
    c = 0.5 * foot_wrench_w[i].transpose() * input.w_wrench_reg *
            foot_wrench_w[i] +
        input.w_wrench_reg * (-input.wrench_d[i]).transpose() *
            foot_wrench_w[i];
    tot += c;
    std::cout << "wrench cost: " << c << std::endl;
  }

  std::cout << "total cost: " << tot << std::endl;
  return tot[0];
}

void QPOutput::print() const {
  std::cout << "===============================================\n";
  std::cout << "accelerations:\n";
  for (int i = 0; i < qdd.rows(); i++)
    std::cout << jointNames[i] << ": " << qdd[i] << std::endl;

  std::cout << "com acc: ";
  std::cout << comdd.transpose() << std::endl;

  std::cout << "pelv acc: ";
  std::cout << pelvdd.transpose() << std::endl;

  std::cout << "torso acc: ";
  std::cout << torsodd.transpose() << std::endl;

  std::cout << "left foot acc: " << footdd[Side::LEFT].transpose() << std::endl;
  std::cout << "right foot acc: " << footdd[Side::RIGHT].transpose()
            << std::endl;

  std::cout << "===============================================\n";
  std::cout << "left foot wrench_w: " << foot_wrench_w[Side::LEFT].transpose()
            << std::endl;
  std::cout << "right foot wrench_w: " << foot_wrench_w[Side::RIGHT].transpose()
            << std::endl;
  std::cout << "left foot wrench in sensor frame: "
            << foot_wrench_in_sensor_frame[Side::LEFT].transpose() << std::endl;
  std::cout << "right foot wrench in sensor frame: "
            << foot_wrench_in_sensor_frame[Side::RIGHT].transpose()
            << std::endl;

  std::cout << "===============================================\n";
  std::cout << "torque:\n";
  for (int i = 0; i < trq.rows(); i++)
    std::cout << jointNames[i + 6] << ": " << trq[i] << std::endl;
}
