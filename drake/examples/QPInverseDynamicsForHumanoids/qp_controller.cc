#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"

#include "qp_controller.h"

using namespace drake::solvers;

// TODO(siyuan.feng@tri.global): some version of this should go to
// mathematical_program.h
static VectorXd VariableList2VectorXd(VariableList const& vlist) {
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

int QPController::Control(const HumanoidStatus& rs, const QPInput& input,
                          QPOutput* output) {
  if (!is_qp_input_sane(input)) {
    std::cerr << "input is invalid\n";
    return -1;
  }

  ////////////////////////////////////////////////////////////////////
  // The equations of motion look like:
  // M(q) * vd + h(q,v) = S * tau + J^T * lambda
  // M(q) is the inertia matrix, h(q,v) is the gravitational and centrifugal
  // force, vd is acceleration, S is the selection matrix (top 6 rows are
  // zeros due to the floating base), tau is joint torque, J^T is the transpose
  // of all contact Jacobian, and lambda is the contact wrench in the world
  // frame.
  //
  // For inverse dynamics, we are usually given desired motions, and
  // we want to solve for tau to achieve those motions.
  // Desired motions can be directly specified as vd_d, or as xdd_d in Cartesian
  // space, which is linear w.r.t. vd as well: xdd_d = J * vd + Jd * v.
  //
  // Note that since S.topRows(6) is zero,
  // tau = M_l * vd + h_l - J^T_l * lamda,
  // where _l means the lower num_torque rows of those matrices.
  // So we just need to solve for vd and lambda, and tau can be computed as
  // above. We can formulate inverse dynamics a QP problem.
  //
  // For the QP problem:
  // the unknown is _X = [vd, lambda]
  // equality constraints:
  //  M_u * vd + h_u = J^T_u * lambda (equations of motion)
  //  J * vd + Jd * v = 0, (contact constraints)
  // inEquality: a bunch, joint torque limit, limits on lambda, etc
  // cost func:
  //  min (Jcom*vd + Jcomd*v - comdd_d)^2
  //    + (vd - vd_d)^2
  //    + (lambda - lambda_d)^2
  //    + all_kinds_of_body_acceleration_cost_terms
  //
  // I made the dynamics and stationary contact equality constraints.
  // Alternatively, they can be setup as high weight cost terms. This is
  // sometimes preferred as it introduce slacks for better stability.
  //
  // We are assuming two foot contacts in this example.
  int num_contacts = 2;
  int num_vd = rs.robot().number_of_velocities();
  int num_wrench = 6 * num_contacts;
  int num_torque = num_vd - 6;
  int num_variable = num_vd + num_wrench;

  MathematicalProgram prog;
  const DecisionVariableView vd = prog.AddContinuousVariables(num_vd, "vd");
  const DecisionVariableView lambda =
      prog.AddContinuousVariables(num_wrench, "lambda");

  int lambda_start = lambda.index();
  int vd_start = vd.index();

  // tau = M_l * vd + h_l - J^T_l * lambda,
  // tau = torque_linear_ * _X + torque_constant_
  torque_linear_ = MatrixXd::Zero(num_torque, num_variable);
  torque_linear_.block(0, vd_start, num_torque, num_vd) =
      rs.M().bottomRows(num_torque);
  for (int i = 0; i < num_contacts; i++) {
    torque_linear_.block(0, lambda_start + i * 6, num_torque, 6) =
        -rs.foot(i).J.block(0, 6, 6, num_torque).transpose();
  }
  torque_constant_ = rs.bias_term().tail(num_torque);

  ////////////////////////////////////////////////////////////////////
  // equality constraints:
  // equations of motion part, 6 rows
  dynamics_linear_ = MatrixXd::Zero(6, num_variable);
  dynamics_linear_.block(0, vd_start, 6, num_vd) = rs.M().topRows(6);
  for (int i = 0; i < num_contacts; i++) {
    dynamics_linear_.block(0, lambda_start + i * 6, 6, 6) =
        -rs.foot(i).J.block<6, 6>(0, 0).transpose();
  }
  dynamics_constant_ = -rs.bias_term().head(6);
  prog.AddLinearEqualityConstraint(dynamics_linear_, dynamics_constant_);

  // contact constraints, 6 rows per contact
  for (int i = 0; i < num_contacts; i++) {
    prog.AddLinearEqualityConstraint(
        rs.foot(i).J, -(rs.foot(i).Jdot_times_v - input.footdd_d[i]), {vd});
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
  int row_idx = 0;
  inequality_linear_ = MatrixXd::Zero(11 * num_contacts, num_wrench);
  inequality_upper_bound_ = VectorXd::Constant(
      11 * num_contacts, std::numeric_limits<double>::infinity());
  inequality_lower_bound_ = VectorXd::Constant(
      11 * num_contacts, -std::numeric_limits<double>::infinity());

  // Fz >= 0;
  for (int i = 0; i < num_contacts; i++) {
    inequality_linear_(row_idx, i * 6 + 5) = 1;
    inequality_lower_bound_(row_idx) = 0;
    row_idx++;
  }
  // This is a very crude model for approximating friction.
  // |Fx| < muFz and |Fy| < muFz is a "box" rather than a "cone" approximation
  // of friction.
  // The magnitude is off by a factor or sqrt(2).
  // Fx <= mu * Fz, Fx - mu * Fz <= 0
  for (int i = 0; i < num_contacts; i++) {
    inequality_linear_(row_idx, i * 6 + 3) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = -param.mu;
    inequality_upper_bound_(row_idx) = 0;
    row_idx++;
  }
  // Fx >= -mu * Fz, Fx + mu * Fz >= 0
  for (int i = 0; i < num_contacts; i++) {
    inequality_linear_(row_idx, i * 6 + 3) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = param.mu;
    inequality_lower_bound_(row_idx) = 0;
    row_idx++;
  }
  // Fy <= mu * Fz, Fy - mu * Fz <= 0
  for (int i = 0; i < num_contacts; i++) {
    inequality_linear_(row_idx, i * 6 + 4) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = -param.mu;
    inequality_upper_bound_(row_idx) = 0;
    row_idx++;
  }
  // Fy >= -mu * Fz, Fy + mu * Fz >= 0
  for (int i = 0; i < num_contacts; i++) {
    inequality_linear_(row_idx, i * 6 + 4) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = param.mu;
    inequality_lower_bound_(row_idx) = 0;
    row_idx++;
  }
  // These are poor approximations for the normal torque especially when the
  // center of pressure is at a corner of the foot. I used this in this
  // example for its simplicity. A better approximation is to use point contact
  // forces at multiple contact positions.
  // Mz <= mu * Fz, Mz - mu * Fz <= 0
  for (int i = 0; i < num_contacts; i++) {
    inequality_linear_(row_idx, i * 6 + 2) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = -param.mu_Mz;
    inequality_upper_bound_(row_idx) = 0;
    row_idx++;
  }
  // Mz >= -mu * Fz, Mz + mu * Fz >= 0
  for (int i = 0; i < num_contacts; i++) {
    inequality_linear_(row_idx, i * 6 + 2) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = param.mu_Mz;
    inequality_lower_bound_(row_idx) = 0;
    row_idx++;
  }
  // cop_x <= x_max, -My / Fz <= x_max, -My - x_max * Fz <= 0
  for (int i = 0; i < num_contacts; i++) {
    inequality_linear_(row_idx, i * 6 + 1) = -1;
    inequality_linear_(row_idx, i * 6 + 5) = -param.x_max;
    inequality_upper_bound_(row_idx) = 0;
    row_idx++;
  }
  // cop_x >= x_min, -My / Fz >= x_min, -My - x_min * Fz >= 0
  for (int i = 0; i < num_contacts; i++) {
    inequality_linear_(row_idx, i * 6 + 1) = -1;
    inequality_linear_(row_idx, i * 6 + 5) = -param.x_min;
    inequality_lower_bound_(row_idx) = 0;
    row_idx++;
  }
  // cop_y <= y_max, Mx / Fz <= y_max, Mx - y_max * Fz <= 0
  for (int i = 0; i < num_contacts; i++) {
    inequality_linear_(row_idx, i * 6 + 0) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = -param.y_max;
    inequality_upper_bound_(row_idx) = 0;
    row_idx++;
  }
  // cop_y >= y_min, Mx / Fz >= y_min, Mx - y_min * Fz >= 0
  for (int i = 0; i < num_contacts; i++) {
    inequality_linear_(row_idx, i * 6 + 0) = 1;
    inequality_linear_(row_idx, i * 6 + 5) = -param.y_min;
    inequality_lower_bound_(row_idx) = 0;
    row_idx++;
  }
  // Since all of the above are constraints on wrench expressed in the body
  // frame, we need to rotate the lambda into body frame.
  MatrixXd world_to_foot(MatrixXd::Zero(num_wrench, num_wrench));
  for (int i = 0; i < num_contacts; i++) {
    world_to_foot.block<3, 3>(i * 6, i * 6) =
        rs.foot(i).pose.linear().transpose();
    world_to_foot.block<3, 3>(i * 6 + 3, i * 6 + 3) =
        world_to_foot.block<3, 3>(i * 6, i * 6);
  }
  inequality_linear_ = inequality_linear_ * world_to_foot;
  prog.AddLinearConstraint(inequality_linear_, inequality_lower_bound_,
                           inequality_upper_bound_, {lambda});

  // torque limits: min <= tau <= max, num_torque rows
  // min <= M_l * vd + h_l - J^T_l * lambda <= max
  // min - h_l <= M_l * vd - J^T_l * lambda <= max - h_l
  // tau = rs.robot->B.bottomRows(num_torque) * u,
  // u = rs.robot->B.bottomRows(num_torque).transpose() * tau
  // since B should be orthonormal.
  // tau is joint space indexed, and u is actuator space indexed.
  // constraints are specified with u index.
  inequality_linear_ =
      rs.robot().B.bottomRows(num_torque).transpose() * torque_linear_;
  inequality_upper_bound_ = inequality_lower_bound_ =
      -rs.robot().B.bottomRows(num_torque).transpose() * torque_constant_;
  for (size_t i = 0; i < rs.robot().actuators.size(); i++) {
    inequality_lower_bound_[i] += rs.robot().actuators[i].effort_limit_min_;
    inequality_upper_bound_[i] += rs.robot().actuators[i].effort_limit_max_;
  }
  prog.AddLinearConstraint(inequality_linear_, inequality_lower_bound_,
                           inequality_upper_bound_, {vd, lambda});

  ////////////////////////////////////////////////////////////////////
  // cost function:
  // CoM term (task space acceleration costs)
  // w * (J * vd + Jdv - comdd_d)^T * (J * vd + Jdv - comdd_d)
  prog.AddQuadraticCost(
      input.w_com * rs.J_com().transpose() * rs.J_com(),
      input.w_com * (rs.Jdot_times_v_com() - input.comdd_d).transpose() *
          rs.J_com(),
      {vd});

  prog.AddQuadraticCost(
      input.w_pelv * rs.pelv().J.transpose() * rs.pelv().J,
      input.w_pelv * (rs.pelv().Jdot_times_v - input.pelvdd_d).transpose() *
          rs.pelv().J,
      {vd});

  prog.AddQuadraticCost(
      input.w_torso * rs.torso().J.transpose() * rs.torso().J,
      input.w_torso * (rs.torso().Jdot_times_v - input.torsodd_d).transpose() *
          rs.torso().J,
      {vd});

  // regularize vd to vd_d
  prog.AddQuadraticCost(input.w_vd * MatrixXd::Identity(num_vd, num_vd),
                        input.w_vd * (-input.vd_d), {vd});

  // regularize lambda to lambda_d
  VectorXd lambda_d(VectorXd::Zero(num_wrench));
  for (int i = 0; i < 2; i++) lambda_d.segment<6>(6 * i) = input.wrench_d[i];
  prog.AddQuadraticCost(
      input.w_wrench_reg * MatrixXd::Identity(num_wrench, num_wrench),
      input.w_wrench_reg * (-lambda_d), {lambda});

  ////////////////////////////////////////////////////////////////////
  // solve
  prog.SetInitialGuess(vd, VectorXd::Zero(num_vd));
  VectorXd lambda0 = VectorXd::Zero(num_wrench);
  prog.SetInitialGuess(lambda, lambda0);
  SolutionResult result;
  SnoptSolver snopt;
  if (!snopt.available()) {
    std::cerr << "Solver (SNOPT) not available.\n";
    return -1;
  }
  result = snopt.Solve(prog);
  if (result != drake::solvers::SolutionResult::kSolutionFound) {
    std::cerr << "solution not found\n";
    return -1;
  }

  ////////////////////////////////////////////////////////////////////
  // example of inspecting each cost / eq, ineq term
  // These will not be called in a real controller
  auto costs = prog.generic_costs();
  auto eqs = prog.linear_equality_constraints();
  auto ineqs = prog.linear_constraints();

  for (auto cost_b : costs) {
    VectorXd val;
    std::shared_ptr<Constraint> cost = cost_b.constraint();
    cost->Eval(VariableList2VectorXd(cost_b.variable_list()), val);
    std::cout << "cost term 0.5 x^T * H * x + h0 * x: " << val.transpose()
              << std::endl;
  }

  for (auto eq_b : eqs) {
    std::shared_ptr<LinearEqualityConstraint> eq = eq_b.constraint();
    VectorXd X = VariableList2VectorXd(eq_b.variable_list());
    assert((eq->A() * X - eq->lower_bound()).isZero());
  }

  for (auto ineq_b : ineqs) {
    std::shared_ptr<LinearConstraint> ineq = ineq_b.constraint();
    VectorXd X = VariableList2VectorXd(ineq_b.variable_list());
    X = ineq->A() * X;
    for (int i = 0; i < X.size(); i++) {
      assert(X[i] >= ineq->lower_bound()[i] && X[i] <= ineq->upper_bound()[i]);
    }
  }

  ////////////////////////////////////////////////////////////////////
  // parse result
  output->vd = vd.value();
  output->comdd = rs.J_com() * output->vd + rs.Jdot_times_v_com();
  output->pelvdd = rs.pelv().J * output->vd + rs.pelv().Jdot_times_v;
  output->torsodd = rs.torso().J * output->vd + rs.torso().Jdot_times_v;

  for (int i = 0; i < num_contacts; i++) {
    output->footdd[i] = (rs.foot(i).J * output->vd + rs.foot(i).Jdot_times_v);
    output->foot_wrench_in_world_frame[i] = lambda.value().segment<6>(i * 6);
  }

  output->joint_torque = rs.M().bottomRows(num_torque) * output->vd +
                         rs.bias_term().tail(num_torque);
  for (int i = 0; i < num_contacts; i++) {
    output->joint_torque -=
        rs.foot(i).J.block(0, 6, 6, num_torque).transpose() *
        output->foot_wrench_in_world_frame[i];
  }

  for (int i = 0; i < num_contacts; i++) {
    Isometry3d T(Isometry3d::Identity());
    T.translation() =
        rs.foot(i).pose.translation() - rs.foot_sensor(i).pose.translation();

    output->foot_wrench_in_sensor_frame[i] =
        transformSpatialForce(T, output->foot_wrench_in_world_frame[i]);

    output->foot_wrench_in_sensor_frame[i].head<3>() =
        rs.foot_sensor(i).pose.linear().transpose() *
        output->foot_wrench_in_sensor_frame[i].head<3>();
    output->foot_wrench_in_sensor_frame[i].tail<3>() =
        rs.foot_sensor(i).pose.linear().transpose() *
        output->foot_wrench_in_sensor_frame[i].tail<3>();
  }

  // Check equality constraints:
  // Dynamics: M(q) * vd + h(q,v) = S * tau + J^T * lambda
  // Foot not moving: J * vd + Jd * v = 0
  VectorXd residual = rs.M() * output->vd + rs.bias_term();
  for (int i = 0; i < num_contacts; i++)
    residual -=
        rs.foot(i).J.transpose() * output->foot_wrench_in_world_frame[i];
  residual.tail(num_torque) -= output->joint_torque;
  assert(residual.isZero());

  for (int i = 0; i < num_contacts; i++) {
    assert(output->footdd[i].isZero());
  }

  if (!is_qp_output_sane(*output)) {
    std::cerr << "output is invalid\n";
    return -1;
  }

  return 0;
}

void InitQPInput(const RigidBodyTree& r, QPInput* input) {
  input->vd_d.resize(r.number_of_velocities());
  input->coord_names.resize(r.number_of_velocities());
  for (int i = 0; i < r.number_of_velocities(); i++) {
    // strip out the "dot" part from name
    input->coord_names[i] =
        r.getVelocityName(i).substr(0, r.getVelocityName(i).size() - 3);
  }
}

void InitQPOutput(const RigidBodyTree& r, QPOutput* output) {
  output->vd.resize(r.number_of_velocities());
  output->coord_names.resize(r.number_of_velocities());
  for (int i = 0; i < r.number_of_velocities(); i++) {
    // strip out the "dot" part from name
    output->coord_names[i] =
        r.getVelocityName(i).substr(0, r.getVelocityName(i).size() - 3);
  }
}

double ComputeQPCost(const HumanoidStatus& rs, const QPInput& input,
                     const QPOutput& output) {
  VectorXd c, tot;
  c = 0.5 * output.vd.transpose() * input.w_com * rs.J_com().transpose() *
          rs.J_com() * output.vd +
      input.w_com * (rs.Jdot_times_v_com() - input.comdd_d).transpose() *
          rs.J_com() * output.vd;
  tot = c;
  std::cout << "com cost: " << c << std::endl;

  c = 0.5 * output.vd.transpose() * input.w_pelv * rs.pelv().J.transpose() *
          rs.pelv().J * output.vd +
      input.w_pelv * (rs.pelv().Jdot_times_v - input.pelvdd_d).transpose() *
          rs.pelv().J * output.vd;
  tot += c;
  std::cout << "pelv cost: " << c << std::endl;

  c = 0.5 * output.vd.transpose() * input.w_torso * rs.torso().J.transpose() *
          rs.torso().J * output.vd +
      input.w_torso * (rs.torso().Jdot_times_v - input.torsodd_d).transpose() *
          rs.torso().J * output.vd;
  tot += c;
  std::cout << "torso cost: " << c << std::endl;

  c = 0.5 * output.vd.transpose() * input.w_vd * output.vd +
      input.w_vd * (-input.vd_d).transpose() * output.vd;
  tot += c;
  std::cout << "vd cost: " << c << std::endl;

  for (int i = 0; i < 2; i++) {
    c = 0.5 * output.foot_wrench_in_world_frame[i].transpose() *
            input.w_wrench_reg * output.foot_wrench_in_world_frame[i] +
        input.w_wrench_reg * (-input.wrench_d[i]).transpose() *
            output.foot_wrench_in_world_frame[i];
    tot += c;
    std::cout << "wrench cost " << i << ": " << c << std::endl;
  }

  std::cout << "total cost: " << tot << std::endl;
  return tot[0];
}

void PrintQPOutput(const QPOutput& output) {
  std::cout << "===============================================\n";
  std::cout << "accelerations:\n";
  for (int i = 0; i < output.vd.rows(); i++)
    std::cout << output.coord_names[i] << ": " << output.vd[i] << std::endl;

  std::cout << "com acc: ";
  std::cout << output.comdd.transpose() << std::endl;

  std::cout << "pelv acc: ";
  std::cout << output.pelvdd.transpose() << std::endl;

  std::cout << "torso acc: ";
  std::cout << output.torsodd.transpose() << std::endl;

  std::cout << "left foot acc: " << output.footdd[Side::LEFT].transpose()
            << std::endl;
  std::cout << "right foot acc: " << output.footdd[Side::RIGHT].transpose()
            << std::endl;

  std::cout << "===============================================\n";
  std::cout << "left foot wrench_w: "
            << output.foot_wrench_in_world_frame[Side::LEFT].transpose()
            << std::endl;
  std::cout << "right foot wrench_w: "
            << output.foot_wrench_in_world_frame[Side::RIGHT].transpose()
            << std::endl;
  std::cout << "left foot wrench in sensor frame: "
            << output.foot_wrench_in_sensor_frame[Side::LEFT].transpose()
            << std::endl;
  std::cout << "right foot wrench in sensor frame: "
            << output.foot_wrench_in_sensor_frame[Side::RIGHT].transpose()
            << std::endl;

  std::cout << "===============================================\n";
  std::cout << "torque:\n";
  for (int i = 0; i < output.joint_torque.rows(); i++)
    std::cout << output.coord_names[i + 6] << ": " << output.joint_torque[i]
              << std::endl;
  std::cout << "===============================================\n";
}
