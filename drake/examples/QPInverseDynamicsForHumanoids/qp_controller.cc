#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/gurobi_solver.h"

#include "qp_controller.h"
#include "drake/math/cross_product.h"

using namespace drake::solvers;

void QPController::ResizeQP(const HumanoidStatus& rs,
                            const std::vector<SupportElement>& all_supports) {
  // Figure out dimensions
  int num_contacts = all_supports.size();
  int num_vd = rs.robot().get_num_velocities();
  int num_basis = 0;
  int num_point_forces = 0;
  for (size_t i = 0; i < all_supports.size(); i++) {
    num_point_forces += all_supports[i].contact_points().size();
    num_basis +=
        num_basis_per_contact_point_ * all_supports[i].contact_points().size();
  }
  int num_torque = rs.robot().actuators.size();
  int num_variable = num_vd + num_basis;

  if (num_contacts == num_supports_ && num_vd == num_vd_ &&
      num_basis == num_basis_ && num_point_forces == num_point_forces_ &&
      num_torque == num_torque_ && num_variable == num_variable_)
    return;

  num_supports_ = num_contacts;
  num_vd_ = num_vd;
  num_basis_ = num_basis;
  num_point_forces_ = num_point_forces;
  num_torque_ = num_torque;
  num_variable_ = num_variable;

  // The order of insertion is important, the rest of the program assumes this
  // layout.
  prog_ = MathematicalProgram();
  DecisionVariableView vd = prog_.AddContinuousVariables(num_vd_, "vd");
  DecisionVariableView basis =
      prog_.AddContinuousVariables(num_basis_, "basis");

  // Allocate space for contact force jacobian and basis matrix.
  stacked_contact_jacobians_.resize(3 * num_point_forces_, num_vd_);
  basis_to_force_matrix_.resize(3 * num_point_forces_, num_basis_);
  stacked_contact_jacobians_dot_times_v_.resize(3 * num_point_forces_);
  torque_linear_.resize(num_torque_, num_variable_);
  dynamics_linear_.resize(6, num_variable_);
  point_force_to_wrench_.resize(6 * num_supports_, 3 * num_point_forces_);

  // Allocate equality constraints
  // Dyanmics
  eq_dynamics_ = prog_.AddLinearEqualityConstraint(
      MatrixXd::Zero(6, num_variable_), Matrix<double, 6, 1>::Zero(),
      {vd, basis});
  eq_dynamics_->set_description("dynamics eq");
  eq_contacts_.resize(num_supports_);
  // Contact constraints, 3 rows per contact point
  for (int i = 0; i < num_supports_; i++) {
    eq_contacts_[i] = prog_.AddLinearEqualityConstraint(
        MatrixXd::Zero(3 * all_supports[i].contact_points().size(), num_vd_),
        VectorXd::Zero(3 * all_supports[i].contact_points().size()), {vd});
    eq_contacts_[i]->set_description("contact eq");
  }

  // Allocate inequality constraints
  // Contact force scalar (Beta).
  // This is constant and does not depend on the robot configuration.
  ineq_contact_wrench_ = prog_.AddLinearConstraint(
      MatrixXd::Identity(num_basis_, num_basis_), VectorXd::Zero(num_basis_),
      VectorXd::Constant(num_basis_, 1000), {basis});
  ineq_contact_wrench_->set_description("contact force basis ineq");
  // Torque limit
  ineq_torque_limit_ = prog_.AddLinearConstraint(
      MatrixXd::Zero(num_torque_, num_variable_), VectorXd::Zero(num_torque_),
      VectorXd::Zero(num_torque_), {vd, basis});
  ineq_torque_limit_->set_description("torque limit ineq");

  // Allocate cost function
  Eigen::MatrixXd tmp_matrix_vd(num_vd_, num_vd_);
  Eigen::VectorXd tmp_vector_vd(num_vd_);
  // CoMdd
  cost_comdd_ = prog_.AddQuadraticCost(tmp_matrix_vd, tmp_vector_vd, {vd});
  cost_comdd_->set_description("com cost");
  // Pelvdd
  cost_pelvdd_ = prog_.AddQuadraticCost(tmp_matrix_vd, tmp_vector_vd, {vd});
  cost_pelvdd_->set_description("pelv cost");
  // Torsodd
  cost_torsodd_ = prog_.AddQuadraticCost(tmp_matrix_vd, tmp_vector_vd, {vd});
  cost_torsodd_->set_description("torso cost");
  // Left footdd
  cost_footdd_[Side::LEFT] =
      prog_.AddQuadraticCost(tmp_matrix_vd, tmp_vector_vd, {vd});
  cost_footdd_[Side::LEFT]->set_description("left foot cost");
  // Right footdd
  cost_footdd_[Side::RIGHT] =
      prog_.AddQuadraticCost(tmp_matrix_vd, tmp_vector_vd, {vd});
  cost_footdd_[Side::RIGHT]->set_description("right foot cost");

  // Regularize vd
  cost_vd_reg_ = prog_.AddQuadraticCost(tmp_matrix_vd, tmp_vector_vd, {vd});
  cost_vd_reg_->set_description("vd cost");
}

int QPController::Control(const HumanoidStatus& rs, const QPInput& input,
                          QPOutput* output) {
  if (!input.is_valid()) {
    std::cerr << "input is invalid\n";
    return -1;
  }

  // Resize and zero temporary matrices.
  ResizeQP(rs, input.supports());
  SetTempMatricesToZero();

  ////////////////////////////////////////////////////////////////////
  // The equations of motion look like:
  // M(q) * vd + h(q,v) = S * tau + J^T * lambda
  // M(q) is the inertia matrix, h(q,v) is the gravitational and centrifugal
  // force, vd is acceleration, S is the selection matrix (top 6 rows are
  // zeros due to the floating base), tau is joint torque, J^T is the transpose
  // of all contact Jacobian, and lambda is the contact wrench in the world
  // frame.
  // In this implementation, lambda is replaced by a set of point forces
  // applied at different contact points per each contact link.
  // The equations of motion is updated to:
  // M(q) * vd + h(q,v) = S * tau + J^T * basis * Beta
  //
  // For inverse dynamics, we are usually given desired motions, and
  // we want to solve for tau to achieve those motions.
  // Desired motions can be directly specified as vd_d, or as xdd_d in Cartesian
  // space, which is linear w.r.t. vd as well: xdd_d = J * vd + Jd * v.
  //
  // Note that since S.topRows(6) is zero,
  // tau = M_l * vd + h_l - (J^T * basis)_l * Beta
  // where _l means the lower num_torque_ rows of those matrices.
  // So we just need to solve for vd and Beta, and tau can be computed as
  // above. We can formulate inverse dynamics a QP problem.
  //
  // For the QP problem:
  // the unknown is _X = [vd, Beta]
  // equality constraints:
  //  M_u * vd + h_u = (J^T * basis)_u * Beta *  (equations of motion)
  //  J * vd + Jd * v = 0, (contact constraints)
  // inEquality: joint torque limit, limits on Beta, etc.
  // cost func:
  //  min (Jcom*vd + Jcomd*v - comdd_d)^2
  //    + (vd - vd_d)^2
  //    + all_kinds_of_body_acceleration_cost_terms (pelvis, torso, feet, etc)
  //
  // I made the dynamics and stationary contact equality constraints.
  // Alternatively, they can be setup as high weight cost terms. This is
  // sometimes preferred as it introduce slacks for better stability.

  const DecisionVariableView vd = prog_.GetVariable("vd");
  const DecisionVariableView basis = prog_.GetVariable("basis");

  int basis_start = basis.index();
  int vd_start = vd.index();

  // Stack the contact Jacobians and basis matrices for each contact link
  int rowIdx = 0;
  int colIdx = 0;
  for (int b = 0; b < num_supports_; b++) {
    const SupportElement& support = input.support(b);
    int force_dim = 3 * support.contact_points().size();
    int basis_dim =
        num_basis_per_contact_point_ * support.contact_points().size();
    basis_to_force_matrix_.block(rowIdx, colIdx, force_dim, basis_dim) =
        support.ComputeBasisMatrix(rs.robot(), rs.cache(),
                                   num_basis_per_contact_point_);
    stacked_contact_jacobians_.block(rowIdx, 0, force_dim, num_vd_) =
        support.ComputeJacobianAtContactPoints(rs.robot(), rs.cache());
    stacked_contact_jacobians_dot_times_v_.segment(rowIdx, force_dim) =
        support.ComputeJacobianDotTimesVAtContactPoints(rs.robot(), rs.cache());
    rowIdx += force_dim;
    colIdx += basis_dim;
  }
  JB_ = stacked_contact_jacobians_.transpose() * basis_to_force_matrix_;
  DRAKE_ASSERT(rowIdx == num_point_forces_ * 3);
  DRAKE_ASSERT(colIdx == num_basis_);

  // tau = M_l * vd + h_l - (J^T * basis)_l * Beta,
  // tau = torque_linear_ * X + torque_constant_
  torque_linear_.block(0, vd_start, num_torque_, num_vd_) =
      rs.M().bottomRows(num_torque_);
  torque_linear_.block(0, basis_start, num_torque_, num_basis_) =
      -JB_.bottomRows(num_torque_);
  torque_constant_ = rs.bias_term().tail(num_torque_);

  ////////////////////////////////////////////////////////////////////
  // Equality constraints:
  // Equations of motion part, 6 rows
  dynamics_linear_.block(0, vd_start, 6, num_vd_) = rs.M().topRows(6);
  dynamics_linear_.block(0, basis_start, 6, num_basis_) = -JB_.topRows(6);
  dynamics_constant_ = -rs.bias_term().head(6);
  eq_dynamics_->UpdateConstraint(dynamics_linear_, dynamics_constant_);

  // contact constraints, 3 rows per contact point
  rowIdx = 0;
  for (int i = 0; i < num_supports_; i++) {
    int force_dim = 3 * input.support(i).contact_points().size();
    eq_contacts_[i]->UpdateConstraint(
        stacked_contact_jacobians_.block(rowIdx, 0, force_dim, num_vd_),
        -stacked_contact_jacobians_dot_times_v_.segment(rowIdx, force_dim));
    rowIdx += force_dim;
  }
  DRAKE_ASSERT(rowIdx == num_point_forces_ * 3);

  ////////////////////////////////////////////////////////////////////
  // Inequality constraints:
  // For the contact point force basis, the constraints are always > 0,
  // so the stay constant as in ResizeQP.

  // Torque limits: min <= tau <= max, num_torque_ rows
  // min <= M_l * vd + h_l - (J^T * basis)_l * Beta <= max
  // min - h_l <= M_l * vd - (J^T * basis)_l * Beta <= max - h_l
  // tau = rs.robot->B.bottomRows(num_torque_) * u,
  // u = rs.robot->B.bottomRows(num_torque_).transpose() * tau
  // since B should be orthonormal.
  // tau is joint space indexed, and u is actuator space indexed.
  // constraints are specified with u index.
  inequality_linear_ =
      rs.robot().B.bottomRows(num_torque_).transpose() * torque_linear_;
  inequality_upper_bound_ = inequality_lower_bound_ =
      -rs.robot().B.bottomRows(num_torque_).transpose() * torque_constant_;
  for (size_t i = 0; i < rs.robot().actuators.size(); i++) {
    inequality_lower_bound_[i] += rs.robot().actuators[i].effort_limit_min_;
    inequality_upper_bound_[i] += rs.robot().actuators[i].effort_limit_max_;
  }
  ineq_torque_limit_->UpdateConstraint(
      inequality_linear_, inequality_lower_bound_, inequality_upper_bound_);

  ////////////////////////////////////////////////////////////////////
  // Cost function:
  // CoM term (task space acceleration costs)
  // w * (J * vd + Jdv - comdd_d)^T * (J * vd + Jdv - comdd_d)
  cost_comdd_->UpdateConstraint(
      input.w_com() * rs.J_com().transpose() * rs.J_com(),
      input.w_com() * rs.J_com().transpose() *
          (rs.Jdot_times_v_com() - input.comdd_d()));

  // For pelvisdd
  cost_pelvdd_->UpdateConstraint(
      input.w_pelv() * rs.pelv().J().transpose() * rs.pelv().J(),
      input.w_pelv() * rs.pelv().J().transpose() *
          (rs.pelv().Jdot_times_v() - input.pelvdd_d()));

  // For torsodd
  cost_torsodd_->UpdateConstraint(
      input.w_torso() * rs.torso().J().transpose() * rs.torso().J(),
      input.w_torso() * rs.torso().J().transpose() *
          (rs.torso().Jdot_times_v() - input.torsodd_d()));

  // For footdd
  for (int i = 0; i < 2; i++) {
    cost_footdd_[i]->UpdateConstraint(
        input.w_foot() * rs.foot(i).J().transpose() * rs.foot(i).J(),
        input.w_foot() * rs.foot(i).J().transpose() *
            (rs.foot(i).Jdot_times_v() - input.footdd_d(i)));
  }

  // Regularize vd to vd_d
  cost_vd_reg_->UpdateConstraint(
      input.w_vd() * MatrixXd::Identity(num_vd_, num_vd_),
      input.w_vd() * (-input.vd_d()));

  ////////////////////////////////////////////////////////////////////
  // Call solver
  SolutionResult result;
  SnoptSolver solver_;
  if (!solver_.available()) {
    std::cerr << "Solver not available.\n";
    return -1;
  }
  result = solver_.Solve(prog_);
  if (result != drake::solvers::SolutionResult::kSolutionFound) {
    std::cerr << "solution not found\n";
    return -1;
  }
  VectorXd solution = prog_.GetSolution();

  ////////////////////////////////////////////////////////////////////
  // Example of inspecting each cost / eq, ineq term
  auto costs = prog_.quadratic_costs();
  auto eqs = prog_.linear_equality_constraints();
  auto ineqs = prog_.linear_constraints();

  output->mutable_costs().resize(costs.size());
  int ctr = 0;
  for (auto cost_b : costs) {
    VectorXd val;
    std::shared_ptr<Constraint> cost = cost_b.constraint();
    cost->Eval(cost_b.VariableListToVectorXd(), val);
    output->mutable_cost(ctr).first = cost->get_description();
    output->mutable_cost(ctr).second = val(0);
    ctr++;
  }

  for (auto eq_b : eqs) {
    std::shared_ptr<LinearEqualityConstraint> eq = eq_b.constraint();
    VectorXd X = eq_b.VariableListToVectorXd();
    DRAKE_ASSERT((eq->A() * X - eq->lower_bound()).isZero(EPSILON));
  }

  for (auto ineq_b : ineqs) {
    std::shared_ptr<LinearConstraint> ineq = ineq_b.constraint();
    VectorXd X = ineq_b.VariableListToVectorXd();
    X = ineq->A() * X;
    for (int i = 0; i < X.size(); i++) {
      DRAKE_ASSERT(X[i] >= ineq->lower_bound()[i] - EPSILON &&
                   X[i] <= ineq->upper_bound()[i] + EPSILON);
    }
  }

  ////////////////////////////////////////////////////////////////////
  // Parse result
  output->mutable_vd() = vd.value();
  output->mutable_comdd() = rs.J_com() * output->vd() + rs.Jdot_times_v_com();
  output->mutable_pelvdd() =
      rs.pelv().J() * output->vd() + rs.pelv().Jdot_times_v();
  output->mutable_torsodd() =
      rs.torso().J() * output->vd() + rs.torso().Jdot_times_v();

  output->mutable_joint_torque() = torque_linear_ * solution + torque_constant_;

  for (int i = 0; i < 2; i++) {
    output->mutable_footdd(i) =
        (rs.foot(i).J() * output->vd() + rs.foot(i).Jdot_times_v());
    output->mutable_foot_wrench_in_world_frame(i).setZero();
    output->mutable_foot_wrench_in_sensor_frame(i).setZero();
  }

  // Matrix from basis to wrench
  rowIdx = colIdx = 0;
  for (int i = 0; i < num_supports_; i++) {
    const SupportElement& support = input.support(i);
    const Vector3d ref_point = rs.robot().transformPoints(
        rs.cache(), Vector3d::Zero(), support.body().get_body_index(), 0);

    for (size_t j = 0; j < support.contact_points().size(); j++) {
      const Vector3d contact_point =
          rs.robot().transformPoints(rs.cache(), support.contact_points()[j],
                                     support.body().get_body_index(), 0);
      // Force part: just sum up all the point forces, so these are I
      point_force_to_wrench_.block<3, 3>(rowIdx + 3, colIdx).setIdentity();
      // Torque part:
      point_force_to_wrench_.block<3, 3>(rowIdx, colIdx) =
          drake::math::VectorToSkewSymmetric(contact_point - ref_point);
      colIdx += 3;
    }
    rowIdx += 6;
  }
  DRAKE_ASSERT(rowIdx == 6 * num_supports_);
  DRAKE_ASSERT(colIdx == 3 * num_point_forces_);
  contact_wrenches_ =
      point_force_to_wrench_ * basis_to_force_matrix_ * basis.value();

  // Compute wrench for left and right foot in the world frame.
  for (int i = 0; i < num_supports_; i++) {
    int contact_body_idx = input.support(i).body().get_body_index();
    if (contact_body_idx ==
        rs.foot_sensor(Side::LEFT).body().get_body_index()) {
      output->mutable_foot_wrench_in_world_frame(Side::LEFT) =
          contact_wrenches_.segment<6>(6 * i);
    } else if (contact_body_idx ==
               rs.foot_sensor(Side::RIGHT).body().get_body_index()) {
      output->mutable_foot_wrench_in_world_frame(Side::RIGHT) =
          contact_wrenches_.segment<6>(6 * i);
    }
  }

  // Convert world frame wrench to sensor frame
  for (int i = 0; i < 2; i++) {
    // H^w_s = sensor frame = rs.foot_sensor(i).pose()
    // H^w_ak = world frame aligned, but located at ankle joint = [I,
    // rs.foot(i).pose().translation()]
    // To transform wrench from ak frame to s frame, we need H^s_ak.
    Isometry3d H_w_to_s = rs.foot_sensor(i).pose().inverse();
    Isometry3d H_ak_to_w(Isometry3d::Identity());
    H_ak_to_w.translation() = rs.foot(i).pose().translation();

    output->mutable_foot_wrench_in_sensor_frame(i) = transformSpatialForce(
        H_w_to_s * H_ak_to_w, output->foot_wrench_in_world_frame(i));
  }

  // Sanity check, net external wrench should = centroidal_matrix * vd +
  // centroidal_matrix_dot * v
  Vector6d Ld = rs.centroidal_momentum_matrix() * output->vd() +
                rs.centroidal_momentum_matrix_dot_times_v();
  Vector6d net_wrench = rs.robot().getMass() * rs.robot().a_grav;
  for (int i = 0; i < num_supports_; i++) {
    int contact_body_idx = input.support(i).body().get_body_index();
    net_wrench += contact_wrenches_.segment<6>(6 * i);
    const Vector3d ref_point = rs.robot().transformPoints(
        rs.cache(), Vector3d::Zero(), contact_body_idx, 0);
    net_wrench.segment<3>(0) +=
        (ref_point - rs.com()).cross(contact_wrenches_.segment<3>(6 * i + 3));
  }
  DRAKE_ASSERT((net_wrench - Ld).isZero(EPSILON));

  if (!output->is_valid()) {
    std::cerr << "output is invalid\n";
    return -1;
  }

  return 0;
}

std::ostream& operator<<(std::ostream& out, const QPInput& input) {
  out << "===============================================\n";
  out << "comdd_d: " << input.comdd_d().transpose() << std::endl;
  out << "pelvdd_d: " << input.pelvdd_d().transpose() << std::endl;
  out << "torsodd_d: " << input.torsodd_d().transpose() << std::endl;
  out << "footdd_d[0]: " << input.footdd_d(0).transpose() << std::endl;
  out << "footdd_d[1]: " << input.footdd_d(1).transpose() << std::endl;
  out << "vd_d: " << input.vd_d().transpose() << std::endl;

  out << "w_com: " << input.w_com() << std::endl;
  out << "w_pelv: " << input.w_pelv() << std::endl;
  out << "w_torso: " << input.w_torso() << std::endl;
  out << "w_foot: " << input.w_foot() << std::endl;
  out << "w_vd: " << input.w_vd() << std::endl;
  out << "w_basis_reg: " << input.w_basis_reg() << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out, const QPOutput& output) {
  out << "===============================================\n";
  out << "accelerations:\n";
  for (int i = 0; i < output.vd().size(); i++)
    out << output.coord_name(i) << ": " << output.vd()[i] << std::endl;

  out << "com acc: ";
  out << output.comdd().transpose() << std::endl;

  out << "pelv acc: ";
  out << output.pelvdd().transpose() << std::endl;

  out << "torso acc: ";
  out << output.torsodd().transpose() << std::endl;

  out << "left foot acc: " << output.footdd(Side::LEFT).transpose()
      << std::endl;
  out << "right foot acc: " << output.footdd(Side::RIGHT).transpose()
      << std::endl;

  out << "===============================================\n";
  out << "left foot wrench_w: "
      << output.foot_wrench_in_world_frame(Side::LEFT).transpose() << std::endl;
  out << "right foot wrench_w: "
      << output.foot_wrench_in_world_frame(Side::RIGHT).transpose()
      << std::endl;
  out << "left foot wrench in sensor frame: "
      << output.foot_wrench_in_sensor_frame(Side::LEFT).transpose()
      << std::endl;
  out << "right foot wrench in sensor frame: "
      << output.foot_wrench_in_sensor_frame(Side::RIGHT).transpose()
      << std::endl;

  out << "===============================================\n";
  out << "torque:\n";
  for (int i = 0; i < output.joint_torque().size(); i++)
    out << output.coord_name(i + 6) << ": " << output.joint_torque()[i]
        << std::endl;
  out << "===============================================\n";
  out << "costs:\n";
  for (size_t i = 0; i < output.costs().size(); i++)
    out << output.costs(i).first << ": " << output.costs(i).second << std::endl;

  return out;
}
