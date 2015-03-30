#include "QPCommon.h"
#include "controlUtil.h"
#include <Eigen/StdVector>

template<int M, int N>
void matlabToCArrayOfArrays(const mxArray *source, const int idx, const char *fieldname, double *destination)  {
  // Matlab arrays come in as column-major data. To represent a matrix in C++, as we do in our LCM messages, we need an array of arrays. But that convention results in a row-major storage, so we have to be careful about how we copy data in. 
  const mxArray *field = myGetField(source, idx, fieldname);
  sizecheck(field, M, N);
  Map<Matrix<double, M, N>>A(mxGetPrSafe(field));
  // C is row-major, matlab is column-major
  Matrix<double, N, M> A_t = A.transpose();
  memcpy(destination, A_t.data(), sizeof(double)*M*N);
  return;
}


std::shared_ptr<drake::lcmt_qp_controller_input> encodeQPInputLCM(const mxArray *qp_input) {
  // Take a matlab data structure corresponding to a QPInputConstantHeight object and parse it down to its representation as an equivalent LCM message. 
  std::shared_ptr<drake::lcmt_qp_controller_input> msg (new drake::lcmt_qp_controller_input());

  msg->be_silent = mxGetScalar(myGetProperty(qp_input, "be_silent")) > 0.5;

  msg->timestamp = (int64_t) (mxGetScalar(myGetProperty(qp_input, "timestamp")) * 1000000);

  const mxArray* zmp_data = myGetProperty(qp_input, "zmp_data");

  matlabToCArrayOfArrays<4, 4>(zmp_data, 0, "A", &msg->zmp_data.A[0][0]);
  matlabToCArrayOfArrays<4, 2>(zmp_data, 0, "B", &msg->zmp_data.B[0][0]);
  matlabToCArrayOfArrays<2, 4>(zmp_data, 0, "C", &msg->zmp_data.C[0][0]);
  matlabToCArrayOfArrays<2, 2>(zmp_data, 0, "D", &msg->zmp_data.D[0][0]);
  matlabToCArrayOfArrays<4, 1>(zmp_data, 0, "x0", &msg->zmp_data.x0[0][0]);
  matlabToCArrayOfArrays<2, 1>(zmp_data, 0, "y0", &msg->zmp_data.y0[0][0]);
  matlabToCArrayOfArrays<2, 1>(zmp_data, 0, "u0", &msg->zmp_data.u0[0][0]);
  matlabToCArrayOfArrays<2, 2>(zmp_data, 0, "R", &msg->zmp_data.R[0][0]);
  matlabToCArrayOfArrays<2, 2>(zmp_data, 0, "Qy", &msg->zmp_data.Qy[0][0]);
  matlabToCArrayOfArrays<4, 4>(zmp_data, 0, "S", &msg->zmp_data.S[0][0]);
  matlabToCArrayOfArrays<4, 1>(zmp_data, 0, "s1", &msg->zmp_data.s1[0][0]);
  matlabToCArrayOfArrays<4, 1>(zmp_data, 0, "s1dot", &msg->zmp_data.s1dot[0][0]);
  msg->zmp_data.s2 = mxGetScalar(myGetField(zmp_data, "s2"));
  msg->zmp_data.s2dot = mxGetScalar(myGetField(zmp_data, "s2dot"));
  msg->zmp_data.timestamp = msg->timestamp;


  const mxArray* support_data = myGetProperty(qp_input, "support_data");
  int nsupp = mxGetN(support_data);
  msg->num_support_data = (int32_t) nsupp;
  double double_logic_map[4][1];
  msg->support_data.resize(nsupp);
  if (nsupp > 0) {
    if (mxGetM(support_data) != 1) {
      mexErrMsgTxt("support data should be a struct array with M=1");
    }
    for (int i=0; i < nsupp; i++) {
      msg->support_data[i].timestamp = msg->timestamp;
      msg->support_data[i].body_id = (int32_t) mxGetScalar(myGetField(support_data, i, "body_id"));

      const mxArray *contact_pts = myGetField(support_data, i, "contact_pts");
      if (!contact_pts) mexErrMsgTxt("couldn't get points");
      Map<MatrixXd>contact_pts_mat(mxGetPr(contact_pts), mxGetM(contact_pts), mxGetN(contact_pts));
      msg->support_data[i].num_contact_pts = (int32_t) mxGetN(contact_pts);
      msg->support_data[i].contact_pts.resize(3);
      for (int j=0; j < 3; j++) {
        msg->support_data[i].contact_pts[j].resize(msg->support_data[i].num_contact_pts);
        for (int k=0; k < msg->support_data[i].num_contact_pts; k++) {
          msg->support_data[i].contact_pts[j][k] = contact_pts_mat(j, k);
        }
      }

      matlabToCArrayOfArrays<4, 1>(support_data, i, "support_logic_map", &double_logic_map[0][0]);
      for (int j=0; j < 4; j++) {
        msg->support_data[i].support_logic_map[j] = (double_logic_map[j][0] != 0);
      }
      msg->support_data[i].mu = mxGetScalar(myGetField(support_data, i, "mu"));
      msg->support_data[i].contact_surfaces = (int32_t) mxGetScalar(myGetField(support_data, i, "contact_surfaces"));
    }
  }

  const mxArray* body_motion_data = myGetProperty(qp_input, "body_motion_data");
  const int nbod = mxGetN(body_motion_data);
  msg->num_tracked_bodies = nbod;
  msg->body_motion_data.resize(nbod);
  if (nbod > 0) {
    if (mxGetM(body_motion_data) != 1) {
      mexErrMsgTxt("body motion data should be a 1xN struct array");
    }
    for (int i=0; i < nbod; i++) {
      msg->body_motion_data[i].timestamp = msg->timestamp;
      msg->body_motion_data[i].body_id = (int32_t) mxGetScalar(myGetField(body_motion_data, i, "body_id"));
      memcpy(msg->body_motion_data[i].ts, mxGetPr(myGetField(body_motion_data, i, "ts")), 2*sizeof(double));
      const mxArray* coefs = myGetField(body_motion_data, i, "coefs");
      if (mxGetNumberOfDimensions(coefs) != 3) mexErrMsgTxt("coefs should be a dimension-3 array");
      const mwSize* dim = mxGetDimensions(coefs);
      if (dim[0] != 6 || dim[1] != 1 || dim[2] != 4) mexErrMsgTxt("coefs should be size 6x4");
      matlabToCArrayOfArrays<6, 4>(body_motion_data, i, "coefs", &msg->body_motion_data[i].coefs[0][0]);
    }
  }

  const mxArray* whole_body_data = myGetProperty(qp_input, "whole_body_data");
  if (mxGetN(whole_body_data) != 1 || mxGetM(whole_body_data) != 1) mexErrMsgTxt("whole_body_data should be a 1x1 struct");
  const mxArray* q_des = myGetField(whole_body_data, "q_des");
  if (mxGetN(q_des) != 1) mexErrMsgTxt("q_des should be a column vector");
  const int npos = mxGetM(q_des);
  msg->whole_body_data.timestamp = msg->timestamp;
  msg->whole_body_data.num_positions = npos;
  Map<VectorXd>q_des_vec(mxGetPr(q_des), npos);
  msg->whole_body_data.q_des.resize(npos);

  for (int i=0; i < npos; i++) {
    msg->whole_body_data.q_des[i] = q_des_vec(i);
  }

  const mxArray* condof = myGetField(whole_body_data, "constrained_dofs");
  const int ncons = mxGetNumberOfElements(condof);
  msg->whole_body_data.num_constrained_dofs = ncons;
  msg->whole_body_data.constrained_dofs.resize(ncons);
  if (ncons > 0) {
    if (mxGetN(condof) != 1) mexErrMsgTxt("constrained dofs should be a column vector");
    Map<VectorXd>condof_vec(mxGetPrSafe(condof), ncons);

    for (int i=0; i < ncons; i++) {
      msg->whole_body_data.constrained_dofs[i] = static_cast<int32_t>(condof_vec(i));
    }
  }

  const mxArray* joint_override = myGetProperty(qp_input, "joint_pd_override");
  int num_joint_pd_overrides = mxGetNumberOfElements(joint_override);
  msg->num_joint_pd_overrides = num_joint_pd_overrides;
  msg->joint_pd_override.resize(num_joint_pd_overrides);
  for (int i=0; i < num_joint_pd_overrides; i++) {
    msg->joint_pd_override[i].position_ind = (int32_t) mxGetScalar(myGetField(joint_override, i, "position_ind"));
    msg->joint_pd_override[i].qi_des = mxGetScalar(myGetField(joint_override, i, "qi_des"));
    msg->joint_pd_override[i].qdi_des = mxGetScalar(myGetField(joint_override, i, "qdi_des"));
    msg->joint_pd_override[i].kp = mxGetScalar(myGetField(joint_override, i, "kp"));
    msg->joint_pd_override[i].kd = mxGetScalar(myGetField(joint_override, i, "kd"));
    msg->joint_pd_override[i].weight = mxGetScalar(myGetField(joint_override, i, "weight"));
  }

  msg->param_set_name = mxArrayToString(myGetProperty(qp_input, "param_set_name"));
  return msg;
}

PIDOutput wholeBodyPID(NewQPControllerData *pdata, double t, const Ref<const VectorXd> &q, const Ref<const VectorXd> &qd, const Ref<const VectorXd> &q_des, WholeBodyParams *params) {
  // Run a PID controller on the whole-body state to produce desired accelerations and reference posture
  PIDOutput out;
  double dt = 0;
  int nq = pdata->r->num_positions;
  assert(q.size() == nq);
  assert(qd.size() == pdata->r->num_velocities);
  assert(q_des.size() == params->integrator.gains.size());
  if (nq != pdata->r->num_velocities) {
    mexErrMsgTxt("this function will need to be rewritten when num_pos != num_vel");
  }
  if (pdata->state.t_prev != 0) {
    dt = t - pdata->state.t_prev;
  }
  pdata->state.q_integrator_state = (1-params->integrator.eta) * pdata->state.q_integrator_state + params->integrator.gains.cwiseProduct(q_des - q) * dt;
  pdata->state.q_integrator_state = pdata->state.q_integrator_state.array().max(-params->integrator.clamps.array());
  pdata->state.q_integrator_state = pdata->state.q_integrator_state.array().min(params->integrator.clamps.array());
  out.q_ref = q_des + pdata->state.q_integrator_state;
  out.q_ref = out.q_ref.array().max((pdata->r->joint_limit_min - params->integrator.clamps).array());
  out.q_ref = out.q_ref.array().min((pdata->r->joint_limit_max + params->integrator.clamps).array());

  pdata->state.q_integrator_state = pdata->state.q_integrator_state.array().max(-params->integrator.clamps.array());
  pdata->state.q_integrator_state = pdata->state.q_integrator_state.array().min(params->integrator.clamps.array());

  VectorXd err_q;
  err_q.resize(nq);
  err_q.head(3) = q_des.head(3) - q.head(3);
  for (int j = 3; j < nq; j++) {
    err_q(j) = angleDiff(q(j), q_des(j));
  }
  out.qddot_des = params->Kp.cwiseProduct(err_q) - params->Kd.cwiseProduct(qd);
  out.qddot_des = out.qddot_des.array().max(params->qdd_bounds.min.array());
  out.qddot_des = out.qddot_des.array().min(params->qdd_bounds.max.array());
  return out;
}

VectorXd velocityReference(NewQPControllerData *pdata, double t, const Ref<VectorXd> &q, const Ref<VectorXd> &qd, const Ref<VectorXd> &qdd, bool foot_contact[2], VRefIntegratorParams *params, RobotPropertyCache *rpc) {
  // Integrate expected accelerations to determine a target feed-forward velocity, which we can pass in to Atlas
  int i;
  assert(qdd.size() == pdata->r->num_velocities);

  double dt = 0;
  if (pdata->state.t_prev != 0) {
    dt = t - pdata->state.t_prev;
  }

  pdata->state.vref_integrator_state = (1-params->eta)*pdata->state.vref_integrator_state + params->eta*qd + qdd*dt;

  if (params->zero_ankles_on_contact && foot_contact[0] == 1) {
    for (i=0; i < rpc->position_indices.l_leg_ak.size(); i++) {
      pdata->state.vref_integrator_state(rpc->position_indices.l_leg_ak(i)) = 0;
    }
  }
  if (params->zero_ankles_on_contact && foot_contact[1] == 1) {
    for (i=0; i < rpc->position_indices.r_leg_ak.size(); i++) {
      pdata->state.vref_integrator_state(rpc->position_indices.r_leg_ak(i)) = 0;
    }
  }
  if (pdata->state.foot_contact_prev[0] != foot_contact[0]) {
    // contact state changed, reset integrated velocities
    for (i=0; i < rpc->position_indices.l_leg.size(); i++) {
      pdata->state.vref_integrator_state(rpc->position_indices.l_leg(i)) = qd(rpc->position_indices.l_leg(i));
    }
  }
  if (pdata->state.foot_contact_prev[1] != foot_contact[1]) {
    // contact state changed, reset integrated velocities
    for (i=0; i < rpc->position_indices.r_leg.size(); i++) {
      pdata->state.vref_integrator_state(rpc->position_indices.r_leg(i)) = qd(rpc->position_indices.r_leg(i));
    }
  }

  pdata->state.foot_contact_prev[0] = foot_contact[0];
  pdata->state.foot_contact_prev[1] = foot_contact[1];

  VectorXd qd_err = pdata->state.vref_integrator_state - qd;

  // do not velocity control ankles when in contact
  if (params->zero_ankles_on_contact && foot_contact[0] == 1) {
    for (i=0; i < rpc->position_indices.l_leg_ak.size(); i++) {
      qd_err(rpc->position_indices.l_leg_ak(i)) = 0;
    }
  }
  if (params->zero_ankles_on_contact && foot_contact[1] == 1) {
    for (i=0; i < rpc->position_indices.r_leg_ak.size(); i++) {
      qd_err(rpc->position_indices.r_leg_ak(i)) = 0;
    }
  }

  VectorXd qd_ref = qd_err.array().max(-params->delta_max);
  qd_ref = qd_ref.array().min(params->delta_max);
  return qd_ref;
}

std::vector<SupportStateElement> loadAvailableSupports(std::shared_ptr<drake::lcmt_qp_controller_input> qp_input) {
  // Parse a qp_input LCM message to extract its available supports as a vector of SupportStateElements
  std::vector<SupportStateElement> available_supports;
  available_supports.resize(qp_input->num_support_data);
  for (int i=0; i < qp_input->num_support_data; i++) {
    available_supports[i].body_idx = qp_input->support_data[i].body_id - 1;
    available_supports[i].contact_surface = qp_input->support_data[i].contact_surfaces;
    for (int j=0; j < 4; j++) {
      available_supports[i].support_logic_map[j] = qp_input->support_data[i].support_logic_map[j];
    }
    available_supports[i].contact_pts.resize(qp_input->support_data[i].num_contact_pts);
    for (int j=0; j < qp_input->support_data[i].num_contact_pts; j++) {
      for (int k = 0; k < 3; k++) {
        available_supports[i].contact_pts[j][k] = qp_input->support_data[i].contact_pts[k][j];
      }
      available_supports[i].contact_pts[j][3] = 1;
    }
  }
  return available_supports;
}

void applyJointPDOverride(const std::vector<drake::lcmt_joint_pd_override> &joint_pd_override, const DrakeRobotState &robot_state, PIDOutput &pid_out, VectorXd &w_qdd) {
  for (std::vector<drake::lcmt_joint_pd_override>::const_iterator it = joint_pd_override.begin(); it != joint_pd_override.end(); ++it) {
    int ind = it->position_ind - 1;
    double err_q = it->qi_des - robot_state.q(ind);
    double err_qd = it->qdi_des - robot_state.qd(ind);
    pid_out.qddot_des(ind) = it->kp * err_q + it->kd * err_qd;
    w_qdd(ind) = it->weight;
  }
}

int setupAndSolveQP(NewQPControllerData *pdata, std::shared_ptr<drake::lcmt_qp_controller_input> qp_input, DrakeRobotState &robot_state, const Ref<Matrix<bool, Dynamic, 1>> &b_contact_force, QPControllerOutput *qp_output, std::shared_ptr<QPControllerDebugData> debug) {
  // The primary solve loop for our controller. This constructs and solves a Quadratic Program and produces the instantaneous desired torques, along with reference positions, velocities, and accelerations. It mirrors the Matlab implementation in atlasControllers.InstantaneousQPController.setupAndSolveQP(), and more documentation can be found there. 
  // Note: argument `debug` MAY be set to NULL, which signals that no debug information is requested.

  // look up the param set by name
  AtlasParams *params; 
  std::map<string,AtlasParams>::iterator it;
  it = pdata->param_sets.find(qp_input->param_set_name);
  if (it == pdata->param_sets.end()) {
    mexWarnMsgTxt("Got a param set I don't recognize! Using standing params instead");
    it = pdata->param_sets.find("standing");
    if (it == pdata->param_sets.end()) {
      mexErrMsgTxt("Could not fall back to standing parameters either. I have to give up here.");
    }
  }
  // cout << "using params set: " + it->first + ", ";
  params = &(it->second);
  // mexPrintf("Kp_accel: %f, ", params->Kp_accel);

  int nu = pdata->B.cols();
  int nq = pdata->r->num_positions;

  // zmp_data
  Map<Matrix<double, 4, 4, RowMajor>> A_ls(&qp_input->zmp_data.A[0][0]);
  Map<Matrix<double, 4, 2, RowMajor>> B_ls(&qp_input->zmp_data.B[0][0]);
  Map<Matrix<double, 2, 4, RowMajor>> C_ls(&qp_input->zmp_data.C[0][0]);
  Map<Matrix<double, 2, 2, RowMajor>> D_ls(&qp_input->zmp_data.D[0][0]);
  Map<Matrix<double, 4, 1>> x0(&qp_input->zmp_data.x0[0][0]);
  Map<Matrix<double, 2, 1>> y0(&qp_input->zmp_data.y0[0][0]);
  Map<Matrix<double, 2, 1>> u0(&qp_input->zmp_data.u0[0][0]);
  Map<Matrix<double, 2, 2, RowMajor>> R_ls(&qp_input->zmp_data.R[0][0]);
  Map<Matrix<double, 2, 2, RowMajor>> Qy(&qp_input->zmp_data.Qy[0][0]);
  Map<Matrix<double, 4, 4, RowMajor>> S(&qp_input->zmp_data.S[0][0]);
  Map<Matrix<double, 4, 1>> s1(&qp_input->zmp_data.s1[0][0]);
  Map<Matrix<double, 4, 1>> s1dot(&qp_input->zmp_data.s1dot[0][0]);

  // // whole_body_data
  if (qp_input->whole_body_data.num_positions != nq) mexErrMsgTxt("number of positions doesn't match num_dof for this robot");
  Map<VectorXd> q_des(qp_input->whole_body_data.q_des.data(), nq);
  if (qp_input->whole_body_data.constrained_dofs.size() != qp_input->whole_body_data.num_constrained_dofs) {
    mexErrMsgTxt("size of constrained dofs does not match num_constrained_dofs");
  }
  Map<VectorXi> condof(qp_input->whole_body_data.constrained_dofs.data(), qp_input->whole_body_data.num_constrained_dofs);
  PIDOutput pid_out = wholeBodyPID(pdata, robot_state.t, robot_state.q, robot_state.qd, q_des, &params->whole_body);
  VectorXd w_qdd = params->whole_body.w_qdd;
  applyJointPDOverride(qp_input->joint_pd_override, robot_state, pid_out, w_qdd);

  qp_output->q_ref = pid_out.q_ref;

  // mu
  // NOTE: we're using the same mu for all supports
  double mu;
  if (qp_input->num_support_data == 0) {
    mu = 1.0;
  } else {
    mu = qp_input->support_data[0].mu;
    for (int i=1; i < qp_input->num_support_data; i++) {
      if (qp_input->support_data[i].mu != mu) {
        mexWarnMsgTxt("Currently, we assume that all supports have the same value of mu");
      }
    }
  }

  const int dim = 3, // 3D
  nd = 2*m_surface_tangents; // for friction cone approx, hard coded for now
  
  assert(nu+6 == nq);

  std::vector<DesiredBodyAcceleration> desired_body_accelerations;
  desired_body_accelerations.resize(qp_input->num_tracked_bodies);
  Vector6d body_pose_des, body_v_des, body_vdot_des;
  Vector6d body_vdot;

  for (int i=0; i < qp_input->num_tracked_bodies; i++) {
    int body_id0 = qp_input->body_motion_data[i].body_id - 1;
    double weight = params->body_motion[body_id0].weight;
    desired_body_accelerations[i].body_id0 = body_id0;
    Map<Matrix<double, 6, 4,RowMajor>>coefs_rowmaj(&qp_input->body_motion_data[i].coefs[0][0]);
    Matrix<double, 6, 4> coefs = coefs_rowmaj;
    double t_spline = std::max(qp_input->body_motion_data[i].ts[0], std::min(qp_input->body_motion_data[i].ts[1], robot_state.t));
    evaluateCubicSplineSegment(t_spline - qp_input->body_motion_data[i].ts[0], coefs, body_pose_des, body_v_des, body_vdot_des);
    desired_body_accelerations[i].body_vdot = bodyMotionPD(pdata->r, robot_state, body_id0, body_pose_des, body_v_des, body_vdot_des, params->body_motion[body_id0].Kp, params->body_motion[body_id0].Kd);
    desired_body_accelerations[i].weight = weight;
    desired_body_accelerations[i].accel_bounds = params->body_motion[body_id0].accel_bounds;
    // mexPrintf("body: %d, vdot: %f %f %f %f %f %f weight: %f\n", body_id0, 
    //           desired_body_accelerations[i].body_vdot(0), 
    //           desired_body_accelerations[i].body_vdot(1), 
    //           desired_body_accelerations[i].body_vdot(2), 
    //           desired_body_accelerations[i].body_vdot(3), 
    //           desired_body_accelerations[i].body_vdot(4), 
    //           desired_body_accelerations[i].body_vdot(5),
    //           weight);
      // mexPrintf("tracking body: %d, coefs[:,0]: %f %f %f %f %f %f coefs(", body_id0,
  }

  int n_body_accel_eq_constraints = 0;
  for (int i=0; i < desired_body_accelerations.size(); i++) {
    if (desired_body_accelerations[i].weight < 0)
      n_body_accel_eq_constraints++;
  }

  MatrixXd R_DQyD_ls = R_ls + D_ls.transpose()*Qy*D_ls;

  pdata->r->doKinematics(robot_state.q,false,robot_state.qd);

  //---------------------------------------------------------------------

  std::vector<SupportStateElement> available_supports = loadAvailableSupports(qp_input);
  std::vector<SupportStateElement> active_supports = getActiveSupports(pdata->r, pdata->map_ptr, robot_state.q, robot_state.qd, available_supports, b_contact_force, params->contact_threshold, pdata->default_terrain_height);

  int num_active_contact_pts=0;
  for (std::vector<SupportStateElement>::iterator iter = active_supports.begin(); iter!=active_supports.end(); iter++) {
    num_active_contact_pts += iter->contact_pts.size();
  }

  pdata->r->HandC(robot_state.q,robot_state.qd,(MatrixXd*)nullptr,pdata->H,pdata->C,(MatrixXd*)nullptr,(MatrixXd*)nullptr,(MatrixXd*)nullptr);

  pdata->H_float = pdata->H.topRows(6);
  pdata->H_act = pdata->H.bottomRows(nu);
  pdata->C_float = pdata->C.head(6);
  pdata->C_act = pdata->C.tail(nu);

  bool include_angular_momentum = (params->W_kdot.array().maxCoeff() > 1e-10);

  if (include_angular_momentum) {
    pdata->r->getCMM(robot_state.q,robot_state.qd,pdata->Ag,pdata->Agdot);
    pdata->Ak = pdata->Ag.topRows(3);
    pdata->Akdot = pdata->Agdot.topRows(3);
  }
  Vector3d xcom;
  // consider making all J's into row-major
  

  pdata->r->getCOM(xcom);
  pdata->r->getCOMJac(pdata->J);
  pdata->r->getCOMJacDot(pdata->Jdot);
  pdata->J_xy = pdata->J.topRows(2);
  pdata->Jdot_xy = pdata->Jdot.topRows(2);

  MatrixXd Jcom,Jcomdot;

  if (x0.size()==6) {
    Jcom = pdata->J;
    Jcomdot = pdata->Jdot;
  }
  else {
    Jcom = pdata->J_xy;
    Jcomdot = pdata->Jdot_xy;
  }
  
  MatrixXd B,JB,Jp,Jpdot,normals;
  int nc = contactConstraintsBV(pdata->r,num_active_contact_pts,mu,active_supports,pdata->map_ptr,B,JB,Jp,Jpdot,normals,pdata->default_terrain_height);
  int neps = nc*dim;

  VectorXd x_bar,xlimp;
  MatrixXd D_float(6,JB.cols()), D_act(nu,JB.cols());
  if (nc>0) {
    if (x0.size()==6) {
      // x,y,z com 
      xlimp.resize(6); 
      xlimp.topRows(3) = xcom;
      xlimp.bottomRows(3) = Jcom*robot_state.qd;
    }
    else {
      xlimp.resize(4); 
      xlimp.topRows(2) = xcom.topRows(2);
      xlimp.bottomRows(2) = Jcom*robot_state.qd;
    }
    x_bar = xlimp-x0;

    D_float = JB.topRows(6);
    D_act = JB.bottomRows(nu);
  }

  int nf = nc*nd; // number of contact force variables
  int nparams = nq+nf+neps;

  Vector3d kdot_des; 
  if (include_angular_momentum) {
    VectorXd k = pdata->Ak*robot_state.qd;
    kdot_des = -params->Kp_ang*k; // TODO: parameterize
  }
  
  //----------------------------------------------------------------------
  // QP cost function ----------------------------------------------------
  //
  //  min: ybar*Qy*ybar + ubar*R*ubar + (2*S*xbar + s1)*(A*x + B*u) +
  //    w_qdd*quad(qddot_ref - qdd) + w_eps*quad(epsilon) +
  //    w_grf*quad(beta) + quad(kdot_des - (A*qdd + Adot*qd))  
  VectorXd f(nparams);
  {      
    if (nc > 0) {
      // NOTE: moved Hqp calcs below, because I compute the inverse directly for FastQP (and sparse Hqp for gurobi)
      VectorXd tmp = C_ls*xlimp;
      VectorXd tmp1 = Jcomdot*robot_state.qd;
      MatrixXd tmp2 = R_DQyD_ls*Jcom;

      pdata->fqp = tmp.transpose()*Qy*D_ls*Jcom;
      // mexPrintf("fqp head: %f %f %f\n", pdata->fqp(0), pdata->fqp(1), pdata->fqp(2));
      pdata->fqp += tmp1.transpose()*tmp2;
      pdata->fqp += (S*x_bar + 0.5*s1).transpose()*B_ls*Jcom;
      pdata->fqp -= u0.transpose()*tmp2;
      pdata->fqp -= y0.transpose()*Qy*D_ls*Jcom;
      pdata->fqp -= (w_qdd.array()*pid_out.qddot_des.array()).matrix().transpose();
      if (include_angular_momentum) {
        pdata->fqp += robot_state.qd.transpose()*pdata->Akdot.transpose()*params->W_kdot*pdata->Ak;
        pdata->fqp -= kdot_des.transpose()*params->W_kdot*pdata->Ak;
      }
      f.head(nq) = pdata->fqp.transpose();
     } else {
      f.head(nq) = -pid_out.qddot_des;
    } 
  }
  f.tail(nf+neps) = VectorXd::Zero(nf+neps);
  
  int neq = 6+neps+6*n_body_accel_eq_constraints+qp_input->whole_body_data.num_constrained_dofs;
  MatrixXd Aeq = MatrixXd::Zero(neq,nparams);
  VectorXd beq = VectorXd::Zero(neq);
  
  // constrained floating base dynamics
  //  H_float*qdd - J_float'*lambda - Dbar_float*beta = -C_float
  Aeq.topLeftCorner(6,nq) = pdata->H_float;
  beq.topRows(6) = -pdata->C_float;
    
  if (nc>0) {
    Aeq.block(0,nq,6,nc*nd) = -D_float;
  }
  
  if (nc > 0) {
    // relative acceleration constraint
    Aeq.block(6,0,neps,nq) = Jp;
    Aeq.block(6,nq,neps,nf) = MatrixXd::Zero(neps,nf);  // note: obvious sparsity here
    Aeq.block(6,nq+nf,neps,neps) = MatrixXd::Identity(neps,neps);             // note: obvious sparsity here
    beq.segment(6,neps) = (-Jpdot -params->Kp_accel*Jp)*robot_state.qd; 
  }    
  
  // add in body spatial equality constraints
  // VectorXd body_vdot;
  MatrixXd orig = MatrixXd::Zero(4,1);
  orig(3,0) = 1;
  int equality_ind = 6+neps;
  MatrixXd Jb(6,nq);
  MatrixXd Jbdot(6,nq);
  for (int i=0; i<desired_body_accelerations.size(); i++) {
    if (desired_body_accelerations[i].weight < 0) { // negative implies constraint
      if (!inSupport(active_supports,desired_body_accelerations[i].body_id0)) {
        pdata->r->forwardJac(desired_body_accelerations[i].body_id0,orig,1,Jb);
        pdata->r->forwardJacDot(desired_body_accelerations[i].body_id0,orig,1,Jbdot);

        for (int j=0; j<6; j++) {
          if (!std::isnan(desired_body_accelerations[i].body_vdot(j))) {
            Aeq.block(equality_ind,0,1,nq) = Jb.row(j);
            beq[equality_ind++] = -Jbdot.row(j)*robot_state.qd + desired_body_accelerations[i].body_vdot(j);
          }
        }
      }
    }
  }

  if (qp_input->whole_body_data.num_constrained_dofs>0) {
    // add joint acceleration constraints
    for (int i=0; i<qp_input->whole_body_data.num_constrained_dofs; i++) {
      Aeq(equality_ind,(int)condof[i]-1) = 1;
      beq[equality_ind++] = pid_out.qddot_des[(int)condof[i]-1];
    }
  }  
  
  int n_ineq = 2*nu+2*6*desired_body_accelerations.size();
  MatrixXd Ain = MatrixXd::Zero(n_ineq,nparams);  // note: obvious sparsity here
  VectorXd bin = VectorXd::Zero(n_ineq);

  // linear input saturation constraints
  // u=B_act'*(H_act*qdd + C_act - Jz_act'*z - Dbar_act*beta)
  // using transpose instead of inverse because B is orthogonal
  Ain.topLeftCorner(nu,nq) = pdata->B_act.transpose()*pdata->H_act;
  Ain.block(0,nq,nu,nc*nd) = -pdata->B_act.transpose()*D_act;
  bin.head(nu) = -pdata->B_act.transpose()*pdata->C_act + pdata->umax;

  Ain.block(nu,0,nu,nparams) = -1*Ain.block(0,0,nu,nparams);
  bin.segment(nu,nu) = pdata->B_act.transpose()*pdata->C_act - pdata->umin;

  int constraint_start_index = 2*nu;
  for (int i=0; i<desired_body_accelerations.size(); i++) {
    pdata->r->forwardJac(desired_body_accelerations[i].body_id0,orig,1,Jb);
    pdata->r->forwardJacDot(desired_body_accelerations[i].body_id0,orig,1,Jbdot);
    Ain.block(constraint_start_index,0,6,pdata->r->num_positions) = Jb;
    bin.segment(constraint_start_index,6) = -Jbdot*robot_state.qd + desired_body_accelerations[i].accel_bounds.max;
    constraint_start_index += 6;
    Ain.block(constraint_start_index,0,6,pdata->r->num_positions) = -Jb;
    bin.segment(constraint_start_index,6) = Jbdot*robot_state.qd - desired_body_accelerations[i].accel_bounds.min;
    constraint_start_index += 6;
  }
       
  for (int i=0; i<n_ineq; i++) {
    // remove inf constraints---needed by gurobi
    if (std::isinf(double(bin(i)))) {
      Ain.row(i) = 0*Ain.row(i);
      bin(i)=0;
    }  
  }

  GRBmodel * model = nullptr;
  int info=-1;
  
  // set obj,lb,up
  VectorXd lb(nparams), ub(nparams);
  lb.head(nq) = pdata->qdd_lb;
  ub.head(nq) = pdata->qdd_ub;
  lb.segment(nq,nf) = VectorXd::Zero(nf);
  ub.segment(nq,nf) = 1e3*VectorXd::Ones(nf);
  lb.tail(neps) = -params->slack_limit*VectorXd::Ones(neps);
  ub.tail(neps) = params->slack_limit*VectorXd::Ones(neps);

  VectorXd alpha(nparams);

  MatrixXd Qnfdiag(nf,1), Qneps(neps,1);
  std::vector<MatrixXd*> QBlkDiag( nc>0 ? 3 : 1 );  // nq, nf, neps   // this one is for gurobi
  
  VectorXd w = (w_qdd.array() + REG).matrix();

  if (nc != pdata->state.num_active_contact_pts) {
    // Number of contact points has changed, so our active set is invalid
    pdata->state.active.clear();
  }
  pdata->state.num_active_contact_pts = nc;

  #ifdef USE_MATRIX_INVERSION_LEMMA
  double max_body_accel_weight = -numeric_limits<double>::infinity();
  for (int i=0; i < desired_body_accelerations.size(); i++) {
    max_body_accel_weight = max(max_body_accel_weight, desired_body_accelerations[i].weight);
  }
  bool include_body_accel_cost_terms = desired_body_accelerations.size() > 0 && max_body_accel_weight > 1e-10;
  if (pdata->use_fast_qp > 0 && !include_angular_momentum && !include_body_accel_cost_terms)
  { 
    // TODO: update to include angular momentum, body accel objectives.

    //    We want Hqp inverse, which I can compute efficiently using the
    //    matrix inversion lemma (see wikipedia):
    //    inv(A + U'CV) = inv(A) - inv(A)*U* inv([ inv(C)+ V*inv(A)*U ]) V inv(A)
    if (nc>0) {
      MatrixXd Wi = ((1/(w_qdd.array() + REG)).matrix()).asDiagonal();
      if (R_DQyD_ls.trace()>1e-15) { // R_DQyD_ls is not zero
        pdata->Hqp = Wi - Wi*Jcom.transpose()*(R_DQyD_ls.inverse() + Jcom*Wi*Jcom.transpose()).inverse()*Jcom*Wi;
      }
    } 
    else {
      pdata->Hqp = MatrixXd::Constant(nq,1,1/(1+REG));
    }

    #ifdef TEST_FAST_QP
      if (nc>0) {
        MatrixXd Hqp_test(nq,nq);
        MatrixXd W = w.asDiagonal();
        Hqp_test = (Jcom.transpose()*R_DQyD_ls*Jcom + W).inverse();
        if (((Hqp_test-pdata->Hqp).array().abs()).maxCoeff() > 1e-6) {
          mexErrMsgTxt("Q submatrix inverse from matrix inversion lemma does not match direct Q inverse.");
        }
      }
    #endif

    Qnfdiag = MatrixXd::Constant(nf,1,1/REG);
    Qneps = MatrixXd::Constant(neps,1,1/(.001+REG));

    QBlkDiag[0] = &pdata->Hqp;
    if (nc>0) {
      QBlkDiag[1] = &Qnfdiag;
      QBlkDiag[2] = &Qneps;     // quadratic slack var cost, Q(nparams-neps:end,nparams-neps:end)=eye(neps)
    }

    MatrixXd Ain_lb_ub(n_ineq+2*nparams,nparams);
    VectorXd bin_lb_ub(n_ineq+2*nparams);
    Ain_lb_ub << Ain,            // note: obvious sparsity here
        -MatrixXd::Identity(nparams,nparams),
        MatrixXd::Identity(nparams,nparams);
    bin_lb_ub << bin, -lb, ub;

    for (std::set<int>::iterator it = pdata->state.active.begin(); it != pdata->state.active.end(); it++) {
      if (std::isnan(bin_lb_ub(*it)) || std::isinf(bin_lb_ub(*it))) {
        pdata->state.active.clear();
        break;
      }
    }

    info = fastQPThatTakesQinv(QBlkDiag, f, Aeq, beq, Ain_lb_ub, bin_lb_ub, pdata->state.active, alpha);

    //if (info<0)   mexPrintf("fastQP info = %d.  Calling gurobi.\n", info);
  }
  else {
  #endif

    if (nc>0) {
      pdata->Hqp = Jcom.transpose()*R_DQyD_ls*Jcom;
      if (include_angular_momentum) {
        pdata->Hqp += pdata->Ak.transpose()*params->W_kdot*pdata->Ak;
      }
      pdata->Hqp += w_qdd.asDiagonal();
      pdata->Hqp += REG*MatrixXd::Identity(nq,nq);
    } else {
      pdata->Hqp = (1+REG)*MatrixXd::Identity(nq,nq);
    }

    // add in body spatial acceleration cost terms
    for (int i=0; i<desired_body_accelerations.size(); i++) {
      if (desired_body_accelerations[i].weight > 0) {
        if (!inSupport(active_supports,desired_body_accelerations[i].body_id0)) {
          pdata->r->forwardJac(desired_body_accelerations[i].body_id0,orig,1,Jb);
          pdata->r->forwardJacDot(desired_body_accelerations[i].body_id0,orig,1,Jbdot);

          for (int j=0; j<6; j++) {
            if (!std::isnan(desired_body_accelerations[i].body_vdot[j])) {
              pdata->Hqp += desired_body_accelerations[i].weight*(Jb.row(j)).transpose()*Jb.row(j);
              f.head(nq) += desired_body_accelerations[i].weight*(robot_state.qd.transpose()*Jbdot.row(j).transpose() - desired_body_accelerations[i].body_vdot[j])*Jb.row(j).transpose();
            }
          }
        }
      }
    }

    Qnfdiag = MatrixXd::Constant(nf,1,params->w_grf+REG);
    Qneps = MatrixXd::Constant(neps,1,params->w_slack+REG);

    QBlkDiag[0] = &pdata->Hqp;
    if (nc>0) {
      QBlkDiag[1] = &Qnfdiag;
      QBlkDiag[2] = &Qneps;     // quadratic slack var cost, Q(nparams-neps:end,nparams-neps:end)=eye(neps)
    }


    MatrixXd Ain_lb_ub(n_ineq+2*nparams,nparams);
    VectorXd bin_lb_ub(n_ineq+2*nparams);
    Ain_lb_ub << Ain,            // note: obvious sparsity here
        -MatrixXd::Identity(nparams,nparams),
        MatrixXd::Identity(nparams,nparams);
    bin_lb_ub << bin, -lb, ub;

    for (std::set<int>::iterator it = pdata->state.active.begin(); it != pdata->state.active.end(); it++) {
      if (std::isnan(bin_lb_ub(*it)) || std::isinf(bin_lb_ub(*it))) {
        pdata->state.active.clear();
        break;
      }
    }

    if (pdata->use_fast_qp > 0)
    { // set up and call fastqp
      info = fastQP(QBlkDiag, f, Aeq, beq, Ain_lb_ub, bin_lb_ub, pdata->state.active, alpha);
      //if (info<0)    mexPrintf("fastQP info=%d... calling Gurobi.\n", info);
    }
    else {
      // use gurobi active set 
      model = gurobiActiveSetQP(pdata->env,QBlkDiag,f,Aeq,beq,Ain,bin,lb,ub,pdata->state.vbasis,pdata->state.vbasis_len,pdata->state.cbasis,pdata->state.cbasis_len,alpha);
      CGE(GRBgetintattr(model,"NumVars",&(pdata->state.vbasis_len)), pdata->env);
      CGE(GRBgetintattr(model,"NumConstrs",&(pdata->state.cbasis_len)), pdata->env);
      info=66;
      //info = -1;
    }

    if (info<0) {
      model = gurobiQP(pdata->env,QBlkDiag,f,Aeq,beq,Ain,bin,lb,ub,pdata->state.active,alpha);
      int status; CGE(GRBgetintattr(model, "Status", &status), pdata->env);
      //if (status!=2) mexPrintf("Gurobi reports non-optimal status = %d\n", status);
    }
  #ifdef USE_MATRIX_INVERSION_LEMMA
  }
  #endif

  //----------------------------------------------------------------------
  // Solve for inputs ----------------------------------------------------
  qp_output->qdd = alpha.head(nq);
  VectorXd beta = alpha.segment(nq,nc*nd);

  // use transpose because B_act is orthogonal
  qp_output->u = pdata->B_act.transpose()*(pdata->H_act*qp_output->qdd + pdata->C_act - D_act*beta);
  for (int i=0; i < qp_output->u.size(); i++) {
    if (isnan(qp_output->u(i))) qp_output->u(i) = 0;
  }
  //y = pdata->B_act.jacobiSvd(ComputeThinU|ComputeThinV).solve(pdata->H_act*qdd + pdata->C_act - Jz_act.transpose()*lambda - D_act*beta);

  bool foot_contact[2];
  foot_contact[0] = b_contact_force(pdata->rpc.body_ids.l_foot) == 1;
  foot_contact[1] = b_contact_force(pdata->rpc.body_ids.r_foot) == 1;
  qp_output->qd_ref = velocityReference(pdata, robot_state.t, robot_state.q, robot_state.qd, qp_output->qdd, foot_contact, &(params->vref_integrator), &(pdata->rpc));

  // Remember t for next time around
  pdata->state.t_prev = robot_state.t;

  // If a debug pointer was passed in, fill it with useful data
  if (debug) {
    debug->active_supports.resize(active_supports.size());
    for (int i=0; i < active_supports.size(); i++) {
      debug->active_supports[i] = active_supports[i];
    }
    debug->nc = nc;
    debug->normals = normals;
    debug->B = B;
    debug->alpha = alpha;
    debug->f = f;
    debug->Aeq = Aeq;
    debug->beq = beq;
    debug->Ain_lb_ub = Ain_lb_ub;
    debug->bin_lb_ub = bin_lb_ub;
    debug->Qnfdiag = Qnfdiag;
    debug->Qneps = Qneps;
    debug->x_bar = x_bar;
    debug->S = S;
    debug->s1 = s1;
    debug->s1dot = s1dot;
    debug->s2dot = qp_input->zmp_data.s2dot;
    debug->A_ls = A_ls;
    debug->B_ls = B_ls;
    debug->Jcom = Jcom;
    debug->Jcomdot = Jcomdot;
    debug->beta = beta;
  }

  // if we used gurobi, clean up
  if (model) { 
    GRBfreemodel(model); 
  } 
  //  GRBfreeenv(env);

  return info;
}

