/* 
 * A c++ version of (significant pieces of) the QPController.m mimoOutput method. 
 *
 * Todo:
 *   switch to spatial accelerations in motion constraints
 *   use fixed-size matrices (or at least pre-allocated)
 *       for instance: #define nq 
 *       set MaxRowsAtCompileTime (http://eigen.tuxfamily.org/dox/TutorialMatrixClass.html)
 *   some matrices might be better off using RowMajor
 */

#include "QPCommon.h"
#include <Eigen/StdVector>
#include <limits>
#include <cmath>
#include "drakeUtil.h"

//#define TEST_FAST_QP
//#define USE_MATRIX_INVERSION_LEMMA

using namespace std;

// shared_ptr<WholeBodyParams> parseWholeBodyParams(const mxArray* params_obj, RigidBodyManipulator *r) {
//   const mxArray *wb_obj = myGetProperty(params_obj, "whole_body");
//   const mxArray *int_obj = myGetField(wb_obj, "integrator");
//   const mxArray *qddbound_obj = myGetField(wb_obj, "qdd_bounds");
//   int nq = r->num_dof;
//   int nv = r->num_velocities;

//   if (mxGetNumberOfElements(myGetField(wb_obj, "Kp")) != nq) mexErrMsgTxt("Kp should be of size nq");
//   if (mxGetNumberOfElements(myGetField(wb_obj, "Kd")) != nq) mexErrMsgTxt("Kd should be of size nq");
//   if (mxGetNumberOfElements(myGetField(wb_obj, "w_qdd")) != nv) mexErrMsgTxt("w_qdd should be of size nv");
//   if (mxGetNumberOfElements(myGetField(int_obj, "gains")) != nq) mexErrMsgTxt("gains should be of size nq");
//   if (mxGetNumberOfElements(myGetField(int_obj, "clamps")) != nq) mexErrMsgTxt("clamps should be of size nq");
//   if (mxGetNumberOfElements(myGetField(qddbound_obj, "min")) != nv) mexErrMsgTxt("qdd min should be of size nv");
//   if (mxGetNumberOfElements(myGetField(qddbound_obj, "max")) != nv) mexErrMsgTxt("qdd max should be of size nv");

//   shared_ptr<WholeBodyParams> params(
//     new WholeBodyParams(mxGetPr(myGetField(wb_obj, "Kp")), nq,
//                         mxGetPr(myGetField(wb_obj, "Kd")), nq,
//                         mxGetPr(myGetField(wb_obj, "w_qdd")), nv,
//                         mxGetPr(myGetField(int_obj, "gains")), nq,
//                         mxGetPr(myGetField(int_obj, "clamps")), nq,
//                         mxGetPr(myGetField(qddbound_obj, "min")), nv,
//                         mxGetPr(myGetField(qddbound_obj, "max")), nv));

//   params->damping_ratio = mxGetScalar(myGetField(wb_obj, "damping_ratio"));
//   params->integrator.eta = mxGetScalar(myGetField(int_obj, "eta"));
//   return params;
// }

// shared_ptr<RobotPropertyCache> parseRobotPropertyCache(const mxArray *rpc_obj) {
//   shared_ptr<RobotPropertyCache> rpc(new RobotPropertyCache());
//   const mxArray *pobj;

//   pobj = myGetField(myGetField(rpc_obj, "position_indices"), "r_leg_kny");
//   Map<VectorXd>r_leg_kny(mxGetPr(pobj), mxGetNumberOfElements(pobj));
//   rpc->position_indices.r_leg_kny = r_leg_kny.cast<int>();

//   pobj = myGetField(myGetField(rpc_obj, "position_indices"), "l_leg_kny");
//   Map<VectorXd>l_leg_kny(mxGetPr(pobj), mxGetNumberOfElements(pobj));
//   rpc->position_indices.l_leg_kny = l_leg_kny.cast<int>();

//   pobj = myGetField(myGetField(rpc_obj, "position_indices"), "r_leg");
//   Map<VectorXd>r_leg(mxGetPr(pobj), mxGetNumberOfElements(pobj));
//   rpc->position_indices.r_leg = r_leg.cast<int>();

//   pobj = myGetField(myGetField(rpc_obj, "position_indices"), "l_leg");
//   Map<VectorXd>l_leg(mxGetPr(pobj), mxGetNumberOfElements(pobj));
//   rpc->position_indices.l_leg = l_leg.cast<int>();

//   pobj = myGetField(myGetField(rpc_obj, "position_indices"), "r_leg_ak");
//   Map<VectorXd>r_leg_ak(mxGetPr(pobj), mxGetNumberOfElements(pobj));
//   rpc->position_indices.r_leg_ak = r_leg_ak.cast<int>();

//   pobj = myGetField(myGetField(rpc_obj, "position_indices"), "l_leg_ak");
//   Map<VectorXd>l_leg_ak(mxGetPr(pobj), mxGetNumberOfElements(pobj));
//   rpc->position_indices.l_leg_ak = l_leg_ak.cast<int>();

//   rpc->body_ids.r_foot = (int) mxGetScalar(myGetField(myGetField(rpc_obj, "body_ids"), "r_foot"));
//   rpc->body_ids.l_foot = (int) mxGetScalar(myGetField(myGetField(rpc_obj, "body_ids"), "l_foot"));
//   rpc->body_ids.pelvis = (int) mxGetScalar(myGetField(myGetField(rpc_obj, "body_ids"), "pelvis"));

//   pobj = myGetField(rpc_obj, "actuated_indices");
//   Map<VectorXd>actuated_indices(mxGetPr(pobj), mxGetNumberOfElements(pobj));
//   rpc->actuated_indices = actuated_indices.cast<int>();

//   return rpc;
// }


VectorXd wholeBodyPID(NewQPControllerData *pdata, double t, double *q, double *qd, VectorXd q_des, WholeBodyParams *params) {
  assert(q_des.size() == params->integrator.gains.size());
  double dt = 0;
  int nq = pdata->r->num_dof;
  int nv = pdata->r->num_velocities;
  if (nq != pdata->r->num_velocities) {
    mexErrMsgTxt("this function will need to be rewritten when num_pos != num_vel");
  }
  Map<VectorXd>q_vec(q, q_des.size(), nq);
  Map<VectorXd>qd_vec(qd, nv);
  if (pdata->state.t_prev != 0) {
    dt = t - pdata->state.t_prev;
  }
  pdata->state.q_integrator_state = params->integrator.eta * pdata->state.q_integrator_state + params->integrator.gains.cwiseProduct(q_des - q_vec) * dt;
  pdata->state.q_integrator_state = pdata->state.q_integrator_state.array().max(-params->integrator.clamps.array());
  pdata->state.q_integrator_state = pdata->state.q_integrator_state.array().min(params->integrator.clamps.array());
  q_des = q_des + pdata->state.q_integrator_state;
  q_des = q_des.array().max((pdata->r->joint_limit_min - params->integrator.clamps).array());
  q_des = q_des.array().min((pdata->r->joint_limit_max + params->integrator.clamps).array());

  pdata->state.q_integrator_state = pdata->state.q_integrator_state.array().max(-params->integrator.clamps.array());
  pdata->state.q_integrator_state = pdata->state.q_integrator_state.array().min(params->integrator.clamps.array());

  VectorXd err_q;
  err_q.resize(nq);
  err_q.head(3) = q_des.head(3) - q_vec.head(3);
  for (int j = 3; j < nq; j++) {
    err_q(j) = angleDiff(q_vec(j), q_des(j));
  }
  VectorXd qddot_des = params->Kp.cwiseProduct(err_q) - params->Kd.cwiseProduct(qd_vec);
  qddot_des = qddot_des.array().max(params->qdd_bounds.min.array());
  qddot_des = qddot_des.array().min(params->qdd_bounds.max.array());
  return qddot_des;
}

VectorXd velocityReference(NewQPControllerData *pdata, double t, double *q, double *qd, VectorXd qdd, bool foot_contact[2], VRefIntegratorParams *params, RobotPropertyCache *rpc) {
  int nv = pdata->r->num_velocities;
  int i;
  assert(qdd.size() == nv);
  Map<VectorXd> qd_vec(qd, nv);

  double dt = 0;
  if (pdata->state.t_prev != 0) {
    dt = t - pdata->state.t_prev;
  }

  pdata->state.vref_integrator_state = (1-params->eta)*pdata->state.vref_integrator_state + params->eta*qd_vec + qdd*dt;

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
      pdata->state.vref_integrator_state(rpc->position_indices.l_leg(i)) = qd_vec(rpc->position_indices.l_leg(i));
    }
  }
  if (pdata->state.foot_contact_prev[1] != foot_contact[1]) {
    // contact state changed, reset integrated velocities
    for (i=0; i < rpc->position_indices.r_leg.size(); i++) {
      pdata->state.vref_integrator_state(rpc->position_indices.r_leg(i)) = qd_vec(rpc->position_indices.r_leg(i));
    }
  }

  pdata->state.foot_contact_prev[0] = foot_contact[0];
  pdata->state.foot_contact_prev[1] = foot_contact[1];

  VectorXd qd_err = pdata->state.vref_integrator_state - qd_vec;

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

  double delta_max = 1.0;
  VectorXd v_ref = VectorXd::Zero(rpc->actuated_indices.size());
  for (i=0; i < rpc->actuated_indices.size(); i++) {
    v_ref(i) = qd_err(rpc->actuated_indices(i));
  }
  v_ref = v_ref.array().max(-delta_max);
  v_ref = v_ref.array().min(delta_max);
  return v_ref;
}


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs<1) mexErrMsgTxt("usage: alpha=QPControllermex(ptr,t,params_obj...,...)");
  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");

  struct NewQPControllerData* pdata;
  // struct QPControllerState* pstate;
  double* pr;

  // first get the ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:QPControllermex:BadInputs","the first argument should be the ptr");
  memcpy(&pdata,mxGetData(prhs[0]),sizeof(pdata));
  int nu = pdata->B.cols(), nq = pdata->r->num_dof;

  int narg=1;
  // if (!mxIsNumeric(prhs[narg]) || mxGetNumberOfElements(prhs[narg])!=1)
  //   mexErrMsgIdAndTxt("Drake:QPControllermex:BadInputs","the second argument should be the state ptr");
  // memcpy(&pstate,mxGetData(prhs[narg]),sizeof(pstate));
  // narg++;

  // now retrieve the runtime params from their matlab object

  // t
  double t = mxGetScalar(prhs[narg++]);

  // x
  double *q = mxGetPr(prhs[narg++]);
  double *qd = &q[nq];

  // zmp_data
  const mxArray* pobj = mxGetField(prhs[narg],0,"A");
  Map<MatrixXd> A_ls(mxGetPr(pobj), mxGetM(pobj), mxGetN(pobj));
  pobj = mxGetField(prhs[narg],0,"B");
  Map<MatrixXd> B_ls(mxGetPr(pobj), mxGetM(pobj), mxGetN(pobj));
  pobj = mxGetField(prhs[narg],0,"C");
  Map<MatrixXd> C_ls(mxGetPr(pobj), mxGetM(pobj), mxGetN(pobj));
  pobj = mxGetField(prhs[narg],0,"D");
  Map<MatrixXd> D_ls(mxGetPr(pobj), mxGetM(pobj), mxGetN(pobj));
  pobj = mxGetField(prhs[narg],0,"Qy");
  Map<MatrixXd> Qy(mxGetPr(pobj), mxGetM(pobj), mxGetN(pobj));
  pobj = mxGetField(prhs[narg],0,"R");
  Map<MatrixXd> R_ls(mxGetPr(pobj), mxGetM(pobj), mxGetN(pobj));
  pobj = mxGetField(prhs[narg],0,"S");
  Map<MatrixXd> S(mxGetPr(pobj), mxGetM(pobj), mxGetN(pobj));
  pobj = mxGetField(prhs[narg],0,"s1");
  Map<MatrixXd> s1(mxGetPr(pobj), mxGetM(pobj), mxGetN(pobj));
  pobj = mxGetField(prhs[narg],0,"s1dot");
  Map<MatrixXd> s1dot(mxGetPr(pobj), mxGetM(pobj), mxGetN(pobj));
  pobj = mxGetField(prhs[narg],0,"s2dot");
  double s2dot = mxGetScalar(pobj);
  pobj = mxGetField(prhs[narg],0,"x0");
  Map<MatrixXd> x0(mxGetPr(pobj), mxGetM(pobj), mxGetN(pobj));
  pobj = mxGetField(prhs[narg],0,"u0");
  Map<MatrixXd> u0(mxGetPr(pobj), mxGetM(pobj), mxGetN(pobj));
  pobj = mxGetField(prhs[narg],0,"y0");
  Map<MatrixXd> y0(mxGetPr(pobj), mxGetM(pobj), mxGetN(pobj));
  narg++;

  // mexPrintf("map test\n");
  MapTest map_test(mxGetPr(pobj), mxGetNumberOfElements(pobj));
  // map_test.A_data = mxGetPr(pobj);
  // Map<VectorXd>A_local(map_test.A_data, mxGetNumberOfElements(pobj));
  // map_test = new struct MapTest;
  // new (&map_test->A) Map<VectorXd>(mxGetPr(pobj), mxGetNumberOfElements(pobj));
  // mexPrintf("made map\n");
  // mexPrintf("A(1,3): %f A(1,4): %f\n", map_test->A(0,2), map_test->A(0,3));


  // whole_body_data
  pobj = prhs[narg];
  Map<VectorXd> q_des(mxGetPr(myGetField(pobj, "q_des")), nq);
  int num_condof;
  pobj = myGetField(pobj, "constrained_dofs");
  if (!mxIsEmpty(pobj)) {
    assert(mxGetN(pobj)==1);
  }
  num_condof=mxGetNumberOfElements(pobj);
  Map<VectorXd> condof(mxGetPr(pobj), num_condof);
  narg++;

  // mexPrintf("num condof: %d\n", num_condof);

  // body_motion_data
  const mxArray* acc_obj = prhs[narg];
  narg++;

  // available_supports
  const mxArray* available_supp_obj = prhs[narg];
  narg++;
  // int desired_support_argid = narg++;

  // contact_sensor
  pobj = prhs[narg];
  Map<VectorXd> contact_force_detected(mxGetPr(pobj), mxGetNumberOfElements(pobj), 1);
  Matrix<bool, Dynamic, 1> b_contact_force = Matrix<bool, Dynamic, 1>::Zero(contact_force_detected.size());
  for (int i=0; i < b_contact_force.size(); i++) {
    b_contact_force(i) = (contact_force_detected(i) != 0);
  }
  narg++;

  // use_fastqp
  int use_fast_qp = (int) mxGetScalar(prhs[narg]);
  narg++;

  // // qdd_lb
  // Map<VectorXd> qdd_lb(mxGetPr(prhs[narg]),mxGetM(prhs[narg])); narg++;

  // // qdd_ub
  // Map<VectorXd> qdd_ub(mxGetPr(prhs[narg]),mxGetM(prhs[narg])); narg++;

  // mu
  double mu = mxGetScalar(prhs[narg++]);

  // height
  double terrain_height = mxGetScalar(prhs[narg++]); // nonzero if we're using DRCFlatTerrainMap

  // // robot_property_cache
  // shared_ptr<RobotPropertyCache> rpc = parseRobotPropertyCache(prhs[narg]);
  // narg++;

  // params set name
  string param_set_name = mxArrayToString(prhs[narg]);
  AtlasParams *params; 
  if (!param_set_name.compare("walking")) {
    params = &(pdata->param_sets.walking);
  } else if (!param_set_name.compare("standing")) {
    params = &(pdata->param_sets.standing);
  } else if (!param_set_name.compare("kinematic")) {
    params = &(pdata->param_sets.kinematic);
  } else {
    params = &(pdata->param_sets.standing); // just so we don't get a compiler warning
    mexWarnMsgTxt("Got a param set I don't recognize! Using standing params instead");
  }
  narg++;

  const int dim = 3, // 3D
  nd = 2*m_surface_tangents; // for friction cone approx, hard coded for now
  
  assert(nu+6 == nq);

  
  VectorXd qddot_des = wholeBodyPID(pdata, t, q, qd, q_des, &(params->whole_body));
  // Map< VectorXd > qddot_des(mxGetPr(prhs[narg++]),nq);
  
  int n_body_accel_inputs = (int) mxGetN(acc_obj);
  VectorXd body_accel_input_weights = VectorXd::Zero(n_body_accel_inputs);
  VectorXi accel_bound_body_idx = VectorXi::Zero(n_body_accel_inputs);
  vector<Vector6d> min_body_acceleration;
  min_body_acceleration.resize(n_body_accel_inputs);
  vector<Vector6d> max_body_acceleration;
  max_body_acceleration.resize(n_body_accel_inputs);

  vector<Vector6d,aligned_allocator<Vector6d>> body_accel_inputs;
  Vector6d body_des, body_v_des, body_vdot_des;
  for (int i=0; i < n_body_accel_inputs; i++) {
    int body_id = (int) mxGetScalar(myGetField(acc_obj, i, "body_id")) - 1;
    double t0 = mxGetScalar(myGetField(acc_obj, i, "ts")); // gets the first element in the list
    const mxArray *coefs_obj = myGetField(acc_obj, i, "coefs");

    if (mxGetNumberOfDimensions(coefs_obj) != 3) {
      mexErrMsgTxt("coefs obj should be a 3-d matrix");
    }
    const int *dim = mxGetDimensions(coefs_obj);
    if (dim[0] != 6 || dim[1] != 1 || dim[2] != 4) {
      mexErrMsgTxt("coefs should be a 6x1x4");
    }
    Map<Matrix<double, 6, 4>>coefs(mxGetPr(coefs_obj));

    evaluateCubicSplineSegment(t-t0, coefs, body_des, body_v_des, body_vdot_des);
    Vector6d body_vdot = bodyMotionPD(pdata->r, q, qd, body_id, body_des, body_v_des, body_vdot_des, params->body_motion[body_id].Kp, params->body_motion[body_id].Kd);

    accel_bound_body_idx[i] = body_id;

    body_accel_inputs.push_back(body_vdot);

    min_body_acceleration[i] = params->body_motion[body_id].accel_bounds.min;
    max_body_acceleration[i] = params->body_motion[body_id].accel_bounds.max;
    body_accel_input_weights[i] = params->body_motion[body_id].weight;
  }

  int n_body_accel_eq_constraints = 0;
  for (int i=0; i < n_body_accel_inputs; i++) {
    if (body_accel_input_weights(i) < 0)
      n_body_accel_eq_constraints++;
  }

  MatrixXd R_DQyD_ls = R_ls + D_ls.transpose()*Qy*D_ls;

  pdata->r->doKinematics(q,false,qd);

  //---------------------------------------------------------------------

  int num_active_contact_pts=0;
  vector<SupportStateElement> available_supports = parseSupportData(available_supp_obj);
  vector<SupportStateElement> active_supports = getActiveSupports(pdata->r, pdata->map_ptr, q, qd, available_supports, b_contact_force, params->contact_threshold, terrain_height);

  for (vector<SupportStateElement>::iterator iter = active_supports.begin(); iter!=active_supports.end(); iter++) {
    num_active_contact_pts += iter->contact_pts.size();
  }

  pdata->r->HandC(q,qd,(MatrixXd*)NULL,pdata->H,pdata->C,(MatrixXd*)NULL,(MatrixXd*)NULL,(MatrixXd*)NULL);

  pdata->H_float = pdata->H.topRows(6);
  pdata->H_act = pdata->H.bottomRows(nu);
  pdata->C_float = pdata->C.head(6);
  pdata->C_act = pdata->C.tail(nu);

  bool include_angular_momentum = (params->W_kdot.array().maxCoeff() > 1e-10);

  if (include_angular_momentum) {
    pdata->r->getCMM(q,qd,pdata->Ag,pdata->Agdot);
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
  Map<VectorXd> qdvec(qd,nq);
  
  MatrixXd B,JB,Jp,Jpdot,normals;
  int nc = contactConstraintsBV(pdata->r,num_active_contact_pts,mu,active_supports,pdata->map_ptr,B,JB,Jp,Jpdot,normals,terrain_height);
  int neps = nc*dim;

  VectorXd x_bar,xlimp;
  MatrixXd D_float(6,JB.cols()), D_act(nu,JB.cols());
  if (nc>0) {
    if (x0.size()==6) {
      // x,y,z com 
      xlimp.resize(6); 
      xlimp.topRows(3) = xcom;
      xlimp.bottomRows(3) = Jcom*qdvec;
    }
    else {
      xlimp.resize(4); 
      xlimp.topRows(2) = xcom.topRows(2);
      xlimp.bottomRows(2) = Jcom*qdvec;
    }
    x_bar = xlimp-x0;

    D_float = JB.topRows(6);
    D_act = JB.bottomRows(nu);
  }

  int nf = nc*nd; // number of contact force variables
  int nparams = nq+nf+neps;

  Vector3d kdot_des; 
  if (include_angular_momentum) {
    VectorXd k = pdata->Ak*qdvec;
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
      VectorXd tmp1 = Jcomdot*qdvec;
      MatrixXd tmp2 = R_DQyD_ls*Jcom;

      pdata->fqp = tmp.transpose()*Qy*D_ls*Jcom;
      pdata->fqp += tmp1.transpose()*tmp2;
      pdata->fqp += (S*x_bar + 0.5*s1).transpose()*B_ls*Jcom;
      pdata->fqp -= u0.transpose()*tmp2;
      pdata->fqp -= y0.transpose()*Qy*D_ls*Jcom;
      pdata->fqp -= (params->whole_body.w_qdd.array()*qddot_des.array()).matrix().transpose();
      if (include_angular_momentum) {
        pdata->fqp += qdvec.transpose()*pdata->Akdot.transpose()*params->W_kdot*pdata->Ak;
        pdata->fqp -= kdot_des.transpose()*params->W_kdot*pdata->Ak;
      }
      f.head(nq) = pdata->fqp.transpose();
     } else {
      f.head(nq) = -qddot_des;
    } 
  }
  f.tail(nf+neps) = VectorXd::Zero(nf+neps);
  
  int neq = 6+neps+6*n_body_accel_eq_constraints+num_condof;
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
    beq.segment(6,neps) = (-Jpdot -params->Kp_accel*Jp)*qdvec; 
  }    
  
  // add in body spatial equality constraints
  VectorXd body_vdot;
  MatrixXd orig = MatrixXd::Zero(4,1);
  orig(3,0) = 1;
  int body_idx;
  int equality_ind = 6+neps;
  MatrixXd Jb(6,nq);
  MatrixXd Jbdot(6,nq);
  for (int i=0; i<n_body_accel_inputs; i++) {
    if (body_accel_input_weights(i) < 0) {
      // negative implies constraint
      body_vdot = body_accel_inputs[i];
      body_idx = accel_bound_body_idx[i];

      if (!inSupport(active_supports,body_idx)) {
        pdata->r->forwardJac(body_idx,orig,1,Jb);
        pdata->r->forwardJacDot(body_idx,orig,1,Jbdot);

        for (int j=0; j<6; j++) {
          if (!std::isnan(body_vdot[j])) {
            Aeq.block(equality_ind,0,1,nq) = Jb.row(j);
            beq[equality_ind++] = -Jbdot.row(j)*qdvec + body_vdot[j];
          }
        }
      }
    }
  }

  if (num_condof>0) {
    // add joint acceleration constraints
    for (int i=0; i<num_condof; i++) {
      Aeq(equality_ind,(int)condof[i]-1) = 1;
      beq[equality_ind++] = qddot_des[(int)condof[i]-1];
    }
  }  
  
  int n_ineq = 2*nu+2*6*n_body_accel_inputs;
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

  int body_index;
  int constraint_start_index = 2*nu;
  for (int i=0; i<n_body_accel_inputs; i++) {
    body_index = accel_bound_body_idx[i];
    pdata->r->forwardJac(body_index,orig,1,Jb);
    pdata->r->forwardJacDot(body_index,orig,1,Jbdot);
    Ain.block(constraint_start_index,0,6,pdata->r->num_dof) = Jb;
    bin.segment(constraint_start_index,6) = -Jbdot*qdvec + max_body_acceleration[i];
    constraint_start_index += 6;
    Ain.block(constraint_start_index,0,6,pdata->r->num_dof) = -Jb;
    bin.segment(constraint_start_index,6) = Jbdot*qdvec - min_body_acceleration[i];
    constraint_start_index += 6;
  }
       
  for (int i=0; i<n_ineq; i++) {
    // remove inf constraints---needed by gurobi
    if (std::isinf(double(bin(i)))) {
      Ain.row(i) = 0*Ain.row(i);
      bin(i)=0;
    }  
  }

  GRBmodel * model = NULL;
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
  vector<MatrixXd*> QBlkDiag( nc>0 ? 3 : 1 );  // nq, nf, neps   // this one is for gurobi
  
  VectorXd w = (params->whole_body.w_qdd.array() + REG).matrix();
  #ifdef USE_MATRIX_INVERSION_LEMMA
  bool include_body_accel_cost_terms = n_body_accel_inputs > 0 && body_accel_input_weights.array().maxCoeff() > 1e-10;
  if (use_fast_qp > 0 && !include_angular_momentum && !include_body_accel_cost_terms)
  { 
    // TODO: update to include angular momentum, body accel objectives.

  	//    We want Hqp inverse, which I can compute efficiently using the
  	//    matrix inversion lemma (see wikipedia):
  	//    inv(A + U'CV) = inv(A) - inv(A)*U* inv([ inv(C)+ V*inv(A)*U ]) V inv(A)
  	if (nc>0) {
      MatrixXd Wi = ((1/(params->whole_body.w_qdd.array() + REG)).matrix()).asDiagonal();
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
    Ain_lb_ub << Ain, 			     // note: obvious sparsity here
    		-MatrixXd::Identity(nparams,nparams),
    		MatrixXd::Identity(nparams,nparams);
    bin_lb_ub << bin, -lb, ub;

    info = fastQPThatTakesQinv(QBlkDiag, f, Aeq, beq, Ain_lb_ub, bin_lb_ub, pdata->state.active, alpha);

    //if (info<0)  	mexPrintf("fastQP info = %d.  Calling gurobi.\n", info);
  }
  else {
  #endif

    if (nc>0) {
      pdata->Hqp = Jcom.transpose()*R_DQyD_ls*Jcom;
      if (include_angular_momentum) {
        pdata->Hqp += pdata->Ak.transpose()*params->W_kdot*pdata->Ak;
      }
      pdata->Hqp += params->whole_body.w_qdd.asDiagonal();
      pdata->Hqp += REG*MatrixXd::Identity(nq,nq);
    } else {
      pdata->Hqp = (1+REG)*MatrixXd::Identity(nq,nq);
    }

    // add in body spatial acceleration cost terms
    double w_i;
    for (int i=0; i<n_body_accel_inputs; i++) {
      w_i=body_accel_input_weights(i);
      if (w_i > 0) {
        body_vdot = body_accel_inputs[i];
        body_idx = accel_bound_body_idx[i];
        
        if (!inSupport(active_supports,body_idx)) {
          pdata->r->forwardJac(body_idx,orig,1,Jb);
          pdata->r->forwardJacDot(body_idx,orig,1,Jbdot);

          for (int j=0; j<6; j++) {
            if (!std::isnan(body_vdot[j])) {
              pdata->Hqp += w_i*(Jb.row(j)).transpose()*Jb.row(j);
              f.head(nq) += w_i*(qdvec.transpose()*Jbdot.row(j).transpose() - body_vdot[j])*Jb.row(j).transpose();
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


    if (use_fast_qp > 0)
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
  VectorXd y(nu);
  VectorXd qdd = alpha.head(nq);
  VectorXd beta = alpha.segment(nq,nc*nd);

  // use transpose because B_act is orthogonal
  y = pdata->B_act.transpose()*(pdata->H_act*qdd + pdata->C_act - D_act*beta);
  //y = pdata->B_act.jacobiSvd(ComputeThinU|ComputeThinV).solve(pdata->H_act*qdd + pdata->C_act - Jz_act.transpose()*lambda - D_act*beta);

  bool foot_contact[2];
  foot_contact[0] = contact_force_detected[pdata->rpc.body_ids.r_foot] == 1;
  foot_contact[1] = contact_force_detected[pdata->rpc.body_ids.l_foot] == 1;

  VectorXd v_ref = velocityReference(pdata, t, q, qd, qdd, foot_contact, &(params->vref_integrator), &(pdata->rpc));
  pdata->state.t_prev = t;
  
  narg = 0;
  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(y);
  }
  narg++;
  
  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(qdd);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(v_ref);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = mxCreateNumericMatrix(1,1,mxINT32_CLASS,mxREAL);
    memcpy(mxGetData(plhs[narg]),&info,sizeof(int));
  }
  narg++;

  if (nlhs>narg) {
      plhs[narg] = mxCreateDoubleMatrix(1,active_supports.size(),mxREAL);
      pr = mxGetPr(plhs[narg]);
      int i=0;
      for (vector<SupportStateElement>::iterator iter = active_supports.begin(); iter!=active_supports.end(); iter++) {
          pr[i++] = (double) (iter->body_idx + 1);
      }
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(alpha);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(pdata->Hqp);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(f);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(Aeq);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(beq);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(Ain_lb_ub);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(bin_lb_ub);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(Qnfdiag);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(Qneps);
  }
  narg++;

  if (nlhs>narg) {
    double Vdot;
    if (nc>0) 
      // note: Sdot is 0 for ZMP/double integrator dynamics, so we omit that term here
      Vdot = ((2*x_bar.transpose()*S + s1.transpose())*(A_ls*x_bar + B_ls*(Jcomdot*qdvec + Jcom*qdd)) + s1dot.transpose()*x_bar)(0) + s2dot;
    else
      Vdot = 0;
    plhs[narg] = mxCreateDoubleScalar(Vdot);
  }
  narg++;

  if (nlhs>narg) {
    RigidBodyManipulator* r = pdata->r;

    VectorXd individual_cops = individualSupportCOPs(r, active_supports, normals, B, beta);
    plhs[narg] = eigenToMatlab(individual_cops);
  }
  narg++;

  if (model) { 
    GRBfreemodel(model); 
  } 
  //  GRBfreeenv(env);
} 
