#include "QPCommon.h"
#include <Eigen/StdVector>
// #include <limits>
// #include <cmath>
// #include "drakeUtil.h"

using namespace std;

void parseWholeBodyParams(const mxArray *params_obj, RigidBodyManipulator *r, WholeBodyParams *params) {
  const mxArray *int_obj = myGetField(params_obj, "integrator");
  const mxArray *qddbound_obj = myGetField(params_obj, "qdd_bounds");
  int nq = r->num_positions;
  int nv = r->num_velocities;

  if (mxGetNumberOfElements(myGetField(params_obj, "Kp")) != nq) mexErrMsgTxt("Kp should be of size nq");
  if (mxGetNumberOfElements(myGetField(params_obj, "Kd")) != nq) mexErrMsgTxt("Kd should be of size nq");
  if (mxGetNumberOfElements(myGetField(params_obj, "w_qdd")) != nv) mexErrMsgTxt("w_qdd should be of size nv");
  if (mxGetNumberOfElements(myGetField(int_obj, "gains")) != nq) mexErrMsgTxt("gains should be of size nq");
  if (mxGetNumberOfElements(myGetField(int_obj, "clamps")) != nq) mexErrMsgTxt("clamps should be of size nq");
  if (mxGetNumberOfElements(myGetField(qddbound_obj, "min")) != nv) mexErrMsgTxt("qdd min should be of size nv");
  if (mxGetNumberOfElements(myGetField(qddbound_obj, "max")) != nv) mexErrMsgTxt("qdd max should be of size nv");

  Map<VectorXd>Kp(mxGetPr(myGetField(params_obj, "Kp")), mxGetNumberOfElements(myGetField(params_obj, "Kp")));
  params->Kp = Kp;

  Map<VectorXd>Kd(mxGetPr(myGetField(params_obj, "Kd")), mxGetNumberOfElements(myGetField(params_obj, "Kd")));
  params->Kd = Kd;

  Map<VectorXd>w_qdd(mxGetPr(myGetField(params_obj, "w_qdd")), mxGetNumberOfElements(myGetField(params_obj, "w_qdd")));
  params->w_qdd = w_qdd;

  Map<VectorXd>gains(mxGetPr(myGetField(int_obj, "gains")), mxGetNumberOfElements(myGetField(int_obj, "gains")));
  params->integrator.gains = gains;

  Map<VectorXd>clamps(mxGetPr(myGetField(int_obj, "clamps")), mxGetNumberOfElements(myGetField(int_obj, "clamps")));
  params->integrator.clamps = clamps;

  Map<VectorXd>min(mxGetPr(myGetField(qddbound_obj, "min")), mxGetNumberOfElements(myGetField(qddbound_obj, "min")));
  params->qdd_bounds.min = min;

  Map<VectorXd>max(mxGetPr(myGetField(qddbound_obj, "max")), mxGetNumberOfElements(myGetField(qddbound_obj, "max")));
  params->qdd_bounds.max = max;
  return;
}

void parseBodyMotionParams(const mxArray *params_obj, int i, BodyMotionParams *params) {
  const mxArray *pobj;
  const mxArray *bounds_obj = myGetField(params_obj, i, "accel_bounds");

  pobj = myGetField(params_obj, i, "Kp");
  sizecheck(pobj, 6, 1);
  Map<Vector6d>Kp(mxGetPr(pobj));
  params->Kp = Kp;

  pobj = myGetField(params_obj, i, "Kd");
  sizecheck(pobj, 6, 1);
  Map<Vector6d>Kd(mxGetPr(pobj));
  params->Kd = Kd;

  pobj = myGetField(params_obj, i, "weight");
  sizecheck(pobj, 1, 1);
  params->weight = mxGetScalar(pobj);

  pobj = myGetField(bounds_obj, "min");
  sizecheck(pobj, 6, 1);
  Map<Vector6d>min(mxGetPr(pobj));
  params->accel_bounds.min = min;

  pobj = myGetField(bounds_obj, "max");
  sizecheck(pobj, 6, 1);
  Map<Vector6d>max(mxGetPr(pobj));
  params->accel_bounds.max = max;
  return;
}

void parseVRefIntegratorParams(const mxArray *params_obj, VRefIntegratorParams *params) {
  const mxArray *pobj;

  pobj = myGetField(params_obj, "zero_ankles_on_contact");
  if (!mxIsDouble(pobj)) mexErrMsgTxt("zero_ankles_on_contact should be a double (yes, even though it's treated as a logical. sorry...)");
  sizecheck(pobj, 1, 1);
  params->zero_ankles_on_contact = (mxGetScalar(pobj) != 0);

  pobj = myGetField(params_obj, "eta");
  sizecheck(pobj, 1, 1);
  params->eta = mxGetScalar(pobj);
  return;
}

void parseAtlasParams(const mxArray *params_obj, RigidBodyManipulator *r, AtlasParams *params) {
  const mxArray *pobj;

  pobj = myGetProperty(params_obj, "W_kdot");
  sizecheck(pobj, 3, 3);
  Map<Matrix3d>W_kdot(mxGetPr(pobj));
  params->W_kdot = W_kdot;

  pobj = myGetProperty(params_obj, "Kp_ang");
  sizecheck(pobj, 1, 1);
  params->Kp_ang = mxGetScalar(pobj);

  pobj = myGetProperty(params_obj, "w_slack");
  sizecheck(pobj, 1, 1);
  params->w_slack = mxGetScalar(pobj);

  pobj = myGetProperty(params_obj, "slack_limit");
  sizecheck(pobj, 1, 1);
  params->slack_limit = mxGetScalar(pobj);

  pobj = myGetProperty(params_obj, "w_grf");
  sizecheck(pobj, 1, 1);
  params->w_grf = mxGetScalar(pobj);

  pobj = myGetProperty(params_obj, "Kp_accel");
  sizecheck(pobj, 1, 1);
  params->Kp_accel = mxGetScalar(pobj);

  pobj = myGetProperty(params_obj, "contact_threshold");
  sizecheck(pobj, 1, 1);
  params->contact_threshold = mxGetScalar(pobj);

  pobj = myGetProperty(params_obj, "min_knee_angle");
  sizecheck(pobj, 1, 1);
  params->min_knee_angle = mxGetScalar(pobj);

  parseWholeBodyParams(myGetProperty(params_obj, "whole_body"), r, &(params->whole_body));
  parseVRefIntegratorParams(myGetProperty(params_obj, "vref_integrator"), &(params->vref_integrator));

  BodyMotionParams body_motion_params;
  const mxArray *body_motion_obj = myGetProperty(params_obj, "body_motion");
  int num_tracked_bodies = mxGetNumberOfElements(body_motion_obj);
  params->body_motion.resize(num_tracked_bodies);
  for (int i=0; i < num_tracked_bodies; i++) {
    parseBodyMotionParams(body_motion_obj, i, &body_motion_params);
    params->body_motion[i] = body_motion_params;
  }
  return;
}

void parseAtlasParamSets(const mxArray *pobj, RigidBodyManipulator *r, map<string,AtlasParams> *param_sets) {
  int num_fields = mxGetNumberOfFields(pobj);
  if (num_fields == 0) mexErrMsgTxt("could not get any field names from the param_sets object\n"); 

  AtlasParams params;
  const char* fieldname;
  for (int i=0; i < num_fields; i++) {
    fieldname = mxGetFieldNameByNumber(pobj, i);
    parseAtlasParams(myGetField(pobj, fieldname), r, &params);
    param_sets->insert(pair<string,AtlasParams>(string(fieldname), params));
  }

  return;
}

void parseRobotPropertyCache(const mxArray *rpc_obj, RobotPropertyCache *rpc) {
  const mxArray *pobj;

  pobj = myGetField(myGetField(rpc_obj, "position_indices"), "r_leg_kny");
  Map<VectorXd>r_leg_kny(mxGetPr(pobj), mxGetNumberOfElements(pobj));
  rpc->position_indices.r_leg_kny = r_leg_kny.cast<int>();

  pobj = myGetField(myGetField(rpc_obj, "position_indices"), "l_leg_kny");
  Map<VectorXd>l_leg_kny(mxGetPr(pobj), mxGetNumberOfElements(pobj));
  rpc->position_indices.l_leg_kny = l_leg_kny.cast<int>();

  pobj = myGetField(myGetField(rpc_obj, "position_indices"), "r_leg");
  Map<VectorXd>r_leg(mxGetPr(pobj), mxGetNumberOfElements(pobj));
  rpc->position_indices.r_leg = r_leg.cast<int>();

  pobj = myGetField(myGetField(rpc_obj, "position_indices"), "l_leg");
  Map<VectorXd>l_leg(mxGetPr(pobj), mxGetNumberOfElements(pobj));
  rpc->position_indices.l_leg = l_leg.cast<int>();

  pobj = myGetField(myGetField(rpc_obj, "position_indices"), "r_leg_ak");
  Map<VectorXd>r_leg_ak(mxGetPr(pobj), mxGetNumberOfElements(pobj));
  rpc->position_indices.r_leg_ak = r_leg_ak.cast<int>();

  pobj = myGetField(myGetField(rpc_obj, "position_indices"), "l_leg_ak");
  Map<VectorXd>l_leg_ak(mxGetPr(pobj), mxGetNumberOfElements(pobj));
  rpc->position_indices.l_leg_ak = l_leg_ak.cast<int>();

  rpc->body_ids.r_foot = (int) mxGetScalar(myGetField(myGetField(rpc_obj, "body_ids"), "r_foot"));
  rpc->body_ids.l_foot = (int) mxGetScalar(myGetField(myGetField(rpc_obj, "body_ids"), "l_foot"));
  rpc->body_ids.pelvis = (int) mxGetScalar(myGetField(myGetField(rpc_obj, "body_ids"), "pelvis"));

  pobj = myGetField(rpc_obj, "actuated_indices");
  Map<VectorXd>actuated_indices(mxGetPr(pobj), mxGetNumberOfElements(pobj));
  rpc->actuated_indices = actuated_indices.cast<int>();

  return;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs<1) mexErrMsgTxt("usage: ptr = constructQPDataPointerMex(robot_obj, params_sets, robot_property_cache, B, umin, umax, terrain_map_ptr, gurobi_opts);");

  if (nrhs == 1) {
    // By convention, calling the constructor with just one argument (the pointer) should delete the pointer
    if (isa(prhs[0],"DrakeMexPointer")) { 
      destroyDrakeMexPointer<NewQPControllerData*>(prhs[0]);
      return;
    } else {
      mexErrMsgIdAndTxt("Drake:constructQPDataPointerMex:BadInputs", "Expected a DrakeMexPointer (or a subclass)");
    }
  }


  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");

  struct NewQPControllerData* pdata;
  pdata = new struct NewQPControllerData;

  int narg = 0;
  
  // robot_obj
  pdata->r = (RigidBodyManipulator*) getDrakeMexPointer(prhs[narg]);
  narg++;

  // param_sets
  parseAtlasParamSets(prhs[narg], pdata->r, &(pdata->param_sets));
  narg++;

  // robot_property_cache
  parseRobotPropertyCache(prhs[narg], &(pdata->rpc));
  narg++;

  // B
  pdata->B.resize(mxGetM(prhs[narg]),mxGetN(prhs[narg]));
  memcpy(pdata->B.data(),mxGetPr(prhs[narg]),sizeof(double)*mxGetM(prhs[narg])*mxGetN(prhs[narg]));
  narg++;

  // umin
  int nq = pdata->r->num_positions, nu = pdata->B.cols();
  pdata->umin.resize(nu);
  pdata->umax.resize(nu);
  memcpy(pdata->umin.data(),mxGetPr(prhs[narg++]),sizeof(double)*nu);

  // umax
  memcpy(pdata->umax.data(),mxGetPr(prhs[narg++]),sizeof(double)*nu);

  // terrain_map_ptr
  if (!mxIsNumeric(prhs[narg]) || mxGetNumberOfElements(prhs[narg])!=1) mexErrMsgIdAndTxt("Drake:constructQPDataPointerMex:BadInputs","this should be a map pointer");
  memcpy(&pdata->map_ptr,mxGetPr(prhs[narg]),sizeof(pdata->map_ptr));
  if (!pdata->map_ptr) mexWarnMsgTxt("Map ptr is NULL. Assuming flat ground.");
  narg++;

  // default_terrain_height
  pdata->default_terrain_height = mxGetScalar(prhs[narg]);
  narg++;

  // use_fast_qp
  pdata->use_fast_qp = (int) mxGetScalar(prhs[narg]);
  narg++;

  // gurobi_opts
  const mxArray* psolveropts = prhs[narg];

  // Done parsing inputs

  pdata->B_act.resize(nu,nu);
  pdata->B_act = pdata->B.bottomRows(nu);

  pdata->qdd_lb = VectorXd::Zero(nq).array() - numeric_limits<double>::infinity();
  pdata->qdd_ub = VectorXd::Zero(nq).array() + numeric_limits<double>::infinity();

  //create gurobi environment
  int error = GRBloadenv(&(pdata->env),NULL);
  if (error)
    mexErrMsgTxt("Cannot load gurobi environment.");

  // set solver params (http://www.gurobi.com/documentation/5.5/reference-manual/node798#sec:Parameters)
  int method = (int) mxGetScalar(myGetField(psolveropts,"method"));
  CGE ( GRBsetintparam(pdata->env,"outputflag",0), pdata->env );
  CGE ( GRBsetintparam(pdata->env,"method",method), pdata->env );
  // CGE ( GRBsetintparam(pdata->env,"method",method), pdata->env );
  CGE ( GRBsetintparam(pdata->env,"presolve",0), pdata->env );
  if (method==2) {
    CGE ( GRBsetintparam(pdata->env,"bariterlimit",20), pdata->env );
    CGE ( GRBsetintparam(pdata->env,"barhomogeneous",0), pdata->env );
    CGE ( GRBsetdblparam(pdata->env,"barconvtol",0.0005), pdata->env );
  }

  // preallocate some memory
  pdata->H.resize(nq,nq);
  pdata->H_float.resize(6,nq);
  pdata->H_act.resize(nu,nq);

  pdata->C.resize(nq);
  pdata->C_float.resize(6);
  pdata->C_act.resize(nu);

  pdata->J.resize(3,nq);
  pdata->Jdot.resize(3,nq);
  pdata->J_xy.resize(2,nq);
  pdata->Jdot_xy.resize(2,nq);
  pdata->Hqp.resize(nq,nq);
  pdata->fqp.resize(nq);
  pdata->Ag.resize(6,nq);
  pdata->Agdot.resize(6,nq);
  pdata->Ak.resize(3,nq);
  pdata->Akdot.resize(3,nq);

  pdata->state.vbasis_len = 0;
  pdata->state.cbasis_len = 0;
  pdata->state.vbasis = NULL;
  pdata->state.cbasis = NULL;

  pdata->state.t_prev = 0;
  pdata->state.vref_integrator_state = VectorXd::Zero(pdata->r->num_velocities);
  pdata->state.q_integrator_state = VectorXd::Zero(pdata->r->num_positions);
  pdata->state.foot_contact_prev[0] = false;
  pdata->state.foot_contact_prev[1] = false;

  plhs[0] = createDrakeMexPointer((void*) pdata, "NewQPControllerData");

  return;
}












