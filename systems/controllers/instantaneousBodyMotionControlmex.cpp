/* 
 * A simple PD control block for regulating a body pose given a desired
 * position, velocity, and acceleration. This is similar to
 * bodyMotionControlmex.cpp, but it maintains no internal state and can
 * therefore be called for multiple different robot bodies.
 */
#include "controlUtil.h"
#include "drakeUtil.h"


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs<1) mexErrMsgTxt("usage: y=instantaneousBodyMotionControlmex(mex_ptr,x,body_ind,body_pose_des,body_v_des,body_vdot_des,params)");
  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");
  
  // first get the ptr back from matlab
  if (mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("DRC:bodyMotionControlmex:BadInputs","the first argument should be the robot");
  RigidBodyManipulator *r = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  mxArray* pm;
  int body_index;

  int narg = 1;
  int nq = r->num_positions;
  int nv = r->num_velocities;
  double *q_ptr = mxGetPr(prhs[narg]);
  double *qd_ptr = &q_ptr[nq];
  Map<VectorXd> q(q_ptr, nq);
  Map<VectorXd> qd(qd_ptr, nv);
  narg++;

  body_index = (int) mxGetScalar(prhs[narg++]) - 1;

  assert(mxGetM(prhs[narg])==6); assert(mxGetN(prhs[narg])==1);
  Map< Vector6d > body_pose_des(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==6); assert(mxGetN(prhs[narg])==1);
  Map< Vector6d > body_v_des(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==6); assert(mxGetN(prhs[narg])==1);
  Map< Vector6d > body_vdot_des(mxGetPr(prhs[narg++]));

  pm = mxGetField(prhs[narg], 0, "Kp");
  assert(mxGetM(pm)==6); assert(mxGetN(pm) == 1);
  Map< Vector6d > Kp(mxGetPr(pm));
  // assert(mxGetM(mxGetField(prhs[narg]),0,"Kp"))==6); assert(mxGetN(prgs[narg])==1);
  // Map< Vector6d > Kp(mxGetPr(prhs[narg++]));

  pm = mxGetField(prhs[narg], 0, "Kd");
  assert(mxGetM(pm)==6); assert(mxGetN(pm) == 1);
  Map< Vector6d > Kd(mxGetPr(pm));
  // assert(mxGetM(prhs[narg])==6); assert(mxGetN(prgs[narg])==1);
  // Map< Vector6d > Kd(mxGetPr(prhs[narg++]));

  Vector6d body_vdot = bodyMotionPD(r, q, qd, body_index, body_pose_des, body_v_des, body_vdot_des, Kp, Kd);
  
  plhs[0] = eigenToMatlab(body_vdot);
}


