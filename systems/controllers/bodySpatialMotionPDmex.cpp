#include "controlUtil.h"
#include "drakeUtil.h"
#include "drakeGeometryUtil.h"
#include "mex.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if(nrhs != 11)
  {
    mexErrMsgTxt("Drake:bodySpatialMotionPD:IncorrectInputs");
  }
  int rhs_args = 0;
  RigidBodyManipulator* r = (RigidBodyManipulator*) getDrakeMexPointer(prhs[rhs_args++]);
  int nq = r->num_positions;
  if(mxGetNumberOfElements(prhs[rhs_args]) != 1)
  {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: t should be a scalar");
  }
  double t = mxGetScalar(prhs[rhs_args++]);
  if(mxGetM(prhs[rhs_args]) != nq || mxGetN(prhs[rhs_args]) != 1)
  {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: q should be nq x 1");
  }
  Map<VectorXd> q(mxGetPr(prhs[rhs_args++]),nq);
  if(mxGetM(prhs[rhs_args]) != nq || mxGetN(prhs[rhs_args]) != 1)
  {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: qd should be nq x 1");
  }
  Map<VectorXd> qd(mxGetPr(prhs[rhs_args++]),nq);
  DrakeRobotState robot_state;
  robot_state.t = t;
  robot_state.q = q;
  robot_state.qd = qd;
  if(mxGetNumberOfElements(prhs[rhs_args]) != 1)
  {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: body_indx should be a scalar");
  }
  int body_index = (int)mxGetScalar(prhs[rhs_args++]) - 1;
  if(mxGetM(prhs[rhs_args]) != 7 || mxGetN(prhs[rhs_args]) != 1)
  {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: body_xyzquat_des should be 7 x 1");
  }
  Map<Matrix<double,7,1>> body_xyzquat_des(mxGetPr(prhs[rhs_args++]));
  Isometry3d body_pose_des;
  body_pose_des.translation() = body_xyzquat_des.head<3>();
  body_pose_des.linear() = quat2rotmat((body_xyzquat_des.tail<4>()).eval());
  if(mxGetM(prhs[rhs_args]) != 6 || mxGetN(prhs[rhs_args]) != 1)
  {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: body_v_des should be 6 x 1");
  }
  Map<Matrix<double,6,1>> body_v_des(mxGetPr(prhs[rhs_args++]));
  if(mxGetM(prhs[rhs_args]) != 6 || mxGetN(prhs[rhs_args]) != 1)
  {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: body_vdot_des should be 6 x 1");
  }
  Map<Matrix<double,6,1>> body_vdot_des(mxGetPr(prhs[rhs_args++]));
  if(mxGetM(prhs[rhs_args]) != 6 || mxGetN(prhs[rhs_args]) != 1)
  {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: Kp should be 6 x 1");
  }
  Map<Matrix<double,6,1>> Kp(mxGetPr(prhs[rhs_args++]));
  if(mxGetM(prhs[rhs_args]) != 6 || mxGetN(prhs[rhs_args]) != 1)
  {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: Kd should be 6 x 1");
  }
  Map<Matrix<double,6,1>> Kd(mxGetPr(prhs[rhs_args++]));
  if(mxGetM(prhs[rhs_args]) != 7 || mxGetN(prhs[rhs_args]) != 1)
  {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: xyzquat_task_to_world should be 7 x 1");
  }
  Map<Matrix<double,7,1>> xyzquat_task_to_world(mxGetPr(prhs[rhs_args++]));
  Isometry3d T_task_to_world;
  T_task_to_world.translation() = xyzquat_task_to_world.head<3>();
  T_task_to_world.linear() = quat2rotmat((xyzquat_task_to_world.tail<4>()).eval());
  Vector6d body_vdot = bodySpatialMotionPD(r, robot_state, body_index, body_pose_des, body_v_des, body_vdot_des, Kp,Kd, T_task_to_world);
  plhs[0] = eigenToMatlab(body_vdot);
}
