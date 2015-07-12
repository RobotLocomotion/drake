#include "controlUtil.h"
#include "drakeMexUtil.h"
#include "drakeGeometryUtil.h"
#include "mex.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  /*
   * The input should be consistent with controlUtil.h:bodySpatialMotionPD
   * robot   RigidBodyManipulator
   * t    time
   * q    configuration
   * qd   configuration derivative
   * body_index   the index of the body
   * body_xyzquat_desire    desired xyz position and quaternion of the body, specified in task frame
   * body_v_desire       desired xyz_dot and exponential map dot of the body, specified in task frame
   * body_vdot_desire    desired xyz_ddot and exponential map ddot of the body, specified in task frame
   * Kp                  The P gain on error, specified in task frame
   * Kd                  The D gain on error, specified in task frame
   * xyzquat_task_to_world    The transformation xyz, quaternion from task frame to the world frame
   */
  if (nrhs != 11) {
    mexErrMsgTxt("Drake:bodySpatialMotionPD:IncorrectInputs: body_twist_dot = bodySpatialMotionPD(robot,t,q,qd,body_index,body_xyzquat_desire,body_v_desire,body_vdot_desire,Kp,Kd,xyzquat_task_to_world");
  }
  int rhs_args = 0;
  RigidBodyManipulator* r = (RigidBodyManipulator*) getDrakeMexPointer(prhs[rhs_args++]);
  int nq = r->num_positions;
  if (mxGetNumberOfElements(prhs[rhs_args]) != 1) {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: t should be a scalar");
  }
  double t = mxGetScalar(prhs[rhs_args++]);
  if (mxGetM(prhs[rhs_args]) != nq || mxGetN(prhs[rhs_args]) != 1) {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: q should be nq x 1");
  }
  Map<VectorXd> q(mxGetPrSafe(prhs[rhs_args++]),nq);
  if (mxGetM(prhs[rhs_args]) != nq || mxGetN(prhs[rhs_args]) != 1) {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: qd should be nq x 1");
  }
  Map<VectorXd> qd(mxGetPrSafe(prhs[rhs_args++]),nq);
  DrakeRobotState robot_state;
  robot_state.t = t;
  robot_state.q = q;
  robot_state.qd = qd;
  if (mxGetNumberOfElements(prhs[rhs_args]) != 1) {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: body_indx should be a scalar");
  }
  int body_index = (int)mxGetScalar(prhs[rhs_args++]) - 1;
  if (mxGetM(prhs[rhs_args]) != 7 || mxGetN(prhs[rhs_args]) != 1) {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: body_xyzquat_des should be 7 x 1");
  }
  Map<Matrix<double,7,1>> body_xyzquat_des(mxGetPrSafe(prhs[rhs_args++]));
  Isometry3d body_pose_des;
  body_pose_des.translation() = body_xyzquat_des.head<3>();
  body_pose_des.linear() = quat2rotmat((body_xyzquat_des.tail<4>()).eval());
  if (mxGetM(prhs[rhs_args]) != 6 || mxGetN(prhs[rhs_args]) != 1) {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: body_v_des should be 6 x 1");
  }
  Map<Matrix<double,6,1>> body_v_des(mxGetPrSafe(prhs[rhs_args++]));
  if (mxGetM(prhs[rhs_args]) != 6 || mxGetN(prhs[rhs_args]) != 1) {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: body_vdot_des should be 6 x 1");
  }
  Map<Matrix<double,6,1>> body_vdot_des(mxGetPrSafe(prhs[rhs_args++]));
  if (mxGetM(prhs[rhs_args]) != 6 || mxGetN(prhs[rhs_args]) != 1) {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: Kp should be 6 x 1");
  }
  Map<Matrix<double,6,1>> Kp(mxGetPrSafe(prhs[rhs_args++]));
  if (mxGetM(prhs[rhs_args]) != 6 || mxGetN(prhs[rhs_args]) != 1) {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: Kd should be 6 x 1");
  }
  Map<Matrix<double,6,1>> Kd(mxGetPrSafe(prhs[rhs_args++]));
  if (mxGetM(prhs[rhs_args]) != 7 || mxGetN(prhs[rhs_args]) != 1) {
    mexErrMsgTxt("Drake:bodySpatialMotionPD: xyzquat_task_to_world should be 7 x 1");
  }
  Map<Matrix<double,7,1>> xyzquat_task_to_world(mxGetPrSafe(prhs[rhs_args++]));
  Isometry3d T_task_to_world;
  T_task_to_world.translation() = xyzquat_task_to_world.head<3>();
  T_task_to_world.linear() = quat2rotmat((xyzquat_task_to_world.tail<4>()).eval());
  Vector6d body_vdot = bodySpatialMotionPD(r, robot_state, body_index, body_pose_des, body_v_des, body_vdot_des, Kp,Kd, T_task_to_world);
  plhs[0] = eigenToMatlab(body_vdot);
}
