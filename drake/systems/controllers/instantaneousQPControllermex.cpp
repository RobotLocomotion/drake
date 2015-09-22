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
#include <limits>
#include <cmath>
#include "drakeMexUtil.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs < 5)
    mexErrMsgTxt("usage: alpha=QPControllermex(ptr,t,x,qp_input,contact_sensor,foot_force_torque_measurements)");
  if (nlhs < 1)
    mexErrMsgTxt("take at least one output... please.");

  double* pr;

  // first get the ptr back from matlab
  NewQPControllerData *pdata = (NewQPControllerData*) getDrakeMexPointer(prhs[0]);

  // now retrieve the runtime params from their matlab object
  int narg=1;

  // t
  double t = mxGetScalar(prhs[narg++]);

  // x
  int nq = pdata->r->num_positions;
  int nv = pdata->r->num_velocities;
  if (mxGetNumberOfElements(prhs[narg]) != (nq + nv)) mexErrMsgTxt("size of x should be nq + nv\n");
  if (nq!=nv) mexErrMsgTxt("still assume nv==nq");
  double *q_ptr = mxGetPrSafe(prhs[narg]);
  double *qd_ptr = &q_ptr[nq];
  Map<VectorXd> q(q_ptr, nq);
  Map<VectorXd> qd(qd_ptr, nq);
  narg++;

  // qp_input
  shared_ptr<drake::lcmt_qp_controller_input> qp_input(new drake::lcmt_qp_controller_input());
  const mxArray* lcm_message_mex = prhs[narg];
  if (!mxIsInt8(lcm_message_mex))
    mexErrMsgTxt("Expected an int8 array as the qp_input argument");
  qp_input->decode(mxGetData(lcm_message_mex), 0, mxGetNumberOfElements(prhs[narg]));
  narg++;

  // contact_sensor
  const mxArray *pobj = prhs[narg];
  Map<VectorXd> contact_force_detected(mxGetPrSafe(pobj), mxGetNumberOfElements(pobj), 1);
  Matrix<bool, Dynamic, 1> b_contact_force = Matrix<bool, Dynamic, 1>::Zero(contact_force_detected.size());
  for (int i=0; i < b_contact_force.size(); i++) {
    b_contact_force(i) = (contact_force_detected(i) != 0);
  }
  narg++;

  QPControllerOutput qp_output;
  shared_ptr<QPControllerDebugData> debug;

  DrakeRobotState robot_state;
  robot_state.t = t;
  robot_state.q = q;
  robot_state.qd = qd;

  if (nlhs>3) {
    debug.reset(new QPControllerDebugData());
  }

  std::map<Side, ForceTorqueMeasurement> foot_force_torque_measurements;
  if (nrhs > 5) {
    const mxArray* mex_foot_force_torque_measurements = prhs[narg++];
    if (!mxIsEmpty(mex_foot_force_torque_measurements)) {
      foot_force_torque_measurements[Side::LEFT].frame_idx = pdata->r->findLinkId("l_foot");
      foot_force_torque_measurements[Side::LEFT].wrench = matlabToEigenMap<TWIST_SIZE, 1>(mxGetFieldSafe(mex_foot_force_torque_measurements, "left"));
      foot_force_torque_measurements[Side::RIGHT].frame_idx = pdata->r->findLinkId("r_foot");
      foot_force_torque_measurements[Side::RIGHT].wrench = matlabToEigenMap<TWIST_SIZE, 1>(mxGetFieldSafe(mex_foot_force_torque_measurements, "right"));
    }
  }
  int info = setupAndSolveQP(pdata, qp_input, robot_state, b_contact_force, foot_force_torque_measurements, &qp_output, debug);

  // return to matlab
  narg = 0;
  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(qp_output.u);
  }
  narg++;
  
  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(qp_output.qdd);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(qp_output.qd_ref);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = mxCreateNumericMatrix(1,1,mxINT32_CLASS,mxREAL);
    memcpy(mxGetData(plhs[narg]),&(info),sizeof(int));
  }
  narg++;

  if (nlhs>narg) {
      plhs[narg] = mxCreateDoubleMatrix(1,debug->active_supports.size(),mxREAL);
      pr = mxGetPrSafe(plhs[narg]);
      int i=0;
      for (vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>>::iterator iter = debug->active_supports.begin(); iter!=debug->active_supports.end(); iter++) {
          pr[i++] = (double) (iter->body_idx + 1);
      }
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(debug->alpha);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(pdata->Hqp);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(debug->f);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(debug->Aeq);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(debug->beq);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(debug->Ain_lb_ub);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(debug->bin_lb_ub);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(debug->Qnfdiag);
  }
  narg++;

  if (nlhs>narg) {
    plhs[narg] = eigenToMatlab(debug->Qneps);
  }
  narg++;

  if (nlhs>narg) {
    double Vdot;
    if (debug->nc>0) 
      // note: Sdot is 0 for ZMP/double integrator dynamics, so we omit that term here
      Vdot = ((2*debug->x_bar.transpose()*debug->S + debug->s1.transpose())*(debug->A_ls*debug->x_bar + debug->B_ls*(debug->Jcomdotv + debug->Jcom*qp_output.qdd)) + debug->s1dot.transpose()*debug->x_bar)(0) + debug->s2dot;
    else
      Vdot = 0;
    plhs[narg] = mxCreateDoubleScalar(Vdot);
  }
  narg++;

  if (nlhs>narg) {
    RigidBodyManipulator* r = pdata->r;

    VectorXd individual_cops = individualSupportCOPs(r, pdata->cache, debug->active_supports, debug->normals, debug->B, debug->beta);
    plhs[narg] = eigenToMatlab(individual_cops);
  }
  narg++;
} 
