/*
 * A c++ version of (significant pieces of) the QPController.m mimoOutput
 *method.
 *
 * Todo:
 *   switch to spatial accelerations in motion constraints
 *   use fixed-size matrices (or at least pre-allocated)
 *       for instance: #define nq
 *       set MaxRowsAtCompileTime
 *(http://eigen.tuxfamily.org/dox/TutorialMatrixClass.html)
 *   some matrices might be better off using RowMajor
 */

#include "drake/systems/controllers/InstantaneousQPController.h"

#include <cmath>
#include <limits>

#include "drake/common/eigen_types.h"
#include "drake/matlab/util/drakeMexUtil.h"

using namespace std;
using namespace Eigen;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nrhs < 5)
    mexErrMsgTxt(
        "usage: "
        "alpha=QPControllermex(ptr, t, x, qp_input, contact_sensor, foot_force_"
        "torque_measurements)");
  if (nlhs < 1) mexErrMsgTxt("take at least one output... please.");

  // first get the ptr back from matlab
  InstantaneousQPController* controller =
      (InstantaneousQPController*)getDrakeMexPointer(prhs[0]);

  // now retrieve the runtime params from their matlab object
  int narg = 1;

  // t
  double t = mxGetScalar(prhs[narg++]);

  // x
  int nq = controller->getRobot().get_num_positions();
  int nv = controller->getRobot().get_num_velocities();
  if (static_cast<int>(mxGetNumberOfElements(prhs[narg])) != (nq + nv))
    mexErrMsgTxt("size of x should be nq + nv\n");
  if (nq != nv) mexErrMsgTxt("still assume nv==nq");
  double* q_ptr = mxGetPrSafe(prhs[narg]);
  double* qd_ptr = &q_ptr[nq];
  Map<VectorXd> q(q_ptr, nq);
  Map<VectorXd> qd(qd_ptr, nq);
  narg++;

  // qp_input
  drake::lcmt_qp_controller_input qp_input;
  const mxArray* lcm_message_mex = prhs[narg];
  if (!mxIsInt8(lcm_message_mex))
    mexErrMsgTxt("Expected an int8 array as the qp_input argument");
  qp_input.decode(mxGetData(lcm_message_mex), 0,
                  mxGetNumberOfElements(prhs[narg]));
  narg++;

  // contact_sensor
  const mxArray* pobj = prhs[narg];
  Matrix<bool, Dynamic, 1> b_contact_force =
      Matrix<bool, Dynamic, 1>::Zero(controller->getRobot().bodies.size())
          .array();
  int num_bodies_in_contact = mxGetNumberOfElements(pobj);
  for (int i = 0; i < num_bodies_in_contact; i++) {
    b_contact_force(controller->body_or_frame_name_to_id.at(
        mxGetStdString(mxGetCell(pobj, i)))) = 1;
  }
  narg++;

  QPControllerOutput qp_output;
  shared_ptr<QPControllerDebugData> debug;

  DrakeRobotState robot_state;
  robot_state.t = t;
  robot_state.q = q;
  robot_state.qd = qd;

  if (nlhs > 3) {
    debug.reset(new QPControllerDebugData());
  }

  drake::eigen_aligned_std_map<Side, ForceTorqueMeasurement>
      foot_force_torque_measurements;
  if (nrhs > 5) {
    const mxArray* mex_foot_force_torque_measurements = prhs[narg++];
    if (!mxIsEmpty(mex_foot_force_torque_measurements)) {
      foot_force_torque_measurements[Side::LEFT].frame_idx =
          controller->getRobot().FindBodyIndex("l_foot");
      foot_force_torque_measurements[Side::LEFT].wrench =
          matlabToEigenMap<drake::kTwistSize, 1>(
              mxGetFieldSafe(mex_foot_force_torque_measurements, "left"));
      foot_force_torque_measurements[Side::RIGHT].frame_idx =
          controller->getRobot().FindBodyIndex("r_foot");
      foot_force_torque_measurements[Side::RIGHT].wrench =
          matlabToEigenMap<drake::kTwistSize, 1>(
              mxGetFieldSafe(mex_foot_force_torque_measurements, "right"));
    }
  }
  int info = controller->setupAndSolveQP(qp_input, robot_state, b_contact_force,
                                         foot_force_torque_measurements,
                                         qp_output, debug.get());

  // return to matlab
  narg = 0;
  if (nlhs > narg) {
    plhs[narg] = eigenToMatlab(qp_output.u);
  }
  narg++;

  if (nlhs > narg) {
    plhs[narg] = eigenToMatlab(qp_output.qdd);
  }
  narg++;

  if (nlhs > narg) {
    plhs[narg] = eigenToMatlab(qp_output.qd_ref);
  }
  narg++;

  if (nlhs > narg) {
    plhs[narg] = mxCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
    memcpy(mxGetData(plhs[narg]), &(info), sizeof(int));
  }
  narg++;
}
