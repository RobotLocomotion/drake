#include <mex.h>

#include "drake/matlab/util/drakeMexUtil.h"
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyTree.h"

using namespace std;
using namespace Eigen;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nlhs != 19 || nrhs != 1) {
    mexErrMsgIdAndTxt(
        "Drake:testIKoptions:BadInputs",
        "Usage [robot_address, Q, Qa, Qv, debug_mode, "
        "sequentialSeedFlag, majorFeasibilityTolerance,"
        "majorIterationsLimit, iterationsLimit, superbasicsLimit,"
        "majorOptimalityTolerance, additional_tSamples,"
        "fixInitialState, q0_lb, q0_ub, qd0_lb, qd0_ub, qdf_lb, qdf_ub]"
        " = testIKoptionsmex(ikoptions_ptr)");
  }
  IKoptions* ikoptions = (IKoptions*)getDrakeMexPointer(prhs[0]);
  // NOLINTNEXTLINE(runtime/int)
  auto robot_address = reinterpret_cast<long long>(ikoptions->getRobotPtr());
  int nq = ikoptions->getRobotPtr()->get_num_positions();
  MatrixXd Q;
  ikoptions->getQ(Q);
  MatrixXd Qv;
  ikoptions->getQv(Qv);
  MatrixXd Qa;
  ikoptions->getQa(Qa);
  bool debug_mode = ikoptions->getDebug();
  bool sequentialSeedFlag = ikoptions->getSequentialSeedFlag();
  double majorFeasibilityTolerance = ikoptions->getMajorFeasibilityTolerance();
  int majorIterationsLimit = ikoptions->getMajorIterationsLimit();
  int iterationsLimit = ikoptions->getIterationsLimit();
  int superbasicsLimit = ikoptions->getSuperbasicsLimit();
  double majorOptimalityTolerance = ikoptions->getMajorOptimalityTolerance();
  RowVectorXd t_samples;
  ikoptions->getAdditionaltSamples(t_samples);
  bool fixInitialState = ikoptions->getFixInitialState();
  VectorXd q0_lb, q0_ub;
  VectorXd qd0_lb, qd0_ub;
  VectorXd qdf_lb, qdf_ub;
  ikoptions->getq0(q0_lb, q0_ub);
  ikoptions->getqd0(qd0_lb, qd0_ub);
  ikoptions->getqdf(qdf_lb, qdf_ub);
  plhs[0] = mxCreateDoubleScalar((double)robot_address);
  plhs[1] = mxCreateDoubleMatrix(nq, nq, mxREAL);
  memcpy(mxGetPrSafe(plhs[1]), Q.data(), sizeof(double) * nq * nq);
  plhs[2] = mxCreateDoubleMatrix(nq, nq, mxREAL);
  memcpy(mxGetPrSafe(plhs[2]), Qa.data(), sizeof(double) * nq * nq);
  plhs[3] = mxCreateDoubleMatrix(nq, nq, mxREAL);
  memcpy(mxGetPrSafe(plhs[3]), Qv.data(), sizeof(double) * nq * nq);
  plhs[4] = mxCreateLogicalScalar(debug_mode);
  plhs[5] = mxCreateLogicalScalar(sequentialSeedFlag);
  plhs[6] = mxCreateDoubleScalar(majorFeasibilityTolerance);
  plhs[7] = mxCreateDoubleScalar((double)majorIterationsLimit);
  plhs[8] = mxCreateDoubleScalar((double)iterationsLimit);
  plhs[9] = mxCreateDoubleScalar((double)superbasicsLimit);
  plhs[10] = mxCreateDoubleScalar(majorOptimalityTolerance);
  if (t_samples.size() > 0) {
    plhs[11] =
        mxCreateDoubleMatrix(1, static_cast<int>(t_samples.size()), mxREAL);
    memcpy(mxGetPrSafe(plhs[11]), t_samples.data(),
           sizeof(double) * t_samples.size());
  } else {
    plhs[11] = mxCreateDoubleMatrix(0, 0, mxREAL);
  }
  plhs[12] = mxCreateLogicalScalar(fixInitialState);
  plhs[13] = mxCreateDoubleMatrix(nq, 1, mxREAL);
  memcpy(mxGetPrSafe(plhs[13]), q0_lb.data(), sizeof(double) * nq);
  plhs[14] = mxCreateDoubleMatrix(nq, 1, mxREAL);
  memcpy(mxGetPrSafe(plhs[14]), q0_ub.data(), sizeof(double) * nq);
  plhs[15] = mxCreateDoubleMatrix(nq, 1, mxREAL);
  memcpy(mxGetPrSafe(plhs[15]), qd0_lb.data(), sizeof(double) * nq);
  plhs[16] = mxCreateDoubleMatrix(nq, 1, mxREAL);
  memcpy(mxGetPrSafe(plhs[16]), qd0_ub.data(), sizeof(double) * nq);
  plhs[17] = mxCreateDoubleMatrix(nq, 1, mxREAL);
  memcpy(mxGetPrSafe(plhs[17]), qdf_lb.data(), sizeof(double) * nq);
  plhs[18] = mxCreateDoubleMatrix(nq, 1, mxREAL);
  memcpy(mxGetPrSafe(plhs[18]), qdf_ub.data(), sizeof(double) * nq);
}
