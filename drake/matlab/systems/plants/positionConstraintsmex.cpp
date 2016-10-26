#include <mex.h>

#include "drake/matlab/util/drakeMexUtil.h"
#include "drake/systems/plants/RigidBodyTree.h"

using namespace Eigen;
using namespace std;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs < 2 || nlhs < 2) {
    mexErrMsgIdAndTxt(
        "Drake:positionConstraintsmex:InvalidCall",
        "Usage: [phi, J] = positionConstraintsmex(mex_model_ptr, q) ");
  }

  RigidBodyTree *model = (RigidBodyTree *)getDrakeMexPointer(prhs[0]);

  const size_t nq = model->get_num_positions();

  if (mxGetNumberOfElements(prhs[1]) != nq) {
    mexErrMsgIdAndTxt(
        "Drake:positionConstraintsmex:InvalidPositionVectorLength",
        "q contains the wrong number of elements");
  }

  Map<VectorXd> q(mxGetPrSafe(prhs[1]), nq);

  KinematicsCache<double> cache =
      model->doKinematics(q);  // FIXME: KinematicsCache should be passed in!

  auto phi = model->positionConstraints(cache);
  plhs[0] = eigenToMatlab(phi);

  auto dphi = model->positionConstraintsJacobian(cache);
  plhs[1] = eigenToMatlab(dphi);
}
