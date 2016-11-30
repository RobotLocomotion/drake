#include <mex.h>

#include "drake/util/drakeUtil.h"
#include "drake/matlab/util/drakeMexUtil.h"

using namespace std;
using namespace Eigen;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  std::string usage =
      "Usage [cop, normal_torque_at_cop] = resolveCenterOfPressure(torque, "
      "force, normal, point_on_contact_plane)";
  if (nrhs != 4) {
    mexErrMsgIdAndTxt("Drake:resolveCenterOfPressuremex:WrongNumberOfInputs",
                      usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:resolveCenterOfPressuremex:WrongNumberOfOutputs",
                      usage.c_str());
  }

  auto torque = matlabToEigenMap<3, 1>(prhs[0]);
  auto force = matlabToEigenMap<3, 1>(prhs[1]);
  auto normal = matlabToEigenMap<3, 1>(prhs[2]);
  auto point_on_contact_plane = matlabToEigenMap<3, 1>(prhs[3]);
  std::pair<Eigen::Vector3d, double> ret =
      resolveCenterOfPressure(torque, force, normal, point_on_contact_plane);
  if (nlhs > 0) plhs[0] = eigenToMatlab(ret.first);
  if (nlhs > 1) plhs[1] = mxCreateDoubleScalar(ret.second);
}
