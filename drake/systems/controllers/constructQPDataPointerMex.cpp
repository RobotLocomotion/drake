#include "drake/util/drakeMexUtil.h"
#include "drake/examples/Atlas/atlasUtil.h"
#include "InstantaneousQPController.h"

using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs == 1) {
    // By convention, calling the constructor with just one argument (the pointer) should delete the pointer
    if (isa(prhs[0],"DrakeMexPointer")) { 
      destroyDrakeMexPointer<InstantaneousQPController*>(prhs[0]);
      return;
    } else {
      mexErrMsgIdAndTxt("Drake:constructQPDataPointerMex:BadInputs", "Expected a DrakeMexPointer (or a subclass)");
    }
  }

  if (nrhs != 3) mexErrMsgTxt("usage: ptr = constructQPDataPointerMex(urdf_filename, control_config_filename, urdf_modifications_filename);");

  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");

  int narg = 0;

  auto urdf_filename = mxGetStdString(prhs[narg++]);
  std::string control_config_filename = mxGetStdString(prhs[narg++]);
  std::string urdf_modifications_filename = mxGetStdString(prhs[narg++]);

  std::unique_ptr<RigidBodyTree> atlas = Atlas::constructAtlas(urdf_filename, urdf_modifications_filename);
  InstantaneousQPController* controller = new InstantaneousQPController(std::move(atlas), control_config_filename);
  plhs[0] = createDrakeMexPointer((void*) controller, "InstantaneousQPController");

  return;
}












