#include <mex.h>
#include <iostream>
#include "drakeMexUtil.h"
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * A C version of the findKinematicPath function
 *
 * Call with findKinematicPathmex(model_ptr, start_body_or_frame_idx, end_body_or_frame_idx);
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  std::string usage = "Usage [body_path, joint_path, signs] = findKinematicPathmex(model_ptr, start_body_or_frame_idx, end_body_or_frame_idx)";
  if (nrhs != 3) {
    mexErrMsgIdAndTxt("Drake:findKinematicPathmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 3) {
    mexErrMsgIdAndTxt("Drake:findKinematicPathmex:WrongNumberOfOutputs", usage.c_str());
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  int start_body_or_frame_idx = ((int) mxGetScalar(prhs[1])) - 1; // base 1 to base 0
  int end_body_or_frame_idx = ((int) mxGetScalar(prhs[2])) - 1; // base 1 to base 0

  KinematicPath path = model->findKinematicPath(start_body_or_frame_idx, end_body_or_frame_idx);

  if (nlhs > 0) {
    baseZeroToBaseOne(path.body_path);
    plhs[0] = stdVectorToMatlab(path.body_path);
  }
  if (nlhs > 1) {
    baseZeroToBaseOne(path.joint_path);
    plhs[1] = stdVectorToMatlab(path.joint_path);
  }
  if (nlhs > 2) {
    plhs[2] = stdVectorToMatlab(path.joint_direction_signs);
  }
}
