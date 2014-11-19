#include "mex.h"
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"


using namespace Eigen;
using namespace std;

/*
 * A C version of the doKinematics function
 *
 * Piggybacks on HandCmex.cpp to properly initialize and destroy models
 * Call with doKinematicsmex(q,b_compute_second_derivatives);
 */

 
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {
  if (nrhs != 4) {
    mexErrMsgIdAndTxt("Drake:doKinematicsmex:NotEnoughInputs", "Usage doKinematicsmex(model_ptr,q,b_compute_second_derivatives,qd)");
  }
  
  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  
  double *q, *qd=NULL;
  if (mxGetNumberOfElements(prhs[1])!=model->NB)
    mexErrMsgIdAndTxt("Drake:doKinematicsmex:BadInputs", "q must be size %d x 1", model->NB);
  q = mxGetPr(prhs[1]);
  bool b_compute_second_derivatives = (mxGetScalar(prhs[2])!=0.0);
  if (mxGetNumberOfElements(prhs[3])>0) 
    qd = mxGetPr(prhs[3]);
     
  model->doKinematics(q,b_compute_second_derivatives,qd);
}
