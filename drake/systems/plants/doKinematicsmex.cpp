#include "mex.h"
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"


using namespace Eigen;
using namespace std;

/*
 * A C version of the doKinematics function
 *
 * Call with doKinematicsmex(q,b_compute_second_derivatives);
 */

 
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {
  if (nrhs != 4) {
    mexErrMsgIdAndTxt("Drake:doKinematicsmex:NotEnoughInputs", "Usage doKinematicsmex(model_ptr,q,b_compute_second_derivatives,qd)");
  }
  
  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  int nq = model->num_positions;
  int nv = model->num_velocities;
  
  if (mxGetNumberOfElements(prhs[1])!=nq)
    mexErrMsgIdAndTxt("Drake:doKinematicsmex:BadInputs", "q must be size %d x 1", nq);
  
  Map<VectorXd> q(mxGetPrSafe(prhs[1]), nq);
  bool b_compute_second_derivatives = (mxGetScalar(prhs[2])!=0.0);
  if (mxGetNumberOfElements(prhs[3])>0) 
  {
    Map<VectorXd> qd(mxGetPrSafe(prhs[3]), nv);
    model->doKinematics(q,b_compute_second_derivatives,qd);
  } else {
    model->doKinematics(q,b_compute_second_derivatives);
  }  
}
