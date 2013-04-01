#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "RigidBodyManipulator.h"

#define INF -2147483648

using namespace Eigen;
using namespace std;

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {

  if (nrhs<1) {
    mexErrMsgIdAndTxt("Drake:deleteModelmex:NotEnoughInputs","Usage deleteModelMex(model_ptr)");
  }

  RigidBodyManipulator *model=NULL;

  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:deleteModelmex:BadInputs","the first argument should be the model_ptr");
  memcpy(&model,mxGetData(prhs[0]),sizeof(model));
    
  delete model;
}
