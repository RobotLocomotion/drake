#include <mex.h>
#include "drakeUtil.h"
#include "IKoptions.h"
#include "RigidBodyManipulator.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if(nrhs != 1 || nlhs != 1)
  {
    mexErrMsgIdAndTxt("Drake:IKoptions:BadInputs","Usage ptr = constructPtrIKoptionsmex(robot)");
  }
  RigidBodyManipulator* robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  IKoptions* ikoptions = new IKoptions(robot);
  plhs[0] = createDrakeMexPointer((void*) ikoptions,"deletePtrIKoptionsmex","IKoptions");
}

