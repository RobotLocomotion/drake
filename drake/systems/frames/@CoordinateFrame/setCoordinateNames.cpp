#include "drakeMexUtil.h"
#include "CoordinateFrame.h"

using namespace std;

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {
  if (nrhs!=2) mexErrMsgIdAndTxt("Drake:CoordinateFrame:BadInputs","Usage: setCoordinateNames(obj,cnames)");
  CoordinateFrame* frame = static_cast<CoordinateFrame*>(getDrakeMexPointer(mxGetProperty(prhs[0],0,"mex_ptr")));
  frame->setCoordinateNames(mxGetVectorOfStdStrings(prhs[1]));
}
