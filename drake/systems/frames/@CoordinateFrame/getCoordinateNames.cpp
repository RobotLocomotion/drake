#include "drakeMexUtil.h"
#include "CoordinateFrame.h"

using namespace std;

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {
  if (nrhs!=1) mexErrMsgIdAndTxt("Drake:CoordinateFrame:BadInputs","Usage: getCoordinateNames(obj)");
  CoordinateFrame* frame = static_cast<CoordinateFrame*>(getDrakeMexPointer(mxGetProperty(prhs[0],0,"mex_ptr")));
  plhs[0] = vectorOfStdStringsToMatlab(frame->getCoordinateNames());
}
