#include "drakeMexUtil.h"
#include "CoordinateFrame.h"

using namespace std;

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {
  if (nrhs!=2) mexErrMsgIdAndTxt("Drake:CoordinateFrame:BadInputs","Usage: getCoordinateName(obj,i)");
  CoordinateFrame* frame = static_cast<CoordinateFrame*>(getDrakeMexPointer(mxGetProperty(prhs[0],0,"mex_ptr")));
  unsigned int index = static_cast<unsigned int>(mxGetScalar(prhs[1])-1);
  plhs[0] = mxCreateString(frame->getCoordinateName(index).c_str());
}
