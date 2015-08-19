#include "drakeMexUtil.h"
#include "CoordinateFrame.h"

using namespace std;

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {
  if (nrhs!=1) mexErrMsgIdAndTxt("Drake:CoordinateFrame:BadInputs","Usage: dim(obj)");
  CoordinateFrame* frame = static_cast<CoordinateFrame*>(getDrakeMexPointer(prhs[0]));
  plhs[0] = mxCreateDoubleScalar(static_cast<double>(frame->getDim()));
}
