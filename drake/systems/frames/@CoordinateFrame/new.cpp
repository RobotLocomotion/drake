#include "drakeMexUtil.h"
#include "CoordinateFrame.h"

using namespace std;

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {
  if (nrhs==1) {
    destroyDrakeMexPointer<CoordinateFrame*>(prhs[0]);
    return;
  }

  if (nrhs<4) mexErrMsgIdAndTxt("Drake:CoordinateFrame:BadInputs","Usage: new(name,dim,prefix,coordinates)");

  string name = mxGetStdString(prhs[0]);
  unsigned int dim = static_cast<unsigned int>(mxGetScalar(prhs[1]));
  string prefix = mxGetStdString(prhs[2]);
  vector<string> coordinates = mxGetVectorOfStdStrings(prhs[3]);

  CoordinateFrame* frame = new CoordinateFrame(name,dim,prefix,coordinates);

  populateDrakeMexPointerArguments(nlhs, plhs, frame, name,0,NULL,"CoordinateFrame.");
}
