#include "drakeMexUtil.h"
#include "CoordinateFrame.h"

using namespace std;

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {
  if (nrhs==1) {
    destroyDrakeMexPointer<CoordinateFrame*>(prhs[0]);  // note: prhs[0] is the actual DrakeMexPointer object (not the CoordinateFrame object, as it would be in the non-static class methods)
    return;
  }

  if (nrhs<2) mexErrMsgIdAndTxt("Drake:CoordinateFrame:BadInputs","Usage: new(name,coordinates)");

  string name = mxGetStdString(prhs[0]);
  vector<string> coordinates = mxGetVectorOfStdStrings(prhs[1]);

  CoordinateFrame* frame = new CoordinateFrame(name,coordinates);

  plhs[0] = createDrakeMexPointer(frame,name,0,NULL,"","CoordinateFrame.");
}
