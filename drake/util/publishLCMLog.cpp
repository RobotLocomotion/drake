#include <mex.h>

#include <lcm/lcm.h>

static lcm_t* lcm = NULL;

void cleanup(void) { lcm_destroy(lcm); }

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nrhs < 1) {
    mexPrintf("Usage: publishLCMLog(lcm_log)\n");
    return;
  }

  if (!lcm) lcm = lcm_create(NULL);
  if (!lcm) mexErrMsgTxt("failed to create lcm node");

  char* channel;
  mxArray* data;

  int channel_field_number = mxGetFieldNumber(prhs[0], "channel"),
      data_field_number = mxGetFieldNumber(prhs[0], "data");

  if (channel_field_number < 0 || data_field_number < 0)
    mexErrMsgTxt(
        "publishLCMLog failed: input must be a structure with fields 'channel' "
        "and 'data'");

  for (size_t i = 0; i < mxGetNumberOfElements(prhs[0]); i++) {
    channel =
        mxArrayToString(mxGetFieldByNumber(prhs[0], i, channel_field_number));
    data = mxGetFieldByNumber(prhs[0], i, data_field_number);
    lcm_publish(lcm, channel, mxGetData(data), mxGetNumberOfElements(data));
    mxFree(channel);
  }
}
