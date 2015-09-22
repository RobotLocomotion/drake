#include <lcm/lcm-cpp.hpp>
#include "QPCommon.h"
#include "drakeMexUtil.h"
#include "controlMexUtil.h"

using namespace std;
using namespace Eigen;

template<int M, int N>
void checkAndCopy(const mxArray *source, const int idx, const char *fieldname, double *destination)  {
  const mxArray *field = myGetField(source, idx, fieldname);
  sizecheck(field, M, N);
  Map<Matrix<double, M, N>>A(mxGetPrSafe(field));
  // C is row-major, matlab is column-major
  Matrix<double, N, M> A_t = A.transpose();
  memcpy(destination, A_t.data(), sizeof(double)*M*N);
  return;
}


void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if (nrhs < 1 || nrhs > 2) {
    mexErrMsgTxt("usage: msg_str=encodeQPInputLCMMex(qp_input) OR msg_str=encodeQPInputLCMMex(qp_input, publish_flag)");
  }

  lcm::LCM lcm;
  if(!lcm.good()) {
    mexErrMsgTxt("bad lcm");
  }

  int narg = 0;
  const mxArray* qp_input = prhs[narg];
  narg++;

  bool publish_flag;
  if (narg < nrhs) {
    publish_flag = static_cast<bool> (mxGetScalar(prhs[narg]));
  } else {
    publish_flag = true;
  }
  narg++;

  shared_ptr<drake::lcmt_qp_controller_input> msg = encodeQPInputLCM(qp_input);

  if (publish_flag) {
    lcm.publish("QP_CONTROLLER_INPUT", &*msg);
  }

  if (nlhs > 0) {
    int msg_size = msg->getEncodedSize();
    plhs[0] = mxCreateNumericMatrix(msg_size, 1, mxUINT8_CLASS, mxREAL);
    void *msg_ptr = mxGetData(plhs[0]);
    int num_written = msg->encode(msg_ptr, 0, msg_size);
    if (num_written != msg_size) {
      mexPrintf("Number written: %d, message size: %d\n", num_written, msg_size);
      mexErrMsgTxt("Number of bytes written does not match message size.\n");
    }
  }
}
