#include <lcm/lcm-cpp.hpp>
#include "drake/lcmt_qp_controller_input.hpp"
#include "QPCommon.h"

using namespace std;

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
	if (nrhs < 1) {
		mexErrMsgTxt("usage: msg_ptr=encodeQPInputLCMMex(qp_input)");
	}
	// if (nlhs < 1) {
	// 	mexErrMsgTxt("please take one output");
	// }

	lcm::LCM lcm;
	if(!lcm.good()) {
		mexErrMsgTxt("bad lcm");
	}

	int narg = 0;
	const mxArray* qp_input = prhs[narg];
	narg++;

  shared_ptr<drake::lcmt_qp_controller_input> msg = encodeQPInputLCM(qp_input);

	lcm.publish("QP_CONTROLLER_INPUT", &*msg);

	// drake::lcmt_qp_controller_input* msg_ptr = &msg;

 //    mxClassID cid;
 //    if (sizeof(msg_ptr)==4) cid = mxUINT32_CLASS;
 //    else if (sizeof(msg_ptr)==8) cid = mxUINT64_CLASS;
 //    else mexErrMsgIdAndTxt("Drake:supportDetectmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
     
 //    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
 //    memcpy(mxGetData(plhs[0]),&msg_ptr,sizeof(msg_ptr));

 //    return;
}