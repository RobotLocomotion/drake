#include <lcm/lcm-cpp.hpp>
#include "drake/lcmt_qp_controller_input.hpp"
#include "controlUtil.h"

using namespace std;

template<int M, int N>
void checkAndCopy(const mxArray *source, const int idx, const char *fieldname, double *destination)  {
	const mxArray *field = myGetField(source, idx, fieldname);
  sizecheck(field, M, N);
	Map<Matrix<double, M, N>>A(mxGetPr(field));
  memcpy(destination, A.data(), sizeof(double)*M*N);
	return;
}


void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
	if (nrhs < 2) {
		mexErrMsgTxt("usage: msg_ptr=encodeQPInputLCMMex(qp_input, t)");
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

	double t = mxGetScalar(prhs[narg]);
	narg++;

	drake::lcmt_qp_controller_input msg;

	msg.timestamp = (int64_t) t * 1000000;

  mexPrintf("timestamp ");


	const mxArray* zmp_data = myGetProperty(qp_input, "zmp_data");

  checkAndCopy<4, 4>(zmp_data, 0, "A", &msg.zmp_data.A[0][0]);
  checkAndCopy<4, 2>(zmp_data, 0, "B", &msg.zmp_data.B[0][0]);
  checkAndCopy<2, 4>(zmp_data, 0, "C", &msg.zmp_data.C[0][0]);
  checkAndCopy<2, 2>(zmp_data, 0, "D", &msg.zmp_data.D[0][0]);
  checkAndCopy<4, 1>(zmp_data, 0, "x0", &msg.zmp_data.x0[0][0]);
  checkAndCopy<2, 1>(zmp_data, 0, "y0", &msg.zmp_data.y0[0][0]);
  checkAndCopy<2, 1>(zmp_data, 0, "u0", &msg.zmp_data.u0[0][0]);
  checkAndCopy<2, 2>(zmp_data, 0, "R", &msg.zmp_data.R[0][0]);
  checkAndCopy<2, 2>(zmp_data, 0, "Qy", &msg.zmp_data.Qy[0][0]);
  checkAndCopy<4, 4>(zmp_data, 0, "S", &msg.zmp_data.S[0][0]);
  checkAndCopy<4, 1>(zmp_data, 0, "s1", &msg.zmp_data.s1[0][0]);
  checkAndCopy<4, 1>(zmp_data, 0, "s1dot", &msg.zmp_data.s1dot[0][0]);
  msg.zmp_data.s2 = mxGetScalar(myGetField(zmp_data, "s2"));
  msg.zmp_data.s2dot = mxGetScalar(myGetField(zmp_data, "s2dot"));

  mexPrintf("copy ");

  const mxArray* support_data = myGetProperty(qp_input, "support_data");
  if (mxGetM(support_data) != 1) {
    mexErrMsgTxt("support data should be a struct array with M=1");
  }
  int nsupp = mxGetN(support_data);

  msg.num_support_data = (int32_t) nsupp;
  drake::lcmt_support_data supp_msg;
  double double_logic_map[4][1];
  for (int i=0; i < nsupp; i++) {
    supp_msg.body_id = (int32_t) mxGetScalar(mxGetField(support_data, i, "body_id"));

    // const mxArray *contact_pts = mxGetField(support_data, i, "contact_pts");
    // if (!contact_pts) mexErrMsgTxt("couldn't get points");
    // supp_msg.num_contact_pts = (int32_t) mxGetN(contact_pts);
    // supp_msg.contact_pts.resize(3);
    // for (int j=0; j < 3; j++) {
    //   supp_msg.contact_pts[j].resize(supp_msg.num_contact_pts);
    //   for (int k=0; k < supp_msg.num_contact_pts; k++) {
    //     supp_msg.contact_pts[j][k] = contact_pts_mat(j, k);
    //   }
    // }

    checkAndCopy<4, 1>(support_data, i, "support_logic_map", &double_logic_map[0][0]);
    for (int j=0; j < 4; j++) {
      supp_msg.support_logic_map[j][0] = (int8_t) (double_logic_map[j][0] != 0);
    }
    supp_msg.mu = mxGetScalar(mxGetField(support_data, i, "mu"));
    supp_msg.contact_surfaces = (int32_t) mxGetScalar(mxGetField(support_data, i, "contact_surfaces"));
  }

  msg.num_tracked_bodies = 0;
  msg.whole_body_data.num_positions = 0;
  msg.param_set_name = "walking";
  mexPrintf("debug_fields ");

	// field = myGetField(zmp_data, "A");
	// if (mxGetNumberOfElements(field) != 16) {
	// 	mexErrMsgTxt("Expected A to be 4x4. We'll have to reimplment this for the 3D ZMP case");
	// }
	// Map<Matrix4d>A(mxGetPr(field));
	// memcpy(msg.zmp_data.A, A.data(), sizeof(double)*mxGetNumberOfElements(field));



	lcm.publish("QPINPUT_DEBUG", &msg);
  mexPrintf("publish ");

	// drake::lcmt_qp_controller_input* msg_ptr = &msg;

 //    mxClassID cid;
 //    if (sizeof(msg_ptr)==4) cid = mxUINT32_CLASS;
 //    else if (sizeof(msg_ptr)==8) cid = mxUINT64_CLASS;
 //    else mexErrMsgIdAndTxt("Drake:supportDetectmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
     
 //    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
 //    memcpy(mxGetData(plhs[0]),&msg_ptr,sizeof(msg_ptr));

 //    return;
}