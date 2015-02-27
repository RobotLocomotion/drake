#include "QPCommon.h"
#include <Eigen/StdVector>
#include "drakeUtil.h"
#include "controlUtil.h"

using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (nrhs<1) mexErrMsgTxt("usage: mask=getActiveSupportsmex(ptr,x,support_data,contact_sensor,contact_threshold,terrain_height))");
	if (nlhs<1) mexErrMsgTxt("please take at least one output");

	struct QPControllerData* pdata;
	const mxArray* pm;

	// first get the ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:QPControllermex:BadInputs","the first argument should be the ptr");
  memcpy(&pdata,mxGetData(prhs[0]),sizeof(pdata));

  int nq = pdata->r->num_dof;

  int narg=1;  
  double *q = mxGetPr(prhs[narg]);
  double *qd = &q[nq];
  narg++;

  const mxArray* supp_data = prhs[narg];
  narg++;

  pm = prhs[narg];
  Map<VectorXd> contact_force_detected(mxGetPr(pm), mxGetNumberOfElements(pm), 1);
  narg++;

  pm = prhs[narg];
  double contact_threshold = mxGetScalar(pm);
  narg++;

  pm = prhs[narg];
  double terrain_height = mxGetScalar(pm);
  narg++;

  Matrix<bool, Dynamic, 1> b_contact_force_detected = Matrix<bool, Dynamic, 1>::Zero(contact_force_detected.size());
  for (int i = 0; i < contact_force_detected.size(); i++) {
    b_contact_force_detected(i) = (contact_force_detected(i) != 0);
  }

  vector<SupportStateElement> available_supports = parseSupportData(supp_data);
  Matrix<bool, Dynamic, 1> b_mask = getActiveSupportMask(pdata->r, pdata->map_ptr, q, qd, available_supports, b_contact_force_detected, contact_threshold, terrain_height);

  VectorXd active_supp_mask = VectorXd::Zero(b_mask.size());
  for (int i = 0; i < b_mask.size(); i++) {
    if (b_mask(i)) {
      active_supp_mask(i) = 1.0;
    }
  }
  plhs[0] = eigenToMatlab(active_supp_mask);
}

