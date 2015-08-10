#include "QPCommon.h"
#include <Eigen/StdVector>
#include "drakeMexUtil.h"
#include "controlMexUtil.h"

using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (nrhs<1) mexErrMsgTxt("usage: mask=getActiveSupportsmex(ptr,x,support_data,contact_sensor,contact_threshold,terrain_height))");
	if (nlhs<1) mexErrMsgTxt("please take at least one output");

	const mxArray* pm;

	// first get the ptr back from matlab
  NewQPControllerData *pdata = (NewQPControllerData*) getDrakeMexPointer(prhs[0]);

  int nq = pdata->r->num_positions;
  int nv = pdata->r->num_velocities;

  int narg=1;  
  double *q_ptr = mxGetPrSafe(prhs[narg]);
  double *qd_ptr = &q_ptr[nq];
  Map<VectorXd> q(q_ptr, nq);
  Map<VectorXd> qd(qd_ptr, nv);
  narg++;

  const mxArray* supp_data = prhs[narg];
  narg++;

  pm = prhs[narg];
  Map<VectorXd> contact_force_detected(mxGetPrSafe(pm), mxGetNumberOfElements(pm), 1);
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

  vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> available_supports = parseSupportData(supp_data);
  Matrix<bool, Dynamic, 1> b_mask = getActiveSupportMask(pdata->r, pdata->map_ptr, q, qd, available_supports, b_contact_force_detected, contact_threshold, terrain_height);

  VectorXd active_supp_mask = VectorXd::Zero(b_mask.size());
  for (int i = 0; i < b_mask.size(); i++) {
    if (b_mask(i)) {
      active_supp_mask(i) = 1.0;
    }
  }
  plhs[0] = eigenToMatlab(active_supp_mask);
}
