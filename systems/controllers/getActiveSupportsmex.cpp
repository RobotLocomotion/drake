#include "QPCommon.h"
#include <Eigen/StdVector>
#include "drakeUtil.h"
#include "controlUtil.h"

using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (nrhs<1) mexErrMsgTxt("usage: mask=getActiveSupportsmex(ptr,support_data,contact_sensor,contact_threshold,terrain_height))");
	if (nlhs<1) mexErrMsgTxt("please take at least one output");

	struct QPControllerData* pdata;
	const mxArray* pm;

	// first get the ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:QPControllermex:BadInputs","the first argument should be the ptr");
  memcpy(&pdata,mxGetData(prhs[0]),sizeof(pdata));

  int narg = 1;
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

  int nsupp = mxGetN(supp_data);
  assert(!mxIsEmpty(supp_data));

  // Matrix<bool, Dynamic, 1> needs_kin_check;
  // needs_kin_check = Matrix<bool, Dynamic, 1>::Zero(nbod);
  bool needs_kin_check;
  bool kin_contact;
  bool force_contact;
  bool is_active;

  VectorXd active_supp_mask = VectorXd::Zero(nsupp);
  vector<SupportStateElement> available_supports = parseSupportData(supp_data);
  vector<SupportStateElement> active_supports;

  // MatrixXd contact_pts;
  // Vector4d contact_pt = Vector4d::Zero();
  // contact_pt(3) = 1.0;
  // int num_pts;
  VectorXd phi;
  SupportStateElement se;

  for (int i = 0; i < available_supports.size(); i++) {
    se = available_supports[i];

    force_contact = (contact_force_detected(se.body_idx) != 0);
    // Determine if the body needs to be checked for kinematic contact. We only
    // need to check for kin contact if the logic map indicates that the
    // presence or absence of such contact would  affect the decision about
    // whether to use that body as a support.
    needs_kin_check = (((se.support_logic_map[1] != se.support_logic_map[0]) && (contact_force_detected(se.body_idx) == 0)) ||
                       ((se.support_logic_map[3] != se.support_logic_map[2]) && (contact_force_detected(se.body_idx) == 1)));


    if (needs_kin_check) {
      if (contact_threshold == -1) {
        kin_contact = true;
      } else {
        contactPhi(pdata->r,se,pdata->map_ptr,phi,terrain_height);
        kin_contact = (phi.minCoeff()<=contact_threshold);
      }
    } else {
      kin_contact = false; // we've determined already that kin contact doesn't matter for this support element
    }

    // mexPrintf("phi\n");
    // Implement the logic described in QPInput2D.m
    if (!force_contact && !kin_contact) {
      is_active = se.support_logic_map[0]; 
    } else if (!force_contact && kin_contact) {
      is_active = se.support_logic_map[1];
    } else if (force_contact && !kin_contact) {
      is_active = se.support_logic_map[2];
    } else  { // (force_contact && kin_contact)
      is_active = se.support_logic_map[3];
    }
    // mexPrintf("logic map\n");

    if (is_active) {
      active_supports.push_back(se);
      active_supp_mask(i) = 1;
    }
  }

  plhs[0] = eigenToMatlab(active_supp_mask);
}

