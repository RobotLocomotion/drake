#include "QPCommon.h"
#include <Eigen/StdVector>
#include "drakeUtil.h"

using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (nrhs<1) mexErrMsgTxt("usage: mask=getActiveSupportsmex(ptr,support_data,contact_sensor,contact_threshold,terrain_height)) or [mask,supp_element_ptr]=getActiveSupportsmex(ptr,support_data,contact_sensor,contact_threshold,terrain_height))");
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

  int body_id;
  bool logic_map[4];
  double *logic_map_double;

  VectorXd active_supp_mask = VectorXd::Zero(nsupp);
  vector<SupportStateElement> active_supports;

  MatrixXd contact_pts;
  Vector4d contact_pt = Vector4d::Zero();
  contact_pt(3) = 1.0;
  int num_pts;
  VectorXd phi;

  int i, j;
  for (i = 0; i < nsupp; i++) {
    // mexPrintf("i\n");
    body_id = ((int) mxGetScalar(mxGetField(supp_data, i, "body_id"))) - 1;
    // mexPrintf("bodyid\n");
    num_pts = mxGetN(mxGetField(supp_data, i, "contact_pts"));
    // mexPrintf("num_pts\n");

    pm = mxGetField(supp_data, i, "support_logic_map");
    if (mxIsDouble(pm)) {
      logic_map_double = mxGetPr(pm);
      for (j = 0; j < 4; j++) {
        logic_map[j] = logic_map_double[j] != 0;
      }
    } else {
      mexErrMsgTxt("Please convert support_logic_map to double");
    }
    // mexPrintf("logic map: %f %f %f %f\n", logic_map_double[0], logic_map_double[1], logic_map_double[2], logic_map_double[3]);
    pm = mxGetField(supp_data, i, "contact_pts");
    contact_pts.resize(mxGetM(pm), mxGetN(pm));
    memcpy(contact_pts.data(), mxGetPr(pm), sizeof(double)*mxGetNumberOfElements(pm));

    force_contact = (contact_force_detected(body_id) != 0);
    // Determine if the body needs to be checked for kinematic contact. We only
    // need to check for kin contact if the logic map indicates that the
    // presence or absence of such contact would  affect the decision about
    // whether to use that body as a support.
    needs_kin_check = (((logic_map[1] != logic_map[0]) && (contact_force_detected(body_id) == 0)) ||
                       ((logic_map[3] != logic_map[2]) && (contact_force_detected(body_id) == 1)));

    // mexPrintf("kin check\n");
    SupportStateElement se;
    se.body_idx = body_id;
    for (j = 0; j < num_pts; j++) {
      contact_pt.head(3) = contact_pts.col(j);
      se.contact_pts.push_back(contact_pt);
    }
    se.contact_surface = ((int) mxGetScalar(mxGetField(supp_data, i, "contact_surfaces"))) - 1;

    // mexPrintf("contact_surfaces\n");
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
      is_active = logic_map[0]; 
    } else if (!force_contact && kin_contact) {
      is_active = logic_map[1];
    } else if (force_contact && !kin_contact) {
      is_active = logic_map[2];
    } else  { // (force_contact && kin_contact)
      is_active = logic_map[3];
    }
    // mexPrintf("logic map\n");

    if (is_active) {
      active_supports.push_back(se);
      active_supp_mask(i) = 1;
    }
  }

  plhs[0] = eigenToMatlab(active_supp_mask);

  if (nlhs >= 1) {
    mxClassID cid;
    if (sizeof(pdata)==4) cid = mxUINT32_CLASS;
    else if (sizeof(pdata)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("Drake:constructModelmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
    
    plhs[1] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[1]),&active_supports,sizeof(active_supports));
  }
}

