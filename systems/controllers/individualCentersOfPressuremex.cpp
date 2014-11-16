#include "mex.h"
#include "controlUtil.h"
#include "drakeUtil.h"
#include <iostream>

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if (nrhs != 6 || nlhs != 1)
  {
    mexErrMsgIdAndTxt("Drake:testControlUtil:BadInputs","Usage r, active_supports, normals, nd, B, beta");
  }

  RigidBodyManipulator* r = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  int desired_support_argid = 1;
  vector<SupportStateElement> active_supports;
  int num_active_contact_pts = 0;
  if (!mxIsEmpty(prhs[desired_support_argid])) {
    mxArray* mxBodies = myGetField(prhs[desired_support_argid], "bodies");
    if (!mxBodies)
      mexErrMsgTxt("couldn't get bodies");
    double* pBodies = mxGetPr(mxBodies);

    mxArray* mxContactPts = myGetField(prhs[desired_support_argid], "contact_pts");
    if (!mxContactPts)
      mexErrMsgTxt("couldn't get contact points");

    mxArray* mxContactSurfaces = myGetField(prhs[desired_support_argid], "contact_surfaces");
    if (!mxContactSurfaces)
      mexErrMsgTxt("couldn't get contact surfaces");
    double* pContactSurfaces = mxGetPr(mxContactSurfaces);

    for (int i = 0; i < mxGetNumberOfElements(mxBodies); i++) {
      mxArray* mxBodyContactPts = mxGetCell(mxContactPts, i);
      int nc = static_cast<int>(mxGetNumberOfElements(mxBodyContactPts));
      if (nc < 1)
        continue;

      SupportStateElement se;
      se.body_idx = (int) pBodies[i] - 1;
      double* pr = mxGetPr(mxBodyContactPts);
      for (int j = 0; j < nc; j++) {
        se.contact_pt_inds.insert((int) pr[j] - 1);
      }
      se.contact_surface = (int) pContactSurfaces[i] - 1;

      active_supports.push_back(se);
      num_active_contact_pts += nc;
    }
  }

//  df_ext = new Map<MatrixXd>(mxGetPr(prhs[4]),6*model->NB,2*model->num_dof);
  MatrixXd normals(3,num_active_contact_pts);
  memcpy(normals.data(), mxGetPr(prhs[2]), sizeof(double)*normals.size());

  int nd = static_cast<int>(mxGetScalar(prhs[3]));
  MatrixXd B(3, num_active_contact_pts * nd);
  memcpy(B.data(), mxGetPr(prhs[4]), sizeof(double)*B.size());

  VectorXd beta(num_active_contact_pts * nd);
  memcpy(beta.data(), mxGetPr(prhs[5]), sizeof(double)*beta.size());

  MatrixXd individual_cops = individualSupportCOPs(r, active_supports, normals, B, beta);

  plhs[0] = eigenToMatlab(individual_cops);
}
