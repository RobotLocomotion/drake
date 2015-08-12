#include "mex.h"
#include "controlUtil.h"
#include "drakeMexUtil.h"
#include "controlMexUtil.h"
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
  vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> active_supports;
  int num_active_contact_pts = 0;
  if (!mxIsEmpty(prhs[desired_support_argid])) {
    mxArray* mxBodies = myGetField(prhs[desired_support_argid], "bodies");
    if (!mxBodies)
      mexErrMsgTxt("couldn't get bodies");
    double* pBodies = mxGetPrSafe(mxBodies);

    mxArray* mxContactPts = myGetField(prhs[desired_support_argid], "contact_pts");
    if (!mxContactPts)
      mexErrMsgTxt("couldn't get contact points");

    for (int i = 0; i < mxGetNumberOfElements(mxBodies); i++) {
      mxArray* mxBodyContactPts = mxGetCell(mxContactPts,i);
      assert(mxGetM(mxBodyContactPts) == 3);
      int nc = static_cast<int>(mxGetN(mxBodyContactPts));
      if (nc<1) continue;
      
      Map<MatrixXd> all_body_contact_pts(mxGetPrSafe(mxBodyContactPts), mxGetM(mxBodyContactPts), mxGetN(mxBodyContactPts));

      SupportStateElement se;
      se.body_idx = (int) pBodies[i]-1;
      for (int j = 0; j < nc; j++) {
        se.contact_pts.push_back(all_body_contact_pts.col(j));
      }

      active_supports.push_back(se);
      num_active_contact_pts += nc;
    }
  }

//  df_ext = new Map<MatrixXd>(mxGetPrSafe(prhs[4]),6*model->NB,2*model->num_dof);
  MatrixXd normals(3,num_active_contact_pts);
  memcpy(normals.data(), mxGetPrSafe(prhs[2]), sizeof(double)*normals.size());

  int nd = static_cast<int>(mxGetScalar(prhs[3]));
  MatrixXd B(3, num_active_contact_pts * nd);
  memcpy(B.data(), mxGetPrSafe(prhs[4]), sizeof(double)*B.size());

  VectorXd beta(num_active_contact_pts * nd);
  memcpy(beta.data(), mxGetPrSafe(prhs[5]), sizeof(double)*beta.size());

  MatrixXd individual_cops = individualSupportCOPs(r, active_supports, normals, B, beta);

  plhs[0] = eigenToMatlab(individual_cops);
}
