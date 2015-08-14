#include "mex.h"
#include "controlUtil.h"
#include "drakeMexUtil.h"
#include "controlMexUtil.h"
#include <iostream>

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if (nrhs != 7 || nlhs != 1)
  {
    mexErrMsgIdAndTxt("Drake:testControlUtil:BadInputs","Usage: individualCentersOfPressuremex(model_ptr, cache_ptr, active_supports, normals, nd, B, beta");
  }

  int arg_num = 0;
  RigidBodyManipulator *model = static_cast<RigidBodyManipulator*>(getDrakeMexPointer(prhs[arg_num++]));
  KinematicsCache<double>* cache = static_cast<KinematicsCache<double>*>(getDrakeMexPointer(prhs[arg_num++]));

  vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> active_supports;
  int num_active_contact_pts = 0;
  if (!mxIsEmpty(prhs[arg_num])) {
    mxArray* mxBodies = myGetField(prhs[arg_num], "bodies");
    if (!mxBodies)
      mexErrMsgTxt("couldn't get bodies");
    double* pBodies = mxGetPrSafe(mxBodies);

    mxArray* mxContactPts = myGetField(prhs[arg_num], "contact_pts");
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
  arg_num++;

//  df_ext = new Map<MatrixXd>(mxGetPrSafe(prhs[4]),6*model->NB,2*model->num_dof);
  MatrixXd normals(3,num_active_contact_pts);
  memcpy(normals.data(), mxGetPrSafe(prhs[arg_num++]), sizeof(double)*normals.size());

  int nd = static_cast<int>(mxGetScalar(prhs[arg_num++]));
  MatrixXd B(3, num_active_contact_pts * nd);
  memcpy(B.data(), mxGetPrSafe(prhs[arg_num++]), sizeof(double)*B.size());

  VectorXd beta(num_active_contact_pts * nd);
  memcpy(beta.data(), mxGetPrSafe(prhs[arg_num++]), sizeof(double)*beta.size());

  MatrixXd individual_cops = individualSupportCOPs(model, *cache, active_supports, normals, B, beta);

  plhs[0] = eigenToMatlab(individual_cops);
}
