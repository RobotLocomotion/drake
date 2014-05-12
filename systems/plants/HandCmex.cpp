#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"

#define INF -2147483648

using namespace Eigen;
using namespace std;

/*
 * A C version of the HandC function from the Featherstone library
 *
 * To set up the model, use HandCpmex(model[,grav_accn])
 * Then to evaluate the dynamics use
 *   [H,C] = HandCmex(q,qd[,f_ext]);
 */


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {

  if (nrhs<1) {
    mexErrMsgIdAndTxt("Drake:HandCmex:NotEnoughInputs","Usage [H,C,dH,dC] = HandCmex(model_ptr,q,qd[,f_ext,df_ext]).");
  }

  if (nrhs==4 && nlhs>2) {
    mexErrMsgIdAndTxt("Drake:HandCmex:NotEnoughInputs","You need to provide df_ext if you request dH or dC while supplying f_ext");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  
  double *q,*qd;
  Map<MatrixXd> *f_ext=NULL;
  Map<MatrixXd> *df_ext=NULL;
  if (static_cast<int>(mxGetNumberOfElements(prhs[1]))!=model->num_dof || static_cast<int>(mxGetNumberOfElements(prhs[2]))!=model->num_dof)
    mexErrMsgIdAndTxt("Drake:HandCmex:BadInputs","q and qd must be size %d x 1",model->num_dof);
  q = mxGetPr(prhs[1]);
  qd = mxGetPr(prhs[2]);
  if (nrhs>3) {
    if (!mxIsEmpty(prhs[3])) {
      f_ext = new Map<MatrixXd>(mxGetPr(prhs[3]),6,model->NB);
    }
  }
  if (nrhs>4) {
    if (!mxIsEmpty(prhs[4])) {
      df_ext = new Map<MatrixXd>(mxGetPr(prhs[4]),6*model->NB,2*model->num_dof);
    }
  }
  
  Map<MatrixXd> *dH=NULL, *dC=NULL;
  
  plhs[0] = mxCreateDoubleMatrix(model->num_dof,model->num_dof,mxREAL);
  Map<MatrixXd> H(mxGetPr(plhs[0]),model->num_dof,model->num_dof);

  plhs[1] = mxCreateDoubleMatrix(model->num_dof,1,mxREAL);
  Map<VectorXd> C(mxGetPr(plhs[1]),model->num_dof);

  if (nlhs>2) {
    plhs[2] = mxCreateDoubleMatrix(model->num_dof*model->num_dof,model->num_dof,mxREAL);
    dH = new Map<MatrixXd>(mxGetPr(plhs[2]),model->num_dof*model->num_dof,model->num_dof);
  }
  if (nlhs>3) {
    plhs[3] = mxCreateDoubleMatrix(model->num_dof,2*model->num_dof,mxREAL);
    dC = new Map<MatrixXd>(mxGetPr(plhs[3]),model->num_dof,2*model->num_dof);
  }  
  
  model->HandC(q,qd,f_ext,H,C,dH,dC,df_ext);
  
  // destroy dynamically allocated Map<MatrixXd> (but not the underlying data!)
  if (f_ext) delete f_ext;
  if (df_ext) delete df_ext;
  if (nlhs>2) delete dH;
  if (nlhs>3) delete dC;
}
