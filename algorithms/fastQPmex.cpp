#include <math.h>
#include <iostream>
#include <mex.h>

#include "fastQP.h"

using namespace Eigen;
using namespace std;


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  
  if (nrhs < 4) {
    mexErrMsgIdAndTxt("Drake:fastQP:NotEnoughInputs", "Usage [x,info,active] = fastQP(Q,f,Aeq,beq,Ain,bin,active)");
  }
  if (nlhs<1) return;

  int arg=0;
  
  mxArray* QblkDiagCellArray = (mxArray *) prhs[arg++];
  vector<Map<MatrixXd> > QblkMat;
  for (int i=0; i< mxGetNumberOfElements(QblkDiagCellArray);i++) {
  	mxArray* Qblk = mxGetCell(QblkDiagCellArray,i);
  	QblkMat.push_back(Map<MatrixXd>(mxGetPr(Qblk), mxGetM(Qblk), mxGetN(Qblk)));
  }

  Map<VectorXd> f(mxGetPr(prhs[arg]),mxGetM(prhs[arg]),mxGetN(prhs[arg])); arg++;
  Map<MatrixXd> Aeq(mxGetPr(prhs[arg]),mxGetM(prhs[arg]),mxGetN(prhs[arg])); arg++;
  Map<VectorXd> beq(mxGetPr(prhs[arg]),mxGetNumberOfElements(prhs[arg])); arg++;
  Map<MatrixXd> Ain(mxGetPr(prhs[arg]),mxGetM(prhs[arg]),mxGetN(prhs[arg])); arg++;
  Map<VectorXd> bin(mxGetPr(prhs[arg]),mxGetNumberOfElements(prhs[arg])); arg++;

  set<int> active;
  double* pact = mxGetPr(prhs[arg]);
  for (int i=0; i<mxGetNumberOfElements(prhs[arg]); i++)
  		active.insert((int)pact[i]);

  plhs[0] = mxCreateDoubleMatrix(f.rows(), 1, mxREAL);
  Map<VectorXd> x(mxGetPr(plhs[0]),f.rows());

  int info = fastQP(QblkMat,f,Aeq,beq,Ain,bin,active,x);

  if (nlhs>1) plhs[1] = mxCreateDoubleScalar((double)info);
  if (nlhs>2) {
  	plhs[2] = mxCreateDoubleMatrix(active.size(),1,mxREAL); pact = mxGetPr(plhs[2]);
  	int i=0;
  	for (set<int>::iterator iter = active.begin(); iter!=active.end(); iter++)
  		pact[i++] = (double) *iter;
  }
}

