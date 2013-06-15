#include <math.h>
#include <iostream>
#include <mex.h>

#include "fastQP.h"

using namespace Eigen;
using namespace std;


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  
  if (nrhs < 2) {
    mexErrMsgIdAndTxt("Drake:fastQP:NotEnoughInputs", "Usage [x,info,active] = fastQP(Q,f[,Aeq,beq,Ain,bin,active])");
  }
  if (nlhs<1) return;

  int arg=0;

	vector< MatrixBase <Map <MatrixXd> > > QblkMat;
  if (mxIsCell(prhs[arg])) {
		mxArray* QblkDiagCellArray = (mxArray *) prhs[arg++];
		for (int i=0; i< mxGetNumberOfElements(QblkDiagCellArray);i++) {
			mxArray* Qblk = mxGetCell(QblkDiagCellArray,i);
			QblkMat.push_back(Map<MatrixXd>(mxGetPr(Qblk), mxGetM(Qblk), mxGetN(Qblk)));
		}
  } else {
  	QblkMat.push_back(Map<MatrixXd>(mxGetPr(prhs[arg]),mxGetM(prhs[arg]),mxGetN(prhs[arg]))); arg++;
  }

  Map<VectorXd> f(mxGetPr(prhs[arg]),mxGetM(prhs[arg]),mxGetN(prhs[arg])); arg++;
  int N = f.rows()*f.cols();  // support row or column vectors

  Map<MatrixXd> *Aeq,*Ain;  Map<VectorXd> *beq, *bin;

  if (nrhs>arg) Aeq = new Map<MatrixXd>(mxGetPr(prhs[arg]),mxGetM(prhs[arg]),mxGetN(prhs[arg]));
  else Aeq = new Map<MatrixXd>(NULL,0,N);
  arg++;

  if (nrhs>arg) beq = new Map<VectorXd>(mxGetPr(prhs[arg]),mxGetNumberOfElements(prhs[arg]));
  else beq = new Map<VectorXd>(NULL,0);
  arg++;

  if (nrhs>arg) Ain = new Map<MatrixXd>(mxGetPr(prhs[arg]),mxGetM(prhs[arg]),mxGetN(prhs[arg]));
  else Ain = new Map<MatrixXd>(NULL,0,N);
  arg++;

  if (nrhs>arg) bin = new Map<VectorXd>(mxGetPr(prhs[arg]),mxGetNumberOfElements(prhs[arg]));
  else bin = new Map<VectorXd>(NULL,0);
  arg++;

  set<int> active;
  if (nrhs>arg) {
  	double* pact = mxGetPr(prhs[arg]);
  	for (int i=0; i<mxGetNumberOfElements(prhs[arg]); i++)
  		active.insert((int)pact[i]);
  }

  plhs[0] = mxCreateDoubleMatrix(f.rows(), 1, mxREAL);
  Map<VectorXd> x(mxGetPr(plhs[0]),f.rows());

  int info = fastQP(QblkMat,f,*Aeq,*beq,*Ain,*bin,active,x);

  if (nlhs>1) plhs[1] = mxCreateDoubleScalar((double)info);
  if (nlhs>2) {
  	plhs[2] = mxCreateDoubleMatrix(active.size(),1,mxREAL);
  	int i=0; double* pact = mxGetPr(plhs[2]);
  	for (set<int>::iterator iter = active.begin(); iter!=active.end(); iter++)
  		pact[i++] = (double) *iter;
  }
}

