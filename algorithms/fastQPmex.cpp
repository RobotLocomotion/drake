#include <math.h>
#include <iostream>
#include <mex.h>
#include <vector>
#include <set>
#include "fastQP.h"

using namespace Eigen;
using namespace std;


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  
  if (nrhs < 2) {
    mexErrMsgIdAndTxt("Drake:fastQP:NotEnoughInputs", "Usage [x,info,active] = fastQP(Q,f[,Aeq,beq,Ain,bin,active])");
  }
  if (nlhs<1) return;

  int arg=0;

  vector< Map<MatrixXd> > QblkMat;
  if (mxIsCell(prhs[arg])) {
  	mxArray* QblkDiagCellArray = (mxArray *) prhs[arg++];
  	for (int i=0; i< mxGetNumberOfElements(QblkDiagCellArray);i++) {
  		mxArray* Qblk = mxGetCell(QblkDiagCellArray,i);
  		int m=mxGetM(Qblk),n=mxGetN(Qblk);
  		if (m*n==0) continue;
  		else if (m==1 || n==1) // then it's a vector
  			QblkMat.push_back(Map<MatrixXd>(mxGetPr(Qblk), m*n, 1));  // always want a column vector
  		else
  			QblkMat.push_back(Map<MatrixXd>(mxGetPr(Qblk), m, n));
  	}
    if (QblkMat.size()<1) mexErrMsgIdAndTxt("Drake:FastQP:BadInputs","Q is empty");
  } else {
		int m=mxGetM(prhs[arg]),n=mxGetN(prhs[arg]);
		if (m*n==0)
			mexErrMsgIdAndTxt("Drake:FastQP:BadInputs","Q is empty");
		else if (m==1 || n==1) // then it's a vector
			QblkMat.push_back(Map<MatrixXd>(mxGetPr(prhs[arg]),m*n,1)); // always want a column vector
		else
			QblkMat.push_back(Map<MatrixXd>(mxGetPr(prhs[arg]),m,n));
		arg++;
  }

  Map<VectorXd>f(mxGetPr(prhs[arg]),mxGetNumberOfElements(prhs[arg])); arg++;
  int N = f.rows();  // support row or column vectors

  Map<MatrixXd> Aeq(NULL,0,N);
  Map<VectorXd> beq(NULL,0);
  Map<MatrixXd> Ain(NULL,0,N);
  Map<VectorXd> bin(NULL,0);

  if (nrhs>arg) new (&Aeq) Map<MatrixXd>(mxGetPr(prhs[arg]),mxGetM(prhs[arg]),mxGetN(prhs[arg]));
  arg++;

  if (nrhs>arg) new (&beq) Map<VectorXd>(mxGetPr(prhs[arg]),mxGetNumberOfElements(prhs[arg]));
  arg++;

  if (nrhs>arg) new (&Ain) Map<MatrixXd>(mxGetPr(prhs[arg]),mxGetM(prhs[arg]),mxGetN(prhs[arg]));
  arg++;

  if (nrhs>arg) new (&bin) Map<VectorXd>(mxGetPr(prhs[arg]),mxGetNumberOfElements(prhs[arg]));
  arg++;

  set<int> active;
  if (nrhs>arg) {
  	double* pact = mxGetPr(prhs[arg]);
  	for (int i=0; i<mxGetNumberOfElements(prhs[arg]); i++)
  		active.insert((int)pact[i]);
  }

  plhs[0] = mxCreateDoubleMatrix(f.rows(), 1, mxREAL);
  Map<VectorXd> x(mxGetPr(plhs[0]),f.rows());
  
  int info = fastQP(QblkMat,f, Aeq, beq, Ain, bin,active,x);
  if (nlhs>1) plhs[1] = mxCreateDoubleScalar((double)info);
  if (nlhs>2) {
  	plhs[2] = mxCreateDoubleMatrix(active.size(),1,mxREAL);
  	int i=0; double* pact = mxGetPr(plhs[2]);
  	for (set<int>::iterator iter = active.begin(); iter!=active.end(); iter++)
  		pact[i++] = (double) *iter;
  }
}

