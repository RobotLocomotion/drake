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

  int arg=0, nblks=1;

  if (mxIsCell(prhs[arg])) nblks = mxGetNumberOfElements(prhs[arg]);
  MatrixXd* Q = new MatrixXd[nblks];

  vector < MatrixXd* > QblkMat;

  /*
   * NOTE: I am copying memory from the matlab inputs to the MatrixXd structures in the loop below.
   * This could be avoided by passing Map<>* through to fastQP, but the getting all of the templates
   *  right is a pain and I'm out of time.  :)  Will finish it later.
   */

  if (mxIsCell(prhs[arg])) {
  	mxArray* QblkDiagCellArray = (mxArray *) prhs[arg++];
  	for (int i=0; i<nblks; i++) {
  		mxArray* Qblk = mxGetCell(QblkDiagCellArray,i);
  		int m=mxGetM(Qblk),n=mxGetN(Qblk);
  		if (m*n==0) continue;
  		else if (m==1 || n==1)  // then it's a vector
  			Q[i] = Map<MatrixXd>(mxGetPr(Qblk), m*n, 1);
  		else
  			Q[i] = Map<MatrixXd>(mxGetPr(Qblk), m, n);
  		QblkMat.push_back(&Q[i]);
  	}
    if (QblkMat.size()<1) mexErrMsgIdAndTxt("Drake:FastQP:BadInputs","Q is empty");
  } else {
		int m=mxGetM(prhs[arg]),n=mxGetN(prhs[arg]);
		if (m*n==0)
			mexErrMsgIdAndTxt("Drake:FastQP:BadInputs","Q is empty");
		else if (m==1 || n==1) // then it's a vector
			Q[0] = Map<MatrixXd>(mxGetPr(prhs[arg]),m*n,1); // always want a column vector
		else
			Q[0] = Map<MatrixXd>(mxGetPr(prhs[arg]),m,n);
		arg++;
		QblkMat.push_back(&Q[0]);
  }


  Map<VectorXd>f(mxGetPr(prhs[arg]),mxGetNumberOfElements(prhs[arg])); arg++;
  int N = f.rows();  // support row or column vectors

  Map<MatrixXd> Aeq(NULL,0,N);
  Map<VectorXd> beq(NULL,0);
  Map<MatrixXd> Ain(NULL,0,N);
  Map<VectorXd> bin(NULL,0);

  if (nrhs>arg && mxGetNumberOfElements(prhs[arg])>0) new (&Ain) Map<MatrixXd>(mxGetPr(prhs[arg]),mxGetM(prhs[arg]),mxGetN(prhs[arg]));
  arg++;

  if (nrhs>arg && mxGetNumberOfElements(prhs[arg])>0) new (&bin) Map<VectorXd>(mxGetPr(prhs[arg]),mxGetNumberOfElements(prhs[arg]));
  arg++;

  if (nrhs>arg && mxGetNumberOfElements(prhs[arg])>0) new (&Aeq) Map<MatrixXd>(mxGetPr(prhs[arg]),mxGetM(prhs[arg]),mxGetN(prhs[arg]));
  arg++;

  if (nrhs>arg && mxGetNumberOfElements(prhs[arg])>0) new (&beq) Map<VectorXd>(mxGetPr(prhs[arg]),mxGetNumberOfElements(prhs[arg]));
  arg++;

  set<int> active;
  if (nrhs>arg) {
  	double* pact = mxGetPr(prhs[arg]);
  	for (int i=0; i<mxGetNumberOfElements(prhs[arg]); i++)
  		active.insert((int)pact[i]-1);
  }

  // NOTE: another copy happening down here.  TODO: fix it with templates (later)
  VectorXd x(f.rows());
  int info = fastQP(QblkMat,f, Aeq, beq, Ain, bin, active, x);

  plhs[0] = mxCreateDoubleMatrix(f.rows(), 1, mxREAL);
  memcpy(mxGetPr(plhs[0]),x.data(),sizeof(double)*f.rows());

  if (nlhs>1) plhs[1] = mxCreateDoubleScalar((double)info);
  if (nlhs>2) {
  	plhs[2] = mxCreateDoubleMatrix(active.size(),1,mxREAL);
  	int i=0; double* pact = mxGetPr(plhs[2]);
  	for (set<int>::iterator iter = active.begin(); iter!=active.end(); iter++)
  		pact[i++] = (double) (*iter + 1);
  }
}

