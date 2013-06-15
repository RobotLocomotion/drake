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
    mexErrMsgIdAndTxt("Drake:gurobiQP:NotEnoughInputs", "Usage [x,status,active] = gurobiQP(Q,f[,Aeq,beq,Ain,bin,lb,ub,active])");
  }
  if (nlhs<1) return;

  GRBenv *env = NULL;
  CGE ( GRBloadenv(&env,NULL), env);

  // set solver params (http://www.gurobi.com/documentation/5.5/reference-manual/node798#sec:Parameters)
  // todo: take these as an argument?
  CGE ( GRBsetintparam(env,"outputflag",0), env);
  CGE ( GRBsetintparam(env,"method",2), env);
  CGE ( GRBsetintparam(env,"presolve",0), env);
  CGE ( GRBsetintparam(env,"bariterlimit",20), env);
  CGE ( GRBsetintparam(env,"barhomogenous",0), env);
  CGE ( GRBsetdblparam(env,"barconvtol",0.0005), env);

  int arg=0;

  vector<Map<MatrixXd> > QblkMat;
  if (mxIsCell(prhs[arg])) {
  	mxArray* QblkDiagCellArray = (mxArray *) prhs[arg++];
  	for (int i=0; i< mxGetNumberOfElements(QblkDiagCellArray);i++) {
  		mxArray* Qblk = mxGetCell(QblkDiagCellArray,i);
  		QblkMat.push_back(Map<MatrixXd>(mxGetPr(Qblk), mxGetM(Qblk), mxGetN(Qblk)));
  	}
  } else {
  	QblkMat.push_back(Map<MatrixXd>(mxGetPr(prhs[arg]),mxGetM(prhs[arg]),mxGetN(prhs[arg]))); arg++;
  }

  int nparams = mxGetNumberOfElements(prhs[arg]);

  VectorXd f(nparams);
  memcpy(f.data(),mxGetPr(prhs[arg++]),sizeof(double)*nparams);

  Map<MatrixXd> Aeq(NULL,0,nparams);
  Map<VectorXd> beq(NULL,0);
  Map<MatrixXd> Ain(NULL,0,nparams);
  Map<VectorXd> bin(NULL,0);
  VectorXd lb;
  VectorXd ub;

  if (nrhs>arg) new (&Aeq) Map<MatrixXd>(mxGetPr(prhs[arg]),mxGetM(prhs[arg]),mxGetN(prhs[arg]));
  arg++;

  if (nrhs>arg) new (&beq) Map<VectorXd>(mxGetPr(prhs[arg]),mxGetNumberOfElements(prhs[arg]));
  arg++;

  if (nrhs>arg) new (&Ain) Map<MatrixXd>(mxGetPr(prhs[arg]),mxGetM(prhs[arg]),mxGetN(prhs[arg]));
  arg++;

  if (nrhs>arg) new (&bin) Map<VectorXd>(mxGetPr(prhs[arg]),mxGetNumberOfElements(prhs[arg]));
  arg++;

  if (nrhs>arg && mxGetNumberOfElements(prhs[arg])>0) {
  	if (nparams != mxGetNumberOfElements(prhs[arg])) mexErrMsgTxt("lb must be the same size as f");
  	lb.resize(nparams);
  	memcpy(lb.data(),mxGetPr(prhs[arg]),sizeof(double)*lb.rows());
  }
  arg++;

  if (nrhs>arg && mxGetNumberOfElements(prhs[arg])>0) {
  	ub.resize(mxGetNumberOfElements(prhs[arg]));
  	memcpy(ub.data(),mxGetPr(prhs[arg]),sizeof(double)*ub.rows());
  }
  arg++;

  set<int> active;
  if (nrhs>arg) {
  	double* pact = mxGetPr(prhs[arg]);
  	for (int i=0; i<mxGetNumberOfElements(prhs[arg]); i++)
  		active.insert((int)pact[i]);
  }

  VectorXd x(f.rows());
  GRBmodel * model = gurobiQP(env,QblkMat,f, Aeq, beq, Ain, bin, lb, ub, active, x);

  plhs[0] = mxCreateDoubleMatrix(f.rows(), 1, mxREAL);
  memcpy(mxGetPr(plhs[0]),x.data(),sizeof(double)*f.rows());

  int status;
  CGE ( GRBgetintattr(model, "Status", &status) , env);
  if (nlhs>1) plhs[1] = mxCreateDoubleScalar((double)status);
  if (nlhs>2) {
  	plhs[2] = mxCreateDoubleMatrix(active.size(),1,mxREAL);
  	int i=0; double* pact = mxGetPr(plhs[2]);
  	for (set<int>::iterator iter = active.begin(); iter!=active.end(); iter++)
  		pact[i++] = (double) *iter;
  }
  GRBfreemodel(model);
}

