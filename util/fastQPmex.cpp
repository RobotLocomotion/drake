#include <vector>
#include <math.h>
#include <set>
#include <Eigen/Dense>
#include <iostream>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <mex.h>

#define _USE_MATH_DEFINES

#define MAX_CONSTRS 1000
#define MAX_STATE   1000
#define MAX_ITER    20

using namespace Eigen;
using namespace std;

int fastQP(vector< Map<MatrixXd> > QblkDiag, const Map<VectorXd> f, const Map<MatrixXd> Aeq, const Map<VectorXd> beq, const Map<MatrixXd> Ain, const Map<VectorXd> bin, set<int>& active, Map<VectorXd>& x) {

  int i,d;
  int fail = 0;
  int iterCnt = 0;
  
  int M_in = bin.size();
  int M = Aeq.rows();
  int N = Aeq.cols();
  
  if (f.rows() != N) mexErrMsgTxt("size of f doesn't match cols of Aeq");
  if (beq.rows() !=M) mexErrMsgTxt("size of beq doesn't match rows of Aeq");
  if (Ain.cols() !=N) mexErrMsgTxt("cols of Ain doesn't match cols of Aeq");
  if (bin.rows() != Ain.rows()) mexErrMsgTxt("bin rows doesn't match Ain rows");
  if (x.rows() != N) mexErrMsgTxt("x doesn't match Aeq");

  int n_active = active.size();
  MatrixXd Aact = MatrixXd(n_active, N);
  VectorXd bact = VectorXd(n_active);
  VectorXd Qdiag = VectorXd::Constant(N,1e-8);  

  MatrixXd QinvAteq(N,M);
  VectorXd minusQinvf(N);

  vector<MatrixXd> Qinv;

  // calculate a bunch of stuff that is constant during each iteration
  int startrow=0;
  for (vector<Map<MatrixXd> >::iterator iterQ=QblkDiag.begin(); iterQ!=QblkDiag.end(); iterQ++) {
  	int numRow = iterQ->rows();
  	int numCol = iterQ->cols();

  	if (numRow == 1 || numCol == 1) {  // it's a vector
  		d = numRow*numCol;
      VectorXd Qdiag_mod = *iterQ + VectorXd::Constant(d,1e-8); // regularize
      VectorXd QinvDiag = Qdiag_mod.cwiseInverse();
      QinvAteq.block(startrow,0,d,M)= QinvDiag.asDiagonal()*Aeq.block(0,startrow,M,d).transpose();  // Aeq.transpose().block(startrow,0,d,N)
      minusQinvf.segment(startrow,d) = -QinvDiag.cwiseProduct(f.segment(startrow,d));
      Qinv.push_back(QinvDiag);
      startrow=startrow+d;
  	} else { // potentially dense matrix
  		d = numRow;  assert(numRow==numCol);
      MatrixXd Q_mod = *iterQ + 1e-8*MatrixXd::Identity(d,d);
      MatrixXd thisQinv = Q_mod.inverse();
      QinvAteq.block(startrow,0,d,M) = thisQinv*Aeq.block(0,startrow,M,d).transpose();  // Aeq.transpose().block(startrow,0,d,N)
      minusQinvf.segment(startrow,d) = -thisQinv*f.segment(startrow,d);
      Qinv.push_back(thisQinv);
      startrow=startrow+d;
  	}
  	if (startrow>N) {
  		mexErrMsgTxt("Q is too big!");
  	}
  }

  MatrixXd A;
  VectorXd b;
  MatrixXd QinvAt;
  VectorXd lam, lamIneq;
  VectorXd violated(M_in);
  VectorXd violation;
  
  while(1) {
    n_active = active.size();
    Aact.resize(n_active,N);
    bact.resize(n_active);

    for (set<int>::iterator iter=active.begin(); iter!=active.end(); iter++) {
      Aact.row(i) = Ain.row(*iter);
      bact(i) = bin(*iter);
    }

    A.resize(Aeq.rows() + Aact.rows(),N);
    b.resize(beq.size() + bact.size());
    A << Aeq,Aact;
    b << beq,bact;
    
    if (A.rows() > 0) {
      //Solve H * [x;lam] = [-f;b] using Schur complements
      QinvAt.resize(QinvAteq.rows(), QinvAteq.cols() + Aact.rows());

      if (n_active>0) {
				int startrow=0;
				for (vector<MatrixXd>::iterator iterQinv=Qinv.begin(); iterQinv!=Qinv.end(); iterQinv++) {
					d = iterQinv->rows();
					int numCol = iterQinv->cols();

					if (numCol == 1) {  // it's a vector
						QinvAt.block(startrow,0,d,M) << QinvAteq.block(startrow,0,d,M), ((*iterQinv).asDiagonal())*Aact.block(0,startrow,n_active,d).transpose();
					} else { // it's a matrix
						QinvAt.block(startrow,0,d,M) << QinvAteq.block(startrow,0,d,M), (*iterQinv)*Aact.block(0,startrow,n_active,d).transpose();
					}
				}
      } else {
      	QinvAt = QinvAteq;
      }
      
      lam.resize(QinvAt.cols());
      lam = -(A*QinvAt).ldlt().solve(b + (f.transpose()*QinvAt).transpose());
      
      x = minusQinvf - QinvAt*lam;
      lamIneq = lam.tail(lam.size() - M);
    } else {
      x = minusQinvf;
      lamIneq.resize(0);
    }
    
    if(Ain.rows() == 0) {
      active.clear();
      break;
    }
//     cout << "dbg: " << A << endl;
    
    set<int> new_active;

    violation = Ain*x - bin;

    for (i=0; i<M_in; i++)
      if (violation(i) >= 1e-6)
      	new_active.insert(i);
    
    bool all_pos_mults = true;
    for (i=0; i<n_active; i++) {
    	if (lamIneq(i)<-1e-6) {
    		all_pos_mults = false;
    		break;
    	}
    }

    if (new_active.empty() && all_pos_mults) {
    	// existing active was AOK
    	break;
    }

    i=0;
    for (set<int>::iterator iter=active.begin(); iter!=active.end(); iter++)
    	if (lamIneq(i++)>=-1e-6)
    		active.erase(*iter);
    active.insert(new_active.begin(),new_active.end());

    iterCnt++;

    if (iterCnt > MAX_ITER) {
      //Default to calling this method
      cout << "FastQP max iter reached." << endl;
//       mexErrMsgIdAndTxt("Drake:approximateIKmex:Error", "Max iter reached. Problem is likely infeasible");
      fail = 1;
      return fail;
    }
  }  
  return fail;
}


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

