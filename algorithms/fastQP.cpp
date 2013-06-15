#include <math.h>
#include <iostream>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/SVD>

#include "fastQP.h"

#define _USE_MATH_DEFINES

#define MAX_CONSTRS 1000
#define MAX_STATE   1000
#define MAX_ITER    20

using namespace Eigen;
using namespace std;

template <typename tA, typename tB, typename tC, typename tD, typename tE, typename tF, typename tG>
int fastQP(vector< MatrixBase<tA> > QblkDiag, const MatrixBase<tB> f, const MatrixBase<tC> Aeq, const MatrixBase<tD> beq, const MatrixBase<tE> Ain, const MatrixBase<tF> bin, set<int>& active, MatrixBase<tG>& x)
{
  int i,d;
  int fail = 0;
  int iterCnt = 0;
  
  int M_in = bin.size();
  int M = Aeq.rows();
  int N = Aeq.cols();
  
  if (f.rows() != N) { cerr << "size of f (" << f.rows() << " by " << f.cols() << ") doesn't match cols of Aeq (" << Aeq.rows() << " by " << Aeq.cols() << ")" << endl; return 2; }
  if (beq.rows() !=M) { cerr << "size of beq doesn't match rows of Aeq" << endl; return 2; }
  if (Ain.cols() !=N) { cerr << "cols of Ain doesn't match cols of Aeq" << endl; return 2; };
  if (bin.rows() != Ain.rows()) { cerr << "bin rows doesn't match Ain rows" << endl; return 2; };
  if (x.rows() != N) { cerr << "x doesn't match Aeq" << endl; return 2; }

  int n_active = active.size();
  MatrixXd Aact = MatrixXd(n_active, N);
  VectorXd bact = VectorXd(n_active);
  VectorXd Qdiag = VectorXd::Constant(N,1e-8);  

  MatrixXd QinvAteq(N,M);
  VectorXd minusQinvf(N);

  vector<MatrixXd> Qinv;
  typedef typename vector< MatrixBase<tA> >::iterator Qiterator;

  // calculate a bunch of stuff that is constant during each iteration
  int startrow=0;
  for (Qiterator iterQ=QblkDiag.begin(); iterQ!=QblkDiag.end(); iterQ++) {
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
  		cerr << "Q is too big!" << endl;
  		return 2;
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

    i=0;
    for (set<int>::iterator iter=active.begin(); iter!=active.end(); iter++) {
      Aact.row(i) = Ain.row(*iter);
      bact(i++) = bin(*iter);
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
						QinvAt.block(startrow,0,d,M+n_active) << QinvAteq.block(startrow,0,d,M), ((*iterQinv).asDiagonal())*Aact.block(0,startrow,n_active,d).transpose();
					} else { // it's a matrix
						QinvAt.block(startrow,0,d,M+n_active) << QinvAteq.block(startrow,0,d,M), (*iterQinv)*Aact.block(0,startrow,n_active,d).transpose();
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
    set<int>::iterator iter=active.begin(), tmp;
    while (iter!=active.end()) { // to accomodating inloop erase
  		tmp = iter++;
    	if (lamIneq(i++)>=-1e-6) {
    		active.erase(tmp);
    	}
    }
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

template int fastQP(vector< MatrixBase< Map<MatrixXd> > >, const MatrixBase< Map<VectorXd> >, const MatrixBase< Map<MatrixXd> >, const MatrixBase< Map<VectorXd> >, const MatrixBase< Map<MatrixXd> >, const MatrixBase< Map<VectorXd> >, set<int>&, MatrixBase< Map<VectorXd> >&);
template int fastQP(vector< MatrixBase< VectorXd > >, const MatrixBase< VectorXd >, const MatrixBase< Matrix<double,-1,-1,RowMajor,1000,-1> >, const MatrixBase< Matrix<double,-1,1,0,1000,1> >, const MatrixBase< Matrix<double,-1,-1,RowMajor,1000,-1> >, const MatrixBase< Matrix<double,-1,1,0,1000,1> >, set<int>&, MatrixBase< VectorXd >&);


