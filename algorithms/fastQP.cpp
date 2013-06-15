#include <math.h>
#include <iostream>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/SVD>

#include "fastQP.h"

#define _USE_MATH_DEFINES

#define MAX_CONSTRS 1000
#define MAX_STATE   1000
#define MAX_ITER    10


using namespace Eigen;
using namespace std;

template <typename tA, typename tB, typename tC, typename tD, typename tE, typename tF, typename tG>
int fastQP(std::vector< Map<tA> > QblkDiag, const MatrixBase<tB>& f, const MatrixBase<tC>& Aeq, const MatrixBase<tD>& beq, const MatrixBase<tE>& Ain, const MatrixBase<tF>& bin, set<int>& active, MatrixBase<tG>& x)
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

  MatrixXd QinvAteq(N,M);
  VectorXd minusQinvf(N);

  vector<MatrixXd> Qinv;
  #define REG 1e-13
  // calculate a bunch of stuff that is constant during each iteration
  int startrow=0;
  //typedef typename vector< MatrixBase<tA> >::iterator Qiterator;

  for (vector<Map<MatrixXd> >::iterator iterQ=QblkDiag.begin(); iterQ!=QblkDiag.end(); iterQ++) {
  	int numRow = iterQ->rows();
  	int numCol = iterQ->cols();

  	if (numRow == 1 || numCol == 1) {  // it's a vector
  		d = numRow*numCol;
        VectorXd Qdiag_mod = *iterQ + VectorXd::Constant(d,REG); // regularize
        VectorXd QinvDiag = Qdiag_mod.cwiseInverse();
        QinvAteq.block(startrow,0,d,M)= QinvDiag.asDiagonal()*Aeq.block(0,startrow,M,d).transpose();  // Aeq.transpoODse().block(startrow,0,d,N)
        minusQinvf.segment(startrow,d) = -QinvDiag.cwiseProduct(f.segment(startrow,d));
        Qinv.push_back(QinvDiag);
        startrow=startrow+d;
  	} else { // potentially dense matrix
        d = numRow;   
        if (numRow!=numCol) {
            cerr << "Q is not square! " << numRow << "x" << numCol << "\n";
            return 2;
        }

        MatrixXd Q_mod = *iterQ + REG*MatrixXd::Identity(d,d);
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
      //Solve H * [x;lam] = [-f;b] using Schur complements, H = [Q,At';A,0];
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

                    startrow=startrow+d;
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
    
    set<int> new_active;

    violation = Ain*x - bin;
    for (i=0; i<M_in; i++)
      if (violation(i) >= 1e-6)
      	new_active.insert(i);
    
    bool all_pos_mults = true;
    for (i=0; i<n_active; i++) {
    	if (lamIneq(i)<0) {
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
    	if (lamIneq(i++)<0) {
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





template <typename DerivedA,typename DerivedB>
int myGRBaddconstrs(GRBmodel *model, MatrixBase<DerivedA> const & A, MatrixBase<DerivedB> const & b, char sense, double sparseness_threshold = 1e-10)
{
  int i,j,nnz,error;
/*
  // todo: it seems like I should just be able to do something like this:
  SparseMatrix<double,RowMajor> sparseAeq(Aeq.sparseView());
  sparseAeq.makeCompressed();
  error = GRBaddconstrs(model,nq_con,sparseAeq.nonZeros(),sparseAeq.InnerIndices(),sparseAeq.OuterStarts(),sparseAeq.Values(),beq.data(),NULL);
*/

  int *cind = new int[A.cols()];
  double* cval = new double[A.cols()];
  for (i=0; i<A.rows(); i++) {
    nnz=0;
    for (j=0; j<A.cols(); j++) {
      if (abs(A(i,j))>sparseness_threshold) {
        cval[nnz] = A(i,j);
        cind[nnz++] = j;
      }
    }
    error = GRBaddconstr(model,nnz,cind,cval,sense,b(i),NULL);
    if (error) break;
  }

  delete[] cind;
  delete[] cval;
  return error;
}

template <typename tA, typename tB, typename tC, typename tD, typename tE>
GRBmodel* gurobiQP(GRBenv *env, vector< Map<tA> > QblkDiag, VectorXd& f, const MatrixBase<tB>& Aeq, const MatrixBase<tC>& beq, const MatrixBase<tD>& Ain, const MatrixBase<tE>& bin, VectorXd& lb, VectorXd& ub, set<int>& active, VectorXd& x)
{
	// Note: f,lb, and ub are VectorXd instead of const MatrixBase templates because i want to be able to call f.data() on them

	// NOTE:  this allocates memory for a new GRBmodel and returns it. (you should delete this object when you're done with it)
	// NOTE:  by convention here, the active set indices correspond to Ain,bin first, then lb, then ub.

	// WARNING:  If there are no constraints, I think you will need to do scale (f = 2*f);
	//  				 This is very strange, and appears to be related to the bug Michael found in gurobi.

  GRBmodel *model = NULL;

  int i,j,nparams = f.rows();
  CGE (GRBnewmodel(env,&model,"QP",nparams,NULL,lb.data(),ub.data(),NULL,NULL), env);

  int startrow=0,d;
  for (vector<Map<MatrixXd> >::iterator iterQ=QblkDiag.begin(); iterQ!=QblkDiag.end(); iterQ++) {
  	Map<tA> Q=*iterQ;
  	if (Q.rows() == 1 || Q.cols() == 1) {  // it's a vector
  		d = Q.rows()*Q.cols();
  	  for (i=0; i<d; i++)
  	  	CGE (GRBaddqpterms(model,1,&i,&i,&(Q(i))), env);
  	  startrow=startrow+d;
  	} else { // potentially dense matrix
  		d = Q.rows();
  		if (d!=Q.cols()) {
  			cerr << "Q is not square! " << Q.rows() << "x" << Q.cols() << "\n";
  			return NULL;
  		}

  	  for (i=0; i<d; i++)
    	  for (j=0; j<d; j++)
    	  	CGE (GRBaddqpterms(model,1,&i,&j,&(Q(i,j))), env);
  	  startrow=startrow+d;
  	}
  	if (startrow>nparams) {
  		cerr << "Q is too big!" << endl;
  		return NULL;
  	}
  }

  CGE (GRBsetdblattrarray(model,"Obj",0,nparams,f.data()), env);

  CGE (myGRBaddconstrs(model,Aeq,beq,GRB_EQUAL, 1e-10), env);
  CGE (myGRBaddconstrs(model,Ain,bin,GRB_LESS_EQUAL, 1e-10), env);

  CGE (GRBupdatemodel(model), env);
  CGE (GRBoptimize(model), env);

  CGE (GRBgetdblattrarray(model, GRB_DBL_ATTR_X, 0, nparams, x.data()), env);

  // todo: populate active set

  return model;
}


template int fastQP(vector< Map<MatrixXd> > QblkDiag, const MatrixBase< Map<VectorXd> >&, const MatrixBase< Map<MatrixXd> >&, const MatrixBase< Map<VectorXd> >&, const MatrixBase< Map<MatrixXd> >&, const MatrixBase< Map<VectorXd> >&, set<int>&, MatrixBase< Map<VectorXd> >&);
template GRBmodel* gurobiQP(GRBenv *env, vector< Map<MatrixXd> > QblkDiag, VectorXd& f, const MatrixBase< Map<MatrixXd> >& Aeq, const MatrixBase< Map<VectorXd> >& beq, const MatrixBase< Map<MatrixXd> >& Ain, const MatrixBase< Map<VectorXd> >&bin, VectorXd& lb, VectorXd& ub, set<int>&, VectorXd&);

/*
template int fastQP(vector< MatrixBase< VectorXd > >, const MatrixBase< VectorXd >&, const MatrixBase< Matrix<double,-1,-1,RowMajor,1000,-1> >&, const MatrixBase< Matrix<double,-1,1,0,1000,1> >&, const MatrixBase< Matrix<double,-1,-1,RowMajor,1000,-1> >&, const MatrixBase< Matrix<double,-1,1,0,1000,1> >&, set<int>&, MatrixBase< VectorXd >&);
*/



