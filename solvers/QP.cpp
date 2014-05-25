#include <math.h>
#include <iostream>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/SVD>

#include "fastQP.h"
#include "gurobiQP.h"

#define _USE_MATH_DEFINES

#define MAX_CONSTRS 1000
#define MAX_STATE   1000
#define MAX_ITER    10


using namespace Eigen;
using namespace std;


//template <typename tA, typename tB, typename tC, typename tD, typename tE, typename tF, typename tG>
//int fastQPThatTakesQinv(vector< MatrixBase<tA>* > QinvblkDiag, const MatrixBase<tB>& f, const MatrixBase<tC>& Aeq, const MatrixBase<tD>& beq, const MatrixBase<tE>& Ain, const MatrixBase<tF>& bin, set<int>& active, MatrixBase<tG>& x)
int fastQPThatTakesQinv(vector< MatrixXd* > QinvblkDiag, const VectorXd& f, const MatrixXd& Aeq, const VectorXd& beq, const MatrixXd& Ain, const VectorXd& bin, set<int>& active, VectorXd& x)
{
  int i,d;
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

  // calculate a bunch of stuff that is constant during each iteration
  int startrow=0;
//  for (typename vector< MatrixBase<tA>* >::iterator iterQinv=QinvblkDiag.begin(); iterQinv!=QinvblkDiag.end(); iterQinv++) {
//  	MatrixBase<tA> *thisQinv = *iterQinv;
  for (vector< MatrixXd* >::iterator iterQinv=QinvblkDiag.begin(); iterQinv!=QinvblkDiag.end(); iterQinv++) {
  	MatrixXd *thisQinv = *iterQinv;
  	int numRow = thisQinv->rows();
  	int numCol = thisQinv->cols();

  	if (numRow == 1 || numCol == 1) {  // it's a vector
  		d = numRow*numCol;
  		if (M>0) QinvAteq.block(startrow,0,d,M)= thisQinv->asDiagonal()*Aeq.block(0,startrow,M,d).transpose();  // Aeq.transpoODse().block(startrow,0,d,N)
			minusQinvf.segment(startrow,d) = -thisQinv->cwiseProduct(f.segment(startrow,d));
			startrow=startrow+d;
  	} else { // potentially dense matrix
			d = numRow;
			if (numRow!=numCol) {
					cerr << "Q is not square! " << numRow << "x" << numCol << "\n";
					return -2;
			}
			if (M>0) QinvAteq.block(startrow,0,d,M) = thisQinv->operator*(Aeq.block(0,startrow,M,d).transpose());  // Aeq.transpose().block(startrow,0,d,N)
			minusQinvf.segment(startrow,d) = -thisQinv->operator*(f.segment(startrow,d));
			startrow=startrow+d;
		}
  	if (startrow>N) {
			cerr << "Q is too big!" << endl;
			return -2;
		}
  }
  if (startrow!=N) { cerr << "Q is the wrong size.  Got " << startrow << "by" << startrow << " but needed " << N << "by" << N << endl; return -2; }

  MatrixXd A;
  VectorXd b;
  MatrixXd QinvAt;
  VectorXd lam, lamIneq;
  VectorXd violated(M_in);
  VectorXd violation;
  
  while(1) {
    iterCnt++;

    n_active = active.size();
    Aact.resize(n_active,N);
    bact.resize(n_active);

    i=0;
    for (set<int>::iterator iter=active.begin(); iter!=active.end(); iter++) {
    	if (*iter<0 || *iter>=Ain.rows()) {
    		return -3;  // active set is invalid.  exit quietly, because this is expected behavior in normal operation (e.g. it means I should immediately kick out to gurobi)
    	}
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
				for (vector< MatrixXd* >::iterator iterQinv=QinvblkDiag.begin(); iterQinv!=QinvblkDiag.end(); iterQinv++) {
					MatrixXd* thisQinv = (*iterQinv);
					d = thisQinv->rows();
					int numCol = thisQinv->cols();

					if (numCol == 1) {  // it's a vector
						QinvAt.block(startrow,0,d,M+n_active) << QinvAteq.block(startrow,0,d,M), thisQinv->asDiagonal()*Aact.block(0,startrow,n_active,d).transpose();
					} else { // it's a matrix
						QinvAt.block(startrow,0,d,M+n_active) << QinvAteq.block(startrow,0,d,M), thisQinv->operator*(Aact.block(0,startrow,n_active,d).transpose());
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

    if (iterCnt > MAX_ITER) {
      //Default to calling this method
//      cout << "FastQP max iter reached." << endl;
//       mexErrMsgIdAndTxt("Drake:approximateIKmex:Error", "Max iter reached. Problem is likely infeasible");
      return -1;
    }
  }  
  return iterCnt;
}

//template <typename tA, typename tB, typename tC, typename tD, typename tE, typename tF, typename tG>
//int fastQP(vector< MatrixBase<tA>* > QblkDiag, const MatrixBase<tB>& f, const MatrixBase<tC>& Aeq, const MatrixBase<tD>& beq, const MatrixBase<tE>& Ain, const MatrixBase<tF>& bin, set<int>& active, MatrixBase<tG>& x)
int fastQP(vector< MatrixXd* > QblkDiag, const VectorXd& f, const MatrixXd& Aeq, const VectorXd& beq, const MatrixXd& Ain, const VectorXd& bin, set<int>& active, VectorXd& x)
{
  /* min 1/2 * x'QblkDiag'x + f'x s.t A x = b, Ain x <= bin
   * using active set method.  Iterative solve a linearly constrained
   * quadratic minimization problem where linear constraints include
   * Ain(active,:)x == bin(active).  Quit if all dual variables associated
   * with these equations are positive (i.e. they satisfy KKT conditions).
   *
   * Note:
   * fails if QP is infeasible.
   * active == initial rows of Ain to treat as equations.
   * Frank Permenter - June 6th 2013
   *
   * @retval  if feasible then iterCnt, else -1 for infeasible, -2 for input error
   */

	int N = f.rows();

  MatrixXd* Qinv = new MatrixXd[QblkDiag.size()];
  vector< MatrixXd* > Qinvmap;

	#define REG 1e-13
  // calculate a bunch of stuff that is constant during each iteration
  int startrow=0;
  //typedef typename vector< MatrixBase<tA> >::iterator Qiterator;

  int i=0;
  for (vector< MatrixXd* >::iterator iterQ=QblkDiag.begin(); iterQ!=QblkDiag.end(); iterQ++) {
  	MatrixXd* thisQ = *iterQ;
  	int numRow = thisQ->rows();
  	int numCol = thisQ->cols();

  	if (numCol == 1) {  // it's a vector
  		VectorXd Qdiag_mod = thisQ->operator+(VectorXd::Constant(numRow,REG)); // regularize
  		Qinv[i] = Qdiag_mod.cwiseInverse();
  		Qinvmap.push_back( &Qinv[i] );
  		startrow=startrow+numRow;
		} else { // potentially dense matrix
			if (numRow!=numCol) {
				if (numRow==1)
					cerr << "diagonal Q's must be set as column vectors" << endl;
				else
					cerr << "Q is not square! " << numRow << "x" << numCol << endl;
				return -2;
			}

			MatrixXd Q_mod = thisQ->operator+(REG*MatrixXd::Identity(numRow,numRow));
			Qinv[i] = Q_mod.inverse();
  		Qinvmap.push_back( &Qinv[i] );
  		startrow=startrow+numRow;
		}
//  	cout << "Qinv{" << i << "} = " << Qinv[i] << endl;
		if (startrow>N) {
			cerr << "Q is too big!" << endl;
			return -2;
		}
		i++;
  }
  if (startrow!=N) { cerr << "Q is the wrong size.  Got " << startrow << "by" << startrow << " but needed " << N << "by" << N << endl; return -2; }

  int info = fastQPThatTakesQinv(Qinvmap,f,Aeq,beq,Ain,bin,active,x);

  delete[] Qinv;
  return info;
}

/* Example call (allocate inequality matrix, call function, resize inequalites:
  VectorXd binBnd = VectorXd(2*N);
  AinBnd.setZero();
  int numIneq = boundToIneq(ub,lb,AinBnd,binBnd);
  AinBnd.resize(numIneq,N);
  binBnd.resize(numIneq);
*/
/*
int boundToIneq(const VectorXd& uB,const VectorXd& lB, MatrixXd& Ain, VectorXd& bin)
{
    int rCnt = 0;
    int cCnt = 0;

    if (uB.rows()+lB.rows() > A.rows() ) {
        cerr << "not enough memory allocated";
    }

    if (uB.rows()+lB.rows() > b.rows() ) {
        cerr << "not enough memory allocated";
    }

    for (int i = 0; i < lB.rows(); i++ ) {
        if (!isinf(lB(i))) {
            cout << lB(i);
            cout << i;
            Ain(rCnt,cCnt++) = -1;//lB(i);
            bin(rCnt++) = -lB(i);
        }
    }
    cCnt = 0;
    for (int i = 0; i < uB.rows(); i++ ) {
        if (!isinf(uB(i))) {
            Ain(rCnt,cCnt++) = 1;//uB(i);
            bin(rCnt++) = uB(i);
        }
    }

    //resizing inside function all causes exception (why??)
    //A.resize(rCnt,uB.rows());
    return rCnt;
}
*/



template <typename DerivedA,typename DerivedB>
int myGRBaddconstrs(GRBmodel *model, MatrixBase<DerivedA> const & A, MatrixBase<DerivedB> const & b, char sense, double sparseness_threshold = 1e-14)
{
  int i,j,nnz,error=0;
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

//template <typename tA, typename tB, typename tC, typename tD, typename tE>
//GRBmodel* gurobiQP(GRBenv *env, vector< MatrixBase<tA>* > QblkDiag, VectorXd& f, const MatrixBase<tB>& Aeq, const MatrixBase<tC>& beq, const MatrixBase<tD>& Ain, const MatrixBase<tE>& bin, VectorXd& lb, VectorXd& ub, set<int>& active, VectorXd& x)
GRBmodel* gurobiQP(GRBenv *env, vector< MatrixXd* > QblkDiag, VectorXd& f, const MatrixXd& Aeq, const VectorXd& beq, 
  const MatrixXd& Ain, const VectorXd& bin, VectorXd& lb, VectorXd& ub, set<int>& active, VectorXd& x, double active_set_slack_tolerance)
{
	// Note: f,lb, and ub are VectorXd instead of const MatrixBase templates because i want to be able to call f.data() on them

	// NOTE:  this allocates memory for a new GRBmodel and returns it. (you should delete this object when you're done with it)
	// NOTE:  by convention here, the active set indices correspond to Ain,bin first, then lb, then ub.

  GRBmodel *model = NULL;

  int method;  GRBgetintparam(env,"method",&method);

  int i,j,nparams = f.rows(),Qi,Qj;
  double *lbdata = NULL, *ubdata=NULL;
  if (lb.rows()==nparams) lbdata = lb.data();
  if (ub.rows()==nparams) ubdata = ub.data();
  CGE (GRBnewmodel(env,&model,"QP",nparams,NULL,lbdata,ubdata,NULL,NULL), env);

  int startrow=0,d;
  for (vector< MatrixXd* >::iterator iterQ=QblkDiag.begin(); iterQ!=QblkDiag.end(); iterQ++) {
  	MatrixXd* Q=*iterQ;
    
    // WARNING:  If there are no constraints, then gurobi clearly solves a different problem: min 1/2 x'Qx + f'x
    //  				 This is very strange; see the solveWGUROBI method in QuadraticProgram
    if (method==2) //&& (Aeq.rows()+Ain.rows()>0))
      *Q = .5* (*Q);
    
  	if (Q->rows() == 1 || Q->cols() == 1) {  // it's a vector
  		d = Q->rows()*Q->cols();
  	  for (i=0; i<d; i++) {
  	  	Qi=i+startrow;
  	  	CGE (GRBaddqpterms(model,1,&Qi,&Qi,&(Q->operator()(i))), env);
  	  }
  	  startrow=startrow+d;
  	} else { // potentially dense matrix
  		d = Q->rows();
  		if (d!=Q->cols()) {
  			cerr << "Q is not square! " << Q->rows() << "x" << Q->cols() << "\n";
  			return NULL;
  		}

  	  for (i=0; i<d; i++)
    	  for (j=0; j<d; j++) {
    	  	Qi=i+startrow; Qj = j+startrow;
    	  	CGE (GRBaddqpterms(model,1,&Qi,&Qj,&(Q->operator()(i,j))), env);
    	  }
  	  startrow=startrow+d;
  	}
  	if (startrow>nparams) {
  		cerr << "Q is too big!" << endl;
  		return NULL;
  	}
  }

  CGE (GRBsetdblattrarray(model,"Obj",0,nparams,f.data()), env);

  if (Aeq.rows()>0) CGE (myGRBaddconstrs(model,Aeq,beq,GRB_EQUAL, 1e-18), env);
  if (Ain.rows()>0) CGE (myGRBaddconstrs(model,Ain,bin,GRB_LESS_EQUAL, 1e-18), env);

  CGE (GRBupdatemodel(model), env);
  CGE (GRBoptimize(model), env);

  CGE (GRBgetdblattrarray(model, GRB_DBL_ATTR_X, 0, nparams, x.data()), env);

  VectorXd slack(Ain.rows());
  CGE (GRBgetdblattrarray(model, "Slack", Aeq.rows(), Ain.rows(), slack.data()), env);

  int offset=0;
  active.clear();
  for (int i=0; i<Ain.rows(); i++) {
  	if (slack(i)<active_set_slack_tolerance)
  		active.insert(i);
  }
  offset = Ain.rows();
  if (lb.rows()==nparams)
  	for (int i=0; i<nparams; i++)
  		if (x(i)-lb(i) < active_set_slack_tolerance)
  			active.insert(offset+i);
  if (ub.rows()==nparams)
  	for (int i=0; i<nparams; i++)
  		if (ub(i)-x(i) < active_set_slack_tolerance)
  			active.insert(offset+i+nparams);

  return model;
}


GRBmodel* gurobiActiveSetQP(GRBenv *env, vector< MatrixXd* > QblkDiag, VectorXd& f, const MatrixXd& Aeq, const VectorXd& beq, 
  const MatrixXd& Ain, const VectorXd& bin, VectorXd& lb, VectorXd& ub, int* &vbasis, int vbasis_len, int* &cbasis, int cbasis_len, VectorXd& x)
{
  // NOTE:  this allocates memory for a new GRBmodel and returns it. (you should delete this object when you're done with it)
  // NOTE:  by convention here, the active set indices correspond to Ain,bin first, then lb, then ub.
  GRBmodel *model = NULL;

  int method;  GRBgetintparam(env,"method",&method);
  if (!(method==0 || method==1)) {
    cerr<< "gurobiActiveSetQP: method should be 0 or 1" << endl;
    return NULL;
  }

  int i,j,nparams = f.rows(),Qi,Qj;
  double *lbdata = NULL, *ubdata=NULL;
  if (lb.rows()==nparams) lbdata = lb.data();
  if (ub.rows()==nparams) ubdata = ub.data();
  CGE (GRBnewmodel(env,&model,"QP",nparams,NULL,lbdata,ubdata,NULL,NULL), env);

  int startrow=0,d;
  for (vector< MatrixXd* >::iterator iterQ=QblkDiag.begin(); iterQ!=QblkDiag.end(); iterQ++) {
    MatrixXd* Q=*iterQ;
    
	*Q = .5* (*Q);
    
    if (Q->rows() == 1 || Q->cols() == 1) {  // it's a vector
      d = Q->rows()*Q->cols();
      for (i=0; i<d; i++) {
        Qi=i+startrow;
        CGE (GRBaddqpterms(model,1,&Qi,&Qi,&(Q->operator()(i))), env);
      }
      startrow=startrow+d;
    } else { // potentially dense matrix
      d = Q->rows();
      if (d!=Q->cols()) {
        cerr << "Q is not square! " << Q->rows() << "x" << Q->cols() << "\n";
        return NULL;
      }

      for (i=0; i<d; i++)
        for (j=0; j<d; j++) {
          Qi=i+startrow; Qj = j+startrow;
          CGE (GRBaddqpterms(model,1,&Qi,&Qj,&(Q->operator()(i,j))), env);
        }
      startrow=startrow+d;
    }
    if (startrow>nparams) {
      cerr << "Q is too big!" << endl;
      return NULL;
    }
  }

  CGE (GRBsetdblattrarray(model,"Obj",0,nparams,f.data()), env);

  if (Aeq.rows()>0) CGE (myGRBaddconstrs(model,Aeq,beq,GRB_EQUAL, 1e-18), env);
  if (Ain.rows()>0) CGE (myGRBaddconstrs(model,Ain,bin,GRB_LESS_EQUAL, 1e-18), env);

  CGE (GRBupdatemodel(model), env);

  int numvars; CGE(GRBgetintattr(model,"NumVars",&numvars), env);
  if (numvars == vbasis_len) {
    CGE (GRBsetintattrarray(model, "VBasis", 0, numvars, vbasis), env);
  }
  else {
    delete[] vbasis;
    vbasis = new int[numvars];
  }

  int numconstr; CGE(GRBgetintattr(model,"NumConstrs",&numconstr),env);
  if (numconstr == cbasis_len) {
    CGE (GRBsetintattrarray(model, "CBasis", 0, numconstr, cbasis), env);
  }
  else {
    delete[] cbasis;
    cbasis = new int[numconstr];
  }
  CGE (GRBoptimize(model), env);

  CGE (GRBgetdblattrarray(model, GRB_DBL_ATTR_X, 0, nparams, x.data()), env);

  CGE (GRBgetintattrarray(model, "VBasis", 0, numvars, vbasis), env);
  CGE (GRBgetintattrarray(model, "CBasis", 0, numconstr, cbasis), env);

  return model;
}



/*
template int fastQP(vector< MatrixBase<MatrixXd>* > QblkDiag, const MatrixBase< Map<VectorXd> >&, const MatrixBase< Map<MatrixXd> >&, const MatrixBase< Map<VectorXd> >&, const MatrixBase< Map<MatrixXd> >&, const MatrixBase< Map<VectorXd> >&, set<int>&, MatrixBase< Map<VectorXd> >&);
template GRBmodel* gurobiQP(GRBenv *env, vector< MatrixBase<MatrixXd>* > QblkDiag, VectorXd& f, const MatrixBase< Map<MatrixXd> >& Aeq, const MatrixBase< Map<VectorXd> >& beq, const MatrixBase< Map<MatrixXd> >& Ain, const MatrixBase< Map<VectorXd> >&bin, VectorXd& lb, VectorXd& ub, set<int>&, VectorXd&);
template GRBmodel* gurobiQP(GRBenv *env, vector< MatrixBase<MatrixXd>* > QblkDiag, VectorXd& f, const MatrixBase< MatrixXd >& Aeq, const MatrixBase< VectorXd >& beq, const MatrixBase< MatrixXd >& Ain, const MatrixBase< VectorXd >&bin, VectorXd&lb, VectorXd&ub, set<int>&, VectorXd&);
*/

/*
template int fastQP(vector< MatrixBase< VectorXd > >, const MatrixBase< VectorXd >&, const MatrixBase< Matrix<double,-1,-1,RowMajor,1000,-1> >&, const MatrixBase< Matrix<double,-1,1,0,1000,1> >&, const MatrixBase< Matrix<double,-1,-1,RowMajor,1000,-1> >&, const MatrixBase< Matrix<double,-1,1,0,1000,1> >&, set<int>&, MatrixBase< VectorXd >&);
*/



