
#include "Optimization.h"

#include <cstdlib>
#include <memory>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iostream>

namespace snopt {
#include "snopt.hh"
#include "snfilewrapper.hh"
//#include "snoptProblem.hh"
}

// todo:  implement objectives
// todo:  implement nonlinear constraints
// todo:  implement linear inequality constraints
// todo:  special case bounding box constraints
// todo:  handle initial conditions
// todo:  handle snopt options
// todo:  return more information that just the solution (INFO, infeasible constraints, ...)

bool Drake::OptimizationProblem::NonlinearProgram::hasSNOPT() const { return true; }

using namespace std;
using namespace Eigen;
using namespace Drake;

// NOTE: all snopt calls will use this shared memory... so this code is NOT THREAD SAFE
const Drake::OptimizationProblem* current_problem = NULL;
static unique_ptr<snopt::doublereal []> rw;
static unique_ptr<snopt::integer []> iw;
static unique_ptr<char []> cw;
static snopt::integer lenrw=0;
static snopt::integer leniw=0;
static snopt::integer lencw=0;

static int snopt_userfun(snopt::integer *Status, snopt::integer *n, snopt::doublereal x[],
                         snopt::integer *needF, snopt::integer *neF, snopt::doublereal F[],
                         snopt::integer *needG, snopt::integer *neG, snopt::doublereal G[],
                         char *cu, snopt::integer *lencu,
                         snopt::integer iu[], snopt::integer *leniu,
                         snopt::doublereal ru[], snopt::integer *lenru)
{
  snopt::integer i;
  VectorXd xvec(*n);
  for(i=0; i<*n; i++) { xvec(i) = static_cast<double>(x[i]); }

//  cout << "In snopt user fun" << endl;

  // evaluate objective
  auto tx = initTaylorVecXd(xvec);
  TaylorVecXd ty(1);
  for (auto const& obj : current_problem->getGenericObjectives() ) {
    obj->eval(tx,ty);
    F[0] += ty(0).value();
    for (i=0; i<*n; i++) { G[i] = ty(0).derivatives()(i); }
  }

  // todo: evaluate the nonlinear constraints and populate F[] and G[] here
  for (i=1; i<*neF; i++) { F[i]=0.0; }
  for (i=*n; i<*neG; i++) { G[i]=0.0; }

  return 0;
}

void mysnseti(const char* strOpt,snopt::integer val,snopt::integer* iPrint, snopt::integer* iSumm, snopt::integer* INFO_snopt, char* cw, snopt::integer *lencw, snopt::integer* iw, snopt::integer *leniw, snopt::doublereal* rw, snopt::integer *lenrw)
{
  snopt::integer strOpt_len = static_cast<snopt::integer>(strlen(strOpt));
  snopt::snseti_(strOpt,&val,iPrint,iSumm,INFO_snopt,cw,lencw,iw,leniw,rw,lenrw,strOpt_len,8*(*lencw));
}
void mysnsetr(const char* strOpt,snopt::doublereal val,snopt::integer* iPrint, snopt::integer* iSumm, snopt::integer* INFO_snopt, char* cw, snopt::integer *lencw, snopt::integer* iw, snopt::integer *leniw, snopt::doublereal* rw, snopt::integer *lenrw)
{
  snopt::integer strOpt_len = static_cast<snopt::integer>(strlen(strOpt));
  snopt::snsetr_(strOpt,&val,iPrint,iSumm,INFO_snopt,cw,lencw,iw,leniw,rw,lenrw,strOpt_len,8*(*lencw));
}

bool Drake::OptimizationProblem::NonlinearProgram::solveWithSNOPT(OptimizationProblem &prog) const {
  current_problem = &prog;

  if (lenrw==0) { // then initialize (sninit needs some default allocation)
    lenrw = 500000;
    rw.reset(new snopt::doublereal[lenrw]);
    leniw = 500000;
    iw.reset(new snopt::integer[leniw]);
    lencw = 500;
    cw.reset(new char[8*lencw]);
  }

  snopt::integer nx = prog.num_vars;
  snopt::doublereal* x    = new snopt::doublereal[nx];
  snopt::doublereal* xlow = new snopt::doublereal[nx];
  snopt::doublereal* xupp = new snopt::doublereal[nx];
  for(int i=0;i<nx;i++)
  {
    x[i] = static_cast<snopt::doublereal>(0.0);  // todo: pull from initial conditions (possibly w/ memcpy?)
    xlow[i] = static_cast<snopt::doublereal>(-numeric_limits<double>::infinity());
    xupp[i] = static_cast<snopt::doublereal>(numeric_limits<double>::infinity());
  }

  size_t num_nonlinear_constraints=0, max_num_gradients=0;
  for (auto const& c : prog.generic_constraints) {
    size_t n = c->getNumConstraints();
    for (const DecisionVariableView& v : c->getVariableList() ) {
      max_num_gradients += n*v.size();
    }
    num_nonlinear_constraints += n;
  }
  size_t num_linear_constraints=0;
  for (auto const& c : prog.linear_equality_constraints) { num_linear_constraints += c->getNumConstraints(); }

  snopt::integer nF = 1+num_nonlinear_constraints+num_linear_constraints;
  snopt::doublereal* Flow = new snopt::doublereal[nF];
  snopt::doublereal* Fupp = new snopt::doublereal[nF];

  snopt::integer lenG = static_cast<snopt::integer>(max_num_gradients);
  snopt::integer* iGfun = new snopt::integer[lenG];
  snopt::integer* jGvar = new snopt::integer[lenG];

  size_t constraint_index=1, grad_index=0;  // constraint index starts at 1 because the objective is the first row
  for (auto const& c : prog.generic_constraints) {
    size_t n = c->getNumConstraints();

    auto const & lb = c->getLowerBound(), ub = c->getUpperBound();
    for (int i=0; i<n; i++) {
      Flow[constraint_index+i] = static_cast<snopt::doublereal>(lb[i]);
      Fupp[constraint_index+i] = static_cast<snopt::doublereal>(ub[i]);
    }

    for (const DecisionVariableView& v : c->getVariableList() ) {
      for (size_t i=0; i<n; i++) {
        for (size_t j = 0; j < v.size(); j++) {
          iGfun[grad_index] = constraint_index + i + 1;  // column order
          jGvar[grad_index] = v.index() + j + 1;
          grad_index++;
        }
      }
    }
    constraint_index += n;
  }

  // http://eigen.tuxfamily.org/dox/group__TutorialSparse.html
  typedef Eigen::Triplet<double> T;
  std::vector<T> tripletList;
  tripletList.reserve(num_linear_constraints*prog.num_vars);

  size_t linear_constraint_index=0;
  for (auto const& c : prog.linear_equality_constraints) {
    size_t n = c->getNumConstraints();
    size_t var_index = 0;
    SparseMatrix<double> A_constraint = c->getSparseMatrix();
    for (const DecisionVariableView& v : c->getVariableList() ) {
      for (size_t k=0; k<v.size(); ++k) {
        for (SparseMatrix<double>::InnerIterator it(A_constraint, var_index+k); it; ++it) {
          tripletList.push_back(T(linear_constraint_index+it.col(),v.index()+k,it.value()));
//          cout << "A(" << linear_constraint_index+it.col() << "," << v.index()+k << ") = " << it.value() << endl;
        }
      }
      var_index += v.size();
    }

    auto const & b = c->getVector();
    for (int i=0; i<n; i++) {
      Flow[constraint_index+i] = static_cast<snopt::doublereal>(b[i]);
      Fupp[constraint_index+i] = static_cast<snopt::doublereal>(b[i]);
    }
    constraint_index += n;
    linear_constraint_index += n;
  }

  snopt::integer lenA = static_cast<snopt::integer>(tripletList.size());
  snopt::doublereal* A     = new snopt::doublereal[lenA];
  snopt::integer*    iAfun = new snopt::integer[lenA];
  snopt::integer*    jAvar = new snopt::integer[lenA];
  size_t A_index = 0;
  for (auto const & it : tripletList) {
    A[A_index] = it.value();
    iAfun[A_index] = 1+num_nonlinear_constraints+it.row()+1;
    jAvar[A_index] = it.col()+1;
    A_index++;
  }

  snopt::integer INFO_snopt;
  snopt::integer nxname = 1, nFname = 1, npname = 0;
  char xnames[8*1];  // should match nxname
  char Fnames[8*1];  // should match nFname
  char Prob[200]="drake.out";

  snopt::integer iSumm = -1;
  snopt::integer iPrint = -1;

  snopt::integer nS,nInf;
  snopt::doublereal sInf;
  snopt::integer print_file_name_len = static_cast<snopt::integer>(1);
  const char* print_file_name = NULL;
  if(print_file_name_len != 1) { // todo: handle print file logic
/*
    iPrint = 9;
    print_file_name = new char[print_file_name_len];
    mxGetString(pprint_name,print_file_name,print_file_name_len);
    snopt::snopenappend_(&iPrint,print_file_name,&INFO_snopt,print_file_name_len);
*/
    //mysnseti("Major print level",static_cast<snopt::integer>(11),&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw);
    //mysnseti("Print file",iPrint,&iPrint,&iSumm,&INFO_snopt,cw,&lencw,iw,&leniw,rw,&lenrw);
  }

  snopt::integer minrw,miniw,mincw;
  snopt::sninit_(&iPrint,&iSumm,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw,8*lencw);
  snopt::snmema_(&INFO_snopt, &nF, &nx, &nxname, &nFname, &lenA, &lenG, &mincw, &miniw, &minrw, cw.get(), &lencw, iw.get(), &leniw, rw.get(), &lenrw, 8 * lencw);
  if (minrw>lenrw) {
    //mexPrintf("reallocation rw with size %d\n",minrw);
    lenrw = minrw;
    rw.reset(new snopt::doublereal[lenrw]);
  }
  if (miniw>leniw) {
    //mexPrintf("reallocation iw with size %d\n",miniw);
    leniw = miniw;
    iw.reset(new snopt::integer[leniw]);
  }
  if (mincw>lencw) {
    //mexPrintf("reallocation cw with size %d\n",mincw);
    lencw = mincw;
    cw.reset(new char[8*lencw]);
  }
  snopt::sninit_(&iPrint,&iSumm,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw,8*lencw);

  snopt::integer Cold = 0;
  snopt::doublereal *xmul = new snopt::doublereal[nx];
  snopt::integer *xstate = new snopt::integer[nx];
  memset(xstate,0,sizeof(snopt::integer)*nx);

  snopt::doublereal *F      = new snopt::doublereal[nF];
  snopt::doublereal *Fmul   = new snopt::doublereal[nF];
  snopt::integer    *Fstate = new snopt::integer[nF];
  memset(Fstate,0,sizeof(snopt::integer)*nF);

  snopt::doublereal ObjAdd = 0.0;
  snopt::integer ObjRow = 1; // feasibility problem (for now)

/*
  mysnseti("Derivative option",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"DerivativeOption"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
  mysnseti("Major iterations limit",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"MajorIterationsLimit"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
  mysnseti("Minor iterations limit",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"MinorIterationsLimit"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
  mysnsetr("Major optimality tolerance",static_cast<snopt::doublereal>(*mxGetPr(mxGetField(prhs[13],0,"MajorOptimalityTolerance"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
  mysnsetr("Major feasibility tolerance",static_cast<snopt::doublereal>(*mxGetPr(mxGetField(prhs[13],0,"MajorFeasibilityTolerance"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
  mysnsetr("Minor feasibility tolerance",static_cast<snopt::doublereal>(*mxGetPr(mxGetField(prhs[13],0,"MinorFeasibilityTolerance"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
  mysnseti("Superbasics limit",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"SuperbasicsLimit"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
  mysnseti("Verify level",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"VerifyLevel"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
  mysnseti("Iterations Limit",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"IterationsLimit"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
  mysnseti("Scale option",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"ScaleOption"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
  mysnseti("New basis file",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"NewBasisFile"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
  mysnseti("Old basis file",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"OldBasisFile"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
  mysnseti("Backup basis file",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"BackupBasisFile"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
  mysnsetr("Linesearch tolerance",static_cast<snopt::doublereal>(*mxGetPr(mxGetField(prhs[13],0,"LinesearchTolerance"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
*/

  snopt::snopta_
          (&Cold, &nF, &nx, &nxname, &nFname,
           &ObjAdd, &ObjRow, Prob, snopt_userfun,
           iAfun, jAvar, &lenA, &lenA, A,
           iGfun, jGvar, &lenG, &lenG,
           xlow, xupp, xnames, Flow, Fupp, Fnames,
           x, xstate, xmul, F, Fstate, Fmul,
           &INFO_snopt, &mincw, &miniw, &minrw,
           &nS, &nInf, &sInf,
           cw.get(), &lencw, iw.get(), &leniw, rw.get(), &lenrw,
           cw.get(), &lencw, iw.get(), &leniw, rw.get(), &lenrw,
           npname, 8*nxname, 8*nFname,
            8*lencw,8*lencw);

  cout << "SNOPT INFO: " << INFO_snopt << endl;

  VectorXd sol(nx);
  for (int i=0; i<nx; i++) { sol(i) = static_cast<double>( x[i] ); }
  prog.setDecisionVariableValues(sol);
//  prog.printSolution();

  // todo: extract the other useful quantities, too.

  delete[] x;
  delete[] xlow;
  delete[] xupp;
  delete[] Flow;
  delete[] Fupp;
  delete[] A;
  delete[] iAfun;
  delete[] jAvar;
  delete[] iGfun;
  delete[] jGvar;
  delete[] xmul;
  delete[] xstate;
  delete[] F;
  delete[] Fmul;
  delete[] Fstate;

  if(print_file_name_len!= 1) {
    snopt::snclose_(&iPrint);
    delete[] print_file_name;
  }

  return true;
}

