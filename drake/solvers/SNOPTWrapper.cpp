
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

// todo:  implement sparsity inside each objective/constraint
// todo:  handle snopt options
// todo:  return more information that just the solution (INFO, infeasible constraints, ...)
// todo:  avoid all dynamic allocation

bool Drake::OptimizationProblem::NonlinearProgram::hasSNOPT() const { return true; }

using namespace std;
using namespace Eigen;
using namespace Drake;

// NOTE: all snopt calls will use this shared memory... so this code is NOT THREAD SAFE
const Drake::OptimizationProblem* current_problem = NULL;

struct SNOPTData: public Drake::OptimizationProblem::SolverData {
  unique_ptr<snopt::doublereal []> rw;
  unique_ptr<snopt::integer []> iw;
  unique_ptr<char []> cw;
  snopt::integer lenrw=0;
  snopt::integer leniw=0;
  snopt::integer lencw=0;
};

struct SNOPTRun {
  SNOPTRun(SNOPTData& d): D(d) {}
  ~SNOPTRun() {
    if (iPrint >= 0) {
      snopt::snclose_(&iPrint);
    }
  }

  SNOPTData& D;

  snopt::integer iPrint=-1;
  snopt::integer iSumm=-1;

  snopt::integer snSeti(std::string const& opt, snopt::integer val) {
    snopt::integer opt_len = static_cast<snopt::integer>(opt.length());
    snopt::integer err = 0;
    snopt::snseti_(opt.c_str(), &val, &iPrint, &iSumm, &err,
                   D.cw.get(), &D.lencw,
                   D.iw.get(), &D.leniw,
                   D.rw.get(), &D.lenrw,
                   opt_len, 8*D.lencw);
    return err;
  }

  snopt::integer snSetr(std::string const& opt, snopt::doublereal val) {
    snopt::integer opt_len = static_cast<snopt::integer>(opt.length());
    snopt::integer err = 0;
    snopt::snsetr_(opt.c_str(), &val, &iPrint, &iSumm, &err,
                   D.cw.get(), &D.lencw,
                   D.iw.get(), &D.leniw,
                   D.rw.get(), &D.lenrw,
                   opt_len, 8*D.lencw);
    return err;
  }

  void snInit() {
    snopt::sninit_(&iPrint, &iSumm,
                   D.cw.get(), &D.lencw,
                   D.iw.get(), &D.leniw,
                   D.rw.get(), &D.lenrw,
                   8*D.lencw);
  }

  snopt::integer snMemA(snopt::integer nF, snopt::integer nx, snopt::integer nxname,
                        snopt::integer nFname, snopt::integer neA, snopt::integer neG,
                        snopt::integer* mincw, snopt::integer* miniw, snopt::integer* minrw) {
    snopt::integer info = 0;
    snopt::snmema_(&info, &nF, &nx, &nxname, &nFname, &neA, &neG,
                   mincw, miniw, minrw,
                   D.cw.get(), &D.lencw,
                   D.iw.get(), &D.leniw,
                   D.rw.get(), &D.lenrw,
                   8*D.lencw);
    return info;
  }
};

static int snopt_userfun(snopt::integer *Status, snopt::integer *n, snopt::doublereal x[],
                         snopt::integer *needF, snopt::integer *neF, snopt::doublereal F[],
                         snopt::integer *needG, snopt::integer *neG, snopt::doublereal G[],
                         char *cu, snopt::integer *lencu,
                         snopt::integer iu[], snopt::integer *leniu,
                         snopt::doublereal ru[], snopt::integer *lenru)
{
  size_t constraint_index=0, grad_index=0;  // constraint index starts at 1 because the objective is the first row
  snopt::integer i;
  VectorXd xvec(*n);
  for(i=0; i<*n; i++) { xvec(i) = static_cast<double>(x[i]); }

//  cout << "In snopt user fun" << endl;
  F[0] = 0.0;
  memset(G,0,sizeof(*n)*sizeof(snopt::doublereal));

  // evaluate objective
  auto tx = initTaylorVecXd(xvec);
  TaylorVecXd ty(1), this_x(*n);
  for (auto const& obj : current_problem->getGenericObjectives() ) {
    size_t index=0;
    for (const DecisionVariableView& v : obj->getVariableList() ) {
      this_x.segment(index, v.size()) = tx.segment(v.index(), v.size());
      index += v.size();
    }

    obj->eval(tx,ty);

    F[constraint_index++] += static_cast<snopt::doublereal>(ty(0).value()); // cout << "F = " << F[0] << endl;
    for (const DecisionVariableView& v : obj->getVariableList() ) {
      for (size_t j=v.index();j<v.index()+v.size();j++) {
        G[grad_index+j] += static_cast<snopt::doublereal>(ty(0).derivatives()(j));
//      cout << "G[" << j << "] = " << G[j] << endl;
      }
    }
  }
  grad_index += *n;

  for (auto const& c : current_problem->getGenericConstraints()) {
    size_t index=0, num_constraints = c->getNumConstraints();
    for (const DecisionVariableView& v : c->getVariableList() ) {
      this_x.segment(index, v.size()) = tx.segment(v.index(), v.size());
      index += v.size();
    }

    c->eval(tx,ty);

    for (i=0;i<num_constraints;i++) { F[constraint_index++] = static_cast<snopt::doublereal>(ty(i).value()); }
    for (const DecisionVariableView& v : c->getVariableList() ) {
      for (i=0; i<num_constraints; i++) {
        for (size_t j=v.index(); j<v.index()+v.size(); j++) {
          G[grad_index++] = static_cast<snopt::doublereal>(ty(i).derivatives()(j));
        }
      }
    }
  }

  return 0;
}

bool Drake::OptimizationProblem::NonlinearProgram::solveWithSNOPT(OptimizationProblem &prog) const {
  auto d = prog.getSolverData<SNOPTData>();
  SNOPTRun cur(*d);

  current_problem = &prog;
  if (d->lenrw==0) { // then initialize (sninit needs some default allocation)
    d->lenrw = 500000;
    d->rw.reset(new snopt::doublereal[d->lenrw]);
    d->leniw = 500000;
    d->iw.reset(new snopt::integer[d->leniw]);
    d->lencw = 500;
    d->cw.reset(new char[8*d->lencw]);
  }

  snopt::integer nx = prog.num_vars;
  snopt::doublereal* x    = new snopt::doublereal[nx];
  snopt::doublereal* xlow = new snopt::doublereal[nx];
  snopt::doublereal* xupp = new snopt::doublereal[nx];
  for(int i=0;i<nx;i++)
  {
    x[i] = static_cast<snopt::doublereal>(prog.x_initial_guess(i));
    xlow[i] = static_cast<snopt::doublereal>(-numeric_limits<double>::infinity());
    xupp[i] = static_cast<snopt::doublereal>(numeric_limits<double>::infinity());
  }
  for (auto const& c : prog.bbox_constraints ) {
    for (const DecisionVariableView& v : c->getVariableList() ) {
      auto const lb = c->getLowerBound(), ub = c->getUpperBound();
      for (int k=0; k<v.size(); k++) {
        xlow[v.index()+k] = std::max<snopt::doublereal>(static_cast<snopt::doublereal>(lb(k)),xlow[v.index()+k]);
        xupp[v.index()+k] = std::min<snopt::doublereal>(static_cast<snopt::doublereal>(ub(k)),xupp[v.index()+k]);
      }
    }
  }

  size_t num_nonlinear_constraints=0, max_num_gradients=nx;
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
  Flow[0] = static_cast<snopt::doublereal>(-std::numeric_limits<double>::infinity());
  Fupp[0] = static_cast<snopt::doublereal>(std::numeric_limits<double>::infinity());

  snopt::integer lenG = static_cast<snopt::integer>(max_num_gradients);
  snopt::integer* iGfun = new snopt::integer[lenG];
  snopt::integer* jGvar = new snopt::integer[lenG];
  for (snopt::integer i=0; i<nx; i++) {
    iGfun[i]=1;
    jGvar[i]=i+1;
  }

  size_t constraint_index=1, grad_index=nx;  // constraint index starts at 1 because the objective is the first row
  for (auto const& c : prog.generic_constraints) {
    size_t n = c->getNumConstraints();

    auto const lb = c->getLowerBound(), ub = c->getUpperBound();
    for (int i=0; i<n; i++) {
      Flow[constraint_index+i] = static_cast<snopt::doublereal>(lb(i));
      Fupp[constraint_index+i] = static_cast<snopt::doublereal>(ub(i));
    }

    for (const DecisionVariableView& v : c->getVariableList() ) {
      for (size_t i=0; i<n; i++) {
        for (size_t j = 0; j < v.size(); j++) {
          iGfun[grad_index] = constraint_index + i + 1;  // row order
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

    auto const lb = c->getLowerBound(), ub = c->getUpperBound();
    for (int i=0; i<n; i++) {
      Flow[constraint_index+i] = static_cast<snopt::doublereal>(lb(i));
      Fupp[constraint_index+i] = static_cast<snopt::doublereal>(ub(i));
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

  snopt::integer nxname = 1, nFname = 1, npname = 0;
  char xnames[8*1];  // should match nxname
  char Fnames[8*1];  // should match nFname
  char Prob[200]="drake.out";

  cur.snInit();

  snopt::integer nS,nInf;
  snopt::doublereal sInf;
  if (true) { // print to output file (todo: make this an option)
    cur.iPrint = 9;
    char print_file_name[50] = "snopt.out";
    snopt::integer print_file_name_len = static_cast<snopt::integer>(strlen(print_file_name));
    snopt::integer inform;
    snopt::snopenappend_(&cur.iPrint,print_file_name,&inform,print_file_name_len);
    cur.snSeti("Major print level",static_cast<snopt::integer>(11));
    cur.snSeti("Print file",cur.iPrint);
  }

  snopt::integer minrw,miniw,mincw;
  cur.snMemA(nF, nx, nxname, nFname, lenA, lenG, &mincw, &miniw, &minrw);
  if (minrw>d->lenrw) {
    //mexPrintf("reallocation rw with size %d\n",minrw);
    d->lenrw = minrw;
    d->rw.reset(new snopt::doublereal[d->lenrw]);
  }
  if (miniw>d->leniw) {
    //mexPrintf("reallocation iw with size %d\n",miniw);
    d->leniw = miniw;
    d->iw.reset(new snopt::integer[d->leniw]);
  }
  if (mincw>d->lencw) {
    //mexPrintf("reallocation cw with size %d\n",mincw);
    d->lencw = mincw;
    d->cw.reset(new char[8*d->lencw]);
  }
  cur.snInit();

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

  snopt::integer info;
  snopt::snopta_
          (&Cold, &nF, &nx, &nxname, &nFname,
           &ObjAdd, &ObjRow, Prob, snopt_userfun,
           iAfun, jAvar, &lenA, &lenA, A,
           iGfun, jGvar, &lenG, &lenG,
           xlow, xupp, xnames, Flow, Fupp, Fnames,
           x, xstate, xmul, F, Fstate, Fmul,
           &info, &mincw, &miniw, &minrw,
           &nS, &nInf, &sInf,
           d->cw.get(), &d->lencw, d->iw.get(), &d->leniw, d->rw.get(), &d->lenrw,
           d->cw.get(), &d->lencw, d->iw.get(), &d->leniw, d->rw.get(), &d->lenrw,
           npname, 8*nxname, 8*nFname,
            8*d->lencw,8*d->lencw);

  cout << "SNOPT INFO: " << info << endl;

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

  return true;
}

