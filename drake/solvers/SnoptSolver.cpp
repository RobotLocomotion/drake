
#include "drake/solvers/SnoptSolver.h"

#include <cstdlib>
#include <memory>
#include <algorithm>
#include <cstdio>
#include <cstring>

#include "drake/solvers/Optimization.h"

namespace snopt {
#include "snopt.hh"
#include "snfilewrapper.hh"
}


// todo:  implement sparsity inside each objective/constraint
// todo:  handle snopt options
// todo:  return more information that just the solution (INFO, infeasible
// constraints, ...)
// todo:  avoid all dynamic allocation

// snopt minimum workspace requirements
unsigned int constexpr snopt_mincw = 500;
unsigned int constexpr snopt_miniw = 500;
unsigned int constexpr snopt_minrw = 500;

bool Drake::SnoptSolver::available() const {
  return true;
}

struct SNOPTData : public Drake::OptimizationProblem::SolverData {
  std::vector<char> cw;
  std::vector<snopt::integer> iw;
  std::vector<snopt::doublereal> rw;
  snopt::integer lencw = 0;
  snopt::integer leniw = 0;
  snopt::integer lenrw = 0;

  std::vector<snopt::doublereal> x;
  std::vector<snopt::doublereal> xlow;
  std::vector<snopt::doublereal> xupp;
  std::vector<snopt::doublereal> xmul;
  std::vector<snopt::integer> xstate;

  std::vector<snopt::doublereal> F;
  std::vector<snopt::doublereal> Flow;
  std::vector<snopt::doublereal> Fupp;
  std::vector<snopt::doublereal> Fmul;
  std::vector<snopt::integer> Fstate;

  std::vector<snopt::doublereal> A;
  std::vector<snopt::integer> iAfun;
  std::vector<snopt::integer> jAvar;

  std::vector<snopt::integer> iGfun;
  std::vector<snopt::integer> jGvar;

  void min_alloc_w(snopt::integer mincw, snopt::integer miniw,
                   snopt::integer minrw) {
    if (lencw < mincw) {
      lencw = mincw;
      cw.resize(8 * lencw);
    }
    if (leniw < miniw) {
      leniw = miniw;
      iw.resize(leniw);
    }
    if (lenrw < minrw) {
      lenrw = minrw;
      rw.resize(lenrw);
    }
  }

  void min_alloc_x(snopt::integer nx) {
    if (nx > x.size()) {
      x.resize(nx);
      xlow.resize(nx);
      xupp.resize(nx);
      xmul.resize(nx);
      xstate.resize(nx);
    }
  }

  void min_alloc_F(snopt::integer nF) {
    if (nF > F.size()) {
      F.resize(nF);
      Flow.resize(nF);
      Fupp.resize(nF);
      Fmul.resize(nF);
      Fstate.resize(nF);
    }
  }

  void min_alloc_A(snopt::integer nA) {
    if (nA > A.size()) {
      A.resize(nA);
      iAfun.resize(nA);
      jAvar.resize(nA);
    }
  }

  void min_alloc_G(snopt::integer nG) {
    if (nG > iGfun.size()) {
      iGfun.resize(nG);
      jGvar.resize(nG);
    }
  }
};

struct SNOPTRun {
  SNOPTRun(SNOPTData& d) : D(d) {
    // Minimum default allocation needed by snInit
    D.min_alloc_w(snopt_mincw, snopt_miniw * 1000, snopt_minrw * 1000);

    snInit();
  }

  ~SNOPTRun() {
    if (iPrint >= 0) {
      snopt::snclose_(&iPrint);
    }
  }

  SNOPTData& D;

  snopt::integer iPrint = -1;
  snopt::integer iSumm = -1;

  snopt::integer snSeti(std::string const& opt, snopt::integer val) {
    snopt::integer opt_len = static_cast<snopt::integer>(opt.length());
    snopt::integer err = 0;
    snopt::snseti_(opt.c_str(), &val, &iPrint, &iSumm, &err, D.cw.data(),
                   &D.lencw, D.iw.data(), &D.leniw, D.rw.data(), &D.lenrw,
                   opt_len, 8 * D.lencw);
    return err;
  }

  snopt::integer snSetr(std::string const& opt, snopt::doublereal val) {
    snopt::integer opt_len = static_cast<snopt::integer>(opt.length());
    snopt::integer err = 0;
    snopt::snsetr_(opt.c_str(), &val, &iPrint, &iSumm, &err, D.cw.data(),
                   &D.lencw, D.iw.data(), &D.leniw, D.rw.data(), &D.lenrw,
                   opt_len, 8 * D.lencw);
    return err;
  }

  void snInit() {
    snopt::sninit_(&iPrint, &iSumm, D.cw.data(), &D.lencw, D.iw.data(),
                   &D.leniw, D.rw.data(), &D.lenrw, 8 * D.lencw);
  }

  snopt::integer snMemA(snopt::integer nF, snopt::integer nx,
                        snopt::integer nxname, snopt::integer nFname,
                        snopt::integer neA, snopt::integer neG,
                        snopt::integer* mincw, snopt::integer* miniw,
                        snopt::integer* minrw) {
    snopt::integer info = 0;
    snopt::snmema_(&info, &nF, &nx, &nxname, &nFname, &neA, &neG, mincw, miniw,
                   minrw, D.cw.data(), &D.lencw, D.iw.data(), &D.leniw,
                   D.rw.data(), &D.lenrw, 8 * D.lencw);
    return info;
  }
};

static int snopt_userfun(snopt::integer* Status, snopt::integer* n,
                         snopt::doublereal x[], snopt::integer* needF,
                         snopt::integer* neF, snopt::doublereal F[],
                         snopt::integer* needG, snopt::integer* neG,
                         snopt::doublereal G[], char* cu, snopt::integer* lencu,
                         snopt::integer iu[], snopt::integer* leniu,
                         snopt::doublereal ru[], snopt::integer* lenru) {
  // Our snOptA call passes the snopt workspace as the user workspace and
  // reserves one 8-char of space to pass the problem pointer.
  Drake::OptimizationProblem const* current_problem = NULL;
  {
    char* const pcp = reinterpret_cast<char*>(&current_problem);
    char const* const cu_cp = cu + 8 * snopt_mincw;
    std::copy(cu_cp, cu_cp + sizeof(current_problem), pcp);
  }

  size_t constraint_index = 0, grad_index = 0;  // constraint index starts at 1
                                                // because the objective is the
                                                // first row
  snopt::integer i;
  Eigen::VectorXd xvec(*n);
  for (i = 0; i < *n; i++) {
    xvec(i) = static_cast<double>(x[i]);
  }

  F[0] = 0.0;
  memset(G, 0, sizeof(*n) * sizeof(snopt::doublereal));

  // evaluate objective
  auto tx = Drake::initializeAutoDiff(xvec);
  Drake::TaylorVecXd ty(1), this_x(*n);
  for (auto const& binding : current_problem->generic_objectives()) {
    auto const& obj = binding.constraint();
    size_t index = 0;
    for (const Drake::DecisionVariableView& v : binding.variable_list()) {
      this_x.segment(index, v.size()) = tx.segment(v.index(), v.size());
      index += v.size();
    }

    obj->eval(tx, ty);

    F[constraint_index++] += static_cast<snopt::doublereal>(
        ty(0).value());
    for (const Drake::DecisionVariableView& v : binding.variable_list()) {
      for (size_t j = v.index(); j < v.index() + v.size(); j++) {
        G[grad_index + j] +=
            static_cast<snopt::doublereal>(ty(0).derivatives()(j));
      }
    }
  }
  grad_index += *n;

  for (auto const& binding : current_problem->generic_constraints()) {
    auto const& c = binding.constraint();
    size_t index = 0, num_constraints = c->num_constraints();
    for (const Drake::DecisionVariableView& v : binding.variable_list()) {
      this_x.segment(index, v.size()) = tx.segment(v.index(), v.size());
      index += v.size();
    }

    ty.resize(num_constraints);
    c->eval(tx, ty);

    for (i = 0; i < num_constraints; i++) {
      F[constraint_index++] = static_cast<snopt::doublereal>(ty(i).value());
    }
    for (const Drake::DecisionVariableView& v : binding.variable_list()) {
      for (i = 0; i < num_constraints; i++) {
        for (size_t j = v.index(); j < v.index() + v.size(); j++) {
          G[grad_index++] =
              static_cast<snopt::doublereal>(ty(i).derivatives()(j));
        }
      }
    }
  }

  return 0;
}

bool Drake::SnoptSolver::Solve(
    OptimizationProblem& prog) const {
  auto d = prog.GetSolverData<SNOPTData>();
  SNOPTRun cur(*d);

  Drake::OptimizationProblem const* current_problem = &prog;

  // Set the "maxcu" value to tell snopt to reserve one 8-char entry of user
  // workspace.  We are then allowed to use cw(snopt_mincw+1:maxcu), as
  // expressed in Fortran array slicing.   Use the space to pass our problem
  // instance pointer to our userfun.
  cur.snSeti("User character workspace", snopt_mincw + 1);
  {
    char const* const pcp = reinterpret_cast<char*>(&current_problem);
    char* const cu_cp = d->cw.data() + 8 * snopt_mincw;
    std::copy(pcp, pcp + sizeof(current_problem), cu_cp);
  }

  snopt::integer nx = prog.num_vars();
  d->min_alloc_x(nx);
  snopt::doublereal* x = d->x.data();
  snopt::doublereal* xlow = d->xlow.data();
  snopt::doublereal* xupp = d->xupp.data();
  const Eigen::VectorXd x_initial_guess = prog.initial_guess();
  for (int i = 0; i < nx; i++) {
    x[i] = static_cast<snopt::doublereal>(x_initial_guess(i));
    xlow[i] =
        static_cast<snopt::doublereal>(-std::numeric_limits<double>::infinity());
    xupp[i] =
        static_cast<snopt::doublereal>(std::numeric_limits<double>::infinity());
  }
  for (auto const& binding : prog.bounding_box_constraints()) {
    auto const& c = binding.constraint();
    for (const DecisionVariableView& v : binding.variable_list()) {
      auto const lb = c->lower_bound(), ub = c->upper_bound();
      for (int k = 0; k < v.size(); k++) {
        xlow[v.index() + k] = std::max<snopt::doublereal>(
            static_cast<snopt::doublereal>(lb(k)), xlow[v.index() + k]);
        xupp[v.index() + k] = std::min<snopt::doublereal>(
            static_cast<snopt::doublereal>(ub(k)), xupp[v.index() + k]);
      }
    }
  }

  size_t num_nonlinear_constraints = 0, max_num_gradients = nx;
  for (auto const& binding : prog.generic_constraints()) {
    auto const& c = binding.constraint();
    size_t n = c->num_constraints();
    for (const DecisionVariableView& v : binding.variable_list()) {
      max_num_gradients += n * v.size();
    }
    num_nonlinear_constraints += n;
  }
  size_t num_linear_constraints = 0;
  for (auto const& binding : prog.linear_equality_constraints()) {
    num_linear_constraints += binding.constraint()->num_constraints();
  }

  snopt::integer nF = 1 + num_nonlinear_constraints + num_linear_constraints;
  d->min_alloc_F(nF);
  snopt::doublereal* Flow = d->Flow.data();
  snopt::doublereal* Fupp = d->Fupp.data();
  Flow[0] =
      static_cast<snopt::doublereal>(-std::numeric_limits<double>::infinity());
  Fupp[0] =
      static_cast<snopt::doublereal>(std::numeric_limits<double>::infinity());

  snopt::integer lenG = static_cast<snopt::integer>(max_num_gradients);
  d->min_alloc_G(lenG);
  snopt::integer* iGfun = d->iGfun.data();
  snopt::integer* jGvar = d->jGvar.data();
  for (snopt::integer i = 0; i < nx; i++) {
    iGfun[i] = 1;
    jGvar[i] = i + 1;
  }

  size_t constraint_index = 1, grad_index = nx;  // constraint index starts at 1
                                                 // because the objective is the
                                                 // first row
  for (auto const& binding : prog.generic_constraints()) {
    auto const& c = binding.constraint();
    size_t n = c->num_constraints();

    auto const lb = c->lower_bound(), ub = c->upper_bound();
    for (int i = 0; i < n; i++) {
      Flow[constraint_index + i] = static_cast<snopt::doublereal>(lb(i));
      Fupp[constraint_index + i] = static_cast<snopt::doublereal>(ub(i));
    }

    for (const DecisionVariableView& v : binding.variable_list()) {
      for (size_t i = 0; i < n; i++) {
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
  tripletList.reserve(num_linear_constraints * prog.num_vars());

  size_t linear_constraint_index = 0;
  for (auto const& binding : prog.linear_equality_constraints()) {
    auto const& c = binding.constraint();
    size_t n = c->num_constraints();
    size_t var_index = 0;
    Eigen::SparseMatrix<double> A_constraint = c->GetSparseMatrix();
    for (const DecisionVariableView& v : binding.variable_list()) {
      for (size_t k = 0; k < v.size(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(
                 A_constraint, var_index + k);
             it; ++it) {
          tripletList.push_back(
              T(linear_constraint_index + it.col(), v.index() + k, it.value()));
        }
      }
      var_index += v.size();
    }

    auto const lb = c->lower_bound(), ub = c->upper_bound();
    for (int i = 0; i < n; i++) {
      Flow[constraint_index + i] = static_cast<snopt::doublereal>(lb(i));
      Fupp[constraint_index + i] = static_cast<snopt::doublereal>(ub(i));
    }
    constraint_index += n;
    linear_constraint_index += n;
  }

  snopt::integer lenA = static_cast<snopt::integer>(tripletList.size());
  d->min_alloc_A(lenA);
  snopt::doublereal* A = d->A.data();
  snopt::integer* iAfun = d->iAfun.data();
  snopt::integer* jAvar = d->jAvar.data();
  size_t A_index = 0;
  for (auto const& it : tripletList) {
    A[A_index] = it.value();
    iAfun[A_index] = 1 + num_nonlinear_constraints + it.row() + 1;
    jAvar[A_index] = it.col() + 1;
    A_index++;
  }

  snopt::integer nxname = 1, nFname = 1, npname = 0;
  char xnames[8 * 1];  // should match nxname
  char Fnames[8 * 1];  // should match nFname
  char Prob[200] = "drake.out";

  snopt::integer nS, nInf;
  snopt::doublereal sInf;
  if (true) {  // print to output file (todo: make this an option)
    cur.iPrint = 9;
    char print_file_name[50] = "snopt.out";
    snopt::integer print_file_name_len =
        static_cast<snopt::integer>(strlen(print_file_name));
    snopt::integer inform;
    snopt::snopenappend_(&cur.iPrint, print_file_name, &inform,
                         print_file_name_len);
    cur.snSeti("Major print level", static_cast<snopt::integer>(11));
    cur.snSeti("Print file", cur.iPrint);
  }

  snopt::integer minrw, miniw, mincw;
  cur.snMemA(nF, nx, nxname, nFname, lenA, lenG, &mincw, &miniw, &minrw);
  d->min_alloc_w(mincw, miniw, minrw);
  cur.snSeti("Total character workspace", d->lencw);
  cur.snSeti("Total integer workspace", d->leniw);
  cur.snSeti("Total real workspace", d->lenrw);

  snopt::integer Cold = 0;
  snopt::doublereal* xmul = d->xmul.data();
  snopt::integer* xstate = d->xstate.data();
  memset(xstate, 0, sizeof(snopt::integer) * nx);

  snopt::doublereal* F = d->F.data();
  snopt::doublereal* Fmul = d->Fmul.data();
  snopt::integer* Fstate = d->Fstate.data();
  memset(Fstate, 0, sizeof(snopt::integer) * nF);

  snopt::doublereal ObjAdd = 0.0;
  snopt::integer ObjRow = 1;  // feasibility problem (for now)

  // TODO sam.creasey These should be made into options when #1879 is
  // resolved or deleted.
  /*
    mysnseti("Derivative
    option",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"DerivativeOption"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
    mysnseti("Major iterations
    limit",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"MajorIterationsLimit"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
    mysnseti("Minor iterations
    limit",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"MinorIterationsLimit"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
    mysnsetr("Major optimality
    tolerance",static_cast<snopt::doublereal>(*mxGetPr(mxGetField(prhs[13],0,"MajorOptimalityTolerance"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
    mysnsetr("Major feasibility
    tolerance",static_cast<snopt::doublereal>(*mxGetPr(mxGetField(prhs[13],0,"MajorFeasibilityTolerance"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
    mysnsetr("Minor feasibility
    tolerance",static_cast<snopt::doublereal>(*mxGetPr(mxGetField(prhs[13],0,"MinorFeasibilityTolerance"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
    mysnseti("Superbasics
    limit",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"SuperbasicsLimit"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
    mysnseti("Verify
    level",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"VerifyLevel"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
    mysnseti("Iterations
    Limit",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"IterationsLimit"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
    mysnseti("Scale
    option",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"ScaleOption"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
    mysnseti("New basis
    file",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"NewBasisFile"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
    mysnseti("Old basis
    file",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"OldBasisFile"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
    mysnseti("Backup basis
    file",static_cast<snopt::integer>(*mxGetPr(mxGetField(prhs[13],0,"BackupBasisFile"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
    mysnsetr("Linesearch
    tolerance",static_cast<snopt::doublereal>(*mxGetPr(mxGetField(prhs[13],0,"LinesearchTolerance"))),&iPrint,&iSumm,&INFO_snopt,cw.get(),&lencw,iw.get(),&leniw,rw.get(),&lenrw);
  */

  snopt::integer info;
  snopt::snopta_(
      &Cold, &nF, &nx, &nxname, &nFname, &ObjAdd, &ObjRow, Prob, snopt_userfun,
      iAfun, jAvar, &lenA, &lenA, A, iGfun, jGvar, &lenG, &lenG, xlow, xupp,
      xnames, Flow, Fupp, Fnames, x, xstate, xmul, F, Fstate, Fmul, &info,
      &mincw, &miniw, &minrw, &nS, &nInf, &sInf, d->cw.data(), &d->lencw,
      d->iw.data(), &d->leniw, d->rw.data(), &d->lenrw,
      // Pass the snopt workspace as the user workspace.  We already set
      // the maxcu option to reserve some of it for our user code.
      d->cw.data(), &d->lencw, d->iw.data(), &d->leniw, d->rw.data(), &d->lenrw,
      npname, 8 * nxname, 8 * nFname, 8 * d->lencw, 8 * d->lencw);


  Eigen::VectorXd sol(nx);
  for (int i = 0; i < nx; i++) {
    sol(i) = static_cast<double>(x[i]);
  }
  prog.SetDecisionVariableValues(sol);

  // todo: extract the other useful quantities, too.

  return true;
}
