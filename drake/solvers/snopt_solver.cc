#include "drake/solvers/snopt_solver.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

#include "drake/math/autodiff.h"

namespace snopt {
// Needs to include snopt.hh BEFORE snfilewrapper.hh, otherwise compiler does
// not work.
// clang-format wants to switch the order of this inclusion, which causes
// compiler failure.
/* clang-format off */
#include "snopt.hh"
#include "snfilewrapper.hh"
/* clang-format on */
}

// todo(sammy-tri) :  implement sparsity inside each cost/constraint
// todo(sammy-tri) :  handle snopt options
// todo(sammy-tri) :  return more information that just the solution (INFO,
// infeasible constraints, ...)
// todo(sammy-tri) :  avoid all dynamic allocation

namespace drake {
namespace solvers {
namespace {

// snopt minimum workspace requirements
unsigned int constexpr snopt_mincw = 500;
unsigned int constexpr snopt_miniw = 500;
unsigned int constexpr snopt_minrw = 500;

struct SNOPTData : public MathematicalProgram::SolverData {
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
    if (nx > static_cast<snopt::integer>(x.size())) {
      x.resize(nx);
      xlow.resize(nx);
      xupp.resize(nx);
      xmul.resize(nx);
      xstate.resize(nx);
    }
  }

  void min_alloc_F(snopt::integer nF) {
    if (nF > static_cast<snopt::integer>(F.size())) {
      F.resize(nF);
      Flow.resize(nF);
      Fupp.resize(nF);
      Fmul.resize(nF);
      Fstate.resize(nF);
    }
  }

  void min_alloc_A(snopt::integer nA) {
    if (nA > static_cast<snopt::integer>(A.size())) {
      A.resize(nA);
      iAfun.resize(nA);
      jAvar.resize(nA);
    }
  }

  void min_alloc_G(snopt::integer nG) {
    if (nG > static_cast<snopt::integer>(iGfun.size())) {
      iGfun.resize(nG);
      jGvar.resize(nG);
    }
  }
};

struct SNOPTRun {
  SNOPTRun(SNOPTData& d, MathematicalProgram const* current_problem) : D(d) {
    // Use the minimum default allocation needed by snInit.  The +1
    // added to snopt_mincw is to make room for the instance pointer.
    D.min_alloc_w(snopt_mincw + 1, snopt_miniw * 1000, snopt_minrw * 1000);

    snInit();

    // Set the "maxcu" value to tell snopt to reserve one 8-char entry of user
    // workspace.  We are then allowed to use cw(snopt_mincw+1:maxcu), as
    // expressed in Fortran array slicing.  Use the space to pass our problem
    // instance pointer to our userfun.
    snSeti("User character workspace", snopt_mincw + 1);
    {
      char const* const pcp = reinterpret_cast<char*>(&current_problem);
      char* const cu_cp = d.cw.data() + 8 * snopt_mincw;
      std::copy(pcp, pcp + sizeof(current_problem), cu_cp);
    }
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

/*
 * Evaluate the value and gradients of nonlinear constraints.
 * The template type Binding is supposed to be a
 * MathematicalProgram::Binding<Constraint> type.
 * @param constraint_list A list of Binding<Constraint>
 * @param F The value of the constraints
 * @param G The value of the non-zero entries in the gradient
 * @param constraint_index The starting index of the constraint_list(0) in the
 * optimization problem.
 * @param grad_index The starting index of the gradient of constraint_list(0)
 * in the optimization problem.
 * @param tx the AutoDiffMatrixType that stores the value of the decision
 * variable.
 */
template <typename C>
void EvaluateNonlinearConstraints(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& constraint_list, snopt::doublereal F[],
    snopt::doublereal G[], size_t* constraint_index, size_t* grad_index,
    const math::AutoDiffMatrixType<Eigen::VectorXd, Eigen::Dynamic>& tx) {
  TaylorVecXd this_x;
  for (const auto& binding : constraint_list) {
    const auto& c = binding.constraint();
    size_t num_constraints = c->num_constraints();

    int num_v_variables = binding.GetNumElements();
    this_x.resize(num_v_variables);
    for (int i = 0; i < num_v_variables; ++i) {
      this_x(i) = tx(prog.FindDecisionVariableIndex(binding.variables()(i)));
    }

    TaylorVecXd ty;
    ty.resize(num_constraints);
    c->Eval(this_x, ty);

    for (snopt::integer i = 0; i < static_cast<snopt::integer>(num_constraints);
         i++) {
      F[(*constraint_index)++] = static_cast<snopt::doublereal>(ty(i).value());
    }

    for (snopt::integer i = 0; i < static_cast<snopt::integer>(num_constraints);
         i++) {
      for (int j = 0; j < num_v_variables; ++j) {
        G[(*grad_index)++] = static_cast<snopt::doublereal>(ty(i).derivatives()(
            prog.FindDecisionVariableIndex(binding.variables()(j))));
      }
    }
  }
}

int snopt_userfun(snopt::integer* Status, snopt::integer* n,
                  snopt::doublereal x[], snopt::integer* needF,
                  snopt::integer* neF, snopt::doublereal F[],
                  snopt::integer* needG, snopt::integer* neG,
                  snopt::doublereal G[], char* cu, snopt::integer* lencu,
                  snopt::integer iu[], snopt::integer* leniu,
                  snopt::doublereal ru[], snopt::integer* lenru) {
  // Our snOptA call passes the snopt workspace as the user workspace and
  // reserves one 8-char of space to pass the problem pointer.
  MathematicalProgram const* current_problem = NULL;
  {
    char* const pcp = reinterpret_cast<char*>(&current_problem);
    char const* const cu_cp = cu + 8 * snopt_mincw;
    std::copy(cu_cp, cu_cp + sizeof(current_problem), pcp);
  }

  snopt::integer i;
  Eigen::VectorXd xvec(*n);
  for (i = 0; i < *n; i++) {
    xvec(i) = static_cast<double>(x[i]);
  }

  F[0] = 0.0;
  memset(G, 0, (*n) * sizeof(snopt::doublereal));

  // evaluate cost
  auto tx = math::initializeAutoDiff(xvec);
  TaylorVecXd ty(1), this_x;

  for (auto const& binding : current_problem->GetAllCosts()) {
    auto const& obj = binding.constraint();

    int num_v_variables = binding.GetNumElements();
    this_x.resize(num_v_variables);
    for (int j = 0; j < num_v_variables; ++j) {
      this_x(j) = tx(
          current_problem->FindDecisionVariableIndex(binding.variables()(j)));
    }

    obj->Eval(this_x, ty);

    F[0] += static_cast<snopt::doublereal>(ty(0).value());

    for (int j = 0; j < static_cast<int>(binding.GetNumElements()); ++j) {
      size_t vj_index =
          current_problem->FindDecisionVariableIndex(binding.variables()(j));
      G[vj_index] +=
          static_cast<snopt::doublereal>(ty(0).derivatives()(vj_index));
    }
  }

  // The constraint index starts at 1 because the cost is the
  // first row.
  size_t constraint_index = 1;
  // The gradient_index also starts after the cost.
  size_t grad_index = *n;
  EvaluateNonlinearConstraints(*current_problem,
                               current_problem->generic_constraints(), F, G,
                               &constraint_index, &grad_index, tx);
  EvaluateNonlinearConstraints(*current_problem,
                               current_problem->lorentz_cone_constraints(), F,
                               G, &constraint_index, &grad_index, tx);
  EvaluateNonlinearConstraints(
      *current_problem, current_problem->rotated_lorentz_cone_constraints(), F,
      G, &constraint_index, &grad_index, tx);

  return 0;
}

/*
 * Updates the number of nonlinear constraints and the number of gradients by
 * looping through the constraint list
 * @tparam C A Constraint type. Note that some derived classes of Constraint
 * is regarded as generic constraint by SNOPT solver, such as
 * LorentzConeConstraint and RotatedLorentzConeConstraint, so @tparam C can also
 * be these derived classes.
 */
template <typename C>
void UpdateNumNonlinearConstraintsAndGradients(
    const std::vector<Binding<C>>& constraint_list,
    size_t* num_nonlinear_constraints, size_t* max_num_gradients) {
  for (auto const& binding : constraint_list) {
    auto const& c = binding.constraint();
    size_t n = c->num_constraints();
    *max_num_gradients += n * binding.GetNumElements();
    *num_nonlinear_constraints += n;
  }
}

template <typename C>
void UpdateConstraintBoundsAndGradients(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& constraint_list, snopt::doublereal* Flow,
    snopt::doublereal* Fupp, snopt::integer* iGfun, snopt::integer* jGvar,
    size_t* constraint_index, size_t* grad_index) {
  for (auto const& binding : constraint_list) {
    auto const& c = binding.constraint();
    size_t n = c->num_constraints();

    auto const lb = c->lower_bound(), ub = c->upper_bound();
    for (size_t i = 0; i < n; i++) {
      Flow[*constraint_index + i] = static_cast<snopt::doublereal>(lb(i));
      Fupp[*constraint_index + i] = static_cast<snopt::doublereal>(ub(i));
    }

    for (size_t i = 0; i < n; i++) {
      for (int j = 0; j < static_cast<int>(binding.GetNumElements()); ++j) {
        iGfun[*grad_index] = *constraint_index + i + 1;  // row order
        jGvar[*grad_index] =
            prog.FindDecisionVariableIndex(binding.variables()(j)) + 1;
        (*grad_index)++;
      }
    }

    (*constraint_index) += n;
  }
}

}  // anon namespace

bool SnoptSolver::available() const { return true; }

SolutionResult SnoptSolver::Solve(MathematicalProgram& prog) const {
  auto d = prog.GetSolverData<SNOPTData>();
  SNOPTRun cur(*d, &prog);

  snopt::integer nx = prog.num_vars();
  d->min_alloc_x(nx);
  snopt::doublereal* x = d->x.data();
  snopt::doublereal* xlow = d->xlow.data();
  snopt::doublereal* xupp = d->xupp.data();
  const Eigen::VectorXd x_initial_guess = prog.initial_guess();
  for (int i = 0; i < nx; i++) {
    x[i] = static_cast<snopt::doublereal>(x_initial_guess(i));
    xlow[i] = static_cast<snopt::doublereal>(
        -std::numeric_limits<double>::infinity());
    xupp[i] = static_cast<snopt::doublereal>(  // BR
        std::numeric_limits<double>::infinity());
  }
  for (auto const& binding : prog.bounding_box_constraints()) {
    const auto& c = binding.constraint();
    const auto& lb = c->lower_bound();
    const auto& ub = c->upper_bound();

    for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
      const size_t vk_index =
          prog.FindDecisionVariableIndex(binding.variables()(k));
      xlow[vk_index] = std::max<snopt::doublereal>(
          static_cast<snopt::doublereal>(lb(k)), xlow[vk_index]);
      xupp[vk_index] = std::min<snopt::doublereal>(
          static_cast<snopt::doublereal>(ub(k)), xupp[vk_index]);
    }
  }

  size_t num_nonlinear_constraints = 0, max_num_gradients = nx;
  UpdateNumNonlinearConstraintsAndGradients(prog.generic_constraints(),
                                            &num_nonlinear_constraints,
                                            &max_num_gradients);
  UpdateNumNonlinearConstraintsAndGradients(prog.lorentz_cone_constraints(),
                                            &num_nonlinear_constraints,
                                            &max_num_gradients);
  UpdateNumNonlinearConstraintsAndGradients(
      prog.rotated_lorentz_cone_constraints(), &num_nonlinear_constraints,
      &max_num_gradients);

  size_t num_linear_constraints = 0;
  const auto linear_constraints = prog.GetAllLinearConstraints();
  for (auto const& binding : linear_constraints) {
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
                                                 // because the cost is the
                                                 // first row
  UpdateConstraintBoundsAndGradients(prog, prog.generic_constraints(), Flow,
                                     Fupp, iGfun, jGvar, &constraint_index,
                                     &grad_index);
  UpdateConstraintBoundsAndGradients(prog, prog.lorentz_cone_constraints(),
                                     Flow, Fupp, iGfun, jGvar,
                                     &constraint_index, &grad_index);
  UpdateConstraintBoundsAndGradients(
      prog, prog.rotated_lorentz_cone_constraints(), Flow, Fupp, iGfun, jGvar,
      &constraint_index, &grad_index);

  // http://eigen.tuxfamily.org/dox/group__TutorialSparse.html
  typedef Eigen::Triplet<double> T;
  std::vector<T> tripletList;
  tripletList.reserve(num_linear_constraints * prog.num_vars());

  size_t linear_constraint_index = 0;
  for (auto const& binding : linear_constraints) {
    auto const& c = binding.constraint();
    size_t n = c->num_constraints();

    Eigen::SparseMatrix<double> A_constraint = c->GetSparseMatrix();

    for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(A_constraint, k); it;
           ++it) {
        tripletList.push_back(
            T(linear_constraint_index + it.row(),
              prog.FindDecisionVariableIndex(binding.variables()(k)),
              it.value()));
      }
    }

    auto const lb = c->lower_bound(), ub = c->upper_bound();
    for (size_t i = 0; i < n; i++) {
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

  for (const auto it : prog.GetSolverOptionsDouble(SolverType::kSnopt)) {
    cur.snSetr(it.first, it.second);
  }

  for (const auto it : prog.GetSolverOptionsInt(SolverType::kSnopt)) {
    cur.snSeti(it.first, it.second);
  }

  snopt::integer info;
  snopt::snopta_(
      &Cold, &nF, &nx, &nxname, &nFname, &ObjAdd, &ObjRow, Prob, snopt_userfun,
      iAfun, jAvar, &lenA, &lenA, A, iGfun, jGvar, &lenG, &lenG, xlow, xupp,
      xnames, Flow, Fupp, Fnames, x, xstate, xmul, F, Fstate, Fmul, &info,
      &mincw, &miniw, &minrw, &nS, &nInf, &sInf,
      // Pass the snopt workspace as the user workspace.  We already set
      // the maxcu option to reserve some of it for our user code.
      d->cw.data(), &d->lencw, d->iw.data(), &d->leniw, d->rw.data(), &d->lenrw,
      d->cw.data(), &d->lencw, d->iw.data(), &d->leniw, d->rw.data(), &d->lenrw,
      npname, 8 * nxname, 8 * nFname, 8 * d->lencw, 8 * d->lencw);

  Eigen::VectorXd sol(nx);
  for (int i = 0; i < nx; i++) {
    sol(i) = static_cast<double>(x[i]);
  }
  prog.SetDecisionVariableValues(sol);
  prog.SetSolverResult(solver_type(), info);

  // todo: extract the other useful quantities, too.

  if (info >= 1 && info <= 6) {
    return SolutionResult::kSolutionFound;
  } else if (info >= 11 && info <= 16) {
    return SolutionResult::kInfeasibleConstraints;
  } else if (info == 91) {
    return SolutionResult::kInvalidInput;
  }
  return SolutionResult::kUnknownError;
}

}  // namespace solvers
}  // namespace drake
