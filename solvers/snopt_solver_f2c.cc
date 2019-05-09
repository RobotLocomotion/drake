/* clang-format off to disable clang-format-includes */
#include "drake/solvers/snopt_solver.h"
/* clang-format on */

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"
#include "drake/solvers/mathematical_program.h"

// TODO(#7984) The SNOPT includes we use below are from an older f2c-based
// implementation.  SNOPT has since switched to wrapping using F90 per the
// publicly-available `snopt-interface` headers.  We should consider using that
// header instead, which would remove a bunch of the odd #include and #define
// statements from the below.

// Put SNOPT's and F2C's typedefs into their own namespace.
namespace snopt {
extern "C" {

// Include F2C's typedefs but revert its leaky defines.
#include <f2c.h>
#undef qbit_clear
#undef qbit_set
#undef TRUE_
#undef FALSE_
#undef Extern
#undef VOID
#undef abs
#undef dabs
#undef min
#undef max
#undef dmin
#undef dmax
#undef bit_test
#undef bit_clear
#undef bit_set

// Include SNOPT's function declarations.
#include <cexamples/snopt.h>
#ifdef SNOPT_HAS_SNFILEWRAPPER
#include <cexamples/snfilewrapper.h>
#endif

}  // extern C
}  // namespace snopt

// TODO(jwnimmer-tri) Eventually resolve these warnings.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

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

struct SNOPTData final {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SNOPTData)
  SNOPTData() = default;

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

// This struct is used for passing additional info to the snopt_userfun, which
// evaluates the value and gradient of the cost and constraints. Apart from the
// standard information such as decision variable values, snopt_userfun could
// rely on additional information such as the cost gradient sparsity pattern.
struct SnoptUserFunInfo {
  const MathematicalProgram* prog_;
  const std::unordered_set<int>* cost_gradient_indices_;
};

struct SNOPTRun {
  SNOPTRun(SNOPTData* d, SnoptUserFunInfo const* snopt_userfun_info) : D(*d) {
    // Use the minimum default allocation needed by snInit.  The +1
    // added to snopt_mincw is to make room for the pointer to SnoptUserFunInfo.
    D.min_alloc_w(snopt_mincw + 1, snopt_miniw * 1000, snopt_minrw * 1000);

    snInit();

    // Set the "maxcu" value to tell snopt to reserve one 8-char entry of user
    // workspace.  We are then allowed to use cw(snopt_mincw+1:maxcu), as
    // expressed in Fortran array slicing.  Use the space to pass the pointer
    // to SnoptUserFunInfo.
    snSeti("User character workspace", snopt_mincw + 1);
    {
      char const* const p_snopt_userfun_info =
          reinterpret_cast<char*>(&snopt_userfun_info);
      char* const cu_snopt_userfun_info = d->cw.data() + 8 * snopt_mincw;
      std::copy(p_snopt_userfun_info,
                p_snopt_userfun_info + sizeof(snopt_userfun_info),
                cu_snopt_userfun_info);
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

  // The `opt` is non-const, because snopt wants a non-const char*.
  snopt::integer snSeti(std::string opt, snopt::integer val) {
    DRAKE_DEMAND(!opt.empty());
    snopt::integer opt_len = static_cast<snopt::integer>(opt.length());
    snopt::integer err = 0;
    snopt::snseti_(&opt[0], &val, &iPrint, &iSumm, &err, D.cw.data(),
                   &D.lencw, D.iw.data(), &D.leniw, D.rw.data(), &D.lenrw,
                   opt_len, 8 * D.lencw);
    return err;
  }

  // The `opt` is non-const, because snopt wants a non-const char*.
  snopt::integer snSetr(std::string opt, snopt::doublereal val) {
    DRAKE_DEMAND(!opt.empty());
    snopt::integer opt_len = static_cast<snopt::integer>(opt.length());
    snopt::integer err = 0;
    snopt::snsetr_(&opt[0], &val, &iPrint, &iSumm, &err, D.cw.data(),
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

// Return the number of rows in the nonlinear constraint.
template <typename C>
int SingleNonlinearConstraintSize(const C& constraint) {
  return constraint.num_constraints();
}

template <>
int SingleNonlinearConstraintSize<LinearComplementarityConstraint>(
    const LinearComplementarityConstraint& constraint) {
  return 1;
}

// Evaluate a single nonlinear constraints. For generic Constraint,
// LorentzConeConstraint, RotatedLorentzConeConstraint, we call Eval function
// of the constraint directly. For some other constraint, such as
// LinearComplementaryConstraint, we will evaluate its nonlinear constraint
// differently, than its Eval function.
template <typename C>
void EvaluateSingleNonlinearConstraint(const C& constraint,
                                       const Eigen::VectorXd& this_x,
                                       AutoDiffVecXd* ty) {
  ty->resize(SingleNonlinearConstraintSize(constraint));
  constraint.Eval(math::initializeAutoDiff(this_x), ty);
}

template <>
void EvaluateSingleNonlinearConstraint<LinearComplementarityConstraint>(
    const LinearComplementarityConstraint& constraint,
    const Eigen::VectorXd& this_x, AutoDiffVecXd* ty) {
  ty->resize(1);
  auto tx = math::initializeAutoDiff(this_x);
  (*ty)(0) = tx.dot(constraint.M().cast<AutoDiffXd>() * tx +
                    constraint.q().cast<AutoDiffXd>());
}

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
    const Eigen::VectorXd& xvec) {
  Eigen::VectorXd this_x;
  for (const auto& binding : constraint_list) {
    const auto& c = binding.evaluator();
    int num_constraints = SingleNonlinearConstraintSize(*c);

    int num_v_variables = binding.GetNumElements();
    this_x.resize(num_v_variables);
    for (int i = 0; i < num_v_variables; ++i) {
      this_x(i) = xvec(prog.FindDecisionVariableIndex(binding.variables()(i)));
    }

    AutoDiffVecXd ty;
    ty.resize(num_constraints);
    EvaluateSingleNonlinearConstraint(*c, this_x, &ty);

    for (snopt::integer i = 0; i < static_cast<snopt::integer>(num_constraints);
         i++) {
      F[(*constraint_index)++] = static_cast<snopt::doublereal>(ty(i).value());
    }

    const optional<std::vector<std::pair<int, int>>>&
        gradient_sparsity_pattern =
            binding.evaluator()->gradient_sparsity_pattern();
    if (gradient_sparsity_pattern.has_value()) {
      for (const auto& nonzero_entry : gradient_sparsity_pattern.value()) {
        G[(*grad_index)++] = static_cast<snopt::doublereal>(
            ty(nonzero_entry.first).derivatives().size() > 0
                ? ty(nonzero_entry.first).derivatives()(nonzero_entry.second)
                : 0.0);
      }
    } else {
      for (snopt::integer i = 0;
           i < static_cast<snopt::integer>(num_constraints); i++) {
        if (ty(i).derivatives().size() > 0) {
          for (int j = 0; j < num_v_variables; ++j) {
            G[(*grad_index)++] =
                static_cast<snopt::doublereal>(ty(i).derivatives()(j));
          }
        } else {
          for (int j = 0; j < num_v_variables; ++j) {
            G[(*grad_index)++] = snopt::doublereal(0.0);
          }
        }
      }
    }
  }
}

/*
 * Loops through each cost stored in MathematicalProgram, and obtain the indices
 * of the non-zero gradient, in the summed cost.
 */
std::unordered_set<int> GetCostNonzeroGradientIndices(
    const MathematicalProgram& prog) {
  std::unordered_set<int> cost_gradient_indices;
  cost_gradient_indices.reserve(prog.num_vars());
  for (const auto& cost : prog.GetAllCosts()) {
    for (int i = 0; i < static_cast<int>(cost.GetNumElements()); ++i) {
      cost_gradient_indices.insert(
          prog.FindDecisionVariableIndex(cost.variables()(i)));
    }
  }
  return cost_gradient_indices;
}

void EvaluateAllCosts(const MathematicalProgram& prog,
                      const std::unordered_set<int>& cost_gradient_indices,
                      snopt::doublereal F[], snopt::doublereal G[],
                      size_t* grad_index, const Eigen::VectorXd& xvec) {
  // evaluate cost
  Eigen::VectorXd this_x;
  AutoDiffVecXd ty(1);

  std::vector<snopt::doublereal> cost_gradient(prog.num_vars(), 0);
  for (auto const& binding : prog.GetAllCosts()) {
    auto const& obj = binding.evaluator();

    int num_v_variables = binding.GetNumElements();
    this_x.resize(num_v_variables);
    for (int j = 0; j < num_v_variables; ++j) {
      this_x(j) = xvec(prog.FindDecisionVariableIndex(binding.variables()(j)));
    }

    obj->Eval(math::initializeAutoDiff(this_x), &ty);

    F[0] += static_cast<snopt::doublereal>(ty(0).value());

    if (ty(0).derivatives().size() > 0) {
      for (int j = 0; j < num_v_variables; ++j) {
        size_t vj_index =
            prog.FindDecisionVariableIndex(binding.variables()(j));
        cost_gradient[vj_index] +=
            static_cast<snopt::doublereal>(ty(0).derivatives()(j));
      }
    }
  }
  for (const auto cost_gradient_index : cost_gradient_indices) {
    G[*grad_index] = cost_gradient[cost_gradient_index];
    ++(*grad_index);
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
  SnoptUserFunInfo const* snopt_userfun_info = NULL;
  {
    char* const p_snopt_userfun_info =
        reinterpret_cast<char*>(&snopt_userfun_info);
    char const* const cu_snopt_userfun_info = cu + 8 * snopt_mincw;
    std::copy(cu_snopt_userfun_info,
              cu_snopt_userfun_info + sizeof(snopt_userfun_info),
              p_snopt_userfun_info);
  }
  MathematicalProgram const* current_problem = snopt_userfun_info->prog_;
  std::unordered_set<int> const* cost_gradient_indices =
      snopt_userfun_info->cost_gradient_indices_;

  snopt::integer i;
  Eigen::VectorXd xvec(*n);
  for (i = 0; i < *n; i++) {
    xvec(i) = static_cast<double>(x[i]);
  }

  F[0] = 0.0;
  memset(G, 0, (*n) * sizeof(snopt::doublereal));

  size_t grad_index = 0;

  current_problem->EvalVisualizationCallbacks(xvec);

  EvaluateAllCosts(*current_problem, *cost_gradient_indices, F, G, &grad_index,
                   xvec);

  // The constraint index starts at 1 because the cost is the
  // first row.
  size_t constraint_index = 1;
  // The gradient_index also starts after the cost.
  EvaluateNonlinearConstraints(*current_problem,
                               current_problem->generic_constraints(), F, G,
                               &constraint_index, &grad_index, xvec);
  EvaluateNonlinearConstraints(*current_problem,
                               current_problem->lorentz_cone_constraints(), F,
                               G, &constraint_index, &grad_index, xvec);
  EvaluateNonlinearConstraints(
      *current_problem, current_problem->rotated_lorentz_cone_constraints(), F,
      G, &constraint_index, &grad_index, xvec);
  EvaluateNonlinearConstraints(
      *current_problem, current_problem->linear_complementarity_constraints(),
      F, G, &constraint_index, &grad_index, xvec);

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
    int* num_nonlinear_constraints, int* max_num_gradients) {
  for (auto const& binding : constraint_list) {
    auto const& c = binding.evaluator();
    const int n = c->num_constraints();
    if (binding.evaluator()->gradient_sparsity_pattern().has_value()) {
      *max_num_gradients += static_cast<int>(
          binding.evaluator()->gradient_sparsity_pattern().value().size());
    } else {
      *max_num_gradients += n * binding.GetNumElements();
    }
    *num_nonlinear_constraints += n;
  }
}

// For linear complementary condition
// 0 <= x ⊥ Mx + q >= 0
// we add the nonlinear constraint xᵀ(Mx+q) = 0
// So we only add one row of nonlinear constraint, and update the gradient of
// this nonlinear constraint accordingly.
template <>
void UpdateNumNonlinearConstraintsAndGradients<LinearComplementarityConstraint>(
    const std::vector<Binding<LinearComplementarityConstraint>>&
        constraint_list,
    int* num_nonlinear_constraints, int* max_num_gradients) {
  *num_nonlinear_constraints += constraint_list.size();
  for (const auto& binding : constraint_list) {
    *max_num_gradients += binding.evaluator()->M().rows();
  }
}

template <typename C>
void UpdateConstraintBoundsAndGradients(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& constraint_list, snopt::doublereal* Flow,
    snopt::doublereal* Fupp, snopt::integer* iGfun, snopt::integer* jGvar,
    size_t* constraint_index, size_t* grad_index) {
  for (auto const& binding : constraint_list) {
    auto const& c = binding.evaluator();
    int n = c->num_constraints();

    auto const lb = c->lower_bound(), ub = c->upper_bound();
    for (int i = 0; i < n; i++) {
      Flow[*constraint_index + i] = static_cast<snopt::doublereal>(lb(i));
      Fupp[*constraint_index + i] = static_cast<snopt::doublereal>(ub(i));
    }

    const std::vector<int> bound_var_indices_in_prog =
        prog.FindDecisionVariableIndices(binding.variables());

    const optional<std::vector<std::pair<int, int>>>&
        gradient_sparsity_pattern =
            binding.evaluator()->gradient_sparsity_pattern();
    if (gradient_sparsity_pattern.has_value()) {
      for (const auto& nonzero_entry : gradient_sparsity_pattern.value()) {
        // Fortran is 1-indexed.
        iGfun[*grad_index] = 1 + *constraint_index + nonzero_entry.first;
        jGvar[*grad_index] =
            1 + bound_var_indices_in_prog[nonzero_entry.second];
        (*grad_index)++;
      }
    } else {
      for (int i = 0; i < n; i++) {
        for (int j = 0; j < static_cast<int>(binding.GetNumElements()); ++j) {
          iGfun[*grad_index] = *constraint_index + i + 1;  // row order
          jGvar[*grad_index] = bound_var_indices_in_prog[j] + 1;
          (*grad_index)++;
        }
      }
    }

    (*constraint_index) += n;
  }
}

// For linear complementary condition
// 0 <= x ⊥ Mx + q >= 0
// we add the nonlinear constraint xᵀ(Mx + q) = 0
// The bound of this constraint is 0. The indices of the non-zero gradient
// of this constraint is updated accordingly.
template <>
void UpdateConstraintBoundsAndGradients<LinearComplementarityConstraint>(
    const MathematicalProgram& prog,
    const std::vector<Binding<LinearComplementarityConstraint>>&
        constraint_list,
    snopt::doublereal* Flow, snopt::doublereal* Fupp, snopt::integer* iGfun,
    snopt::integer* jGvar, size_t* constraint_index, size_t* grad_index) {
  for (const auto& binding : constraint_list) {
    Flow[*constraint_index] = 0;
    Fupp[*constraint_index] = 0;
    for (int j = 0; j < binding.evaluator()->M().rows(); ++j) {
      iGfun[*grad_index] = *constraint_index + 1;
      jGvar[*grad_index] =
          prog.FindDecisionVariableIndex(binding.variables()(j)) + 1;
      (*grad_index)++;
    }
    ++(*constraint_index);
  }
}

template <typename C>
Eigen::SparseMatrix<double> LinearConstraintA(const C& constraint) {
  return constraint.GetSparseMatrix();
}

// Return the number of rows in the linear constraint
template <typename C>
int LinearConstraintSize(const C& constraint) {
  return constraint.num_constraints();
}

// For linear complementary condition
// 0 <= x ⊥ Mx + q >= 0
// The linear constraint we add to the program is Mx >= -q
// This linear constraint has the same number of rows, as matrix M.
template <>
int LinearConstraintSize<LinearComplementarityConstraint>(
    const LinearComplementarityConstraint& constraint) {
  return constraint.M().rows();
}

template <>
Eigen::SparseMatrix<double> LinearConstraintA<LinearComplementarityConstraint>(
    const LinearComplementarityConstraint& constraint) {
  return constraint.M().sparseView();
}

template <typename C>
std::pair<Eigen::VectorXd, Eigen::VectorXd> LinearConstraintBounds(
    const C& constraint) {
  return std::make_pair(constraint.lower_bound(), constraint.upper_bound());
}

// For linear complementary condition
// 0 <= x ⊥ Mx + q >= 0
// we add the constraint Mx >= -q
template <>
std::pair<Eigen::VectorXd, Eigen::VectorXd>
LinearConstraintBounds<LinearComplementarityConstraint>(
    const LinearComplementarityConstraint& constraint) {
  return std::make_pair(
      -constraint.q(),
      Eigen::VectorXd::Constant(constraint.q().rows(),
                                std::numeric_limits<double>::infinity()));
}

template <typename C>
void UpdateLinearConstraint(const MathematicalProgram& prog,
                            const std::vector<Binding<C>>& linear_constraints,
                            std::vector<Eigen::Triplet<double>>* tripletList,
                            snopt::doublereal* Flow, snopt::doublereal* Fupp,
                            size_t* constraint_index,
                            size_t* linear_constraint_index) {
  for (auto const& binding : linear_constraints) {
    auto const& c = binding.evaluator();
    int n = LinearConstraintSize(*c);

    const Eigen::SparseMatrix<double> A_constraint = LinearConstraintA(*c);

    for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(A_constraint, k); it;
           ++it) {
        tripletList->emplace_back(
            *linear_constraint_index + it.row(),
            prog.FindDecisionVariableIndex(binding.variables()(k)), it.value());
      }
    }

    const auto bounds = LinearConstraintBounds(*c);
    for (int i = 0; i < n; i++) {
      Flow[*constraint_index + i] =
          static_cast<snopt::doublereal>(bounds.first(i));
      Fupp[*constraint_index + i] =
          static_cast<snopt::doublereal>(bounds.second(i));
    }
    *constraint_index += n;
    *linear_constraint_index += n;
  }
}
}  // anon namespace

bool SnoptSolver::is_available() { return true; }

void SnoptSolver::DoSolve(
    const MathematicalProgram& prog,
    const Eigen::VectorXd& initial_guess,
    const SolverOptions& merged_options,
    MathematicalProgramResult* result) const {
  // TODO(hongkai.dai): put SNOPTData inside SnoptSolverDetails, so that we do
  // not need to allocate memory for SNOPTData when we call Solve repeatedly.
  SNOPTData snopt_data{};
  auto d = &snopt_data;

  const std::unordered_set<int> cost_gradient_indices =
      GetCostNonzeroGradientIndices(prog);
  SnoptUserFunInfo snopt_userfun_info;
  snopt_userfun_info.prog_ = &prog;
  snopt_userfun_info.cost_gradient_indices_ = &cost_gradient_indices;
  SNOPTRun cur(d, &snopt_userfun_info);

  snopt::integer nx = prog.num_vars();
  d->min_alloc_x(nx);
  snopt::doublereal* x = d->x.data();
  snopt::doublereal* xlow = d->xlow.data();
  snopt::doublereal* xupp = d->xupp.data();
  for (int i = 0; i < nx; i++) {
    if (!std::isnan(initial_guess(i))) {
      x[i] = static_cast<snopt::doublereal>(initial_guess(i));
    } else {
      x[i] = 0.0;
    }
    xlow[i] = static_cast<snopt::doublereal>(
        -std::numeric_limits<double>::infinity());
    xupp[i] = static_cast<snopt::doublereal>(  // BR
        std::numeric_limits<double>::infinity());
  }
  for (auto const& binding : prog.bounding_box_constraints()) {
    const auto& c = binding.evaluator();
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

  // For linear complementary condition
  // 0 <= x ⊥ Mx + q >= 0
  // we add the bounding box constraint x >= 0
  for (const auto& binding : prog.linear_complementarity_constraints()) {
    for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
      const size_t vk_index =
          prog.FindDecisionVariableIndex(binding.variables()(k));
      xlow[vk_index] =
          std::max<snopt::doublereal>(xlow[vk_index], snopt::doublereal(0));
    }
  }


  int num_nonlinear_constraints = 0;
  int max_num_gradients = cost_gradient_indices.size();
  UpdateNumNonlinearConstraintsAndGradients(prog.generic_constraints(),
                                            &num_nonlinear_constraints,
                                            &max_num_gradients);
  UpdateNumNonlinearConstraintsAndGradients(prog.lorentz_cone_constraints(),
                                            &num_nonlinear_constraints,
                                            &max_num_gradients);
  UpdateNumNonlinearConstraintsAndGradients(
      prog.rotated_lorentz_cone_constraints(), &num_nonlinear_constraints,
      &max_num_gradients);
  UpdateNumNonlinearConstraintsAndGradients(
      prog.linear_complementarity_constraints(), &num_nonlinear_constraints,
      &max_num_gradients);

  int num_linear_constraints = 0;
  const auto linear_constraints = prog.GetAllLinearConstraints();
  for (auto const& binding : linear_constraints) {
    num_linear_constraints += binding.evaluator()->num_constraints();
  }

  // For linear complementary condition
  // 0 <= x ⊥ Mx + q >= 0
  // The linear constraint we add is Mx + q >= 0, so we will append
  // M.rows() rows to the linear constraints.
  for (const auto& binding : prog.linear_complementarity_constraints()) {
    num_linear_constraints += binding.evaluator()->M().rows();
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
  size_t grad_index = 0;
  for (const auto cost_gradient_index : cost_gradient_indices) {
    iGfun[grad_index] = 1;
    jGvar[grad_index] = cost_gradient_index + 1;
    ++grad_index;
  }

  size_t constraint_index = 1;  // constraint index starts at 1
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
  UpdateConstraintBoundsAndGradients(
      prog, prog.linear_complementarity_constraints(), Flow, Fupp, iGfun, jGvar,
      &constraint_index, &grad_index);

  // http://eigen.tuxfamily.org/dox/group__TutorialSparse.html
  typedef Eigen::Triplet<double> T;
  std::vector<T> tripletList;
  tripletList.reserve(num_linear_constraints * prog.num_vars());

  size_t linear_constraint_index = 0;
  UpdateLinearConstraint(prog, linear_constraints, &tripletList, Flow, Fupp,
                         &constraint_index, &linear_constraint_index);
  UpdateLinearConstraint(prog, prog.linear_complementarity_constraints(),
                         &tripletList, Flow, Fupp, &constraint_index,
                         &linear_constraint_index);

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

  // Determines if we should print out snopt debugging info.
  const std::unordered_map<std::string, std::string>& snopt_option_str =
      merged_options.GetOptionsStr(id());
  const auto print_file_it = snopt_option_str.find("Print file");
  if (print_file_it != snopt_option_str.end()) {
    std::string print_file_name(print_file_it->second);
    cur.iPrint = 9;
    snopt::integer print_file_name_len =
        static_cast<snopt::integer>(print_file_name.length());
    snopt::integer inform;
    snopt::snopenappend_(&cur.iPrint, &(print_file_name[0]), &inform,
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

  for (const auto it : merged_options.GetOptionsDouble(id())) {
    cur.snSetr(it.first, it.second);
  }

  for (const auto it : merged_options.GetOptionsInt(id())) {
    cur.snSeti(it.first, it.second);
  }

  snopt::integer info;
  snopt::snopta_(
      &Cold, &nF, &nx, &nxname, &nFname, &ObjAdd, &ObjRow, Prob,
      reinterpret_cast<snopt::U_fp>(&snopt_userfun),
      iAfun, jAvar, &lenA, &lenA, A, iGfun, jGvar, &lenG, &lenG, xlow, xupp,
      xnames, Flow, Fupp, Fnames, x, xstate, xmul, F, Fstate, Fmul, &info,
      &mincw, &miniw, &minrw, &nS, &nInf, &sInf,
      // Pass the snopt workspace as the user workspace.  We already set
      // the maxcu option to reserve some of it for our user code.
      d->cw.data(), &d->lencw, d->iw.data(), &d->leniw, d->rw.data(), &d->lenrw,
      d->cw.data(), &d->lencw, d->iw.data(), &d->leniw, d->rw.data(), &d->lenrw,
      npname, 8 * nxname, 8 * nFname, 8 * d->lencw, 8 * d->lencw);

  // Populate our results structure.
  SnoptSolverDetails& solver_details =
      result->SetSolverDetailsType<SnoptSolverDetails>();
  solver_details.info = info;
  solver_details.xmul = Eigen::Map<Eigen::VectorXd>(xmul, nx);
  solver_details.F = Eigen::Map<Eigen::VectorXd>(F, nF);
  solver_details.Fmul = Eigen::Map<Eigen::VectorXd>(Fmul, nF);
  SolutionResult solution_result{SolutionResult::kUnknownError};
  if (info >= 1 && info <= 6) {
    solution_result = SolutionResult::kSolutionFound;
  } else {
    drake::log()->debug("Snopt returns code {}\n", info);
    if (info >= 11 && info <= 16) {
      solution_result = SolutionResult::kInfeasibleConstraints;
    } else if (info >= 20 && info <= 22) {
      solution_result = SolutionResult::kUnbounded;
    } else if (info >= 30 && info <= 32) {
      solution_result = SolutionResult::kIterationLimit;
    } else if (info == 91) {
      solution_result = SolutionResult::kInvalidInput;
    }
  }
  result->set_solution_result(solution_result);
  result->set_x_val(
      Eigen::Map<VectorX<snopt::doublereal>>(x, nx).cast<double>());
  if (solution_result == SolutionResult::kUnbounded) {
    result->set_optimal_cost(MathematicalProgram::kUnboundedCost);
  } else {
    result->set_optimal_cost(*F);
  }
}

bool SnoptSolver::is_thread_safe() { return false; }

bool SnoptSolver::is_bounded_lp_broken() { return false; }

}  // namespace solvers
}  // namespace drake

#pragma GCC diagnostic pop  // "-Wunused-parameter"
