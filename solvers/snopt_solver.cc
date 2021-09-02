#include "drake/solvers/snopt_solver.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

// NOLINTNEXTLINE(build/include)
#include "snopt.h"

#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"
#include "drake/solvers/mathematical_program.h"

// TODO(jwnimmer-tri) Eventually resolve these warnings.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// todo(sammy-tri) :  return more information that just the solution (INFO,
// infeasible constraints, ...)
// todo(sammy-tri) :  avoid all dynamic allocation


namespace {

// Fortran has a pool of integers, which it uses as file handles for debug
// output.  When "Print file" output is enabled, we will need to tell SNOPT
// which integer to use for a given thread's debug output, so therefore we
// maintain a singleton pool of those integers here.
// See also http://fortranwiki.org/fortran/show/newunit.
class FortranUnitFactory {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FortranUnitFactory)

  static FortranUnitFactory& singleton() {
    static drake::never_destroyed<FortranUnitFactory> result;
    return result.access();
  }

  // Default constructor; don't call this -- only use the singleton().
  // (We can't mark this private, due to never_destroyed's use of it.)
  FortranUnitFactory() {
    // Populate the pool; we'll work from the back (i.e., starting with 10).
    // The range 10..1000 is borrowed from SNOPT 7.6's snopt-interface code,
    // also found at http://fortranwiki.org/fortran/show/newunit.
    for (int i = 10; i < 1000; ++i) {
      available_units_.push_front(i);
    }
  }

  // Returns an available new unit.
  int Allocate() {
    std::lock_guard<std::mutex> guard(mutex_);
    DRAKE_DEMAND(!available_units_.empty());
    int result = available_units_.back();
    available_units_.pop_back();
    return result;
  }

  // Reclaims a unit, returning it to the pool.
  void Release(int unit) {
    DRAKE_DEMAND(unit != 0);
    std::lock_guard<std::mutex> guard(mutex_);
    available_units_.push_back(unit);
  }

 private:
  std::mutex mutex_;
  std::deque<int> available_units_;
};

// This struct is a helper to bridge the gap between SNOPT's 7.4 and 7.6 APIs.
// Its specializations provide static methods that express the SNOPT 7.6 APIs.
// When compiled using SNOPT 7.6, these static methods are mere aliases to the
// underlying SNOPT 7.6 functions.  When compiled using SNOPT 7.4, these static
// methods rewrite the arguments and call the older 7.4 APIs.
template <bool is_snopt_76>
struct SnoptImpl {};

// This is the SNOPT 7.6 implementation.  It just aliases the function pointers.
template<>
struct SnoptImpl<true> {
#pragma GCC diagnostic push  // Silence spurious warnings from macOS llvm.
#pragma GCC diagnostic ignored "-Wpragmas"
#pragma GCC diagnostic ignored "-Wunused-const-variable"
  static constexpr auto snend = ::f_snend;
  static constexpr auto sninit = ::f_sninit;
  static constexpr auto snkera = ::f_snkera;
  static constexpr auto snmema = ::f_snmema;
  static constexpr auto snseti = ::f_snseti;
  static constexpr auto snsetr = ::f_snsetr;
  static constexpr auto snset = ::f_snset;
#pragma GCC diagnostic pop
};

// This is the SNOPT 7.4 implementation.
//
// It re-spells the 7.6-style arguments into 7.4-style calls, with the most
// common change being that int and double are passed by-value in 7.6 and
// by-mutable-pointer in 7.4.
//
// Below, we use `Int* iw` as a template argument (instead of `int* iw`), so
// that SFINAE ignores the function bodies when the user has SNOPT 7.6.
template<>
struct SnoptImpl<false> {
  // The unit number for our "Print file" log for the current thread.  When the
  // "Print file" option is not enabled, this is zero.
  thread_local inline static int g_iprint;

  template <typename Int>
  static void snend(
      Int* iw, int leniw, double* rw, int lenrw) {
    // Close the print file and then release its unit (if necessary).
    Int iprint = g_iprint;
    ::f_snend(&iprint);
    if (g_iprint) {
      FortranUnitFactory::singleton().Release(g_iprint);
      g_iprint = 0;
    }
  }
  template <typename Int>
  static void sninit(
      const char* name, int len, int summOn,
      Int* iw, int leniw, double* rw, int lenrw) {
    // Allocate a unit number for the "Print file" (if necessary); the code
    // within f_sninit will open the file.
    if (len == 0) {
      g_iprint = 0;
    } else {
      g_iprint = FortranUnitFactory::singleton().Allocate();
    }
    Int iprint = g_iprint;
    ::f_sninit(name, &len, &iprint, &summOn, iw, &leniw, rw, &lenrw);
  }
  template <typename Int>
  static void snkera(
      int start, const char* name,
      int nf, int n, double objadd, int objrow,
      snFunA usrfun, isnLog snLog, isnLog2 snLog2,
      isqLog sqLog, isnSTOP snSTOP,
      int* iAfun, int* jAvar, int neA, double* A,
      int* iGfun, int* jGvar, int neG,
      double* xlow, double* xupp,
      double* flow, double* fupp,
      double* x, int* xstate, double* xmul,
      double* f, int* fstate, double* fmul,
      int* inform, int* ns, int* ninf, double* sinf,
      int* miniw, int* minrw,
      int* iu, int leniu, double* ru, int lenru,
      Int* iw, int leniw, double* rw, int lenrw) {
    ::f_snkera(
         &start, name,
         &nf, &n, &objadd, &objrow,
         usrfun, snLog, snLog2, sqLog, snSTOP,
         iAfun, jAvar, &neA, A,
         iGfun, jGvar, &neG,
         xlow, xupp,
         flow, fupp,
         x, xstate, xmul,
         f, fstate, fmul,
         inform, ns, ninf, sinf,
         miniw,  minrw,
         iu, &leniu,
         ru, &lenru,
         iw, &leniw,
         rw, &lenrw);
  }
  template <typename Int>
  static void snmema(
      int* info, int nf, int n, int neA, int neG, int* miniw, int* minrw,
      Int* iw, int leniw, double* rw, int lenrw) {
    ::f_snmema(info, &nf, &n, &neA, &neG, miniw, minrw, iw, &leniw, rw, &lenrw);
  }
  template <typename Int>
  static void snseti(
      const char* buffer, int len, int iopt, int* errors,
      Int* iw, int leniw, double* rw, int lenrw) {
    ::f_snseti(buffer, &len, &iopt, errors, iw, &leniw, rw, &lenrw);
  }
  template <typename Int>
  static void snsetr(
      const char* buffer, int len, double rvalue, int* errors,
      Int* iw, int leniw, double* rw, int lenrw) {
    ::f_snsetr(buffer, &len, &rvalue, errors, iw, &leniw, rw, &lenrw);
  }
  template <typename Int>
  static void snset(const char* buffer, int len, int* errors,
      Int* iw, int leniw, double* rw, int lenrw) {
    ::f_snset(buffer, &len, errors, iw, &leniw, rw, &lenrw);
  }
};

// Choose the correct SnoptImpl specialization.
#pragma GCC diagnostic push  // Silence spurious warnings from macOS llvm.
#pragma GCC diagnostic ignored "-Wpragmas"
#pragma GCC diagnostic ignored "-Wunneeded-internal-declaration"
void f_sninit_76_prototype(const char*, int, int, int[], int, double[], int) {}
#pragma GCC diagnostic pop
const bool kIsSnopt76 =
    std::is_same_v<decltype(&f_sninit), decltype(&f_sninit_76_prototype)>;
using Snopt = SnoptImpl<kIsSnopt76>;

}  // namespace

namespace drake {
namespace solvers {
namespace {

// This class is used for passing additional info to the snopt_userfun, which
// evaluates the value and gradient of the cost and constraints. Apart from the
// standard information such as decision variable values, snopt_userfun could
// rely on additional information such as the cost gradient sparsity pattern.
//
// In order to use access class in SNOPT's userfun callback, we need to provide
// for a pointer to this object.  SNOPT's C interface does not allow using the
// char workspace, as explained in http://ccom.ucsd.edu/~optimizers/usage/c/ --
// "due to the incompatibility of strings in Fortran/C, all character arrays
// are disabled." Therefore, we use the integer user data (`int iu[]`) instead.
class SnoptUserFunInfo {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SnoptUserFunInfo)

  // Pointers to the parameters ('prog' and 'nonlinear_cost_gradient_indices')
  // are retained internally, so the supplied objects must have lifetimes longer
  // than the SnoptUserFuncInfo object.
  explicit SnoptUserFunInfo(const MathematicalProgram* prog)
      : this_pointer_as_int_array_(MakeThisAsInts()),
        prog_(*prog) {}

  const MathematicalProgram& mathematical_program() const { return prog_; }

  std::set<int>& nonlinear_cost_gradient_indices() {
    return nonlinear_cost_gradient_indices_;
  }
  const std::set<int>& nonlinear_cost_gradient_indices() const {
    return nonlinear_cost_gradient_indices_;
  }

  int* iu() const {
    return const_cast<int*>(this_pointer_as_int_array_.data());
  }
  int leniu() const {
    return this_pointer_as_int_array_.size();
  }

  // Converts the `int iu[]` data back into a reference to this class.
  static const SnoptUserFunInfo& GetFrom(const int* iu, int leniu) {
    DRAKE_ASSERT(iu != nullptr);
    DRAKE_ASSERT(leniu == kIntCount);

    const SnoptUserFunInfo* result = nullptr;
    std::copy(reinterpret_cast<const char*>(iu),
              reinterpret_cast<const char*>(iu) + sizeof(result),
              reinterpret_cast<char*>(&result));

    DRAKE_ASSERT(result != nullptr);
    return *result;
  }

 private:
  // We need this many `int`s to store a pointer.  Round up any fractional
  // remainder.
  static constexpr size_t kIntCount =
      (sizeof(SnoptUserFunInfo*) + sizeof(int) - 1) / sizeof(int);

  // Converts the `this` pointer into an integer array.
  std::array<int, kIntCount> MakeThisAsInts() {
    std::array<int, kIntCount> result;
    result.fill(0);

    const SnoptUserFunInfo* const value = this;
    std::copy(reinterpret_cast<const char*>(&value),
              reinterpret_cast<const char*>(&value) + sizeof(value),
              reinterpret_cast<char*>(result.data()));

    return result;
  }

  const std::array<int, kIntCount> this_pointer_as_int_array_;
  const MathematicalProgram& prog_;
  std::set<int> nonlinear_cost_gradient_indices_;
};

// Storage that we pass in and out of SNOPT APIs.
class WorkspaceStorage {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WorkspaceStorage)

  explicit WorkspaceStorage(const SnoptUserFunInfo* user_info)
      : user_info_(user_info) {
    DRAKE_DEMAND(user_info_ != nullptr);
    iw_.resize(500);
    rw_.resize(500);
  }

  int* iw() { return iw_.data(); }
  int leniw() const { return iw_.size(); }
  void resize_iw(int size) { iw_.resize(size); }

  double* rw() { return rw_.data(); }
  int lenrw() const { return rw_.size(); }
  void resize_rw(int size) { rw_.resize(size); }

  int* iu() { return user_info_->iu(); }
  int leniu() const { return user_info_->leniu(); }

  double* ru() { return nullptr; }
  int lenru() const { return 0; }

 private:
  std::vector<int> iw_;
  std::vector<double> rw_;

  const SnoptUserFunInfo* const user_info_;
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
void EvaluateSingleNonlinearConstraint(
    const C& constraint, const Eigen::Ref<const AutoDiffVecXd>& tx,
    AutoDiffVecXd* ty) {
  ty->resize(SingleNonlinearConstraintSize(constraint));
  constraint.Eval(tx, ty);
}

template <>
void EvaluateSingleNonlinearConstraint<LinearComplementarityConstraint>(
    const LinearComplementarityConstraint& constraint,
    const Eigen::Ref<const AutoDiffVecXd>& tx, AutoDiffVecXd* ty) {
  ty->resize(1);
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
 * @param xvec the value of the decision variables.
 */
template <typename C>
void EvaluateNonlinearConstraints(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& constraint_list, double F[], double G[],
    size_t* constraint_index, size_t* grad_index, const Eigen::VectorXd& xvec) {
  const auto & scale_map = prog.GetVariableScaling();
  Eigen::VectorXd this_x;
  for (const auto& binding : constraint_list) {
    const auto& c = binding.evaluator();
    int num_constraints = SingleNonlinearConstraintSize(*c);

    const int num_variables = binding.GetNumElements();
    this_x.resize(num_variables);
    // binding_var_indices[i] is the index of binding.variables()(i) in prog's
    // decision variables.
    std::vector<int> binding_var_indices(num_variables);
    for (int i = 0; i < num_variables; ++i) {
      binding_var_indices[i] =
          prog.FindDecisionVariableIndex(binding.variables()(i));
      this_x(i) = xvec(binding_var_indices[i]);
    }

    // Scale this_x
    auto this_x_scaled = math::InitializeAutoDiff(this_x);
    for (int i = 0; i < num_variables; i++) {
      auto it = scale_map.find(binding_var_indices[i]);
      if (it != scale_map.end()) {
        this_x_scaled(i) *= it->second;
      }
    }

    AutoDiffVecXd ty;
    ty.resize(num_constraints);
    EvaluateSingleNonlinearConstraint(*c, this_x_scaled, &ty);

    for (int i = 0; i < num_constraints; i++) {
      F[(*constraint_index)++] = ty(i).value();
    }

    const std::optional<std::vector<std::pair<int, int>>>&
        gradient_sparsity_pattern =
            binding.evaluator()->gradient_sparsity_pattern();
    if (gradient_sparsity_pattern.has_value()) {
      for (const auto& nonzero_entry : gradient_sparsity_pattern.value()) {
        G[(*grad_index)++] =
            ty(nonzero_entry.first).derivatives().size() > 0
                ? ty(nonzero_entry.first).derivatives()(nonzero_entry.second)
                : 0.0;
      }
    } else {
      for (int i = 0; i < num_constraints; i++) {
        if (ty(i).derivatives().size() > 0) {
          for (int j = 0; j < num_variables; ++j) {
            G[(*grad_index)++] = ty(i).derivatives()(j);
          }
        } else {
          for (int j = 0; j < num_variables; ++j) {
            G[(*grad_index)++] = 0.0;
          }
        }
      }
    }
  }
}

// Find the variables with non-zero gradient in @p costs, and add the indices of
// these variable to cost_gradient_indices.
template <typename C>
void GetNonlinearCostNonzeroGradientIndices(
    const MathematicalProgram& prog, const std::vector<Binding<C>>& costs,
    std::set<int>* cost_gradient_indices) {
  for (const auto& cost : costs) {
    for (int i = 0; i < static_cast<int>(cost.GetNumElements()); ++i) {
      cost_gradient_indices->insert(
          prog.FindDecisionVariableIndex(cost.variables()(i)));
    }
  }
}

/*
 * Loops through each nonlinear cost stored in MathematicalProgram, and obtains
 * the indices of the non-zero gradient, in the summed cost.
 */
std::set<int> GetAllNonlinearCostNonzeroGradientIndices(
    const MathematicalProgram& prog) {
  // The nonlinear costs include the quadratic and generic costs. Linear costs
  // are not included. In the future if we support more costs (like polynomial
  // costs), we should add it here.
  std::set<int> cost_gradient_indices;
  GetNonlinearCostNonzeroGradientIndices(prog, prog.quadratic_costs(),
                                         &cost_gradient_indices);
  GetNonlinearCostNonzeroGradientIndices(prog, prog.generic_costs(),
                                         &cost_gradient_indices);
  return cost_gradient_indices;
}

/*
 * Evaluates all the nonlinear costs, adds the value of the costs to
 * @p total_cost, and also adds the gradients to @p nonlinear_cost_gradients.
 */
template <typename C>
void EvaluateAndAddNonlinearCosts(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& nonlinear_costs, const Eigen::VectorXd& x,
    double* total_cost, std::vector<double>* nonlinear_cost_gradients) {
  const auto & scale_map = prog.GetVariableScaling();
  for (const auto& binding : nonlinear_costs) {
    const auto& obj = binding.evaluator();
    const int num_variables = binding.GetNumElements();

    Eigen::VectorXd this_x(num_variables);
    // binding_var_indices[i] is the index of binding.variables()(i) in prog's
    // decision variables.
    std::vector<int> binding_var_indices(num_variables);
    for (int i = 0; i < num_variables; ++i) {
      binding_var_indices[i] =
          prog.FindDecisionVariableIndex(binding.variables()(i));
      this_x(i) = x(binding_var_indices[i]);
    }
    AutoDiffVecXd ty(1);
    // Scale this_x
    auto this_x_scaled = math::InitializeAutoDiff(this_x);
    for (int i = 0; i < num_variables; i++) {
      auto it = scale_map.find(binding_var_indices[i]);
      if (it != scale_map.end()) {
        this_x_scaled(i) *= it->second;
      }
    }
    obj->Eval(this_x_scaled, &ty);

    *total_cost += ty(0).value();
    if (ty(0).derivatives().size() > 0) {
      for (int i = 0; i < num_variables; ++i) {
        (*nonlinear_cost_gradients)[binding_var_indices[i]] +=
            ty(0).derivatives()(i);
      }
    }
  }
}

// Evaluates all nonlinear costs, including the quadratic and generic costs.
// SNOPT stores the value of the total cost in F[0], and the nonzero gradient
// in array G. After calling this function, G[0], G[1], ..., G[grad_index-1]
// will store the nonzero gradient of the cost.
void EvaluateAllNonlinearCosts(
    const MathematicalProgram& prog, const Eigen::VectorXd& xvec,
    const std::set<int>& nonlinear_cost_gradient_indices, double F[],
    double G[], size_t* grad_index) {
  std::vector<double> cost_gradients(prog.num_vars(), 0);
  // Quadratic costs.
  EvaluateAndAddNonlinearCosts(prog, prog.quadratic_costs(), xvec, &(F[0]),
                               &cost_gradients);
  // Generic costs.
  EvaluateAndAddNonlinearCosts(prog, prog.generic_costs(), xvec, &(F[0]),
                               &cost_gradients);

  for (const int cost_gradient_index : nonlinear_cost_gradient_indices) {
    G[*grad_index] = cost_gradients[cost_gradient_index];
    ++(*grad_index);
  }
}

void snopt_userfun(int* Status, int* n, double x[], int* needF, int* neF,
                   double F[], int* needG, int* neG, double G[], char* cu,
                   int* lencu, int iu[], int* leniu, double ru[], int* lenru) {
  const SnoptUserFunInfo& info = SnoptUserFunInfo::GetFrom(iu, *leniu);
  const MathematicalProgram& current_problem = info.mathematical_program();
  const auto & scale_map = current_problem.GetVariableScaling();

  Eigen::VectorXd xvec(*n);
  for (int i = 0; i < *n; i++) {
    xvec(i) = x[i];
  }

  F[0] = 0.0;
  memset(G, 0, (*n) * sizeof(double));

  size_t grad_index = 0;

  // Scale xvec
  Eigen::VectorXd xvec_scaled = xvec;
  for (const auto & member : scale_map) {
    xvec_scaled(member.first) *= member.second;
  }
  current_problem.EvalVisualizationCallbacks(xvec_scaled);

  EvaluateAllNonlinearCosts(current_problem, xvec,
                            info.nonlinear_cost_gradient_indices(), F, G,
                            &grad_index);

  // The constraint index starts at 1 because the cost is the
  // first row.
  size_t constraint_index = 1;
  // The gradient_index also starts after the cost.
  EvaluateNonlinearConstraints(current_problem,
                               current_problem.generic_constraints(), F, G,
                               &constraint_index, &grad_index, xvec);
  EvaluateNonlinearConstraints(current_problem,
                               current_problem.lorentz_cone_constraints(), F, G,
                               &constraint_index, &grad_index, xvec);
  EvaluateNonlinearConstraints(
      current_problem, current_problem.rotated_lorentz_cone_constraints(), F, G,
      &constraint_index, &grad_index, xvec);
  EvaluateNonlinearConstraints(
      current_problem, current_problem.linear_complementarity_constraints(), F,
      G, &constraint_index, &grad_index, xvec);
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
    int* num_nonlinear_constraints, int* max_num_gradients,
    std::unordered_map<Binding<Constraint>, int>* constraint_dual_start_index) {
  for (auto const& binding : constraint_list) {
    auto const& c = binding.evaluator();
    int n = c->num_constraints();
    if (binding.evaluator()->gradient_sparsity_pattern().has_value()) {
      *max_num_gradients += static_cast<int>(
          binding.evaluator()->gradient_sparsity_pattern().value().size());
    } else {
      *max_num_gradients += n * binding.GetNumElements();
    }
    const Binding<Constraint> binding_cast =
        internal::BindingDynamicCast<Constraint>(binding);
    constraint_dual_start_index->emplace(binding_cast,
                                         1 + *num_nonlinear_constraints);
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
    int* num_nonlinear_constraints, int* max_num_gradients,
    std::unordered_map<Binding<Constraint>, int>* constraint_dual_start_index) {
  *num_nonlinear_constraints += static_cast<int>(constraint_list.size());
  for (const auto& binding : constraint_list) {
    *max_num_gradients += binding.evaluator()->M().rows();
  }
}

template <typename C>
void UpdateConstraintBoundsAndGradients(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& constraint_list, std::vector<double>* Flow,
    std::vector<double>* Fupp, std::vector<int>* iGfun, std::vector<int>* jGvar,
    size_t* constraint_index, size_t* grad_index) {
  for (auto const& binding : constraint_list) {
    auto const& c = binding.evaluator();
    int n = c->num_constraints();

    auto const lb = c->lower_bound(), ub = c->upper_bound();
    for (int i = 0; i < n; i++) {
      const int constraint_index_i = *constraint_index + i;
      (*Flow)[constraint_index_i] = lb(i);
      (*Fupp)[constraint_index_i] = ub(i);
    }

    const std::vector<int> bound_var_indices_in_prog =
        prog.FindDecisionVariableIndices(binding.variables());

    const std::optional<std::vector<std::pair<int, int>>>&
        gradient_sparsity_pattern =
            binding.evaluator()->gradient_sparsity_pattern();
    if (gradient_sparsity_pattern.has_value()) {
      for (const auto& nonzero_entry : gradient_sparsity_pattern.value()) {
        // Fortran is 1-indexed.
        (*iGfun)[*grad_index] = 1 + *constraint_index + nonzero_entry.first;
        (*jGvar)[*grad_index] =
            1 + bound_var_indices_in_prog[nonzero_entry.second];
        (*grad_index)++;
      }
    } else {
      for (int i = 0; i < n; i++) {
        for (int j = 0; j < static_cast<int>(binding.GetNumElements()); ++j) {
          // Fortran is 1-indexed.
          (*iGfun)[*grad_index] = 1 + *constraint_index + i;  // row order
          (*jGvar)[*grad_index] = 1 + bound_var_indices_in_prog[j];
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
    std::vector<double>* Flow, std::vector<double>* Fupp,
    std::vector<int>* iGfun, std::vector<int>* jGvar, size_t* constraint_index,
    size_t* grad_index) {
  for (const auto& binding : constraint_list) {
    (*Flow)[*constraint_index] = 0;
    (*Fupp)[*constraint_index] = 0;
    for (int j = 0; j < binding.evaluator()->M().rows(); ++j) {
      // Fortran is 1-indexed.
      (*iGfun)[*grad_index] = 1 + *constraint_index;
      (*jGvar)[*grad_index] =
          1 + prog.FindDecisionVariableIndex(binding.variables()(j));
      (*grad_index)++;
    }
    ++(*constraint_index);
  }
}

template <typename C>
Eigen::SparseMatrix<double> LinearEvaluatorA(const C& evaluator) {
  return evaluator.GetSparseMatrix();
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
Eigen::SparseMatrix<double> LinearEvaluatorA<LinearComplementarityConstraint>(
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

void UpdateLinearCost(
    const MathematicalProgram& prog,
    std::unordered_map<int, double>* variable_to_coefficient_map,
    double* linear_cost_constant_term) {
  const auto & scale_map = prog.GetVariableScaling();
  double scale = 1;
  for (const auto& binding : prog.linear_costs()) {
    *linear_cost_constant_term += binding.evaluator()->b();
    for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
      const int variable_index =
          prog.FindDecisionVariableIndex(binding.variables()(k));
      // Scale the coefficient as well
      auto it = scale_map.find(variable_index);
      if (it != scale_map.end()) {
        scale = it->second;
      } else {
        scale = 1;
      }
      (*variable_to_coefficient_map)[variable_index] +=
          binding.evaluator()->a()(k) * scale;
    }
  }
}

template <typename C>
void UpdateLinearConstraint(const MathematicalProgram& prog,
                            const std::vector<Binding<C>>& linear_constraints,
                            std::vector<Eigen::Triplet<double>>* tripletList,
                            std::vector<double>* Flow,
                            std::vector<double>* Fupp, size_t* constraint_index,
                            size_t* linear_constraint_index) {
  for (auto const& binding : linear_constraints) {
    auto const& c = binding.evaluator();
    int n = LinearConstraintSize(*c);

    const Eigen::SparseMatrix<double> A_constraint = LinearEvaluatorA(*c);

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
      const int constraint_index_i = *constraint_index + i;
      (*Flow)[constraint_index_i] = bounds.first(i);
      (*Fupp)[constraint_index_i] = bounds.second(i);
    }
    *constraint_index += n;
    *linear_constraint_index += n;
  }
}

using BoundingBoxDualIndices = std::pair<std::vector<int>, std::vector<int>>;

void SetVariableBounds(
    const MathematicalProgram& prog, std::vector<double>* xlow,
    std::vector<double>* xupp,
    std::unordered_map<Binding<BoundingBoxConstraint>, BoundingBoxDualIndices>*
        bb_con_dual_variable_indices) {
  // Set up the lower and upper bounds.
  for (auto const& binding : prog.bounding_box_constraints()) {
    const auto& c = binding.evaluator();
    const auto& lb = c->lower_bound();
    const auto& ub = c->upper_bound();

    for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
      const size_t vk_index =
          prog.FindDecisionVariableIndex(binding.variables()(k));
      (*xlow)[vk_index] = std::max(lb(k), (*xlow)[vk_index]);
      (*xupp)[vk_index] = std::min(ub(k), (*xupp)[vk_index]);
    }
  }
  // For linear complementary condition
  // 0 <= x ⊥ Mx + q >= 0
  // we add the bounding box constraint x >= 0
  for (const auto& binding : prog.linear_complementarity_constraints()) {
    for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
      const size_t vk_index =
          prog.FindDecisionVariableIndex(binding.variables()(k));
      (*xlow)[vk_index] = std::max((*xlow)[vk_index], 0.0);
    }
  }
  for (const auto& binding : prog.bounding_box_constraints()) {
    std::vector<int> lower_dual_indices(binding.evaluator()->num_constraints(),
                                        -1);
    std::vector<int> upper_dual_indices(binding.evaluator()->num_constraints(),
                                        -1);
    for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
      const int idx = prog.FindDecisionVariableIndex(binding.variables()(k));
      if ((*xlow)[idx] == binding.evaluator()->lower_bound()(k)) {
        lower_dual_indices[k] = idx;
      }
      if ((*xupp)[idx] == binding.evaluator()->upper_bound()(k)) {
        upper_dual_indices[k] = idx;
      }
    }
    bb_con_dual_variable_indices->emplace(
        binding, std::make_pair(lower_dual_indices, upper_dual_indices));
  }
  const auto& scale_map = prog.GetVariableScaling();
  // Scale lower and upper bounds
  for (const auto& member : scale_map) {
    (*xlow)[member.first] /= member.second;
    (*xupp)[member.first] /= member.second;
  }
}

void SetBoundingBoxConstraintDualSolution(
    const MathematicalProgram& prog, const Eigen::VectorXd& xmul,
    const std::unordered_map<Binding<BoundingBoxConstraint>,
                             BoundingBoxDualIndices>&
        bb_con_dual_variable_indices,
    MathematicalProgramResult* result) {
  const auto& scale_map = prog.GetVariableScaling();
  // If the variable x(i) is scaled by s, then its bounding box constraint
  // becoms lower / s <= x(i) <= upper / s. Perturbing the bounds of the
  // scaled constraint by eps is equivalent to pertubing the original bounds
  // by s * eps. Hence the shadow cost of the original constraint should be
  // divided by s.
  Eigen::VectorXd xmul_scaled_back = xmul;
  for (const auto& member : scale_map) {
    xmul_scaled_back(member.first) /= member.second;
  }
  for (const auto& binding : prog.bounding_box_constraints()) {
    std::vector<int> lower_dual_indices, upper_dual_indices;
    std::tie(lower_dual_indices, upper_dual_indices) =
        bb_con_dual_variable_indices.at(binding);
    Eigen::VectorXd dual_solution =
        Eigen::VectorXd::Zero(binding.GetNumElements());
    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      if (lower_dual_indices[i] != -1 && xmul(lower_dual_indices[i]) >= 0) {
        // When xmul >= 0, Snopt thinks the lower bound is active.
        dual_solution(i) = xmul_scaled_back(lower_dual_indices[i]);
      } else if (upper_dual_indices[i] != -1 &&
                 xmul(upper_dual_indices[i]) <= 0) {
        // When xmul <= 0, Snopt thinks the upper bound is active.
        dual_solution(i) = xmul_scaled_back(upper_dual_indices[i]);
      }
    }
    result->set_dual_solution(binding, dual_solution);
  }
}

template <typename C>
void SetConstraintDualSolution(
    const std::vector<Binding<C>>& bindings, const Eigen::VectorXd& Fmul,
    const std::unordered_map<Binding<Constraint>, int>&
        constraint_dual_start_index,
    MathematicalProgramResult* result) {
  for (const auto& binding : bindings) {
    const Binding<Constraint> binding_cast =
        internal::BindingDynamicCast<Constraint>(binding);

    result->set_dual_solution(
        binding, Fmul.segment(constraint_dual_start_index.at(binding),
                              binding.evaluator()->num_constraints()));
  }
}

void SetConstraintDualSolutions(
    const MathematicalProgram& prog, const Eigen::VectorXd& Fmul,
    const std::unordered_map<Binding<Constraint>, int>&
        constraint_dual_start_index,
    MathematicalProgramResult* result) {
  SetConstraintDualSolution(prog.generic_constraints(), Fmul,
                            constraint_dual_start_index, result);
  SetConstraintDualSolution(prog.lorentz_cone_constraints(), Fmul,
                            constraint_dual_start_index, result);
  SetConstraintDualSolution(prog.rotated_lorentz_cone_constraints(), Fmul,
                            constraint_dual_start_index, result);
  SetConstraintDualSolution(prog.rotated_lorentz_cone_constraints(), Fmul,
                            constraint_dual_start_index, result);
  SetConstraintDualSolution(prog.linear_constraints(), Fmul,
                            constraint_dual_start_index, result);
  SetConstraintDualSolution(prog.linear_equality_constraints(), Fmul,
                            constraint_dual_start_index, result);
}

SolutionResult MapSnoptInfoToSolutionResult(int snopt_info) {
  SolutionResult solution_result{SolutionResult::kUnknownError};
  // Please refer to User's Guide for SNOPT Verions 7, section 8.6 on the
  // meaning of these snopt_info.
  if (snopt_info >= 1 && snopt_info <= 6) {
    solution_result = SolutionResult::kSolutionFound;
  } else {
    log()->debug("Snopt returns code {}\n", snopt_info);
    if (snopt_info >= 11 && snopt_info <= 16) {
      solution_result = SolutionResult::kInfeasibleConstraints;
    } else if (snopt_info >= 20 && snopt_info <= 22) {
      solution_result = SolutionResult::kUnbounded;
    } else if (snopt_info >= 30 && snopt_info <= 32) {
      solution_result = SolutionResult::kIterationLimit;
    } else if (snopt_info == 91) {
      solution_result = SolutionResult::kInvalidInput;
    }
  }
  return solution_result;
}

void SetMathematicalProgramResult(
    const MathematicalProgram& prog, int snopt_status,
    const Eigen::VectorXd& x_val,
    const std::unordered_map<Binding<BoundingBoxConstraint>,
                             BoundingBoxDualIndices>&
        bb_con_dual_variable_indices,
    const std::unordered_map<Binding<Constraint>, int>&
        constraint_dual_start_index,
    double objective_constant, MathematicalProgramResult* result) {
  SnoptSolverDetails& solver_details =
      result->SetSolverDetailsType<SnoptSolverDetails>();
  // Populate our results structure.
  const SolutionResult solution_result =
      MapSnoptInfoToSolutionResult(snopt_status);
  result->set_solution_result(solution_result);
  result->set_x_val(x_val);
  SetBoundingBoxConstraintDualSolution(prog, solver_details.xmul,
                                       bb_con_dual_variable_indices, result);
  SetConstraintDualSolutions(prog, solver_details.Fmul,
                             constraint_dual_start_index, result);
  if (solution_result == SolutionResult::kUnbounded) {
    result->set_optimal_cost(MathematicalProgram::kUnboundedCost);
  } else {
    result->set_optimal_cost(solver_details.F(0) + objective_constant);
  }
}

void UpdateNumConstraintsAndGradients(
    const MathematicalProgram& prog,
    const std::vector<Binding<LinearConstraint>>& linear_constraints,
    int* num_nonlinear_constraints, int* num_linear_constraints,
    int* max_num_gradients,
    std::unordered_map<Binding<Constraint>, int>* constraint_dual_start_index) {
  UpdateNumNonlinearConstraintsAndGradients(
      prog.generic_constraints(), num_nonlinear_constraints, max_num_gradients,
      constraint_dual_start_index);
  UpdateNumNonlinearConstraintsAndGradients(
      prog.lorentz_cone_constraints(), num_nonlinear_constraints,
      max_num_gradients, constraint_dual_start_index);
  UpdateNumNonlinearConstraintsAndGradients(
      prog.rotated_lorentz_cone_constraints(), num_nonlinear_constraints,
      max_num_gradients, constraint_dual_start_index);
  UpdateNumNonlinearConstraintsAndGradients(
      prog.linear_complementarity_constraints(), num_nonlinear_constraints,
      max_num_gradients, constraint_dual_start_index);

  // Update linear constraints.
  for (auto const& binding : linear_constraints) {
    const Binding<Constraint> binding_cast =
        internal::BindingDynamicCast<Constraint>(binding);
    constraint_dual_start_index->emplace(
        binding_cast, 1 + *num_nonlinear_constraints + *num_linear_constraints);
    *num_linear_constraints += binding.evaluator()->num_constraints();
  }

  // For linear complementary condition
  // 0 <= x ⊥ Mx + q >= 0
  // The linear constraint we add is Mx + q >= 0, so we will append
  // M.rows() rows to the linear constraints.
  for (const auto& binding : prog.linear_complementarity_constraints()) {
    *num_linear_constraints += binding.evaluator()->M().rows();
  }
}

void SolveWithGivenOptions(
    const MathematicalProgram& prog,
    const Eigen::Ref<const Eigen::VectorXd>& x_init,
    const std::unordered_map<std::string, std::string>& snopt_options_string,
    const std::unordered_map<std::string, int>& snopt_options_int,
    const std::unordered_map<std::string, double>& snopt_options_double,
    const std::string& print_file_common, MathematicalProgramResult* result) {
  SnoptSolverDetails& solver_details =
      result->SetSolverDetailsType<SnoptSolverDetails>();

  SnoptUserFunInfo user_info(&prog);
  WorkspaceStorage storage(&user_info);
  const auto & scale_map = prog.GetVariableScaling();

  std::string print_file_name = print_file_common;
  const auto print_file_it = snopt_options_string.find("Print file");
  if (print_file_it != snopt_options_string.end()) {
    print_file_name = print_file_it->second;
  }
  Snopt::sninit(
      print_file_name.c_str(), print_file_name.length(), 0 /* no summary */,
      storage.iw(), storage.leniw(),
      storage.rw(), storage.lenrw());
  ScopeExit guard([&storage]() {
    Snopt::snend(
        storage.iw(), storage.leniw(),
        storage.rw(), storage.lenrw());
  });

  int nx = prog.num_vars();
  std::vector<double> x(nx, 0.0);
  std::vector<double> xlow(nx, -std::numeric_limits<double>::infinity());
  std::vector<double> xupp(nx, std::numeric_limits<double>::infinity());
  solver_details.xmul.resize(nx);
  solver_details.xmul.setZero();
  std::vector<int> xstate(nx, 0);

  // Initialize the guess for x.
  for (int i = 0; i < nx; ++i) {
    if (!std::isnan(x_init(i))) {
      x[i] = x_init(i);
    } else {
      x[i] = 0.0;
    }
  }
  // Scale initial guess
  for (const auto & member : scale_map) {
    x[member.first] /= member.second;
  }
  // bb_con_dual_variable_indices[constraint] maps the bounding box constraint
  // to the indices of its dual variables (one for lower bound and one for upper
  // bound). If this constraint doesn't have a dual variable (because the bound
  // is looser than some other bounding box constraint, hence this constraint
  // can never be active), then the index is set to -1.
  std::unordered_map<Binding<BoundingBoxConstraint>, BoundingBoxDualIndices>
      bb_con_dual_variable_indices;

  SetVariableBounds(prog, &xlow, &xupp, &bb_con_dual_variable_indices);

  // constraint_dual_start_index[constraint] stores the starting index of
  // the dual variables in Fmul.
  std::unordered_map<Binding<Constraint>, int> constraint_dual_start_index;

  // Update nonlinear constraints.
  user_info.nonlinear_cost_gradient_indices() =
      GetAllNonlinearCostNonzeroGradientIndices(prog);
  int num_nonlinear_constraints = 0;
  int max_num_gradients = user_info.nonlinear_cost_gradient_indices().size();
  int num_linear_constraints = 0;
  const auto linear_constraints = prog.GetAllLinearConstraints();
  UpdateNumConstraintsAndGradients(prog, linear_constraints,
                                   &num_nonlinear_constraints,
                                   &num_linear_constraints, &max_num_gradients,
                                   &constraint_dual_start_index);

  // Update the bound of the constraint.
  int nF = 1 + num_nonlinear_constraints + num_linear_constraints;
  solver_details.F.resize(nF);
  solver_details.F.setZero();
  std::vector<double> Flow(nF, -std::numeric_limits<double>::infinity());
  std::vector<double> Fupp(nF, std::numeric_limits<double>::infinity());
  solver_details.Fmul.resize(nF);
  solver_details.Fmul.setZero();
  std::vector<int> Fstate(nF, 0);

  // Set up the gradient sparsity pattern.
  int lenG = max_num_gradients;
  std::vector<int> iGfun(lenG, 0);
  std::vector<int> jGvar(lenG, 0);
  size_t grad_index = 0;
  for (const auto cost_gradient_index :
         user_info.nonlinear_cost_gradient_indices()) {
    // Fortran is 1-indexed.
    iGfun[grad_index] = 1;
    jGvar[grad_index] = 1 + cost_gradient_index;
    ++grad_index;
  }

  // constraint_index starts at 1 because row 0 is the cost.
  size_t constraint_index = 1;
  UpdateConstraintBoundsAndGradients(prog, prog.generic_constraints(), &Flow,
                                     &Fupp, &iGfun, &jGvar, &constraint_index,
                                     &grad_index);
  UpdateConstraintBoundsAndGradients(prog, prog.lorentz_cone_constraints(),
                                     &Flow, &Fupp, &iGfun, &jGvar,
                                     &constraint_index, &grad_index);
  UpdateConstraintBoundsAndGradients(
      prog, prog.rotated_lorentz_cone_constraints(), &Flow, &Fupp, &iGfun,
      &jGvar, &constraint_index, &grad_index);
  UpdateConstraintBoundsAndGradients(
      prog, prog.linear_complementarity_constraints(), &Flow, &Fupp, &iGfun,
      &jGvar, &constraint_index, &grad_index);

  // Now find the sparsity pattern of the linear constraints/costs, and also
  // update Flow and Fupp corresponding to the linear constraints.
  // We use Eigen::Triplet to store the non-zero entries in the linear
  // constraint
  // http://eigen.tuxfamily.org/dox/group__TutorialSparse.html
  typedef Eigen::Triplet<double> T;
  double linear_cost_constant_term = 0;
  std::unordered_map<int, double> variable_to_linear_cost_coefficient;
  UpdateLinearCost(prog, &variable_to_linear_cost_coefficient,
                   &linear_cost_constant_term);

  std::vector<T> linear_constraints_triplets;
  linear_constraints_triplets.reserve(num_linear_constraints * prog.num_vars());
  size_t linear_constraint_index = 0;
  UpdateLinearConstraint(prog, linear_constraints, &linear_constraints_triplets,
                         &Flow, &Fupp, &constraint_index,
                         &linear_constraint_index);
  UpdateLinearConstraint(prog, prog.linear_complementarity_constraints(),
                         &linear_constraints_triplets, &Flow, &Fupp,
                         &constraint_index, &linear_constraint_index);
  // Scale the matrix in linear constraints
  for (auto & triplet : linear_constraints_triplets) {
    auto it = scale_map.find(triplet.col());
    if (it != scale_map.end()) {
      triplet = Eigen::Triplet<double>(triplet.row(), triplet.col(),
        triplet.value()*(it->second));
    }
  }

  int lenA = variable_to_linear_cost_coefficient.size() +
             linear_constraints_triplets.size();
  std::vector<double> A(lenA, 0.0);
  std::vector<int> iAfun(lenA, 0);
  std::vector<int> jAvar(lenA, 0);
  size_t A_index = 0;
  for (const auto& it : variable_to_linear_cost_coefficient) {
    A[A_index] = it.second;
    // Fortran uses 1-index.
    iAfun[A_index] = 1;
    jAvar[A_index] = it.first + 1;
    A_index++;
  }
  for (const auto& it : linear_constraints_triplets) {
    A[A_index] = it.value();
    // Fortran is 1-indexed.
    iAfun[A_index] = 2 + num_nonlinear_constraints + it.row();
    jAvar[A_index] = 1 + it.col();
    A_index++;
  }

  for (const auto& it : snopt_options_double) {
    int errors = 0;
    Snopt::snsetr(
        it.first.c_str(), it.first.length(), it.second, &errors,
        storage.iw(), storage.leniw(),
        storage.rw(), storage.lenrw());
    if (errors > 0) {
      throw std::runtime_error("Error setting Snopt double parameter " +
                               it.first);
    }
  }

  for (const auto& it : snopt_options_int) {
    int errors = 0;
    Snopt::snseti(
        it.first.c_str(), it.first.length(), it.second, &errors,
        storage.iw(), storage.leniw(),
        storage.rw(), storage.lenrw());
    if (errors > 0) {
      throw std::runtime_error("Error setting Snopt integer parameter " +
                               it.first);
    }
  }

  for (const auto& it : snopt_options_string) {
    int errors = 0;
    auto option_string = it.first + " " + it.second;
    if (it.first == "Print file") {
      // Already handled during sninit, above
      continue;
    }
    Snopt::snset(
        option_string.c_str(), option_string.length(), &errors,
        storage.iw(), storage.leniw(),
        storage.rw(), storage.lenrw());
    if (errors > 0) {
      throw std::runtime_error("Error setting Snopt string parameter " +
                               it.first);
    }
  }

  int Cold = 0;
  double objective_constant = linear_cost_constant_term;
  // The index of the objective row among all function evaluation F (notice that
  // due to SNOPT using Fortran, this is 1-indexed).
  int ObjRow = 1;
  int nS = 0;
  int nInf{0};
  double sInf{0.0};

  // Reallocate int and real workspace.
  int miniw, minrw;
  int snopt_status{0};
  Snopt::snmema(&snopt_status, nF, nx, lenA, lenG, &miniw, &minrw, storage.iw(),
                storage.leniw(), storage.rw(), storage.lenrw());
  // TODO(jwnimmer-tri) Check snopt_status for errors.
  if (miniw > storage.leniw()) {
    storage.resize_iw(miniw);
    const std::string option = "Total int workspace";
    int errors;
    Snopt::snseti(
        option.c_str(), option.length(), storage.leniw(), &errors,
        storage.iw(), storage.leniw(),
        storage.rw(), storage.lenrw());
    // TODO(hongkai.dai): report the error in SnoptSolverDetails.
  }
  if (minrw > storage.lenrw()) {
    storage.resize_rw(minrw);
    const std::string option = "Total real workspace";
    int errors;
    Snopt::snseti(
        option.c_str(), option.length(), storage.lenrw(), &errors,
        storage.iw(), storage.leniw(),
        storage.rw(), storage.lenrw());
    // TODO(hongkai.dai): report the error in SnoptSolverDetails.
  }
  // Actual solve.
  const char problem_name[] = "drake_problem";
  // clang-format off
  Snopt::snkera(Cold, problem_name, nF, nx, objective_constant, ObjRow,
                snopt_userfun,
                nullptr /* isnLog snLog */, nullptr /* isnLog2 snLog2 */,
                nullptr /* isqLog sqLog */, nullptr /* isnSTOP snSTOP */,
                iAfun.data(), jAvar.data(), lenA, A.data(),
                iGfun.data(), jGvar.data(), lenG,
                xlow.data(), xupp.data(),
                Flow.data(), Fupp.data(),
                x.data(), xstate.data(), solver_details.xmul.data(),
                solver_details.F.data(), Fstate.data(),
                solver_details.Fmul.data(),
                &snopt_status, &nS, &nInf, &sInf, &miniw, &minrw,
                storage.iu(), storage.leniu(),
                storage.ru(), storage.lenru(),
                storage.iw(), storage.leniw(),
                storage.rw(), storage.lenrw());
  // clang-format on

  Eigen::VectorXd x_val = Eigen::Map<Eigen::VectorXd>(x.data(), nx);
  // Scale solution back
  for (const auto & member : scale_map) {
    x_val(member.first) *= member.second;
  }
  solver_details.info = snopt_status;

  SetMathematicalProgramResult(
      prog, snopt_status, x_val, bb_con_dual_variable_indices,
      constraint_dual_start_index, objective_constant, result);
}

}  // namespace

bool SnoptSolver::is_available() { return true; }

void SnoptSolver::DoSolve(
    const MathematicalProgram& prog,
    const Eigen::VectorXd& initial_guess,
    const SolverOptions& merged_options,
    MathematicalProgramResult* result) const {
  // Call SNOPT.
  std::unordered_map<std::string, int> int_options =
      merged_options.GetOptionsInt(id());

  // If "Timing level" is not zero, then snopt periodically calls etime to
  // determine it's usage of cpu time (which on Linux calls getrusage in the
  // libgfortran implementation).  Unfortunately getrusage is called using
  // RUSAGE_SELF, which has unfortunate consenquences when using threads,
  // namely (1) It returns the total count of CPU usage for all threads, so
  // the result is garbage, and (2) on Linux the kernel holds a process wide
  // lock inside getrusage when RUSAGE_SELF is specified, so other threads
  // using snopt end up blocking on their getrusage calls.  Under the theory
  // that a user who actually wants this behavior will turn it on
  // deliberately, set "Timing level" to zero if the user hasn't requested
  // another value.
  const std::string kTimingLevel = "Timing level";
  if (int_options.count(kTimingLevel) == 0) {
    int_options[kTimingLevel] = 0;
  }

  SolveWithGivenOptions(prog, initial_guess, merged_options.GetOptionsStr(id()),
                        int_options, merged_options.GetOptionsDouble(id()),
                        merged_options.get_print_file_name(), result);
}

bool SnoptSolver::is_bounded_lp_broken() { return true; }
}  // namespace solvers
}  // namespace drake

#pragma GCC diagnostic pop  // "-Wunused-parameter"
