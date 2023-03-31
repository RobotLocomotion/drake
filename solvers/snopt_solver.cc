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

#include <iostream>

#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"
#include "drake/solvers/mathematical_program.h"

// TODO(jwnimmer-tri) Eventually resolve these warnings.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

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
template <>
struct SnoptImpl<true> {
#pragma GCC diagnostic push  // Silence spurious warnings from macOS llvm.
#pragma GCC diagnostic ignored "-Wpragmas"
#pragma GCC diagnostic ignored "-Wunused-const-variable"
  static constexpr auto snclose = snopt::snclose_;
  static constexpr auto sninit = snopt::sninit_;
  static constexpr auto snopta = snopt::snopta_;
  static constexpr auto snmema = snopt::snmema_;
  static constexpr auto snseti = snopt::snseti_;
  static constexpr auto snsetr = snopt::snsetr_;
  static constexpr auto snset = snopt::snset_;
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
template <>
struct SnoptImpl<false> {
  // The unit number for our "Print file" log for the current thread.  When the
  // "Print file" option is not enabled, this is zero.
  thread_local inline static int g_iprint;

  static void snclose() {
    // Close the print file and then release its unit (if necessary).
    snopt::integer iprint = g_iprint;
    snopt::snclose_(&iprint);
    if (g_iprint) {
      FortranUnitFactory::singleton().Release(g_iprint);
      g_iprint = 0;
    }
  }
  template <typename Int>
  static void sninit(
      const char* name, int len, int summOn, char* cw, snopt::integer lencw,
      Int* iw, snopt::integer leniw, double* rw, snopt::integer lenrw) {
    // Allocate a unit number for the "Print file" (if necessary); the code
    // within f_sninit will open the file.
    if (len == 0) {
      g_iprint = 0;
    } else {
      g_iprint = FortranUnitFactory::singleton().Allocate();
    }
    snopt::integer iprint = static_cast<snopt::integer>(g_iprint);
    snopt::integer summ_on = static_cast<snopt::integer>(summOn);

    snopt::sninit_(&iprint, &summ_on, cw, &lencw, iw, &leniw, rw, &lenrw, 8 * lencw);

    // Set the print file name if enabled.
    if (iprint > 0) {
      snopt::integer errors;
      snopt::snopenappend_(&iprint, const_cast<char*>(name), &errors, len);
    }
  }
  // Turn clang-format off for readability.
  // clang-format off
  template <typename Int>
  static void snopta(
      snopt::integer start, char* name,
      snopt::integer nf, snopt::integer n, double objadd, snopt::integer objrow,
      snopt::U_fp usrfun,
      snopt::integer* iAfun, snopt::integer* jAvar, snopt::integer neA, double* A,
      snopt::integer* iGfun, snopt::integer* jGvar, snopt::integer neG,
      double* xlow, double* xupp,
      double* flow, double* fupp,
      double* x, snopt::integer* xstate, double* xmul,
      double* f, snopt::integer* fstate, double* fmul,
      snopt::integer* inform, snopt::integer* ns, snopt::integer* ninf, double* sinf,
      snopt::integer* mincw, snopt::integer* miniw, snopt::integer* minrw,
      snopt::integer* iu, snopt::integer leniu, double* ru, snopt::integer lenru,
      char* cw, snopt::integer lencw,
      Int* iw, snopt::integer leniw, double* rw, snopt::integer lenrw,
      snopt::integer nxname, snopt::integer nFname) {
    snopt::integer npname = strlen(name);
    char xnames[8 * 1];  // should match nxname
    char Fnames[8 * 1];  // should match nFname
    snopt::snopta_(
         &start,
         &nf, &n, &nxname, &nFname, &objadd, &objrow,
         name, usrfun,
         iAfun, jAvar, &neA, &neA, A,
         iGfun, jGvar, &neG, &neG,
         xlow, xupp, xnames,
         flow, fupp, Fnames,
         x, xstate, xmul,
         f, fstate, fmul,
         inform,
         mincw, miniw,  minrw,
         ns, ninf, sinf,
         cw, &lencw, iu, &leniu, ru, &lenru,
         cw, &lencw, iw, &leniw, rw, &lenrw,
         npname, 8 * nxname, 8 * nFname, 8 * lencw, 8 * lencw);
  }
  // clang-format on
  template <typename Int>
  static void snmema(
      snopt::integer* info,
      snopt::integer nf, snopt::integer n, snopt::integer neA, snopt::integer neG,
      snopt::integer* mincw, snopt::integer* miniw, snopt::integer* minrw,
      char* cw, snopt::integer lencw, Int* iw, snopt::integer leniw,
      double* rw, snopt::integer lenrw,
      snopt::integer* nxname, snopt::integer* nFname) {
    snopt::snmema_(info, &nf, &n, nxname, nFname, &neA, &neG, mincw, miniw, minrw,
                   cw, &lencw, iw, &leniw, rw, &lenrw, 8 * lencw);
  }
  template <typename Int>
  static void snseti(
      const char* buffer, int len, int ival, snopt::integer* errors,
      char* cw, snopt::integer lencw, Int* iw, snopt::integer leniw,
      double* rw, snopt::integer lenrw) {
        snopt::integer iPrint = -1;
        snopt::integer iSumm = -1;
        snopt::integer opt_val = static_cast<snopt::integer>(ival);
    snopt::snseti_(const_cast<char*>(buffer), &opt_val, &iPrint, &iSumm, errors, cw, &lencw,
                   iw, &leniw, rw, &lenrw, len, 8 * lencw);
  }
  template <typename Int>
  static void snsetr(
      const char* buffer, int len, double rvalue, snopt::integer* errors,
      char* cw, snopt::integer lencw, Int* iw, snopt::integer leniw,
      double* rw, snopt::integer lenrw) {
        snopt::integer iPrint = -1;
        snopt::integer iSumm = -1;
        snopt::doublereal r_val = static_cast<snopt::doublereal>(rvalue);
    snopt::snsetr_(const_cast<char*>(buffer), &r_val, &iPrint, &iSumm, errors, cw, &lencw,
                  iw, &leniw, rw, &lenrw, len, 8 * lencw);
  }
};

// Choose the correct SnoptImpl specialization.
#pragma GCC diagnostic push  // Silence spurious warnings from macOS llvm.
#pragma GCC diagnostic ignored "-Wpragmas"
#pragma GCC diagnostic ignored "-Wunneeded-internal-declaration"
void f_sninit_76_prototype(const char*, int, int, int[], int, double[], int) {}
#pragma GCC diagnostic pop
const bool kIsSnopt76 =
    std::is_same_v<decltype(&snopt::sninit_), decltype(&f_sninit_76_prototype)>;
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

  std::vector<int>& duplicate_to_G_index_map() {
    return duplicate_to_G_index_map_;
  }

  const std::vector<int>& duplicate_to_G_index_map() const {
    return duplicate_to_G_index_map_;
  }

  void set_lenG(int lenG) { lenG_ = lenG; }

  [[nodiscard]] int lenG() const { return lenG_; }

  // If and only if the userfun experiences an exception, the exception message
  // will be stashed here. All callers of snOptA or similar must check this to
  // find out if there were any errors.
  std::optional<std::string>& userfun_error_message() {
    return userfun_error_message_;
  }

  int* iu() const {
    return const_cast<int*>(this_pointer_as_int_array_.data());
  }
  int leniu() const {
    return this_pointer_as_int_array_.size();
  }

  // Converts the `int iu[]` data back into a reference to this class.
  static SnoptUserFunInfo& GetFrom(const snopt::integer* iu, int leniu) {
    DRAKE_ASSERT(iu != nullptr);
    DRAKE_ASSERT(leniu == kIntCount);

    SnoptUserFunInfo* result = nullptr;
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
  // When evaluating the nonlinear costs/constraints, the Binding could contain
  // duplicated variables. We need to sum up the entries in the gradient vector
  // corresponding to the same constraint and variables. We first evaluate all
  // costs and constraints, not accounting for duplicated variable. We denote
  // this gradient vector as G_w_duplicate. We then sum up the entries in
  // G_w_duplicate that correspond to the same constraint and gradient, and
  // denote the resulting gradient vector as G.
  // duplicate_to_G_index_map_ has the same length as G_w_duplicate. We add
  // G_w_duplicate[i] to G[duplicate_to_G_index_map[i]].
  std::vector<int> duplicate_to_G_index_map_;
  int lenG_;

  std::optional<std::string> userfun_error_message_;
};

// Storage that we pass in and out of SNOPT APIs.
class WorkspaceStorage {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WorkspaceStorage)

  explicit WorkspaceStorage(const SnoptUserFunInfo* user_info)
      : user_info_(user_info) {
    DRAKE_DEMAND(user_info_ != nullptr);
    iw_.resize(500 * 1000);
    rw_.resize(500 * 1000);
    cw_.resize(8 * 501);
  }

  snopt::integer* iw() { return reinterpret_cast<snopt::integer*>(iw_.data()); }
  int leniw() const { return iw_.size(); }
  void resize_iw(int size) { iw_.resize(size); }

  double* rw() { return rw_.data(); }
  int lenrw() const { return rw_.size(); }
  void resize_rw(int size) { rw_.resize(size); }

  char* cw() { return cw_.data(); }
  int lencw() const { return cw_.size(); }
  void resize_cw(int size) { cw_.resize(size); }

  snopt::integer* iu() { return reinterpret_cast<snopt::integer*>(user_info_->iu()); }
  int leniu() const { return user_info_->leniu(); }

  double* ru() { return nullptr; }
  int lenru() const { return 0; }

 private:
  std::vector<char> cw_;
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
// QuadraticConstraint, LorentzConeConstraint, RotatedLorentzConeConstraint, we
// call Eval function of the constraint directly. For some other constraint,
// such as LinearComplementaryConstraint, we will evaluate its nonlinear
// constraint differently, than its Eval function.
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
    const std::vector<Binding<C>>& constraint_list, double F[],
    std::vector<double>* G_w_duplicate, size_t* constraint_index,
    size_t* grad_index, const Eigen::VectorXd& xvec) {
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
        (*G_w_duplicate)[(*grad_index)++] =
            ty(nonzero_entry.first).derivatives().size() > 0
                ? ty(nonzero_entry.first).derivatives()(nonzero_entry.second)
                : 0.0;
      }
    } else {
      for (int i = 0; i < num_constraints; i++) {
        if (ty(i).derivatives().size() > 0) {
          for (int j = 0; j < num_variables; ++j) {
            (*G_w_duplicate)[(*grad_index)++] = ty(i).derivatives()(j);
          }
        } else {
          for (int j = 0; j < num_variables; ++j) {
            (*G_w_duplicate)[(*grad_index)++] = 0.0;
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
  // The nonlinear costs include the quadratic, L2, and generic costs. Linear
  // costs are not included. In the future if we support more costs (like
  // polynomial costs), we should add it here.
  std::set<int> cost_gradient_indices;
  GetNonlinearCostNonzeroGradientIndices(prog, prog.quadratic_costs(),
                                         &cost_gradient_indices);
  GetNonlinearCostNonzeroGradientIndices(prog, prog.l2norm_costs(),
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
  const auto& scale_map = prog.GetVariableScaling();
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
    std::vector<double>* G_w_duplicate, size_t* grad_index) {
  std::vector<double> cost_gradients(prog.num_vars(), 0);
  // Quadratic costs.
  EvaluateAndAddNonlinearCosts(prog, prog.quadratic_costs(), xvec, &(F[0]),
                               &cost_gradients);
  // L2Norm costs.
  EvaluateAndAddNonlinearCosts(prog, prog.l2norm_costs(), xvec, &(F[0]),
                               &cost_gradients);
  // Generic costs.
  EvaluateAndAddNonlinearCosts(prog, prog.generic_costs(), xvec, &(F[0]),
                               &cost_gradients);

  for (const int cost_gradient_index : nonlinear_cost_gradient_indices) {
    (*G_w_duplicate)[*grad_index] = cost_gradients[cost_gradient_index];
    ++(*grad_index);
  }
}

void EvaluateCostsConstraints(
    const SnoptUserFunInfo& info, int n, double x[], double F[], double G[]) {
  const MathematicalProgram& current_problem = info.mathematical_program();
  const auto & scale_map = current_problem.GetVariableScaling();

  Eigen::VectorXd xvec(n);
  for (int i = 0; i < n; i++) {
    xvec(i) = x[i];
  }

  F[0] = 0.0;
  memset(G, 0, info.lenG() * sizeof(double));
  std::vector<double> G_w_duplicate(info.duplicate_to_G_index_map().size(), 0);

  size_t grad_index = 0;

  // Scale xvec
  Eigen::VectorXd xvec_scaled = xvec;
  for (const auto& member : scale_map) {
    xvec_scaled(member.first) *= member.second;
  }
  current_problem.EvalVisualizationCallbacks(xvec_scaled);

  EvaluateAllNonlinearCosts(current_problem, xvec,
                            info.nonlinear_cost_gradient_indices(), F,
                            &G_w_duplicate, &grad_index);

  // The constraint index starts at 1 because the cost is the
  // first row.
  size_t constraint_index = 1;
  // The gradient_index also starts after the cost.
  EvaluateNonlinearConstraints(
      current_problem, current_problem.generic_constraints(), F, &G_w_duplicate,
      &constraint_index, &grad_index, xvec);
  EvaluateNonlinearConstraints(
      current_problem, current_problem.quadratic_constraints(), F,
      &G_w_duplicate, &constraint_index, &grad_index, xvec);
  EvaluateNonlinearConstraints(
      current_problem, current_problem.lorentz_cone_constraints(), F,
      &G_w_duplicate, &constraint_index, &grad_index, xvec);
  EvaluateNonlinearConstraints(
      current_problem, current_problem.rotated_lorentz_cone_constraints(), F,
      &G_w_duplicate, &constraint_index, &grad_index, xvec);
  EvaluateNonlinearConstraints(
      current_problem, current_problem.linear_complementarity_constraints(), F,
      &G_w_duplicate, &constraint_index, &grad_index, xvec);

  for (int i = 0; i < static_cast<int>(info.duplicate_to_G_index_map().size());
       ++i) {
    G[info.duplicate_to_G_index_map()[i]] += G_w_duplicate[i];
  }
}

// This function is what SNOPT calls to compute the values and derivatives.
// Because SNOPT calls this using the C ABI we cannot throw any exceptions,
// otherwise macOS will immediately abort. Therefore, we need to catch all
// exceptions and manually shepherd them back to our C++ code that called
// into SNOPT.
int snopt_userfun(snopt::integer* Status, snopt::integer* n,
                  snopt::doublereal x[], snopt::integer* needF,
                  snopt::integer* neF, snopt::doublereal F[],
                  snopt::integer* needG, snopt::integer* neG,
                  snopt::doublereal G[], char* cu, snopt::integer* lencu,
                  snopt::integer iu[], snopt::integer* leniu,
                  snopt::doublereal ru[], snopt::integer* lenru) {
  SnoptUserFunInfo& info = SnoptUserFunInfo::GetFrom(iu, *leniu);
  try {
    EvaluateCostsConstraints(info, *n, x, F, G);
  } catch (const std::exception& e) {
    info.userfun_error_message() = fmt::format(
        "Exception while evaluating SNOPT costs and constraints: '{}'",
        e.what());
    // The SNOPT manual says "Set Status < -1 if you want snOptA to stop."
    *Status = -2;
  }
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
    std::vector<double>* Fupp, std::vector<int>* iGfun_w_duplicate,
    std::vector<int>* jGvar_w_duplicate, size_t* constraint_index,
    size_t* grad_index) {
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
        (*iGfun_w_duplicate)[*grad_index] =
            1 + *constraint_index + nonzero_entry.first;
        (*jGvar_w_duplicate)[*grad_index] =
            1 + bound_var_indices_in_prog[nonzero_entry.second];
        (*grad_index)++;
      }
    } else {
      for (int i = 0; i < n; i++) {
        for (int j = 0; j < static_cast<int>(binding.GetNumElements()); ++j) {
          // Fortran is 1-indexed.
          (*iGfun_w_duplicate)[*grad_index] =
              1 + *constraint_index + i;  // row order
          (*jGvar_w_duplicate)[*grad_index] = 1 + bound_var_indices_in_prog[j];
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
    std::vector<int>* iGfun_w_duplicate, std::vector<int>* jGvar_w_duplicate,
    size_t* constraint_index, size_t* grad_index) {
  for (const auto& binding : constraint_list) {
    (*Flow)[*constraint_index] = 0;
    (*Fupp)[*constraint_index] = 0;
    for (int j = 0; j < binding.evaluator()->M().rows(); ++j) {
      // Fortran is 1-indexed.
      (*iGfun_w_duplicate)[*grad_index] = 1 + *constraint_index;
      (*jGvar_w_duplicate)[*grad_index] =
          1 + prog.FindDecisionVariableIndex(binding.variables()(j));
      (*grad_index)++;
    }
    ++(*constraint_index);
  }
}

template <typename C>
Eigen::SparseMatrix<double> LinearEvaluatorA(const C& evaluator) {
  if constexpr (std::is_same_v<C, LinearComplementarityConstraint>) {
    // TODO(hongkai.dai): change LinearComplementarityConstraint to store a
    // sparse matrix M, and change the return type here to const
    // SparseMatrix<double>&.
    return evaluator.M().sparseView();
  } else {
    return evaluator.get_sparse_A();
  }
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
  const auto& scale_map = prog.GetVariableScaling();
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
  // becomes lower / s <= x(i) <= upper / s. Perturbing the bounds of the
  // scaled constraint by eps is equivalent to perturbing the original bounds
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
  SetConstraintDualSolution(prog.quadratic_constraints(), Fmul,
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
  // Please refer to User's Guide for SNOPT Versions 7, section 8.6 on the
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
      prog.quadratic_constraints(), num_nonlinear_constraints,
      max_num_gradients, constraint_dual_start_index);
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

// The pair (iGfun_w_duplicate[i], jGvar_w_duplicate[i]) might contain
// duplicated entries. We move the duplication, such that (iGfun[i], jGvar[i])
// don't contain duplicated variables, and iGfun_w_duplicate[i] =
// iGfun[duplicate_to_G_index_map[i]], jGvar_w_duplicate[i] =
// jGvar[duplicate_to_G_index_map[i]].
void PruneGradientDuplication(int nx, const std::vector<int>& iGfun_w_duplicate,
                              const std::vector<int>& jGvar_w_duplicate,
                              std::vector<int>* duplicate_to_G_index_map,
                              std::vector<snopt::integer>* iGfun,
                              std::vector<snopt::integer>* jGvar) {
  auto gradient_index = [nx](int row, int col) { return row * nx + col; };
  std::unordered_map<int, int> gradient_index_to_G;
  duplicate_to_G_index_map->reserve(iGfun_w_duplicate.size());
  iGfun->reserve(iGfun_w_duplicate.size());
  jGvar->reserve(jGvar_w_duplicate.size());
  for (int i = 0; i < static_cast<int>(iGfun_w_duplicate.size()); ++i) {
    const int index =
        gradient_index(iGfun_w_duplicate[i], jGvar_w_duplicate[i]);
    auto it = gradient_index_to_G.find(index);
    if (it == gradient_index_to_G.end()) {
      duplicate_to_G_index_map->push_back(iGfun->size());
      gradient_index_to_G.emplace_hint(it, index, iGfun->size());
      iGfun->push_back(static_cast<snopt::integer>(iGfun_w_duplicate[i]));
      jGvar->push_back(static_cast<snopt::integer>(jGvar_w_duplicate[i]));
    } else {
      duplicate_to_G_index_map->push_back(it->second);
    }
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
  const auto& scale_map = prog.GetVariableScaling();

  std::string print_file_name = print_file_common;
  const auto print_file_it = snopt_options_string.find("Print file");
  if (print_file_it != snopt_options_string.end()) {
    print_file_name = print_file_it->second;
  }
  Snopt::sninit(
      print_file_name.c_str(), print_file_name.length(), 6 /* no summary */,
      storage.cw(), storage.lencw(),
      storage.iw(), storage.leniw(),
      storage.rw(), storage.lenrw());
  ScopeExit guard([&storage]() {
    Snopt::snclose();
  });

  int nx = prog.num_vars();
  std::vector<double> x(nx, 0.0);
  std::vector<double> xlow(nx, -std::numeric_limits<double>::infinity());
  std::vector<double> xupp(nx, std::numeric_limits<double>::infinity());
  solver_details.xmul.resize(nx);
  solver_details.xmul.setZero();
  std::vector<snopt::integer> xstate(nx, 0);

  // Initialize the guess for x.
  for (int i = 0; i < nx; ++i) {
    if (!std::isnan(x_init(i))) {
      x[i] = x_init(i);
    } else {
      x[i] = 0.0;
    }
  }
  // Scale initial guess
  for (const auto& member : scale_map) {
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
  std::vector<snopt::integer> Fstate(nF, 0);

  // Set up the gradient sparsity pattern.
  int lenG_w_duplicate = max_num_gradients;
  std::vector<int> iGfun_w_duplicate(lenG_w_duplicate, 0);
  std::vector<int> jGvar_w_duplicate(lenG_w_duplicate, 0);
  size_t grad_index = 0;
  for (const auto cost_gradient_index :
       user_info.nonlinear_cost_gradient_indices()) {
    // Fortran is 1-indexed.
    iGfun_w_duplicate[grad_index] = 1;
    jGvar_w_duplicate[grad_index] = 1 + cost_gradient_index;
    ++grad_index;
  }

  // constraint_index starts at 1 because row 0 is the cost.
  size_t constraint_index = 1;
  UpdateConstraintBoundsAndGradients(
      prog, prog.generic_constraints(), &Flow, &Fupp, &iGfun_w_duplicate,
      &jGvar_w_duplicate, &constraint_index, &grad_index);
  UpdateConstraintBoundsAndGradients(
      prog, prog.quadratic_constraints(), &Flow, &Fupp, &iGfun_w_duplicate,
      &jGvar_w_duplicate, &constraint_index, &grad_index);
  UpdateConstraintBoundsAndGradients(
      prog, prog.lorentz_cone_constraints(), &Flow, &Fupp, &iGfun_w_duplicate,
      &jGvar_w_duplicate, &constraint_index, &grad_index);
  UpdateConstraintBoundsAndGradients(
      prog, prog.rotated_lorentz_cone_constraints(), &Flow, &Fupp,
      &iGfun_w_duplicate, &jGvar_w_duplicate, &constraint_index, &grad_index);
  UpdateConstraintBoundsAndGradients(
      prog, prog.linear_complementarity_constraints(), &Flow, &Fupp,
      &iGfun_w_duplicate, &jGvar_w_duplicate, &constraint_index, &grad_index);

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
  for (auto& triplet : linear_constraints_triplets) {
    auto it = scale_map.find(triplet.col());
    if (it != scale_map.end()) {
      triplet = Eigen::Triplet<double>(triplet.row(), triplet.col(),
                                       triplet.value() * (it->second));
    }
  }

  Eigen::SparseMatrix<double> linear_constraints_A(nF - 1, prog.num_vars());
  // setFromTriplets sums up the duplicated entries.
  linear_constraints_A.setFromTriplets(linear_constraints_triplets.begin(),
                                       linear_constraints_triplets.end());
  int lenA = variable_to_linear_cost_coefficient.size() +
             linear_constraints_A.nonZeros();
  std::vector<double> A(lenA, 0.0);
  std::vector<snopt::integer> iAfun(lenA, 0);
  std::vector<snopt::integer> jAvar(lenA, 0);
  size_t A_index = 0;
  for (const auto& it : variable_to_linear_cost_coefficient) {
    A[A_index] = it.second;
    // Fortran uses 1-index.
    iAfun[A_index] = 1;
    jAvar[A_index] = it.first + 1;
    A_index++;
  }
  for (int i = 0; i < linear_constraints_A.outerSize(); ++i) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(linear_constraints_A, i);
         it; ++it) {
      A[A_index] = it.value();
      // Fortran uses 1-index
      iAfun[A_index] = num_nonlinear_constraints + it.row() + 2;
      jAvar[A_index] = it.col() + 1;
      A_index++;
    }
  }

  std::vector<snopt::integer> iGfun;
  std::vector<snopt::integer> jGvar;
  auto& duplicate_to_G_index_map = user_info.duplicate_to_G_index_map();
  PruneGradientDuplication(nx, iGfun_w_duplicate, jGvar_w_duplicate,
                           &duplicate_to_G_index_map, &iGfun, &jGvar);
  const int lenG = iGfun.size();
  user_info.set_lenG(lenG);

  for (const auto& it : snopt_options_double) {
    snopt::integer errors = 0;
    Snopt::snsetr(
        it.first.c_str(), it.first.length(), it.second, &errors,
        storage.cw(), storage.lencw(),
        storage.iw(), storage.leniw(),
        storage.rw(), storage.lenrw());
    if (errors > 0) {
      throw std::runtime_error("Error setting Snopt double parameter " +
                               it.first);
    }
  }

  for (const auto& it : snopt_options_int) {
    snopt::integer errors = 0;
    Snopt::snseti(
        it.first.c_str(), it.first.length(), it.second, &errors,
        storage.cw(), storage.lencw(),
        storage.iw(), storage.leniw(),
        storage.rw(), storage.lenrw());
    if (errors > 0) {
      throw std::runtime_error("Error setting Snopt integer parameter " +
                               it.first);
    }
  }

  snopt::integer Cold = 0;
  double objective_constant = linear_cost_constant_term;
  // The index of the objective row among all function evaluation F (notice that
  // due to SNOPT using Fortran, this is 1-indexed).
  snopt::integer ObjRow = 1;
  snopt::integer nS = 0;
  snopt::integer nInf{0};
  double sInf{0.0};

  // Reallocate char, int and real workspace.
  snopt::integer mincw, miniw, minrw;
  snopt::integer snopt_status{0};
  snopt::integer nxname = 1, nFname = 1;
  Snopt::snmema(&snopt_status, nF, nx, lenA, lenG, &mincw, &miniw, &minrw,
                storage.cw(), storage.lencw(), storage.iw(), storage.leniw(),
                storage.rw(), storage.lenrw(), &nxname, &nFname);
  // TODO(jwnimmer-tri) Check snopt_status for errors.
  if (mincw > storage.lencw()) {
    storage.resize_cw(mincw);
    const std::string option = "Total character workspace";
    snopt::integer errors;
    Snopt::snseti(
        option.c_str(), option.length(), storage.lencw(), &errors,
        storage.cw(), storage.lencw(),
        storage.iw(), storage.leniw(),
        storage.rw(), storage.lenrw());
    // TODO(hongkai.dai): report the error in SnoptSolverDetails.
  }
  if (miniw > storage.leniw()) {
    storage.resize_iw(miniw);
    const std::string option = "Total integer workspace";
    snopt::integer errors;
    Snopt::snseti(
        option.c_str(), option.length(), storage.leniw(), &errors,
        storage.cw(), storage.lencw(),
        storage.iw(), storage.leniw(),
        storage.rw(), storage.lenrw());
    // TODO(hongkai.dai): report the error in SnoptSolverDetails.
  }
  if (minrw > storage.lenrw()) {
    storage.resize_rw(minrw);
    const std::string option = "Total real workspace";
    snopt::integer errors;
    Snopt::snseti(
        option.c_str(), option.length(), storage.lenrw(), &errors,
        storage.cw(), storage.lencw(),
        storage.iw(), storage.leniw(),
        storage.rw(), storage.lenrw());
    // TODO(hongkai.dai): report the error in SnoptSolverDetails.
  }
  // Actual solve.
  char problem_name[] = "drake_problem";
  // clang-format off
  Snopt::snopta(Cold, problem_name, nF, nx, objective_constant, ObjRow,
                reinterpret_cast<snopt::U_fp>(&snopt_userfun),
                iAfun.data(), jAvar.data(), lenA, A.data(),
                iGfun.data(), jGvar.data(), lenG,
                xlow.data(), xupp.data(),
                Flow.data(), Fupp.data(),
                x.data(), xstate.data(), solver_details.xmul.data(),
                solver_details.F.data(), Fstate.data(),
                solver_details.Fmul.data(),
                &snopt_status, &nS, &nInf, &sInf, &mincw, &miniw, &minrw,
                storage.iu(), storage.leniu(),
                storage.ru(), storage.lenru(),
                storage.cw(), storage.lencw(),
                storage.iw(), storage.leniw(),
                storage.rw(), storage.lenrw(), nxname, nFname);
  // clang-format on
  if (user_info.userfun_error_message().has_value()) {
    throw std::runtime_error(*user_info.userfun_error_message());
  }

  Eigen::VectorXd x_val = Eigen::Map<Eigen::VectorXd>(x.data(), nx);
  // Scale solution back
  for (const auto& member : scale_map) {
    x_val(member.first) *= member.second;
  }
  solver_details.info = snopt_status;

  SetMathematicalProgramResult(
      prog, snopt_status, x_val, bb_con_dual_variable_indices,
      constraint_dual_start_index, objective_constant, result);
}

}  // namespace

bool SnoptSolver::is_available() {
  return true;
}

void SnoptSolver::DoSolve(const MathematicalProgram& prog,
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

bool SnoptSolver::is_bounded_lp_broken() {
  return true;
}
}  // namespace solvers
}  // namespace drake

#pragma GCC diagnostic pop  // "-Wunused-parameter"
