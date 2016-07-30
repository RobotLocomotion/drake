#pragma once

#include <algorithm>
#include <initializer_list>
#include <list>
#include <map>
#include <memory>
#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/core/Function.h"
#include "drake/core/Gradient.h"
#include "drake/drakeOptimization_export.h"
#include "drake/solvers/Constraint.h"
#include "drake/solvers/MathematicalProgram.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/solution_result.h"
#include "drake/util/Polynomial.h"

namespace drake {
namespace solvers {

class DRAKEOPTIMIZATION_EXPORT OptimizationProblem {
  /** Binding
   * @brief A binding on constraint type C is a mapping of the decision
   * variables onto the inputs of C.  This allows the constraint to operate
   * on a vector made up of different elements of the decision variables.
   */
  template <typename C>
  class Binding {
    std::shared_ptr<C> constraint_;
    VariableList variable_list_;

   public:
    Binding(std::shared_ptr<C> const& c, VariableList const& v)
        : constraint_(c), variable_list_(v) {}
    template <typename U>
    Binding(
        Binding<U> const& b,
        typename std::enable_if<std::is_convertible<
            std::shared_ptr<U>, std::shared_ptr<C>>::value>::type* = nullptr)
        : Binding(b.constraint(), b.variable_list()) {}

    std::shared_ptr<C> const& constraint() const { return constraint_; }

    VariableList const& variable_list() const { return variable_list_; }

    /** covers()
     * @brief returns true iff the given @p index of the enclosing
     * OptimizationProblem is included in this Binding.*/
    bool Covers(size_t index) const {
      for (auto view : variable_list_) {
        if (view.covers(index)) {
          return true;
        }
      }
      return false;
    }

    size_t GetNumElements() const {
      // TODO(ggould-tri) assumes that no index appears more than once in the
      // view, which is nowhere asserted (but seems assumed elsewhere).
      size_t count = 0;
      for (auto view : variable_list_) {
        count += view.size();
      }
      return count;
    }

    /** writeThrough()
     * @brief Write the elements of @p solution to the bound elements of
     * the @p output vector.
     */
    void WriteThrough(const Eigen::VectorXd& solution,
                      Eigen::VectorXd* output) const {
      DRAKE_ASSERT(static_cast<size_t>(solution.rows()) == GetNumElements());
      size_t solution_index = 0;
      for (auto view : variable_list_) {
        const auto& solution_segment =
            solution.segment(solution_index, view.size());
        output->segment(view.index(), view.size()) = solution_segment;
        solution_index += view.size();
      }
    }
  };

  template <typename F>
  class ConstraintImpl : public Constraint {
    F const f_;

   public:
    // Construct by copying from an lvalue.
    template <typename... Args>
    ConstraintImpl(F const& f, Args&&... args)
        : Constraint(drake::FunctionTraits<F>::numOutputs(f),
                     std::forward<Args>(args)...),
          f_(f) {}

    // Construct by moving from an rvalue.
    template <typename... Args>
    ConstraintImpl(F&& f, Args&&... args)
        : Constraint(drake::FunctionTraits<F>::numOutputs(f),
                     std::forward<Args>(args)...),
          f_(std::forward<F>(f)) {}

    void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
      y.resize(drake::FunctionTraits<F>::numOutputs(f_));
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) ==
                   drake::FunctionTraits<F>::numInputs(f_));
      DRAKE_ASSERT(static_cast<size_t>(y.rows()) ==
                   drake::FunctionTraits<F>::numOutputs(f_));
      drake::FunctionTraits<F>::eval(f_, x, y);
    }
    void Eval(const Eigen::Ref<const drake::TaylorVecXd>& x,
              drake::TaylorVecXd& y) const override {
      y.resize(drake::FunctionTraits<F>::numOutputs(f_));
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) ==
                   drake::FunctionTraits<F>::numInputs(f_));
      DRAKE_ASSERT(static_cast<size_t>(y.rows()) ==
                   drake::FunctionTraits<F>::numOutputs(f_));
      drake::FunctionTraits<F>::eval(f_, x, y);
    }
  };

 public:
  enum {
    INITIAL_VARIABLE_ALLOCATION_NUM = 100
  };  // not const static int because the VectorXd constructor takes a reference
      // to int so it is odr-used (see
  // https://gcc.gnu.org/wiki/VerboseDiagnostics#missing_static_const_definition)
  OptimizationProblem()
      : num_vars_(0),
        x_initial_guess_(
            static_cast<Eigen::Index>(INITIAL_VARIABLE_ALLOCATION_NUM)),
        problem_type_(),
        solver_result_(0) {}

  /// Add continuous variables to this OptimizationProblem.
  /**
   * Add continuous variables, appending them to an internal list of any
   * existing vars.
   * The new variables are uninitialized: callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   *
   * @return The DecisionVariableView of the new vars (not all the vars stored).
   */
  const DecisionVariableView AddContinuousVariables(std::size_t num_new_vars,
                                                    std::string name = "x") {
    DecisionVariable v(DecisionVariable::VarType::CONTINUOUS, name,
                       num_new_vars, num_vars_);
    num_vars_ += num_new_vars;
    variables_.push_back(v);
    variable_views_.push_back(DecisionVariableView(variables_.back()));
    x_initial_guess_.conservativeResize(num_vars_);
    x_initial_guess_.tail(num_new_vars) =
        0.1 * Eigen::VectorXd::Random(num_new_vars);

    return variable_views_.back();
  }

  //    const DecisionVariable& AddIntegerVariables(size_t num_new_vars,
  //    std::string name);
  //  ...

  void AddCost(std::shared_ptr<Constraint> const& obj,
               VariableList const& vars) {
    problem_type_.AddGenericCost();
    generic_costs_.push_back(Binding<Constraint>(obj, vars));
  }

  void AddCost(std::shared_ptr<Constraint> const& obj) {
    AddCost(obj, variable_views_);
  }

  template <typename F>
  typename std::enable_if<
      !std::is_convertible<F, std::shared_ptr<Constraint>>::value,
      std::shared_ptr<Constraint>>::type
  AddCost(F&& f, VariableList const& vars) {
    auto c = std::make_shared<
        ConstraintImpl<typename std::remove_reference<F>::type>>(
        std::forward<F>(f));
    AddCost(c, vars);
    return c;
  }

  template <typename F>
  typename std::enable_if<
      !std::is_convertible<F, std::shared_ptr<Constraint>>::value,
      std::shared_ptr<Constraint>>::type
  AddCost(F&& f) {
    return AddCost(std::forward<F>(f), variable_views_);
  }

  // libstdc++ 4.9 evaluates
  // `std::is_convertible<std::unique_ptr<Unrelated>,
  // std::shared_ptr<Constraint>>::value`
  // incorrectly as `true` so our enable_if overload is not used.
  // Provide an explicit alternative for this case.
  template <typename F>
  std::shared_ptr<Constraint> AddCost(std::unique_ptr<F>&& f,
                                      VariableList const& vars) {
    auto c = std::make_shared<ConstraintImpl<std::unique_ptr<F>>>(
        std::forward<std::unique_ptr<F>>(f));
    AddCost(c, vars);
    return c;
  }
  template <typename F>
  std::shared_ptr<Constraint> AddCost(std::unique_ptr<F>&& f) {
    return AddCost(std::forward<std::unique_ptr<F>>(f), variable_views_);
  }

  /** AddQuadraticErrorCost
   * @brief Adds a cost term of the form (x-x_desired)'*Q*(x-x_desired).
   */
  template <typename DerivedQ, typename Derivedb>
  std::shared_ptr<QuadraticConstraint> AddQuadraticErrorCost(
      const Eigen::MatrixBase<DerivedQ>& Q,
      const Eigen::MatrixBase<Derivedb>& x_desired, const VariableList& vars) {
    std::shared_ptr<QuadraticConstraint> cost(new QuadraticConstraint(
        2 * Q, -2 * Q * x_desired, -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()));
    AddQuadraticCost(cost, vars);
    return cost;
  }

  /** AddQuadraticErrorCost
   * @brief Adds a cost term of the form (x-x_desired)'*Q*(x-x_desired).
   * Applied to all (currently existing) variables.
   */
  template <typename DerivedQ, typename Derivedb>
  std::shared_ptr<QuadraticConstraint> AddQuadraticErrorCost(
      const Eigen::MatrixBase<DerivedQ>& Q,
      const Eigen::MatrixBase<Derivedb>& x_desired) {
    return AddQuadraticErrorCost(Q, x_desired, variable_views_);
  }
  /** AddQuadraticCost
   * @brief Adds a cost term of the form 0.5*x'*Q*x + b'x
   * Applied to subset of the variables and pushes onto
   * the quadratic cost data structure.
   */
  void AddQuadraticCost(std::shared_ptr<QuadraticConstraint> const& obj,
                        VariableList const& vars) {
    problem_type_.AddQuadraticCost();
    quadratic_costs_.push_back(Binding<QuadraticConstraint>(obj, vars));
  }

  /** AddQuadraticCost
   * @brief Adds a cost term of the form 0.5*x'*Q*x + b'x
   * Applied to all (currently existing) variables.
   */
  void AddQuadraticCost(std::shared_ptr<QuadraticConstraint> const& obj) {
    AddQuadraticCost(obj, variable_views_);
  }

  /** AddQuadraticCost
   * @brief Adds a cost term of the form 0.5*x'*Q*x + b'x
   * Applied to subset of the variables
   */
  template <typename DerivedQ, typename Derivedb>
  std::shared_ptr<QuadraticConstraint> AddQuadraticCost(
      const Eigen::MatrixBase<DerivedQ>& Q,
      const Eigen::MatrixBase<Derivedb>& b, const VariableList& vars) {
    std::shared_ptr<QuadraticConstraint> cost(
        new QuadraticConstraint(Q, b, -std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::infinity()));
    AddQuadraticCost(cost, vars);
    return cost;
  }

  /** AddQuadraticCost
   * @brief Adds a cost term of the form 0.5*x'*Q*x + b'x
   * Applies to all (currently existing) variables.
   */
  template <typename DerivedQ, typename Derivedb>
  std::shared_ptr<QuadraticConstraint> AddQuadraticCost(
      const Eigen::MatrixBase<DerivedQ>& Q,
      const Eigen::MatrixBase<Derivedb>& b) {
    return AddQuadraticCost(Q, b, variable_views_);
  }

  /** AddGenericConstraint
   *
   * @brief Adds a generic constraint to the program.  This should
   * only be used if a more specific type of constraint is not
   * available, as it may require the use of a significantly more
   * expensive solver.
   */
  void AddGenericConstraint(std::shared_ptr<Constraint> con,
                            VariableList const& vars) {
    problem_type_.AddGenericConstraint();
    generic_constraints_.push_back(Binding<Constraint>(con, vars));
  }
  void AddGenericConstraint(std::shared_ptr<Constraint> con) {
    AddGenericConstraint(con, variable_views_);
  }

  /** AddLinearConstraint
   *
   * @brief Adds linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   */
  void AddLinearConstraint(std::shared_ptr<LinearConstraint> con,
                           VariableList const& vars) {
    problem_type_.AddLinearConstraint();
    linear_constraints_.push_back(Binding<LinearConstraint>(con, vars));
  }

  /** AddLinearConstraint
   *
   * @brief Adds linear constraints to the program for all (currently existing)
   * variables.
   */
  void AddLinearConstraint(std::shared_ptr<LinearConstraint> con) {
    AddLinearConstraint(con, variable_views_);
  }

  /** AddLinearConstraint
   *
   * @brief Adds linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   */
  template <typename DerivedA, typename DerivedLB, typename DerivedUB>
  std::shared_ptr<LinearConstraint> AddLinearConstraint(
      const Eigen::MatrixBase<DerivedA>& A,
      const Eigen::MatrixBase<DerivedLB>& lb,
      const Eigen::MatrixBase<DerivedUB>& ub, const VariableList& vars) {
    auto constraint = std::make_shared<LinearConstraint>(A, lb, ub);
    AddLinearConstraint(constraint, vars);
    return constraint;
  }

  /** AddLinearConstraint
   *
   * @brief Adds linear constraints to the program for all (currently existing)
   * variables.
   */
  template <typename DerivedA, typename DerivedLB, typename DerivedUB>
  std::shared_ptr<LinearConstraint> AddLinearConstraint(
      const Eigen::MatrixBase<DerivedA>& A,
      const Eigen::MatrixBase<DerivedLB>& lb,
      const Eigen::MatrixBase<DerivedUB>& ub) {
    return AddLinearConstraint(A, lb, ub, variable_views_);
  }

  /** AddLinearEqualityConstraint
   *
   * @brief Adds linear equality constraints referencing potentially a
   * subset of the decision variables (defined in the vars parameter).
   */
  void AddLinearEqualityConstraint(
      std::shared_ptr<LinearEqualityConstraint> con, VariableList const& vars) {
    problem_type_.AddLinearEqualityConstraint();
    linear_equality_constraints_.push_back(
        Binding<LinearEqualityConstraint>(con, vars));
  }

  /** AddLinearEqualityConstraint
   *
   * @brief Adds linear equality constraints to the program for all
   * (currently existing) variables.
   */
  void AddLinearEqualityConstraint(
      std::shared_ptr<LinearEqualityConstraint> con) {
    AddLinearEqualityConstraint(con, variable_views_);
  }

  /** AddLinearEqualityConstraint
   *
   * @brief Adds linear equality constraints referencing potentially a subset of
   * the decision variables.
   * Example: to add and equality constraint which only depends on two of the
   * elements of x, you could use
   *   auto x = prog.AddContinuousDecisionVariable(6,"myvar");
   *   prog.AddLinearEqualityConstraint(Aeq, beq,{x.row(2), x.row(5)});
   * where Aeq has exactly two columns.
   */
  template <typename DerivedA, typename DerivedB>
  std::shared_ptr<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::MatrixBase<DerivedA>& Aeq,
      const Eigen::MatrixBase<DerivedB>& beq, const VariableList& vars) {
    auto constraint = std::make_shared<LinearEqualityConstraint>(Aeq, beq);
    AddLinearEqualityConstraint(constraint, vars);
    return constraint;
  }

  /** AddLinearEqualityConstraint
   *
   * @brief Adds linear equality constraints to the program for all
   * (currently existing) variables.
   */
  template <typename DerivedA, typename DerivedB>
  std::shared_ptr<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::MatrixBase<DerivedA>& Aeq,
      const Eigen::MatrixBase<DerivedB>& beq) {
    return AddLinearEqualityConstraint(Aeq, beq, variable_views_);
  }

  /** AddBoundingBoxConstraint
   *
   * @brief Adds bounding box constraints referencing potentially a subset of
   * the decision variables.
   */
  void AddBoundingBoxConstraint(std::shared_ptr<BoundingBoxConstraint> con,
                                VariableList const& vars) {
    problem_type_.AddLinearConstraint();
    bbox_constraints_.push_back(Binding<BoundingBoxConstraint>(con, vars));
  }

  /** AddBoundingBoxConstraint
   *
   * @brief Adds bounding box constraints to the program for all
   * (currently existing) variables.
   */
  void AddBoundingBoxConstraint(std::shared_ptr<BoundingBoxConstraint> con) {
    AddBoundingBoxConstraint(con, variable_views_);
  }

  /** AddBoundingBoxConstraint
   *
   * @brief Adds bounding box constraints referencing potentially a
   * subset of the decision variables (defined in the vars parameter).
   */
  template <typename DerivedLB, typename DerivedUB>
  std::shared_ptr<BoundingBoxConstraint> AddBoundingBoxConstraint(
      const Eigen::MatrixBase<DerivedLB>& lb,
      const Eigen::MatrixBase<DerivedUB>& ub, const VariableList& vars) {
    std::shared_ptr<BoundingBoxConstraint> constraint(
        new BoundingBoxConstraint(lb, ub));
    AddBoundingBoxConstraint(constraint, vars);
    return constraint;
  }

  /** AddBoundingBoxConstraint
   *
   * @brief Adds bounding box constraints to the program for all
   * (currently existing) variables.
   */
  template <typename DerivedLB, typename DerivedUB>
  std::shared_ptr<BoundingBoxConstraint> AddBoundingBoxConstraint(
      const Eigen::MatrixBase<DerivedLB>& lb,
      const Eigen::MatrixBase<DerivedUB>& ub) {
    return AddBoundingBoxConstraint(lb, ub, variable_views_);
  }

  /** AddLinearComplementarityConstraint
   *
   * @brief Adds a linear complementarity constraints referencing a subset of
   * the decision variables.
   */
  template <typename DerivedM, typename Derivedq>
  std::shared_ptr<LinearComplementarityConstraint>
  AddLinearComplementarityConstraint(const Eigen::MatrixBase<DerivedM>& M,
                                     const Eigen::MatrixBase<Derivedq>& q,
                                     const VariableList& vars) {
    problem_type_.AddLinearComplementarityConstraint();

    // Linear Complementarity Constraint cannot currently coexist with any
    // other types of constraint or cost.
    // (TODO(ggould-tri) relax this to non-overlapping bindings, possibly by
    // calling multiple solvers.)
    DRAKE_ASSERT(generic_constraints_.empty());
    DRAKE_ASSERT(generic_costs_.empty());
    DRAKE_ASSERT(linear_constraints_.empty());
    DRAKE_ASSERT(linear_equality_constraints_.empty());
    DRAKE_ASSERT(bbox_constraints_.empty());

    std::shared_ptr<LinearComplementarityConstraint> constraint(
        new LinearComplementarityConstraint(M, q));
    linear_complementarity_constraints_.push_back(
        Binding<LinearComplementarityConstraint>(constraint, vars));
    return constraint;
  }

  /** AddLinearComplementarityConstraint
   *
   * @brief Adds a linear complementarity constraint to the program for all
   * (currently existing) variables.
   */
  template <typename DerivedM, typename Derivedq>
  std::shared_ptr<LinearComplementarityConstraint>
  AddLinearComplementarityConstraint(const Eigen::MatrixBase<DerivedM>& M,
                                     const Eigen::MatrixBase<Derivedq>& q) {
    return AddLinearComplementarityConstraint(M, q, variable_views_);
  }

  /** AddPolynomialConstraint
   *
   * @brief Adds a polynomial constraint to the program referencing a subset
   * of the decision variables (defined in the vars parameter).
   */
  std::shared_ptr<Constraint> AddPolynomialConstraint(
      const VectorXPoly& polynomials,
      const std::vector<Polynomiald::VarType>& poly_vars,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
      const VariableList& vars) {
    // Polynomials that are actually affine (a sum of linear terms + a
    // constant) can be special-cased.  Other polynomials are treated as
    // generic for now.
    // TODO(ggould-tri) There may be other such special easy cases.
    bool all_affine = true;
    for (int i = 0; i < polynomials.rows(); i++) {
      if (!polynomials[i].IsAffine()) {
        all_affine = false;
        break;
      }
    }
    if (all_affine) {
      Eigen::MatrixXd linear_constraint_matrix =
          Eigen::MatrixXd::Zero(polynomials.rows(), poly_vars.size());
      Eigen::VectorXd linear_constraint_lb = lb;
      Eigen::VectorXd linear_constraint_ub = ub;
      for (int poly_num = 0; poly_num < polynomials.rows(); poly_num++) {
        for (const auto& monomial : polynomials[poly_num].GetMonomials()) {
          if (monomial.terms.size() == 0) {
            linear_constraint_lb[poly_num] -= monomial.coefficient;
            linear_constraint_ub[poly_num] -= monomial.coefficient;
          } else if (monomial.terms.size() == 1) {
            const Polynomiald::VarType term_var = monomial.terms[0].var;
            int var_num = (std::find(poly_vars.begin(), poly_vars.end(),
                                     term_var) -
                           poly_vars.begin());
            DRAKE_ASSERT(var_num < static_cast<int>(poly_vars.size()));
            linear_constraint_matrix(poly_num, var_num) = monomial.coefficient;
          } else {
            DRAKE_ABORT();  // Can't happen (unless isAffine() lied to us).
          }
        }
      }
      if (ub == lb) {
        std::shared_ptr<LinearEqualityConstraint> constraint(
            new LinearEqualityConstraint(linear_constraint_matrix,
                                         linear_constraint_ub));
        AddLinearEqualityConstraint(constraint, vars);
        return constraint;
      } else {
        std::shared_ptr<LinearConstraint> constraint(
            new LinearConstraint(linear_constraint_matrix,
                                 linear_constraint_lb,
                               linear_constraint_ub));
        AddLinearConstraint(constraint, vars);
        return constraint;
      }
    } else {
      std::shared_ptr<PolynomialConstraint> constraint(
          new PolynomialConstraint(polynomials, poly_vars, lb, ub));
      AddGenericConstraint(constraint, vars);
      return constraint;
    }
  }

  /** AddPolynomialConstraint
   *
   * @brief Adds a polynomial constraint to the program referencing all of the
   * decision variables.
   */
  std::shared_ptr<Constraint> AddPolynomialConstraint(
      const VectorXPoly& polynomials,
      const std::vector<Polynomiald::VarType>& poly_vars,
      const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) {
    return AddPolynomialConstraint(polynomials, poly_vars, lb, ub,
                                   variable_views_);
  }

  // template <typename FunctionType>
  // void AddCost(std::function..);
  // void AddLinearCost(const Eigen::MatrixBase<Derived>& c, const vector<const
  // DecisionVariable&>& vars)
  // void addQuadraticCost ...

  template <typename Derived>
  void SetInitialGuess(const DecisionVariableView& var,
                       const Eigen::MatrixBase<Derived>& x0) {
    x_initial_guess_.segment(var.index(), var.size()) = x0;
  }

  /**
   * Solve the OptimizationProblem.
   *
   * @return SolutionResult indicating if the solution was successful.
   */
  SolutionResult Solve() {
    return problem_type_.Solve(*this);
  }  // TODO(naveenoid) : add argument for options

  //    template <typename Derived>
  //    bool solve(const Eigen::MatrixBase<Derived>& x0);

  //    getCostValue();
  //    getExitFlag();
  //    getInfeasibleConstraintNames();

  void PrintSolution() {
    for (const auto& v : variables_) {
      std::cout << v.name() << " = " << v.value().transpose() << std::endl;
    }
  }

  template <typename Derived>
  void SetDecisionVariableValues(const Eigen::MatrixBase<Derived>& x) {
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) == num_vars_);
    size_t index = 0;
    for (auto& v : variables_) {
      v.set_value(x.middleRows(index, v.value().rows()));
      index += v.value().rows();
    }
  }

  /**
   * Set an option for a particular solver.  This interface does not
   * do any verification of solver parameters beyond what an
   * individual solver does for itself.  It does not even verify that
   * the specifed solver exists.  Use this only when you have
   * particular knowledge of what solver is being invoked, and exactly
   * what tuning is required.
   *
   * Supported solver names/options:
   *
   * "SNOPT" -- Paramater names and values as specified in SNOPT
   * User's Guide section 7.7 "Description ofthe optional parameters",
   * used as described in section 7.5 for snSet().
   *
   * "IPOPT" -- Paramater names and values as specified in IPOPT users
   * guide section "Options Reference"
   * http://www.coin-or.org/Ipopt/documentation/node40.html
   *
   * "Mosek" -- Accepts two parameters:
   * - "maxormin"
   *   + Maximize or minimize current problem using either "max" or "min".
   * - "problemtype"
   *   + Currently only accepts "linear" or "quadratic". Use the appropriate
   *     one.
   * TODO: Calling OptimizationProblem::Solve will not invoke mosek at this
   * time.
   */
  void SetSolverOption(const std::string& solver_name,
                       const std::string& solver_option, double option_value) {
    solver_options_double_[solver_name][solver_option] = option_value;
  }

  void SetSolverOption(const std::string& solver_name,
                       const std::string& solver_option, int option_value) {
    solver_options_int_[solver_name][solver_option] = option_value;
  }

  void SetSolverOption(const std::string& solver_name,
                       const std::string& solver_option,
                       const std::string& option_value) {
    solver_options_str_[solver_name][solver_option] = option_value;
  }

  const std::map<std::string, double>& GetSolverOptionsDouble(
      const std::string& solver_name) {
    return solver_options_double_[solver_name];
  }

  const std::map<std::string, int>& GetSolverOptionsInt(
      const std::string& solver_name) {
    return solver_options_int_[solver_name];
  }

  const std::map<std::string, std::string>& GetSolverOptionsStr(
      const std::string& solver_name) {
    return solver_options_str_[solver_name];
  }

  /**
   * Get the name and result code of the particular solver which was
   * used to solve this OptimizationProblem.  The solver names and
   * results are not documented here as this function is only intended
   * for debugging, testing, and support of certain legacy
   * APIs.
   */
  void GetSolverResult(std::string* solver_name, int* solver_result) const {
    *solver_name = solver_name_;
    *solver_result = solver_result_;
  }

  void SetSolverResult(const std::string& solver_name, int solver_result) {
    solver_name_ = solver_name;
    solver_result_ = solver_result;
  }

  const std::list<Binding<Constraint>>& generic_costs() const {
    return generic_costs_;
  }  // e.g. for snopt_user_fun

  const std::list<Binding<Constraint>>& generic_constraints() const {
    return generic_constraints_;
  }  // e.g. for snopt_user_fun

  const std::list<Binding<LinearEqualityConstraint>>&
  linear_equality_constraints() const {
    return linear_equality_constraints_;
  }

  /** Getter for quadratic costs. */
  const std::list<Binding<QuadraticConstraint>>& quadratic_costs() const {
    return quadratic_costs_;
  }

  // TODO(naveenoid) : getter for quadratic_constraints
  const std::list<Binding<LinearConstraint>>& linear_constraints() const {
    return linear_constraints_;
  }

  /** GetAllCosts
   *
   * @brief Getter returning all costs (for now quadratic costs appended to
   * generic costs).
   */
  std::list<Binding<Constraint>> GetAllCosts() const {
    std::list<Binding<Constraint>> costlist = generic_costs_;
    costlist.insert(costlist.end(), quadratic_costs_.begin(),
                    quadratic_costs_.end());
    return costlist;
  }

  std::list<Binding<LinearConstraint>> GetAllLinearConstraints() const {
    std::list<Binding<LinearConstraint>> conlist = linear_constraints_;
    conlist.insert(conlist.end(), linear_equality_constraints_.begin(),
                   linear_equality_constraints_.end());
    return conlist;
  }
  const std::list<Binding<BoundingBoxConstraint>>& bounding_box_constraints()
      const {
    return bbox_constraints_;
  }
  const std::list<Binding<LinearComplementarityConstraint>>&
  linear_complementarity_constraints() const {
    return linear_complementarity_constraints_;
  }

  // Base class for solver-specific data.  A solver implementation may derive
  // a helper class from this for use with getSolverData.
  struct SolverData {
    virtual ~SolverData() {}
  };

  // Call from solver implementations to get a persistently-stored
  // helper structure of type T (derived from SolverData).  If no
  // data of type T is already stored then a new one will be created
  // and stored, replacing data from any other solver in this problem
  // instance.
  template <typename T>
  std::shared_ptr<T> GetSolverData() {
    auto p = std::dynamic_pointer_cast<T>(solver_data_);
    if (!p) {
      p = std::make_shared<T>();
      solver_data_ = p;
    }
    return p;
  }

  size_t num_vars() const { return num_vars_; }
  const Eigen::VectorXd& initial_guess() const { return x_initial_guess_; }

 private:
  // note: use std::list instead of std::vector because realloc in std::vector
  // invalidates existing references to the elements
  std::list<DecisionVariable> variables_;
  VariableList variable_views_;
  std::list<Binding<Constraint>> generic_costs_;
  std::list<Binding<Constraint>> generic_constraints_;
  std::list<Binding<QuadraticConstraint>> quadratic_costs_;
  // TODO(naveenoid) : quadratic_constraints_

  // note: linear_constraints_ does not include linear_equality_constraints_
  std::list<Binding<LinearConstraint>> linear_constraints_;
  std::list<Binding<LinearEqualityConstraint>> linear_equality_constraints_;
  std::list<Binding<BoundingBoxConstraint>> bbox_constraints_;

  // Invariant:  The bindings in this list must be non-overlapping.
  // TODO(ggould-tri) can this constraint be relaxed?
  std::list<Binding<LinearComplementarityConstraint>>
      linear_complementarity_constraints_;

  size_t num_vars_;
  Eigen::VectorXd x_initial_guess_;
  std::shared_ptr<SolverData> solver_data_;
  MathematicalProgram problem_type_;
  std::string solver_name_;
  int solver_result_;
  std::map<std::string, std::map<std::string, double>> solver_options_double_;
  std::map<std::string, std::map<std::string, int>> solver_options_int_;
  std::map<std::string, std::map<std::string, std::string>> solver_options_str_;
};

}  // end namespace solvers
}  // end namespace drake
