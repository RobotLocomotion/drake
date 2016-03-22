#ifndef DRAKE_OPTIMIZATION_H
#define DRAKE_OPTIMIZATION_H

#include <list>
#include <memory>
#include <initializer_list>
#include <Eigen/Core>
#include "drake/core/Core.h"
#include "drake/drakeOptimization_export.h"
#include "Constraint.h"
#include "MathematicalProgram.h"


namespace Drake {

/**
 * DecisionVariable
 * @brief Provides storage for a decision variable inside an OptimizationProblem.
 */
class DecisionVariable {
public:
  enum class VarType { CONTINUOUS, INTEGER, BINARY };

  DecisionVariable(VarType type, const std::string& name, 
                   size_t num_vars, size_t start_index) 
      : type_(type), name_(name), 
        data_(Eigen::VectorXd::Zero(num_vars)), start_index_(start_index) {}

  /** index()
   * @brief returns the first index of this variable in the entire variable
   * vector for the program
   */
  size_t index() const { return start_index_; }
  /** size()
   * @brief returns the number of elements in the decision variable vector
   */
  size_t size() const { return data_.size(); }
  /** name()
   * @return the name of the DecisionVariable
   */
  const std::string& name() const { return name_; }
  /** value()
   * @brief returns the actual stored value; which is only meaningful after
   * calling solve() in the program.
   */
  const Eigen::VectorXd& value() const { return data_; }
  void set_value(const Eigen::VectorXd& new_data) { data_ = new_data; }

 private:
  VarType type_;
  std::string name_;
  Eigen::VectorXd data_;
  size_t start_index_;
};

class DecisionVariableView {  // enables users to access pieces of the decision
                              // variables like they would any other eigen
                              // vector
 public:
  /// Create a view which covers an entire DecisionVariable.
  ///
  /// @p var is aliased, and must remain valid for the lifetime of the view.
  DecisionVariableView(const DecisionVariable& var)
      : var(var), start_index(0), length(var.value().rows()) {}

  /// Create a view covering part of a DecisionVariable.
  ///
  /// @p var is aliased, and must remain valid for the lifetime of the view.
  DecisionVariableView(const DecisionVariable& var, size_t start, size_t n)
      : var(var), start_index(start), length(n) {
    assert(start + n < var.value().rows());
  }

  /** index()
   * @brief returns the first index of this variable in the entire variable
   * vector for the program
   */
  size_t index() const { return var.index() + start_index; }

  /** size()
   * @brief returns the number of elements in the decision variable vector
   */
  size_t size() const { return length; }

  /** value()
   * @brief returns the actual stored value; which is only meaningful after
   * calling solve() in the program.
   */
  Eigen::VectorBlock<const Eigen::VectorXd, Eigen::Dynamic> value() const {
    return var.value().segment(start_index, length);
  }

  std::string getName() const {
    if (start_index == 0 && length == var.value().size()) {
      return var.name();
    } else {
      return var.name() + "(" + std::to_string(start_index) + ":" +
             std::to_string(start_index + length) + ")";
    }
  }

  /** covers()
   * @brief returns true iff the given @p index of the enclosing
   * OptimizationProblem is included in this VariableView.*/
  bool covers(size_t index) const {
    return (index >= start_index) && (index < start_index + length);
  }

  const DecisionVariableView operator()(size_t i) const {
    assert(i <= length);
    return DecisionVariableView(var, start_index + i, 1);
  }
  const DecisionVariableView row(size_t i) const {
    assert(i <= length);
    return DecisionVariableView(var, start_index + i, 1);
  }
  const DecisionVariableView head(size_t n) const {
    assert(n <= length);
    return DecisionVariableView(var, start_index, n);
  }
  const DecisionVariableView tail(size_t n) const {
    assert(n <= length);
    return DecisionVariableView(var, start_index + length - n, n);
  }
  const DecisionVariableView segment(size_t start, size_t n) const {
    assert(start + n <= length);
    return DecisionVariableView(var, start_index + start, n);
  }

 private:
  const DecisionVariable& var;
  size_t start_index, length;
};

typedef std::list<DecisionVariableView> VariableList;

class DRAKEOPTIMIZATION_EXPORT OptimizationProblem {

  /** Binding
   * @brief A binding on constraint type C is a mapping of the decision
   * variables onto the inputs of C.  This allows the constraint to operate
   * on a vector made up of different elements of the decision variables.
   */
  template <typename C>
  class Binding {
    std::shared_ptr<C> constraint;
    VariableList variable_list;

   public:
    Binding(std::shared_ptr<C> const& c, VariableList const& v)
        : constraint(c), variable_list(v) {}
    template <typename U>
    Binding(
        Binding<U> const& b,
        typename std::enable_if<std::is_convertible<
            std::shared_ptr<U>, std::shared_ptr<C>>::value>::type* = nullptr)
        : Binding(b.getConstraint(), b.getVariableList()) {}

    std::shared_ptr<C> const& getConstraint() const { return constraint; }

    VariableList const& getVariableList() const { return variable_list; }

    /** covers()
     * @brief returns true iff the given @p index of the enclosing
     * OptimizationProblem is included in this Binding.*/
    bool covers(size_t index) const {
      for (auto view : getVariableList()) {
        if (view.covers(index)) {
          return true;
        }
      }
      return false;
    }

    size_t getNumElements() const {
      // TODO ggould assumes that no index appears more than once in the
      // view, which is nowhere asserted (but seems assumed elsewhere).
      size_t count = 0;
      for (auto view : getVariableList()) {
        count += view.size();
      }
      return count;
    }

    /** writeThrough()
     * @brief Write the elements of @p solution to the bound elements of
     * the @p output vector.
     */
    void writeThrough(const Eigen::VectorXd& solution,
                      Eigen::VectorXd* output) const {
      assert(solution.rows() == getNumElements());
      size_t solution_index = 0;
      for (auto view : variable_list) {
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
        : Constraint(FunctionTraits<F>::numOutputs(f),
                     std::forward<Args>(args)...),
          f_(f) {}

    // Construct by moving from an rvalue.
    template <typename... Args>
    ConstraintImpl(F&& f, Args&&... args)
        : Constraint(FunctionTraits<F>::numOutputs(f),
                     std::forward<Args>(args)...),
          f_(std::forward<F>(f)) {}

    virtual void eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                      Eigen::VectorXd& y) const override {
      y.resize(FunctionTraits<F>::numOutputs(f_));
      assert(x.rows() == FunctionTraits<F>::numInputs(f_));
      assert(y.rows() == FunctionTraits<F>::numOutputs(f_));
      FunctionTraits<F>::eval(f_, x, y);
    }
    virtual void eval(const Eigen::Ref<const TaylorVecXd>& x,
                      TaylorVecXd& y) const override {
      y.resize(FunctionTraits<F>::numOutputs(f_));
      assert(x.rows() == FunctionTraits<F>::numInputs(f_));
      assert(y.rows() == FunctionTraits<F>::numOutputs(f_));
      FunctionTraits<F>::eval(f_, x, y);
    }
  };

 public:
  enum {
    INITIAL_VARIABLE_ALLOCATION_NUM = 100
  };  // not const static int because the VectorXd constructor takes a reference
      // to int so it is odr-used (see
  // https://gcc.gnu.org/wiki/VerboseDiagnostics#missing_static_const_definition)
  OptimizationProblem()
      : problem_type(MathematicalProgramInterface::getLeastSquaresProgram()),
        num_vars(0),
        x_initial_guess(
            static_cast<Eigen::Index>(INITIAL_VARIABLE_ALLOCATION_NUM)){};

  const DecisionVariableView addContinuousVariables(std::size_t num_new_vars,
                                                    std::string name = "x") {
    DecisionVariable v(DecisionVariable::VarType::CONTINUOUS, name,
                       num_new_vars, num_vars);
    num_vars += num_new_vars;
    variables.push_back(v);
    variable_views.push_back(DecisionVariableView(variables.back()));
    x_initial_guess.conservativeResize(num_vars);
    x_initial_guess.tail(num_vars) = 0.1 * Eigen::VectorXd::Random(num_vars);

    return variable_views.back();
  }
  //    const DecisionVariable& addIntegerVariables(size_t num_new_vars,
  //    std::string name);
  //  ...

  void addCost(std::shared_ptr<Constraint> const& obj,
               VariableList const& vars) {
    problem_type.reset(problem_type->addGenericObjective());
    generic_objectives.push_back(Binding<Constraint>(obj, vars));
  }

  void addCost(std::shared_ptr<Constraint> const& obj) {
    addCost(obj, variable_views);
  }

  template <typename F>
  typename std::enable_if<
      !std::is_convertible<F, std::shared_ptr<Constraint>>::value,
      std::shared_ptr<Constraint>>::type
  addCost(F&& f, VariableList const& vars) {
    auto c = std::make_shared<
        ConstraintImpl<typename std::remove_reference<F>::type>>(
        std::forward<F>(f));
    addCost(c, vars);
    return c;
  }

  template <typename F>
  typename std::enable_if<
      !std::is_convertible<F, std::shared_ptr<Constraint>>::value,
      std::shared_ptr<Constraint>>::type
  addCost(F&& f) {
    return addCost(std::forward<F>(f), variable_views);
  }

  // libstdc++ 4.9 evaluates
  // `std::is_convertible<std::unique_ptr<Unrelated>,
  // std::shared_ptr<Constraint>>::value`
  // incorrectly as `true` so our enable_if overload is not used.
  // Provide an explicit alternative for this case.
  template <typename F>
  std::shared_ptr<Constraint> addCost(std::unique_ptr<F>&& f,
                                      VariableList const& vars) {
    auto c = std::make_shared<ConstraintImpl<std::unique_ptr<F>>>(
        std::forward<std::unique_ptr<F>>(f));
    addCost(c, vars);
    return c;
  }
  template <typename F>
  std::shared_ptr<Constraint> addCost(std::unique_ptr<F>&& f) {
    return addCost(std::forward<std::unique_ptr<F>>(f), variable_views);
  }

  /** addQuadraticCost
   * @brief adds a cost term of the form (x-x_desired)'*Q*(x-x_desired)
   */
  template <typename DerivedQ, typename Derivedb>
  std::shared_ptr<QuadraticConstraint> addQuadraticCost(
      const Eigen::MatrixBase<DerivedQ>& Q,
      const Eigen::MatrixBase<Derivedb>& x_desired, const VariableList& vars) {
    std::shared_ptr<QuadraticConstraint> objective(new QuadraticConstraint(
        2 * Q, -2 * Q * x_desired, -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()));
    addCost(objective, vars);
    return objective;
  };

  template <typename DerivedQ, typename Derivedb>
  std::shared_ptr<QuadraticConstraint> addQuadraticCost(
      const Eigen::MatrixBase<DerivedQ>& Q,
      const Eigen::MatrixBase<Derivedb>& x_desired) {
    return addQuadraticCost(Q, x_desired, variable_views);
  };

  /** addConstraint
   * @brief adds a constraint to the program.  method specializations ensure
   * that the constraint gets added in the right way
   */
  void addConstraint(std::shared_ptr<Constraint> con,
                     VariableList const& vars) {
    problem_type.reset(problem_type->addGenericConstraint());
    generic_constraints.push_back(Binding<Constraint>(con, vars));
  }
  void addConstraint(std::shared_ptr<Constraint> con) {
    addConstraint(con, variable_views);
  }

  void addConstraint(std::shared_ptr<LinearEqualityConstraint> con,
                     VariableList const& vars) {
    problem_type.reset(problem_type->addLinearEqualityConstraint());
    linear_equality_constraints.push_back(
        Binding<LinearEqualityConstraint>(con, vars));
  }

  void addConstraint(std::shared_ptr<LinearEqualityConstraint> con) {
    addConstraint(con, variable_views);
  }

  /** addLinearConstraint
   * @brief adds linear constraints to the program for all (currently existing)
   * variables
   */
  template <typename DerivedA, typename DerivedLB, typename DerivedUB>
  std::shared_ptr<LinearConstraint> addLinearConstraint(
      const Eigen::MatrixBase<DerivedA>& A,
      const Eigen::MatrixBase<DerivedLB>& lb,
      const Eigen::MatrixBase<DerivedUB>& ub) {
    return addLinearConstraint(A, lb, ub, variable_views);
  }

  /** addLinearConstraint
   * @brief adds linear constraints referencing potentially a subset of the
   * decision variables.
   */
  template <typename DerivedA, typename DerivedLB, typename DerivedUB>
  std::shared_ptr<LinearConstraint> addLinearConstraint(
      const Eigen::MatrixBase<DerivedA>& A,
      const Eigen::MatrixBase<DerivedLB>& lb,
      const Eigen::MatrixBase<DerivedUB>& ub, const VariableList& vars) {
    problem_type.reset(problem_type->addLinearConstraint());

    auto constraint = std::make_shared<LinearConstraint>(A, lb, ub);
    linear_constraints.push_back(Binding<LinearConstraint>(constraint, vars));
    return constraint;
  }

  /** addLinearEqualityConstraint
   * @brief adds linear equality constraints to the program for all (currently
   * existing) variables
   */
  template <typename DerivedA, typename DerivedB>
  std::shared_ptr<LinearEqualityConstraint> addLinearEqualityConstraint(
      const Eigen::MatrixBase<DerivedA>& Aeq,
      const Eigen::MatrixBase<DerivedB>& beq) {
    return addLinearEqualityConstraint(Aeq, beq, variable_views);
  }

  /** addLinearEqualityConstraint
   * @brief adds linear equality constraints referencing potentially a subset of
   * the decision variables.
   * Example: to add and equality constraint which only depends on two of the
   * elements of x, you could use
   *   auto x = prog.addContinuousDecisionVariable(6,"myvar");
   *   prog.addLinearEqualityConstraint(Aeq, beq,{x.row(2), x.row(5)});
   * where Aeq has exactly two columns.
   */
  template <typename DerivedA, typename DerivedB>
  std::shared_ptr<LinearEqualityConstraint> addLinearEqualityConstraint(
      const Eigen::MatrixBase<DerivedA>& Aeq,
      const Eigen::MatrixBase<DerivedB>& beq, const VariableList& vars) {
    problem_type.reset(problem_type->addLinearEqualityConstraint());

    auto constraint = std::make_shared<LinearEqualityConstraint>(Aeq, beq);
    linear_equality_constraints.push_back(
        Binding<LinearEqualityConstraint>(constraint, vars));
    return constraint;
  }

  /** addBoundingBoxConstraint
   * @brief adds bounding box constraints to the program for all (currently
   * existing) variables
   */
  template <typename DerivedLB, typename DerivedUB>
  std::shared_ptr<BoundingBoxConstraint> addBoundingBoxConstraint(
      const Eigen::MatrixBase<DerivedLB>& lb,
      const Eigen::MatrixBase<DerivedUB>& ub) {
    return addBoundingBoxConstraint(lb, ub, variable_views);
  }

  /** addBoundingBoxConstraint
   * @brief adds bounding box constraints referencing potentially a subset of
   * the decision variables.
   */
  template <typename DerivedLB, typename DerivedUB>
  std::shared_ptr<BoundingBoxConstraint> addBoundingBoxConstraint(
      const Eigen::MatrixBase<DerivedLB>& lb,
      const Eigen::MatrixBase<DerivedUB>& ub, const VariableList& vars) {
    problem_type.reset(problem_type->addLinearConstraint());

    std::shared_ptr<BoundingBoxConstraint> constraint(
        new BoundingBoxConstraint(lb, ub));
    bbox_constraints.push_back(
        Binding<BoundingBoxConstraint>(constraint, vars));
    return constraint;
  }

  /** addLinearComplementarityConstraint
   *
   * @brief adds a linear complementarity constraint to the program for all
   * (currently existing) variables.
   */
  template <typename DerivedM, typename Derivedq>
  std::shared_ptr<LinearComplementarityConstraint>
      addLinearComplementarityConstraint(
          const Eigen::MatrixBase<DerivedM>& M,
          const Eigen::MatrixBase<Derivedq>& q) {
    return addLinearComplementarityConstraint(M, q, variable_views);
  }

  /** addLinearComplementarityConstraint
   *
   * @brief adds a linear complementarity constraints referencing a subset of
   * the decision variables.
   */
  template <typename DerivedM, typename Derivedq>
  std::shared_ptr<LinearComplementarityConstraint>
      addLinearComplementarityConstraint(
          const Eigen::MatrixBase<DerivedM>& M,
          const Eigen::MatrixBase<Derivedq>& q,
          const VariableList& vars) {
    problem_type.reset(
        problem_type->addLinearComplementarityConstraint());

    // Linear Complementarity Constraint cannot currently coexist with any
    // other types of constraint or objective.
    // (TODO ggould relax this to non-overlapping bindings, possibly by
    // calling multiple solvers.)
    assert(generic_constraints.empty());
    assert(generic_objectives.empty());
    assert(linear_constraints.empty());
    assert(linear_equality_constraints.empty());
    assert(bbox_constraints.empty());

    std::shared_ptr<LinearComplementarityConstraint> constraint(
        new LinearComplementarityConstraint(M, q));
    linear_complementarity_constraints.push_back(
        Binding<LinearComplementarityConstraint>(constraint, vars));
    return constraint;
  }

  // template <typename FunctionType>
  // void addCost(std::function..);
  // void addLinearCost(const Eigen::MatrixBase<Derived>& c, const vector<const
  // DecisionVariable&>& vars)
  // void addQuadraticCost ...

  //    void addConstraint(const LinearConstraint& constraint, const
  //    std::vector<const DecisionVariable&>& vars);
  //  void addConstraint(const BoundingBoxConstraint& constraint, const
  //  std::vector<const DecisionVariable&>& vars);

  template <typename Derived>
  void setInitialGuess(const DecisionVariableView& var,
                       const Eigen::MatrixBase<Derived>& x0) {
    x_initial_guess.segment(var.index(), var.size()) = x0;
  }

  bool solve() {
    return problem_type->solve(*this);
  };  // todo: add argument for options

  //    template <typename Derived>
  //    bool solve(const Eigen::MatrixBase<Derived>& x0);

  //    getObjectiveValue();
  //    getExitFlag();
  //    getInfeasibleConstraintNames();

  void printSolution() {
    for (const auto& v : variables) {
      std::cout << v.name() << " = " << v.value().transpose() << std::endl;
    }
  }

  template <typename Derived>
  void setDecisionVariableValues(const Eigen::MatrixBase<Derived>& x) {
    assert(x.rows() == num_vars);
    size_t index = 0;
    for (auto& v : variables) {
      v.set_value(x.middleRows(index, v.value().rows()));
      index += v.value().rows();
    }
  }

  const std::list<Binding<Constraint>>& getGenericObjectives() const {
    return generic_objectives;
  }  // e.g. for snopt_user_fun
  const std::list<Binding<Constraint>>& getGenericConstraints() const {
    return generic_constraints;
  }  // e.g. for snopt_user_fun
  const std::list<Binding<LinearEqualityConstraint>>&
      getLinearEqualityConstraints() const {
    return linear_equality_constraints;
  }
  std::list<Binding<LinearConstraint>> getAllLinearConstraints() const {
    std::list<Binding<LinearConstraint>> conlist = linear_constraints;
    conlist.insert(conlist.end(), linear_equality_constraints.begin(),
                   linear_equality_constraints.end());
    return conlist;
  }
  const std::list<Binding<BoundingBoxConstraint>>&
      getBoundingBoxConstraints() const {
    return bbox_constraints;
  }
  const std::list<Binding<LinearComplementarityConstraint>>&
      getLinearComplementarityConstraints() const {
    return linear_complementarity_constraints;
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
  std::shared_ptr<T> getSolverData() {
    auto p = std::dynamic_pointer_cast<T>(solver_data);
    if (!p) {
      p = std::make_shared<T>();
      solver_data = p;
    }
    return p;
  }

  size_t getNumVars() const { return num_vars; }
  const Eigen::VectorXd& getInitialGuess() const { return x_initial_guess; }

 private:
  void checkVariables(const Constraint& con) {
    assert(checkVariablesImpl(con) &&
           "Constraint depends on variables that are not associated with this "
           "OptimizationProblem");
  }
  bool checkVariablesImpl(const Constraint& con) {
    // todo: implement this
    return true;
  }

  // note: use std::list instead of std::vector because realloc in std::vector
  // invalidates existing references to the elements
  std::list<DecisionVariable> variables;
  VariableList variable_views;
  std::list<Binding<Constraint>> generic_objectives;
  std::list<Binding<Constraint>> generic_constraints;
  std::list<Binding<LinearConstraint>>
      linear_constraints;  // note: does not include linear_equality_constraints
  std::list<Binding<LinearEqualityConstraint>> linear_equality_constraints;
  std::list<Binding<BoundingBoxConstraint>> bbox_constraints;

  // Invariant:  The bindings in this list must be non-overlapping.
  // TODO ggould can this constraint be relaxed?
  std::list<Binding<LinearComplementarityConstraint>>
      linear_complementarity_constraints;

  size_t num_vars;
  Eigen::VectorXd x_initial_guess;
  std::shared_ptr<SolverData> solver_data;
  std::shared_ptr<MathematicalProgramInterface> problem_type;
};

}  // end namespace Drake

#endif  // DRAKE_OPTIMIZATION_H
