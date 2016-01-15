#ifndef DRAKE_OPTIMIZATION_H
#define DRAKE_OPTIMIZATION_H

#include "drake/core/Core.h"
#include <list>
#include <memory>
#include <initializer_list>

namespace Drake {

  class DecisionVariable {
    enum VarType {
      CONTINUOUS,
      INTEGER,
      BINARY
    };

    /** index()
     * @brief returns the first index of this variable in the entire variable vector for the program
     */
    size_t index() const { return start_index; }
    /** size()
     * @brief returns the number of elements in the decision variable vector
     */
    size_t size() const { return data.size(); }
    /** value()
     * @brief returns the actual stored value; which is only meaningful after calling solve() in the program.
     */
    const Eigen::VectorXd& value() const { return data; }

    VarType type;
    std::string name;     // todo: make this a vector of strings

    friend class OptimizationProblem;
    friend class DecisionVariableView;

  private:
    Eigen::VectorXd data;
    size_t start_index;
  };
  class DecisionVariableView {  // enables users to access pieces of the decision variables like they would any other eigen vector
  public:
    DecisionVariableView(const DecisionVariable& var) : var(var), start_index(0), length(var.data.rows()) {}
    DecisionVariableView(const DecisionVariable& var, size_t start, size_t n) : var(var), start_index(start), length(n) {
      assert(start+n<var.data.rows());
    }

    /** index()
     * @brief returns the first index of this variable in the entire variable vector for the program
     */
    size_t index() const { return var.start_index + start_index; }
    /** size()
     * @brief returns the number of elements in the decision variable vector
     */
    size_t size() const { return length; }
    /** value()
     * @brief returns the actual stored value; which is only meaningful after calling solve() in the program.
     */
    Eigen::VectorBlock<const Eigen::VectorXd,-1> value() const { return var.data.segment(start_index,length); }

    const DecisionVariableView operator()(size_t i) const { assert(i<=length); return DecisionVariableView(var,start_index+i,1);}
    const DecisionVariableView row(size_t i) const { assert(i<=length); return DecisionVariableView(var,start_index+i,1); }
    const DecisionVariableView head(size_t n) const { assert(n<=length); return DecisionVariableView(var,start_index,n); }
    const DecisionVariableView tail(size_t n) const { assert(n<=length); return DecisionVariableView(var,start_index+length-n,n); }
    const DecisionVariableView segment(size_t start, size_t n) const { assert(start+n<=length); return DecisionVariableView(var,start_index+start,n); }

    friend class OptimizationProblem;

  private:
    const DecisionVariable& var;
    size_t start_index, length;
  };

  typedef std::list<DecisionVariableView> VariableList;

  // Quick notes about this design:
  // need to have lists of constraints (if not functions), so I want a (non-templated) base class.
  // would be nice to have templated types for specialized functions.  Not strictly required.
  // approach: let functions be templated, and use lambda closures/std::function to wrap them into the constraint classes.

  /**
   * @defgroup function_concept Function Concept
   * @ingroup concepts
   * @{
   * @brief Describes a mapping from one Vector to another Vector y = f(x)
   *
   * @nbsp
   *
   * | Every model of this concept must implement |  |
   * ---------------------|------------------------------------------------------------|
   * | X::InputVector     | type for the input to the system, which models the Vector<ScalarType> concept |
   * | X::OutputVector    | type for the output from the system, which models the Vector<ScalarType> concept |
   * | template <ScalarType> OutputVector<ScalarType> operator()(const InputVector<ScalarType>& x) | the actual function |
   * | InputOutputRelation getProperties() | |
   * | size_t getNumInputs() | only required if the input vector is dynamically-sized |
   * | size_t getNumOutputs() | only required if the output vector is dynamically-sized |
   *
   * (always try to label your methods with const if possible)
   *
   * @}
   */

  /*
   * some thoughts: a constraint is a function + lower and upper bounds.  it should support evaluating the constraint, adding it to an optimization problem,
   * and have support for constraints that require slack variables (adding additional decision variables to the problem).  There
   * should also be some notion of parameterized constraints:  e.g. the acceleration constraints in the rigid body dynamics are constraints
   * on vdot and f, but are "parameterized" by q and v.
   */
  class Constraint {
  public:
    Constraint(const VariableList& vars) : variable_list(vars) {}
    virtual ~Constraint() {}

    const VariableList& getVariableList() const { return variable_list; }
  protected:
    VariableList variable_list;
  };

  // DifferentiableConstraint, PolynomialConstraint, QuadraticConstraint, ComplementarityConstraint, IntegerConstraint, ...

  /** LinearEqualityConstraint
   * @brief Implements a constraint of the form @f Ax = b @f
   */
  class LinearEqualityConstraint : public Constraint {
  public:
    LinearEqualityConstraint(const VariableList& vars) : Constraint(vars) {}
    virtual ~LinearEqualityConstraint() {}

    virtual const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> & getMatrix() const = 0;
    virtual const Eigen::Matrix<double,Eigen::Dynamic,1>& getVector() const = 0;
  };

  class LinearEqualityConstraintImpl : public LinearEqualityConstraint {
  public:
    template <typename DerivedA,typename DerivedB>
    LinearEqualityConstraintImpl(const VariableList& vars, const Eigen::MatrixBase<DerivedA>& Aeq, const Eigen::MatrixBase<DerivedB>& beq)
            : A(Aeq), b(beq), LinearEqualityConstraint(vars) {
      assert(A.rows()==b.rows());
      // todo: only do this loop if debug mode (only used for assert)
      int num_referenced_vars = 0;
      for (const DecisionVariableView& v : vars) {
        num_referenced_vars += v.size();
      }
      assert(A.cols() == num_referenced_vars);
    }
    virtual ~LinearEqualityConstraintImpl() {}

    const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> & getMatrix() const override { return A; }
    const Eigen::Matrix<double,Eigen::Dynamic,1>& getVector() const override { return b; }

    /* updateConstraint
     * @brief change the parameters of the constraint (A and b), but not the variable associations
     *
     * note that A and b can change size in the rows only (representing a different number of linear constraints, but on the same decision variables)
     */
    template <typename DerivedA,typename DerivedB>
    void updateConstraint(const Eigen::MatrixBase<DerivedA>& Aeq, const Eigen::MatrixBase<DerivedB>& beq)
    {
      assert(Aeq.rows()==beq.rows());
      if (Aeq.cols() != A.cols()) throw std::runtime_error("Can't change the number of decision variables");
      A.conservativeResize(Aeq.rows(),Eigen::NoChange);
      A = Aeq;
      b.conservativeResize(beq.rows());
      b = beq;
    };

  private:
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> A;
    Eigen::Matrix<double,Eigen::Dynamic,1> b;
  };


  class OptimizationProblem {
  public:
    OptimizationProblem() : problem_type(new LeastSquares), num_vars(0) {};

    const DecisionVariableView addContinuousVariables(std::size_t num_new_vars, std::string name = "x") {
      DecisionVariable v;
      v.type = DecisionVariable::CONTINUOUS;
      v.name = name;
      v.data = Eigen::VectorXd::Zero(num_new_vars);
      v.start_index = num_vars;
      variables.push_back(v);
      num_vars += num_new_vars;

      return DecisionVariableView(variables.back());
    }
//    const DecisionVariable& addIntegerVariables(size_t num_new_vars, std::string name);
//  ...

    /** addConstraint
     * @brief adds a constraint to the program.  method specializations ensure that the constraint gets added in the right way
     */
//    void addConstraint(const std::shared_ptr<LinearConstraint>& con, const VariableList& vars) {}

    /** addLinearEqualityConstraint
     * @brief adds linear equality constraints to the program for all (currently existing) variables
     */
    template <typename DerivedA,typename DerivedB>
    std::shared_ptr<LinearEqualityConstraintImpl> addLinearEqualityConstraint(const Eigen::MatrixBase<DerivedA>& Aeq,const Eigen::MatrixBase<DerivedB>& beq) {
      assert(Aeq.cols() == num_vars);
      VariableList vars;
      for (auto const& v : variables) {
        vars.push_back(DecisionVariableView(v));
      }
      return addLinearEqualityConstraint(Aeq,beq,vars);
    }

  /** addLinearEqualityConstraint
   * @brief adds linear equality constraints referencing potentially a subset of the decision variables.
   * Example: to add and equality constraint which only depends on two of the elements of x, you could use
   *   auto x = prog.addContinuousDecisionVariable(6,"myvar");
   *   prog.addLinearEqualityConstraint(Aeq,beq,{x.row(2),x.row(5)});
   * where Aeq has exactly two columns.
   */
    template <typename DerivedA,typename DerivedB>
    std::shared_ptr<LinearEqualityConstraintImpl> addLinearEqualityConstraint(const Eigen::MatrixBase<DerivedA>& Aeq,const Eigen::MatrixBase<DerivedB>& beq, const VariableList& vars) {
      problem_type.reset(problem_type->addLinearEqualityConstraint());

      auto constraint = std::make_shared<LinearEqualityConstraintImpl>(vars,Aeq,beq);
      linear_equality_constraints.push_back(constraint);
      return constraint;
    }

    // template <typename FunctionType>
    // void addCost(std::function..);
    // void addLinearCost(const Eigen::MatrixBase<Derived>& c,const vector<const DecisionVariable&>& vars)
    // void addQuadraticCost ...

//    void addConstraint(const LinearConstraint& constraint, const std::vector<const DecisionVariable&>& vars);
//  void addConstraint(const BoundingBoxConstraint& constraint, const std::vector<const DecisionVariable&>& vars);

//    template <typename DerivedLB,typename DerivedUB>
//    void addBoundingBoxConstraint(const Eigen::MatrixBase<DerivedLB>& lower_bound, const Eigen::MatrixBase<DerivedUB>& upper_bound, const DecisionVariable& var) {  }


    bool solve() { return problem_type->solve(*this); }; // todo: add argument for options

//    template <typename Derived>
//    bool solve(const Eigen::MatrixBase<Derived>& x0);

//    getObjectiveValue();
//    getExitFlag();
//    getInfeasibleConstraintNames();

    void setDecisionVariableValues() {  // todo: overload the ostream operator instead?
      for (const auto& v : variables) {
        std::cout << v.name << " = " << v.data.transpose() << std::endl;
      }
    }


    template <typename Derived>
    void setDecisionVariableValues(const Eigen::MatrixBase<Derived>& x) {
      assert(x.rows()==num_vars);
      size_t index=0;
      for (auto &v : variables) {
        v.data = x.middleRows(index,v.data.rows());
        index += v.data.rows();
      }
    }

  private:
    // note: use std::list instead of std::vector because realloc in std::vector invalidates existing references to the elements
    std::list<DecisionVariable> variables;
    std::list<std::shared_ptr<LinearEqualityConstraint>> linear_equality_constraints;
    size_t num_vars;

  private:
    // uses virtual methods to crawl up the complexity hiearchy as new decision variables and constraints are added to the program
    // note that there is dynamic allocation happening in here, but on a structure of negligible size.  (is there a better way?)
    struct MathematicalProgram {
    /* these would be used to fill out the optimization hierarchy prototyped below
      virtual MathematicalProgram* addIntegerVariable() { return new MathematicalProgram; };

      virtual MathematicalProgram* addLinearCost() { return new MathematicalProgram; };
      virtual MathematicalProgram* addQuadraticCost() { return new MathematicalProgram; };
      virtual MathematicalProgram* addNonlinearCost() { return new MathematicalProgram; };

      virtual MathematicalProgram* addSumsOfSquaresConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addLinearMatrixInequalityConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addSecondOrderConeConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addComplementarityConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addNonlinearConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addLinearInequalityConstraint() { return new MathematicalProgram; };
     */
      virtual MathematicalProgram* addLinearEqualityConstraint() { return new MathematicalProgram; };

      virtual bool solve(OptimizationProblem& prog) { throw std::runtime_error("not implemented yet"); }
    };

/*  // Prototype of the more complete optimization problem class hiearchy (to be implemented only as needed)
    struct MixedIntegerNonlinearProgram : public MathematicalProgram {};
    struct MixedIntegerSemidefiniteProgram : public MixedIntegerNonlinearProgram {};
    struct MixedIntegerSecondOrderConeProgram : public MixedIntegerSemidefiniteProgram {};
    struct MixedIntegerQuadraticProgram : public MixedIntegerSecondOrderConeProgram {};
    struct MixedIntegerLinearProgram : public MixedIntegerQuadraticProgram {};

    struct NonlinearProgram : public MixedIntegerNonlinearProgram {};
    struct SemidefiniteProgram : public NonlinearProgram, public MixedIntegerSemidefiniteProgram {};
    struct SecondOrderConeProgram : public SemidefiniteProgram, public MixedIntegerSecondOrderConeProgram {};
    struct QuadraticProgram : public SecondOrderConeProgram, public MixedIntegerQuadraticProgram {};
    struct LinearProgram : public QuadraticProgram, public MixedIntegerLinearProgram {
      virtual MathematicalProgram* addLinearEqualityConstraint() { return new LinearProgram; };
      virtual MathematicalProgram* addLinearInequalityConstraint() { return new LinearProgram; };
    };

    struct NonlinearComplementarityProblem : public NonlinearProgram {};
    struct LinearComplementarityProblem : public NonlinearComplementarityProblem {};
*/
    struct LeastSquares : public MathematicalProgram { //public LinearProgram, public LinearComplementarityProblem {
      virtual MathematicalProgram* addLinearEqualityConstraint() { return new LeastSquares; };
      virtual bool solve(OptimizationProblem& prog) {
        size_t num_constraints = 0;
        for (auto const& c : prog.linear_equality_constraints) {
          num_constraints += c->getMatrix().rows();
        }

        Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(num_constraints,prog.num_vars);  // todo: use a sparse matrix here?
        Eigen::VectorXd beq(num_constraints);

        size_t constraint_index = 0;
        for (auto const& c : prog.linear_equality_constraints) {
          size_t n = c->getMatrix().rows();
          size_t var_index = 0;
          for (const DecisionVariableView& v : c->getVariableList() ) {
            Aeq.block(constraint_index, v.index(), n, v.size()) = c->getMatrix().middleCols(var_index, v.size());
            var_index += v.size();
          }
          beq.segment(constraint_index, n) = c->getVector();  // = c->getUpperBound() since it's an equality constraint
          constraint_index += n;
        }

        // least-squares solution
        prog.setDecisionVariableValues(Aeq.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(beq));
        return true;
      }
    };
    std::shared_ptr<MathematicalProgram> problem_type;
  };

} // end namespace Drake

#endif //DRAKE_OPTIMIZATION_H
