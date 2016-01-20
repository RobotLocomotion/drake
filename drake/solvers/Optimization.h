#ifndef DRAKE_OPTIMIZATION_H
#define DRAKE_OPTIMIZATION_H

#include <list>
#include <memory>
#include <initializer_list>
#include <Eigen/SparseCore>
#include "drake/core/Core.h"

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
    std::string name;

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
    std::string getName() const { if (start_index==0 && length==var.data.size())  { return var.name; } else { return var.name + "(" + std::to_string(start_index) + ":" + std::to_string(start_index+length) + ")"; }  }

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

  typedef std::list<const DecisionVariableView> VariableList;
  size_t size(const VariableList& var_list) {
    size_t s = 0;
    for (const auto& var : var_list) s+= var.size();
    return s;
  }

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
    Constraint(const VariableList& vars, size_t num_constraints) : variable_list(vars), lower_bound(num_constraints), upper_bound(num_constraints) {
      assert(lower_bound.size() == upper_bound.size() && "Lower bound and upper bound must be the same size");
      lower_bound.setConstant(-std::numeric_limits<double>::infinity());
      upper_bound.setConstant(std::numeric_limits<double>::infinity());
    }

    template <typename DerivedLB, typename DerivedUB>
    Constraint(const VariableList& vars, const Eigen::MatrixBase<DerivedLB>& lb, const Eigen::MatrixBase<DerivedUB>& ub) : variable_list(vars), lower_bound(lb), upper_bound(ub)
    {}
    virtual ~Constraint() {}

    virtual void eval(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const = 0;
    virtual void eval(const Eigen::Ref<const TaylorVecXd>& x, TaylorVecXd& y) const = 0;  // move this to DifferentiableConstraint derived class if/when we need to support non-differentiable functions

    virtual Eigen::VectorXd getLowerBound() const { return lower_bound; }
    virtual Eigen::VectorXd getUpperBound() const { return upper_bound; }
    size_t getNumConstraints() const { return lower_bound.size(); }

    const VariableList& getVariableList() const { return variable_list; }
  protected:
    VariableList variable_list;
    Eigen::VectorXd lower_bound, upper_bound;
  };

  class FunctionConstraint : public Constraint {
  public:
    FunctionConstraint(const VariableList& vars, const std::shared_ptr<DifferentiableFunction>& func, size_t num_constraints) : Constraint(vars,num_constraints), func(func) {}

    template <typename DerivedLB, typename DerivedUB>
    FunctionConstraint(const VariableList& vars, const std::shared_ptr<DifferentiableFunction>& func, const Eigen::MatrixBase<DerivedLB>& lb, const Eigen::MatrixBase<DerivedUB>& ub) : Constraint(vars,lb,ub), func(func) {}
    virtual ~FunctionConstraint() {}

    virtual void eval(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const override {
      func->eval(x,y);
    }
    virtual void eval(const Eigen::Ref<const TaylorVecXd>& x, TaylorVecXd& y) const override {
      func->eval(x,y);
    }

  private:
    const std::shared_ptr<DifferentiableFunction> func;
  };

  // todo: consider implementing DifferentiableConstraint, TwiceDifferentiableConstraint, PolynomialConstraint, QuadraticConstraint, ComplementarityConstraint, IntegerConstraint, ...
  /** LinearConstraint
   * @brief Implements a constraint of the form @f -lb <= Ax <= ub @f
   */
  class LinearConstraint : public Constraint {
  public:
    LinearConstraint(const VariableList& vars, size_t num_constraints) : Constraint(vars, num_constraints) {}
    template <typename DerivedLB, typename DerivedUB>
    LinearConstraint(const VariableList& vars, const Eigen::MatrixBase<DerivedLB>& lb, const Eigen::MatrixBase<DerivedUB>& ub) : Constraint(vars,lb,ub) {};
    virtual ~LinearConstraint() {}

    virtual void eval(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const override { y.resize(getNumConstraints()); y=getMatrix()*x; }
    virtual void eval(const Eigen::Ref<const TaylorVecXd>& x, TaylorVecXd& y) const override { y.resize(getNumConstraints()); y = getMatrix().cast<TaylorVarXd>()*x; };

    virtual Eigen::SparseMatrix<double> getSparseMatrix() const { return getMatrix().sparseView(); };
    virtual Eigen::MatrixXd getMatrix() const = 0;
  };

  class LinearConstraintImpl : public LinearConstraint {
  public:
    template <typename DerivedA,typename DerivedLB,typename DerivedUB>
    LinearConstraintImpl(const VariableList& vars, const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedLB>& lb, const Eigen::MatrixBase<DerivedUB>& ub)
            : A(A), LinearConstraint(vars,lb,ub) {
      assert(A.rows()==lb.rows());
      assert(A.cols() == size(vars));
    }
    virtual ~LinearConstraintImpl() {}

    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> getMatrix() const override { return A; }

  protected:
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> A;
  };

  /** LinearEqualityConstraint
   * @brief Implements a constraint of the form @f Ax = b @f
   */
  class LinearEqualityConstraint : public LinearConstraint {
  public:
    LinearEqualityConstraint(const VariableList& vars, size_t num_constraints) : LinearConstraint(vars, num_constraints) { }
    template <typename DerivedLB, typename DerivedUB>
    LinearEqualityConstraint(const VariableList& vars, const Eigen::MatrixBase<DerivedLB>& lb, const Eigen::MatrixBase<DerivedUB>& ub) : LinearConstraint(vars,lb,ub) {};
    virtual ~LinearEqualityConstraint() {}

    virtual Eigen::VectorXd getVector() const = 0;
    virtual Eigen::VectorXd getLowerBound() const override { return getVector(); }
    virtual Eigen::VectorXd getUpperBound() const override { return getVector(); }
  };

  class LinearEqualityConstraintImpl : public LinearEqualityConstraint {
  public:
    template <typename DerivedA,typename DerivedB>
    LinearEqualityConstraintImpl(const VariableList& vars, const Eigen::MatrixBase<DerivedA>& Aeq, const Eigen::MatrixBase<DerivedB>& beq)
            : LinearEqualityConstraint(vars,beq,beq), A(Aeq) {}
    virtual ~LinearEqualityConstraintImpl() {}

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
      lower_bound.conservativeResize(beq.rows());
      lower_bound = beq;
      upper_bound.conservativeResize(beq.rows());
      upper_bound = beq;
    };
    virtual Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> getMatrix() const override { return A; }
    virtual Eigen::VectorXd getVector() const override { return lower_bound; };

  protected:
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> A;
  };

  /** BoundingBoxConstraint
 * @brief Implements a constraint of the form @f lb <= x <= ub @f
 * Note: the base Constraint class (as implemented at the moment) could play this role.  But this class enforces
 * that it is ONLY a bounding box constraint, and not something more general.
 */
  class BoundingBoxConstraint : public LinearConstraint {
  public:
    template <typename DerivedLB, typename DerivedUB>
    BoundingBoxConstraint(const VariableList& vars, const Eigen::MatrixBase<DerivedLB>& lb, const Eigen::MatrixBase<DerivedUB>& ub) : LinearConstraint(vars,lb,ub) {}
    virtual ~BoundingBoxConstraint() {}

    virtual void eval(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const override { y.resize(getNumConstraints()); y=x; }
    virtual void eval(const Eigen::Ref<const TaylorVecXd>& x, TaylorVecXd& y) const override { y.resize(getNumConstraints()); y=x; }

    virtual Eigen::MatrixXd getMatrix() const override { return Eigen::MatrixXd::Identity(getNumConstraints(),getNumConstraints()); }
  };

  class OptimizationProblem {
  public:
    OptimizationProblem() : problem_type(new LeastSquares), num_vars(0), x_initial_guess(100) {};

    const DecisionVariableView addContinuousVariables(std::size_t num_new_vars, std::string name = "x") {
      DecisionVariable v;
      v.type = DecisionVariable::CONTINUOUS;
      v.name = name;
      v.data = Eigen::VectorXd::Zero(num_new_vars);
      v.start_index = num_vars;
      num_vars += num_new_vars;
      variables.push_back(v);
      variable_views.push_back(DecisionVariableView(variables.back()));
      x_initial_guess.conservativeResize(num_vars);
      x_initial_guess.tail(num_vars) = 0.1*Eigen::VectorXd::Random(num_vars);

      return variable_views.back();
    }
//    const DecisionVariable& addIntegerVariables(size_t num_new_vars, std::string name);
//  ...

    void addObjective(const std::shared_ptr<Constraint>& obj) {
      checkVariables(obj);
      problem_type.reset(problem_type->addGenericObjective());
      generic_objectives.push_back(obj);
    }

    /** addConstraint
     * @brief adds a constraint to the program.  method specializations ensure that the constraint gets added in the right way
     */
    void addConstraint(const std::shared_ptr<Constraint>& con) {
      checkVariables(con);
      problem_type.reset(problem_type->addGenericConstraint());
      generic_constraints.push_back(con);
    }
    void addConstraint(const std::shared_ptr<LinearEqualityConstraint>& con) {
      checkVariables(con);
      problem_type.reset(problem_type->addLinearEqualityConstraint());
      linear_equality_constraints.push_back(con);
    }

    /** addLinearConstraint
     * @brief adds linear constraints to the program for all (currently existing) variables
     */
    template <typename DerivedA,typename DerivedLB,typename DerivedUB>
    std::shared_ptr<LinearConstraintImpl> addLinearConstraint(const Eigen::MatrixBase<DerivedA>& A,const Eigen::MatrixBase<DerivedLB>& lb,const Eigen::MatrixBase<DerivedUB>& ub) {
      return addLinearConstraint(A,lb,ub,variable_views);
    }

    /** addLinearConstraint
     * @brief adds linear constraints referencing potentially a subset of the decision variables.
     */
    template <typename DerivedA,typename DerivedLB,typename DerivedUB>
    std::shared_ptr<LinearConstraintImpl> addLinearConstraint(const Eigen::MatrixBase<DerivedA>& A,const Eigen::MatrixBase<DerivedLB>& lb, const Eigen::MatrixBase<DerivedUB>& ub, const VariableList& vars) {
      problem_type.reset(problem_type->addLinearConstraint());

      auto constraint = std::make_shared<LinearConstraintImpl>(vars,A,lb,ub);
      linear_constraints.push_back(constraint);
      return constraint;
    }


    /** addLinearEqualityConstraint
     * @brief adds linear equality constraints to the program for all (currently existing) variables
     */
    template <typename DerivedA,typename DerivedB>
    std::shared_ptr<LinearEqualityConstraintImpl> addLinearEqualityConstraint(const Eigen::MatrixBase<DerivedA>& Aeq,const Eigen::MatrixBase<DerivedB>& beq) {
      return addLinearEqualityConstraint(Aeq,beq,variable_views);
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


    /** addBoundingBoxConstraint
     * @brief adds bounding box constraints to the program for all (currently existing) variables
     */
    template <typename DerivedLB,typename DerivedUB>
    std::shared_ptr<BoundingBoxConstraint> addBoundingBoxConstraint(const Eigen::MatrixBase<DerivedLB>& lb,const Eigen::MatrixBase<DerivedUB>& ub) {
      return addBoundingBoxConstraint(lb,ub,variable_views);
    }

    /** addBoundingBoxConstraint
     * @brief adds bounding box constraints referencing potentially a subset of the decision variables.
     */
    template <typename DerivedLB,typename DerivedUB>
    std::shared_ptr<BoundingBoxConstraint> addBoundingBoxConstraint(const Eigen::MatrixBase<DerivedLB>& lb, const Eigen::MatrixBase<DerivedUB>& ub, const VariableList& vars) {
      problem_type.reset(problem_type->addLinearConstraint());

      std::shared_ptr<BoundingBoxConstraint> constraint(new BoundingBoxConstraint(vars,lb,ub));
      bbox_constraints.push_back(constraint);
      return constraint;
    }

    // template <typename FunctionType>
    // void addCost(std::function..);
    // void addLinearCost(const Eigen::MatrixBase<Derived>& c,const vector<const DecisionVariable&>& vars)
    // void addQuadraticCost ...

//    void addConstraint(const LinearConstraint& constraint, const std::vector<const DecisionVariable&>& vars);
//  void addConstraint(const BoundingBoxConstraint& constraint, const std::vector<const DecisionVariable&>& vars);


    template <typename Derived>
    void setInitialGuess(const DecisionVariableView& var, const Eigen::MatrixBase<Derived>& x0)
    {
      x_initial_guess.segment(var.index(),var.size()) = x0;
    }

    bool solve() { return problem_type->solve(*this); }; // todo: add argument for options

//    template <typename Derived>
//    bool solve(const Eigen::MatrixBase<Derived>& x0);

//    getObjectiveValue();
//    getExitFlag();
//    getInfeasibleConstraintNames();

    void printSolution() {  // todo: overload the ostream operator instead?
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

    const std::list<std::shared_ptr<Constraint>>& getGenericObjectives() const { return generic_objectives; } // e.g. for snopt_user_fun
    const std::list<std::shared_ptr<Constraint>>& getGenericConstraints() const { return generic_constraints; } // e.g. for snopt_user_fun
    std::list<std::shared_ptr<LinearConstraint>> getAllLinearConstraints() const {
      std::list<std::shared_ptr<LinearConstraint>> conlist = linear_constraints;
      conlist.insert(conlist.end(),linear_equality_constraints.begin(),linear_equality_constraints.end());
      return conlist;
    }

  private:
    // note: use std::list instead of std::vector because realloc in std::vector invalidates existing references to the elements
    std::list<DecisionVariable> variables;
    VariableList variable_views;
    std::list<std::shared_ptr<Constraint>> generic_objectives;
    std::list<std::shared_ptr<Constraint>> generic_constraints;
    std::list<std::shared_ptr<LinearConstraint>> linear_constraints;  // note: does not include linear_equality_constraints
    std::list<std::shared_ptr<LinearEqualityConstraint>> linear_equality_constraints;
    std::list<std::shared_ptr<BoundingBoxConstraint>> bbox_constraints;
    size_t num_vars;
    Eigen::VectorXd x_initial_guess;
  private:

    void checkVariables(const std::shared_ptr<Constraint>& con) {
      assert(checkVariablesImpl(con) && "Constraint depends on variables that are not associated with this OptimizationProblem");
    }
    bool checkVariablesImpl(const std::shared_ptr<Constraint>& con) {
      // todo: implement this
      return true;
    }

    // uses virtual methods to crawl up the complexity hiearchy as new decision variables and constraints are added to the program
    // note that there is dynamic allocation happening in here, but on a structure of negligible size.  (is there a better way?)
    struct MathematicalProgram {
    /* these would be used to fill out the optimization hierarchy prototyped below
      virtual MathematicalProgram* addIntegerVariable() { return new MathematicalProgram; };

      virtual MathematicalProgram* addLinearCost() { return new MathematicalProgram; };
      virtual MathematicalProgram* addQuadraticCost() { return new MathematicalProgram; };
      virtual MathematicalProgram* addCost() { return new MathematicalProgram; };

      virtual MathematicalProgram* addSumsOfSquaresConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addLinearMatrixInequalityConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addSecondOrderConeConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addComplementarityConstraint() { return new MathematicalProgram; };
      */
      virtual MathematicalProgram* addGenericObjective() { return new MathematicalProgram; };
      virtual MathematicalProgram* addGenericConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addLinearConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addLinearEqualityConstraint() { return new MathematicalProgram; };
      virtual bool solve(OptimizationProblem& prog) const { throw std::runtime_error("not implemented yet"); }
    };

    struct NonlinearProgram : public MathematicalProgram {
      virtual MathematicalProgram* addGenericObjective() override { return new NonlinearProgram; };
      virtual MathematicalProgram* addGenericConstraint() override { return new NonlinearProgram; };
      virtual MathematicalProgram* addLinearConstraint() override { return new NonlinearProgram; };
      virtual MathematicalProgram* addLinearEqualityConstraint() override { return new NonlinearProgram; };

      virtual bool solve(OptimizationProblem &prog) const override {
        if (hasSNOPT()) return solveWithSNOPT(prog);
        return MathematicalProgram::solve(prog);
      }

    protected:
      bool solveWithSNOPT(OptimizationProblem& prog) const;
      bool hasSNOPT() const;
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
    struct LeastSquares : public NonlinearProgram { //public LinearProgram, public LinearComplementarityProblem {
      virtual MathematicalProgram* addLinearEqualityConstraint() override { return new LeastSquares; };
      virtual bool solve(OptimizationProblem& prog) const override {
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
