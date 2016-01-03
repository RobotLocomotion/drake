#ifndef DRAKE_OPTIMIZATION_H
#define DRAKE_OPTIMIZATION_H

#include "Core.h"
#include <list>
#include <memory>

namespace Drake {

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
  /*
  class Constraint {
  public:
    Constraint(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& func, const Eigen::MatrixBase<DerivedA>& lb, const Eigen::MatrixBase<DerivedB>& ub)
      : eval(func), lower_bound(lb), upper_bound(ub)
    {};

    template <typename FunctionType, typename DerivedA, typename DerivedB>
    Constraint(const FunctionType& func, const Eigen::MatrixBase<DerivedA>& lb, const Eigen::MatrixBase<DerivedB>& ub)
      : eval([=] (const Eigen::VectorXd& x) {return func(x);}),
        lower_bound(lb), upper_bound(ub)
    {};

    isEqualityConstraint(void) { return (upper_bound-lower_bound).isZero(); }

    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> eval;
//    InputOutputRelation structure;

    Eigen::VectorXd lower_bound, upper_bound;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // DifferentiableConstraint, PolynomialConstraint, QuadraticConstraint, ComplementarityConstraint, IntegerConstraint, ...
*/
  /** LinearConstraint
   * @brief Implements a constraint of the form @f lb \le Ax \le ub @f
   */
/*
  class LinearConstraint { // : public Constraint {
  public:
    template <typename DerivedA,typename DerivedLB,typename DerivedUB>
    LinearConstraint(const Eigen::MatrixBase<DerivedA>& A,const Eigen::MatrixBase<DerivedLB>& lower_bound, const Eigen::MatrixBase<DerivedUB>& upper_bound)
            : A(A), lb(lower_bound), ub(upper_bound) {
      assert(lb.rows()==ub.rows());
      assert(A.rows()==lb.rows());
    }
//    template <typename FunctionType> LinearConstraint(lb,ub);  // construct using autodiff

    const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> & getMatrix() { return A; }
    const Eigen::Matrix<double,Eigen::Dynamic,1>& getLowerBound() { return lb; }
    const Eigen::Matrix<double,Eigen::Dynamic,1>& getUpperBound() { return ub; }

  private:
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>  A;
    Eigen::Matrix<double,Eigen::Dynamic,1> lb, ub;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
*/

  class OptimizationProblem {
  public:
    OptimizationProblem() : problem_type(new LeastSquares), num_vars(0) {};

    // todo: add methods that allow one to get a reference into a subset of each decision variable
    // e.g. var.block(3,2) returns a new structure which maps back into the original, but can be passed into the constraint methods below
    struct DecisionVariable {
      enum Type {
        CONTINUOUS,
        INTEGER,
        BINARY
      };
      Type type;
      std::string name;
      Eigen::VectorXd value;
      size_t start_index;
    };

    const DecisionVariable& addContinuousVariables(std::size_t num_new_vars, std::string name = "x") {
      DecisionVariable v;
      v.type = DecisionVariable::CONTINUOUS;
      v.name = name;
      v.value = Eigen::VectorXd::Zero(num_new_vars);
      v.start_index = num_vars;
      variables.push_back(v);
      num_vars += num_new_vars;

      Aeq.conservativeResize(Eigen::NoChange,num_vars);
      Aeq.rightCols(num_new_vars).setZero();

      return variables.back();
    }
//    const DecisionVariable& addIntegerVariables(size_t num_new_vars, std::string name);
//  ...

 // template <typename FunctionType>
 // void addCost(std::function..);
 // void addLinearCost(const Eigen::MatrixBase<Derived>& c,const vector<const DecisionVariable&>& vars)
 // void addQuadraticCost ...

//    void addConstraint(const LinearConstraint& constraint, const std::vector<const DecisionVariable&>& vars);
//  void addConstraint(const BoundingBoxConstraint& constraint, const std::vector<const DecisionVariable&>& vars);

    template <typename DerivedA,typename DerivedB>
    void addLinearEqualityConstraint(const Eigen::MatrixBase<DerivedA>& _Aeq,const Eigen::MatrixBase<DerivedB>& _beq) {
      assert(_Aeq.cols()==num_vars);
      assert(_Aeq.rows()==_beq.rows());
      problem_type.reset(problem_type->addLinearEqualityConstraint());
      size_t num_con = Aeq.rows();
      Aeq.conservativeResize(num_con+_Aeq.rows(),Eigen::NoChange);
      Aeq.bottomRows(_Aeq.rows()) = _Aeq;
      beq.conservativeResize(num_con+_beq.rows());
      beq.bottomRows(_beq.rows()) = _beq;
    }

    // todo: take in an initializer_list<const DecisionVariable&> instead
    template <typename DerivedA,typename DerivedB>
    void addLinearEqualityConstraint(const Eigen::MatrixBase<DerivedA>& _Aeq,const Eigen::MatrixBase<DerivedB>& _beq, const DecisionVariable& var) {
      assert(_Aeq.cols()==var.value.rows());
      assert(_Aeq.rows()==_beq.rows());
      problem_type.reset(problem_type->addLinearEqualityConstraint());
      size_t num_con = Aeq.rows();
      Aeq.conservativeResize(num_con+_Aeq.rows(),Eigen::NoChange);
      Aeq.bottomRows(_Aeq.rows()).setZero();
      Aeq.block(num_con,var.start_index,_Aeq.rows(),var.value.rows()) = _Aeq;
      beq.conservativeResize(num_con+_beq.rows());
      beq.bottomRows(_beq.rows()) = _beq;
    }
//    template <typename DerivedA,typename DerivedB>
//    void addLinearEqualityConstraint(const Eigen::MatrixBase<DerivedA>& Aeq,const Eigen::MatrixBase<DerivedB>& beq, const std::vector<const DecisionVariable&>& vars) {
//      problem_type.reset(problem_type->addLinearEqualityConstraint());
//    }

//    template <typename DerivedLB,typename DerivedUB>
//    void addBoundingBoxConstraint(const Eigen::MatrixBase<DerivedLB>& lower_bound, const Eigen::MatrixBase<DerivedUB>& upper_bound, const DecisionVariable& var) {  }


    bool solve() { return problem_type->solve(*this); }; // todo: add argument for options

//    template <typename Derived>
//    bool solve(const Eigen::MatrixBase<Derived>& x0);

//    getObjectiveValue();
//    getExitFlag();
//    getInfeasibleConstraintNames();

    void printSolution() {  // todo: overload the ostream operator instead?
      for (const auto& v : variables) {
        std::cout << v.name << " = " << v.value.transpose() << std::endl;
      }
    }


  private:
    Eigen::MatrixXd Aeq;
    Eigen::VectorXd beq;

    template <typename Derived>
    void setSolution(const Eigen::MatrixBase<Derived>& x) {
      size_t index=0;
      for (auto &v : variables) {
        v.value = x.middleRows(index,v.value.rows());
        index += v.value.rows();
      }
    }

    std::list<DecisionVariable> variables; // prefer list to vector because realloc in vector (when adding new decision variables) invalidates the old references
    size_t num_vars;

  private:
    // uses virtual methods to crawl up the complexity hiearchy as new decision variables and constraints are added to the program
    // note that there is dynamic allocation happening in here, but on a structure of negligible size.  (is there a better way?)
    struct MathematicalProgram {
      virtual MathematicalProgram* addNonlinearConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addLinearInequalityConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addLinearEqualityConstraint() { return new MathematicalProgram; };
      virtual bool solve(OptimizationProblem& prog) { throw std::runtime_error("not implemented yet"); }
    };
    struct LinearProgram : public MathematicalProgram {
      virtual MathematicalProgram* addLinearEqualityConstraint() { return new LinearProgram; };
      virtual MathematicalProgram* addLinearInequalityConstraint() { return new LinearProgram; };
    };
    struct LeastSquares : public LinearProgram {
      virtual MathematicalProgram* addLinearEqualityConstraint() { return new LeastSquares; };
      virtual bool solve(OptimizationProblem& prog) {
        // least-squares solution
        prog.setSolution(prog.Aeq.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(prog.beq));
        return true;
      }
    };
    std::shared_ptr<MathematicalProgram> problem_type;
  };

} // end namespace Drake

#endif //DRAKE_OPTIMIZATION_H
