#pragma once

#include <limits>
#include <memory>
#include <set>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace test {
enum class CostForm {
  kGeneric = 0,
  kNonSymbolic = 1,
  kSymbolic = 2,
};

enum class ConstraintForm {
  kGeneric = 0,
  kNonSymbolic = 1,
  kSymbolic = 2,
  kFormula = 3,
};

class OptimizationProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OptimizationProgram)

  OptimizationProgram(CostForm cost_form, ConstraintForm cnstr_form);

  virtual ~OptimizationProgram() {}

  MathematicalProgram* prog() const { return prog_.get(); }

  virtual void CheckSolution(SolverType solver_type) const = 0;

  double GetSolverSolutionDefaultCompareTolerance(SolverType solver_type) const;

  void RunProblem(MathematicalProgramSolverInterface* solver);

 private:
  std::unique_ptr<MathematicalProgram> prog_;
};

/**
 * Simple example x = b
 */
class LinearSystemExample1 {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearSystemExample1)

  LinearSystemExample1();
  virtual ~LinearSystemExample1() {}

  MathematicalProgram* prog() const { return prog_.get(); }

  const VectorDecisionVariable<4>& x() const { return x_; }

  const Eigen::Vector4d b() const { return b_; }

  std::shared_ptr<LinearEqualityConstraint> con() const { return con_; }

  virtual bool CheckSolution() const;

 protected:
  double tol() const { return 1E-10; }

 private:
  std::unique_ptr<MathematicalProgram> prog_;
  VectorDecisionVariable<4> x_;
  Eigen::Vector4d b_;
  std::shared_ptr<LinearEqualityConstraint> con_;
};

/**
 * Simple linear system
 *     x    = b
 * 2 * y(0) = b(0)
 * 2 * y(1) = b(1)
 */
class LinearSystemExample2 : public LinearSystemExample1 {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearSystemExample2)

  LinearSystemExample2();
  ~LinearSystemExample2() override {}

  VectorDecisionVariable<2> y() const { return y_; }

  bool CheckSolution() const override;

 private:
  VectorDecisionVariable<2> y_;
};

/**
 * Simple linear system
 * 3 * x    = b
 * 2 * y(0) = b(0)
 * 2 * y(1) = b(1)
 */
class LinearSystemExample3 : public LinearSystemExample2 {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearSystemExample3)

  LinearSystemExample3();
  ~LinearSystemExample3() override {}

  bool CheckSolution() const override;
};

/**
 * For a stable linear system ẋ = A x, find its Lyapunov function by solving
 * the Lyapunov equality on the symmetric matrix X
 * Aᵀ * X + X * A = -E
 */
class LinearMatrixEqualityExample {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearMatrixEqualityExample)

  LinearMatrixEqualityExample();

  MathematicalProgram* prog() const { return prog_.get(); }

  bool CheckSolution() const;

 private:
  std::unique_ptr<MathematicalProgram> prog_;
  MatrixDecisionVariable<3, 3> X_;
  Eigen::Matrix3d A_;
};

/// This test comes from Section 2.2 of
/// Handbook of Test Problems in Local and Global Optimization.
/// © 1999
/// ISBN 978-1-4757-3040-1
class NonConvexQPproblem1 {
  /// This is a non-convex quadratic program with inequality constraints.
  /// We choose to add the cost and constraints through different forms,
  /// to test different solvers, and whether MathematicalProgram can parse
  /// constraints in different forms.
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NonConvexQPproblem1)

  static std::vector<CostForm> cost_forms() {
    std::vector<CostForm> costs{CostForm::kGeneric, CostForm::kNonSymbolic};
    return costs;
  }

  static ::std::vector<ConstraintForm> constraint_forms() {
    std::vector<ConstraintForm> cnstr{ConstraintForm::kSymbolic,
                                      ConstraintForm::kNonSymbolic};
    return cnstr;
  }

  NonConvexQPproblem1(CostForm cost_form, ConstraintForm constraint_form);

  MathematicalProgram* prog() const { return prog_.get(); }

  bool CheckSolution() const;

 private:
  class TestProblem1Cost {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TestProblem1Cost)

    TestProblem1Cost() = default;

    static size_t numInputs() { return 5; }
    static size_t numOutputs() { return 1; }

    template <typename ScalarType>
    void eval(detail::VecIn<ScalarType> const& x,
              // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              detail::VecOut<ScalarType>& y) const {
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
      DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
      y(0) = (-50.0 * x(0) * x(0)) + (42 * x(0)) - (50.0 * x(1) * x(1)) +
             (44 * x(1)) - (50.0 * x(2) * x(2)) + (45 * x(2)) -
             (50.0 * x(3) * x(3)) + (47 * x(3)) - (50.0 * x(4) * x(4)) +
             (47.5 * x(4));
    }
  };

  void AddConstraint();

  void AddSymbolicConstraint();

  void AddQuadraticCost();

  std::unique_ptr<MathematicalProgram> prog_;
  VectorDecisionVariable<5> x_;
  Eigen::Matrix<double, 5, 1> x_expected_;
};

/// This test comes from Section 2.3 of
/// Handbook of Test Problems in Local and Global Optimization.
/// © 1999
/// ISBN 978-1-4757-3040-1
class NonConvexQPproblem2 {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NonConvexQPproblem2)

  static std::vector<CostForm> cost_forms() {
    std::vector<CostForm> costs{CostForm::kGeneric, CostForm::kNonSymbolic};
    return costs;
  }

  static std::vector<ConstraintForm> constraint_forms() {
    std::vector<ConstraintForm> cnstr{ConstraintForm::kNonSymbolic,
                                      ConstraintForm::kSymbolic};
    return cnstr;
  }

  NonConvexQPproblem2(CostForm cost_form, ConstraintForm cnstr_form);

  bool CheckSolution() const;

  MathematicalProgram* prog() const { return prog_.get(); }

 private:
  class TestProblem2Cost {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TestProblem2Cost)

    TestProblem2Cost() = default;

    static size_t numInputs() { return 6; }
    static size_t numOutputs() { return 1; }

    template <typename ScalarType>
    void eval(detail::VecIn<ScalarType> const& x,
              // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              detail::VecOut<ScalarType>& y) const {
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
      DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
      y(0) = (-50.0 * x(0) * x(0)) + (-10.5 * x(0)) - (50.0 * x(1) * x(1)) +
             (-7.5 * x(1)) - (50.0 * x(2) * x(2)) + (-3.5 * x(2)) -
             (50.0 * x(3) * x(3)) + (-2.5 * x(3)) - (50.0 * x(4) * x(4)) +
             (-1.5 * x(4)) + (-10.0 * x(5));
    }
  };

  void AddQuadraticCost();

  void AddNonSymbolicConstraint();

  void AddSymbolicConstraint();

  std::unique_ptr<MathematicalProgram> prog_;
  Eigen::Matrix<symbolic::Variable, 6, 1> x_;
  Eigen::Matrix<double, 6, 1> x_expected_;
};

/// This test comes from Section 3.4 of
/// Handbook of Test Problems in Local and Global Optimization.
/// © 1999
/// ISBN 978-1-4757-3040-1
class LowerBoundedProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LowerBoundedProblem)

  static std::vector<ConstraintForm> constraint_forms() {
    std::vector<ConstraintForm> cnstr{ConstraintForm::kNonSymbolic,
                                      ConstraintForm::kSymbolic};
    return cnstr;
  }

  explicit LowerBoundedProblem(ConstraintForm cnstr_form);

  bool CheckSolution() const;

  MathematicalProgram* prog() { return prog_.get(); }

  void SetInitialGuess1();

  void SetInitialGuess2();

 private:
  class LowerBoundTestCost {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LowerBoundTestCost)

    LowerBoundTestCost() = default;

    static size_t numInputs() { return 6; }
    static size_t numOutputs() { return 1; }

    template <typename ScalarType>
    void eval(detail::VecIn<ScalarType> const& x,
              // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              detail::VecOut<ScalarType>& y) const {
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
      DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
      y(0) = -25 * (x(0) - 2) * (x(0) - 2) + (x(1) - 2) * (x(1) - 2) -
             (x(2) - 1) * (x(2) - 1) - (x(3) - 4) * (x(3) - 4) -
             (x(4) - 1) * (x(4) - 1) - (x(5) - 4) * (x(5) - 4);
    }
  };

  class LowerBoundTestConstraint : public Constraint {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LowerBoundTestConstraint)

    LowerBoundTestConstraint(int i1, int i2)
        : Constraint(
              1, Eigen::Dynamic, Vector1d::Constant(4),
              Vector1d::Constant(std::numeric_limits<double>::infinity())),
          i1_(i1),
          i2_(i2) {}

   protected:
    // For just these two types, implementing this locally is almost cleaner...
    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                Eigen::VectorXd& y) const override {
      EvalImpl(x, y);
    }
    void DoEval(const Eigen::Ref<const TaylorVecXd>& x,
                TaylorVecXd& y) const override {
      EvalImpl(x, y);
    }

   private:
    template <typename ScalarType>
    void EvalImpl(
        const Eigen::Ref<const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>>& x,
        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
        Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& y) const {
      y.resize(1);
      y(0) = (x(i1_) - 3) * (x(i1_) - 3) + x(i2_);
    }

    int i1_;
    int i2_;
  };

  void AddSymbolicConstraint();

  void AddNonSymbolicConstraint();

  std::unique_ptr<MathematicalProgram> prog_;
  Eigen::Matrix<symbolic::Variable, 6, 1> x_;
  Eigen::Matrix<double, 6, 1> x_expected_;
};

/// gloptiPolyConstrainedMinimization
/// @brief From section 5.8.2 of the gloptipoly3 documentation.
///
/// Which is from section 3.5 in
///   Handbook of Test Problems in Local and Global Optimization
///   © 1999
///   ISBN 978-1-4757-3040-1
/// We deliberately duplicate the problem, with the same constraints and
/// costs on decision variables x and y, so as to test out program works
/// correctly with multiple decision variables.
class GloptiPolyConstrainedMinimizationProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GloptiPolyConstrainedMinimizationProblem)

  static std::vector<CostForm> cost_forms() {
    std::vector<CostForm> costs{CostForm::kGeneric, CostForm::kNonSymbolic,
                                CostForm::kSymbolic};
    return costs;
  }

  static std::vector<ConstraintForm> constraint_forms() {
    std::vector<ConstraintForm> cnstr{ConstraintForm::kNonSymbolic,
                                      ConstraintForm::kSymbolic};
    return cnstr;
  }

  GloptiPolyConstrainedMinimizationProblem(CostForm cost_form,
                                           ConstraintForm cnstr_form);

  MathematicalProgram* prog() const { return prog_.get(); }

  bool CheckSolution() const;

 private:
  class GloptipolyConstrainedExampleCost {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GloptipolyConstrainedExampleCost)

    GloptipolyConstrainedExampleCost() = default;

    static size_t numInputs() { return 3; }
    static size_t numOutputs() { return 1; }

    template <typename ScalarType>
    void eval(detail::VecIn<ScalarType> const& x,
              // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              detail::VecOut<ScalarType>& y) const {
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
      DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
      y(0) = -2 * x(0) + x(1) - x(2);
    }
  };

  class GloptipolyConstrainedExampleConstraint
      : public Constraint {  // Want to also support deriving directly from
                             // constraint without going through Function.
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GloptipolyConstrainedExampleConstraint)

    GloptipolyConstrainedExampleConstraint()
        : Constraint(
              1, 3, Vector1d::Constant(0),
              Vector1d::Constant(std::numeric_limits<double>::infinity())) {}

   protected:
    // For just these two types, implementing this locally is almost cleaner.
    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                Eigen::VectorXd& y) const override {
      EvalImpl(x, &y);
    }
    void DoEval(const Eigen::Ref<const TaylorVecXd>& x,
                TaylorVecXd& y) const override {
      EvalImpl(x, &y);
    }

   private:
    template <typename ScalarType>
    void EvalImpl(
        const Eigen::Ref<const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>>& x,
        Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>* y) const {
      y->resize(1);
      (*y)(0) = 24 - 20 * x(0) + 9 * x(1) - 13 * x(2) + 4 * x(0) * x(0) -
                4 * x(0) * x(1) + 4 * x(0) * x(2) + 2 * x(1) * x(1) -
                2 * x(1) * x(2) + 2 * x(2) * x(2);
    }
  };

  void AddGenericCost();

  void AddSymbolicCost();

  void AddNonSymbolicCost();

  void AddNonSymbolicConstraint();

  void AddSymbolicConstraint();

  std::unique_ptr<MathematicalProgram> prog_;
  VectorDecisionVariable<3> x_;
  VectorDecisionVariable<3> y_;
  Eigen::Vector3d expected_;
};

/// An SOCP with Lorentz cone and rotated Lorentz cone constraints.
/// The objective is to find the smallest distance from a hyperplane
/// A * x = b to the origin.
/// We can solve the following SOCP with Lorentz cone constraint
/// min  t
///  s.t t >= sqrt(xᵀ*x)
///      A * x = b.
/// Alternatively, we can solve the following SOCP with rotated Lorentz cone
/// constraint
/// min t
/// s.t t >= xᵀ*x
///     A * x = b.
///
/// The optimal solution of this equality constrained QP can be found using
/// Lagrangian method. The optimal solution x* and Lagrangian multiplier z*
/// satisfy
/// A_hat * [x*; z*] = [b; 0]
/// where A_hat = [A 0; 2*I Aᵀ].
class MinDistanceFromPlaneToOrigin {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MinDistanceFromPlaneToOrigin)

  static std::vector<CostForm> cost_forms() {
    std::vector<CostForm> costs{CostForm::kNonSymbolic, CostForm::kSymbolic};
    return costs;
  }

  static std::vector<ConstraintForm> constraint_forms() {
    std::vector<ConstraintForm> cnstr{ConstraintForm::kNonSymbolic,
                                      ConstraintForm::kSymbolic};
    return cnstr;
  }

  MinDistanceFromPlaneToOrigin(const Eigen::MatrixXd& A,
                               const Eigen::VectorXd& b, CostForm cost_form,
                               ConstraintForm cnstr_form);

  MathematicalProgram* prog_lorentz() const { return prog_lorentz_.get(); }

  MathematicalProgram* prog_rotated_lorentz() const {
    return prog_rotated_lorentz_.get();
  }

  void SetInitialGuess();

  bool CheckSolution(bool is_rotated_cone) const;

 private:
  void AddNonSymbolicConstraint();
  void AddSymbolicConstraint();

  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  std::unique_ptr<MathematicalProgram> prog_lorentz_;
  std::unique_ptr<MathematicalProgram> prog_rotated_lorentz_;
  VectorDecisionVariable<1> t_lorentz_;
  VectorXDecisionVariable x_lorentz_;
  VectorDecisionVariable<1> t_rotated_lorentz_;
  VectorXDecisionVariable x_rotated_lorentz_;
  Eigen::VectorXd x_expected_;
};

/**
 * A simple convex optimization program
 * min -12 * x + x³
 * s.t  x >= 0
 * Notice the objective function is convex in the feasible region x >= 0
 * The optimal solution is x = 2.
 */
class ConvexCubicProgramExample : public MathematicalProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConvexCubicProgramExample)

  ConvexCubicProgramExample();

  ~ConvexCubicProgramExample() override {};

  bool CheckSolution() const;

 private:
  VectorDecisionVariable<1> x_;
};

std::set<CostForm> linear_cost_form();

std::set<CostForm> quadratic_cost_form();

std::set<ConstraintForm> linear_constraint_form();
}  // namespace test
}  // namespace solvers
}  // namespace drake
