#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_interface.h"
#include "drake/solvers/solver_type.h"

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

void ExpectSolutionCostAccurate(const MathematicalProgram& prog,
                                const MathematicalProgramResult& result,
                                double tol);

class OptimizationProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OptimizationProgram)

  OptimizationProgram(CostForm cost_form, ConstraintForm constraint_form);

  virtual ~OptimizationProgram() {}

  CostForm cost_form() const {return cost_form_;}

  ConstraintForm constraint_form() const {return constraint_form_;}

  MathematicalProgram* prog() const { return prog_.get(); }

  virtual const std::optional<Eigen::VectorXd>& initial_guess() const {
    return initial_guess_;
  }

  virtual void CheckSolution(const MathematicalProgramResult& result) const = 0;

  double GetSolverSolutionDefaultCompareTolerance(SolverId solver_id) const;

  void RunProblem(SolverInterface* solver);

 private:
  CostForm cost_form_;
  ConstraintForm constraint_form_;
  std::unique_ptr<MathematicalProgram> prog_;
  std::optional<Eigen::VectorXd> initial_guess_;
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

  const Eigen::Vector4d& initial_guess() const { return initial_guess_; }

  std::shared_ptr<LinearEqualityConstraint> con() const { return con_; }

  virtual void CheckSolution(const MathematicalProgramResult& result) const;

 protected:
  double tol() const { return 1E-10; }

 private:
  std::unique_ptr<MathematicalProgram> prog_;
  VectorDecisionVariable<4> x_;
  Eigen::Vector4d initial_guess_;
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

  Vector6<double> initial_guess() const { return Vector6<double>::Zero(); }

  VectorDecisionVariable<2> y() const { return y_; }

  void CheckSolution(const MathematicalProgramResult& result) const override;

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

  void CheckSolution(const MathematicalProgramResult& result) const override;
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

  void CheckSolution(const MathematicalProgramResult& result) const;

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

  Eigen::Matrix<double, 5, 1> initial_guess() const;

  void CheckSolution(const MathematicalProgramResult& result) const;

 private:
  class TestProblem1Cost {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TestProblem1Cost)

    TestProblem1Cost() = default;

    static size_t numInputs() { return 5; }
    static size_t numOutputs() { return 1; }

    template <typename ScalarType>
    void eval(internal::VecIn<ScalarType> const& x,
              internal::VecOut<ScalarType>* y) const {
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
      DRAKE_ASSERT(static_cast<size_t>(y->rows()) == numOutputs());
      (*y)(0) = (-50.0 * x(0) * x(0)) + (42 * x(0)) - (50.0 * x(1) * x(1)) +
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

  NonConvexQPproblem2(CostForm cost_form, ConstraintForm constraint_form);

  Vector6<double> initial_guess() const;

  void CheckSolution(const MathematicalProgramResult& result) const;

  MathematicalProgram* prog() const { return prog_.get(); }

 private:
  class TestProblem2Cost {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TestProblem2Cost)

    TestProblem2Cost() = default;

    static size_t numInputs() { return 6; }
    static size_t numOutputs() { return 1; }

    template <typename ScalarType>
    void eval(internal::VecIn<ScalarType> const& x,
              internal::VecOut<ScalarType>* y) const {
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
      DRAKE_ASSERT(static_cast<size_t>(y->rows()) == numOutputs());
      (*y)(0) = (-50.0 * x(0) * x(0)) + (-10.5 * x(0)) - (50.0 * x(1) * x(1)) +
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
  Vector6d x_expected_;
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

  explicit LowerBoundedProblem(ConstraintForm constraint_form);

  void CheckSolution(const MathematicalProgramResult& result) const;

  MathematicalProgram* prog() { return prog_.get(); }

  Vector6<double> initial_guess1() const;

  Vector6<double> initial_guess2() const;

 private:
  class LowerBoundTestCost {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LowerBoundTestCost)

    LowerBoundTestCost() = default;

    static size_t numInputs() { return 6; }
    static size_t numOutputs() { return 1; }

    template <typename ScalarType>
    void eval(internal::VecIn<ScalarType> const& x,
              internal::VecOut<ScalarType>* y) const {
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
      DRAKE_ASSERT(static_cast<size_t>(y->rows()) == numOutputs());
      (*y)(0) = -25 * (x(0) - 2) * (x(0) - 2) + (x(1) - 2) * (x(1) - 2) -
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
                Eigen::VectorXd* y) const override {
      EvalImpl(x, y);
    }
    void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                AutoDiffVecXd* y) const override {
      // Check that the autodiff vector was initialized to the proper (minimal)
      // size.
      EXPECT_EQ(x.size(), x(0).derivatives().size());
      EvalImpl(x, y);
    }
    void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
                VectorX<symbolic::Expression>*) const override {
      throw std::logic_error(
          "LowerBoundTestConstraint does not support symbolic evaluation.");
    }

   private:
    template <typename ScalarType>
    void EvalImpl(
        const Eigen::Ref<const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>>& x,
        Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>* y) const {
      y->resize(1);
      (*y)(0) = (x(i1_) - 3) * (x(i1_) - 3) + x(i2_);
    }

    int i1_;
    int i2_;
  };

  void AddSymbolicConstraint();

  void AddNonSymbolicConstraint();

  std::unique_ptr<MathematicalProgram> prog_;
  Eigen::Matrix<symbolic::Variable, 6, 1> x_;
  Vector6d x_expected_;
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
                                           ConstraintForm constraint_form);

  MathematicalProgram* prog() const { return prog_.get(); }

  void CheckSolution(const MathematicalProgramResult& result) const;

  Vector6<double> initial_guess() const;

 private:
  class GloptipolyConstrainedExampleCost {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GloptipolyConstrainedExampleCost)

    GloptipolyConstrainedExampleCost() = default;

    static size_t numInputs() { return 3; }
    static size_t numOutputs() { return 1; }

    template <typename ScalarType>
    void eval(internal::VecIn<ScalarType> const& x,
              internal::VecOut<ScalarType>* y) const {
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
      DRAKE_ASSERT(static_cast<size_t>(y->rows()) == numOutputs());
      (*y)(0) = -2 * x(0) + x(1) - x(2);
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
                Eigen::VectorXd* y) const override {
      EvalImpl(x, y);
    }
    void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                AutoDiffVecXd* y) const override {
      EvalImpl(x, y);
    }

    void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
                VectorX<symbolic::Expression>*) const override {
      throw std::logic_error(
          "GloptipolyConstrainedExampleConstraint does not support symbolic "
          "evaluation.");
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
                               ConstraintForm constraint_form);

  MathematicalProgram* prog_lorentz() const { return prog_lorentz_.get(); }

  MathematicalProgram* prog_rotated_lorentz() const {
    return prog_rotated_lorentz_.get();
  }

  Eigen::VectorXd prog_lorentz_initial_guess() const;

  Eigen::VectorXd prog_rotated_lorentz_initial_guess() const;

  void CheckSolution(const MathematicalProgramResult& result,
                     bool is_rotated_cone) const;

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

  void CheckSolution(const MathematicalProgramResult& result) const;

 private:
  VectorDecisionVariable<1> x_;
};

/**
 * A simple non-convex problem with a quadratic equality constraint
 * min 0
 * s.t xᵀx = 1
 * This test is meant to verify that we can add a quadratic constraint to a
 * program, and solve it through nonlinear optimization.
 */
class UnitLengthProgramExample : public MathematicalProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnitLengthProgramExample)

  UnitLengthProgramExample();

  ~UnitLengthProgramExample() override {};

  void CheckSolution(const MathematicalProgramResult& result,
                     double tolerance) const;

 private:
  VectorDecisionVariable<4> x_;
};

// Finds a point Q outside a tetrahedron, and with a specified distance to the
// tetrahedron. The tetrahedron's shape is fixed. Both the point and the
// tetrahedron can move in space.
// We pick this problem to break SNOPT 7.6, as explained in
// https://github.com/snopt/snopt-interface/issues/19#issuecomment-410346280
// This is just a feasibility problem, without a cost.
class DistanceToTetrahedronExample : public MathematicalProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DistanceToTetrahedronExample)

  explicit DistanceToTetrahedronExample(double distance_expected);

  ~DistanceToTetrahedronExample() override {}

  const VectorDecisionVariable<18>& x() const { return x_; }

  const Eigen::Matrix<double, 4, 3> A_tetrahedron() const {
    return A_tetrahedron_;
  }

  const Eigen::Vector4d b_tetrahedron() const { return b_tetrahedron_; }

 private:
  // TODO(hongkai.dai): explain the mathematical formulation of this constraint.
  class DistanceToTetrahedronNonlinearConstraint : public Constraint {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DistanceToTetrahedronNonlinearConstraint)

    DistanceToTetrahedronNonlinearConstraint(
        const Eigen::Matrix<double, 4, 3>& A_tetrahedron,
        const Eigen::Vector4d& b_tetrahedron);

    ~DistanceToTetrahedronNonlinearConstraint() override {}

   private:
    template <typename DerivedX, typename ScalarY>
    void DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                       VectorX<ScalarY>* y) const {
      DRAKE_DEMAND(x.size() == 18);
      y->resize(15);
      using ScalarX = typename DerivedX::Scalar;
      Vector3<ScalarX> p_WB = x.template head<3>();
      Vector3<ScalarX> p_WQ = x.template segment<3>(3);
      Vector3<ScalarX> n_W = x.template segment<3>(6);
      Vector4<ScalarX> quat_WB = x.template segment<4>(9);
      Vector3<ScalarX> p_WP = x.template segment<3>(13);
      ScalarX d = x(16);
      ScalarX phi = x(17);

      // p_BV are the vertices of the tetrahedron in the body frame B.
      Eigen::Matrix<double, 4, 3> p_BV;
      // clang-format off
      p_BV << 0, 0, 0,
              1, 0, 0,
              0, 1, 0,
              0, 0, 1;
      // clang-format on
      (*y)(0) = quat_WB.dot(quat_WB);
      (*y)(1) = n_W.dot(n_W);
      (*y)(2) = n_W.dot(p_WP) - d;
      (*y)(3) = phi - n_W.dot(p_WQ - p_WP);
      y->template segment<3>(4) = n_W * phi - p_WQ + p_WP;

      const ScalarX ww = quat_WB(0) * quat_WB(0);
      const ScalarX xx = quat_WB(1) * quat_WB(1);
      const ScalarX yy = quat_WB(2) * quat_WB(2);
      const ScalarX zz = quat_WB(3) * quat_WB(3);
      const ScalarX wx = quat_WB(0) * quat_WB(1);
      const ScalarX wy = quat_WB(0) * quat_WB(2);
      const ScalarX wz = quat_WB(0) * quat_WB(3);
      const ScalarX xy = quat_WB(1) * quat_WB(2);
      const ScalarX xz = quat_WB(1) * quat_WB(3);
      const ScalarX yz = quat_WB(2) * quat_WB(3);
      Matrix3<ScalarX> R_WB;
      // clang-format off
      R_WB <<  ww + xx - yy - zz, 2 * xy - 2 * wz, 2 * xz + 2 * wy,
               2 * xy + 2 * wz, ww  + yy - xx - zz, 2 * yz - 2 * wx,
               2 * xz - 2 * wy, 2 * yz + 2 * wx, ww + zz - xx - yy;
      // clang-format on
      for (int i = 0; i < 4; ++i) {
        const Vector3<ScalarX> p_WVi = p_WB + R_WB * p_BV.row(i).transpose();
        (*y)(7 + i) = n_W.dot(p_WVi) - d;
      }
      // A * (R_WBᵀ * (p_WQ - p_WB))
      y->template segment<4>(11) =
          A_tetrahedron_ * R_WB.transpose() * (p_WQ - p_WB);
    }

    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                Eigen::VectorXd* y) const override {
      DoEvalGeneric(x, y);
    }

    void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                AutoDiffVecXd* y) const override {
      DoEvalGeneric(x, y);
    }

    void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
                VectorX<symbolic::Expression>* y) const override {
      DoEvalGeneric(x.cast<symbolic::Expression>(), y);
    }

   private:
    Eigen::Matrix<double, 4, 3> A_tetrahedron_;
  };

  VectorDecisionVariable<18> x_;
  // The tetrahedron can be described as A_tetrahedron * x<=b_tetrahedron, where
  // x is the position of a point within the tetrahedron, in the tetrahedron
  // body frame B.
  Eigen::Matrix<double, 4, 3> A_tetrahedron_;
  Eigen::Vector4d b_tetrahedron_;
};

/**
 * This problem is taken from Pseudo-complementary algorithms for mathematical
 * programming by U. Eckhardt in Numerical Methods for Nonlinear Optimization,
 * 1972. This problem has a sparse gradient.
 * max x0
 * s.t x1 - exp(x0) >= 0
 *     x2 - exp(x1) >= 0
 *     0 <= x0 <= 100
 *     0 <= x1 <= 100
 *     0 <= x2 <= 10
 */
class EckhardtProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EckhardtProblem)

  explicit EckhardtProblem(bool set_sparsity_pattern);

  void CheckSolution(const MathematicalProgramResult& result, double tol) const;

  const MathematicalProgram& prog() const { return *prog_; }

 private:
  class EckhardtConstraint : public Constraint {
   public:
    explicit EckhardtConstraint(bool set_sparsity_pattern);

   private:
    template <typename T>
    void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                       VectorX<T>* y) const {
      using std::exp;
      y->resize(2);
      (*y)(0) = x(1) - exp(x(0));
      (*y)(1) = x(2) - exp(x(1));
    }

    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                Eigen::VectorXd* y) const override {
      DoEvalGeneric(x, y);
    }

    void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                AutoDiffVecXd* y) const override {
      DoEvalGeneric(x, y);
    }

    void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
                VectorX<symbolic::Expression>* y) const override {
      DoEvalGeneric<symbolic::Expression>(x.cast<symbolic::Expression>(), y);
    }
  };

  std::unique_ptr<MathematicalProgram> prog_;
  Vector3<symbolic::Variable> x_;
};

/**
 * Test dual solution for Eckhardt problem.
 */
void TestEckhardtDualSolution(const SolverInterface& solver,
                              const Eigen::Ref<const Eigen::VectorXd>& x_init,
                              double tol = 1e-6);

/**
 * This is problem 106 from  Test examples for Nonlinear Programming
 * Codes by Will Hock and Klaus Schittkowski, Springer. The constraint of this
 * problem has sparse gradient.
 */
class HeatExchangerDesignProblem {
 public:
  HeatExchangerDesignProblem();

  void CheckSolution(const MathematicalProgramResult& result, double tol) const;

  const MathematicalProgram& prog() const { return *prog_; }

 private:
  class HeatExchangerDesignConstraint1 : public solvers::Constraint {
   public:
    HeatExchangerDesignConstraint1();

    ~HeatExchangerDesignConstraint1() override {}

   private:
    template <typename T>
    void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                       VectorX<T>* y) const {
      y->resize(1);
      (*y)(0) = x(0) * x(5) - 833.33252 * x(3) - 100 * x(0) + 83333.333;
    }

    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                Eigen::VectorXd* y) const override {
      DoEvalGeneric(x, y);
    }

    void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                AutoDiffVecXd* y) const override {
      DoEvalGeneric(x, y);
    }

    void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
                VectorX<symbolic::Expression>* y) const override {
      DoEvalGeneric<symbolic::Expression>(x.cast<symbolic::Expression>(), y);
    }
  };

  class HeatExchangerDesignConstraint2 : public solvers::Constraint {
   public:
    HeatExchangerDesignConstraint2();

    ~HeatExchangerDesignConstraint2() override {}

   private:
    template <typename T>
    void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                       VectorX<T>* y) const {
      y->resize(2);
      (*y)(0) = x(0) * x(5) - 1250 * x(3) - x(0) * x(2) + 1250 * x(2);
      (*y)(1) = x(1) * x(6) - 1250000 - x(1) * x(3) + 2500 * x(3);
    }

    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                Eigen::VectorXd* y) const override {
      DoEvalGeneric(x, y);
    }

    void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                AutoDiffVecXd* y) const override {
      DoEvalGeneric(x, y);
    }

    void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
                VectorX<symbolic::Expression>* y) const override {
      DoEvalGeneric<symbolic::Expression>(x.cast<symbolic::Expression>(), y);
    }
  };
  std::unique_ptr<MathematicalProgram> prog_;
  Eigen::Matrix<symbolic::Variable, 8, 1> x_;
};

/// In Eigen's autodiff, when the derivatives() vector has empty size, it is
/// interpreted as 0 gradient (i.e., the gradient has value 0, with the size of
/// the gradient matrix being arbitrary). On the other hand, many solvers
/// interpret empty size gradient in a different way, that the variable to be
/// taken derivative with has 0 size. This test guarantees that when Eigen
/// autodiff returns an empty size gradient, we can manually set the gradient
/// size to be the right size. This class represents the following trivial
/// problem
/// <pre>
/// min f(x)
/// s.t g(x) <= 0
/// </pre>
/// where f(x) = 1 and g(x) = 0. x.rows() == 2.
/// When evaluating f(x) and g(x), autodiff returns an empty gradient. But the
/// solvers expect to see gradient ∂f/∂x = [0 0] and ∂g/∂x = [0 0], namely
/// matrices of size 1 x 2, not empty size matrix. This test shows that we can
/// automatically set the gradient to the right size, although Eigen's autodiff
/// returns an empty size gradient.
class EmptyGradientProblem {
 public:
  EmptyGradientProblem();

  const MathematicalProgram& prog() const { return *prog_; }

  void CheckSolution(const MathematicalProgramResult& result) const;

 private:
  class EmptyGradientCost : public Cost {
   public:
    EmptyGradientCost() : Cost(2) {}

   private:
    template <typename T>
    void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>&,
                       VectorX<T>* y) const {
      y->resize(1);
      (*y)(0) = T(1);
    }

    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                VectorX<double>* y) const override {
      DoEvalGeneric(x, y);
    }

    void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                AutoDiffVecXd* y) const override {
      DoEvalGeneric(x, y);
    }

    void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
                VectorX<symbolic::Expression>* y) const override {
      DoEvalGeneric<symbolic::Expression>(x.cast<symbolic::Expression>(), y);
    }
  };

  class EmptyGradientConstraint : public Constraint {
   public:
    EmptyGradientConstraint();

   private:
    template <typename T>
    void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>&,
                       VectorX<T>* y) const {
      y->resize(1);
      (*y)(0) = T(0);
    }

    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                VectorX<double>* y) const override {
      DoEvalGeneric(x, y);
    }

    void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                AutoDiffVecXd* y) const override {
      DoEvalGeneric(x, y);
    }

    void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
                VectorX<symbolic::Expression>* y) const override {
      DoEvalGeneric<symbolic::Expression>(x.cast<symbolic::Expression>(), y);
    }
  };
  std::unique_ptr<MathematicalProgram> prog_;
  Vector2<symbolic::Variable> x_;
};

std::set<CostForm> linear_cost_form();

std::set<CostForm> quadratic_cost_form();

std::set<ConstraintForm> linear_constraint_form();
}  // namespace test
}  // namespace solvers
}  // namespace drake
