#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace test {
/**
 * Simple example x = b
 */
class LinearSystemExample1 {
 public:
  LinearSystemExample1() : prog_(std::make_shared<MathematicalProgram>()), x_{}, b_{}, con_{} {
    x_ = prog_->NewContinuousVariables<4>();
    b_ = Eigen::Vector4d::Random();
    con_ = prog_->AddLinearEqualityConstraint(Eigen::Matrix4d::Identity(), b_, x_);
    prog_->SetInitialGuessForAllVariables(Eigen::Vector4d::Zero());
  }

  std::shared_ptr<MathematicalProgram> prog() const {return prog_;}

  const VectorDecisionVariable<4>& x() const {return x_;}

  const Eigen::Vector4d b() const {return b_;}

  std::shared_ptr<LinearEqualityConstraint> con() const {return con_;}

  virtual bool CheckSolution() const {
    auto x_sol = prog_->GetSolution(x_);
    if (!CompareMatrices(x_sol, b_, 1E-10, MatrixCompareType::absolute)) {
      return false;
    }
    for (int i = 0; i < 4; ++i) {
      if (std::abs(x_sol(i) - b_(i)) > 1E-10) {
        return false;
      }
      if (!CompareMatrices(x_sol.head(i), b_.head(i), 1E-10, MatrixCompareType::absolute)) {
        return false;
      }
    }
    return true;
  }

 private:
  std::shared_ptr<MathematicalProgram> prog_;
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
  LinearSystemExample2() : LinearSystemExample1(), y_{} {
    y_ = prog()->NewContinuousVariables<2>();
    prog()->AddLinearEqualityConstraint(2 * Eigen::Matrix2d::Identity(), b().topRows<2>(), y_);
  }

  VectorDecisionVariable<2> y() const {return y_;}

  bool CheckSolution() const override {
    if (!LinearSystemExample1::CheckSolution()) {
      return false;
    }
    if (!CompareMatrices(prog()->GetSolution(y_), b().topRows<2>()/2, 1E-10, MatrixCompareType::absolute)) {
      return false;
    }
    return true;
  }

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
  LinearSystemExample3() : LinearSystemExample2() {
    con()->UpdateConstraint(3 * Eigen::Matrix4d::Identity(), b());
  }

  bool CheckSolution() const override {
    if (!CompareMatrices(prog()->GetSolution(x()), b()/3, 1E-10, MatrixCompareType::absolute)) {
      return false;
    }
    if (!CompareMatrices(prog()->GetSolution(y()), b().topRows<2>() / 2, 1E-10, MatrixCompareType::absolute)) {
      return false;
    }
    return true;
  }
};

// This test comes from Section 2.2 of "Handbook of Test Problems in
// Local and Global Optimization."
class TestProblem1Cost {
 public:
  static size_t numInputs() { return 5; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void eval(detail::VecIn<ScalarType> const& x, detail::VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
    y(0) = (-50.0 * x(0) * x(0)) + (42 * x(0)) - (50.0 * x(1) * x(1)) +
        (44 * x(1)) - (50.0 * x(2) * x(2)) + (45 * x(2)) -
        (50.0 * x(3) * x(3)) + (47 * x(3)) - (50.0 * x(4) * x(4)) +
        (47.5 * x(4));
  }
};

class QPproblem1 {
  /// This is a convex quadratic program with inequality constraints. We choose
  /// to add the cost and constraints through different forms, to test different
  /// solvers, and whether MathematicalProgram can parse constraints in
  /// different forms.
 public:
  enum CostForm {
    kCostBegin = 0,
    kGenericCost = 0,
    kQuadraticCost = 1,
    kCostEnd = 1
    // TODO(hongkai.dai) Add quadratic symbolic cost
  };

  enum ConstraintForm {
    kConstraintBegin = 0,
    kSymbolicConstraint = 0,
    kNonSymbolicConstraint = 1,
    kConstraintEnd = 1
  };

 public:
  QPproblem1(CostForm cost_form, ConstraintForm constraint_form) : prog_(std::make_shared<MathematicalProgram>()), x_{}, x_expected_{} {
    x_ = prog_->NewContinuousVariables<5>("x");
    prog_->AddBoundingBoxConstraint(0, 1, x_);
    switch (cost_form) {
      case kGenericCost : {
        prog_->AddCost(TestProblem1Cost());
        break;
      }
      case kQuadraticCost : {
        AddQuadraticCost();
        break;
      }
      default:
        throw std::runtime_error("unsupported cost form");
    }
    switch (constraint_form) {
      case kSymbolicConstraint : {
        AddSymbolicConstraint();
        break;
      }
      case kNonSymbolicConstraint : {
        AddConstraint();
        break;
      }
      default:
        throw std::runtime_error("unsupported constraint form");
    }

    x_expected_ << 1, 1, 0, 1, 0;
    prog_->SetInitialGuess(x_, x_expected_ + 0.01 * Eigen::Matrix<double, 5, 1>::Random());
  }

  std::shared_ptr<MathematicalProgram> prog() const {return prog_;}

  bool CheckSolution() const {
    const auto& x_value = prog_->GetSolution(x_);
    return CompareMatrices(x_value, x_expected_, 1E-9, MatrixCompareType::absolute);
  }

 private:
  void AddConstraint() {
    Eigen::Matrix<double, 1, 5> a;
    a << 20, 12, 11, 7, 4;
    prog_->AddLinearConstraint(a, -std::numeric_limits<double>::infinity(), 40);
  }

  void AddSymbolicConstraint() {
    const auto constraint = 20 * x_(0) + 12 * x_(1) + 11 * x_(2) + 7 * x_(3) + 4 * x_(4);
    prog_->AddLinearConstraint(constraint, -std::numeric_limits<double>::infinity(), 40);
  }

  void AddQuadraticCost() {
    Eigen::Matrix<double, 5, 5> Q = -100 * Eigen::Matrix<double, 5, 5>::Identity();
    Eigen::Matrix<double, 5, 1> c;
    c << 42, 44, 45, 47, 47.5;
    prog_->AddQuadraticCost(Q, c);
  }

  std::shared_ptr<MathematicalProgram> prog_;
  VectorDecisionVariable<5> x_;
  Eigen::Matrix<double, 5, 1> x_expected_;
};
}  // namespace test
}  // namespace solvers
}  // namespace drake