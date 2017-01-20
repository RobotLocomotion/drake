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
}  // namespace test
}  // namespace solvers
}  // namespace drake