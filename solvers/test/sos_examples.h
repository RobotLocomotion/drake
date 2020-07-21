#pragma once
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace solvers {
/**
 * This is example 3.35 from Semidefinite Optimization and Convex Algebraic
 * Geometry by G. Blekherman, P. Parrilo and R. Thomas. Solve a semidefinite
 * programming problem to verify that the univariate quartic polynomial p(x)
 * = x⁴+4x³+6x²+4x+5 is sum-of-squares (sos).
 */
class UnivariateQuarticSos {
 public:
  UnivariateQuarticSos();

  const MathematicalProgram& prog() const { return prog_; }

  void CheckResult(const MathematicalProgramResult& result, double tol) const;

 private:
  MathematicalProgram prog_;
  symbolic::Polynomial p_;
  MatrixXDecisionVariable gram_;
  VectorX<symbolic::Monomial> monomial_basis_;
};

/**
 * This is example 3.38 from Semidefinite Optimization and Convex Algebraic
 * Geometry by G. Blekherman, P. Parrilo and R. Thomas. Solve a semidefinite
 * programming problem to verify that the bivariate quartic polynomial p(x, y) =
 * 2x⁴+5y⁴−2x²y²+2x³y+2x+2 is sum-of-squares (sos).
 */
class BivariateQuarticSos {
 public:
  BivariateQuarticSos();

  const MathematicalProgram& prog() const { return prog_; }

  void CheckResult(const MathematicalProgramResult& result, double tol) const;

 private:
  MathematicalProgram prog_;
  symbolic::Polynomial p_;
  MatrixXDecisionVariable gram_;
  VectorX<symbolic::Monomial> monomial_basis_;
};

/**
 * This is example 3.50 from Semidefinite Optimization and Convex Algebraic
 * Geometry by G. Blekherman, P. Parrilo and R. Thomas. Solve a simple sos
 * program
 * max a + b
 * s.t x⁴ + ax + 2+b is sos
 *     (a-b+1)x² + bx + 1 is sos
 */
class SimpleSos1 {
 public:
  SimpleSos1();

  const MathematicalProgram& prog() const { return prog_; }

  void CheckResult(const MathematicalProgramResult& result, double tol) const;

 private:
  MathematicalProgram prog_;
  symbolic::Variable a_;
  symbolic::Variable b_;
  symbolic::Variable x_;
  symbolic::Polynomial p1_;
  symbolic::Polynomial p2_;
  MatrixXDecisionVariable gram1_;
  MatrixXDecisionVariable gram2_;
  VectorX<symbolic::Monomial> monomial_basis1_;
  VectorX<symbolic::Monomial> monomial_basis2_;
};
}  // namespace solvers
}  // namespace drake
