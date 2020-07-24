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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnivariateQuarticSos)

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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BivariateQuarticSos)

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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleSos1)

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

/**
 * Prove that the Motzkin polynomial m(x, y) = x⁴y² + x²y⁴ + 1 − 3x²y² is
 * always non-negative.
 * One ceritificate for the proof is the existence of a polynomial r(x, y)
 * satisfying r(x, y) being sos and r(x, y) > 0  for all x, y ≠ 0, such that
 * r(x, y) * m(x, y) is sos.
 * So we solve the following problem
 * find r(x, y)
 * s.t r(x, y) - x² − y² is sos
 *     r(x, y) * m(x, y) is sos
 */
class MotzkinPolynomial {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MotzkinPolynomial)

  MotzkinPolynomial();

  const MathematicalProgram& prog() const { return prog_; }

  void CheckResult(const MathematicalProgramResult& result, double tol) const;

 private:
  MathematicalProgram prog_;
  symbolic::Variable x_;
  symbolic::Variable y_;
  symbolic::Polynomial m_;
  symbolic::Polynomial r_;
  MatrixXDecisionVariable gram1_;
  MatrixXDecisionVariable gram2_;
  VectorX<symbolic::Monomial> monomial_basis1_;
  VectorX<symbolic::Monomial> monomial_basis2_;
};

/**
 * Solve the following optimization problem for a univariate polynomial
 * max a + b + c
 * s.t p(x) = x⁴+ax³+bx²+c+1>=0 for all x >= 0
 *     p(1) = 1
 * According to theorem 3.71 in Semidefinite Optimization and Convex Algebraic
 * Geometry by G. Blekherman, P. Parrilo and R. Thomas, this is equivalent to
 * the following SOS problem
 * max a + b + c
 * s.t p(x) = s(x) + x * t(x)
 *     p(1) = 1
 *     s(x) is sos
 *     t(x) is sos
 */
class UnivariateNonnegative1 {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnivariateNonnegative1)

  UnivariateNonnegative1();

  const MathematicalProgram& prog() const { return prog_; }

  void CheckResult(const MathematicalProgramResult& result, double tol) const;

 private:
  MathematicalProgram prog_;
  symbolic::Variable a_;
  symbolic::Variable b_;
  symbolic::Variable c_;
  symbolic::Variable x_;
  symbolic::Polynomial p_;
  symbolic::Polynomial s_;
  symbolic::Polynomial t_;
  MatrixXDecisionVariable gram_s_;
  MatrixXDecisionVariable gram_t_;
};
}  // namespace solvers
}  // namespace drake
