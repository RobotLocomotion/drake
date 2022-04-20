#include "drake/common/symbolic.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/tools/performance/fixture_common.h"

namespace drake {
namespace solvers {
namespace {

static void BenchmarkSosProgram1(benchmark::State& state) {  // NOLINT
  // Formulate a mathematical program with sum-of-squares constraints. This
  // will involve a lot of computation on symbolic::Polynomial and imposing
  // linear constraints.
  // We construct symbolic::Polynomial directly from a Monomial map instead of
  // parsing a symbolic::Expression.
  for (auto _ : state) {
    MathematicalProgram prog;
    // Impose a constraint p_i(x).dot(q_i(x)) is sos, where both p_i(x) and
    // q_i(x) are vectors of polynomials.
    const auto x = prog.NewIndeterminates<7>();
    const symbolic::Variables x_vars(x);
    const auto monomial_basis = symbolic::MonomialBasis(x_vars, 3);
    for (int i = 0; i < 10; ++i) {
      VectorX<symbolic::Polynomial> p_i(15);
      VectorX<symbolic::Polynomial> q_i(p_i.rows());
      for (int j = 0; j < p_i.rows(); ++j) {
        symbolic::Polynomial::MapType p_map;
        for (int k = 0; k < monomial_basis.rows(); ++k) {
          p_map.emplace(monomial_basis(k),
                        static_cast<double>(i * j) / 100.0 + k / 10.0 + 1);
        }
        p_i(j) = symbolic::Polynomial(std::move(p_map));
        q_i(j) = prog.NewFreePolynomial(x_vars, 2);
      }
      prog.AddSosConstraint(p_i.dot(q_i));
    }
  }
}

static void BenchmarkSosProgram2(benchmark::State& state) {  // NOLINT
  // Formulate a mathematical program with sum-of-squares constraints. This
  // will involve a lot of computation on symbolic::Polynomial and imposing
  // linear constraints. Specifically it would involve the multiplication
  // between two polynomials, one of the polynomial has all the coefficients as
  // double constants, and the other polynomial coefficients are linear
  // expressions, constructed using symbolic polynomial operations. This is
  // different from BenchmarkSosProgram1 where the second polynomial
  // coefficients are symbolic variables. We construct symbolic::Polynomial
  // directly from a Monomial map instead of parsing a symbolic::Expression.
  for (auto _ : state) {
    MathematicalProgram prog;
    // Impose a constraint p(x).dot(q(x)) is sos, where both p(x) and
    // q(x) are both polynomials.
    // where q(x) = ∂V/∂x * f(x)
    // This ∂V/∂x * f(x) is often found in Lyapunov stability analysis.
    const auto x = prog.NewIndeterminates<5>();
    const symbolic::Variables x_vars(x);
    const VectorX<symbolic::Monomial> monomial_basis =
        symbolic::MonomialBasis(x_vars, 3);
    const symbolic::Polynomial V = prog.NewFreePolynomial(x_vars, 4);
    const Eigen::Matrix<symbolic::Polynomial, 1, 5> dVdx = V.Jacobian(x);
    symbolic::Polynomial::MapType p_map;
    for (int k = 0; k < monomial_basis.rows(); ++k) {
      p_map.emplace(monomial_basis(k), static_cast<double>(k) / 10.0 + 1);
    }
    const symbolic::Polynomial p = symbolic::Polynomial(std::move(p_map));
    // Construct f(x) ∈ ℝⁿ[x] where n = x.rows()
    VectorX<symbolic::Polynomial> f(x.rows());
    for (int i = 0; i < x.rows(); ++i) {
      symbolic::Polynomial::MapType f_map;
      for (int k = 0; k < monomial_basis.rows(); ++k) {
        f_map.emplace(monomial_basis(k), static_cast<double>(k) * 2.0 + 1);
      }
      f(i) = symbolic::Polynomial(std::move(f_map));
    }
    const symbolic::Polynomial q = dVdx.dot(f);

    prog.AddSosConstraint(p * q);
  }
}

BENCHMARK(BenchmarkSosProgram1);
BENCHMARK(BenchmarkSosProgram2);
}  // namespace
}  // namespace solvers
}  // namespace drake

BENCHMARK_MAIN();
