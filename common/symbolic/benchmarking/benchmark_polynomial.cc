#include <fmt/format.h>

#include "drake/common/symbolic/monomial_util.h"
#include "drake/common/symbolic/polynomial.h"
#include "drake/tools/performance/fixture_common.h"

namespace drake {
namespace symbolic {
namespace {
static void BenchmarkPolynomialEvaluatePartial(
    benchmark::State& state) {  // NOLINT
  Eigen::Matrix<symbolic::Variable, 12, 1> x;
  for (int i = 0; i < 12; ++i) {
    x(i) = symbolic::Variable("x" + std::to_string(i));
  }
  const symbolic::Variables x_set(x);
  const auto monomial_basis =
      internal::ComputeMonomialBasis<Eigen::Dynamic>(x_set, 2);
  MatrixX<symbolic::Variable> S(monomial_basis.rows(), monomial_basis.rows());
  VectorX<symbolic::Variable> S_upper((S.rows() + 1) * S.rows() / 2);
  int upper_count = 0;
  for (int i = 0; i < monomial_basis.rows(); ++i) {
    S(i, i) = symbolic::Variable(fmt::format("S({}, {})", i, i));
    S_upper(upper_count++) = S(i, i);
    for (int j = i + 1; j < monomial_basis.rows(); ++j) {
      S(i, j) = symbolic::Variable(fmt::format("S({}, {})", i, j));
      S_upper(upper_count++) = S(i, j);
      S(j, i) = S(i, j);
    }
  }

  symbolic::Polynomial p;
  for (int i = 0; i < S.rows(); ++i) {
    p.AddProduct(S(i, i), symbolic::pow(monomial_basis(i), 2));
    for (int j = i + 1; j < S.cols(); ++j) {
      p.AddProduct(2 * S(i, j), monomial_basis(i) * monomial_basis(j));
    }
  }
  Eigen::MatrixXd S_val = Eigen::MatrixXd::Ones(S.rows(), S.cols());
  symbolic::Environment env;
  env.insert(S_upper, Eigen::VectorXd::Ones(S_upper.rows()));
  for (auto _ : state) {
    const auto p_evaluate = p.EvaluatePartial(env);
  }
}

BENCHMARK(BenchmarkPolynomialEvaluatePartial);
}  // namespace
}  // namespace symbolic
}  // namespace drake
