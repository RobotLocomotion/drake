#include <utility>

#include <fmt/format.h>

#include "drake/common/symbolic/monomial_util.h"
#include "drake/common/symbolic/polynomial.h"
#include "drake/tools/performance/fixture_common.h"

namespace drake {
namespace symbolic {
namespace {
void PolynomialEvaluatePartial(benchmark::State& state) {  // NOLINT
  Eigen::Matrix<symbolic::Variable, 12, 1> x =
      MakeVectorContinuousVariable(12, "x");
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
  symbolic::Environment env;
  // Test with S = 𝟏 (every entry of S is 1). During EvaluatePartial, we will
  // compute S(i, j) * monomial_basis(i) * monomial_basis(j). Since S(i, j) = 1,
  // each term will have non-zero coefficient (there won't be two terms
  // cancelling each other).
  env.insert(S_upper, Eigen::VectorXd::Ones(S_upper.rows()));
  for (auto _ : state) {
    const auto p_evaluate = p.EvaluatePartial(env);
  }
}

/* A benchmark for tr(Q @ X); Q's type is double, X's type is Variable. */
void MatrixInnerProduct(benchmark::State& state) {  // NOLINT
  const int n = state.range(0);
  const bool sparse_Q = state.range(1);
  Eigen::MatrixXd Q(n, n);
  if (sparse_Q) {
    Q.setZero();
    for (int i = 0; i < n; ++i) {
      Q(i, i) = std::sin(i);
    }
  } else {
    for (int i = 0; i < n; ++i) {
      Q(i, i) = std::sin(i);
      for (int j = i + 1; j < n; ++j) {
        Q(i, j) = std::cos(i + 2 * j);
        Q(j, i) = Q(i, j);
      }
    }
  }
  MatrixX<symbolic::Variable> X(n, n);
  for (int i = 0; i < n; ++i) {
    X(i, i) = symbolic::Variable(fmt::format("X({}, {})", i, i));
    for (int j = i + 1; j < n; ++j) {
      X(i, j) = symbolic::Variable(fmt::format("X({}, {})", i, j));
      X(j, i) = X(i, j);
    }
  }

  for (auto _ : state) {
    symbolic::Polynomial((Q * X).trace());
  }
}

/* Creates a pair of matrices with arbitrary data, and in some cases with some
matrix elements populated as variables instead of constants. The data types of
the returned matrices are T1 and T2, respectively.
@tparam T1 must be either double or Expression.
@tparam T2 must be either Variable or Expression. */
template <typename T1, typename T2>
std::pair<MatrixX<T1>, MatrixX<T2>> CreateGemmInput(int n) {
  MatrixX<T1> A(n, n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      int k = i + 2 * j;
      A(i, j) = std::cos(k);
      if constexpr (std::is_same_v<T1, double>) {
        // Nothing more to do.
      } else {
        static_assert(std::is_same_v<T1, Expression>);
        if ((k % 3) == 0) {
          A(i, j) *= Variable("X");
        }
      }
    }
  }

  MatrixX<T2> B(n, n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      if constexpr (std::is_same_v<T2, Variable>) {
        B(i, j) = Variable("X");
      } else {
        static_assert(std::is_same_v<T2, Expression>);
        int k = i + 7 * j;
        B(i, j) = std::cos(k);
        if ((k % 3) == 0) {
          B(i, j) *= Variable("X");
        }
      }
    }
  }

  return {std::move(A), std::move(B)};
}

// Benchmarking four pairs of types is sufficient to cover all of the
// interesting cases of symbolic matrix multiplication.

// A benchmark for A @ B where A is T=double and B is T=Variable.
void GemmDV(benchmark::State& state) {  // NOLINT
  const int n = state.range(0);
  const auto& [A, B] = CreateGemmInput<double, Variable>(n);
  for (auto _ : state) {
    (A * B).eval();
  }
}

// A benchmark for A @ B where A is T=double and B is T=Expression.
void GemmDE(benchmark::State& state) {  // NOLINT
  const int n = state.range(0);
  const auto& [A, B] = CreateGemmInput<double, Expression>(n);
  for (auto _ : state) {
    (A * B).eval();
  }
}

// A benchmark for A @ B where A is T=Variable and B is T=Expression.
void GemmVE(benchmark::State& state) {  // NOLINT
  const int n = state.range(0);
  const auto& [B, A] = CreateGemmInput<Expression, Variable>(n);
  for (auto _ : state) {
    (A * B).eval();
  }
}

// A benchmark for A @ B where both matrix types are Expression.
void GemmEE(benchmark::State& state) {  // NOLINT
  const int n = state.range(0);
  const auto& [A, B] = CreateGemmInput<Expression, Expression>(n);
  for (auto _ : state) {
    (A * B).eval();
  }
}

BENCHMARK(PolynomialEvaluatePartial)->Unit(benchmark::kMicrosecond);
BENCHMARK(MatrixInnerProduct)
    ->ArgsProduct({{10, 50, 100, 200}, {false, true}})
    ->Unit(benchmark::kSecond);
BENCHMARK(GemmDV)
    ->ArgsProduct({{10, 50, 100, 200}})
    ->Unit(benchmark::kMillisecond);
BENCHMARK(GemmDE)
    ->ArgsProduct({{10, 50, 100, 200}})
    ->Unit(benchmark::kMillisecond);
BENCHMARK(GemmVE)
    ->ArgsProduct({{10, 50, 100, 200}})
    ->Unit(benchmark::kMillisecond);
BENCHMARK(GemmEE)
    ->ArgsProduct({{10, 50, 100, 200}})
    ->Unit(benchmark::kMillisecond);
}  // namespace
}  // namespace symbolic
}  // namespace drake
