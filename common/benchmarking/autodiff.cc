#include <tuple>

#include "drake/common/ad/auto_diff.h"
#include "drake/tools/performance/fixture_common.h"
#include "drake/tools/performance/fixture_memory.h"

/* This is a micro-benchmark of our AutoDiff implementation w.r.t. Eigen's basic
operations (element-wise addition and multiplication, inner products, matrix
multiplication, etc.) */

namespace drake {
namespace ad {
namespace {

/* The AutoDiff scalar type being benchmarked. */
using AD = AutoDiff;

using Eigen::VectorXd;

/* Describes the scalar configuration to be used during benchmarking, intended
for use as a benchmark `->Args(...)` set of values to sweep. */
enum ScalarConfig : int {
  /* Non-autodiff (i.e., `double`). */
  kDerNone = -1,
  /* AutoDiff with empty derivatives (i.e., size() == 0). */
  kDerEmpty = 0,
  /* AutoDiff with dense derivatives (all partials are non-zero). */
  kDerDense = 2,
};

/* Unpacks the benchmark Args into a convenient tuple of (left_scalar,
right_scalar, size). */
std::tuple<ScalarConfig, ScalarConfig, int> GetArgs(
    const benchmark::State& state) {
  const ScalarConfig left_scalar = static_cast<ScalarConfig>(state.range(0));
  const ScalarConfig right_scalar = static_cast<ScalarConfig>(state.range(1));
  const int size = state.range(2);
  return std::make_tuple(left_scalar, right_scalar, size);
}

/* The size at compile time used for test cases with fixed-size vectors and
matrices. */
constexpr int kSmallSize = 3;

/* Returns T{value} with its derivatives set per the given config and size.
We've marked this `noinline` so that the AutoDiff logic doesn't peek inside this
function and optimize away any header-file branching; we want to benchmark the
full implementation without any compile-time short circuiting. */
template <typename T>
T __attribute__((noinline)) MakeScalar(double value, ScalarConfig config,
                                       int size) {
  if (config == kDerNone || config == kDerEmpty) {
    return value;
  }
  if constexpr (std::is_same_v<T, AD>) {
    if (config == kDerDense) {
      return AD(value, VectorXd::LinSpaced(size, 0.5, 2.5));
    }
  }
  DRAKE_UNREACHABLE();
}

/* Returns a Vector<T> or Matrix<T> filled with arbitrary data, with its
derivatives set per the given config and size. The FixedRows and FixedCols are
the compile-time matrix dimensions; when set to Eigen::Dynamic, the `size` will
be used instead. We've marked this `noinline` so that the AutoDiff logic doesn't
peek inside this function and optimize away any header-file branching; we want
to benchmark the full implementation without any compile-time short
circuiting. */
template <typename T, int FixedRows, int FixedCols>
auto __attribute__((noinline)) MakeMatrix(ScalarConfig config, int size) {
  const int num_rows = FixedRows >= 0 ? FixedRows : size;
  const int num_cols = FixedCols >= 0 ? FixedCols : size;
  Eigen::Matrix<T, FixedRows, FixedCols> result(num_rows, num_cols);
  for (int row = 0; row < num_rows; ++row) {
    for (int col = 0; col < num_cols; ++col) {
      const double value = 5 * std::cos(row + 2 * col);
      if (config == kDerNone || config == kDerEmpty) {
        result(row, col) = value;
        continue;
      }
      if constexpr (std::is_same_v<T, AD>) {
        if (config == kDerDense) {
          const double lo = 5 * std::cos(row + 7 * col);
          const double hi = 5 * std::cos(row + 13 * col);
          result(row, col) = AD(value, VectorXd::LinSpaced(size, lo, hi));
          continue;
        }
      }
      DRAKE_UNREACHABLE();
    }
  }
  return result;
}

/* This function projects the runtime args into compile-time args, so that we
can use `if constexpr` within the benchmarking logic. Specifically, this calls
func<...>() with two compile-time template arguments based on the runtime args
of the given `state`:
- int FixedSize: either kSmallSize or Eigen::Dynamic
- typename RightScalar: either AD or double */
template <typename Func>
void RunWithCompileTimeArgs(Func&& func, benchmark::State& state) {  // NOLINT
  const auto& [left_scalar, right_scalar, size] = GetArgs(state);
  DRAKE_DEMAND(left_scalar != kDerNone);
  if (size == kSmallSize) {
    if (right_scalar == kDerNone) {
      func.template operator()<kSmallSize, double>();
    } else {
      func.template operator()<kSmallSize, AD>();
    }
  } else {
    if (right_scalar == kDerNone) {
      func.template operator()<Eigen::Dynamic, double>();
    } else {
      func.template operator()<Eigen::Dynamic, AD>();
    }
  }
}

// Measures broadcast multiply-accumulate:
//  `Vector<AD> * double + double` or
//  `Vector<AD> * AD     + AD`.
void ArrayScalarMac(benchmark::State& state) {  // NOLINT
  RunWithCompileTimeArgs(
      [&state]<int FixedSize, typename RightScalar>() {
        const auto& [left_scalar, right_scalar, size] = GetArgs(state);
        const auto lhs = MakeMatrix<AD, FixedSize, 1>(left_scalar, size);
        const auto mul = MakeScalar<RightScalar>(0.22, right_scalar, size);
        const auto add = MakeScalar<RightScalar>(0.33, right_scalar, size);
        tools::performance::TareMemoryManager();
        for (auto _ : state) {
          auto result = (lhs.array() * mul + add).eval();
          benchmark::DoNotOptimize(result);
        }
      },
      state);
}

// Measures elementwise multiply-accumulate:
//  `Vector<AD> ⊙ Vector<double> + Vector<double>` or
//  `Vector<AD> ⊙ Vector<AD>     + Vector<AD>`.
void ArrayArrayMac(benchmark::State& state) {  // NOLINT
  RunWithCompileTimeArgs(
      [&state]<int FixedSize, typename RightScalar>() {
        const auto& [left_scalar, right_scalar, size] = GetArgs(state);
        const auto lhs = MakeMatrix<AD, FixedSize, 1>(left_scalar, size);
        const auto mul =
            MakeMatrix<RightScalar, FixedSize, 1>(right_scalar, size);
        const auto add =
            MakeMatrix<RightScalar, FixedSize, 1>(right_scalar, size);
        tools::performance::TareMemoryManager();
        for (auto _ : state) {
          auto result = (lhs.array() * mul.array() + add.array()).eval();
          benchmark::DoNotOptimize(result);
        }
      },
      state);
}

// Measures vector dot product:
//  `Vector<AD> * Vector<double>` or
//  `Vector<AD> * Vector<AD>`.
void VectorDotProduct(benchmark::State& state) {  // NOLINT
  RunWithCompileTimeArgs(
      [&state]<int FixedSize, typename RightScalar>() {
        const auto& [left_scalar, right_scalar, size] = GetArgs(state);
        const auto lhs = MakeMatrix<AD, FixedSize, 1>(left_scalar, size);
        const auto rhs =
            MakeMatrix<RightScalar, FixedSize, 1>(right_scalar, size);
        tools::performance::TareMemoryManager();
        for (auto _ : state) {
          auto result = lhs.dot(rhs);
          benchmark::DoNotOptimize(result);
        }
      },
      state);
}

// Measures matrix multiplication:
//  `Matrix<AD> @ Matrix<AD>`.
// We don't test mixed AD-double because Eigen's GEMM doesn't support that.
void MatrixMultiply(benchmark::State& state) {  // NOLINT
  RunWithCompileTimeArgs(
      [&state]<int FixedSize, typename RightScalar>() {
        if constexpr (std::is_same_v<RightScalar, AD>) {
          const auto& [left_scalar, right_scalar, size] = GetArgs(state);
          const auto lhs =
              MakeMatrix<AD, FixedSize, FixedSize>(left_scalar, size);
          const auto rhs =
              MakeMatrix<AD, FixedSize, FixedSize>(right_scalar, size);
          tools::performance::TareMemoryManager();
          for (auto _ : state) {
            auto result = (lhs * rhs).eval();
            benchmark::DoNotOptimize(result);
          }
        } else {
          DRAKE_UNREACHABLE();
        }
      },
      state);
}

BENCHMARK(ArrayScalarMac)
    ->ArgsProduct({{kDerEmpty, kDerDense},
                   {kDerNone, kDerEmpty, kDerDense},
                   {kSmallSize, 100}})
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(ArrayArrayMac)
    ->ArgsProduct({{kDerEmpty, kDerDense},
                   {kDerNone, kDerEmpty, kDerDense},
                   {kSmallSize, 100}})
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(VectorDotProduct)
    ->ArgsProduct({{kDerEmpty, kDerDense},
                   {kDerNone, kDerEmpty, kDerDense},
                   {kSmallSize, 100}})
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(MatrixMultiply)
    ->ArgsProduct({{kDerEmpty, kDerDense},
                   {kDerEmpty, kDerDense},
                   {kSmallSize, 10}})
    ->Unit(benchmark::kMicrosecond);

}  // namespace
}  // namespace ad
}  // namespace drake
