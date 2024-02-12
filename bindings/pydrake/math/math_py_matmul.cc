#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {
namespace {

using symbolic::Expression;
using symbolic::Monomial;
using symbolic::Variable;

template <typename T>
std::string_view GetDtypeName() {
  if constexpr (std::is_same_v<T, double>) {
    return "float";
  }
  if constexpr (std::is_same_v<T, AutoDiffXd>) {
    return "AutoDiffXd";
  }
  if constexpr (std::is_same_v<T, Variable>) {
    return "Variable";
  }
  if constexpr (std::is_same_v<T, Expression>) {
    return "Expression";
  }
  if constexpr (std::is_same_v<T, symbolic::Polynomial>) {
    return "Polynomial";
  }
  if constexpr (std::is_same_v<T, Monomial>) {
    return "Monomial";
  }
}

}  // namespace

void DefineMathMatmul(py::module m) {
  const auto bind = [&m]<typename T1, typename T2>() {
    using T3 = typename decltype(std::declval<MatrixX<T1>>() *
                                 std::declval<MatrixX<T2>>())::Scalar;
    // To avoid too much doc spam, we'll use a more descriptive docstring for
    // the first overload only.
    const std::string_view extra_doc =
        (std::is_same_v<T1, double> && std::is_same_v<T2, double>)
            ? " The numpy matmul ``A @ B`` is typically slow when multiplying "
              "user-defined dtypes such as AutoDiffXd or Expression. Use this "
              "function for better performance, e.g., ``matmul(A, B)``. For a "
              "dtype=float @ dtype=float, this might be a little slower than "
              "numpy, but is provided here for convenience so that the user "
              "doesn't need to be overly careful about the dtype of arguments."
            : "";
    const std::string doc = fmt::format(
        "Matrix product for dtype={} @ dtype={} -> dtype={}.{}",
        GetDtypeName<T1>(), GetDtypeName<T2>(), GetDtypeName<T3>(), extra_doc);
    m.def(
        "matmul",
        [](const Eigen::Ref<const MatrixX<T1>, 0, StrideX>& A,
            const Eigen::Ref<const MatrixX<T2>, 0, StrideX>& B) -> MatrixX<T3> {
          if constexpr (std::is_same_v<T3, AutoDiffXd>) {
            return A.template cast<AutoDiffXd>() *
                   B.template cast<AutoDiffXd>();
          } else {
            return A * B;
          }
        },
        doc.c_str());
  };  // NOLINT(readability/braces)

  // The ordering of the calls to `bind` here are sorted fastest-to-slowest to
  // ensure that overload resolution chooses the fastest one.

  // Bind a double-only overload for convenience.
  bind.operator()<double, double>();

  // Bind the AutoDiff-related overloads.
  bind.operator()<double, AutoDiffXd>();
  bind.operator()<AutoDiffXd, double>();
  bind.operator()<AutoDiffXd, AutoDiffXd>();

  // Bind the symbolic expression family of overloads.
  //
  // The order here is important for choosing the most specific types (e.g., so
  // that a Polynomial stays as a Polynomial instead of lapsing back into an
  // Expression). The order should go start from the narrowest type and work up
  // to the broadest type.
  //
  // - One operand is a double.
  bind.operator()<double, Variable>();
  bind.operator()<Variable, double>();
  bind.operator()<double, Monomial>();
  bind.operator()<Monomial, double>();
  bind.operator()<double, symbolic::Polynomial>();
  bind.operator()<symbolic::Polynomial, double>();
  bind.operator()<double, Expression>();
  bind.operator()<Expression, double>();
  // - One operand is a Variable.
  bind.operator()<Variable, Variable>();
  bind.operator()<Variable, Monomial>();
  bind.operator()<Monomial, Variable>();
  bind.operator()<Variable, symbolic::Polynomial>();
  bind.operator()<symbolic::Polynomial, Variable>();
  bind.operator()<Variable, Expression>();
  bind.operator()<Expression, Variable>();
  // - One operand is a Monomial.
  bind.operator()<Monomial, Monomial>();
  bind.operator()<Monomial, symbolic::Polynomial>();
  bind.operator()<symbolic::Polynomial, Monomial>();
  bind.operator()<Monomial, Expression>();
  bind.operator()<Expression, Monomial>();
  // - One operand is a Polynomial.
  bind.operator()<symbolic::Polynomial, symbolic::Polynomial>();
  bind.operator()<symbolic::Polynomial, Expression>();
  bind.operator()<Expression, symbolic::Polynomial>();
  // - Both operands are Expression.
  bind.operator()<Expression, Expression>();
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
