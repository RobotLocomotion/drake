#pragma once

#include <memory>

#include <Eigen/Dense>

namespace drake {
namespace solvers {
namespace detail {

template <typename ScalarType>
using VecIn = Eigen::Ref<Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> const>;
template <typename ScalarType>
using VecOut = Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>;

/** FunctionTraits
 * @brief Define interface to a function of the form y = f(x).
 */
template <typename F>
struct FunctionTraits {
  // TODO(bradking): add in/out relation, possibly distinguish
  // differentiable functions
  static size_t numInputs(F const& f) { return f.numInputs(); }
  static size_t numOutputs(F const& f) { return f.numOutputs(); }
  template <typename ScalarType>
  static void eval(F const& f, VecIn<ScalarType> const& x,
                   // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                   VecOut<ScalarType>& y) {
    f.eval(x, y);
  }
};

template <typename F>
struct FunctionTraits<std::reference_wrapper<F>> {
  static size_t numInputs(std::reference_wrapper<F> const& f) {
    return FunctionTraits<F>::numInputs(f.get());
  }
  static size_t numOutputs(std::reference_wrapper<F> const& f) {
    return FunctionTraits<F>::numOutputs(f.get());
  }
  template <typename ScalarType>
  static void eval(std::reference_wrapper<F> const& f,
                   // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                   VecIn<ScalarType> const& x, VecOut<ScalarType>& y) {
    FunctionTraits<F>::eval(f.get(), x, y);
  }
};

template <typename F>
struct FunctionTraits<std::shared_ptr<F>> {
  static size_t numInputs(std::shared_ptr<F> const& f) {
    return FunctionTraits<F>::numInputs(*f);
  }
  static size_t numOutputs(std::shared_ptr<F> const& f) {
    return FunctionTraits<F>::numOutputs(*f);
  }
  template <typename ScalarType>
  static void eval(std::shared_ptr<F> const& f, VecIn<ScalarType> const& x,
                   // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                   VecOut<ScalarType>& y) {
    FunctionTraits<F>::eval(*f, x, y);
  }
};

template <typename F>
struct FunctionTraits<std::unique_ptr<F>> {
  static size_t numInputs(std::unique_ptr<F> const& f) {
    return FunctionTraits<F>::numInputs(*f);
  }
  static size_t numOutputs(std::unique_ptr<F> const& f) {
    return FunctionTraits<F>::numOutputs(*f);
  }
  template <typename ScalarType>
  static void eval(std::unique_ptr<F> const& f, VecIn<ScalarType> const& x,
                   // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                   VecOut<ScalarType>& y) {
    FunctionTraits<F>::eval(*f, x, y);
  }
};

// idea: use templates to support multi-input, multi-output functions which
// implement, e.g.
// void eval(x1,..., xn,  y1,..., ym), and
// InputOutputRelation getInputOutputRelation(input_index, output_index)

}  // namespace detail
}  // namespace solvers
}  // namespace drake
