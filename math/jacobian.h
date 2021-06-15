#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

#include "drake/common/autodiff.h"

namespace drake {
namespace math {

/** Computes a matrix of AutoDiffScalars from which both the value and
   the Jacobian of a function
   @f[
   f:\mathbb{R}^{n\times m}\rightarrow\mathbb{R}^{p\times q}
   @f]
   (f: R^n*m -> R^p*q) can be extracted.

   The derivative vector for each AutoDiffScalar in the output contains the
   derivatives with respect to all components of the argument @f$ x @f$.

   The return type of this function is a matrix with the `best' possible
   AutoDiffScalar scalar type, in the following sense:
   - If the number of derivatives can be determined at compile time, the
     AutoDiffScalar derivative vector will have that fixed size.
   - If the maximum number of derivatives can be determined at compile time, the
     AutoDiffScalar derivative vector will have that maximum fixed size.
   - If neither the number, nor the maximum number of derivatives can be
     determined at compile time, the output AutoDiffScalar derivative vector
     will be dynamically sized.

   @p f should have a templated call operator that maps an Eigen matrix
   argument to another Eigen matrix. The scalar type of the output of @f$ f @f$
   need not match the scalar type of the input (useful in recursive calls to the
   function to determine higher order derivatives). The easiest way to create an
   @p f is using a C++14 generic lambda.

   The algorithm computes the Jacobian in chunks of up to @p MaxChunkSize
   derivatives at a time. This has three purposes:
   - It makes it so that derivative vectors can be allocated on the stack,
     eliminating dynamic allocations and improving performance if the maximum
     number of derivatives cannot be determined at compile time.
   - It gives control over, and limits the number of required
     instantiations of the call operator of f and all the functions it calls.
   - Excessively large derivative vectors can result in CPU capacity cache
     misses; even if the number of derivatives is fixed at compile time, it may
     be better to break up into chunks if that means that capacity cache misses
     can be prevented.

   @param f function
   @param x function argument value at which Jacobian will be evaluated
   @return AutoDiffScalar matrix corresponding to the Jacobian of f evaluated
   at x.
 */
template <int MaxChunkSize = 10, class F, class Arg>
decltype(auto) jacobian(F &&f, Arg &&x) {
  using Eigen::AutoDiffScalar;
  using Eigen::Index;
  using Eigen::Matrix;

  using ArgNoRef = typename std::remove_reference_t<Arg>;

  // Argument scalar type.
  using ArgScalar = typename ArgNoRef::Scalar;

  // Argument scalar type corresponding to return value of this function.
  using ReturnArgDerType = Matrix<ArgScalar, ArgNoRef::SizeAtCompileTime, 1, 0,
                                  ArgNoRef::MaxSizeAtCompileTime, 1>;
  using ReturnArgAutoDiffScalar = AutoDiffScalar<ReturnArgDerType>;

  // Return type of this function.
  using ReturnArgAutoDiffType =
      decltype(x.template cast<ReturnArgAutoDiffScalar>().eval());
  using ReturnType = decltype(f(std::declval<ReturnArgAutoDiffType>()));

  // Scalar type of chunk arguments.
  using ChunkArgDerType =
      Matrix<ArgScalar, Eigen::Dynamic, 1, 0, MaxChunkSize, 1>;
  using ChunkArgAutoDiffScalar = AutoDiffScalar<ChunkArgDerType>;

  // Allocate output.
  ReturnType ret;

  // Compute derivatives chunk by chunk.
  constexpr Index kMaxChunkSize = MaxChunkSize;
  Index num_derivs = x.size();
  bool values_initialized = false;
  for (Index deriv_num_start = 0; deriv_num_start < num_derivs;
       deriv_num_start += kMaxChunkSize) {
    // Compute chunk size.
    Index num_derivs_to_go = num_derivs - deriv_num_start;
    Index chunk_size = std::min(kMaxChunkSize, num_derivs_to_go);

    // Initialize chunk argument.
    auto chunk_arg = x.template cast<ChunkArgAutoDiffScalar>().eval();
    for (Index i = 0; i < x.size(); i++) {
      chunk_arg(i).derivatives().setZero(chunk_size);
    }
    for (Index i = 0; i < chunk_size; i++) {
      Index deriv_num = deriv_num_start + i;
      chunk_arg(deriv_num).derivatives()(i) = ArgScalar(1);
    }

    // Compute Jacobian chunk.
    auto chunk_result = f(chunk_arg);

    // On first chunk, resize output to match chunk and copy values from chunk
    // to result.
    if (!values_initialized) {
      ret.resize(chunk_result.rows(), chunk_result.cols());

      for (Index i = 0; i < chunk_result.size(); i++) {
        ret(i).value() = chunk_result(i).value();
        ret(i).derivatives().resize(num_derivs);
      }
      values_initialized = true;
    }

    // Copy derivatives from chunk to result.
    for (Index i = 0; i < chunk_result.size(); i++) {
      // Intuitive thing to do, but results in problems with non-matching scalar
      // types for recursive jacobian calls:
      // ret(i).derivatives().segment(deriv_num_start, chunk_size) =
      // chunk_result(i).derivatives();

      // Instead, assign each element individually, making use of conversion
      // constructors.
      for (Index j = 0; j < chunk_size; j++) {
        ret(i).derivatives()(deriv_num_start + j) =
            chunk_result(i).derivatives()(j);
      }
    }
  }

  return ret;
}

/** Computes a matrix of AutoDiffScalars from which the value, Jacobian,
   and Hessian of a function
   @f[
   f:\mathbb{R}^{n\times m}\rightarrow\mathbb{R}^{p\times q}
   @f]
   (f: R^n*m -> R^p*q) can be extracted.

   The output is a matrix of nested AutoDiffScalars, being the result of calling
   ::jacobian on a function that returns the output of ::jacobian,
   called on @p f.

   @p MaxChunkSizeOuter and @p MaxChunkSizeInner can be used to control chunk
   sizes (see ::jacobian).

   See ::jacobian for requirements on the function @p f and the argument
   @p x.

   @param f function
   @param x function argument value at which Hessian will be evaluated
   @return AutoDiffScalar matrix corresponding to the Hessian of f evaluated at
   x
 */
template <int MaxChunkSizeOuter = 10, int MaxChunkSizeInner = 10, class F,
          class Arg>
decltype(auto) hessian(F &&f, Arg &&x) {
  auto jac_fun = [&](const auto &x_inner) {
    return jacobian<MaxChunkSizeInner>(f, x_inner);
  };
  return jacobian<MaxChunkSizeOuter>(jac_fun, x);
}

}  // namespace math
}  // namespace drake
