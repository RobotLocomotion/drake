#pragma once

#include <functional>
#include <type_traits>

#include "drake/common/double_overloads.h"

namespace drake {
/** @name cond
  Constructs conditional expression (similar to Lisp's cond).

  @verbatim
    cond(cond_1, expr_1,
         cond_2, expr_2,
            ...,   ...,
         cond_n, expr_n,
         expr_{n+1})
  @endverbatim

  The value returned by the above cond expression is @c expr_1 if @c cond_1 is
  true; else if @c cond_2 is true then @c expr_2; ... ; else if @c cond_n is
  true then @c expr_n. If none of the conditions are true, it returns @c
  expr_{n+1}.

  @note This functions assumes that @p ScalarType provides @c operator< and the
  type of @c f_cond is the type of the return type of <tt>operator<(ScalarType,
  ScalarType)</tt>. For example, @c symbolic::Expression can be used as a @p
  ScalarType because it provides <tt>symbolic::Formula
  operator<(symbolic::Expression, symbolic::Expression)</tt>.


  @{
 */
template <typename ScalarType>
ScalarType cond(const ScalarType& e) {
  return e;
}
template <typename ScalarType, typename... Rest>
ScalarType cond(const decltype(ScalarType() < ScalarType()) & f_cond,
                const ScalarType& e_then, Rest... rest) {
  return if_then_else(f_cond, e_then, cond(rest...));
}
///@}
}  // namespace drake
