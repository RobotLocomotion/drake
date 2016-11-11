#pragma once

#include <functional>
#include <type_traits>

namespace drake {
/** @name cond
  Constructs conditional expression (similar to Lisp's cond).

  @verbatim
    cond(cond_1, exp_1,
         cond_2, exp_2,
            ...,   ...,
         cond_n, exp_n,
         exp_{n+1})
  @endverbatim

  The value returned by the above cond expression is @c exp_1 if @c cond_1 is
  true; else if @c cond_2 is true then @c exp_2; ... ; else if @c cond_n is true
  then @c exp_n. If none of the conditions are true, it returns @c exp_{n+1}.

  @note This functions assumes that @p ScalarType provides @c operator< and the
  type of @c f_cond is the type of the return type of <tt>operator<(ScalarType,
  ScalarType)</tt>. For example, @c symbolic::Expression can be used as a @p
  ScalarType because it provides <tt>symbolic::Formula
  operator<(symbolic::Expression, symbolic::Expression)</tt>.

 */
///@{
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
