#pragma once
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
 */
///@{
template <typename ScalarType>
ScalarType cond(const ScalarType& e) {
  return e;
}

template <typename ScalarType, typename BooleanType, typename... Rest>
ScalarType cond(const BooleanType& f_cond, const ScalarType& e_then,
                Rest... rest) {
  return if_then_else(f_cond, e_then, cond(rest...));
}
///@}
}  // namespace drake
