#pragma once

#include <memory>
#include <type_traits>
#include <utility>

#include "drake/common/symbolic_expression.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/function.h"

namespace drake {
namespace solvers {

/** \addtogroup FunctionCostCreators */
/*@{*/

/**
 * Converts an input of type @p F to a FunctionCost object.
 * @tparam F This class should have functions numInputs(), numOutputs and
 * eval(x, y).
 * @see detail::FunctionTraits
 */
template <typename F>
std::shared_ptr<Cost> CreateFunctionCost(F&& f) {
  return std::make_shared<
      FunctionCost<typename std::remove_reference<F>::type>>(
      std::forward<F>(f));
}

namespace detail {

// From: drake-distro (git sha: 24452c1)
// //drake/solvers/mathematical_program.h:739
// libstdc++ 4.9 evaluates
// `std::is_convertible<std::unique_ptr<Unrelated>,
// std::shared_ptr<Constraint>>::value`
// incorrectly as `true` so our enable_if overload is not used.
// Provide an explicit alternative for this case.
template <typename A, typename B>
struct is_convertible_workaround : std::is_convertible<A, B> {};
template <typename A, typename B>
struct is_convertible_workaround<std::unique_ptr<A>, std::shared_ptr<B>>
    : std::is_convertible<A*, B*> {};

/**
 * Enable us to catch and provide a meaningful assertion if a Constraint is
 * passed in, when we should have a Cost.
 * @tparam F The class to test if it is convertible to variants of C.
 * @tparam C Intended to be either Cost or Constraint.
 */
template <typename F, typename C>
struct is_binding_compatible
    : std::integral_constant<
          bool, (is_convertible_workaround<F, C>::value) ||
                    (is_convertible_workaround<F, std::shared_ptr<C>>::value) ||
                    (is_convertible_workaround<F, std::unique_ptr<C>>::value) ||
                    (is_convertible_workaround<F, Binding<C>>::value)> {};

/**
 * Template condition to check if @p F is a candidate to be used to construct a
 * FunctionCost object for generic costs.
 * @tparam T The type to be tested.
 * @note Constraint is used to ensure that we do not preclude cost objects
 * that lost their CostShim type somewhere in the process.
 */
template <typename F>
struct is_cost_functor_candidate
    : std::integral_constant<
          bool,
              (!is_binding_compatible<F, Cost>::value) &&
              (!is_convertible_workaround<F, symbolic::Expression>::value)> {};

/**
 * Template condition to only catch when Constraints are inadvertently passed
 * as an argument. If the class is binding-compatible with a Constraint, then
 * this will provide a static assertion to enable easier debugging of which
 * type failed.
 * @tparam F The type to be tested.
 * @see http://stackoverflow.com/a/13366183/7829525
 */
template <typename F>
struct assert_if_is_constraint {
  static constexpr bool value = is_binding_compatible<F, Constraint>::value;
  // Use deferred evaluation
  static_assert(!value, "You cannot pass a Constraint to "
      "create a FunctionCost object. Please ensure you are passing a Cost.");
};

};  // namespace detail

// TODO(eric.cousineau): For is_cost_functor_candidate, consider
// changing implementation to simply check if F is callable (after removing
// pointers, decaying, etc.)
// @ref http://stackoverflow.com/a/5117641/7829525

/**
 * Creates a cost that may be specified using a callable object.
 * @tparam F The function / functor's type.
 * @see FunctionCost, detail::FunctionTraits
 */
template <typename F>
typename std::enable_if<detail::is_cost_functor_candidate<F>::value,
                        std::shared_ptr<Cost>>::type
CreateCost(F&& f) {
  return CreateFunctionCost(std::forward<F>(f));
}

/*@}*/  // \addtogroup FunctionCostCreators

}  // namespace solvers
}  // namespace drake
