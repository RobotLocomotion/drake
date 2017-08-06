#pragma once

#include <memory>
#include <type_traits>
#include <utility>

#include "drake/common/symbolic.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/function.h"

namespace drake {
namespace solvers {
namespace internal {

/*
 * Assist MathematicalProgram::AddLinearCost(...).
 */
Binding<LinearCost> ParseLinearCost(const symbolic::Expression& e);

/*
 * Assist MathematicalProgram::AddQuadraticCost(...).
 */
Binding<QuadraticCost> ParseQuadraticCost(const symbolic::Expression& e);

/*
 * Assist MathematicalProgram::AddPolynomialCost(...).
 */
Binding<PolynomialCost> ParsePolynomialCost(const symbolic::Expression& e);

/*
 * Assist MathematicalProgram::AddCost(...).
 */
Binding<Cost> ParseCost(const symbolic::Expression& e);

}  // namespace internal

// TODO(eric.cousineau): Remove this when functor cost is no longer exposed
// externally, and must be explicitly called.
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
 * Enables us to catch and provide a meaningful assertion if a Constraint is
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
    : std::integral_constant<bool, (!is_binding_compatible<F, Cost>::value) &&
                                       (!is_convertible_workaround<
                                           F, symbolic::Expression>::value)> {};

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
  static_assert(
      !value,
      "You cannot pass a Constraint to "
      "create a FunctionCost object. Please ensure you are passing a Cost.");
};

}  // namespace detail

}  // namespace solvers
}  // namespace drake
