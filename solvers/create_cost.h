#pragma once

#include <memory>
#include <optional>
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
Binding<QuadraticCost> ParseQuadraticCost(
    const symbolic::Expression& e,
    std::optional<bool> is_convex = std::nullopt);

/*
 * Assist MathematicalProgram::AddPolynomialCost(...).
 */
Binding<PolynomialCost> ParsePolynomialCost(const symbolic::Expression& e);

/*
 * Assist MathematicalProgram::AddCost(...).
 */
Binding<Cost> ParseCost(const symbolic::Expression& e);

// TODO(eric.cousineau): Remove this when functor cost is no longer exposed
// externally, and must be explicitly called.

/**
 * Enables us to catch and provide a meaningful assertion if a Constraint is
 * passed in, when we should have a Cost.
 * @tparam F The class to test if it is convertible to variants of C.
 * @tparam C Intended to be either Cost or Constraint.
 */
template <typename F, typename C>
struct is_binding_compatible
    : std::bool_constant<
          (std::is_convertible_v<F, C>) ||
              (std::is_convertible_v<F, std::shared_ptr<C>>) ||
              (std::is_convertible_v<F, std::unique_ptr<C>>) ||
              (std::is_convertible_v<F, Binding<C>>)> {};

/**
 * Template condition to check if @p F is a candidate to be used to construct a
 * FunctionCost object for generic costs.
 * @tparam T The type to be tested.
 * @note Constraint is used to ensure that we do not preclude cost objects
 * that lost their CostShim type somewhere in the process.
 */
template <typename F>
struct is_cost_functor_candidate
    : std::bool_constant<(!is_binding_compatible<F, Cost>::value) &&
                             (!std::is_convertible_v<
                                 F, symbolic::Expression>)> {};

}  // namespace internal
}  // namespace solvers
}  // namespace drake
