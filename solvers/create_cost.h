#pragma once

#include <memory>
#include <type_traits>
#include <utility>

#include "drake/common/symbolic.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/cost.h"

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

}  // namespace solvers
}  // namespace drake
