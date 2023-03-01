#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_EXPRESSION_ALL
#error Do not include this file. Use "drake/common/symbolic/expression.h".
#endif

#include <cstdint>

namespace drake {
namespace symbolic {

/** Kinds of symbolic expressions.
@internal The constants here are carefully chosen to support nanboxing. For all
elements except Constant, the bit pattern must have have 0x7FF0 bits set but
must not be exactly 0x7FF0 nor 0xFFF0 (reserved for ±infinity). Refer to the
details in boxed_cell.h for more information. */
enum class ExpressionKind : std::uint16_t {
  // clang-format off
  Constant = 0,           ///< constant (double)
  Var = 0x7FF1u,          ///< variable
  Add,                    ///< addition (+)
  Mul,                    ///< multiplication (*)
  Div,                    ///< division (/)
  Log,                    ///< logarithms
  Abs,                    ///< absolute value function
  Exp,                    ///< exponentiation
  Sqrt,                   ///< square root
  Pow,                    ///< power function
  Sin,                    ///< sine
  Cos,                    ///< cosine
  Tan,                    ///< tangent
  Asin,                   ///< arcsine
  Acos,                   ///< arccosine
  Atan,                   ///< arctangent
  // Here we have Atan = 0x7FFFu, but we can't overflow to 0x8000 for Atan2 so
  // restart numbering at the next available value (0xFFF1).
  Atan2 = 0xFFF1u,        ///< arctangent2 (atan2(y,x) = atan(y/x))
  Sinh,                   ///< hyperbolic sine
  Cosh,                   ///< hyperbolic cosine
  Tanh,                   ///< hyperbolic tangent
  Min,                    ///< min
  Max,                    ///< max
  Ceil,                   ///< ceil
  Floor,                  ///< floor
  IfThenElse,             ///< if then else
  NaN,                    ///< NaN
  UninterpretedFunction,  ///< Uninterpreted function
  // TODO(soonho): add Integral
  // clang-format on
};

/** Total ordering between ExpressionKinds. */
inline bool operator<(ExpressionKind k1, ExpressionKind k2) {
  return static_cast<std::uint16_t>(k1) < static_cast<std::uint16_t>(k2);
}

}  // namespace symbolic
}  // namespace drake
