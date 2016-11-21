/// @file Provides necessary operations on double to have it as a ScalarType in
/// drake.

#pragma once

namespace drake {
/// Provides if-then-else expression for double.
double if_then_else(bool f_cond, double v_then, double v_else);
}  // namespace drake
