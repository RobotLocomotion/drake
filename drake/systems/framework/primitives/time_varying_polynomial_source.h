# pragma once


//#include <cstdint>
//#include <memory>
//
//#include "drake/systems/framework/context_base.h"
//#include "drake/systems/framework/leaf_system.h"
//#include "drake/systems/framework/system_output.h"
//#include "drake/systems/trajectories/PiecewisePolynomial.h"
//
//namespace drake {
//namespace systems {
//
///// A source block with a which generates the value of  at all times.
///// @tparam T The vector element type, which must be a valid Eigen scalar.
/////
///// Instantiated templates for the following kinds of T's are provided:
///// - double
/////
///// They are already available to link against in libdrakeSystemFramework.
///// No other values for T are currently supported.
//template <typename T>
//class TimeVaryingPolynomialSource : public LeafSystem<T> {
// public:
//  /// Constructs a system with a vector output that is constant and equals the
//  /// supplied @p source_value at all times.
//  /// @param source_value the constant value of the output so that
//  /// `y = source_value` at all times.
//  explicit TimeVaryingPolynomialSource(const PiecewisePolynomialType& pp_traj);
//
//  /// Outputs a signal with a fixed value as specified by the user.
//  void EvalOutput(const ContextBase<T>& context,
//                  SystemOutput<T>* output) const override;
//
// private:
//   const PiecewisePolynomialType& pp_traj_;
//};
//
//}  // namespace systems
//}  // namespace drake
