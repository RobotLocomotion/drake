#pragma once

#include <functional>
#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/system_common.h"

namespace drake {
namespace systems {

template <typename T>
class System;

template <typename T>
class Context;

/// A SystemConstraint is a generic base-class for constraints on Systems.
///
/// This class is intentionally compatible with, but (so far) independent from
/// solvers::Constraint. This is primarily because there is no notion of
/// decision variables in the system classes (yet); rather each individual
/// algorithm (e.g. trajectory optimization, or system identification)
/// constructs decision variables for the particular mathematical program that
/// is being formulated, and must bind the system constraint to those variables
/// (e.g. by populating the Context with the decision variables and calling
/// Calc).
///
/// @see LeafSystem<T>::DeclareConstraint for more details.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class SystemConstraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemConstraint)

  // TODO(russt): replace the argument VectorX<T>* with an Eigen::Ref* using
  // whatever magic jwnimmer and soonho figured out a few weeks back.
  using CalcCallback = std::function<void(const Context<T>&, VectorX<T>*)>;

  /// Constructs the SystemConstraint.
  SystemConstraint(CalcCallback calc_function,
                   const Eigen::Ref<const Eigen::VectorXd>& lower_bound,
                   const Eigen::Ref<const Eigen::VectorXd>& upper_bound,
                   const std::string& description = "")
      : calc_function_(calc_function),
        lower_bound_(lower_bound),
        upper_bound_(upper_bound),
        description_(description) {
    DRAKE_DEMAND(lower_bound.rows() == upper_bound.rows());
  }
  virtual ~SystemConstraint() = default;

  /// Evaluate the function pointer, ensuring that the value vector
  /// is properly sized.
  void Calc(const Context<T>& context, VectorX<T>* value) const {
    value->resize(lower_bound_.rows());
    calc_function_(context, value);
    DRAKE_DEMAND(value->size() == lower_bound_.rows());
  }

  // Accessor methods.
  int size() const { return lower_bound_.rows(); }
  Eigen::VectorXd const& lower_bound() const { return lower_bound_; }
  Eigen::VectorXd const& upper_bound() const { return upper_bound_; }
  const std::string& description() const { return description_; }

 private:
  const CalcCallback calc_function_;
  const Eigen::VectorXd lower_bound_;
  const Eigen::VectorXd upper_bound_;
  const std::string description_;
};

}  // namespace systems
}  // namespace drake
