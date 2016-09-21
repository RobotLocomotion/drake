#pragma once

#include <memory>

#include "drake/examples/BouncingBall/ball-inl.h"

namespace drake {
namespace bouncingball {

/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in drakeSystemFramework.
///
/// To use other specific scalar types see bouncing_ball-inl.h.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class BouncingBall : public Ball<T> {
 public:
  /// Constructs a BouncingBall system.
  BouncingBall();

  T EvalGuard(const systems::Context<T>& context) const override;

  void PerformReset(systems::Context<T>* context) const override;

 private:
  //const T r = 1;  // radius of ball
  const T cor = 0.8;  // coefficient of restitution 
};

}  // namespace bouncingball
}  // namespace drake
