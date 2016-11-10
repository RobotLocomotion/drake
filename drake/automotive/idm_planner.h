#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// IdmPlanner -- an IDM (Intelligent Driver Modal) planner.
///
/// IDM: Intelligent Driver Model:
///    https://en.wikipedia.org/wiki/Intelligent_driver_model
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::TaylorVarXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in libdrakeAutomotive.
///
/// @ingroup automotive_systems
///
/// Inputs: ego car position (scalar) [m], ego car velocity (scalar)
/// [m/s], agent car position (scalar) [m], agent car velocity
/// (scalar) [m/s].
/// Output: linear acceleration of the ego car (scalar) [m/s^2].
template <typename T>
class IdmPlanner : public systems::LeafSystem<T> {
 public:
  /// @param v_ref desired velocity of the ego car in units of m/s.
  explicit IdmPlanner(const T& v_ref);
  ~IdmPlanner() override;

  /// The output of this system is an algbraic relation of its inputs.
  bool has_any_direct_feedthrough() const override { return true; }

  // System<T> override.
  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

 private:
  const T v_ref_;  // Desired vehicle velocity.

  // Disable copy and assignment.
  IdmPlanner(const IdmPlanner<T>&) = delete;
  IdmPlanner& operator=(const IdmPlanner<T>&) = delete;
  IdmPlanner(IdmPlanner<T>&&) = delete;
  IdmPlanner& operator=(IdmPlanner<T>&&) = delete;
};

}  // namespace automotive
}  // namespace drake
