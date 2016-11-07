#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// IdmPlanner -- an IDM (Intelligent Driver Modal)[1].
///
/// [1] IDM: Intelligent Driver Model:
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
template <typename T>
class IdmPlanner : public systems::LeafSystem<T> {
 public:
  IdmPlanner(const T& v_0);
  ~IdmPlanner() override;

  /// The output of this system is an algbraic relation of its inputs.
  bool has_any_direct_feedthrough() const override { return true; }

  // System<T> overrides
  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

 protected:
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override;

 private:
  const T v_0_;  // Desired vehicle velocity.
};

}  // namespace automotive
}  // namespace drake
