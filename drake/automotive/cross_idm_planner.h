#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// IdmPlanner -- an IDM (Intelligent Driver Model) planner.
///
/// IDM: Intelligent Driver Model:
///    https://en.wikipedia.org/wiki/Intelligent_driver_model
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::TaylorVarXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
///
/// Inputs:
///   0: @p x_ego ego car position (scalar) [m]
///   1: @p v_ego ego car velocity (scalar) [m/s]
///   2: @p x_agent agent car position (scalar) [m]
///   3: @p v_agent agent car velocity (scalar) [m/s]
/// Outputs:
///   0: @p vdot_ego linear acceleration of the ego car (scalar) [m/s^2].
template <typename T>
class CrossIdmPlanner : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CrossIdmPlanner)

  /// @p v_ref desired velocity of the ego car in units of m/s.
  explicit CrossIdmPlanner(const T& v_ref);
  ~CrossIdmPlanner() override;

  /// Returns the port to the ego car input subvector.
  const systems::InputPortDescriptor<T>& get_ego_port() const;

  /// Returns the port to the agent car input subvector.
  const systems::InputPortDescriptor<T>& get_agent_port() const;

  // System<T> overrides.
  std::unique_ptr<systems::Parameters<T>> AllocateParameters() const override;

  void SetDefaultParameters(const systems::LeafContext<T>& context,
                            systems::Parameters<T>* params) const override;

 protected:
  // The output of this system is an algebraic relation of its inputs.
  bool DoHasDirectFeedthrough(const systems::SparsityMatrix* sparsity,
                              int input_port, int output_port) const override {
    return true;
  }

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  const T v_ref_;  // Desired vehicle velocity.
};

}  // namespace automotive
}  // namespace drake
