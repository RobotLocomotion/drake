#pragma once

#include <memory>

#include "drake/automotive/idm_planner.h"
#include "drake/automotive/linear_car.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"

namespace drake {
namespace automotive {

/// System consisting of two cars: an ego and an agent, where the ego
/// is governed by an IDM (intelligent driver model) planner, and
/// where the agent is fed a constant acceleration input.
///
///   +--------------+         +-------------+
///   | Acceleration | v_dot_a |  Agent Car  |  x_a, v_a
///   |    Input     |-------->|             |----+
///   |  (Constant)  |         | (LinearCar) |    |
///   +--------------+         +-------------+    |
///                                               |
///       +---------------------------------------+
///       | port
///       |  1   +--------------+         +-------------+
///       +----->|   Planner    | v_dot_e |   Ego Car   |  x_e, v_e
///              |              |-------->|             |----+
///       +----->| (IdmPlanner) |         | (LinearCar) |    |
///       | port +--------------+         +-------------+    |
///       |  0                                               |
///       +--------------------------------------------------+
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in libdrakeAutomotive.
///
/// @ingroup automotive_systems
template <typename T>
class SingleLaneEgoAndAgent : public systems::Diagram<T> {
 public:
  /// Constructs a two-car system.
  ///
  /// @p v_ref desired velocity of the ego (controlled) car.
  /// @p a_agent constant acceleration of the agent car.
  SingleLaneEgoAndAgent(const T& x_ego_init, const T& v_ego_init,
                        const T& x_agent_init, const T& v_agent_init,
                        const T& v_ref, const T& a_agent);

  ~SingleLaneEgoAndAgent() override {}

  bool has_any_direct_feedthrough() const override;

  /// Sets the continuous states in @p context to default values.
  void SetDefaultState(systems::Context<T>* context) const;

  /// Getters for the ego and agent car systems.
  const LinearCar<T>* get_ego_car_system() const { return ego_car_; }
  const LinearCar<T>* get_agent_car_system() const { return agent_car_; }
  const IdmPlanner<T>* get_planner_system() const { return planner_; }

  // Disable copy and assignment.
  SingleLaneEgoAndAgent(const SingleLaneEgoAndAgent<T>&) = delete;
  SingleLaneEgoAndAgent& operator=(const SingleLaneEgoAndAgent<T>&) = delete;
  SingleLaneEgoAndAgent(SingleLaneEgoAndAgent<T>&&) = delete;
  SingleLaneEgoAndAgent& operator=(SingleLaneEgoAndAgent<T>&&) = delete;

 private:
  const LinearCar<T>* ego_car_ = nullptr;
  const LinearCar<T>* agent_car_ = nullptr;
  const IdmPlanner<T>* planner_ = nullptr;
};

}  // namespace automotive
}  // namespace drake
