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
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
/// @ingroup primitive_systems
template <typename T>
class SingleLaneEgoAndAgent : public systems::Diagram<T> {
 public:
  /// Constructs a two-car system.
  ///
  /// @param v_0 desired velocity of the ego car.
  /// @param a_agent constant acceleration of the agent car.
  SingleLaneEgoAndAgent(const T& v_0, const T& a_agent);

  ~SingleLaneEgoAndAgent() override {}

  bool has_any_direct_feedthrough() const override;

  /// Sets @p context to a default state.
  void SetDefaultState(systems::Context<T>* context) const;

  /// Getters for the ego and agent car systems.
  const LinearCar<T>* get_ego_car_system() { return ego_car_; }
  const LinearCar<T>* get_agent_car_system() { return agent_car_; }

 private:
  LinearCar<T>* ego_car_ = nullptr;
  LinearCar<T>* agent_car_ = nullptr;
  IdmPlanner<T>* planner_ = nullptr;
  systems::ConstantVectorSource<T>* value_ = nullptr;
};

}  // namespace automotive
}  // namespace drake
