#pragma once

#include <memory>

//#include "drake/systems/framework/primitives/integrator.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
//#include "drake/automotive/gen/single_lane_ego_and_agent_state.h"
#include "drake/automotive/linear_car.h"
#include "drake/automotive/idm_planner.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"

namespace drake {
namespace automotive {

/// System with cars arranged in a diagram:
///
///   +--------------+         +-------------+
///   |   Constant   | v_dot_a |  Agent Car  |  x_a, v_a
///   | Acceleration |-------->|             |----+
///   |    Input     |         | (LinearCar) |    |
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

 private:
  LinearCar<T>* ego_car_ = nullptr;
  LinearCar<T>* agent_car_ = nullptr;
  IdmPlanner<T>* planner_ = nullptr;
  systems::ConstantVectorSource<T>* value_ = nullptr;
};

}  // namespace automotive
}  // namespace drake
