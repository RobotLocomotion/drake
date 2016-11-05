#pragma once

#include <memory>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/automotive/gen/single_lane_ego_and_agent_state.h"
#include "drake/automotive/linear_car.h"
#include "drake/automotive/idm_planner.h"

namespace drake {
namespace automotive {

/// 
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
  SingleLaneEgoAndAgent(const T& v_0);

  ~SingleLaneEgoAndAgent() override {}

 private:
  LinearCar<T>* ego_car_ = nullptr;
  LinearCar<T>* agent_car_ = nullptr;
  IdmPlanner<T>* planner_ = nullptr;
};

}  // namespace automotive
}  // namespace drake
