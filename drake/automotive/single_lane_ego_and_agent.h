#pragma once

#include <memory>

#include "drake/automotive/idm_planner.h"
#include "drake/automotive/linear_car.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace automotive {

/// Idm (Intelligent Driver Model) is a System that takes in the positions and
/// velocities of an ego car and agent car, and produces an output with the
/// computed IDM acceleration (see IdmPlanner).
///
/// Inputs:
///   0: A BasicVector consisting of
///       - @p x_ego ego car position (scalar) [m]
///       - @p v_ego ego car velocity (scalar) [m/s]
///      (InputPortDescriptor getter: get_ego_port())
///   1: A BasicVector consisting of
///       - @p x_agent agent car position (scalar) [m]
///       - @p v_agent agent car velocity (scalar) [m/s]
/// Output:
///   0: @p vdot_ego longitudinal acceleration request to the ego car (scalar)
///      [m/s^2].
template <typename T>
class Idm : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Idm)

  Idm();
  ~Idm() override;

  /// Getters for the ego and agent car ports.
  const systems::InputPortDescriptor<T>& get_ego_port() const;
  const systems::InputPortDescriptor<T>& get_agent_port() const;

  /// System<T> overrides.
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  std::unique_ptr<systems::Parameters<T>> AllocateParameters() const override;

  void SetDefaultParameters(const systems::LeafContext<T>& context,
                            systems::Parameters<T>* params) const override;
};

/// SingleLaneEgoAndAgent is a System consisting of two cars: an ego and an
/// agent, where the ego is governed by an IDM (intelligent driver model)
/// planner, and where the agent is fed a constant acceleration input.
///
/// <pre>
///   +--------------+         +-------------+
///   | Acceleration | v_dot_a |  Agent Car  |  x_a, v_a
///   |    Input     |-------->|             |----+
///   |  (Constant)  |         | (LinearCar) |    |
///   +--------------+         +-------------+    |
///                                               |
///       +---------------------------------------+
///       | port
///       |  1   +-------------+         +-------------+
///       +----->|   Planner   | v_dot_e |   Ego Car   |  x_e, v_e
///              |             |-------->|             |----+
///       +----->|    (Idm)    |         | (LinearCar) |    |
///       | port +-------------+         +-------------+    |
///       |  0                                              |
///       +-------------------------------------------------+
/// </pre>
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
template <typename T>
class SingleLaneEgoAndAgent : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SingleLaneEgoAndAgent)

  /// Constructs a two-car system.
  ///
  /// @p v_ref desired velocity of the ego (controlled) car.
  /// @p a_agent constant acceleration of the agent car.
  SingleLaneEgoAndAgent(const T& x_ego_init, const T& v_ego_init,
                        const T& x_agent_init, const T& v_agent_init,
                        const T& v_ref, const T& a_agent);

  ~SingleLaneEgoAndAgent() override = default;

  /// Getters for the ego and agent car systems.
  const LinearCar<T>* get_ego_car_system() const { return ego_car_; }
  const LinearCar<T>* get_agent_car_system() const { return agent_car_; }
  const Idm<T>* get_planner_system() const { return planner_; }

 private:
  const LinearCar<T>* ego_car_ = nullptr;
  const LinearCar<T>* agent_car_ = nullptr;
  const Idm<T>* planner_ = nullptr;
};

}  // namespace automotive
}  // namespace drake
