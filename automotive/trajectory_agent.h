#pragma once

#include <Eigen/Geometry>

#include "drake/automotive/agent_trajectory.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/extract_double.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// Container for static data for a given agent model.
struct AgentData {
  enum class AgentType { kCar, kBicycle, kPedestrian };
  // TODO(jadecastro) Add more types as necessary.

  explicit AgentData(const AgentType& t) : type(t) {}

  AgentType type{AgentType::kCar};
};

// Provide an explicit specialization for the kCar agent type.
struct CarAgent : AgentData {
  CarAgent() : AgentData(AgentType::kCar) {}
};

/// TrajectoryAgent models an agent that follows a pre-established trajectory,
/// taking as input an AgentData structure and an AgentTrajectory.
///
/// Note that, when T = AutoDiffXd, the AutoDiffXd derivatives for each element
/// of the the outputs are empty.
///
/// output port 0: A SimpleCarState containing:
/// * position: x, y, heading;
///   heading is 0 rad when pointed +x, pi/2 rad when pointed +y;
///   heading is defined around the +z axis, positive-left-turn.
/// * speed: s = √{ẋ² + ẏ²}
///   (OutputPort getter: raw_pose_output())
///
/// output port 1: A PoseVector containing X_WA, where A is the agent's
/// reference frame.
///   (OutputPort getter: pose_output())
///
/// output port 2: A FrameVelocity containing the spatial velocity V_WA, where A
/// is the agent's reference frame.
///   (OutputPort getter: velocity_output())
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_plants
template <typename T>
class TrajectoryAgent final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryAgent)

  /// Constructs a TrajectoryAgent system that traces a given AgentTrajectory.
  ///
  /// @param agent_data an AgentData structure defining the agent.
  /// @param trajectory an AgentTrajectory containing the trajectory.
  /// @param sampling_time_sec the requested sampling time (in sec) for this
  /// system.  @default 0.01.
  TrajectoryAgent(const AgentData& agent_data,
                  const AgentTrajectory& trajectory,
                  double sampling_time_sec = 0.01);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit TrajectoryAgent(const TrajectoryAgent<U>& other)
      : TrajectoryAgent<T>(other.agent_data_, other.trajectory_) {}

  /// @name Accessors for the outputs, as enumerated in the class documentation.
  /// @{
  const systems::OutputPort<T>& raw_pose_output() const {
    return this->get_output_port(0);
  }
  const systems::OutputPort<T>& pose_output() const {
    return this->get_output_port(1);
  }
  const systems::OutputPort<T>& velocity_output() const {
    return this->get_output_port(2);
  }
  /// @}

 private:
  // Converts a PoseVelocity, evaluated at the current time, into a
  // SimpleCarState output.
  void CalcStateOutput(const systems::Context<T>& context,
                       SimpleCarState<T>* output_vector) const;

  // Converts a PoseVelocity, evaluated at the current time, into a PoseVector
  // output.
  void CalcPoseOutput(const systems::Context<T>& context,
                      systems::rendering::PoseVector<T>* pose) const;

  // Converts a PoseVelocity, evaluated at the current time, into a
  // FrameVelocity output.
  void CalcVelocityOutput(const systems::Context<T>& context,
                          systems::rendering::FrameVelocity<T>* velocity) const;

  // Extracts the PoseVelocity value at the current time, as provided in
  // Context.
  PoseVelocity GetValues(const systems::Context<T>& context) const;

  // Allow different specializations to access each other's private data.
  template <typename>
  friend class TrajectoryAgent;

  const AgentData agent_data_;
  const AgentTrajectory trajectory_;
};

}  // namespace automotive
}  // namespace drake
