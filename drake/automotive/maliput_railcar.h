#pragma once

#include <memory>

#include "drake/automotive/gen/maliput_railcar_config.h"
#include "drake/automotive/gen/maliput_railcar_state.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/sparsity_matrix.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// LaneDirection holds the lane that a MaliputRailcar is traversing and the
/// direction in which it is moving. A MaliputRailcar can either travel in the
/// increasing-`s` direction or in the decreasing-`s` direction.
struct LaneDirection {
  /// Default constructor.
  LaneDirection() {}

  /// A constructor that sets `with_s` to be `true`.
  explicit LaneDirection(const maliput::api::Lane* lane_input)
      : LaneDirection(lane_input, true) {}

  /// Fully parameterized constructor.
  LaneDirection(const maliput::api::Lane* lane_input, bool with_s_input)
      : lane(lane_input), with_s(with_s_input) {}

  const maliput::api::Lane* lane{nullptr};

  /// True means that the MaliputRailcar's `s` coordinate increases when the
  /// vehicle has positive speed. False means the opposite.
  bool with_s{true};
};

/// MaliputRailcar models a vehicle that follows a maliput::api::Lane as if it
/// were on rails and neglecting all physics.
///
/// Configuration:
///   * See MaliputRailcarConfig.
///
/// State vector:
///   * See MaliputRailcarState.
///
/// Abstract state:
///   * See LaneDirection.
///
/// <B>Input Port Accessors:</B>
///
///   - command_input(): Contains the desired acceleration. This port
///     contains a systems::BasicVector of size 1. It is optional in that it
///     need not be connected. When it is unconnected, the railcar will travel
///     at its initial velocity, which is specified in MaliputRailcarConfig.
///
/// <B>Output Port Accessors:</B>
///
///   - state_output(): Contains this system's state vector. See
///     MaliputRailcarState.
///
///   - lane_state_output(): Contains this system's lane direction state. See
///     LaneDirection.
///
///   - pose_output(): Contains PoseVector `X_WC`, where `C` is the car frame
///     and `W` is the world frame.
///
/// @tparam T must support certain arithmetic operations;
/// for details, see drake::symbolic::Expression.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
template <typename T>
class MaliputRailcar : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MaliputRailcar)

  /// The constructor.
  ///
  /// @param initial_lane_direction The initial lane and direction of travel.
  ///
  explicit MaliputRailcar(const LaneDirection& initial_lane_direction);

  // System<T> overrides.
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // LeafSystem<T> overrides.
  void SetDefaultParameters(const systems::LeafContext<T>& context,
                            systems::Parameters<T>* params) const override;

  /// Sets `config` to contain the default parameters for MaliputRailcar.
  static void SetDefaultParameters(MaliputRailcarConfig<T>* config);

  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;

  /// Sets `railcar_state` to contain the default state for MaliputRailcar.
  static void SetDefaultState(MaliputRailcarState<T>* railcar_state);

  /// Getter methods for input and output port descriptors.
  /// @{
  const systems::InputPortDescriptor<T>& command_input() const;
  const systems::OutputPortDescriptor<T>& state_output() const;
  const systems::OutputPortDescriptor<T>& lane_state_output() const;
  const systems::OutputPortDescriptor<T>& pose_output() const;
  /// @}

  static constexpr T kDefaultR = T(0);
  static constexpr T kDefaultH = T(0);
  static constexpr T kDefaultInitialS = T(0);
  static constexpr T kDefaultInitialSpeed = T(1);
  static constexpr T kDefaultMaxSpeed = T(45);
  static constexpr T kDefaultVelocityLimitKp = T(10);

 protected:
  // LeafSystem<T> overrides.
  std::unique_ptr<systems::AbstractValues> AllocateAbstractState()
      const override;
  std::unique_ptr<systems::Parameters<T>> AllocateParameters() const override;
  bool DoHasDirectFeedthrough(const systems::SparsityMatrix* sparsity,
                              int input_port, int output_port) const override;
  void DoCalcNextUpdateTime(const systems::Context<T>& context,
                            systems::UpdateActions<T>* actions) const override;
  void DoCalcUnrestrictedUpdate(const systems::Context<T>& context,
                                systems::State<T>* state) const override;

 private:
  void ImplCalcOutput(
      const MaliputRailcarState<T>& state,
      MaliputRailcarState<T>* output) const;

  void ImplCalcLaneOutput(
      const LaneDirection& lane_direction,
      LaneDirection* output) const;

  void ImplCalcPose(
      const MaliputRailcarConfig<T>& config,
      const MaliputRailcarState<T>& state,
      const LaneDirection& lane_direction,
      systems::rendering::PoseVector<T>* pose) const;

  void ImplCalcTimeDerivatives(
      const MaliputRailcarConfig<T>& config,
      const MaliputRailcarState<T>& state,
      const LaneDirection& lane_direction,
      const systems::BasicVector<T>& input,
      MaliputRailcarState<T>* rates) const;

  void ImplCalcTimeDerivativesDouble(
    const MaliputRailcarConfig<double>& config,
    const MaliputRailcarState<double>& state,
    MaliputRailcarState<double>* rates) const;

  const LaneDirection initial_lane_direction_{};
  int command_input_port_index_{};
  int state_output_port_index_{};
  int lane_state_output_port_index_{};
  int pose_output_port_index_{};
};

}  // namespace automotive
}  // namespace drake
