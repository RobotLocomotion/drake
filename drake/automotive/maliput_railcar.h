#pragma once

#include <memory>

#include "drake/automotive/gen/maliput_railcar_params.h"
#include "drake/automotive/gen/maliput_railcar_state.h"
#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/sparsity_matrix.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// MaliputRailcar models a vehicle that follows a maliput::api::Lane as if it
/// were on rails and neglecting all physics.
///
/// Parameters:
///   * See MaliputRailcarParams.
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
///     at its initial velocity, which is specified in MaliputRailcarParams.
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
  /// Defines a distance that is "close enough" to the end of a lane for the
  /// vehicle to transition to an ongoing branch. The primary constraint on the
  /// selection of this variable is the application's degree of sensitivity to
  /// position state discontinuity when the MaliputRailcar "jumps" from its
  /// current lane to a lane in an ongoing branch. A smaller value results in a
  /// smaller spatial discontinuity. If this value is zero, the spatial
  /// discontinuity will be zero. However, it will trigger the use of
  /// kTimeEpsilon, which results in a temporal discontinuity.
  static constexpr double kLaneEndEpsilon{1e-12};

  /// Defines a time interval that is used to ensure a desired update time is
  /// always greater than (i.e., after) the current time. Despite the spatial
  /// window provided by kLaneEndEpsilon, it is still possible for the vehicle
  /// to end up precisely at the end of its current lane (e.g., it could be
  /// initialized in this state). In this scenario, the next update time will be
  /// equal to the current time. The integrator, however, requires that the next
  /// update time be strictly after the current time, which is when this
  /// constant is used. The primary constraint on the selection of this constant
  /// is the application's sensitivity to a MaliputRailcar being "late" in its
  /// transition to an ongoing branch once it is at the end of its current lane.
  /// The smaller this value, the less "late" the transition will occur. This
  /// value cannot be zero since that will violate the integrator's need for the
  /// next update time to be strictly after the current time.
  static constexpr double kTimeEpsilon{1e-12};

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

  static constexpr T kDefaultInitialS = T(0);
  static constexpr T kDefaultInitialSpeed = T(1);

 protected:
  // LeafSystem<T> overrides.
  std::unique_ptr<systems::AbstractValues> AllocateAbstractState()
      const override;
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
      const MaliputRailcarParams<T>& params,
      const MaliputRailcarState<T>& state,
      const LaneDirection& lane_direction,
      systems::rendering::PoseVector<T>* pose) const;

  void ImplCalcTimeDerivatives(
      const MaliputRailcarParams<T>& params,
      const MaliputRailcarState<T>& state,
      const LaneDirection& lane_direction,
      const systems::BasicVector<T>& input,
      MaliputRailcarState<T>* rates) const;

  void ImplCalcTimeDerivativesDouble(
      const MaliputRailcarParams<double>& params,
      const MaliputRailcarState<double>& state,
      MaliputRailcarState<double>* rates) const;

  // Calculates the vehicle's `r` coordinate based on whether it's traveling
  // with or against `s` in the current lane relative to the initial lane.
  T CalcR(const MaliputRailcarParams<T>& params,
          const LaneDirection& lane_direction) const;

  const LaneDirection initial_lane_direction_{};
  int command_input_port_index_{};
  int state_output_port_index_{};
  int lane_state_output_port_index_{};
  int pose_output_port_index_{};
};

}  // namespace automotive
}  // namespace drake
