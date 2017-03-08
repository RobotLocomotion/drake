#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "drake/automotive/dev/infinite_circuit_road.h"
#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/endless_road_car_config.h"
#include "drake/automotive/gen/endless_road_car_state.h"
#include "drake/automotive/gen/endless_road_oracle_output.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// A non-physical car that operates in LANE-space on an InfiniteCircuitRoad,
/// i.e., a maliput road network that only has a single Lane and infinite
/// longitudinal extent.  The car is constrained to operating within the
/// driveable-bounds of the road network:  a "Magic Guard Rail" feature will
/// clamp lateral (`r`) derivatives to zero if they would push the car's
/// position past the driveable-bounds.
///
/// (Elevation-above-road 'h' is implicitly zero, too.)
///
/// configuration:  EndlessRoadCarConfig
///
/// state vector:  EndlessRoadCarState
/// * planar LANE-space position:  (s, r)
/// * planar heading and speed (isometric LANE-space forward velocity)
///
/// input vector:  Depends on @p control_type specified at construction;
/// see EndlessRoadCar() constructor for details.
///
/// output vector: EndlessRoadCarState (same as state vector)
///
/// @tparam T must support certain arithmetic operations.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///
/// @ingroup automotive_systems
template <typename T>
class EndlessRoadCar : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EndlessRoadCar)

  enum ControlType {
    kNone,  // no controls, i.e., constant velocity
    kUser,  // processes DrivingCommand input
    kIdm,   // IDM controller using input from EndlessRoadOracle
  };

  /// Constructs an EndlessRoadCar.
  ///
  /// @param id  ID string, helpful for logging and visualization
  /// @param road  the maliput::api::RoadGeometry on which this car will
  ///              be driving
  /// @param control_type  how the car will be controlled
  /// @param config  configurable parameters of the car
  ///
  /// @p control_type affects the number and type of input ports:
  /// * `kNone`:  no input ports (accelerations are just zero)
  /// * `kUser`:  single port accepting DrivingCommand
  /// * `kIdm`:   single port accepting EndlessRoadOracleOutput
  ///
  /// In `kUser` mode, `throttle` and `brake` from DrivingCommand are
  /// normalized against a car's `max_acceleration` and `max_deceleration`,
  /// respectively, from its EndlessRoadCarConfig parameter.
  EndlessRoadCar(const std::string& id,
                 const maliput::utility::InfiniteCircuitRoad* road,
                 const ControlType control_type);

  const std::string& id() const { return id_; }

  ControlType control_type() const { return control_type_; }

 public:
  // System<T> overrides
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // LeafSystem<T> overrides
  void SetDefaultParameters(const systems::LeafContext<T>& context,
                            systems::Parameters<T>* params) const override;

  /// Sets `config` to contain the default parameters for EndlessRoadCar.
  static void SetDefaultParameters(EndlessRoadCarConfig<T>* config);

 protected:
  // LeafSystem<T> overrides
  std::unique_ptr<systems::ContinuousState<T>
                  > AllocateContinuousState() const override;
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::OutputPortDescriptor<T>& descriptor) const override;
  std::unique_ptr<systems::Parameters<T>> AllocateParameters() const override;
  bool DoHasDirectFeedthrough(const systems::SparsityMatrix* sparsity,
                              int input_port, int output_port) const override;

 private:
  struct Accelerations {
    Accelerations(T forward_in, T lateral_in)
        : forward(forward_in), lateral(lateral_in) {}

    T forward{};
    T lateral{};
  };

  void ImplCalcOutput(const EndlessRoadCarState<T>&,
                      EndlessRoadCarState<T>*) const;

  Accelerations ComputeUserAccelerations(
      const EndlessRoadCarState<T>& state,
      const DrivingCommand<T>& input,
      const EndlessRoadCarConfig<T>& config) const;

  Accelerations ComputeIdmAccelerations(
      const EndlessRoadCarState<T>& state,
      const EndlessRoadOracleOutput<T>& input,
      const EndlessRoadCarConfig<T>& config) const;

  void ImplCalcTimeDerivatives(const EndlessRoadCarState<T>&,
                               const Accelerations& accelerations,
                               EndlessRoadCarState<T>*) const;

  const std::string id_;
  const maliput::utility::InfiniteCircuitRoad* const road_;
  const ControlType control_type_;
};

}  // namespace automotive
}  // namespace drake
