#pragma once

#include "drake/systems/sensors/depth_sensor.h"

namespace drake {
namespace systems {
namespace sensors {

/// DropoutDepthSensor is the same as DepthSensor, but it periodically produces
/// a
/// distances matrix that has all elements set to kErrorDistance.
///
/// The sensor determines whether it should return correct data or error data as
/// follows. @p update_period is the period, measured in seconds, with which the
/// sensor will update its internal state.  @p dropout_duty_cycle is a
/// percentage, between 0 and 100. The sensor cycles between returning correct
/// and erroneous data over a time period of 100*(@p update_period ).  The
/// sensor will begin by returning correct data for the first ( 100 - @p
/// dropout_duty_cycle ) percentage of the time defined by 100*(@p
/// update_period), and will return erroneous data for the remaining @p
/// dropout_duty_cycle percent of the period.
///
/// For example, if @p update_period = 0.1 and @p dropout_duty_cycle = 20, then:
/// - for the first 8 seconds, sensor will return correct measurements
/// - for the next 2 seconds (20% of the period), sensor will return error
///   measurements
///
/// Notes:
/// - If @p update_period is not supplied, the default is 1.0.
/// - The type of dropout_duty_cycle is the parametric type T, to allow future
///   instantiation with the autodifferentiable type
/// - The distance matrix that consists of kErrorDistance is somemtimes referred
///   to as the "dropped frame"

template <typename T>
class DropoutDepthSensor : public DepthSensor {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DropoutDepthSensor)

  /// Constructors delegate to the DepthSensor constructor, and also register to
  /// update its internal state with a period of @p update_period.
  DropoutDepthSensor<T>(const std::string& name,
                        const RigidBodyTree<double>& tree,
                        const RigidBodyFrame<double>& frame,
                        const DepthSensorSpecification& specification,
                        const double update_period, const T dropout_duty_cycle);

  DropoutDepthSensor<T>(const std::string& name,
                        const RigidBodyTree<double>& tree,
                        const RigidBodyFrame<double>& frame,
                        const DepthSensorSpecification& specification,
                        const T dropout_duty_cycle);

 protected:
  /// Outputs the depth information, subject to dropout
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  /// Returns the distances matrix that consists of all errors
  VectorX<double> get_dropped_frame() const;
  void UpdateOutputsDroppedFrame(const VectorX<double>& distances,
                                 const systems::Context<double>& context,
                                 systems::SystemOutput<double>* output) const;
  bool is_time_to_drop_frame(const Context<T>& context) const;
  T dropout_count_increment_;

 private:
  // To avoid computing the error frame every time we need to return it, the
  // error frame is computed once when the sensor is constructed
  void PrecomputeDroppedFrame();

  void DoCalcDiscreteVariableUpdates(const Context<T>& context,
                                     DiscreteValues<T>* updates) const override;
  VectorX<double> kDroppedFrame;
};
}  // namespace sensors
}  // namespace systems
}  // namespace drake
