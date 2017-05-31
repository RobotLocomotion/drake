#pragma once

#include "drake/systems/sensors/depth_sensor.h"

namespace drake {
namespace systems {
namespace sensors {

// TODO(nikos.arechiga) Will need to inherit from parametrized DepthSensor, once
// that exists
/// TODO(nikos.arechiga) Add documentation
template <typename T>
class DropoutDepthSensor : public DepthSensor {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DropoutDepthSensor)

  /// TODO(nikos.arechiga) Add documentation
  DropoutDepthSensor<T>(const std::string& name,
                        const RigidBodyTree<double>& tree,
                        const RigidBodyFrame<double>& frame,
                        const DepthSensorSpecification& specification,
                        const double dropout_period,
                        const T dropout_duty_cycle);

  DropoutDepthSensor<T>(const std::string& name,
                        const RigidBodyTree<double>& tree,
                        const RigidBodyFrame<double>& frame,
                        const DepthSensorSpecification& specification,
                        const T dropout_duty_cycle);

 protected:
  /// Outputs the depth information, subject to dropout
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  VectorX<double> get_dropped_frame() const;
  void UpdateOutputsDroppedFrame(const VectorX<double>& distances,
                                 const systems::Context<double>& context,
                                 systems::SystemOutput<double>* output) const;
  bool is_time_to_drop_frame(const Context<T>& context) const;
  T dropout_count_increment;

 private:
  void PrecomputeDroppedFrame();
  void DoCalcDiscreteVariableUpdates(const Context<T>& context,
                                     DiscreteValues<T>* updates) const override;
  VectorX<double> kDroppedFrame;
};
}  // namespace sensors
}  // namespace systems
}  // namespace drake
