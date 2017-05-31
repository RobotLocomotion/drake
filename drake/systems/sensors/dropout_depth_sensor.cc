#include "drake/systems/sensors/dropout_depth_sensor.h"
#include "Eigen/src/Core/Matrix.h"
#include "depth_sensor_output.h"

using Eigen::VectorXd;

namespace drake {
namespace systems {
namespace sensors {

template <typename T>
DropoutDepthSensor<T>::DropoutDepthSensor(
    const std::string& name, const RigidBodyTree<double>& tree,
    const RigidBodyFrame<double>& frame,
    const DepthSensorSpecification& specification, const double sample_time,
    const T dropout_duty_cycle)
    : DepthSensor::DepthSensor(name, tree, frame, specification) {
  DRAKE_THROW_UNLESS((0 <= dropout_duty_cycle) && (dropout_duty_cycle <= 100));
  dropout_count_increment = (dropout_duty_cycle * sample_time) / 100.0;
  PrecomputeDroppedFrame();

  DeclareDiscreteState(1);
  this->DeclareDiscreteUpdatePeriodSec(sample_time);
}

template <typename T>
DropoutDepthSensor<T>::DropoutDepthSensor(
    const std::string& name, const RigidBodyTree<double>& tree,
    const RigidBodyFrame<double>& frame,
    const DepthSensorSpecification& specification, const T dropout_duty_cycle)
    : DepthSensor::DepthSensor(name, tree, frame, specification) {
  DRAKE_THROW_UNLESS((0 <= dropout_duty_cycle) && (dropout_duty_cycle <= 100));
  double sample_time = 1;  // Default value if none is provided
  dropout_count_increment = (dropout_duty_cycle * sample_time) / 100.0;
  PrecomputeDroppedFrame();

  DeclareDiscreteState(1);
  this->DeclareDiscreteUpdatePeriodSec(sample_time);
}

template <typename T>
void DropoutDepthSensor<T>::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  if (is_time_to_drop_frame(context)) {
    VectorX<double> distances = get_dropped_frame();
    UpdateOutputsDroppedFrame(distances, context, output);
  } else {
    DepthSensor::DoCalcOutput(context, output);
  }
}

template <typename T>
void DropoutDepthSensor<T>::DoCalcDiscreteVariableUpdates(
    const Context<T>& context, DiscreteValues<T>* updates) const {
  T accumulated_error = context.get_discrete_state(0)->GetAtIndex(0);
  accumulated_error += dropout_count_increment;
  if (accumulated_error >= 100) {
    accumulated_error = 0.0;
  }

  (*updates)[0] = accumulated_error;
}

template <typename T>
void DropoutDepthSensor<T>::UpdateOutputsDroppedFrame(
    const VectorX<double>& distances, const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  VectorXd u = this->EvalEigenVectorInput(context, 0);
  auto q = u.head(get_tree().get_num_positions());
  KinematicsCache<double> kinematics_cache = get_tree().doKinematics(q);
  DepthSensor::UpdateOutputs(distances, kinematics_cache, output);
}

template <typename T>
bool DropoutDepthSensor<T>::is_time_to_drop_frame(
    const Context<T>& context) const {
  T accumulated_error = context.get_discrete_state(0)->GetAtIndex(0);

  if (accumulated_error <
      dropout_count_increment) {  // This will happen if it was
                                  // set to zero in the update
    return true;
  } else {
    return false;
  }
}

template <typename T>
VectorX<double> DropoutDepthSensor<T>::get_dropped_frame() const {
  return kDroppedFrame;
}

template <typename T>
void DropoutDepthSensor<T>::PrecomputeDroppedFrame() {
  for (int i = 0; i < kDroppedFrame.size(); ++i) {
    kDroppedFrame[i] = DepthSensorOutput<double>::GetErrorDistance();
  }
}

    template class DropoutDepthSensor<double>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
