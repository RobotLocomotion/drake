#include "drake/automotive/maliput_railcar_to_euler_floating_joint.h"

namespace drake {
namespace automotive {

template <typename T>
MaliputRailcarToEulerFloatingJoint<T>::MaliputRailcarToEulerFloatingJoint(
    const maliput::api::Lane& lane,
    const MaliputRailcarConfig<T>& config)
    : lane_(lane) {
  config_.set_r(config.r());
  config_.set_h(config.h());
  config_.set_initial_speed(config.initial_speed());
  this->set_name("MaliputRailcarToEulerFloatingJoint");
  input_port_index_ = this->DeclareInputPort(systems::kVectorValued,
      MaliputRailcarStateIndices::kNumCoordinates).get_index();
  output_port_index_ = this->DeclareOutputPort(systems::kVectorValued,
      EulerFloatingJointStateIndices::kNumCoordinates).get_index();
}

template <typename T>
void MaliputRailcarToEulerFloatingJoint<T>::DoCalcOutput(
    const systems::Context<T>& context,
    systems::SystemOutput<T>* output) const {
  typedef systems::VectorBase<T> Base;
  const Base* const input_vector = this->EvalVectorInput(context,
                                                         input_port_index_);
  DRAKE_ASSERT(input_vector != nullptr);
  const MaliputRailcarState<T>* const input_data =
      dynamic_cast<const MaliputRailcarState<T>*>(input_vector);
  DRAKE_ASSERT(input_data != nullptr);

  Base* const output_vector = output->GetMutableVectorData(
      output_port_index_);
  DRAKE_ASSERT(output_vector != nullptr);
  EulerFloatingJointState<T>* const output_data =
      dynamic_cast<EulerFloatingJointState<T>*>(output_vector);
  DRAKE_ASSERT(output_data != nullptr);

  const maliput::api::LanePosition lane_position(
      input_data->s(), config_.r(), config_.h());
  const maliput::api::GeoPosition geo_position =
      lane_.ToGeoPosition(lane_position);
  const maliput::api::Rotation rotation =
      lane_.GetOrientation(lane_position);

  output_data->set_x(geo_position.x);
  output_data->set_y(geo_position.y);
  output_data->set_z(geo_position.z);
  output_data->set_roll(rotation.roll);
  output_data->set_pitch(rotation.pitch);
  output_data->set_yaw(rotation.yaw);
}

// These instantiations must match the API documentation in
// maliput_railcar_to_euler_floating_joint.h.
template class MaliputRailcarToEulerFloatingJoint<double>;

}  // namespace automotive
}  // namespace drake
