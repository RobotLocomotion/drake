#include "drake/automotive/pure_pursuit_controller.h"

#include <cmath>

#include "drake/automotive/pose_selector.h"
#include "drake/automotive/pure_pursuit.h"
#include "drake/common/drake_assert.h"

namespace drake {

using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::Junction;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::PoseVector;

namespace automotive {

static constexpr int kPpParamsIndex{0};
static constexpr int kCarParamsIndex{1};

template <typename T>
PurePursuitController<T>::PurePursuitController(const RoadGeometry& road)
    : road_(road) {
  // Declare the input port for the DrivingCommand request.
  command_input_index_ =
      this->DeclareInputPort(systems::kVectorValued,
                             DrivingCommandIndices::kNumCoordinates)
          .get_index();
  // Declare the input port for the desired LaneDirection.
  lane_index_ = this->DeclareAbstractInputPort().get_index();
  // Declare the input port for the ego car PoseVector.
  ego_pose_index_ =
      this->DeclareInputPort(systems::kVectorValued, PoseVector<T>::kSize)
          .get_index();
  // Declare the DrivingCommand output port.
  command_output_index_ =
      this->DeclareVectorOutputPort(DrivingCommand<T>()).get_index();

  this->DeclareNumericParameter(PurePursuitParams<T>());
  this->DeclareNumericParameter(SimpleCarParams<T>());
}

template <typename T>
PurePursuitController<T>::~PurePursuitController() {}

template <typename T>
const systems::InputPortDescriptor<T>&
PurePursuitController<T>::driving_command_input() const {
  return systems::System<T>::get_input_port(command_input_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>& PurePursuitController<T>::lane_input()
    const {
  return systems::System<T>::get_input_port(lane_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>&
PurePursuitController<T>::ego_pose_input() const {
  return systems::System<T>::get_input_port(ego_pose_index_);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
PurePursuitController<T>::driving_command_output() const {
  return systems::System<T>::get_output_port(command_output_index_);
}

template <typename T>
void PurePursuitController<T>::DoCalcOutput(
    const systems::Context<T>& context,
    systems::SystemOutput<T>* output) const {
  // Obtain the parameters.
  const PurePursuitParams<T>& pp_params =
      this->template GetNumericParameter<PurePursuitParams>(context,
                                                            kPpParamsIndex);
  const SimpleCarParams<T>& car_params =
      this->template GetNumericParameter<SimpleCarParams>(context,
                                                          kCarParamsIndex);

  // Obtain the input/output data structures.
  const DrivingCommand<T>* const input_command =
      this->template EvalVectorInput<DrivingCommand>(
          context, this->driving_command_input().get_index());
  DRAKE_ASSERT(input_command != nullptr);

  const LaneDirection* const lane_direction =
      this->template EvalInputValue<LaneDirection>(
          context, this->lane_input().get_index());
  DRAKE_ASSERT(lane_direction != nullptr);

  const PoseVector<T>* const ego_pose =
      this->template EvalVectorInput<PoseVector>(
          context, this->ego_pose_input().get_index());
  DRAKE_ASSERT(ego_pose != nullptr);

  systems::BasicVector<T>* const command_output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(command_output_vector != nullptr);
  DrivingCommand<T>* const output_command =
      dynamic_cast<DrivingCommand<T>*>(command_output_vector);
  DRAKE_ASSERT(output_command != nullptr);

  ImplDoCalcOutput(pp_params, car_params, *input_command, *lane_direction,
                   *ego_pose, output_command);
}

template <typename T>
void PurePursuitController<T>::ImplDoCalcOutput(
    const PurePursuitParams<T>& pp_params, const SimpleCarParams<T>& car_params,
    const DrivingCommand<T>& input_command, const LaneDirection& lane_direction,
    const PoseVector<T>& ego_pose, DrivingCommand<T>* output_command) const {
  DRAKE_DEMAND(car_params.IsValid());
  DRAKE_DEMAND(pp_params.IsValid());

  // Compute the steering angle using the pure-pursuit method.  N.B. Assumes
  // zero elevation and superelevation.
  const T steering_angle = PurePursuit<T>::Evaluate(
      pp_params, car_params, lane_direction, road_, ego_pose);

  // Pass the commanded throttle and acceleration through to the output,
  // applying the required steering angle.
  output_command->set_acceleration(input_command.acceleration());
  output_command->set_steering_angle(steering_angle);
}

// These instantiations must match the API documentation in
// pure_pursuit_controller.h.
template class PurePursuitController<double>;

}  // namespace automotive
}  // namespace drake
