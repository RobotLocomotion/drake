#include "drake/examples/allegro_hand/in_hand_manipulation/object_pose_publisher.h"

namespace drake {
namespace examples {
namespace allegro_hand {

ObjectPosePublisher::ObjectPosePublisher(const MultibodyPlant<double>& plant,
                                         const std::string& obj_body_name)
    : plant_(&plant), obj_body_name_(obj_body_name) {
  state_input_port_ = this->DeclareInputPort(systems::kVectorValued,
                                             plant_->num_multibody_states())
                          .get_index();
  plant_context_ = plant_->CreateDefaultContext();
  this->DeclarePeriodicPublish(kObjectStatePublishPeriod);
}

void ObjectPosePublisher::DoPublish(
    const systems::Context<double>& context,
    const std::vector<const systems::PublishEvent<double>*>&) const {
  const systems::BasicVector<double>* state_vector =
      this->EvalVectorInput(context, state_input_port_);
  plant_->tree().get_mutable_multibody_state_vector(plant_context_.get()) =
      state_vector->get_value();

  const auto X_WF = plant_->tree().CalcRelativeTransform(
      *plant_context_, plant_->world_frame(),
      plant_->GetFrameByName(obj_body_name_));
  Eigen::Quaternion<double> obj_roation_quaternion(X_WF.rotation());
  robotlocomotion::pose_t msg_obj_pose;
  msg_obj_pose.position.x = X_WF.translation()(0);
  msg_obj_pose.position.y = X_WF.translation()(1);
  msg_obj_pose.position.z = X_WF.translation()(2);
  msg_obj_pose.orientation.w = obj_roation_quaternion.w();
  msg_obj_pose.orientation.x = obj_roation_quaternion.x();
  msg_obj_pose.orientation.y = obj_roation_quaternion.y();
  msg_obj_pose.orientation.z = obj_roation_quaternion.z();

  lcm::DrakeLcm lcm;
  lcm.get_lcm_instance()->publish(kObjectPoseLCMChannel, &msg_obj_pose);
}

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
