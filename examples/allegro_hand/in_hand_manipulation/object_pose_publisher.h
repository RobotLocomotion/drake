#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

#include "robotlocomotion/pose_t.hpp"

namespace drake {
namespace examples {
namespace allegro_hand {

using drake::multibody::multibody_plant::MultibodyPlant;

const std::string kObjectPoseLCMChannel = "TARGET_OBJ_POSE";

const double kObjectStatePublishPeriod = 0.05;

/// The class track the pose of the target object and publish it
/// to LCM channel
class ObjectPosePublisher : public systems::LeafSystem<double> {
 public:
  ObjectPosePublisher(const MultibodyPlant<double>& plant,
                      const std::string& obj_body_name);

  const systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_input_port_);
  }

 private:
  void DoPublish(const systems::Context<double>& context,
                 const std::vector<const systems::PublishEvent<double>*>&
                     events) const override;

  const MultibodyPlant<double>* plant_{nullptr};
  const std::string obj_body_name_;
  std::unique_ptr<systems::Context<double>> plant_context_;
  int state_input_port_{-1};
};

}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake
