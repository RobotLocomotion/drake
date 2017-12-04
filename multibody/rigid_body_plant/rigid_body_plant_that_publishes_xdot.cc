#include "drake/multibody/rigid_body_plant/rigid_body_plant_that_publishes_xdot.h"

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"

using Eigen::VectorXd;

namespace drake {
namespace systems {

template <typename T>
RigidBodyPlantThatPublishesXdot<T>::RigidBodyPlantThatPublishesXdot(
    std::unique_ptr<const RigidBodyTree<T>> tree,
    const std::string& channel, drake::lcm::DrakeLcmInterface* lcm)
    : RigidBodyPlant<T>::RigidBodyPlant(move(tree)), lcm_(lcm),
      channel_(channel) {
  derivatives_ = LeafSystem<T>::AllocateTimeDerivatives();
  const RigidBodyTree<T>& rigid_body_tree =
      RigidBodyPlant<T>::get_rigid_body_tree();
  const int num_states = RigidBodyPlant<T>::get_num_states();
  message_.dim = num_states;
  message_.val.resize(num_states, 0);  // Initializes vector to contain zeros.
  for (int i = 0; i < rigid_body_tree.get_num_positions(); ++i) {
    message_.coord.push_back(rigid_body_tree.get_position_name(i));
  }
  for (int i = 0; i < rigid_body_tree.get_num_velocities(); ++i) {
    message_.coord.push_back(rigid_body_tree.get_velocity_name(i));
  }
  DRAKE_DEMAND(message_.coord.size() == message_.val.size());
}

template <typename T>
RigidBodyPlantThatPublishesXdot<T>::~RigidBodyPlantThatPublishesXdot() {}

// TODO(liang.fok) Eliminate the re-computation of `xdot` once it is cached.
// Ideally, switch to outputting the derivatives in an output port. This can
// only be done once #2890 is resolved.
template <typename T>
void RigidBodyPlantThatPublishesXdot<T>::DoPublish(
                        const Context<T>& context,
                        const std::vector<const PublishEvent<double>*>&) const {
  RigidBodyPlant<T>::CalcTimeDerivatives(context, derivatives_.get());
  const auto xdot = derivatives_->CopyToVector();
  const int num_states = RigidBodyPlant<T>::get_num_states();

  DRAKE_DEMAND(xdot.size() == num_states);
  VectorXd::Map(message_.val.data(), xdot.size()) = xdot;

  // Saves the timestamp in milliseconds. This matches the behavior in
  // LcmtDrakeSignalTranslator::Serialize().
  message_.timestamp = static_cast<int64_t>(context.get_time() * 1000);

  const int num_bytes = message_.getEncodedSize();
  std::vector<uint8_t> message_bytes(num_bytes);
  const int num_bytes_encoded =
      message_.encode(message_bytes.data(), 0 /* offset */, num_bytes);
  DRAKE_DEMAND(num_bytes == num_bytes_encoded);
  lcm_->Publish(channel_, message_bytes.data(), num_bytes);
}

// Explicitly instantiates on the most common scalar types.
template class RigidBodyPlantThatPublishesXdot<double>;

}  // namespace systems
}  // namespace drake
