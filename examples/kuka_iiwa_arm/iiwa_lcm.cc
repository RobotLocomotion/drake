#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/multibody/rigid_body_plant/contact_results.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

IiwaContactResultsToExternalTorque::IiwaContactResultsToExternalTorque(
    const RigidBodyTree<double>& tree,
    const std::vector<int>& model_instance_ids)
    : num_joints_{tree.get_num_velocities()} {
  int length = 0;
  velocity_map_.resize(tree.get_num_model_instances(),
                       std::pair<int, int>(-1, -1));

  for (const auto& body : tree.get_bodies()) {
    if (!body->has_parent_body()) {
      continue;
    }

    const int instance_id = body->get_model_instance_id();
    const int velocity_start_index = body->get_velocity_start_index();
    const int num_velocities = body->getJoint().get_num_velocities();

    if (std::find(model_instance_ids.begin(), model_instance_ids.end(),
                  instance_id) == model_instance_ids.end()) {
      continue;
    }

    if (num_velocities) {
      if (velocity_map_[instance_id].first == -1) {
        velocity_map_[instance_id] =
            std::pair<int, int>(velocity_start_index, num_velocities);
      } else {
        std::pair<int, int> map_entry = velocity_map_[instance_id];
        DRAKE_DEMAND(velocity_start_index ==
                     map_entry.first + map_entry.second);
        map_entry.second += num_velocities;
        velocity_map_[instance_id] = map_entry;
      }
      length += num_velocities;
    }
  }

  this->DeclareAbstractInputPort(
      systems::kUseDefaultName,
      Value<systems::ContactResults<double>>{});

  this->DeclareVectorOutputPort(
      systems::BasicVector<double>(length),
      &IiwaContactResultsToExternalTorque::OutputExternalTorque);
}

void IiwaContactResultsToExternalTorque::OutputExternalTorque(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output) const {
  const systems::ContactResults<double>* contact_results =
      this->EvalInputValue<systems::ContactResults<double>>(context, 0);

  int start = 0;
  const VectorX<double>& generalized_force =
      contact_results->get_generalized_contact_force();
  DRAKE_DEMAND(generalized_force.size() == num_joints_);

  for (const auto& entry : velocity_map_) {
    const int v_idx = entry.first;
    const int v_length = entry.second;
    if (v_idx == -1) continue;

    for (int idx = 0; idx < v_length; idx++) {
      output->SetAtIndex(start + idx, generalized_force[v_idx + idx]);
    }
    start += v_length;
  }
  DRAKE_DEMAND(start == output->size());
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
