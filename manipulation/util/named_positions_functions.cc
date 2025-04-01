#include "drake/manipulation/util/named_positions_functions.h"

#include "drake/common/drake_throw.h"

namespace drake {
namespace manipulation {

using drake::multibody::Joint;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

void ApplyNamedPositionsAsDefaults(const NamedPositions& input,
                                   MultibodyPlant<double>* plant) {
  for (const auto& [model_instance_name, joint_values] : input) {
    const ModelInstanceIndex model_instance_index =
        plant->GetModelInstanceByName(model_instance_name);
    for (const auto& [joint_name, joint_positions] : joint_values) {
      const Joint<double>& joint =
          plant->GetJointByName(joint_name, model_instance_index);

      if (joint_positions.size() != joint.num_positions()) {
        throw std::runtime_error(fmt::format(
            "{}: The number of positions in the input ({}) does not match the "
            "number of positions of the joint '{}::{}' ({}).",
            __func__, joint_positions.size(), model_instance_name, joint_name,
            joint.num_positions()));
      }

      plant->get_mutable_joint(joint.index())
          .set_default_positions(joint_positions);
    }
  }
}

}  // namespace manipulation
}  // namespace drake
