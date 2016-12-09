#pragma once

#include <memory>

#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace automotive {

std::unique_ptr<systems::Diagram<double>>
CreatCarSimLcmDiagram(
    const DrivingCommandTranslator& driving_command_translator,
    std::unique_ptr<RigidBodyTree<double>> tree,
    lcm::DrakeLcmInterface* lcm);

}  // namespace automotive
}  // namespace drake
