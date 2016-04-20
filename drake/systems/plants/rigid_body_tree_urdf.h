#pragma once

#include <memory>
#include <string>

#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/thirdParty/tinyxml2/tinyxml2.h"

namespace drake {
namespace systems {

/// Manufactures a RigidBodyFrame from the given URDF \p link
/// and \p pose nodes.  The link name must exist in the given
/// \p model.
DRAKERBM_EXPORT std::shared_ptr<RigidBodyFrame> MakeRigidBodyFrameFromURDFNode(
    const RigidBodyTree& model, const tinyxml2::XMLElement* link,
    const tinyxml2::XMLElement* pose, const std::string& name);

}  // namespace systems
}  // namespace drake
