#ifndef DRAKE_SYSTEMS_PLANTS_RIGID_BODY_TREE_URDF_H_
#define DRAKE_SYSTEMS_PLANTS_RIGID_BODY_TREE_URDF_H_

#include <memory>
#include <string>

#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/thirdParty/tinyxml2/tinyxml2.h"

namespace drake {
namespace systems {

/// Manufactures a RigidBodyFrame from the given URDF \p link
/// and \p pose nodes.  The link name must exist in the given
/// \p model.
std::shared_ptr<RigidBodyFrame> MakeRigidBodyFrameFromURDFNode(
    const RigidBodyTree& model, const tinyxml2::XMLElement* link,
    const tinyxml2::XMLElement* pose, std::string name);

}  // namespace systems
}  // namespace drake

#endif  // DRAKE_SYSTEMS_PLANTS_RIGID_BODY_TREE_URDF_H_