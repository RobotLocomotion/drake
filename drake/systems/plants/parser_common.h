#pragma once

// #include <string>

#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace parsers {

/**
 * Adds one floating joint to each link specified in @p link_indices that does
 * not already have a parent. Typically, @p link_indices is created while
 * calling RigidBodyTree::add_rigid_body(). The floating joints connect the
 * links and of their child branches to the rigid body tree.
 *
 * @param[out] tree The RigidBodyTree to which to add the floating joints.
 *
 * @param[in] floating_base_type The floating joint's type.
 *
 * @param[in] link_indices A list of link indexes to check. A floating joint is
 * added to any link in this list that does not have a parent joint.
 *
 * @param[in] weld_to_frame The frame to which the floating joint should attach
 * the parent-less non-world links. This parameter may be nullptr, in which case
 * the link is welded to the world with zero offset.
 *
 * @param[in] pose_map A mapping where the key is the link's name and the value
 * is the transform from the frame of the link to the frame of the model to
 * which the link belongs.
 *
 * @return The number of floating joint added to this rigid body tree.
 *
 * @throws A std::runtime_error if the floating_base_type is unrecognized or
 * zero floating joints were added to the model.
 */
DRAKERBM_EXPORT
int AddFloatingJoint(
	RigidBodyTree* tree,
    FloatingBaseType floating_base_type,
    const std::vector<int>& link_indices,
    const std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr,
    const PoseMap* pose_map = nullptr);

}  // namespace parsers
}  // namespace drake
