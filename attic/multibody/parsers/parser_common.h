#pragma once

#include <memory>
#include <vector>

#include <tinyxml2.h>

#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/compliant_material.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace parsers {

/// Defines constants used by AddFloatingJoint().
struct FloatingJointConstants {
  static const char* const kFloatingJointName;
  static const char* const kWeldJointName;
};

// TODO(liang.fok): Deprecate this method. See: #3361.
/**
 * Adds a floating joint to each body specified by @p body_indices that does
 * not already have a parent.
 *
 * This method is only intended to be called by parsers since parsers add bodies
 * to the RigidBodyTree _en masse_. The logic in this method is necessary to
 * identify which of the rigid bodies specified by @p body_indices get floating
 * joints.
 *
 * When manually adding a model instance to the RigidBodyTree, i.e., directly
 * using the C++ API rather than via a parser, this method should _not_ be
 * necessary since floating joints can be directly added by calling
 * RigidBody::setJoint().
 *
 * @param[in] floating_base_type The floating joint's type.
 *
 * @param[in] body_indices A list of body indexes to check. A floating joint is
 * added to any body in this list that does not have a parent joint.
 *
 * @param[in] weld_to_frame The frame to which the floating joint should attach
 * the parent-less non-world bodies. This parameter may be nullptr, in which
 * case the body is welded to the world with zero offset.
 *
 * @param[in] pose_map A mapping where the key is the body's name and the value
 * is the transform from the frame of the body to the frame of the model to
 * which the body belongs. This parameter will may be nullptr, in which case an
 * identity transform is used.
 *
 * @param[out] tree The RigidBodyTree to which to add the floating joints.
 *
 * @return The number of floating joint added to this rigid body tree.
 *
 * @throws std::runtime_error if the floating_base_type is unrecognized or
 * zero floating joints were added to the model.
 */
int AddFloatingJoint(
    multibody::joints::FloatingBaseType floating_base_type,
    const std::vector<int>& body_indices,
    const std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    const PoseMap* pose_map,
    RigidBodyTree<double>* tree);

/// Instantiates a CompliantMaterial instance from an XMLNode.
/// It ignores unrecognized elements, but throws an exception if a recognized
/// element's contents cannot be converted to a double. Omitted property
/// elements remain tied to the default parameter value.
/// If either friction coefficient is defined, _both_ must be defined.
/// Furthermore, the coefficient for static friction must be greater than or
/// equal to the dynamic friction and both must be non-negative.
///
/// Looks for the following tags in URDF and SDF:
///
/// ```xml
/// ...
/// <collision ...>
///   <geometry...>
///   </geometry>
///
///   <drake_compliance>
///     <youngs_modulus>##</youngs_modulus>
///     <dissipation>##</dissipation>
///     <static_friction>##</static_friction>
///     <dynamic_friction>##</dynamic_friction>
///   </drake_compliance>
///
/// </collision>
/// ...
/// ```
/// @param[in] node The *parent* node which ostensibly contains a declaration of
/// drake compliance.
systems::CompliantMaterial ParseCollisionCompliance(tinyxml2::XMLElement* node);

/// Parses the Drake collision filter group specification. Attempts to add
/// collision filter groups (with their member lists and ignore lists) to the
/// tree specification.  Inconsistent definitions will lead to thrown
/// exceptions.
///
/// See @ref cfg_impl "this discussion" for details on the XML specification.
///
/// @param tree                  The rigid body tree containing the bodies to
///                              which the filters will be applied.
/// @param node                  The XML node containing the filter details.
/// @param model_instance_id     The id of the current model instance.
void ParseCollisionFilterGroup(RigidBodyTree<double>* tree,
                               tinyxml2::XMLElement* node,
                               int model_instance_id);
}  // namespace parsers
}  // namespace drake
