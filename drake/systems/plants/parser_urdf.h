#pragma once

#include <map>
#include <string>

#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/xmlUtil.h"
#include "drake/thirdParty/zlib/tinyxml2/tinyxml2.h"

namespace drake {
namespace parsers {
namespace urdf {

/// Manufactures a `RigidBodyFrame` from the given URDF @p link
/// and @p pose XML nodes.  The link name must exist in the given
/// @p model. Otherwise, a runtime error is thrown.
///
/// @param[in] tree The rigid body tree that holds the rigid body to which the
/// new `RigidBodyFrame` is attached.
///
/// @param[in] link The URDF XML node describing the rigid body to which the
/// new `RigidBodyFrame` is attached.
///
/// @param[in] pose The URDF XML node describing the pose of the new
/// `RigidBodyFrame` in the frame of the rigid body to which it is attached.
/// If this is nullptr, a pose of zero (i.e., identity transform) is used.
///
/// @param[in] name The name of the new `RigidBodyFrame`.
///
/// @return The new `RigidBodyFrame`.
///
/// @throws std::runtime_error if the rigid body to which the new
/// `RigidBodyFrame` is attached is not found.
DRAKERBM_EXPORT
std::shared_ptr<RigidBodyFrame> MakeRigidBodyFrameFromUrdfNode(
    const RigidBodyTree& tree, const tinyxml2::XMLElement& link,
    const tinyxml2::XMLElement* pose, const std::string& name);

/// Reads a single model from a URDF specification and adds a single instance of
/// it to @p tree. The model instance is connected to the world via
/// a joint of type `DrakeJoint::ROLLPITCHYAW`. The model instance's frame
/// is equal to the world's coordinate frame.
///
/// @param[in] description The URDF description of the model. This is the actual
/// URDF text (i.e., it is not the name of a file that contains the URDF text).
/// A new model instance is created based on this URDF text and added to
/// @p tree.
///
/// @param[out] tree The `RigidBodyTree` to which to add the model instance.
/// This parameter must not be `nullptr`.
///
/// @param[out] model_instance_id_map A pointer to a map where the key is the
/// name of the model whose instance was just added to this `RigidBodySystem`
/// and it's `RigidBodyTree` and the value is the unique model instance ID that
/// was assigned to the instance. This parameter may not be `nullptr`.
DRAKERBM_EXPORT
void AddModelInstanceFromUrdfDescription(
    const std::string& description,
    RigidBodyTree* tree,
    RigidBodyTree::ModelToInstanceIDMap* model_instance_id_map);

/// Reads a single model from a URDF specification and adds a single instance of
/// it to @p tree. The model instance is connected to the world via
/// a joint of type `DrakeJoint::ROLLPITCHYAW`. The model instance's frame is
/// equal to the world's coordinate frame.
///
/// @param[in] description The URDF description of the model. This is the actual
/// URDF text (i.e., it is not the name of a file that contains the URDF text).
/// A new model instance is created based on this URDF text and added to
/// @p tree.
///
/// @param[in] package_map A map of ROS package names to their paths. These are
/// the packages to search through when searching for files referenced in the
/// URDF.
///
/// @param[out] tree The `RigidBodyTree` to which to add the model instance.
/// This parameter must not be `nullptr`.
///
/// @param[out] model_instance_id_map A pointer to a map where the key is the
/// name of the model whose instance was just added to this `RigidBodySystem`
/// and it's `RigidBodyTree` and the value is the unique model instance ID that
/// was assigned to the instance. This parameter may not be `nullptr`.
DRAKERBM_EXPORT
void AddModelInstanceFromUrdfDescription(
    const std::string& description,
    std::map<std::string, std::string>& package_map,
    RigidBodyTree* tree,
    RigidBodyTree::ModelToInstanceIDMap* model_instance_id_map);

/// Reads a single model from a URDF specification and adds a single instance of
/// it to @p tree. The model instance is connected to the world via
/// a joint of type @p floating_base_type. The model instance's frame is equal
/// to the world's coordinate frame.
///
/// @param[in] description The URDF description of the model. This is the actual
/// URDF text (i.e., it is not the name of a file that contains the URDF text).
/// A new model instance is created based on this URDF text and added to
/// @p tree.
///
/// @param[in] root_dir The root directory in which to search for files
/// mentioned in the URDF.
///
/// @param[in] floating_base_type The type of joint that connects the model's
/// root to @p tree.
///
/// @param[in] weld_to_frame The frame to which to connect the new model
/// instance.
///
/// @param[out] tree The `RigidBodyTree` to which to add the model instance.
/// This parameter must not be `nullptr`.
///
/// @param[out] model_instance_id_map A pointer to a map where the key is the
/// name of the model whose instance was just added to this `RigidBodySystem`
/// and it's `RigidBodyTree` and the value is the unique model instance ID that
/// was assigned to the instance. This parameter may not be `nullptr`.
DRAKERBM_EXPORT
void AddModelInstanceFromUrdfDescription(
    const std::string& description,
    const std::string& root_dir,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree,
    RigidBodyTree::ModelToInstanceIDMap* model_instance_id_map);

/// Reads a single model from a URDF specification and adds a single instance of
/// it to @p tree.
///
/// @param[in] description The URDF description of the model. This is the actual
/// URDF text (i.e., it is not the name of a file that contains the URDF text).
/// A new model instance is created based on this URDF text and added to
/// @p tree.
///
/// @param[in] package_map A map of ROS package names to their paths. These are
/// the packages to search through when finding files referenced in the URDF.
///
/// @param[in] root_dir The root directory in which to search for files
/// mentioned in the URDF.
///
/// @param[in] floating_base_type The type of joint that connects the model
/// instance's root to @p tree.
///
/// @param[in] weld_to_frame The frame to which to connect the new model
/// instance.
///
/// @param[out] tree The `RigidBodyTree` to which to add the model instance.
/// This parameter must not be `nullptr`.
///
/// @param[out] model_instance_id_map A pointer to a map where the key is the
/// name of the model whose instance was just added to this `RigidBodySystem`
/// and it's `RigidBodyTree` and the value is the unique model instance ID that
/// was assigned to the instance. This parameter may not be `nullptr`. A
/// `std::runtime_error` is thrown if a model instance is created whose name is
/// already in this map.
DRAKERBM_EXPORT
void AddModelInstanceFromUrdfDescription(
    const std::string& description,
    PackageMap& package_map,
    const std::string& root_dir,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree,
    RigidBodyTree::ModelToInstanceIDMap* model_instance_id_map);

/// Reads a single model from a URDF specification and adds a single instance of
/// it to @p tree. The model instance is connected to the world via a joint of
/// type `DrakeJoint::ROLLPITCHYAW`. The model instance's frame is equal to the
/// world's coordinate frame.
///
/// @param[in] filename The name of the file containing a URDF
/// description of the model. An instance of this model will be added to
/// @p tree.
///
/// @param[out] tree The `RigidBodyTree` to which to add the model. This
/// parameter must not be `nullptr`.
///
/// @param[out] model_instance_id_map A pointer to a map where the key is the
/// name of the model whose instance was just added to this `RigidBodySystem`
/// and it's `RigidBodyTree` and the value is the unique model instance ID that
/// was assigned to the instance. This parameter may not be `nullptr`. A
/// `std::runtime_error` is thrown if a model instance is created whose name is
/// already in this map.
DRAKERBM_EXPORT
void AddModelInstanceFromUrdfFile(
    const std::string& filename,
    RigidBodyTree* tree,
    RigidBodyTree::ModelToInstanceIDMap* model_instance_id_map);

/// Reads a single model from a URDF specification and adds a single instance of
/// it to @p tree. The model instance is connected to the world via a joint of
/// type `DrakeJoint::ROLLPITCHYAW`. The model's frame is equal to the world's
/// coordinate frame.
///
/// @param[in] filename The name of the file containing a URDF
/// description of the model. An instance of this model will be added to
/// @p tree.
///
/// @param[in] floating_base_type The type of joint that connects the model
/// instance's root to @p tree.
///
/// @param[out] tree The `RigidBodyTree` to which to add the model instance.
/// This parameter must not be `nullptr`.
///
/// @param[out] model_instance_id_map A pointer to a map where the key is the
/// name of the model whose instance was just added to this `RigidBodySystem`
/// and it's `RigidBodyTree` and the value is the unique model instance ID that
/// was assigned to the instance. This parameter may not be `nullptr`. A
/// `std::runtime_error` is thrown if a model instance is created whose name is
/// already in this map.
DRAKERBM_EXPORT
void AddModelInstanceFromUrdfFile(
    const std::string& filename,
    const DrakeJoint::FloatingBaseType floating_base_type,
    RigidBodyTree* tree,
    RigidBodyTree::ModelToInstanceIDMap* model_instance_id_map);

/// Reads a single model from a URDF specification and adds a single instance of
/// it to @p tree. The model instance is connected to the world via a joint of
/// type `DrakeJoint::ROLLPITCHYAW` joint. The model instance's frame is equal
/// to the world's coordinate frame.
///
/// @param[in] filename The name of the file containing a URDF
/// description of the model. An instance of this model will be added to
/// @p tree.
///
/// @param[in] floating_base_type The type of joint that connects the model
/// instance's root to the @p tree.
///
/// @param[in] weld_to_frame The frame to which to connect the new model
/// instance.
///
/// @param[out] tree The `RigidBodyTree` to which to add the model instance.
/// This parameter must not be `nullptr`.
///
/// @param[out] model_instance_id_map A pointer to a map where the key is the
/// name of the model whose instance was just added to this `RigidBodySystem`
/// and it's `RigidBodyTree` and the value is the unique model instance ID that
/// was assigned to the instance. This parameter may not be `nullptr`. A
/// `std::runtime_error` is thrown if a model instance is created whose name is
/// already in this map.
DRAKERBM_EXPORT
void AddModelInstanceFromUrdfFile(
    const std::string& filename,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree,
    RigidBodyTree::ModelToInstanceIDMap* model_instance_id_map);

/// Reads a single model from a URDF specification and adds a single instance of
/// it to @p tree.  The model instance is connected to the world via a joint of
/// type `DrakeJoint::ROLLPITCHYAW`. The model instance's frame is equal to the
/// world's coordinate frame.
///
/// @param[in] filename The name of the file containing a URDF
/// description of the model. An instance of this model will be added to
/// @p tree.
///
/// @param[in] package_map A map of ROS package names to their paths. These are
/// the packages to search through when finding files referenced in the URDF.
///
/// @param[in] floating_base_type The type of joint that connects the model
/// instance's root to the @p tree.
///
/// @param[in] weld_to_frame The frame to which to connect the new model
/// instance.
///
/// @param[out] tree The `RigidBodyTree` to which to add the model instance.
/// This parameter must not be `nullptr`.
///
/// @param[out] model_instance_id_map A pointer to a map where the key is the
/// name of the model whose instance was just added to this `RigidBodySystem`
/// and it's `RigidBodyTree` and the value is the unique model instance ID that
/// was assigned to the instance. This parameter may be `nullptr`. A
/// `std::runtime_error` is thrown if a model instance is created whose name is
/// already in this map.
DRAKERBM_EXPORT
void AddModelInstanceFromUrdfFile(
    const std::string& filename,
    std::map<std::string, std::string>& package_map,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree,
    RigidBodyTree::ModelToInstanceIDMap* model_instance_id_map = nullptr);

}  // namespace urdf
}  // namespace parsers
}  // namespace drake
