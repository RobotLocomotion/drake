#pragma once

#include <map>
#include <string>

#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/xmlUtil.h"
#include "drake/thirdParty/tinyxml2/tinyxml2.h"

namespace drake {
namespace parsers {
namespace urdf {

/// Manufactures a RigidBodyFrame from the given URDF \p link
/// and \p pose nodes.  The link name must exist in the given
/// \p model.
DRAKERBM_EXPORT
std::shared_ptr<RigidBodyFrame> MakeRigidBodyFrameFromURDFNode(
    const RigidBodyTree& model, const tinyxml2::XMLElement* link,
    const tinyxml2::XMLElement* pose, const std::string& name);

/// Adds a URDF model to a rigid body system. The model is connected to the
/// world via a joint of type `DrakeJoint::ROLLPITCHYAW` joint. The model's
/// frame is equal to the world's coordinate frame.
///
/// @param[in] urdf_string The URDF string.
///
/// @param[out] tree The rigid body tree to which to add the model.
DRAKERBM_EXPORT
void addRobotFromURDFString(
    const std::string& urdf_string,
    RigidBodyTree* tree);

/// Adds a URDF model to a rigid body system. The model is connected to the
/// world via a joint of type `DrakeJoint::ROLLPITCHYAW` joint. The model's
/// frame is equal to the world's coordinate frame.
///
/// @param[in] urdf_string The URDF string.
///
/// @param[in] package_map A map of ROS package names to their paths. These are
/// the packages to search through when finding files referenced in the URDF.
///
/// @param[out] tree The rigid body tree to which to add the model.
DRAKERBM_EXPORT
void addRobotFromURDFString(
    const std::string& urdf_string,
    std::map<std::string, std::string>& package_map,
    RigidBodyTree* tree);


/// Adds a URDF model to a rigid body system.
///
/// @param[in] urdf_string The URDF string.
///
/// @param[in] root_dir The root directory in which to search for files
/// mentioned in the URDF.
///
/// @param[in] floating_base_type The type of joint that connects the model's
/// root to the existing rigid body tree.
///
/// @param[out] tree The rigid body tree to which to add the model.
DRAKERBM_EXPORT
void addRobotFromURDFString(
    const std::string& urdf_string,
    const std::string& root_dir,
    const DrakeJoint::FloatingBaseType floating_base_type,
    RigidBodyTree* tree);

/// Adds a URDF model to a rigid body system.
///
/// @param[in] urdf_string The URDF string.
///
/// @param[in] package_map A map of ROS package names to their paths. These are
/// the packages to search through when finding files referenced in the URDF.
///
/// @param[in] root_dir The root directory in which to search for files
/// mentioned in the URDF.
///
/// @param[in] floating_base_type The type of joint that connects the model's
/// root to the existing rigid body tree.
///
/// @param[in] weld_to_frame The frame to which to connect the new model.
///
/// @param[out] tree The rigid body tree to which to add the model.
DRAKERBM_EXPORT
void addRobotFromURDFString(
    const std::string& urdf_string,
    PackageMap& package_map,
    const std::string& root_dir,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree);

/// Adds a URDF model to a rigid body system. The model is connected to the
/// world via a joint of type `DrakeJoint::ROLLPITCHYAW` joint. The model's
/// frame is equal to the world's coordinate frame.
///
/// @param[in] urdf_filename The URDF file containing the model to be added.
///
/// @param[out] tree The rigid body tree to which to add the model.
DRAKERBM_EXPORT
void addRobotFromURDF(const std::string& urdf_filename,
    RigidBodyTree* tree);

/// Adds a URDF model to a rigid body system. The model is connected to the
/// world via a joint of type `DrakeJoint::ROLLPITCHYAW` joint. The model's
/// frame is equal to the world's coordinate frame.
///
/// @param[in] urdf_filename The URDF file containing the model to be added.
///
/// @param[in] floating_base_type The type of joint that connects the model's
/// root to the existing rigid body tree.
///
/// @param[out] tree The rigid body tree to which to add the model.
DRAKERBM_EXPORT
void addRobotFromURDF(
    const std::string& urdf_filename,
    const DrakeJoint::FloatingBaseType floating_base_type,
    RigidBodyTree* tree);

/// Adds a URDF model to a rigid body system. The model is connected to the
/// world via a joint of type `DrakeJoint::ROLLPITCHYAW` joint. The model's
/// frame is equal to the world's coordinate frame.
///
/// @param[in] urdf_filename The URDF file containing the model to be added.
///
/// @param[in] floating_base_type The type of joint that connects the model's
/// root to the existing rigid body tree.
///
/// @param[in] weld_to_frame The frame to which to connect the new model.
///
/// @param[out] tree The rigid body tree to which to add the model.
DRAKERBM_EXPORT
void addRobotFromURDF(
    const std::string& urdf_filename,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree);

/// Adds a URDF model to a rigid body system. The model is connected to the
/// world via a joint of type `DrakeJoint::ROLLPITCHYAW` joint. The model's
/// frame is equal to the world's coordinate frame.
///
/// @param[in] urdf_filename The URDF file containing the model to be added.
///
/// @param[in] package_map A map of ROS package names to their paths. These are
/// the packages to search through when finding files referenced in the URDF.
///
/// @param[in] floating_base_type The type of joint that connects the model's
/// root to the existing rigid body tree.
///
/// @param[in] weld_to_frame The frame to which to connect the new model.
///
/// @param[out] tree The rigid body tree to which to add the model.
DRAKERBM_EXPORT
void addRobotFromURDF(
    const std::string& urdf_filename,
    std::map<std::string, std::string>& package_map,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree);

}  // namespace urdf
}  // namespace parsers
}  // namespace drake
