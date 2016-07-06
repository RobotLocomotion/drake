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

/// Manufactures a `RigidBodyFrame` from the given URDF \p link
/// and \p pose XML nodes.  The link name must exist in the given
/// \p model. Otherwise, a runtime error is thrown.
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
std::shared_ptr<RigidBodyFrame> MakeRigidBodyFrameFromURDFNode(
    const RigidBodyTree& tree, const tinyxml2::XMLElement& link,
    const tinyxml2::XMLElement* pose, const std::string& name);

// TODO(liang.fok) Replace this method with one that returns a Model object.
//
/// Adds a URDF model to a rigid body system. The model is connected to the
/// world via a joint of type `DrakeJoint::ROLLPITCHYAW` joint. The model's
/// frame is equal to the world's coordinate frame.
///
/// @param[in] urdf_string The URDF description of the model to be added to
/// \p tree.
///
/// @param[out] tree The `RigidBodyTree` to which to add the model.
/// A `std::runtime_error` is thrown if the pointer is null.
DRAKERBM_EXPORT
void addRobotFromURDFString(
    const std::string& urdf_string,
    RigidBodyTree* tree);

// TODO(liang.fok) Replace this method with one that returns a Model object.
//
/// Adds a URDF model to a rigid body system. The model is connected to the
/// world via a joint of type `DrakeJoint::ROLLPITCHYAW` joint. The model's
/// frame is equal to the world's coordinate frame.
///
/// @param[in] urdf_string The URDF description of the model to be added to
/// \p tree.
///
/// @param[in] package_map A map of ROS package names to their paths. These are
/// the packages to search through when finding files referenced in the URDF.
///
/// @param[out] tree The `RigidBodyTree` to which to add the model.
/// A `std::runtime_error` is thrown if the pointer is null.
DRAKERBM_EXPORT
void addRobotFromURDFString(
    const std::string& urdf_string,
    std::map<std::string, std::string>& package_map,
    RigidBodyTree* tree);


// TODO(liang.fok) Replace this method with one that returns a Model object.
//
/// Adds a URDF model to a rigid body system.
///
/// @param[in] urdf_string The URDF description of the model to be added to
/// \p tree.
///
/// @param[in] root_dir The root directory in which to search for files
/// mentioned in the URDF.
///
/// @param[in] floating_base_type The type of joint that connects the model's
/// root to the existing rigid body tree.
///
/// @param[out] tree The `RigidBodyTree` to which to add the model.
/// A `std::runtime_error` is thrown if the pointer is null.
DRAKERBM_EXPORT
void addRobotFromURDFString(
    const std::string& urdf_string,
    const std::string& root_dir,
    const DrakeJoint::FloatingBaseType floating_base_type,
    RigidBodyTree* tree);

// TODO(liang.fok) Replace this method with one that returns a Model object.
//
/// Adds a URDF model to a rigid body system.
///
/// @param[in] urdf_string The URDF description of the model to be added to
/// \p tree.
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
/// @param[out] tree The `RigidBodyTree` to which to add the model.
/// A `std::runtime_error` is thrown if the pointer is null.
DRAKERBM_EXPORT
void addRobotFromURDFString(
    const std::string& urdf_string,
    PackageMap& package_map,
    const std::string& root_dir,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree);

// TODO(liang.fok) Replace this method with one that returns a Model object.
//
/// Adds a URDF model to a rigid body system. The model is connected to the
/// world via a joint of type `DrakeJoint::ROLLPITCHYAW` joint. The model's
/// frame is equal to the world's coordinate frame.
///
/// @param[in] urdf_filename The URDF file containing the model to be added.
///
/// @param[out] tree The `RigidBodyTree` to which to add the model.
/// A `std::runtime_error` is thrown if the pointer is null.
DRAKERBM_EXPORT
void addRobotFromURDF(const std::string& urdf_filename,
    RigidBodyTree* tree);

// TODO(liang.fok) Replace this method with one that returns a Model object.
//
/// Adds a URDF model to a rigid body system. The model is connected to the
/// world via a joint of type `DrakeJoint::ROLLPITCHYAW` joint. The model's
/// frame is equal to the world's coordinate frame.
///
/// @param[in] urdf_filename The URDF file containing the model to be added.
///
/// @param[in] floating_base_type The type of joint that connects the model's
/// root to the existing rigid body tree.
///
/// @param[out] tree The `RigidBodyTree` to which to add the model.
/// A `std::runtime_error` is thrown if the pointer is null.
DRAKERBM_EXPORT
void addRobotFromURDF(
    const std::string& urdf_filename,
    const DrakeJoint::FloatingBaseType floating_base_type,
    RigidBodyTree* tree);

// TODO(liang.fok) Replace this method with one that returns a Model object.
//
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
/// @param[out] tree The `RigidBodyTree` to which to add the model.
/// A `std::runtime_error` is thrown if the pointer is null.
DRAKERBM_EXPORT
void addRobotFromURDF(
    const std::string& urdf_filename,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree);

// TODO(liang.fok) Replace this method with one that returns a Model object.
//
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
/// @param[out] tree The `RigidBodyTree` to which to add the model.
/// A `std::runtime_error` is thrown if the pointer is null.
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
