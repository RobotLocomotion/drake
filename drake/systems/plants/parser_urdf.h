#pragma once

namespace drake {
namespace parsers {

/// Adds a URDF model to a rigid body system. The model is connected to the
/// world via a joint of type `DrakeJoint::ROLLPITCHYAW` joint. The model's
/// frame is equal to the world's coordinate frame.
///
/// @param[in] urdf_string The URDF string.
///
/// @param[out] system The system to which to add the model.
void addRobotFromURDFString(
    const std::string& urdf_string,
    RigidBodySystem* system);

/// Adds a URDF model to a rigid body system. The model is connected to the
/// world via a joint of type `DrakeJoint::ROLLPITCHYAW` joint. The model's
/// frame is equal to the world's coordinate frame.
///
/// @param[in] urdf_string The URDF string.
///
/// @param[in] package_map A map of ROS package names to their paths. These are
/// the packages to search through when finding files referenced in the URDF.
///
/// @param[out] system The system to which to add the model.
void addRobotFromURDFString(
    const std::string& urdf_string,
    std::map<std::string, std::string>& package_map,
    RigidBodySystem* system);


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
/// @param[out] system The system to which to add the model.
void addRobotFromURDFString(
    const string& urdf_string,
    const std::string& root_dir,
    const DrakeJoint::FloatingBaseType floating_base_type,
    RigidBodySystem* system);

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
/// @param[out] system The system to which to add the model.
void addRobotFromURDFString(
    const string& urdf_string,
    PackageMap& package_map,
    const std::string& root_dir,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame
    RigidBodySystem* system);

/// Adds a URDF model to a rigid body system. The model is connected to the
/// world via a joint of type `DrakeJoint::ROLLPITCHYAW` joint. The model's
/// frame is equal to the world's coordinate frame.
///
/// @param[in] urdf_filename The URDF file containing the model to be added.
///
/// @param[out] system The system to which to add the model.
void addRobotFromURDF(const std::string& urdf_filename,
    RigidBodySystem* system);

/// Adds a URDF model to a rigid body system. The model is connected to the
/// world via a joint of type `DrakeJoint::ROLLPITCHYAW` joint. The model's
/// frame is equal to the world's coordinate frame.
///
/// @param[in] urdf_filename The URDF file containing the model to be added.
///
/// @param[in] weld_to_frame The frame to which to connect the new model.
///
/// @param[out] system The system to which to add the model.
void addRobotFromURDF(
    const std::string& urdf_filename,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodySystem* system);

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
/// @param[out] system The system to which to add the model.
void addRobotFromURDF(
    const std::string& urdf_filename,
    std::map<std::string, std::string>& package_map,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodySystem* system);

}  // namespace parsers
}  // namespace drake
