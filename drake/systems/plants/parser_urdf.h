#pragma once

#include <map>
#include <string>

#include "drake/common/drake_deprecated.h"
#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/xmlUtil.h"
#include "drake/thirdParty/zlib/tinyxml2/tinyxml2.h"

namespace drake {
namespace parsers {
namespace urdf {

/**
 * Manufactures a `RigidBodyFrame` from the given URDF @p link
 * and @p pose XML nodes.  The link name must exist in the given
 * @p model. Otherwise, a runtime error is thrown.
 *
 * @param[in] tree The rigid body tree that holds the rigid body to which the
 * new `RigidBodyFrame` is attached.
 *
 * @param[in] link The URDF XML node describing the rigid body to which the
 * new `RigidBodyFrame` is attached.
 *
 * @param[in] pose The URDF XML node describing the pose of the new
 * `RigidBodyFrame` in the frame of the rigid body to which it is attached.
 * If this is nullptr, a pose of zero (i.e., identity transform) is used.
 *
 * @param[in] name The name of the new `RigidBodyFrame`.
 *
 * @param[in] model_instance_id The instance ID of the model to which the frame
 * belongs.
 *
 * @return The new `RigidBodyFrame`.
 *
 * @throws std::runtime_error if the rigid body to which the new
 * `RigidBodyFrame` is attached is not found.
 */
DRAKERBM_EXPORT
std::shared_ptr<RigidBodyFrame> MakeRigidBodyFrameFromUrdfNode(
    const RigidBodyTree& tree, const tinyxml2::XMLElement& link,
    const tinyxml2::XMLElement* pose, const std::string& name,
    int model_instance_id);

/**
 * Reads a URDF model specified by @p urdf_string and adds an instance of it to
 * @p tree. The model instance is connected to the world via a `kRollPitchYaw`
 * joint. When this joint is at its zero position, the model instance's frame is
 * coincident with the world's coordinate frame.
 *
 * @param[in] urdf_string The URDF string of the model. This is the actual
 * URDF text (i.e., it is not the name of a file that contains the URDF text).
 * A new model instance is created based on this URDF text and added to
 * @p tree.
 *
 * @param[out] tree The `RigidBodyTree` to which to add the model instance.
 * This parameter must not be `nullptr`.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to @p tree to their instance IDs, which are unique within @p tree.
 */
DRAKERBM_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfStringWithRpyJointToWorld(
    const std::string& urdf_string, RigidBodyTree* tree);

#ifndef SWIG
  DRAKE_DEPRECATED(
      "Please use AddModelInstanceFromUrdfStringWithRpyJointToWorld().")
#endif
DRAKERBM_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfString(
    const std::string& urdf_string,
    RigidBodyTree* tree);

/**
 * Reads a URDF model specified by @p urdf_string and adds an instance of it to
 * @p tree. The model instance is connected to the world via a `kRollPitchYaw`
 * joint. When this joint is at its zero position, the model instance's frame is
 * coincident with the world's coordinate frame. This method has parameter
 * @p ros_package_map that contains a mapping from ROS package names to their
 * paths on the local file system. This mapping is used to find resources like
 * mesh files that are referenced within the URDF. This method may be called
 * from within the context of a [ROS node](http://wiki.ros.org/Nodes) or a
 * regular non-ROS application.
 *
 * @param[in] urdf_string The URDF string of the model. This is the actual
 * URDF text (i.e., it is not the name of a file that contains the URDF text).
 * A new model instance is created based on this URDF text and added to
 * @p tree.
 *
 * @param[in] ros_package_map A map of ROS package names to their paths. These
 * are the packages to search through when searching for files referenced in the
 * URDF.
 *
 * @param[out] tree The `RigidBodyTree` to which to add the model instance.
 * This parameter must not be `nullptr`.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
DRAKERBM_EXPORT ModelInstanceIdTable
AddModelInstanceFromUrdfStringWithRpyJointToWorldSearchingInRosPackages(
    const std::string& urdf_string,
    std::map<std::string, std::string>& ros_package_map, RigidBodyTree* tree);

#ifndef SWIG
DRAKE_DEPRECATED("Please use AddModelInstanceFromUrdfStringWithRpyJointToWorldSearchingInRosPackages().")  // NOLINT(whitespace/line_length)
#endif
DRAKERBM_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfString(
    const std::string& urdf_string,
    std::map<std::string, std::string>& ros_package_map, RigidBodyTree* tree);

/**
 * Reads a URDF model specified by @p urdf_string and adds an instance of it to
 * @p tree. The model instance is connected to the existing @p tree via a joint
 * of type @p floating_base_type. The body to which this joint attaches and
 * the transform between this body and the model instance's frame when this
 * joint is in its zero position is determined by @p weld_to_frame.
 *
 * @param[in] urdf_string The URDF string of the model. This is the actual
 * URDF text (i.e., it is not the name of a file that contains the URDF text).
 * A new model instance is created based on this URDF text and added to
 * @p tree.
 *
 * @param[in] root_dir The root directory in which to search for files
 * mentioned in the URDF.
 *
 * @param[in] floating_base_type The type of joint that connects the model's
 * root to @p tree.
 *
 * @param[out] tree The `RigidBodyTree` to which to add the model instance.
 * This parameter must not be `nullptr`.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
DRAKERBM_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfString(
    const std::string& urdf_string, const std::string& root_dir,
    const drake::systems::plants::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame, RigidBodyTree* tree);

/**
 * Reads a URDF model specified by @p urdf_string and adds an instance of it to
 * @p tree. The model instance is connected to the existing @p tree via a joint
 * of type @p floating_base_type. The body to which this joint attaches and
 * the transform between this body and the model instance's frame when this
 * joint is in its zero position is determined by @p weld_to_frame. This method
 * has parameter @p ros_package_map that contains a mapping from ROS package names
 * to their paths on the local file system. This mapping is used to find
 * resources like mesh files that are referenced within the URDF. This method
 * may be called from within the context of a
 * [ROS node](http://wiki.ros.org/Nodes) or a regular non-ROS application.
 *
 * @param[in] urdf_string The URDF string of the model. This is the actual
 * URDF text (i.e., it is not the name of a file that contains the URDF text).
 * A new model instance is created based on this URDF text and added to
 * @p tree.
 *
 * @param[in] ros_package_map A map of ROS package names to their paths. These
 * are the packages to search through when finding files referenced in the URDF.
 *
 * @param[in] root_dir The root directory in which to search for files
 * mentioned in the URDF.
 *
 * @param[in] floating_base_type The type of joint that connects the model
 * instance's root to @p tree.
 *
 * @param[in] weld_to_frame The frame to which to connect the new model
 * instance.
 *
 * @param[out] tree The `RigidBodyTree` to which to add the model instance.
 * This parameter must not be `nullptr`.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
DRAKERBM_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfStringSearchingInRosPackages(
    const std::string& urdf_string, PackageMap& ros_package_map,
    const std::string& root_dir,
    const drake::systems::plants::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame, RigidBodyTree* tree);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use AddModelInstanceFromUrdfStringSearchingInRosPackages().")  // NOLINT(whitespace/line_length)
#endif
DRAKERBM_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfString(
    const std::string& urdf_string, PackageMap& ros_package_map,
    const std::string& root_dir,
    const drake::systems::plants::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame, RigidBodyTree* tree);

/**
 * Reads a single model from a URDF specification and adds a single instance of
 * it to @p tree. The model instance is connected to the world via a joint of
 * type `kRollPitchYaw`. The model instance's frame is equal to the
 * world's coordinate frame.
 *
 * @param[in] urdf_filename The name of the file containing a URDF
 * description of the model. An instance of this model will be added to
 * @p tree.
 *
 * @param[out] tree The `RigidBodyTree` to which to add the model. This
 * parameter must not be `nullptr`.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
DRAKERBM_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfFile(
    const std::string& urdf_filename, RigidBodyTree* tree);

/**
 * Reads a single model from a URDF specification and adds a single instance of
 * it to @p tree. The model instance is connected to the world via a joint of
 * type `kRollPitchYaw`. The model's frame is equal to the world's
 * coordinate frame.
 *
 * @param[in] urdf_filename The name of the file containing a URDF
 * description of the model. An instance of this model will be added to
 * @p tree.
 *
 * @param[in] floating_base_type The type of joint that connects the model
 * instance's root to @p tree.
 *
 * @param[out] tree The `RigidBodyTree` to which to add the model instance.
 * This parameter must not be `nullptr`.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
DRAKERBM_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfFile(
    const std::string& urdf_filename,
    const drake::systems::plants::joints::FloatingBaseType floating_base_type,
    RigidBodyTree* tree);

/**
 * Reads a single model from a URDF specification and adds a single instance of
 * it to @p tree. The model instance is connected to the world via a joint of
 * type `kRollPitchYaw` joint. The model instance's frame is equal
 * to the world's coordinate frame.
 *
 * @param[in] urdf_filename The name of the file containing a URDF
 * description of the model. An instance of this model will be added to
 * @p tree.
 *
 * @param[in] floating_base_type The type of joint that connects the model
 * instance's root to the @p tree.
 *
 * @param[in] weld_to_frame The frame to which to connect the new model
 * instance.
 *
 * @param[out] tree The `RigidBodyTree` to which to add the model instance.
 * This parameter must not be `nullptr`.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
DRAKERBM_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfFile(
    const std::string& urdf_filename,
    const drake::systems::plants::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree);

/**
 * Reads a single model from a URDF specification and adds a single instance of
 * it to @p tree.  The model instance is connected to the world via a joint of
 * type `kRollPitchYaw`. The model instance's frame is equal to the
 * world's coordinate frame.
 *
 * @param[in] urdf_filename The name of the file containing a URDF
 * description of the model. An instance of this model will be added to
 * @p tree.
 *
 * @param[in] ros_package_map A map of ROS package names to their paths. These
 * are the packages to search through when finding files referenced in the URDF.
 *
 * @param[in] floating_base_type The type of joint that connects the model
 * instance's root to the @p tree.
 *
 * @param[in] weld_to_frame The frame to which to connect the new model
 * instance.
 *
 * @param[out] tree The `RigidBodyTree` to which to add the model instance.
 * This parameter must not be `nullptr`.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
DRAKERBM_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfFile(
    const std::string& urdf_filename,
    std::map<std::string, std::string>& ros_package_map,
    const drake::systems::plants::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree);

}  // namespace urdf
}  // namespace parsers
}  // namespace drake
