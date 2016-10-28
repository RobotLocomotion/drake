#pragma once

#include <map>
#include <string>

#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_export.h"
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
DRAKE_EXPORT
std::shared_ptr<RigidBodyFrame> MakeRigidBodyFrameFromUrdfNode(
    const RigidBodyTree& tree, const tinyxml2::XMLElement& link,
    const tinyxml2::XMLElement* pose, const std::string& name,
    int model_instance_id);

/**
 * Reads a URDF model specified by @p urdf_string and adds an instance of it to
 * @p tree. Let the "base bodies" be the bodies in the model that do not have
 * parent joints. The base bodies are connected to the world via
 * systems::plants::joints::kRollPitchYaw joints. When these joints are at their
 * zero positions, the base body's frames are coincident with the world's
 * coordinate frame.
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
DRAKE_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfStringWithRpyJointToWorld(
    const std::string& urdf_string, RigidBodyTree* tree);

#ifndef SWIG
  DRAKE_DEPRECATED(
      "Please use AddModelInstanceFromUrdfStringWithRpyJointToWorld().")
#endif
DRAKE_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfString(
    const std::string& urdf_string, RigidBodyTree* tree);

/**
 * This method is the same as
 * AddModelInstanceFromUrdfStringWithRpyJointToWorld() except it has an
 * additional parameter called @p ros_package_map. Parameter @p ros_package_map
 * contains a mapping from ROS package names to their paths on the local file
 * system. The mapping is used to find resources like mesh files that are
 * referenced within the URDF. This method may be called from within the context
 * of a [ROS node](http://wiki.ros.org/Nodes) or a regular non-ROS application.
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
DRAKE_EXPORT ModelInstanceIdTable
AddModelInstanceFromUrdfStringWithRpyJointToWorldSearchingInRosPackages(
    const std::string& urdf_string,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::map<std::string, std::string>& ros_package_map, RigidBodyTree* tree);

#ifndef SWIG
DRAKE_DEPRECATED("Please use AddModelInstanceFromUrdfStringWithRpyJointToWorldSearchingInRosPackages().")  // NOLINT(whitespace/line_length)
#endif
DRAKE_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfString(
    const std::string& urdf_string,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::map<std::string, std::string>& ros_package_map, RigidBodyTree* tree);

/**
 * Reads a URDF model specified by @p urdf_string and adds an instance of it to
 * @p tree. Let the "base bodies" be the bodies in the model that do not have
 * parent joints.  This method connects the base bodies to an existing
 * body in @p tree via joints of type @p floating_base_type. The body in the
 * tree to which to which these joints attach and the transform between this
 * body and the base bodies' frames when the joints are in their zero positions
 * is determined by @p weld_to_frame.
 *
 * @param[in] urdf_string The URDF string of the model. This is the actual
 * URDF text (i.e., it is not the name of a file that contains the URDF text).
 * A new model instance is created based on this URDF text and added to
 * @p tree.
 *
 * @param[in] root_dir The root directory in which to search for files
 * mentioned in the URDF.
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
DRAKE_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfString(
    const std::string& urdf_string, const std::string& root_dir,
    const drake::systems::plants::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame, RigidBodyTree* tree);

/**
 * This method is the same as AddModelInstanceFromUrdfString() except it has an
 * additional parameter called @p ros_package_map. Parameter @p ros_package_map
 * contains a mapping from ROS package names to their paths on the local file
 * system. The mapping is used to find resources like mesh files that are
 * referenced within the URDF. This method may be called from within the context
 * of a [ROS node](http://wiki.ros.org/Nodes) or a regular non-ROS application.
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
DRAKE_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfStringSearchingInRosPackages(
    const std::string& urdf_string,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    PackageMap& ros_package_map,
    const std::string& root_dir,
    const drake::systems::plants::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame, RigidBodyTree* tree);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use AddModelInstanceFromUrdfStringSearchingInRosPackages().")  // NOLINT(whitespace/line_length)
#endif
DRAKE_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfString(
    const std::string& urdf_string,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    PackageMap& ros_package_map,
    const std::string& root_dir,
    const drake::systems::plants::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame, RigidBodyTree* tree);

/**
 * Reads a URDF model specified by @p urdf_filename and adds an instance of it
 * to @p tree. Let the "base bodies" be the bodies in the model that do not have
 * parent joints. This method connects the model instance's base bodies to the
 * world via systems::plants::joints::kRollPitchYaw joints. When
 * this joint is at its zero position, the base bodies' frames are coincident
 * with the world's coordinate frame.
 *
 * @param[in] urdf_filename The name of the file containing the URDF model.
 *
 * @param[out] tree The `RigidBodyTree` to which to add the model instance.
 * This parameter must not be `nullptr`.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to @p tree to their instance IDs, which are unique within @p tree.
 */
DRAKE_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfFileWithRpyJointToWorld(
    const std::string& urdf_filename, RigidBodyTree* tree);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use AddModelInstanceFromUrdfFileWithRpyJointToWorld().")  // NOLINT(whitespace/line_length)
#endif
DRAKE_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfFile(
     const std::string& urdf_filename, RigidBodyTree* tree);

/**
 * Reads a URDF model specified by @p urdf_filename and adds an instance of it
 * to @p tree. Let the "base bodies" be the bodies in the model that do not have
 * parent joints. This method connects the model instance's base bodies to the
 * world via joints of type @p floating_base_type. When these joints are at
 * their zero positions, the base bodies' frames are coincident with the
 * world's coordinate frame.
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
DRAKE_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfFileToWorld(
    const std::string& urdf_filename,
    const drake::systems::plants::joints::FloatingBaseType floating_base_type,
    RigidBodyTree* tree);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use AddModelInstanceFromUrdfFileToWorld().")
#endif
DRAKE_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfFile(
    const std::string& urdf_filename,
    const drake::systems::plants::joints::FloatingBaseType floating_base_type,
    RigidBodyTree* tree);

/**
 * Reads a URDF model specified by @p urdf_filename and adds an instance of it
 * to @p tree. Let the "base bodies" be the bodies in the model that do not have
 * parent joints. This method connects the base bodies to an existing body in
 * the tree using joints of type @p floating_base_type. The body in the tree to
 * which the base bodies are attached and the transform between this body and
 * the base bodies when the joints are in their zero positions is specified by
 * @p weld_to_frame.
 *
 * @param[in] urdf_filename The name of the file containing the URDF model. A
 * new instance of this model is created and added to @p tree.
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
DRAKE_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfFile(
    const std::string& urdf_filename,
    const drake::systems::plants::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree);

/**
 * This method is the same as AddModelInstanceFromUrdfFile() except it has an
 * additional parameter called @p ros_package_map. Parameter @p ros_package_map
 * contains a mapping from ROS package names to their paths on the local file
 * system. The mapping is used to find resources like mesh files that are
 * referenced within the URDF. This method may be called from within the context
 * of a [ROS node](http://wiki.ros.org/Nodes) or a regular non-ROS application.
 *
 * @param[in] urdf_filename The name of the file containing the URDF model.
 * An instance of this model will be added to @p tree.
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
DRAKE_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfFileSearchingInRosPackages(
    const std::string& urdf_filename,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::map<std::string, std::string>& ros_package_map,
    const drake::systems::plants::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use AddModelInstanceFromUrdfFileSearchingInRosPackages().")  // NOLINT(whitespace/line_length)
#endif
DRAKE_EXPORT
ModelInstanceIdTable AddModelInstanceFromUrdfFile(
     const std::string& urdf_filename,
     // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
     std::map<std::string, std::string>& ros_package_map,
     const drake::systems::plants::joints::FloatingBaseType floating_base_type,
     std::shared_ptr<RigidBodyFrame> weld_to_frame,
     RigidBodyTree* tree);
}  // namespace urdf
}  // namespace parsers
}  // namespace drake
