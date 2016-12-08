#pragma once

#include <string>

#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parser_model_instance_id_table.h"
#include "drake/multibody/parser_common.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace parsers {
namespace sdf {

/**
 * Adds the model or models defined within an SDF file to a rigid body tree.
 * One instance of each model is added. The models' frames are made coincident
 * with the world's coordinate frame.
 *
 * This method can only be used with SDF models that do not use `package://` to
 * reference modeling resources like mesh files. SDF models that contain
 * `package://` should instead use
 * AddModelInstancesFromSdfFileInWorldFrameSearchingInRosPackages().
 *
 * @param[in] filename The name of the SDF file containing the model to be
 * added.
 *
 * @param[in] floating_base_type The type of joint that connects the model's
 * root to the existing rigid body tree.
 *
 * @param[out] tree The rigid body tree to which to add the model.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
ModelInstanceIdTable
AddModelInstancesFromSdfFileInWorldFrame(
    const std::string& filename,
    const drake::multibody::joints::FloatingBaseType floating_base_type,
    RigidBodyTree<double>* tree);

/**
 * Adds the model or models defined within an SDF file to a rigid body tree.
 * One instance of each model is added. The models' frames are made coincident
 * with the world's coordinate frame.
 *
 * This method has input parameter @p package_map. This parameter is only
 * necessary if the SDF contains models that reference meshes and other modeling
 * resources using `package://`. If the models in the SDF do not use
 * `package://`, the SDF could instead be loaded using
 * AddModelInstancesFromSdfFileInWorldFrame().
 *
 * @param[in] filename The name of the SDF file containing the model to be
 * added.
 *
 * @param[in] package_map A map of ROS package names to their paths. These are
 * the packages to search through when finding files referenced in the SDF.
 *
 * @param[in] floating_base_type The type of joint that connects the model's
 * root to the existing rigid body tree.
 *
 * @param[out] tree The rigid body tree to which to add the model.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
ModelInstanceIdTable
AddModelInstancesFromSdfFileInWorldFrameSearchingInRosPackages(
    const std::string& filename, const PackageMap& package_map,
    const drake::multibody::joints::FloatingBaseType floating_base_type,
    RigidBodyTree<double>* tree);

/**
 * Adds the model or models defined within an SDF file to a rigid body tree.
 * One instance of each model is added.
 *
 * This method can only be used with SDF models that do not use `package://` to
 * reference modeling resources like mesh files. SDF models that contain
 * `package://` should instead use
 * AddModelInstancesFromSdfFileSearchingInRosPackages().
 *
 * @param[in] filename The name of the SDF file containing the model to be
 * added.
 *
 * @param[in] floating_base_type The type of joint that connects the model's
 * root to the existing rigid body tree.
 *
 * @param[in] weld_to_frame The frame to which to connect the new model.
 *
 * @param[out] tree The rigid body tree to which to add the model.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
ModelInstanceIdTable AddModelInstancesFromSdfFile(
    const std::string& filename,
    const drake::multibody::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    RigidBodyTree<double>* tree);

/**
 * Adds the model or models defined within an SDF file to @p tree. One instance
 * of each model is added.
 *
 * This method has input parameter @p package_map. This parameter is only
 * necessary if the SDF contains models that reference meshes and other modeling
 * resources using `package://`. If the models in the SDF do not use
 * `package://`, the SDF could instead be loaded using
 * AddModelInstancesFromSdfFile().
 *
 * @param[in] filename The name of the SDF file containing the model to be
 * added.
 *
 * @param[in] package_map A map of ROS package names to their paths. These are
 * the packages to search through when finding files referenced in the SDF.
 *
 * @param[in] floating_base_type The type of joint that connects the model's
 * root to the existing rigid body tree.
 *
 * @param[in] weld_to_frame The frame to which to connect the new model.
 *
 * @param[out] tree The rigid body tree to which to add the model.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
ModelInstanceIdTable AddModelInstancesFromSdfFileSearchingInRosPackages(
    const std::string& filename, const PackageMap& package_map,
    const drake::multibody::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    RigidBodyTree<double>* tree);


/**
 * Adds the model or models defined within an SDF description to @p tree. One
 * instance of each model is added.
 *
 * This method can only be used with SDF models that do not use `package://` to
 * reference modeling resources like mesh files. SDF models that contain
 * `package://` should instead use
 * AddModelInstancesFromSdfStringSearchingInRosPackages().
 *
 * @param[in] sdf_string The SDF description of one or more models.
 *
 * @param[in] floating_base_type The type of joint that connects the model's
 * root to the existing rigid body tree.
 *
 * @param[in] weld_to_frame The frame to which to connect the new model.
 *
 * @param[out] tree The rigid body tree to which to add the model.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
ModelInstanceIdTable AddModelInstancesFromSdfString(
    const std::string& sdf_string,
    const drake::multibody::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    RigidBodyTree<double>* tree);

/**
 * Adds the model or models defined within an SDF description to @p tree. One
 * instance of each model is added.
 *
 * This method has input parameter @p package_map. This parameter is only
 * necessary if the SDF contains models that reference meshes and other modeling
 * resources using `package://`. If the models in the SDF do not use
 * `package://`, the SDF could instead be loaded using
 * AddModelInstancesFromSdfFile().
 *
 * @param[in] sdf_string The SDF description of one or more models.
 *
 * @param[in] package_map A map of ROS package names to their paths. These are
 * the packages to search through when finding files referenced in the SDF.
 *
 * @param[in] floating_base_type The type of joint that connects the model's
 * root to the existing rigid body tree.
 *
 * @param[in] weld_to_frame The frame to which to connect the new model.
 *
 * @param[out] tree The rigid body tree to which to add the model.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
ModelInstanceIdTable AddModelInstancesFromSdfStringSearchingInRosPackages(
    const std::string& sdf_string, const PackageMap& package_map,
    const drake::multibody::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    RigidBodyTree<double>* tree);

}  // namespace sdf
}  // namespace parsers
}  // namespace drake
