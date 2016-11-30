#pragma once

#include <map>
#include <string>

#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parser_model_instance_id_table.h"

namespace drake {
namespace parsers {
namespace sdf {

/**
 * Adds the model or models defined within an SDF file to a rigid body tree.
 * One instance of each model is added. The models' frames are made coincident
 * with the world's coordinate frame.
 *
 * @param[in] filename The SDF file containing the model to be added.
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
ModelInstanceIdTable AddModelInstancesFromSdfFileInWorldFrame(
    const std::string& filename,
    const drake::multibody::joints::FloatingBaseType floating_base_type,
    RigidBodyTree<double>* tree);

/**
 * Adds the model or models defined within an SDF file to a rigid body tree.
 * One instance of each model is added.
 *
 * @param[in] filename The SDF file containing the model to be added.
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
ModelInstanceIdTable AddModelInstancesFromSdfFile(
    const std::string& filename,
    const drake::multibody::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree<double>* tree);

/**
 * Adds a SDF model to a rigid body system based on a string containing an SDF
 * description.
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
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree<double>* tree);

}  // namespace sdf
}  // namespace parsers
}  // namespace drake
