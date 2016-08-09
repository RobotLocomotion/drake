#pragma once

#include <map>
#include <string>

#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/xmlUtil.h"
#include "drake/thirdParty/zlib/tinyxml2/tinyxml2.h"

namespace drake {
namespace parsers {
namespace sdf {

/// Adds a SDF model to a rigid body system. The model's frame is equal to the
/// world's coordinate frame.
///
/// @param[in] urdf_filename The URDF file containing the model to be added.
///
/// @param[in] package_map A map of ROS package names to their paths. These are
/// the packages to search through when finding files referenced in the URDF.
///
/// @param[in] floating_base_type The type of joint that connects the model's
/// root to the existing rigid body tree.
///
/// @param[out] tree The rigid body tree to which to add the model.
///
/// @param[out] model_instance_id_table A pointer to a map storing model
/// names and their instance IDs. This parameter is may be `nullptr`. A
/// `std::runtime_error` is thrown if an instance is created of a model whose
/// name is already in this table (assuming the table is not `nullptr`).
DRAKERBM_EXPORT
void AddRobotFromSDFInWorldFrame(
    const std::string& sdf_filename,
    const DrakeJoint::FloatingBaseType floating_base_type,
    RigidBodyTree* tree,
    ModelInstanceIdTable* model_instance_id_table = nullptr);

/// Adds a SDF model to a rigid body system.
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
///
/// @param[out] model_instance_id_table A pointer to a map storing model
/// names and their instance IDs. This parameter is may be `nullptr`. A
/// `std::runtime_error` is thrown if an instance is created of a model whose
/// name is already in this table (assuming the table is not `nullptr`).
DRAKERBM_EXPORT
void AddRobotFromSDF(
    const std::string& sdf_filename,
    const DrakeJoint::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    RigidBodyTree* tree,
    ModelInstanceIdTable* model_instance_id_table = nullptr);

}  // namespace sdf
}  // namespace parsers
}  // namespace drake
