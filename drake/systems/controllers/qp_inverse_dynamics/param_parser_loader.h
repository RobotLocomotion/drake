#pragma once

#include <memory>
#include <string>

#include "drake/multibody/rigid_body_tree_alias_groups.h"
#include "drake/systems/controllers/qp_inverse_dynamics/param_parser.h"

namespace drake {
namespace systems {
namespace controllers {
namespace qp_inverse_dynamics {

/**
 * Loads parameters from a config file for the inverse dynamics controller.
 * The format of the config file is defined in id_controller_config.proto.
 *
 * For the `ContactConfig` and `AccelerationConfig` (defined in
 * id_controller_config.proto), the `name` field should correspond to either
 * a body group or joint group name in the associated
 * RigidBodyTreeAliasGroups. Every member in that group will have the same
 * parameters. A `default` parameter can also be specified for `body_motion`,
 * `dof_motion` and `contact` for InverseDynamicsControllerConfig. It will be
 * returned when parameters are not explicitly specified. It is recommended
 * to supply the default parameters.
 *
 * For AccelerationConfig, the number of recurrence for `kp`, `kd` and
 * `weight` can either be 1 or exactly matches the dimension of the
 * associated parameter. The first case is a simpler way for specifying
 * everything with the same number. For example, when specifying parameters
 * for spatial accelerations, `kp`, `kd` and `weight` need to be 6
 * dimensional. When used for accelerations in the generalized coordinates,
 * `kp`, `kd` and `weight` need to match the dimension of that joint group.
 * E.g. `kp`, `kd` and `weight` need to be 8 dimensional for a joint group
 * that consists of a floating base joint and 2 single dof joints.
 *
 * @param config_path Path to the config file.
 * @param alias_groups Specifies the relationship between body / joint groups
 * and the RigidBodyTree it is constructed from.
 *
 * @throws std::runtime_error if the config file can not be parsed correctly.
 */
std::unique_ptr<ParamSet> ParamSetLoadFromFile(
    const std::string& config_path,
    const RigidBodyTreeAliasGroups<double>& alias_groups);

}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
