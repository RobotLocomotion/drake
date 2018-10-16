#pragma once

#include <map>
#include <string>

namespace drake {
namespace parsers {

/**
 * Defines a data type that maps model names to their instance IDs within the
 * `RigidBodyTree`. The model names are defined in a single URDF or SDF file
 * and are thus guaranteed to be unique within an instance of this data type.
 * The instance IDs are determined by by the `RigidBodyTree` and are unique
 * among all model instances within the tree. This data type is used to inform
 * applications of the IDs that were assigned to model instances as they were
 * added to a `RigidBodyTree` while parsing a URDF or SDF description.
 *
 * The model names within this data type are specified by the URDF and SDF.
 * They are @a not the same as "model instance names" since multiple instances
 * of the same model may be added to the same `RigidBodyTree`. Model instance
 * names can be decided by the application based on the information contained
 * within this data type. It is recommended, but not required, that
 * applications separately create mappings from model instance IDs to
 * meaningful model instance names. This is because an instance ID, as an
 * integer, does not convey much information about the model instance.
 */
typedef std::map<std::string, int> ModelInstanceIdTable;

/**
 * Adds the model instances in @p source_table to @p dest_table.
 * @throws std::runtime_error if there is a collision in the model names.
 */
void AddModelInstancesToTable(
    const drake::parsers::ModelInstanceIdTable& source_table,
    drake::parsers::ModelInstanceIdTable* dest_table);

}  // namespace parsers
}  // namespace drake
