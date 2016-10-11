#pragma once

#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace automotive {

/**
 * Adds a box-shaped terrain to the specified rigid body tree. This directly
 * modifies the existing world rigid body within @p rigid_body_tree and thus
 * does not need to return a model name or model_instance_id value.
 *
 * The X, Y, and Z axes of the box matches the X, Y, and Z-axis of the world.
 * The length and width of the box is aligned with X and Y and are @p box_size
 * long. The depth of the box is aligned with Z and is @p box_depth long. The
 * top surface of the box is at Z = 0.
 *
 * @param[in] tree The rigid body tree to which to add the terrain.
 *
 * @param[in] box_size The length and width of the terrain aligned with the
 * world's X and Y axes.
 *
 * @param[in] box_depth The depth of the terrain aligned with the world's Z
 * axis. Note that regardless of how deep the terrain is, the top surface of the
 * terrain will be at Z = 0.
 */
DRAKE_EXPORT
void AddFlatTerrainToWorld(RigidBodyTree* tree,
                           double box_size = 1000, double box_depth = 10);

}  // namespace automotive
}  // namespace drake
