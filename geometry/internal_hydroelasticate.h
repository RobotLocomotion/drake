#pragma once

#include "drake/geometry/geometry_state.h"
#include "drake/geometry/scene_graph_config.h"

namespace drake {
namespace geometry {
namespace internal {

/* Iterates over all of the proximity geometries of a scene graph, ensuring
   that all proximity geometries have hydroelastic properties. For detailed
   semantics, see the 3-argument overload.

   @pre geometry_state is not nullptr.
   @tparam_default_scalar
 */
template <typename T>
void Hydroelasticate(GeometryState<T>* geometry_state,
                   const SceneGraphConfig& config);

/* Ensures that one proximity geometry has hydroelastic properties.

   Specifically, ensure that either:
   - The geometry has sufficient, compatible hydro properties for contact.
     - The geometry has a defined compliance type.
       - All geometries will be given the requeested default compliance type,
         except:
       - Currently, there is no practical way to make an non-convex surface
         mesh support compliant(soft) operation, so those will be marked rigid,
         regardless of the requested default compliance type.
     - All necessary properties are populated.
   or:
   - The geometry's proximity role is removed.
     - This happens if the geometry is found to be smaller than household dust.
       - Historically, such shapes were added to help stabilize point contact.
     - Only primitive shapes are tested.
       - Meshes are assumed to be large -- otherwise why go the trouble?
       - Half-spaces are infinitely large.

   @pre geometry_state is not nullptr.
   @pre geometry_id indicates a geometry with an assigned proximity role.
   @tparam_default_scalar
 */
template <typename T>
void Hydroelasticate(GeometryState<T>* geometry_state,
                     const SceneGraphConfig& config, GeometryId geometry_id);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
