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

   Specifically, ensure that:
   - The geometry has sufficient, compatible hydro properties for contact.
     - The geometry has a defined compliance type.
     - All geometries will be given the requested default compliance type, but
       see
       `geometry::DefaultProximityProperties::compliance_type_rigid_fallback`
       for an exception to this rule.
     - All necessary properties are populated.

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
