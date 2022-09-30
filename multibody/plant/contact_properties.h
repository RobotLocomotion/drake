#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph_inspector.h"

namespace drake {
namespace multibody {
namespace internal {

/* Returns the point contact stiffness stored in group
 geometry::internal::kMaterialGroup with property
 geometry::internal::kPointStiffness for the specified geometry.
 If the stiffness property is absent, it returns the supplied default value.
 @pre id is a valid GeometryId in the inspector that has proximity properties.
 @pre default_value >= 0. */
template <typename T>
T GetPointContactStiffness(geometry::GeometryId id, double default_value,
                           const geometry::SceneGraphInspector<T>& inspector);

/* Returns the dissipation time constant stored in group
 geometry::internal::kMaterialGroup with property
 "dissipation_time_constant". If the property is absent, it returns the
 supplied default value.
 @throws std::exception if the dissipation time constant is negative.
 @pre id is a valid GeometryId in the inspector that has proximity properties.
 @pre default_value >= 0. */
template <typename T>
T GetDissipationTimeConstant(geometry::GeometryId id, double default_value,
                             const geometry::SceneGraphInspector<T>& inspector,
                             std::string_view body_name);

/* Returns the Coulomb's law coefficient of friction for the geometry with the
 given id stored by SceneGraph.
 @pre id is a valid GeometryId in the inspector that has friction properties. */
template <typename T>
const CoulombFriction<double>& GetCoulombFriction(
    geometry::GeometryId id, const geometry::SceneGraphInspector<T>& inspector);

/* Returns the combined stiffnesses k₁ (of geometry A) and k₂ (of geometry B)
 according to the rule:
   k  = k₁⋅k₂/(k₁+k₂)
 In other words, the combined compliance (the inverse of stiffness) is the
 sum of the individual compliances. k₁ and k₂ are set to the given default value
 if they are not specified in SceneGraph.
 @pre id_A and id_B are valid GeometryIds in the inspector that have proximity
 properties.
 @pre default_value >= 0. */
template <typename T>
T GetCombinedPointContactStiffness(
    geometry::GeometryId id_A, geometry::GeometryId id_B, double default_value,
    const geometry::SceneGraphInspector<T>& inspector);

/* Returns the combined dissipation time constant τ₁ (of geometry A) and τ₂ (of
 geometry B) according to the rule: τ = τ₁ + τ₂. τ₁ and τ₂ are set to the given
 default value if they are not specified in SceneGraph.
 @pre id_A and id_B are valid GeometryIds in the inspector that have proximity
 properties.
 @pre default_value >= 0. */
template <typename T>
T GetCombinedDissipationTimeConstant(
    geometry::GeometryId id_A, geometry::GeometryId id_B, double default_value,
    std::string_view body_A_name, std::string_view body_B_name,
    const geometry::SceneGraphInspector<T>& inspector);

/* Returns the dynamic Coulomb's law coefficients of friction characterizing the
 interaction by friction of the given geometry pair A and B.
 @pre id_A and id_B are both valid GeometryIds in the inspector and their
 corresponding geometries have the friction property. */
template <typename T>
double GetCombinedDynamicCoulombFriction(
    geometry::GeometryId id_A, geometry::GeometryId id_B,
    const geometry::SceneGraphInspector<T>& inspector);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
