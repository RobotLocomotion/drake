#pragma once

#include <utility>

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

/* Returns the hydroelastic modulus stored in group
 geometry::internal::kHydroGroup with property geometry::internal::kElastic for
 the specified geometry.

 This method returns infinity for geometries with value
 geometry::internal::HydroelasticType::kRigid for the property
 geometry::internal::kComplianceType.

 If not defined rigid and if the hydroelastic modulus is absent, it returns the
 supplied default value.

 @pre id is a valid GeometryId in the inspector that has proximity properties.
 @pre default_value >= 0. */
template <typename T>
T GetHydroelasticModulus(geometry::GeometryId id, double default_value,
                         const geometry::SceneGraphInspector<T>& inspector);

/* Returns the Hunt & Crossley dissipation parameter stored in group
 geometry::internal::kMaterialGroup with property
 geometry::internal::kHcDissipation for the specified geometry.
 If the dissipation property is absent, it returns the supplied default value.
 @pre id is a valid GeometryId in the inspector that has proximity properties.
 @pre default_value >= 0. */
template <typename T>
T GetHuntCrossleyDissipation(geometry::GeometryId id, double default_value,
                             const geometry::SceneGraphInspector<T>& inspector);

/* Returns the combined Hunt & Crossley dissipation of geometries A and B.
 Denoting with k₁ and k₂ the `stiffness_A` and `stiffness_B` respectively, and
 with d₁ and d₂ the Hunt & Crossley dissipation coefficients for geometries A
 and B respectively, this method computes the combined dissipation according to
 the rule:
   d = k₂/(k₁+k₂)⋅d₁ + k₁/(k₁+k₂)⋅d₂
 This method requires both values of stiffness coefficients to be positive
 or zero, and it allows one, but only one value, to be infinite. That
 is, one of the geometries can be rigid but not both. For such case, the
 combined dissipation corresponds to the mathematical limit in the expression
 above, i.e., it is the dissipation of the non-rigid geometry.
 If both values of stiffness are zero, zero dissipation is returned.

 @pre stiffness_A >= 0. It can have the value infinity, indicating a rigid
 geometry A.
 @pre stiffness_B >= 0. It can have the value infinity, indicating a rigid
 geometry B.
 @pre At least one of A or B is non-rigid.
 @pre default_dissipation >= 0. Default value of dissipation for both geometries
 in case no value was assigned in the proximity properties. */
template <typename T>
T GetCombinedHuntCrossleyDissipation(
    geometry::GeometryId id_A, geometry::GeometryId id_B, const T& stiffness_A,
    const T& stiffness_B, double default_dissipation,
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

/* Returns the combined stiffnesses k₁ and k₂ according to the rule:
   k = k₁⋅k₂/(k₁+k₂)
 In other words, the combined compliance (the inverse of stiffness) is the
 sum of the individual compliances. In particular, returns k₁ if k₂ is infinite
 and returns k₂ if k₁ is infinite.
 @pre k₁ and k₂ are not both infinite.
 @pre k₁ and k₂ are both non-negative. */
template <typename T>
T GetCombinedPointContactStiffness(const T& k1, const T& k2);

/* Returns the combined stiffnesses k₁ (of geometry A) and k₂ (of geometry B)
 according to the rule:
   k = k₁⋅k₂/(k₁+k₂)
 In other words, the combined compliance (the inverse of stiffness) is the
 sum of the individual compliances. In particular, returns k₁ if k₂ is infinite
 and returns k₂ if k₁ is infinite. k₁ and k₂ are set to the given default values
 if they are not specified in SceneGraph.
 @pre id_A and id_B are valid GeometryIds in the inspector that have proximity
 properties. k₁ and k₂ are not both infinite.
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
