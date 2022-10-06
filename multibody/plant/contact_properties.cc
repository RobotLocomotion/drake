#include "drake/multibody/plant/contact_properties.h"

#include <string>

namespace drake {
namespace multibody {
namespace internal {
namespace {

/* Helper divide function that detects 0 / 0. Returns zero if the denominator is
 zero. */
template <typename T>
T safe_divide(const T& num, const T& denom) {
  return denom == 0.0 ? 0.0 : num / denom;
}

}  // namespace

template <typename T>
T GetPointContactStiffness(geometry::GeometryId id, double default_value,
                           const geometry::SceneGraphInspector<T>& inspector) {
  DRAKE_DEMAND(default_value >= 0.0);
  const geometry::ProximityProperties* prop =
      inspector.GetProximityProperties(id);
  DRAKE_DEMAND(prop != nullptr);
  return prop->template GetPropertyOrDefault<T>(
      geometry::internal::kMaterialGroup, geometry::internal::kPointStiffness,
      default_value);
}

template <typename T>
T GetDissipationTimeConstant(geometry::GeometryId id, double default_value,
                             const geometry::SceneGraphInspector<T>& inspector,
                             std::string_view body_name) {
  DRAKE_DEMAND(default_value >= 0.0);
  const geometry::ProximityProperties* prop =
      inspector.GetProximityProperties(id);
  DRAKE_DEMAND(prop != nullptr);

  auto provide_context_string =
      [&inspector,
       &body_name](geometry::GeometryId geometry_id) -> std::string {
    return fmt::format("For geometry {} on body {}.",
                       inspector.GetName(geometry_id), body_name);
  };
  const T relaxation_time = prop->template GetPropertyOrDefault<double>(
      geometry::internal::kMaterialGroup, "relaxation_time", default_value);
  if (relaxation_time < 0.0) {
    const std::string message = fmt::format(
        "Relaxation time must be non-negative and relaxation_time "
        "= {} was provided. {}",
        relaxation_time, provide_context_string(id));
    throw std::runtime_error(message);
  }
  return relaxation_time;
}

template <typename T>
const CoulombFriction<double>& GetCoulombFriction(
    geometry::GeometryId id,
    const geometry::SceneGraphInspector<T>& inspector) {
  const geometry::ProximityProperties* prop =
      inspector.GetProximityProperties(id);
  DRAKE_DEMAND(prop != nullptr);
  DRAKE_THROW_UNLESS(prop->HasProperty(geometry::internal::kMaterialGroup,
                                       geometry::internal::kFriction));
  return prop->GetProperty<CoulombFriction<double>>(
      geometry::internal::kMaterialGroup, geometry::internal::kFriction);
}

template <typename T>
T GetCombinedPointContactStiffness(
    geometry::GeometryId id_A, geometry::GeometryId id_B, double default_value,
    const geometry::SceneGraphInspector<T>& inspector) {
  const T k1 = GetPointContactStiffness(id_A, default_value, inspector);
  const T k2 = GetPointContactStiffness(id_B, default_value, inspector);
  return safe_divide(k1 * k2, k1 + k2);
}

template <typename T>
T GetCombinedDissipationTimeConstant(
    geometry::GeometryId id_A, geometry::GeometryId id_B, double default_value,
    std::string_view body_A_name, std::string_view body_B_name,
    const geometry::SceneGraphInspector<T>& inspector) {
  return GetDissipationTimeConstant(id_A, default_value, inspector,
                                    body_A_name) +
         GetDissipationTimeConstant(id_B, default_value, inspector,
                                    body_B_name);
}

template <typename T>
double GetCombinedDynamicCoulombFriction(
    geometry::GeometryId id_A, geometry::GeometryId id_B,
    const geometry::SceneGraphInspector<T>& inspector) {
  const auto& mu_A = GetCoulombFriction(id_A, inspector);
  const auto& mu_B = GetCoulombFriction(id_B, inspector);
  return CalcContactFrictionFromSurfaceProperties(mu_A, mu_B)
      .dynamic_friction();
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&GetPointContactStiffness<T>, &GetDissipationTimeConstant<T>,
     &GetCoulombFriction<T>, &GetCombinedPointContactStiffness<T>,
     &GetCombinedDissipationTimeConstant<T>,
     &GetCombinedDynamicCoulombFriction<T>))

}  // namespace internal
}  // namespace multibody
}  // namespace drake
