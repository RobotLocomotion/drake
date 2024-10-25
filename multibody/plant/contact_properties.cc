#include "drake/multibody/plant/contact_properties.h"

#include <limits>
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
T GetPointContactStiffness(geometry::GeometryId id,
                           const geometry::SceneGraphInspector<T>& inspector,
                           std::optional<double> default_value) {
  const geometry::ProximityProperties* prop =
      inspector.GetProximityProperties(id);
  DRAKE_DEMAND(prop != nullptr);
  if (default_value) {
    DRAKE_DEMAND(*default_value >= 0.0);
    return prop->template GetPropertyOrDefault<double>(
        geometry::internal::kMaterialGroup, geometry::internal::kPointStiffness,
        *default_value);
  } else {
    if (!prop->HasProperty(geometry::internal::kMaterialGroup,
                           geometry::internal::kPointStiffness)) {
      throw std::logic_error(fmt::format(
          "Point stiffness missing for geometry: {}. ", inspector.GetName(id)));
    }
    return prop->template GetProperty<double>(
        geometry::internal::kMaterialGroup,
        geometry::internal::kPointStiffness);
  }
}

template <typename T>
T GetHydroelasticModulus(geometry::GeometryId id,
                         const geometry::SceneGraphInspector<T>& inspector,
                         std::optional<double> default_value) {
  const geometry::ProximityProperties* prop =
      inspector.GetProximityProperties(id);
  DRAKE_DEMAND(prop != nullptr);
  // If the geometry is defined to be rigid, we return infinity.
  if (prop->GetPropertyOrDefault(
          geometry::internal::kHydroGroup, geometry::internal::kComplianceType,
          geometry::internal::HydroelasticType::kUndefined) ==
      geometry::internal::HydroelasticType::kRigid) {
    return std::numeric_limits<double>::infinity();
  }

  if (default_value) {
    DRAKE_DEMAND(*default_value >= 0.0);
    return prop->template GetPropertyOrDefault<double>(
        geometry::internal::kHydroGroup, geometry::internal::kElastic,
        *default_value);
  } else {
    if (!prop->HasProperty(geometry::internal::kHydroGroup,
                           geometry::internal::kElastic)) {
      throw std::logic_error(
          fmt::format("Hydroelastic modulus missing for geometry: {}. ",
                      inspector.GetName(id)));
    }
    return prop->template GetProperty<double>(geometry::internal::kHydroGroup,
                                              geometry::internal::kElastic);
  }
}

template <typename T>
T GetHuntCrossleyDissipation(geometry::GeometryId id,
                             const geometry::SceneGraphInspector<T>& inspector,
                             std::optional<double> default_value) {
  const geometry::ProximityProperties* prop =
      inspector.GetProximityProperties(id);
  DRAKE_DEMAND(prop != nullptr);
  if (default_value) {
    DRAKE_DEMAND(*default_value >= 0.0);
    return prop->template GetPropertyOrDefault<double>(
        geometry::internal::kMaterialGroup, geometry::internal::kHcDissipation,
        *default_value);
  } else {
    if (!prop->HasProperty(geometry::internal::kMaterialGroup,
                           geometry::internal::kHcDissipation)) {
      throw std::logic_error(
          fmt::format("Hunt & Crossley dissipation missing for geometry: {}. ",
                      inspector.GetName(id)));
    }
    return prop->template GetProperty<double>(
        geometry::internal::kMaterialGroup, geometry::internal::kHcDissipation);
  }
}

template <typename T>
T GetCombinedHuntCrossleyDissipation(
    geometry::GeometryId id_A, geometry::GeometryId id_B, const T& stiffness_A,
    const T& stiffness_B, const geometry::SceneGraphInspector<T>& inspector,
    std::optional<double> default_dissipation) {
  const double kInf = std::numeric_limits<double>::infinity();
  // Demand that at least one is compliant.
  DRAKE_DEMAND(stiffness_A != kInf || stiffness_B != kInf);
  DRAKE_DEMAND(stiffness_A >= 0.0);
  DRAKE_DEMAND(stiffness_B >= 0.0);
  DRAKE_DEMAND(*default_dissipation >= 0.0);

  // Return zero dissipation if both stiffness values are zero.
  const T denom = stiffness_A + stiffness_B;
  if (denom == 0.0) return 0.0;

  // If only one object is compliant, return the dissipation for that object.
  // N.B. Per the demand above we know at least one is compliant.
  const T dB = GetHuntCrossleyDissipation(id_B, inspector, default_dissipation);
  if (stiffness_A == kInf) return dB;
  const T dA = GetHuntCrossleyDissipation(id_A, inspector, default_dissipation);
  if (stiffness_B == kInf) return dA;

  // At this point we know both geometries are compliant and at least one of
  // them has non-zero stiffness (denom != 0).
  return (stiffness_B / denom) * dA + (stiffness_A / denom) * dB;
}

template <typename T>
T GetDissipationTimeConstant(geometry::GeometryId id,
                             const geometry::SceneGraphInspector<T>& inspector,
                             std::string_view body_name,
                             std::optional<double> default_value) {
  const geometry::ProximityProperties* prop =
      inspector.GetProximityProperties(id);
  DRAKE_DEMAND(prop != nullptr);
  auto throw_if_negative = [&](double value) {
    if (value < 0.0) {
      const std::string message = fmt::format(
          "Relaxation time must be non-negative and relaxation_time = {} was "
          "provided. For geometry {} on body {}.",
          value, inspector.GetName(id), body_name);
      throw std::runtime_error(message);
    }
  };

  if (default_value) {
    DRAKE_DEMAND(*default_value >= 0.0);
    const double relaxation_time = prop->template GetPropertyOrDefault<double>(
        geometry::internal::kMaterialGroup, geometry::internal::kRelaxationTime,
        *default_value);
    throw_if_negative(relaxation_time);
    return relaxation_time;
  } else {
    if (!prop->HasProperty(geometry::internal::kMaterialGroup,
                           geometry::internal::kRelaxationTime)) {
      throw std::logic_error(
          fmt::format("Relaxation time missing for geometry {} on body {}.",
                      inspector.GetName(id), body_name));
    }
    const double relaxation_time =
        prop->template GetProperty<double>(geometry::internal::kMaterialGroup,
                                           geometry::internal::kRelaxationTime);
    throw_if_negative(relaxation_time);
    return relaxation_time;
  }
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
T GetCombinedPointContactStiffness(const T& k1, const T& k2) {
  const T kInf = std::numeric_limits<T>::infinity();
  // Demand that at least one is compliant.
  DRAKE_DEMAND(k1 != kInf || k2 != kInf);
  DRAKE_DEMAND(k1 >= 0.0);
  DRAKE_DEMAND(k2 >= 0.0);
  if (k1 == kInf) return k2;
  if (k2 == kInf) return k1;
  return safe_divide(k1 * k2, k1 + k2);
}

template <typename T>
T GetCombinedPointContactStiffness(
    geometry::GeometryId id_A, geometry::GeometryId id_B,
    const geometry::SceneGraphInspector<T>& inspector,
    std::optional<double> default_value) {
  const T k1 = GetPointContactStiffness(id_A, inspector, default_value);
  const T k2 = GetPointContactStiffness(id_B, inspector, default_value);
  return GetCombinedPointContactStiffness(k1, k2);
}

template <typename T>
T GetCombinedDissipationTimeConstant(
    geometry::GeometryId id_A, geometry::GeometryId id_B,
    std::string_view body_A_name, std::string_view body_B_name,
    const geometry::SceneGraphInspector<T>& inspector,
    std::optional<double> default_value) {
  return GetDissipationTimeConstant(id_A, inspector, body_A_name,
                                    default_value) +
         GetDissipationTimeConstant(id_B, inspector, body_B_name,
                                    default_value);
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

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&GetPointContactStiffness<T>, &GetHydroelasticModulus<T>,
     &GetHuntCrossleyDissipation<T>, &GetCombinedHuntCrossleyDissipation<T>,
     &GetDissipationTimeConstant<T>, &GetCoulombFriction<T>,
     static_cast<T (*)(geometry::GeometryId, geometry::GeometryId,
                       const geometry::SceneGraphInspector<T>&,
                       std::optional<double>)>(
         &GetCombinedPointContactStiffness<T>),
     static_cast<T (*)(const T&, const T&)>(
         &GetCombinedPointContactStiffness<T>),
     &GetCombinedDissipationTimeConstant<T>,
     &GetCombinedDynamicCoulombFriction<T>));

}  // namespace internal
}  // namespace multibody
}  // namespace drake
