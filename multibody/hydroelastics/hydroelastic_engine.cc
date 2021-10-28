#include "drake/multibody/hydroelastics/hydroelastic_engine.h"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/geometry/geometry_roles.h"

namespace drake {
namespace multibody {
namespace hydroelastics {
namespace internal {

using geometry::ContactSurface;
using geometry::GeometryId;
using geometry::ProximityProperties;
using geometry::SceneGraphInspector;

struct MaterialProperties {
  double hydroelastic_modulus{-1};
  double dissipation{-1};
};

template <typename T>
MaterialProperties GetMaterials(GeometryId id,
                                const SceneGraphInspector<T>& inspector) {
  const double kInf = std::numeric_limits<double>::infinity();
  MaterialProperties material;
  if (const ProximityProperties* properties =
          inspector.GetProximityProperties(id)) {
    material.hydroelastic_modulus = properties->GetPropertyOrDefault(
        geometry::internal::kHydroGroup, geometry::internal::kElastic, kInf);
    material.dissipation = properties->GetPropertyOrDefault(
        geometry::internal::kMaterialGroup, geometry::internal::kHcDissipation,
        0.0);
    DRAKE_DEMAND(material.hydroelastic_modulus > 0);
    DRAKE_DEMAND(material.dissipation >= 0);
  } else {
    throw std::runtime_error(fmt::format(
        "Unable to get the material properties for geometry {}; it has no "
        "proximity properties assigned",
        id));
  }
  return material;
}

template <typename T>
double HydroelasticEngine<T>::CalcCombinedElasticModulus(
    GeometryId id_A, GeometryId id_B,
    const SceneGraphInspector<T>& inspector) const {
  const double kInf = std::numeric_limits<double>::infinity();

  const MaterialProperties material_A = GetMaterials(id_A, inspector);
  const MaterialProperties material_B = GetMaterials(id_B, inspector);

  const double E_A = material_A.hydroelastic_modulus;
  const double E_B = material_B.hydroelastic_modulus;
  if (E_A == kInf) return E_B;
  if (E_B == kInf) return E_A;
  return E_A * E_B / (E_A + E_B);
}

// TODO(amcastro-tri): as of 09/18/2019 we still are discussing how to combine
// these material properties. Update this method once the discussion is
// resolved.
template <typename T>
double HydroelasticEngine<T>::CalcCombinedDissipation(
    GeometryId id_A, GeometryId id_B,
    const SceneGraphInspector<T>& inspector) const {
  const double kInf = std::numeric_limits<double>::infinity();
  const MaterialProperties material_A = GetMaterials(id_A, inspector);
  const MaterialProperties material_B = GetMaterials(id_B, inspector);

  const double E_A = material_A.hydroelastic_modulus;
  const double E_B = material_B.hydroelastic_modulus;
  const double d_A = material_A.dissipation;
  const double d_B = material_B.dissipation;
  const double Estar = CalcCombinedElasticModulus(id_A, id_B, inspector);

  // Both bodies are rigid. We simply return the arithmetic average.
  if (Estar == kInf) return 0.5 * (d_A + d_B);

  // At least one body is soft.
  double d_star = 0;
  if (E_A != kInf) d_star += Estar / E_A * d_A;
  if (E_B != kInf) d_star += Estar / E_B * d_B;
  return d_star;
}

}  // namespace internal
}  // namespace hydroelastics
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::hydroelastics::internal::HydroelasticEngine)
