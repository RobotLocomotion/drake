#include "drake/geometry/proximity_properties.h"

#include <cmath>
#include <limits>
#include <utility>

#include <fmt/format.h>

namespace drake {
namespace geometry {

const char* kMaterialGroup = "material";
const char* kElastic = "elastic_modulus";

const char* kHydroGroup = "hydroelastic";
const char* kRezHint = "resolution_hint";
const char* kPressure = "static_pressure";

PressureField MakeUnitPressureField() {
  return {[](double e) { return e; }, [](double) { return 1; }};
}

PressureField MakeLinearPressureField(double elastic_modulus) {
  if (elastic_modulus <= 0 || std::isinf(elastic_modulus)) {
    throw std::logic_error(
        fmt::format("A linear static pressure field requires a finite, "
                    "positive value; given {}",
                    elastic_modulus));
  }
  return {[E = elastic_modulus](double e){return E * e;
}
, [E = elastic_modulus](double) { return E; }
};
}

// Helper function to map PressureModel to PressureField instance.
PressureField MakePressureField(PressureModel pressure_model,
                                double elastic_modulus) {
  PressureField field;
  switch (pressure_model) {
    case PressureModel::kUnit:
      field = MakeUnitPressureField();
      break;
    case PressureModel::kLinear:
      field = MakeLinearPressureField(elastic_modulus);
      break;
    default:
      DRAKE_UNREACHABLE();
  }
  return field;
}

// Sets the static pressure field.
void AddStaticPressure(PressureField static_pressure,
                       ProximityProperties* properties) {
  properties->AddProperty(kHydroGroup, kPressure, static_pressure);
}

// Sets common proximity properties for hydroelastic geometry.
void AddTesselatedHydroelasticProperties(double resolution_hint,
                                         ProximityProperties* properties) {
  properties->AddProperty(kHydroGroup, kRezHint, resolution_hint);
}

void AddRigidHydroelasticProperties(double resolution_hint,
                                    ProximityProperties* properties) {
  AddTesselatedHydroelasticProperties(resolution_hint, properties);
}

void AddSoftHydroelasticProperties(double resolution_hint,
                                   PressureModel pressure_model,
                                   ProximityProperties* properties) {
  const double elastic_modulus =
      properties->GetProperty<double>(kMaterialGroup, kElastic);
  AddSoftHydroelasticProperties(
      resolution_hint, MakePressureField(pressure_model, elastic_modulus),
      properties);
}

void AddSoftHydroelasticProperties(double resolution_hint,
                                   PressureField static_pressure,
                                   ProximityProperties* properties) {
  AddTesselatedHydroelasticProperties(resolution_hint, properties);
  AddStaticPressure(static_pressure, properties);
}

}  // namespace geometry
}  // namespace drake
