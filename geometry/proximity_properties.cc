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

// NOTE: Although these functions currently do the same thing, we're leaving
// the two functions in place to facilitate future differences.

void AddRigidHydroelasticProperties(double resolution_hint,
                                    ProximityProperties* properties) {
  properties->AddProperty(kHydroGroup, kRezHint, resolution_hint);
}

void AddSoftHydroelasticProperties(double resolution_hint,
                                   ProximityProperties* properties) {
  properties->AddProperty(kHydroGroup, kRezHint, resolution_hint);
}

}  // namespace geometry
}  // namespace drake
