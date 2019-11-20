#include "drake/geometry/proximity/hydroelastic_internal.h"

#include <limits>
#include <vector>

#include <fmt/format.h>

#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {

using Eigen::Vector3d;
using std::make_unique;
using std::move;
using std::vector;

std::ostream& operator<<(std::ostream& out, const HydroelasticType& type) {
  switch (type) {
    case HydroelasticType::kUndefined:
      out << "undefined";
      break;
    case HydroelasticType::kRigid:
      out << "rigid";
      break;
    case HydroelasticType::kSoft:
      out << "soft";
      break;
    default:
      DRAKE_UNREACHABLE();
  }
  return out;
}

SoftGeometry& SoftGeometry::operator=(const SoftGeometry& g) {
  if (this == &g) return *this;

  auto mesh = make_unique<VolumeMesh<double>>(g.mesh());
  // We can't simply copy the mesh field; the copy must contain a pointer to the
  // new mesh. So, we use CloneAndSetMesh() instead.
  auto pressure = g.pressure_field().CloneAndSetMesh(mesh.get());
  geometry_ = SoftMesh{move(mesh), move(pressure)};

  return *this;
}

HydroelasticType Geometries::hydroelastic_type(GeometryId id) const {
  auto iter = supported_geometries_.find(id);
  if (iter != supported_geometries_.end()) return iter->second;
  return HydroelasticType::kUndefined;
}

void Geometries::RemoveGeometry(GeometryId id) {
  supported_geometries_.erase(id);
  soft_geometries_.erase(id);
  rigid_geometries_.erase(id);
}

void Geometries::MaybeAddGeometry(const Shape& shape, GeometryId id,
                                  const ProximityProperties& properties) {
  const HydroelasticType type = Classify(properties);
  if (type != HydroelasticType::kUndefined) {
    ReifyData data{type, id, properties};
    shape.Reify(this, &data);
  }
}

void Geometries::ImplementGeometry(const Sphere& sphere, void* user_data) {
  MakeShape(sphere, *reinterpret_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Cylinder& cylinder, void* user_data) {
  MakeShape(cylinder, *reinterpret_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const HalfSpace& half_space,
                                   void* user_data) {
  MakeShape(half_space, *reinterpret_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Box& box, void* user_data) {
  MakeShape(box, *reinterpret_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Capsule& capsule, void* user_data) {
  MakeShape(capsule, *reinterpret_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Ellipsoid& ellipsoid,
                                   void* user_data) {
  MakeShape(ellipsoid, *reinterpret_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Mesh& mesh, void* user_data) {
  MakeShape(mesh, *reinterpret_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Convex& convex, void* user_data) {
  MakeShape(convex, *reinterpret_cast<ReifyData*>(user_data));
}

template <typename ShapeType>
void Geometries::MakeShape(const ShapeType& shape, const ReifyData& data) {
  switch (data.type) {
    case HydroelasticType::kRigid: {
      auto hydro_geometry = MakeRigidRepresentation(shape, data.properties);
      if (hydro_geometry) AddGeometry(data.id, move(*hydro_geometry));
    } break;
    case HydroelasticType::kSoft: {
      auto hydro_geometry = MakeSoftRepresentation(shape, data.properties);
      if (hydro_geometry) AddGeometry(data.id, move(*hydro_geometry));
    } break;
    default:
      // kUndefined requires no action.
      break;
  }
}

void Geometries::AddGeometry(GeometryId id, SoftGeometry geometry) {
  DRAKE_DEMAND(hydroelastic_type(id) == HydroelasticType::kUndefined);
  supported_geometries_[id] = HydroelasticType::kSoft;
  soft_geometries_.insert({id, move(geometry)});
}

void Geometries::AddGeometry(GeometryId id, RigidGeometry geometry) {
  DRAKE_DEMAND(hydroelastic_type(id) == HydroelasticType::kUndefined);
  supported_geometries_[id] = HydroelasticType::kRigid;
  rigid_geometries_.insert({id, move(geometry)});
}

HydroelasticType Classify(const ProximityProperties& props) {
  // Presence of the hydroelastic group triggers an attempt to represent it.
  if (props.HasGroup(kHydroGroup)) {
    if (props.HasProperty(kMaterialGroup, kElastic)) {
      const double elastic_modulus =
          props.GetProperty<double>(kMaterialGroup, kElastic);
      if (std::isinf(elastic_modulus)) {
        return HydroelasticType::kRigid;
      } else if (elastic_modulus <= 0) {
        throw std::logic_error(
            fmt::format("Properties contain a bad value for the ('{}', '{}') "
                        "property: {}; the value must be positive",
                        kMaterialGroup, kElastic, elastic_modulus));
      } else {
        return HydroelasticType::kSoft;
      }
    } else {
      throw std::logic_error(
          fmt::format("Properties contain the '{}' group but is missing the "
                      "('{}', '{}') property; compliance cannot be determined",
                      kHydroGroup, kMaterialGroup, kElastic));
    }
  }
  return HydroelasticType::kUndefined;
}

// Validator interface for use with extracting valid properties. It is
// instantiated with shape (e.g., "Sphere", "Box", etc.) and compliance (i.e.,
// "rigid" or "soft") strings (to help give intelligible error messages) and
// then attempts to extract a typed value from a set of proximity properties --
// spewing meaningful error messages based on absence, type mis-match, and
// invalid values.
template <typename ValueType>
class Validator {
 public:
  Validator(const char* shape_name, const char* compliance)
      : shape_name_(shape_name), compliance_(compliance) {}

  virtual ~Validator() = default;

  // Extract an arbitrary property from the proximity properties. Throws a
  // consistent error message in the case of missing or mis-typed properties.
  // Relies on the ValidateValue() method to validate the value.
  const ValueType& Extract(const ProximityProperties& props,
                           const char* group_name, const char* property_name) {
    const std::string full_property_name =
        fmt::format("('{}', '{}')", group_name, property_name);
    if (!props.HasProperty(group_name, property_name)) {
      throw std::logic_error(
          fmt::format("Cannot create {} {}; missing the {} property",
                      compliance(), shape_name(), full_property_name));
    }
    const auto& value = props.GetProperty<ValueType>(group_name, property_name);
    ValidateValue(value, full_property_name);
    return value;
  }

  const char* shape_name() const { return shape_name_; }
  const char* compliance() const { return compliance_; }

 protected:
  // Does the work of validating the given value. Sub-classes should throw if
  // the provided value is not valid. The first parameter is the value to
  // validate; the second is the full name of the property.
  virtual void ValidateValue(const ValueType&, const std::string&) const {}

 private:
  const char* shape_name_{};
  const char* compliance_{};
};

// Validator that extracts *positive doubles*.
class PositiveDouble : public Validator<double> {
 public:
  PositiveDouble(const char* shape_name, const char* compliance)
      : Validator<double>(shape_name, compliance) {}

 protected:
  void ValidateValue(const double& value,
                     const std::string& property) const override {
    if (value <= 0) {
      throw std::logic_error(
          fmt::format("Cannot create {} {}; the {} property must be positive",
                      compliance(), shape_name(), property));
    }
  }
};

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Sphere& sphere, const ProximityProperties& props) {
  const double edge_length =
      PositiveDouble("Sphere", "rigid").Extract(props, kHydroGroup, kRezHint);
  SurfaceMesh<double> mesh = MakeSphereSurfaceMesh<double>(sphere, edge_length);

  return RigidGeometry(move(mesh));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Box& box, const ProximityProperties& props) {
  const double edge_length =
      PositiveDouble("Box", "rigid").Extract(props, kHydroGroup, kRezHint);
  SurfaceMesh<double> mesh = MakeBoxSurfaceMesh<double>(box, edge_length);

  return RigidGeometry(move(mesh));
}

std::optional<SoftGeometry> MakeSoftRepresentation(
    const Sphere& sphere, const ProximityProperties& props) {
  // First, create the mesh.
  const double edge_length =
      PositiveDouble("Sphere", "soft").Extract(props, kHydroGroup, kRezHint);
  auto mesh = make_unique<VolumeMesh<double>>(
      MakeSphereVolumeMesh<double>(sphere, edge_length));

  // E is the elastic modulus.
  const double E =
      PositiveDouble("Sphere", "soft").Extract(props, kMaterialGroup, kElastic);

  // Second, compute the penetration extent and gradient fields.
  const double r = sphere.radius();
  std::vector<double> p_values;
  p_values.reserve(mesh->num_vertices());
  for (const auto& v : mesh->vertices()) {
    const Vector3d& p_MV = v.r_MV();
    const double p_MV_len = p_MV.norm();
    const double extent = 1.0 - p_MV_len / r;
    p_values.push_back(E * extent);
  }
  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      "pressure", move(p_values), mesh.get());

  return SoftGeometry(move(mesh), move(pressure));
}

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
