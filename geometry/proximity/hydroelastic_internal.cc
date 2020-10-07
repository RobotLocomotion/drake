#include "drake/geometry/proximity/hydroelastic_internal.h"

#include <string>

#include <fmt/format.h>

#include "drake/geometry/proximity/make_box_field.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_cylinder_field.h"
#include "drake/geometry/proximity/make_cylinder_mesh.h"
#include "drake/geometry/proximity/make_ellipsoid_field.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/make_sphere_field.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/tessellation_strategy.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {

using std::make_unique;
using std::move;

SoftMesh& SoftMesh::operator=(const SoftMesh& s) {
  if (this == &s) return *this;

  mesh_ = make_unique<VolumeMesh<double>>(s.mesh());
  // We can't simply copy the mesh field; the copy must contain a pointer to
  // the new mesh. So, we use CloneAndSetMesh() instead.
  pressure_ = s.pressure().CloneAndSetMesh(mesh_.get());
  bvh_ = make_unique<Bvh<VolumeMesh<double>>>(s.bvh());

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
  const HydroelasticType type = properties.GetPropertyOrDefault(
      kHydroGroup, kComplianceType, HydroelasticType::kUndefined);
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

// Validator interface for use with extracting valid properties. It is
// instantiated with shape (e.g., "Sphere", "Box", etc.) and compliance (i.e.,
// "rigid" or "soft") strings (to help give intelligible error messages) and
// then attempts to extract a typed value from a set of proximity properties --
// spewing meaningful error messages based on absence, type mismatch, and
// invalid values.
template <typename ValueType>
class Validator {
 public:
  // Parameters `shape_name` and `compliance` are only for error messages.
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

 protected:
  const char* shape_name() const { return shape_name_; }
  const char* compliance() const { return compliance_; }

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
  // Inherit the constructor from the base class.
  using Validator<double>::Validator;

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
    const HalfSpace& hs, const ProximityProperties&) {
  return RigidGeometry(hs);
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Sphere& sphere, const ProximityProperties& props) {
  PositiveDouble validator("Sphere", "rigid");
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  auto mesh = make_unique<SurfaceMesh<double>>(
      MakeSphereSurfaceMesh<double>(sphere, edge_length));

  return RigidGeometry(RigidMesh(move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Box& box, const ProximityProperties& props) {
  PositiveDouble validator("Box", "rigid");
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  auto mesh = make_unique<SurfaceMesh<double>>(
      MakeBoxSurfaceMesh<double>(box, edge_length));

  return RigidGeometry(RigidMesh(move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Cylinder& cylinder, const ProximityProperties& props) {
  PositiveDouble validator("Cylinder", "rigid");
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  auto mesh = make_unique<SurfaceMesh<double>>(
      MakeCylinderSurfaceMesh<double>(cylinder, edge_length));

  return RigidGeometry(RigidMesh(move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Ellipsoid& ellipsoid, const ProximityProperties& props) {
  PositiveDouble validator("Ellipsoid", "rigid");
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  auto mesh = make_unique<SurfaceMesh<double>>(
      MakeEllipsoidSurfaceMesh<double>(ellipsoid, edge_length));

  return RigidGeometry(RigidMesh(move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Mesh& mesh_spec, const ProximityProperties&) {
  // Mesh does not use any properties.
  auto mesh = make_unique<SurfaceMesh<double>>(
      ReadObjToSurfaceMesh(mesh_spec.filename(), mesh_spec.scale()));

  return RigidGeometry(RigidMesh(move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Convex& convex_spec, const ProximityProperties&) {
  // Convex does not use any properties.
  auto mesh = make_unique<SurfaceMesh<double>>(
      ReadObjToSurfaceMesh(convex_spec.filename(), convex_spec.scale()));

  return RigidGeometry(RigidMesh(move(mesh)));
}

std::optional<SoftGeometry> MakeSoftRepresentation(
    const Sphere& sphere, const ProximityProperties& props) {
  PositiveDouble validator("Sphere", "soft");
  // First, create the mesh.
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  // If nothing is said, let's go for the *cheap* tessellation strategy.
  const TessellationStrategy strategy =
      props.GetPropertyOrDefault(kHydroGroup, "tessellation_strategy",
                                 TessellationStrategy::kSingleInteriorVertex);
  auto mesh = make_unique<VolumeMesh<double>>(
      MakeSphereVolumeMesh<double>(sphere, edge_length, strategy));

  const double elastic_modulus =
      validator.Extract(props, kMaterialGroup, kElastic);

  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeSpherePressureField(sphere, mesh.get(), elastic_modulus));

  return SoftGeometry(SoftMesh(move(mesh), move(pressure)));
}

std::optional<SoftGeometry> MakeSoftRepresentation(
    const Box& box, const ProximityProperties& props) {
  PositiveDouble validator("Box", "soft");
  // First, create the mesh.
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  auto mesh = make_unique<VolumeMesh<double>>(
      MakeBoxVolumeMesh<double>(box, edge_length));

  const double elastic_modulus =
      validator.Extract(props, kMaterialGroup, kElastic);

  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeBoxPressureField(box, mesh.get(), elastic_modulus));

  return SoftGeometry(SoftMesh(move(mesh), move(pressure)));
}

std::optional<SoftGeometry> MakeSoftRepresentation(
    const Cylinder& cylinder, const ProximityProperties& props) {
  PositiveDouble validator("Cylinder", "soft");
  // First, create the mesh.
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  auto mesh = make_unique<VolumeMesh<double>>(
      MakeCylinderVolumeMesh<double>(cylinder, edge_length));

  const double elastic_modulus =
      validator.Extract(props, kMaterialGroup, kElastic);

  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeCylinderPressureField(cylinder, mesh.get(), elastic_modulus));

  return SoftGeometry(SoftMesh(move(mesh), move(pressure)));
}

std::optional<SoftGeometry> MakeSoftRepresentation(
    const Ellipsoid& ellipsoid, const ProximityProperties& props) {
  PositiveDouble validator("Ellipsoid", "soft");
  // First, create the mesh.
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  // If nothing is said, let's go for the *cheap* tessellation strategy.
  const TessellationStrategy strategy =
      props.GetPropertyOrDefault(kHydroGroup, "tessellation_strategy",
                                 TessellationStrategy::kSingleInteriorVertex);
  auto mesh = make_unique<VolumeMesh<double>>(
      MakeEllipsoidVolumeMesh<double>(ellipsoid, edge_length, strategy));

  const double elastic_modulus =
      validator.Extract(props, kMaterialGroup, kElastic);

  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeEllipsoidPressureField(ellipsoid, mesh.get(), elastic_modulus));

  return SoftGeometry(SoftMesh(move(mesh), move(pressure)));
}

std::optional<SoftGeometry> MakeSoftRepresentation(
    const HalfSpace&, const ProximityProperties& props) {
  PositiveDouble validator("HalfSpace", "soft");

  const double thickness =
      validator.Extract(props, kHydroGroup, kSlabThickness);

  const double elastic_modulus =
      validator.Extract(props, kMaterialGroup, kElastic);

  return SoftGeometry(SoftHalfSpace{elastic_modulus / thickness});
}

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
