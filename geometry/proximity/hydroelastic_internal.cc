#include "drake/geometry/proximity/hydroelastic_internal.h"

#include <algorithm>
#include <filesystem>
#include <optional>
#include <string>

#include <fmt/format.h>

#include "drake/geometry/proximity/make_box_field.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_capsule_field.h"
#include "drake/geometry/proximity/make_capsule_mesh.h"
#include "drake/geometry/proximity/make_convex_field.h"
#include "drake/geometry/proximity/make_convex_mesh.h"
#include "drake/geometry/proximity/make_cylinder_field.h"
#include "drake/geometry/proximity/make_cylinder_mesh.h"
#include "drake/geometry/proximity/make_ellipsoid_field.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/make_mesh_field.h"
#include "drake/geometry/proximity/make_mesh_from_vtk.h"
#include "drake/geometry/proximity/make_sphere_field.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/polygon_to_triangle_mesh.h"
#include "drake/geometry/proximity/tessellation_strategy.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {

namespace {

// Decide whether a shape is primitive for vanished-checking purposes. (See
// Geometries::is_vanished() documentation).  This reifier expects that
// user_data will point to a single boolean flag, which is pre-set to `true`.
class IsPrimitiveChecker final : public ShapeReifier {
 private:
  using ShapeReifier::ImplementGeometry;

  // Primitives are the default. The handler here does nothing; rely on the
  // caller to have pre-set the user_data flag to true.
  void DefaultImplementGeometry(const Shape&) final {}

  // Non-primitives.

  void ImplementGeometry(const Convex&, void* user_data) final {
    *static_cast<bool*>(user_data) = false;
  }

  void ImplementGeometry(const HalfSpace&, void* user_data) final {
    *static_cast<bool*>(user_data) = false;
  }

  void ImplementGeometry(const Mesh&, void* user_data) final {
    *static_cast<bool*>(user_data) = false;
  }

  void ImplementGeometry(const MeshcatCone&, void*) final {
    DRAKE_UNREACHABLE();
  }
};

bool is_primitive(const Shape& shape) {
  bool result{true};  // The reifier default is to assume primitive.
  IsPrimitiveChecker checker;
  shape.Reify(&checker, &result);
  return result;
}

}  // namespace

using std::make_unique;

SoftMesh& SoftMesh::operator=(const SoftMesh& s) {
  if (this == &s) return *this;

  mesh_ = make_unique<VolumeMesh<double>>(s.mesh());
  // We can't simply copy the mesh field; the copy must contain a pointer to
  // the new mesh. So, we use CloneAndSetMesh() instead.
  pressure_ = s.pressure().CloneAndSetMesh(mesh_.get());
  bvh_ = make_unique<Bvh<Obb, VolumeMesh<double>>>(s.bvh());

  return *this;
}

HydroelasticType Geometries::hydroelastic_type(GeometryId id) const {
  auto iter = supported_geometries_.find(id);
  if (iter != supported_geometries_.end()) return iter->second;
  return HydroelasticType::kUndefined;
}

bool Geometries::is_vanished(GeometryId id) const {
  return vanished_geometries_.contains(id);
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

void Geometries::ImplementGeometry(const Box& box, void* user_data) {
  MakeShape(box, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Capsule& capsule, void* user_data) {
  MakeShape(capsule, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Convex& convex, void* user_data) {
  MakeShape(convex, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Cylinder& cylinder, void* user_data) {
  MakeShape(cylinder, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Ellipsoid& ellipsoid,
                                   void* user_data) {
  MakeShape(ellipsoid, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const HalfSpace& half_space,
                                   void* user_data) {
  MakeShape(half_space, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Mesh& mesh, void* user_data) {
  MakeShape(mesh, *static_cast<ReifyData*>(user_data));
}

void Geometries::ImplementGeometry(const Sphere& sphere, void* user_data) {
  MakeShape(sphere, *static_cast<ReifyData*>(user_data));
}

template <typename ShapeType>
void Geometries::MakeShape(const ShapeType& shape, const ReifyData& data) {
  switch (data.type) {
    case HydroelasticType::kRigid: {
      auto hydro_geometry = MakeRigidRepresentation(shape, data.properties);
      if (hydro_geometry) AddGeometry(data.id, std::move(*hydro_geometry));
    } break;
    case HydroelasticType::kSoft: {
      auto hydro_geometry = MakeSoftRepresentation(shape, data.properties);
      if (hydro_geometry) {
        if (is_primitive(shape) &&
            hydro_geometry->pressure_field().is_gradient_field_degenerate()) {
          vanished_geometries_.insert(data.id);
        } else {
          AddGeometry(data.id, std::move(*hydro_geometry));
        }
      }
    } break;
    case HydroelasticType::kUndefined:
      // No action required.
      break;
  }
}

void Geometries::AddGeometry(GeometryId id, SoftGeometry geometry) {
  DRAKE_DEMAND(hydroelastic_type(id) == HydroelasticType::kUndefined);
  supported_geometries_[id] = HydroelasticType::kSoft;
  soft_geometries_.insert({id, std::move(geometry)});
}

void Geometries::AddGeometry(GeometryId id, RigidGeometry geometry) {
  DRAKE_DEMAND(hydroelastic_type(id) == HydroelasticType::kUndefined);
  supported_geometries_[id] = HydroelasticType::kRigid;
  rigid_geometries_.insert({id, std::move(geometry)});
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
  // Validator for quantities with no default value, see has_default_value().
  // Parameters `shape_name` and `compliance` are only for error messages.
  Validator(const char* shape_name, const char* compliance)
      : shape_name_(shape_name), compliance_(compliance) {}

  // Validator for quantities with a default value, see has_default_value().
  // Parameters `shape_name` and `compliance` are only for error messages.
  Validator(const char* shape_name, const char* compliance,
            const ValueType default_value)
      : shape_name_(shape_name),
        compliance_(compliance),
        default_value_(std::move(default_value)) {}

  virtual ~Validator() = default;

  bool has_default_value() const { return default_value_.has_value(); }

  // Extract an arbitrary property from the proximity properties. For validators
  // with no default value (has_default_value() returns false), throws a
  // consistent error message in the case of missing or mis-typed properties.
  // For validators with default value, the default value provided at
  // construction is returned in case of a missing property.
  // Relies on the ValidateValue() method to validate the value.
  ValueType Extract(const ProximityProperties& props, const char* group_name,
                    const char* property_name) {
    const std::string full_property_name =
        fmt::format("('{}', '{}')", group_name, property_name);
    if (!has_default_value() && !props.HasProperty(group_name, property_name)) {
      throw std::logic_error(
          fmt::format("Cannot create {} {}; missing the {} property",
                      compliance(), shape_name(), full_property_name));
    }
    const ValueType value =
        has_default_value()
            ? props.GetPropertyOrDefault<ValueType>(group_name, property_name,
                                                    *default_value_)
            : props.GetProperty<ValueType>(group_name, property_name);
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
  std::optional<ValueType> default_value_;
};

// Validator that extracts *strictly positive doubles*.
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

// Validator that extracts *non-negative doubles*, where a zero value is valid.
// In case of missing property, a value of zero is returned.
class NonNegativeDouble : public Validator<double> {
 public:
  NonNegativeDouble(const char* shape_name, const char* compliance)
      : Validator<double>(shape_name, compliance, 0.0) {}

 protected:
  void ValidateValue(const double& value,
                     const std::string& property) const override {
    if (value < 0) {
      throw std::logic_error(fmt::format(
          "Cannot create {} {}; the {} property must be non-negative",
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
  auto mesh = make_unique<TriangleSurfaceMesh<double>>(
      MakeSphereSurfaceMesh<double>(sphere, edge_length));

  return RigidGeometry(RigidMesh(std::move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Box& box, const ProximityProperties&) {
  PositiveDouble validator("Box", "rigid");
  // Use the coarsest mesh for the box. The safety factor 1.1 guarantees the
  // resolution-hint argument is larger than the box size, so the mesh
  // will have only 8 vertices and 12 triangles.
  auto mesh = make_unique<TriangleSurfaceMesh<double>>(
      MakeBoxSurfaceMesh<double>(box, 1.1 * box.size().maxCoeff()));

  return RigidGeometry(RigidMesh(std::move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Cylinder& cylinder, const ProximityProperties& props) {
  PositiveDouble validator("Cylinder", "rigid");
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  auto mesh = make_unique<TriangleSurfaceMesh<double>>(
      MakeCylinderSurfaceMesh<double>(cylinder, edge_length));

  return RigidGeometry(RigidMesh(std::move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Capsule& capsule, const ProximityProperties& props) {
  PositiveDouble validator("Capsule", "rigid");
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  auto mesh = make_unique<TriangleSurfaceMesh<double>>(
      MakeCapsuleSurfaceMesh<double>(capsule, edge_length));

  return RigidGeometry(RigidMesh(std::move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Ellipsoid& ellipsoid, const ProximityProperties& props) {
  PositiveDouble validator("Ellipsoid", "rigid");
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  auto mesh = make_unique<TriangleSurfaceMesh<double>>(
      MakeEllipsoidSurfaceMesh<double>(ellipsoid, edge_length));

  return RigidGeometry(RigidMesh(std::move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Mesh& mesh_spec, const ProximityProperties&) {
  // Mesh does not use any properties.
  std::unique_ptr<TriangleSurfaceMesh<double>> mesh;

  const std::string extension = mesh_spec.extension();
  if (extension == ".obj") {
    mesh = make_unique<TriangleSurfaceMesh<double>>(
        ReadObjToTriangleSurfaceMesh(mesh_spec.filename(), mesh_spec.scale()));
  } else if (extension == ".vtk") {
    mesh = make_unique<TriangleSurfaceMesh<double>>(
        ConvertVolumeToSurfaceMesh(MakeVolumeMeshFromVtk<double>(mesh_spec)));
  } else {
    throw(std::runtime_error(fmt::format(
        "hydroelastic::MakeRigidRepresentation(): for rigid hydroelastic Mesh "
        "shapes can only use .obj or .vtk files; given: {}",
        mesh_spec.filename())));
  }

  return RigidGeometry(RigidMesh(std::move(mesh)));
}

std::optional<RigidGeometry> MakeRigidRepresentation(
    const Convex& convex_spec, const ProximityProperties&) {
  // Simply use the Convex's GetConvexHull().
  return RigidGeometry(RigidMesh(make_unique<TriangleSurfaceMesh<double>>(
      MakeTriangleFromPolygonMesh(convex_spec.GetConvexHull()))));
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

  const double hydroelastic_modulus =
      validator.Extract(props, kHydroGroup, kElastic);

  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeSpherePressureField(sphere, mesh.get(), hydroelastic_modulus));

  return SoftGeometry(SoftMesh(std::move(mesh), std::move(pressure)));
}

std::optional<SoftGeometry> MakeSoftRepresentation(
    const Box& box, const ProximityProperties& props) {
  PositiveDouble validator("Box", "soft");
  const double margin =
      NonNegativeDouble("Box", "soft").Extract(props, kMaterialGroup, kMargin);

  // Define the shape of the "inflated" hydroelastic geometry to include the
  // margin. We inflate all faces of the box a distance "margin" along the
  // outward normal.
  Box inflated_shape(box.size() + Vector3<double>::Constant(2.0 * margin));

  // First, create an inflated mesh.
  auto inflated_mesh = make_unique<VolumeMesh<double>>(
      MakeBoxVolumeMeshWithMa<double>(inflated_shape));

  const double hydroelastic_modulus =
      validator.Extract(props, kHydroGroup, kElastic);

  // We define a pressure field such that its zero level set lies at the
  // boundary of the original input `box`.
  auto pressure =
      make_unique<VolumeMeshFieldLinear<double, double>>(MakeBoxPressureField(
          inflated_shape, inflated_mesh.get(), hydroelastic_modulus, margin));

  return SoftGeometry(SoftMesh(std::move(inflated_mesh), std::move(pressure)));
}

std::optional<SoftGeometry> MakeSoftRepresentation(
    const Cylinder& cylinder, const ProximityProperties& props) {
  PositiveDouble validator("Cylinder", "soft");
  // First, create the mesh.
  const double edge_length = validator.Extract(props, kHydroGroup, kRezHint);
  auto mesh = make_unique<VolumeMesh<double>>(
      MakeCylinderVolumeMeshWithMa<double>(cylinder, edge_length));

  const double hydroelastic_modulus =
      validator.Extract(props, kHydroGroup, kElastic);

  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeCylinderPressureField(cylinder, mesh.get(), hydroelastic_modulus));

  return SoftGeometry(SoftMesh(std::move(mesh), std::move(pressure)));
}

std::optional<SoftGeometry> MakeSoftRepresentation(
    const Capsule& capsule, const ProximityProperties& props) {
  const double margin = NonNegativeDouble("Capsule", "soft")
                            .Extract(props, kMaterialGroup, kMargin);
  const Capsule inflated_capsule(capsule.radius() + margin, capsule.length());

  // First, create the mesh.
  const double edge_length =
      PositiveDouble("Capsule", "soft").Extract(props, kHydroGroup, kRezHint);
  auto inflated_mesh = make_unique<VolumeMesh<double>>(
      MakeCapsuleVolumeMesh<double>(inflated_capsule, edge_length));

  const double hydroelastic_modulus =
      PositiveDouble("Capsule", "soft").Extract(props, kHydroGroup, kElastic);
  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeCapsulePressureField(inflated_capsule, inflated_mesh.get(),
                               hydroelastic_modulus, margin));

  return SoftGeometry(SoftMesh(std::move(inflated_mesh), std::move(pressure)));
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

  const double margin =
      NonNegativeDouble("Box", "soft").Extract(props, kMaterialGroup, kMargin);
  const Ellipsoid inflated_ellipsoid(
      ellipsoid.a() + margin, ellipsoid.b() + margin, ellipsoid.c() + margin);
  auto inflated_mesh =
      make_unique<VolumeMesh<double>>(MakeEllipsoidVolumeMesh<double>(
          inflated_ellipsoid, edge_length, strategy));

  const double hydroelastic_modulus =
      validator.Extract(props, kHydroGroup, kElastic);
  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeEllipsoidPressureField(inflated_ellipsoid, inflated_mesh.get(),
                                 hydroelastic_modulus, margin));

  return SoftGeometry(SoftMesh(std::move(inflated_mesh), std::move(pressure)));
}

std::optional<SoftGeometry> MakeSoftRepresentation(
    const HalfSpace&, const ProximityProperties& props) {
  PositiveDouble validator("HalfSpace", "soft");

  const double thickness =
      validator.Extract(props, kHydroGroup, kSlabThickness);

  const double hydroelastic_modulus =
      validator.Extract(props, kHydroGroup, kElastic);

  return SoftGeometry(SoftHalfSpace{hydroelastic_modulus / thickness});
}

std::optional<SoftGeometry> MakeSoftRepresentation(
    const Convex& convex_spec, const ProximityProperties& props) {
  PositiveDouble validator("Convex", "soft");

  // Use the pre-computed convex hull for the shape.
  const TriangleSurfaceMesh<double> surface_mesh =
      MakeTriangleFromPolygonMesh(convex_spec.GetConvexHull());
  auto mesh = make_unique<VolumeMesh<double>>(
      MakeConvexVolumeMesh<double>(surface_mesh));

  const double hydroelastic_modulus =
      validator.Extract(props, kHydroGroup, kElastic);

  auto pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeConvexPressureField(mesh.get(), hydroelastic_modulus));

  return SoftGeometry(SoftMesh(std::move(mesh), std::move(pressure)));
}

std::optional<SoftGeometry> MakeSoftRepresentation(
    const Mesh& mesh_specification, const ProximityProperties& props) {
  PositiveDouble validator("Mesh", "soft");

  const double hydroelastic_modulus =
      validator.Extract(props, kHydroGroup, kElastic);

  std::unique_ptr<VolumeMesh<double>> mesh;
  std::unique_ptr<VolumeMeshFieldLinear<double, double>> pressure;

  const double margin =
      NonNegativeDouble("Box", "soft").Extract(props, kMaterialGroup, kMargin);

  if (mesh_specification.extension() == ".vtk") {
    // If they've explicitly provided a .vtk file, we'll treat it as it is a
    // volume mesh. If that's not true, we'll get an error.
    mesh = make_unique<VolumeMesh<double>>(
        MakeVolumeMeshFromVtk<double>(mesh_specification));
  } else {
    // Otherwise, we'll create a compliant representation of its convex hull.
    const TriangleSurfaceMesh<double> surface_mesh =
        MakeTriangleFromPolygonMesh(mesh_specification.GetConvexHull());
    mesh = make_unique<VolumeMesh<double>>(
        MakeConvexVolumeMesh<double>(surface_mesh));
  }

  pressure = make_unique<VolumeMeshFieldLinear<double, double>>(
      MakeVolumeMeshPressureField(mesh.get(), hydroelastic_modulus, margin));

  return SoftGeometry(SoftMesh(std::move(mesh), std::move(pressure)));
}

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
