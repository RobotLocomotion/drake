#pragma once

#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <variant>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {

// TODO(SeanCurtis-TRI): When we do soft-soft contact, we'll need ∇p̃(e) as well.
//  ∇p̃(e) is piecewise constant -- one ℜ³ vector per tetrahedron.
/* Defines a soft mesh -- a mesh, its linearized pressure field, p̃(e), and its
 bounding volume hierarchy. While this class retains ownership of the mesh,
 we assume that both the pressure field and the bounding volume hierarchy
 are derived from the mesh. */
class SoftMesh {
 public:
  SoftMesh() = default;

  SoftMesh(std::unique_ptr<VolumeMesh<double>> mesh,
           std::unique_ptr<VolumeMeshFieldLinear<double, double>> pressure)
      : mesh_(std::move(mesh)),
        pressure_(std::move(pressure)),
        bvh_(std::make_unique<Bvh<Obb, VolumeMesh<double>>>(*mesh_)) {
    DRAKE_ASSERT(mesh_.get() == &pressure_->mesh());
  }

  SoftMesh(const SoftMesh& s) { *this = s; }
  SoftMesh& operator=(const SoftMesh& s);
  SoftMesh(SoftMesh&&) = default;
  SoftMesh& operator=(SoftMesh&&) = default;

  const VolumeMesh<double>& mesh() const {
    DRAKE_DEMAND(mesh_ != nullptr);
    return *mesh_;
  }
  const VolumeMeshFieldLinear<double, double>& pressure() const {
    DRAKE_DEMAND(pressure_ != nullptr);
    return *pressure_;
  }
  const Bvh<Obb, VolumeMesh<double>>& bvh() const {
    DRAKE_DEMAND(bvh_ != nullptr);
    return *bvh_;
  }

 private:
  std::unique_ptr<VolumeMesh<double>> mesh_;
  std::unique_ptr<VolumeMeshFieldLinear<double, double>> pressure_;
  std::unique_ptr<Bvh<Obb, VolumeMesh<double>>> bvh_;
};

/* Defines a soft half space. The half space is defined such that the half
 space's boundary plane is z = 0 in Frame H. Vector Hz points _out_ of the half
 space. The half space is considered to be a soft layer of thickness h
 overlaying a rigid substrate of infinite extent. Its compliance is
 characterized by two parameters:

   - elastic modulus: a measure of the stiffness of the material under small
     deformation (<< h), and
   - slab thickness: the thickness of the compliant layer.

 The pressure in the soft layer may be modeled in many ways. Currently, we use
 the simplest model in which the pressure field is `ρ = E⋅d/h`, where `E` is the
 elastic modulus, `d` is the positive penetration measure, and `h` is the slab
 thickness -- a simple linear function where pressure is a scale of the depth.
 This model is valid for small penetrations but fails to capture the increased
 stiffness for deformations that approach or penetrate the rigid substrate.

 The hydroelastic representation combines elastic modulus and slab thickness
 into a "pressure scale" value: `s = E / h`, which maps penetration depth to
 pressure. The pressure gradient is always in the -Hz direction.

 We don't tessellate half spaces because:

   - a finite discretization is a poor representation of an infinite volume, and
   - it is computationally advantageous to *not* discretize the half space.
 */
struct SoftHalfSpace {
  double pressure_scale;
  // TODO(SeanCurtis-TRI): Possibly add a customizable pressure function in the
  //  future; one that isn't simply the scaled, normalized penetration distance.
};

/* Definition of a soft geometry for hydroelastic implementations. To be a
 soft geometry, a shape must be associated with either:

   - a volume mesh (including a linearized scalar pressure field), or
   - a soft half space (with a "slab thickness").  */
class SoftGeometry {
 public:
  /* Constructs a soft half space representation.  */
  explicit SoftGeometry(const SoftHalfSpace& soft_half_space)
      : geometry_(soft_half_space) {}

  /* Constructs a soft mesh representation.  */
  explicit SoftGeometry(SoftMesh&& soft_mesh)
      : geometry_(std::move(soft_mesh)) {}

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SoftGeometry)

  /* @name  Distinguishing compliant representations

   The %SoftGeometry can contain either a volume mesh (used as the
   representation for most shapes) or a half space. Accessing the members of
   either representation (`mesh()`, `pressure_field()`, and `bvh()` for the
   volume mesh or `pressure_scale()` for the half space) is conditioned on
   knowing what type a particular instance holds.

   This can be accomplished by querying `is_half_space()`. Attempting to access
   data members of the *wrong* type will throw an exception.  */
  //@{

  bool is_half_space() const {
    return std::holds_alternative<SoftHalfSpace>(geometry_);
  }

  /* Returns a reference to the volume mesh -- calling this will throw if
   is_half_space() returns `true`.  */
  const VolumeMesh<double>& mesh() const {
    if (is_half_space()) {
      throw std::runtime_error(
          "SoftGeometry::mesh() cannot be invoked for soft half space");
    }
    return std::get<SoftMesh>(geometry_).mesh();
  }

  /* Returns a reference to the mesh's linearized pressure field -- calling
   this will throw if is_half_space() returns `true`.  */
  const VolumeMeshFieldLinear<double, double>& pressure_field() const {
    if (is_half_space()) {
      throw std::runtime_error("SoftGeometry::pressure_field() cannot be "
                               "invoked for soft half space");
    }
    return std::get<SoftMesh>(geometry_).pressure();
  }

  /* Returns a reference to the bounding volume hierarchy -- calling this will
   throw if is_half_space() returns `true`.  */
  const Bvh<Obb, VolumeMesh<double>>& bvh() const {
    if (is_half_space()) {
      throw std::runtime_error(
          "SoftGeometry::bvh() cannot be invoked for soft half space");
    }
    return std::get<SoftMesh>(geometry_).bvh();
  }

  /* Returns the half space's pressure scale -- calling this will throw if
   is_half_space() returns `false`.  */
  double pressure_scale() const {
    if (!is_half_space()) {
      throw std::runtime_error(
          "SoftGeometry::pressure_scale() cannot be invoked for soft mesh");
    }
    return std::get<SoftHalfSpace>(geometry_).pressure_scale;
  }

  //@}

 private:
  std::variant<SoftHalfSpace, SoftMesh> geometry_;
};

/* Defines a rigid mesh -- a surface mesh and its bounding volume hierarchy.
 This class retains ownership of the mesh, with the bounding volume hierarchy
 just referencing it.  */
class RigidMesh {
 public:
  RigidMesh() = default;

  explicit RigidMesh(std::unique_ptr<TriangleSurfaceMesh<double>> mesh)
      : mesh_(std::move(mesh)),
        bvh_(std::make_unique<Bvh<Obb, TriangleSurfaceMesh<double>>>(
            *mesh_)) {}

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RigidMesh)

  const TriangleSurfaceMesh<double>& mesh() const {
    DRAKE_DEMAND(mesh_ != nullptr);
    return *mesh_;
  }
  const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh() const {
    DRAKE_DEMAND(bvh_ != nullptr);
    return *bvh_;
  }

 private:
  copyable_unique_ptr<TriangleSurfaceMesh<double>> mesh_;
  copyable_unique_ptr<Bvh<Obb, TriangleSurfaceMesh<double>>> bvh_;
};

/* The base representation of rigid geometries. Generally, a rigid geometry
 is represented with a TriangleSurfaceMesh. However, half spaces do not get
 tessellated and are treated as primitives. This class contains either
 representation.  */
class RigidGeometry {
 public:
  /* Constructs a rigid half space representation -- the half space, like its
   specification HalfSpace, is defined in its canonical frame H with the
   boundary plane at z = 0 and its outward normal pointing in the Hz direction.
   */
  explicit RigidGeometry(const HalfSpace&) {}

  /* Constructs a rigid mesh representation.  */
  explicit RigidGeometry(RigidMesh&& rigid_mesh)
      : geometry_(RigidMesh(std::move(rigid_mesh))) {}

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RigidGeometry)

  /* Returns true if this RigidGeometry is a half space.  */
  bool is_half_space() const { return !geometry_.has_value(); }

  /* Returns a reference to the surface mesh -- calling this will throw unless
   is_half_space() returns false.  */
  const TriangleSurfaceMesh<double>& mesh() const {
    if (is_half_space()) {
      throw std::runtime_error(
          "RigidGeometry::mesh() cannot be invoked for rigid half space");
    }
    return geometry_->mesh();
  }

  /* Returns a reference to the bounding volume hierarchy -- calling this will
   throw unless is_half_space() returns false.  */
  const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh() const {
    if (is_half_space()) {
      throw std::runtime_error(
          "RigidGeometry::bvh() cannot be invoked for rigid half space");
    }
    return geometry_->bvh();
  }

 private:
  // If the mesh isn't defined, then this is implicitly a rigid half space.
  std::optional<RigidMesh> geometry_{std::nullopt};
};

/* This class stores all instantiated hydroelastic representations of declared
 geometry. They are keyed by the geometry's global GeometryId.

 In order for a geometry with id `g_id` to have a representation in this
 collection, it must:

   - be declared via calling MaybeAddGeometry();
   - the infrastructure must support the type of representation for that
     particular geometry type;
   - it must have a valid set of properties to fully specify the representation;
   - the geometry cannot have been subsequently removed via a call to
     RemoveGeometry().

 If two geometries are in contact, in order to produce the corresponding
 ContactSurface, both ids must have a valid representation in this set.  */
class Geometries final : public ShapeReifier {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Geometries);

  Geometries() = default;

  /* Reports the hydroelastic representation type for the given id. The
   following invariants should always hold:

     - If HydroelasticType::kUndefined is returned, there is no representation
       for that id.
     - If HydroelasticType::kRigid is returned, there is a rigid geometry
       associated with that id and calling rigid_geometry() will return a
       valid RigidGeometry.
     - If HydroelasticType::kSoft is returned, there is a soft geometry
       associated with that id and calling soft_geometry() will return a valid
       SoftGeometry.  */
  HydroelasticType hydroelastic_type(GeometryId id) const;

  /* Returns the representation of the soft geometry with the given `id`.
   @pre hydroelastic_type(id) returns HydroelasticType::kSoft.  */
  const SoftGeometry& soft_geometry(GeometryId id) const {
    DRAKE_DEMAND(hydroelastic_type(id) == HydroelasticType::kSoft);
    return soft_geometries_.at(id);
  }

  /* Returns the representation of the rigid geometry with the given `id`.
   @pre hydroelastic_type(id) returns HydroelasticType::kRigid.  */
  const RigidGeometry& rigid_geometry(GeometryId id) const {
    DRAKE_DEMAND(hydroelastic_type(id) == HydroelasticType::kRigid);
    return rigid_geometries_.at(id);
  }

  /* Removes the geometry (if it has a hydroelastic representation).  */
  void RemoveGeometry(GeometryId id);

  /* Examines the given shape and properties, adding a hydroelastic
   representation as indicated by the `properties` and supported by the current
   hydroelastic infrastructure. No exception is thrown if the given shape is
   not supported in the current infrastructure. However, if it *is* supported,
   but the properties are malformed, an exception will be thrown.

   @param shape         The shape to possibly represent.
   @param id            The unique identifier for the geometry.
   @param properties    The proximity properties which will determine if a
                        hydroelastic representation is requested.
   @throws std::exception if the shape is a supported type but the properties
                          are malformed.
   @pre There is no previous representation associated with id.  */
  void MaybeAddGeometry(const Shape& shape, GeometryId id,
                        const ProximityProperties& properties);

 private:
  // Data to be used during reification. It is passed as the `user_data`
  // parameter in the ImplementGeometry API.
  struct ReifyData {
    HydroelasticType type;
    GeometryId id;
    const ProximityProperties& properties;
  };

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Sphere& sphere, void* user_data) override;
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) override;
  void ImplementGeometry(const HalfSpace&, void* user_data) override;
  void ImplementGeometry(const Box& box, void* user_data) override;
  void ImplementGeometry(const Capsule& capsule, void* user_data) override;
  void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data) override;
  void ImplementGeometry(const Mesh&, void*) override;
  void ImplementGeometry(const Convex& convex, void* user_data) override;

  template <typename ShapeType>
  void MakeShape(const ShapeType& shape, const ReifyData& data);

  // Adds a representation of the soft geometry with the given `id`.
  // @pre there is no previous representation associated with `id`.
  void AddGeometry(GeometryId id, SoftGeometry field);

  // Adds a representation of the rigid geometry with the given `id`.
  // @pre there is no previous representation associated with `id`.
  void AddGeometry(GeometryId id, RigidGeometry mesh);

  // The ids of all geometries that have a hydroelastic representation and
  // the type of representation.
  std::unordered_map<GeometryId, HydroelasticType> supported_geometries_;

  // The representations of all soft geometries.
  std::unordered_map<GeometryId, SoftGeometry> soft_geometries_;

  // The representations of all rigid geometries.
  std::unordered_map<GeometryId, RigidGeometry> rigid_geometries_;
};

/* @name Creating hydroelastic representations of shapes

 By default *no* shapes are supported with hydroelastic representations.
 Attempting to do so will print a warning to the Drake log and these functions
 will return std::nullopt.

 For every shape that *is* supported, an overload of this method on that shape
 type is declared below.  */
//@{

/* Generic interface for handling unsupported rigid Shapes. Unsupported
 geometries will return a std::nullopt.  */
template <typename Shape>
std::optional<RigidGeometry> MakeRigidRepresentation(
    const Shape& shape, const ProximityProperties&) {
  static const logging::Warn log_once(
      "Rigid {} shapes are not currently supported for hydroelastic "
      "contact; registration is allowed, but an error will be thrown "
      "during contact.",
      ShapeName(shape));
  return {};
}

/* Rigid sphere support. Requires the ('hydroelastic', 'resolution_hint')
 property.  */
std::optional<RigidGeometry> MakeRigidRepresentation(
    const Sphere& sphere, const ProximityProperties& props);

/* Rigid box support. It doesn't depend on any of the proximity properties. */
std::optional<RigidGeometry> MakeRigidRepresentation(
    const Box& box, const ProximityProperties& props);

/* Rigid cylinder support. Requires the ('hydroelastic', 'resolution_hint')
 property.  */
std::optional<RigidGeometry> MakeRigidRepresentation(
    const Cylinder& cylinder, const ProximityProperties& props);

/* Rigid capsule support. Requires the ('hydroelastic', 'resolution_hint')
 property.  */
std::optional<RigidGeometry> MakeRigidRepresentation(
    const Capsule& capsule, const ProximityProperties& props);

/* Rigid ellipsoid support. Requires the ('hydroelastic', 'resolution_hint')
 property.  */
std::optional<RigidGeometry> MakeRigidRepresentation(
    const Ellipsoid& ellipsoid, const ProximityProperties& props);

/* Rigid mesh support. It doesn't depend on any of the proximity properties. */
std::optional<RigidGeometry> MakeRigidRepresentation(
    const Mesh& mesh, const ProximityProperties& props);

/* Rigid convex support. It doesn't depend on any of the proximity properties.
 Note: the convexity of the mesh is *not* tested (and does not need to be).
 The representation of a Convex mesh is the same as a general Mesh, so its
 representation and functionality is indistinguishable, whether convex or not.
 */
std::optional<RigidGeometry> MakeRigidRepresentation(
    const Convex& convex, const ProximityProperties& props);

/* Rigid half space support.  */
std::optional<RigidGeometry> MakeRigidRepresentation(
    const HalfSpace& half_space, const ProximityProperties& props);

/* Generic interface for handling unsupported soft Shapes. Unsupported
 geometries will return a std::nullopt.  */
template <typename Shape>
std::optional<SoftGeometry> MakeSoftRepresentation(const Shape& shape,
                                                   const ProximityProperties&) {
  static const logging::Warn log_once(
      "Soft {} shapes are not currently supported for hydroelastic contact; "
      "registration is allowed, but an error will be thrown during contact.",
      ShapeName(shape));
  return {};
}

/* Creates a soft sphere (assuming the proximity properties have sufficient
 information). Requires the ('hydroelastic', 'resolution_hint') and
 ('hydroelastic', 'hydroelastic_modulus') properties.  */
std::optional<SoftGeometry> MakeSoftRepresentation(
    const Sphere& sphere, const ProximityProperties& props);

/* Creates a soft box (assuming the proximity properties have sufficient
 information). Requires the ('hydroelastic', 'hydroelastic_modulus')
 properties.  */
std::optional<SoftGeometry> MakeSoftRepresentation(
    const Box& box, const ProximityProperties& props);

/* Creates a soft cylinder (assuming the proximity properties have sufficient
 information). Requires the ('hydroelastic', 'resolution_hint') and
 ('hydroelastic', 'hydroelastic_modulus') properties.  */
std::optional<SoftGeometry> MakeSoftRepresentation(
    const Cylinder& cylinder, const ProximityProperties& props);

/* Creates a soft capsule (assuming the proximity properties have sufficient
 information). Requires the ('hydroelastic', 'resolution_hint') and
 ('hydroelastic', 'hydroelastic_modulus') properties.  */
std::optional<SoftGeometry> MakeSoftRepresentation(
    const Capsule& capsule, const ProximityProperties& props);

/* Creates a soft ellipsoid (assuming the proximity properties have sufficient
 information). Requires the ('hydroelastic', 'resolution_hint') and
 ('hydroelastic', 'hydroelastic_modulus') properties.  */
std::optional<SoftGeometry> MakeSoftRepresentation(
    const Ellipsoid& ellipsoid, const ProximityProperties& props);

/* Creates a compliant half space (assuming the proximity properties have
 sufficient information). Requires the ('hydroelastic', 'slab_thickness') and
 ('hydroelastic', 'hydroelastic_modulus') properties.  */
std::optional<SoftGeometry> MakeSoftRepresentation(
    const HalfSpace& half_space, const ProximityProperties& props);

//@}

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
