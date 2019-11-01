#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {

// TODO(SeanCurtis-TRI): Update this to have an additional classification: kBoth
//  when we have the need from the algorithm. For example: when we have two
//  very stiff objects, we'd want to process them as soft. But when one
//  very stiff and one very soft object interact, it might make sense to
//  consider the stiff object as effectively rigid and simplify the computation.
//  In this case, the object would get two representations.
/** Classification of the type of representation a shape has for the
 hydroelastic contact model: rigid or soft.  */
enum class HydroelasticType {
  kUndefined,
  kRigid,
  kSoft
};

/** Streaming operator for writing hydroelastic type to output stream.  */
std::ostream& operator<<(std::ostream& out, const HydroelasticType& type);

// TODO(SeanCurtis-TRI): When we do soft-soft contact, we'll need ∇p̃(e) as well.
//  ∇p̃(e) is piecewise constant -- one ℜ³ vector per tetrahedron.
/** Defines a soft mesh -- a mesh and its linearized pressure field, p̃(e).  */
struct SoftMesh {
  std::unique_ptr<VolumeMesh<double>> mesh;
  std::unique_ptr<VolumeMeshField<double, double>> pressure;
};

/** Definition of a soft geometry for hydroelastic implementations. Today, the
 soft geometry is a soft _mesh_. In the future it will include other soft
 representations (e.g., untessellated half space).  */
class SoftGeometry {
 public:
  /** Constructs a soft geometry from a soft mesh.  */
  SoftGeometry(
      std::unique_ptr<VolumeMesh<double>> mesh,
      std::unique_ptr<VolumeMeshField<double, double>> pressure)
      : geometry_(SoftMesh{std::move(mesh), std::move(pressure)}) {}

  SoftGeometry(const SoftGeometry& g) { *this = g; }
  SoftGeometry& operator=(const SoftGeometry& g);
  SoftGeometry(SoftGeometry&&) = default;
  SoftGeometry& operator=(SoftGeometry&&) = default;

  /** Returns a reference to the volume mesh.  */
  const VolumeMesh<double>& mesh() const {
    return *geometry_.mesh;
  }

  /** Returns a reference to the mesh's linearized pressure field.  */
  const VolumeMeshField<double, double>& pressure_field() const {
    return *geometry_.pressure;
  }

 private:
  SoftMesh geometry_;
};

/** The base representation of rigid geometries. A rigid geometry is represented
 with a SurfaceMesh.  */
class RigidGeometry {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RigidGeometry)

  /** Constructs a rigid representation from the given surface mesh.  */
  explicit RigidGeometry(SurfaceMesh<double> mesh) : mesh_(std::move(mesh)) {}

  /** Returns a reference to the surface mesh.  */
  const SurfaceMesh<double>& mesh() const { return mesh_; }

 private:
  SurfaceMesh<double> mesh_;
};

/** This class stores all instantiated hydroelastic representations of declared
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

  /** Reports the hydroelastic representation type for the given id. The
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

  /** Returns the representation of the soft geometry with the given `id`.
   @pre hydroelastic_type(id) returns HydroelasticType::kSoft.  */
  const SoftGeometry& soft_geometry(GeometryId id) const {
    DRAKE_DEMAND(hydroelastic_type(id) == HydroelasticType::kSoft);
    return soft_geometries_.at(id);
  }

  /** Returns the representation of the rigid geometry with the given `id`.
   @pre hydroelastic_type(id) returns HydroelasticType::kRigid.  */
  const RigidGeometry& rigid_geometry(GeometryId id) const {
    DRAKE_DEMAND(hydroelastic_type(id) == HydroelasticType::kRigid);
    return rigid_geometries_.at(id);
  }

  /** Removes the geometry (if it has a hydroelastic representation).  */
  void RemoveGeometry(GeometryId id);

  /** Examines the given shape and properties, adding a hydroelastic
   representation as indicated by the `properties` and supported by the current
   hydroelastic infrastructure. No exception is thrown if the given shape is
   not supported in the current infrastructure. However, if it *is* supported,
   but the properties are malformed, an exception will be thrown.

   @param shape         The shape to possibly represent.
   @param id            The unique identifier for the geometry.
   @param properties    The proximity properties which will determine if a
                        hydroelastic representation is requested.
   @throws std::logic_error if the shape is a supported type but the properties
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

/** Assesses the properties to determine *if* the geometry with the given
 `properties` requires a geometric representation and, if so, whether that
 representation is soft or rigid.

 The conditions are as follows:

   - If `properties` does not have the group "hydroelastic", then it requires no
     hydroelastic representation.
   - Otherwise, its compliance is based on the (material, elastic modulus)
     property. The disposition based on the property is:
       - absent --> error
       - infinity --> rigid
       - greater than zero and less than infinity --> soft
       - less than or equal to zero --> error

 @throws std::logic_error if any of the above error conditions are met.  */
HydroelasticType Classify(const ProximityProperties& properties);

/** @name Creating hydroelastic representations of shapes

 By default *no* shapes are supported with hydroelastic representations.
 Attempting to do so will print a warning to the Drake log and these functions
 will return std::nullopt.

 For every shape that *is* supported, an overload of this method on that shape
 type is declared below.  */
//@{

/** Generic interface for handling unsupported rigid Shapes. Unsupported
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

/** Rigid sphere support. Requires the ('hydroelastic', 'resolution_hint')
 property.  */
std::optional<RigidGeometry> MakeRigidRepresentation(
    const Sphere& sphere, const ProximityProperties& props);

/** Rigid box support. Requires the ('hydroelastic', 'resolution_hint')
 property.  */
std::optional<RigidGeometry> MakeRigidRepresentation(
    const Box& box, const ProximityProperties& props);


/** Generic interface for handling unsupported soft Shapes. Unsupported
 geometries will return a std::nullopt.  */
template <typename Shape>
std::optional<SoftGeometry> MakeSoftRepresentation(
    const Shape& shape, const ProximityProperties&) {
  static const logging::Warn log_once(
        "Soft {} shapes are not currently supported for hydroelastic contact; "
        "registration is allowed, but an error will be thrown during contact.",
        ShapeName(shape));
  return {};
}

/** Creates a soft sphere (assuming the proximity properties has sufficient
 information). Requires the ('hydroelastic', 'resolution_hint') and
 ('material', 'elastic_modulus') properties.  */
std::optional<SoftGeometry> MakeSoftRepresentation(
    const Sphere& sphere, const ProximityProperties& props);

//@}

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
