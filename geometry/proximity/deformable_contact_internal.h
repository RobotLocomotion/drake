#pragma once

#include <unordered_map>
#include <vector>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/deformable_rigid_contact_pair.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/query_results/deformable_contact_data.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace deformable {

/* This class stores all instantiated representations of declared geometries for
 deformable contact. They are keyed by the geometry's global GeometryId.

 In order for a geometry with id `g_id` to have a representation in this
 collection, it must:

   - be declared via calling MaybeAddGeometry();
   - the infrastructure must support the type of representation for that
     particular geometry type;
   - it must have a valid set of properties to fully specify the representation;
   - the geometry cannot have been subsequently removed via a call to
     RemoveGeometry().

 If a geometry satisfies the requirements above, we say that the geometry has a
 "deformable contact representation". If two geometries are in contact, in order
 to produce the corresponding DeformableContactData, both ids must have a valid
 representation in this data structure. Right now, we only support deformable
 vs. rigid contact. In the future, we will also support deformable vs.
 deforamble contact. Rigid vs. rigid contact is handled through point contact or
 hydroelastics. */
class Geometries final : public ShapeReifier {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Geometries);

  Geometries() = default;

  /* Returns the representation of the deformable geometry with the given `id`.
   @throws exception if the a deformable geomtry with the given `id` doesn't
   exist. */
  const DeformableGeometry& deformable_geometry(GeometryId id) const;

  /* Returns the representation of the rigid geometry with the given `id`.
  @throws exception if the a righd geomtry with the given `id` doesn't exist. */
  const RigidGeometry& rigid_geometry(GeometryId id) const;

  /* Returns true if a rigid geometry representation with the given `id` exists.
   */
  bool is_rigid(GeometryId id) const {
    return rigid_geometries_.count(id) != 0;
  }

  /* Returns true if a deformable geometry representation with the given `id`
   * exists. */
  bool is_deformable(GeometryId id) const {
    return deformable_geometries_.count(id) != 0;
  }

  /* Returns the total number of deformable geometries. */
  int num_deformable_geometries() const {
    return deformable_geometries_.size();
  }

  /* Removes the geometry (if it has a deformable contact representation).  */
  void RemoveGeometry(GeometryId id);

  /* Examines the given shape and properties, adding a rigid geometry
   representation as indicated by the `properties` and supported by the current
   infrastructure. No exception is thrown if the given shape is not supported in
   the current infrastructure. However, if it *is* supported, but the properties
   are malformed, an exception will be thrown.

   @param shape         The shape to possibly represent.
   @param id            The unique identifier for the geometry.
   @param properties    The proximity properties that specifies the properties
                        of the rigid representation.
   @throws std::exception if the shape is a supported type but the properties
                          are malformed.
   @pre There is no previous representation associated with id.  */
  void MaybeAddRigidGeometry(const Shape& shape, GeometryId id,
                             const ProximityProperties& properties);

  /* Updates the world pose of the rigid geometry with the given id, if it
   exists, to `X_WG`. */
  void UpdateRigidWorldPose(GeometryId id,
                            const math::RigidTransform<double>& X_WG);

  /* Adds a deformable geometry with the given `shape` and `mesh`.

   @param shape  The shape to possibly represent.
   @param id     The unique identifier for the geometry.
   @param mesh   The volume mesh representation of the deformable geometry.
   @throws std::exception if the shape is not supported.
   @pre There is no previous representation associated with id.  */
  void MaybeAddDeformableGeometry(const Shape& shape, GeometryId id,
                                  const VolumeMesh<double>& mesh);

  /* If the deformable geometry with `id` exists, updates the vertex positions
   of the geometry (in the world frame W) to `q_WG`. */
  void UpdateDeformableVertexPositions(
      GeometryId id, const Eigen::Ref<const VectorX<double>>& q_WG);

  /* For all registered deformable bodies, computes the contact data of that
   deformable body with all registered rigid bodies; if a contact exist, adds
   the contact results to `deformable_contact_data`.
   Assumes the vertex positions and poses of all registered deformable and rigid
   bodies are up to date.
   @pre deformable_contact_data != nullptr. */
  void ComputeAllDeformableContactData(
      std::vector<DeformableContactData<double>>* deformable_contact_data)
      const;

 private:
  friend class GeometriesTester;

  // Data to be used during reification. It is passed as the `user_data`
  // parameter in the ImplementGeometry API.
  struct ReifyData {
    GeometryId id;
    std::optional<ProximityProperties> properties;
    /* Optional mesh volume mesh representation. The geometry is deformable iff
     a mesh representation is supplied. */
    std::optional<VolumeMesh<double>> mesh{std::nullopt};
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

  /* Makes a rigid or deformable representation of the `shape` for deformable
  contact depending on `data`. If the shape with the desired representation type
  is supported, add its deformable contact representation to `this`. Otherwise,
  log a warning or throw an exception that the shape with the desired
  representation is not supported. */
  template <typename ShapeType>
  void MakeShape(const ShapeType& shape, const ReifyData& data);

  /* Calculates the contact data for the deformable body with the given id.
   @pre A deformable representation with `deformable_id` exists. */
  DeformableContactData<double> CalcDeformableContactData(
      GeometryId deformable_id) const;

  /* Calculates the contact data between the deformable geometry with
   `deformable_id` and the rigid geometry with `rigid_id`.
   @pre A rigid representation with `rigid_id` exists.
   @pre A deformable representation with `deformable_id` exists. */
  DeformableRigidContactPair<double> CalcDeformableRigidContactPair(
      GeometryId rigid_id, GeometryId deformable_id) const;

  // The representations of all deformable geometries.
  std::unordered_map<GeometryId, DeformableGeometry> deformable_geometries_;

  // The representations of all rigid geometries.
  std::unordered_map<GeometryId, RigidGeometry> rigid_geometries_;
};

}  // namespace deformable
}  // namespace internal
}  // namespace geometry
}  // namespace drake
